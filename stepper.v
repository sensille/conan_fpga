`timescale 1ns / 1ps
`default_nettype none

module stepper #(
	parameter NSTEPDIR = 6,
	parameter NENDSTOP = 8,
	parameter CMD_BITS = 8,
	parameter CMD_CONFIG_STEPPER = 0,
	parameter CMD_QUEUE_STEP = 0,
	parameter CMD_SET_NEXT_STEP_DIR = 0,
	parameter CMD_RESET_STEP_CLOCK = 0,
	parameter CMD_STEPPER_GET_POS = 0,
	parameter CMD_ENDSTOP_SET_STEPPER = 0,
	parameter CMD_ENDSTOP_QUERY = 0,
	parameter CMD_ENDSTOP_HOME = 0,
	parameter RSP_STEPPER_GET_POS = 0,
	parameter RSP_ENDSTOP_STATE = 0
) (
	input wire clk,
	input wire [31:0] systime,

	input wire [31:0] arg_data,
	output reg arg_advance = 0,
	input wire [CMD_BITS-1:0] cmd,
	input wire cmd_ready,
	output reg cmd_done = 0,

	output reg [31:0] param_data = 0,
	output reg param_write = 0,

	output reg invol_req = 0,
	input wire invol_grant,

	output reg [NSTEPDIR-1:0] step,
	output reg [NSTEPDIR-1:0] dir,
	input wire [NENDSTOP-1:0] endstop
);

/*
        cmdtab[CMD_CONFIG_STEPPER] = { UNIT_STEPPER, ARGS_, 1'b0, 1'b0 };
in: <channel> <dedge>
        cmdtab[CMD_QUEUE_STEP] = { UNIT_STEPPER, ARGS_, 1'b0, 1'b0 };
in: <channel> <interval> <count> <add>
        cmdtab[CMD_SET_NEXT_STEP_DIR] = { UNIT_STEPPER, ARGS_, 1'b0, 1'b0 };
in: <channel> <dir>
        cmdtab[CMD_RESET_STEP_CLOCK] = { UNIT_STEPPER, ARGS_, 1'b0, 1'b0 };
in: <channel> <time>
        cmdtab[CMD_STEPPER_GET_POS] = { UNIT_STEPPER, ARGS_, 1'b0, 1'b0 };
in: <channel>
RSP_STEPPER_GET_POS
out: <position>
        cmdtab[CMD_ENDSTOP_SET_STEPPER] = { UNIT_STEPPER, ARGS_, 1'b0, 1'b0 };
in: <endstop-channel> <stepper-channel>
        cmdtab[CMD_ENDSTOP_QUERY] = { UNIT_STEPPER, ARGS_, 1'b0, 1'b0 };
in: <channel>
RSP_ENDSTOP_QUERY
out: <homing> <pin_value>
        cmdtab[CMD_ENDSTOP_HOME] = { UNIT_STEPPER, ARGS_, 1'b0, 1'b0 };
in: <endstop-channel> <time> <sample_count> <pin_value>

*/

/*
 * queue is 72 bits wide:
 * <move type:3> <dir:1> <interval:22> <count:26> <add:20>
 */
localparam MOVE_TYPE_KLIPPER = 3'b000;
localparam MOVE_TYPE_BITS = 3;
localparam STEP_INTERVAL_BITS = 22;
localparam STEP_COUNT_BITS = 26;
localparam STEP_ADD_BITS = 20;
localparam MOVE_COUNT = 512;
wire [71:0] step_queue_wr_data;
reg [NSTEPDIR-1:0] step_queue_wr_en = 0;
wire [NSTEPDIR-1:0] step_queue_empty;
wire [NSTEPDIR-1:0] step_running;
reg [NSTEPDIR-1:0] step_next_dir = 0;
reg [NSTEPDIR-1:0] step_start = 0;
reg [NSTEPDIR-1:0] step_reset = 0;
reg [NSTEPDIR-1:0] step_start_pending = 0;
reg [31:0] step_start_time [NSTEPDIR];
wire [31:0] step_position[NSTEPDIR];
reg dedge[NSTEPDIR];

genvar stepdir_gi;
generate
	for (stepdir_gi = 0; stepdir_gi < NSTEPDIR; stepdir_gi = stepdir_gi + 1) begin : genstepdir
		stepdir #(
			.MOVE_TYPE_KLIPPER(MOVE_TYPE_KLIPPER),
			.MOVE_TYPE_BITS(MOVE_TYPE_BITS),
			.STEP_INTERVAL_BITS(STEP_INTERVAL_BITS),
			.STEP_COUNT_BITS(STEP_COUNT_BITS),
			.STEP_ADD_BITS(STEP_ADD_BITS),
			.MOVE_COUNT(MOVE_COUNT)
		) u_stepdir (
			.clk(clk),
			.queue_wr_data(step_queue_wr_data),
			.queue_wr_en(step_queue_wr_en[stepdir_gi]),
			.queue_empty(step_queue_empty[stepdir_gi]),
			.running(step_running[stepdir_gi]),
			.start(step_start[stepdir_gi]),
			.reset(step_reset[stepdir_gi]),
			.dedge(dedge[stepdir_gi]),
			.step(step[stepdir_gi]),
			.dir(dir[stepdir_gi]),
			.position(step_position[stepdir_gi])
		);
	end
endgenerate

/*
 * endstops
 */
reg [NENDSTOP-1:0] endstop_homing = 0;
reg [NSTEPDIR-1:0] endstop_stepper[NENDSTOP-1:0];
reg [31:0] endstop_time[NENDSTOP-1:0];
reg [23:0] endstop_sample_count[NENDSTOP-1:0];
reg [15:0] endstop_tick_cnt[NENDSTOP-1:0];
reg [5:0] endstop_sample_cnt[NENDSTOP-1:0];
reg endstop_pin_value[NENDSTOP];
reg [NENDSTOP-1:0] endstop_send_state = 0;
localparam ES_IDLE = 0;
localparam ES_WAIT_FOR_CLOCK = 1;
localparam ES_REST = 2;
localparam ES_SAMPLE = 3;
reg [1:0] endstop_state[NENDSTOP-1:0];
initial begin: init_endstop1
	integer i;

	for (i = 0; i < NENDSTOP; i = i + 1) begin
		endstop_stepper[i] = 0;
	end
end

localparam CHANNEL_BITS = $clog2(NENDSTOP > NSTEPDIR ? NENDSTOP : NSTEPDIR);
reg [CHANNEL_BITS-1:0] channel = 0;

/* registers that compose the queue_write signal */
reg [STEP_INTERVAL_BITS-1:0] q_interval = 0;
reg [STEP_COUNT_BITS-1:0] q_count = 0;
reg [STEP_ADD_BITS-1:0] q_add = 0;
assign step_queue_wr_data = { MOVE_TYPE_KLIPPER, step_next_dir[channel], q_interval, q_count, q_add };

localparam PS_IDLE			= 0;
localparam PS_CONFIG_STEPPER_1		= 1;
localparam PS_QUEUE_STEP_1		= 2;
localparam PS_QUEUE_STEP_2		= 3;
localparam PS_QUEUE_STEP_3		= 4;
localparam PS_SET_NEXT_STEP_DIR_1	= 5;
localparam PS_RESET_STEP_CLOCK_1	= 6;
localparam PS_STEPPER_GET_POS_1		= 7;
localparam PS_STEPPER_GET_POS_2		= 8;
localparam PS_ENDSTOP_SET_STEPPER_1	= 9;
localparam PS_ENDSTOP_QUERY_1		= 10;
localparam PS_ENDSTOP_QUERY_2		= 11;
localparam PS_ENDSTOP_QUERY_3		= 12;
localparam PS_ENDSTOP_HOME_1		= 13;
localparam PS_ENDSTOP_HOME_2		= 14;
localparam PS_ENDSTOP_HOME_3		= 15;
localparam PS_WAIT_GRANT		= 16;
localparam PS_MAX			= 16;

localparam PS_BITS = $clog2(PS_MAX);
localparam NENDSTOP_BITS = $clog2(NENDSTOP);
localparam NSTEPDIR_BITS = $clog2(NSTEPDIR);
reg [3:0] state = PS_IDLE;
reg temp_reg = 0;
/* just keep asserted, we'll read one arg per clock */
assign arg_advance = 1;
always @(posedge clk) begin: main
	integer i;

	if (cmd_done)
		cmd_done <= 0;
	step_start <= 0;
	step_queue_wr_en <= 0;
	if (state == PS_IDLE && cmd_ready) begin
		// common to all cmds
		channel <= arg_data[NSTEPDIR_BITS-1:0];
		if (cmd == CMD_CONFIG_STEPPER) begin
			state <= PS_CONFIG_STEPPER_1;
		end else if (cmd == CMD_QUEUE_STEP) begin
			state <= PS_QUEUE_STEP_1;
		end else if (cmd == CMD_SET_NEXT_STEP_DIR) begin
			state <= PS_SET_NEXT_STEP_DIR_1;
		end else if (cmd == CMD_RESET_STEP_CLOCK) begin
			state <= PS_RESET_STEP_CLOCK_1;
		end else if (cmd == CMD_STEPPER_GET_POS) begin
			state <= PS_STEPPER_GET_POS_1;
		end else if (cmd == CMD_ENDSTOP_SET_STEPPER) begin
			state <= PS_ENDSTOP_SET_STEPPER_1;
		end else if (cmd == CMD_ENDSTOP_QUERY) begin
			state <= PS_ENDSTOP_QUERY_1;
		end else if (cmd == CMD_ENDSTOP_HOME) begin
			state <= PS_ENDSTOP_HOME_1;
		end else begin
			cmd_done <= 1;
		end
	end else if (state == PS_CONFIG_STEPPER_1) begin
		dedge[channel] <= arg_data[0];
		cmd_done <= 1;
		state <= PS_IDLE;
	end else if (state == PS_QUEUE_STEP_1) begin
		/* <channel> <interval> <count> <add> */
		q_interval <= arg_data[STEP_INTERVAL_BITS-1:0];
		state <= PS_QUEUE_STEP_2;
	end else if (state == PS_QUEUE_STEP_2) begin
		q_count <= arg_data[STEP_COUNT_BITS-1:0];
		state <= PS_QUEUE_STEP_3;
	end else if (state == PS_QUEUE_STEP_3) begin
		q_add <= arg_data[STEP_ADD_BITS-1:0];
		step_queue_wr_en[channel] <= 1;
		cmd_done <= 1;
		state <= PS_IDLE;
	end else if (state == PS_SET_NEXT_STEP_DIR_1) begin
		step_next_dir[channel] <= arg_data[0];
		cmd_done <= 1;
		state <= PS_IDLE;
	end else if (state == PS_RESET_STEP_CLOCK_1) begin
		step_start_time[channel] <= arg_data;
		step_start_pending[channel] <= 1;
		cmd_done <= 1;
		state <= PS_IDLE;
	end else if (state == PS_STEPPER_GET_POS_1) begin
		param_data <= step_position[channel];
		param_write <= 1;
		state <= PS_STEPPER_GET_POS_2;
	end else if (state == PS_STEPPER_GET_POS_2) begin
		cmd_done <= 1;
		param_write <= 0;
		param_data <= RSP_STEPPER_GET_POS;
		state <= PS_IDLE;
	end else if (state == PS_ENDSTOP_SET_STEPPER_1) begin
		/* <endstop-channel> <stepper-channel> */
		endstop_stepper[arg_data[NSTEPDIR_BITS-1:0]][channel] <= 1;
		cmd_done <= 1;
		state <= PS_IDLE;
	end else if (state == PS_ENDSTOP_QUERY_1) begin
		param_data <= endstop_homing[channel];
		param_write <= 1;
		state <= PS_ENDSTOP_QUERY_2;
	end else if (state == PS_ENDSTOP_QUERY_2) begin
		param_data <= endstop[channel];
		state <= PS_ENDSTOP_QUERY_3;
	end else if (state == PS_ENDSTOP_QUERY_3) begin
		cmd_done <= 1;
		param_write <= 0;
		param_data <= RSP_ENDSTOP_STATE;
		state <= PS_IDLE;
	end else if (state == PS_ENDSTOP_HOME_1) begin
		/* <endstop-channel> <time> <sample_count> <pin_value> */
		endstop_time[channel] <= arg_data;
		state <= PS_ENDSTOP_HOME_2;
	end else if (state == PS_ENDSTOP_HOME_2) begin
		endstop_sample_count[channel] <= arg_data;
		state <= PS_ENDSTOP_HOME_3;
		temp_reg <= arg_data == 0;
	end else if (state == PS_ENDSTOP_HOME_3) begin
		endstop_pin_value[channel] <= arg_data;
		if (temp_reg)
			endstop_homing[channel] <= 0;	/* sample_count 0, cancel homing */
		else
			endstop_homing[channel] <= 1;
		endstop_state[channel] <= ES_WAIT_FOR_CLOCK;
		cmd_done <= 1;
		state <= PS_IDLE;
	/* send endstop state when in idle */
	end else if (state == PS_IDLE && endstop_send_state) begin
		invol_req <= 1;
		state <= PS_WAIT_GRANT;
	end else if (state == PS_WAIT_GRANT) begin
		invol_req <= 0;
		state <= PS_WAIT_GRANT;
		for (i = 0; i < NENDSTOP; i = i + 1) begin
			if (endstop_send_state[i]) begin
				endstop_send_state[i] <= 0;
				channel <= i;
				state <= PS_ENDSTOP_QUERY_1;
				break;
			end
		end
	end

	/*
	 * reset_step_time handling
	 */
	for (i = 0; i < NSTEPDIR; i = i + 1) begin
		if (step_start_pending[i] && systime == step_start_time[i]) begin
			step_start_pending[i] <= 0;
			step_start[i] <= 1;
		end
	end

	/*
	 * endstop homing engine
	 */
	step_reset <= 0;
	for (i = 0; i < NENDSTOP; i = i + 1) begin
		if (endstop_homing[i]) begin
			if (endstop_state[i] == ES_WAIT_FOR_CLOCK) begin
				if (endstop_time[i] == systime) begin
					endstop_state[i] <= ES_REST;
				end
			end else if (endstop_state[i] == ES_REST) begin
				if (endstop[i] == endstop_pin_value[i]) begin
					endstop_state[i] <= ES_SAMPLE;
					endstop_sample_cnt[i] <= endstop_sample_count[i];
				end
			end else if (endstop_state[i] == ES_SAMPLE) begin
				if (endstop[i] != endstop_pin_value[i]) begin
					endstop_state[i] <= ES_REST;
				end else begin
					if (endstop_sample_cnt[i] == 1) begin
						/* endstop triggered */
						endstop_homing[i] <= 0;
						/* reset stepper */
						step_reset <= step_reset | endstop_stepper[i];
						/* send endstop_state */
						endstop_send_state[i] <= 1;
					end else begin
						endstop_sample_cnt[i] <= endstop_sample_cnt[i] - 1;
					end
				end
			end
		end
	end
end

endmodule