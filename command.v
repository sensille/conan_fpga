`timescale 1ns / 1ps
`default_nettype none

module command #(
	parameter RING_BITS = 8,
	parameter LEN_BITS = 8,
	parameter LEN_FIFO_BITS = 7,
	parameter MOVE_COUNT = 512,
	parameter NGPIO_OUT = 40,
	parameter NGPIO_IN = 40,
	parameter NPWM = 12,
	parameter NSTEPDIR = 6,
	parameter NENDSTOP = 8,
	parameter NUART = 6,
	parameter VERSION
) (
	input wire clk,

	/*
	 * receive side
	 */
	input wire [7:0] msg_data,
	input wire msg_ready,
	output reg msg_rd_en = 0,

	/*
	 * send side
	 */
	output reg send_fifo_wr_en = 0,
	output reg [LEN_BITS-1:0] send_fifo_data = 0,
	input wire send_fifo_full,

	/* ring buffer input */
	output reg [7:0] send_ring_data = 0,
	output reg send_ring_wr_en = 0,
	input wire send_ring_full,

	/* global time */
	input wire [63:0] systime,

	/*
	 * I/O
	 */
	output wire [NGPIO_OUT-1:0] gpio_out,
	input wire [NGPIO_IN-1:0] gpio_in,
	output wire [NPWM-1:0] pwm,
	output wire [NSTEPDIR-1:0] step,
	output wire [NSTEPDIR-1:0] dir,
	input wire [NENDSTOP-1:0] endstop,
	inout wire [NUART-1:0] uart,

	input wire [63:0] time_in,
	output wire [63:0] time_out,
	output wire time_out_en,
	input wire timesync_pulse_in,
	input wire timesync_latch_in,

	/*
	 * debug
	 */
	output wire [63:0] debug
);

/*
 * definitions for each command
 *
 * indexed by command id
 * values number of parameters, has string, unit to dispatch to, cmd sends response
 * strings are always last
 * 
 */
localparam MAX_ARGS = 8;
localparam ARGS_BITS = $clog2(MAX_ARGS);

localparam UNITS_BITS		= 4;
localparam UNIT_PWM		= 4'd0;
localparam UNIT_GPIO_OUT	= 4'd1;
localparam UNIT_GPIO_IN		= 4'd2;
localparam UNIT_ENDSTOP		= 4'd3;
localparam UNIT_UART		= 4'd4;
localparam UNIT_STEPDIR		= 4'd5;
localparam UNIT_SYSTEM		= 4'd6;
localparam NUNITS		= 4'd7;

localparam CMDTAB_SIZE = UNITS_BITS + ARGS_BITS + 1;
localparam CMD_GET_VERSION	= 0;
localparam CMD_SYNC_TIME	= 1;
localparam CMD_GET_TIME		= 2;
localparam CMD_SET_GPIO_OUT	= 3;
localparam CMD_SCHEDULE_GPIO_OUT= 4;
localparam CMD_CONFIG_PWM	= 5;
localparam CMD_SET_PWM		= 6;
localparam CMD_SCHEDULE_PWM	= 7;
localparam CMD_READ_GPIO_IN	= 8;
localparam NCMDS		= 8;
localparam CMD_BITS = $clog2(NCMDS);

localparam RSP_GET_VERSION	= 0;
localparam RSP_GET_TIME		= 1;
localparam RSP_READ_GPIO_IN	= 2;
localparam RSP_POLL_GPIO_IN	= 3;
localparam RSP_ENDSTOP_STATE	= 4;
localparam RSP_STEPPER_POSITION	= 5;
localparam RSP_UART_TRANSFER	= 6;

/* BAD HACK, should go away soon */
wire [ARGS_BITS-1:0] ARGS_0 = 0;
wire [ARGS_BITS-1:0] ARGS_1 = 1;
wire [ARGS_BITS-1:0] ARGS_2 = 2;
wire [ARGS_BITS-1:0] ARGS_3 = 3;
wire [ARGS_BITS-1:0] ARGS_4 = 4;
wire [ARGS_BITS-1:0] ARGS_5 = 5;
wire [ARGS_BITS-1:0] ARGS_6 = 6;
wire [ARGS_BITS-1:0] ARGS_7 = 7;

reg [CMDTAB_SIZE-1:0] cmdtab[NCMDS];
/* { unit, nargs, string_arg, cmd_has_response } */
initial begin
	cmdtab[CMD_GET_VERSION] = { UNIT_SYSTEM, ARGS_0, 1'b0, 1'b1 };
	cmdtab[CMD_SYNC_TIME] = { UNIT_SYSTEM, ARGS_2, 1'b0, 1'b0 };
	cmdtab[CMD_GET_TIME] = { UNIT_SYSTEM, ARGS_0, 1'b0, 1'b1 };
	cmdtab[CMD_CONFIG_PWM] = { UNIT_PWM, ARGS_5, 1'b0, 1'b0 };
	cmdtab[CMD_SET_GPIO_OUT] = { UNIT_GPIO_OUT, ARGS_2, 1'b0, 1'b0 };
end

`ifdef notyet
localparam CMD_SET_GPIO_OUT = 2;
localparam CMD_SCHEDULE_GPIO_OUT = 3;
localparam CMD_READ_GPIO_IN = 4;
localparam CMD_POLL_GPIO_IN = 5;
localparam CMD_CONFIG_STEPPER = 13;
localparam CMD_CONFIG_ENDSTOP = 14;
localparam CMD_ENDSTOP_HOME = 15;
localparam CMD_ENDSTOP_QUERY_STATE = 16;
localparam CMD_ENDSTOP_SET_STEPPER = 18;
localparam CMD_QUEUE_STEP = 19;
localparam CMD_STEPPER_GET_POSITION = 20;
localparam CMD_SET_NEXT_STEP_DIR = 22;
localparam CMD_RESET_STEP_CLOCK = 23;
localparam CMD_CONFIG_UART = 24;
localparam CMD_UART_SEND = 25;
localparam CMD_UART_TRANSFER = 26;
localparam MAX_CMD = 26;
localparam CMD_BITS = $clog2(MAX_CMD);
`endif

reg [ARGS_BITS-1:0] unit_arg_ptr = 0;
reg [31:0] unit_arg_data = 0;
wire [NUNITS-1:0] unit_arg_advance;
wire [CMD_BITS-1:0] unit_cmd;
reg [NUNITS-1:0] unit_cmd_ready = 0;
wire [NUNITS-1:0] unit_cmd_done;
wire [31:0] unit_param_data [NUNITS];
wire [NUNITS-1:0] unit_param_write;
wire [NUNITS-1:0] unit_invol_req;
reg [NUNITS-1:0] unit_invol_grant = 0;
pwm #(
	.NPWM(NPWM),
	.CMD_CONFIG_PWM(CMD_CONFIG_PWM),
	.CMD_SET_PWM(CMD_SET_PWM),
	.CMD_SCHEDULE_PWM(CMD_SCHEDULE_PWM)
) u_pwm (
	.clk(clk),
	.systime(systime),

	.arg_data(unit_arg_data),
	.arg_advance(unit_arg_advance[UNIT_PWM]),
	.cmd(unit_cmd),
	.cmd_ready(unit_cmd_ready[UNIT_PWM]),
	.cmd_done(unit_cmd_done[UNIT_PWM]),

	.param_data(unit_param_data[UNIT_PWM]),
	.param_write(unit_param_write[UNIT_PWM]),

	.invol_req(unit_invol_req[UNIT_PWM]),
	.invol_grant(unit_invol_grant[UNIT_PWM]),

	.pwm(pwm)
);

system #(
	.CMD_GET_VERSION(CMD_GET_VERSION),
	.RSP_GET_VERSION(RSP_GET_VERSION),
	.CMD_SYNC_TIME(CMD_SYNC_TIME),
	.CMD_GET_TIME(CMD_GET_TIME),
	.RSP_GET_TIME(RSP_GET_TIME),
	.VERSION(VERSION)
) u_system (
	.clk(clk),
	.systime(systime),

	.arg_data(unit_arg_data),
	.arg_advance(unit_arg_advance[UNIT_SYSTEM]),
	.cmd(unit_cmd),
	.cmd_ready(unit_cmd_ready[UNIT_SYSTEM]),
	.cmd_done(unit_cmd_done[UNIT_SYSTEM]),

	.param_data(unit_param_data[UNIT_SYSTEM]),
	.param_write(unit_param_write[UNIT_SYSTEM]),

	.invol_req(unit_invol_req[UNIT_SYSTEM]),
	.invol_grant(unit_invol_grant[UNIT_SYSTEM]),


	.time_in(time_in),
	.time_out(time_out),
	.time_out_en(time_out_en),
	.timesync_pulse_in(timesync_pulse_in),
	.timesync_latch_in(timesync_latch_in)
);

assign gpio_out = 0;
assign step = 0;
assign dir = 0;
assign uart = 0;

localparam MST_IDLE = 0;
localparam MST_PARSE_ARG_START = 1;
localparam MST_PARSE_ARG_CONT = 2;
localparam MST_STRING_START = 3;
localparam MST_STRING_ARG = 4;
localparam MST_DISPATCH = 5;
localparam MST_DISPATCH_WAIT_DONE = 6;
localparam MST_PARAM = 7;
localparam MST_PARAM_SKIP = 8;
localparam MST_PARAM_SEND = 9;
localparam MST_MAX = 9;
localparam MST_BITS = $clog2(MST_MAX);

reg [MST_BITS-1:0] msg_state = 0;
reg [CMD_BITS-1:0] msg_cmd = 0;
assign unit_cmd = msg_cmd;

/* input args state */
reg [31:0] args[MAX_ARGS];
reg [ARGS_BITS-1:0] curr_arg = 0;
reg [ARGS_BITS-1:0] nargs = 0;
reg [UNITS_BITS-1:0] unit = 0;
reg cmd_has_response = 0;

/* output parameters state */
localparam MAX_PARAMS = 8;
localparam PARAM_BITS = $clog2(MAX_PARAMS);
reg [31:0] params [MAX_PARAMS];
reg [PARAM_BITS-1:0] nparams;
reg [PARAM_BITS-1:0] curr_param;
reg [34:0] rcv_param;
reg [2:0] curr_cnt;	/* counter for VLQ */
reg [7:0] rsp_len;
reg string_arg = 0;
/* assume max string is 64 */
reg [5:0] str_pos = 0;
reg [5:0] str_len = 0;
reg [7:0] str_buf[64];

reg _arg_end;
integer i;
always @(posedge clk) begin
	if (msg_rd_en) begin
		msg_rd_en <= 0;
	end
	if (send_ring_wr_en) begin
		send_ring_wr_en <= 0;
	end
	if (send_fifo_wr_en) begin
		send_fifo_wr_en <= 0;
	end
	for (i = 0; i < NUNITS; i = i + 1) begin
		if (unit_invol_grant[i])
			unit_invol_grant[i] <= 0;
	end
	/*
	 * ------------------------------------
	 * stage 1, parse arguments, decode VLQ
	 * ------------------------------------
	 */
	if (msg_ready && !msg_rd_en) begin
		/* verilator lint_off BLKSEQ */
		_arg_end = 0;
		/* verilator lint_on BLKSEQ */
		msg_rd_en <= 1;
		if (msg_state == MST_IDLE) begin
			msg_cmd <= msg_data;
			curr_arg <= 0;
			curr_param <= 0;
			{ unit, nargs, string_arg, cmd_has_response } <= cmdtab[msg_data];
			if (cmdtab[msg_data][ARGS_BITS:1] == 0)
				msg_state <= MST_DISPATCH;
			else
				msg_state <= MST_PARSE_ARG_START;
		end else if (msg_state == MST_PARSE_ARG_START) begin
			args[curr_arg] <= msg_data[6:0];
			if (msg_data[6:5] == 2'b11) begin
				/* negative value */
				args[curr_arg][31:7] <= 25'b1111111111111111111111111;
			end
			if (msg_data[7]) begin
				msg_state <= MST_PARSE_ARG_CONT;
			end else begin
				/* verilator lint_off BLKSEQ */
				_arg_end = 1;
				/* verilator lint_on BLKSEQ */
			end
		end else if (msg_state == MST_PARSE_ARG_CONT) begin
			args[curr_arg] <= { args[curr_arg][24:0], msg_data[6:0] };
			if (!msg_data[7]) begin
				/* verilator lint_off BLKSEQ */
				_arg_end = 1;
				/* verilator lint_on BLKSEQ */
			end
		end else if (msg_state == MST_STRING_START) begin
			string_arg <= 0;
			str_pos <= 0;
			str_len <= args[curr_arg - 1];
			msg_state <= MST_STRING_ARG;
			msg_rd_en <= 0;	/* no read in this clock */
		end else if (msg_state == MST_STRING_ARG) begin
			str_buf[str_pos] <= msg_data;
			str_pos <= str_pos + 1;
			if (str_len == str_pos + 1) begin
				msg_state <= MST_DISPATCH;
			end
		end else begin
			/* we're in some of the states below */
			msg_rd_en <= 0;	/* we're in some of the states below */
		end
		if (_arg_end) begin
			curr_arg <= curr_arg + 1;
			if (curr_arg + 1 == nargs) begin
				if (string_arg) begin
					msg_state <= MST_STRING_START;
				end else begin
					msg_state <= MST_DISPATCH;
				end
			end else begin
				msg_state <= MST_PARSE_ARG_START;
			end
		end
	end
	/*
	 * -------------------------
	 * stage 2, dispatch message
	 * -------------------------
	 */
	if (msg_state == MST_DISPATCH) begin
		unit_arg_data <= args[0];
		unit_arg_ptr <= 1;
		nparams <= 0;
		unit_cmd_ready[unit] <= 1;
		msg_state <= MST_DISPATCH_WAIT_DONE;
	end else if (msg_state == MST_DISPATCH_WAIT_DONE) begin
		unit_cmd_ready[unit] <= 0;
		unit_invol_grant[unit] <= 0;
		if (unit_param_write[unit]) begin
			params[nparams] <= unit_param_data[unit];
			nparams <= nparams + 1;
		end
		if (unit_arg_advance[unit]) begin
			unit_arg_data <= args[unit_arg_ptr];
			unit_arg_ptr <= unit_arg_ptr + 1;
		end
		if (unit_cmd_done[unit]) begin
			if (cmd_has_response) begin
				send_ring_data <= unit_param_data[unit];
				send_ring_wr_en <= 1;
				rsp_len <= 1;
				msg_state <= MST_PARAM;
			end else begin
				msg_state <= MST_IDLE;
			end
		end
	/*
	 * ---------------------------------
	 * stage 3, encode and send response
	 * ---------------------------------
	 */
	end else if (msg_state == MST_PARAM) begin
		/*
		 * encode VLQ param
		 */
		/*
		 * < 00000060 && >= ffffffe0 length 1
		 * < 00003000 && >= fffff000 length 2
		 * < 00180000 && >= fff80000 length 3
		 * < 0c000000 && >= fc000000 length 4
		 * else length 5
		 */
		curr_cnt <= 4;
		/* extend by 3 bits, 32+3 == 5 * 7 */
		rcv_param <= { params[curr_param][31], params[curr_param][31],
				params[curr_param][31], params[curr_param] };
		msg_state <= MST_PARAM_SKIP;
	end else if (msg_state == MST_PARAM_SKIP) begin
		if (curr_cnt != 0 &&
		    (rcv_param[34:26] == 9'b111111111 ||
		     rcv_param[34:26] <  9'b000000011)) begin
			curr_cnt <= curr_cnt - 1;
			rcv_param <= { rcv_param[27:0], 7'b0 };
		end else begin
			msg_state <= MST_PARAM_SEND;
		end
	end else if (msg_state == MST_PARAM_SEND) begin
		if (curr_cnt == 0) begin
			if (curr_param + 1 != nparams) begin
				curr_param <= curr_param + 1;
				msg_state <= MST_PARAM;
			end else begin
				send_fifo_data <= rsp_len + 1;
				send_fifo_wr_en <= 1;
				msg_state <= MST_IDLE;
			end
			send_ring_data[7] <= 1'b0;
		end else begin
			send_ring_data[7] <= 1'b1;
		end
		send_ring_data[6:0] <= rcv_param[34:28];
		send_ring_wr_en <= 1;
		rsp_len <= rsp_len + 1;
		curr_cnt <= curr_cnt - 1;
		rcv_param <= { rcv_param[27:0], 7'b0 };
	/*
	 * ------------------------------
	 * stage 5, send involuntary data
	 * ------------------------------
	 *
	 * only send when state machine is idle and no msg has
	 * been pulled at stage 1
	 */
	end else if (!msg_ready && msg_state == MST_IDLE) begin
		for (i = 0; i < NUNITS; i = i + 1) begin
			if (unit_invol_req[i]) begin
				unit <= i;
				nparams <= 0;
				unit_invol_grant[unit] <= 1;
				msg_state <= MST_DISPATCH_WAIT_DONE;
			end
		end
	end
end

assign debug[3:0] = msg_state;
assign debug[11:4] = msg_cmd;
assign debug[63:12] = 0;

endmodule
