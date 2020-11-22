`timescale 1ns / 1ps
`default_nettype none

module pwm #(
	parameter NPWM = 0,
	parameter CMD_BITS = 0,
	parameter CMD_CONFIG_PWM = 0,
	parameter CMD_SCHEDULE_PWM = 0
) (
	input wire clk,
	input wire [31:0] systime,

	input wire [31:0] arg_data,
	output wire arg_advance,
	input wire [CMD_BITS-1:0] cmd,
	input wire cmd_ready,
	output reg cmd_done = 0,

	output reg [32:0] param_data = 0,
	output reg param_write = 0,

	output reg invol_req = 0,
	input wire invol_grant,

	output reg [NPWM-1:0] pwm = 0,

	input wire shutdown,

	output reg missed_clock = 0
);

/* pwm */
localparam PWM_BITS = 26;
reg [PWM_BITS-1:0] on_ticks[NPWM];
reg [PWM_BITS-1:0] off_ticks[NPWM];
reg [PWM_BITS-1:0] next_on_ticks[NPWM];
reg [PWM_BITS-1:0] next_off_ticks[NPWM];
reg [31:0] next_time[NPWM];
reg scheduled[NPWM];
reg default_value[NPWM];
reg [31:0] max_duration[NPWM];
reg [31:0] duration[NPWM];
reg [PWM_BITS-1:0] toggle_cnt [NPWM];
integer i;
initial begin
	for (i = 0; i < NPWM; i = i + 1) begin
		on_ticks[i] = 0;
		off_ticks[i] = 0;
		next_on_ticks[i] = 0;
		next_off_ticks[i] = 0;
		next_time[i] = 0;
		default_value[i] = 1'b0;
		max_duration[i] = 0;
		duration[i] = 0;
		scheduled[i] = 0;
		toggle_cnt[i] = 0;
	end
end

localparam PS_IDLE = 0;
localparam PS_CONFIG_1 = 1;
localparam PS_CONFIG_2 = 2;
localparam PS_CONFIG_3 = 3;
localparam PS_SCHEDULE_PWM_1 = 4;
localparam PS_SCHEDULE_PWM_2 = 5;
localparam PS_SCHEDULE_PWM_3 = 6;
localparam PS_MAX = 6;

localparam PS_BITS = $clog2(PS_MAX + 1);
localparam NPWM_BITS = $clog2(NPWM);
reg [3:0] state = PS_IDLE;
reg [NPWM_BITS-1:0] channel = 0;
/* just keep asserted, we'll read one arg per clock */
assign arg_advance = 1;
always @(posedge clk) begin
	if (cmd_done)
		cmd_done <= 0;

	for (i = 0; i < NPWM; i = i + 1) begin
		if (toggle_cnt[i] == 1) begin
			pwm[i] <= !pwm[i];
			if (!pwm[i])
				toggle_cnt[i] <= on_ticks[i];
			else
				toggle_cnt[i] <= off_ticks[i];
		end else if (toggle_cnt[i] != 0) begin
			toggle_cnt[i] <= toggle_cnt[i] - 1;
		end
		if (scheduled[i] && next_time[i] == systime[31:0]) begin
			on_ticks[i] <= next_on_ticks[i];
			off_ticks[i] <= next_off_ticks[i];
			duration[i] <= max_duration[i];
			scheduled[i] <= 0;
`ifndef NOTYET
			if (toggle_cnt[i] == 0)
				toggle_cnt[i] <= 1;
`else
			if (next_on_ticks[i] == 0) begin
				/* if we're going to a static value, just set it */
				/* XXX maybe be nicer */
				pwm[i] <= 1;
				toggle_cnt[i] <= 0;
			end else if (next_off_ticks[i] == 0) begin
				pwm[i] <= 0;
				toggle_cnt[i] <= 0;
			end else if (toggle_cnt[i] == 0) begin
				/*
				 * if we're coming from a static value, just kick the
				 * machine again. The condition above will do the rest
				 */
				toggle_cnt[i] <= 1;
			end
`endif
		end
	end

	/*
	 * pwm duration safety feature. Must be before state
	 * machine because pwm_duration is set on load.
	 * duration = 0 turns this check off
	 */
	for (i = 0; i < NPWM; i = i + 1) begin
		if (shutdown || duration[i] == 1) begin
			toggle_cnt[i] <= 0;
			pwm[i] <= default_value[i];
		end
		if (duration[i] != 0) begin
			duration[i] <= duration[i] - 1;
		end
	end

	if (state == PS_IDLE && cmd_ready) begin
		// common to all cmds
		channel <= arg_data[NPWM_BITS-1:0];
		if (cmd == CMD_CONFIG_PWM) begin
			state <= PS_CONFIG_1;
		end else if (cmd == CMD_SCHEDULE_PWM) begin
			state <= PS_SCHEDULE_PWM_1;
		end else begin
			cmd_done <= 1;
		end
	end else if (state == PS_CONFIG_1) begin
		toggle_cnt[channel] <= 0;
		pwm[channel] <= arg_data[0];
		state <= PS_CONFIG_2;
	end else if (state == PS_CONFIG_2) begin
		default_value[channel] <= arg_data[0];
		state <= PS_CONFIG_3;
	end else if (state == PS_CONFIG_3) begin
		max_duration[channel] <= arg_data;
		cmd_done <= 1;
		state <= PS_IDLE;
	end else if (state == PS_SCHEDULE_PWM_1) begin
		next_time[channel] <= arg_data;
		if (arg_data - systime >= 32'hc0000000 ||
		    arg_data - systime == 32'h00000000)
			missed_clock <= 1;
		state <= PS_SCHEDULE_PWM_2;
	end else if (state == PS_SCHEDULE_PWM_2) begin
		next_on_ticks[channel] <= arg_data;
		state <= PS_SCHEDULE_PWM_3;
	end else if (state == PS_SCHEDULE_PWM_3) begin
		next_off_ticks[channel] <= arg_data;
		scheduled[channel] <= 1;
		cmd_done <= 1;
		state <= PS_IDLE;
	end
end

endmodule
