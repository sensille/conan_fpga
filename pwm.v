`timescale 1ns / 1ps
`default_nettype none

module pwm #(
	parameter NPWM = 12,
	parameter CMD_BITS = 8,
	parameter CMD_CONFIG_PWM = 2,
	parameter CMD_SCHEDULE_PWM = 3
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

	output reg [NPWM-1:0] pwm = 0,

	input wire shutdown
);

/*
 * NOTE: configure 'on_ticks' has to be set to cycle_ticks - on_ticks, as
 * cycle_ticks count down. The host has to do the conversion
 */

/* pwm */
localparam PWM_BITS = 26;
reg [PWM_BITS-1:0] cycle_ticks[NPWM];
reg [PWM_BITS-1:0] on_ticks[NPWM];
reg [PWM_BITS-1:0] next_on_ticks[NPWM];
reg [31:0] next_time[NPWM];
reg scheduled[NPWM];
reg default_value[NPWM];
reg [31:0] max_duration[NPWM];
reg [31:0] duration[NPWM];
reg [PWM_BITS-1:0] cycle_cnt [NPWM];
integer i;
initial begin
	for (i = 0; i < NPWM; i = i + 1) begin
		cycle_ticks[i] = 0;
		on_ticks[i] = 0;
		next_on_ticks[i] = 0;
		next_time[i] = 0;
		default_value[i] = 1'b0;
		max_duration[i] = 0;
		duration[i] = 0;
		scheduled[i] = 0;
		cycle_cnt[i] = 0;
	end
end

always @(posedge clk) begin
	for (i = 0; i < NPWM; i = i + 1) begin
		if (cycle_cnt[i] == 0) begin
			cycle_cnt[i] <= cycle_ticks[i];
			pwm[i] <= 1'b1;
		end else begin
			cycle_cnt[i] <= cycle_cnt[i] - 1;
		end
		if (cycle_cnt[i] == on_ticks[i])
			pwm[i] <= 1'b0;
	end
end

localparam PS_IDLE = 0;
localparam PS_CONFIG_1 = 1;
localparam PS_CONFIG_2 = 2;
localparam PS_CONFIG_3 = 3;
localparam PS_CONFIG_4 = 4;
localparam PS_SCHEDULE_PWM_1 = 5;
localparam PS_SCHEDULE_PWM_2 = 6;
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

	/*
	 * pwm duration safety feature. Must be before state
	 * machine because pwm_duration is set on load.
	 * duration = 0 turns this check off
	 */
	for (i = 0; i < NPWM; i = i + 1) begin
		if (shutdown || duration[i] == 1) begin
			on_ticks[i] <= { PWM_BITS { default_value[i] } };
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
		cycle_ticks[channel] <= arg_data[PWM_BITS-1:0] - 1;
		state <= PS_CONFIG_2;
	end else if (state == PS_CONFIG_2) begin
		on_ticks[channel] <= arg_data[PWM_BITS-1:0];
		state <= PS_CONFIG_3;
	end else if (state == PS_CONFIG_3) begin
		default_value[channel] <= arg_data[0];
		state <= PS_CONFIG_4;
	end else if (state == PS_CONFIG_4) begin
		max_duration[channel] <= arg_data;
		cmd_done <= 1;
		state <= PS_IDLE;
	end else if (state == PS_SCHEDULE_PWM_1) begin
		next_time[channel] <= arg_data;
		state <= PS_SCHEDULE_PWM_2;
	end else if (state == PS_SCHEDULE_PWM_2) begin
		next_on_ticks[channel] <= arg_data;
		scheduled[channel] <= 1;
		cmd_done <= 1;
		state <= PS_IDLE;
	end

	/*
	 * loading of pwm_on_ticks, schedule
	 */
	for (i = 0; i < NPWM; i = i + 1) begin
		if (scheduled[i] && next_time[i] == systime[31:0]) begin
			on_ticks[i] <= next_on_ticks[i];
			duration[i] <= max_duration[i];
			scheduled[i] <= 0;
		end
	end
end

endmodule
