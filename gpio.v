`timescale 1ns / 1ps
`default_nettype none

module gpio #(
	parameter NGPIO = 9,
	parameter CMD_BITS = 8,
	parameter CMD_SET_DIGITAL_OUT = 1,
	parameter CMD_CONFIG_DIGITAL_OUT = 2,
	parameter CMD_SCHEDULE_DIGITAL_OUT = 3,
	parameter CMD_UPDATE_DIGITAL_OUT = 4
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

	output reg [NGPIO-1:0] gpio = 0
);

reg [31:0] next_time[NGPIO];
reg next_value[NGPIO];
reg scheduled[NGPIO];
reg default_value[NGPIO];
reg [31:0] max_duration[NGPIO];
reg [31:0] duration[NGPIO];
integer i;
initial begin
	for (i = 0; i < NGPIO; i = i + 1) begin
		next_time[i] = 0;
		default_value[i] = 1'b0;
		max_duration[i] = 0;
		duration[i] = 0;
		scheduled[i] = 0;
	end
end

localparam PS_IDLE = 0;
localparam PS_CONFIG_1 = 1;
localparam PS_CONFIG_2 = 2;
localparam PS_CONFIG_3 = 3;
localparam PS_SCHEDULE_GPIO_1 = 4;
localparam PS_SCHEDULE_GPIO_2 = 5;
localparam PS_SET_GPIO_1 = 6;
localparam PS_MAX = 6;
localparam PS_BITS = $clog2(PS_MAX + 1);
localparam NGPIO_BITS = $clog2(NGPIO);
reg [3:0] state = PS_IDLE;
reg [NGPIO_BITS-1:0] channel = 0;
/* just keep asserted, we'll read one arg per clock */
assign arg_advance = 1;
always @(posedge clk) begin
	if (cmd_done)
		cmd_done <= 0;
/*
        "config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u": 21,
        "schedule_digital_out oid=%c clock=%u value=%c": 26,
        "update_digital_out oid=%c value=%c": 60
        "set_digital_out pin=%u value=%c": 6,
*/

	/*
	 * gpio duration safety feature. Must be before state
	 * machine because gpio_duration is set on load.
	 */
	for (i = 0; i < NGPIO; i = i + 1) begin
		if (duration[i] == 1) begin
			gpio[i] <= default_value[i];
		end
		if (duration[i] != 0) begin
			duration[i] <= duration[i] - 1;
		end
	end

	if (state == PS_IDLE && cmd_ready) begin
		// common to all cmds
		channel <= arg_data[NGPIO_BITS-1:0];
		if (cmd == CMD_CONFIG_DIGITAL_OUT) begin
			state <= PS_CONFIG_1;
		end else if (cmd == CMD_SCHEDULE_DIGITAL_OUT) begin
			state <= PS_SCHEDULE_GPIO_1;
		end else if (cmd == CMD_UPDATE_DIGITAL_OUT ||
		             cmd == CMD_SET_DIGITAL_OUT) begin
			/* both commands are the same */
			state <= PS_SET_GPIO_1;
		end else begin
			cmd_done <= 1;
		end
	end else if (state == PS_CONFIG_1) begin
		gpio[channel] <= arg_data[0];
		state <= PS_CONFIG_2;
	end else if (state == PS_CONFIG_2) begin
		default_value[channel] <= arg_data[0];
		state <= PS_CONFIG_3;
	end else if (state == PS_CONFIG_3) begin
		max_duration[channel] <= arg_data;
		cmd_done <= 1;
		state <= PS_IDLE;
	end else if (state == PS_SCHEDULE_GPIO_1) begin
		next_time[channel] <= arg_data;
		state <= PS_SCHEDULE_GPIO_2;
	end else if (state == PS_SCHEDULE_GPIO_2) begin
		next_value[channel] <= arg_data;
		scheduled[channel] <= 1;
		cmd_done <= 1;
		state <= PS_IDLE;
	end else if (state == PS_SET_GPIO_1) begin
		gpio[channel] <= arg_data[0];
		cmd_done <= 1;
		state <= PS_IDLE;
	end

	/*
	 * loading of gpio_value, schedule
	 */
	for (i = 0; i < NGPIO; i = i + 1) begin
		if (scheduled[i] && next_time[i] == systime[31:0]) begin
			gpio[i] <= next_value[i];
			duration[i] <= max_duration[i];
			scheduled[i] <= 0;
		end
	end
end

endmodule
