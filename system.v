`timescale 1ns / 1ps
`default_nettype none

module system #(
	parameter CMD_BITS = 8,
	parameter CMD_GET_VERSION = 0,
	parameter RSP_GET_VERSION = 0,
	parameter CMD_SYNC_TIME = 0,
	parameter CMD_GET_TIME = 0,
	parameter RSP_GET_TIME = 0,
	parameter CMD_SHUTDOWN = 0,
	parameter VERSION = 1,
	parameter MOVE_COUNT = 0,
	parameter NGPIO = 0,
	parameter NPWM = 0,
	parameter NSTEPDIR = 0,
	parameter NENDSTOP = 0,
	parameter NUART = 0
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

	input wire [63:0] time_in,
	output reg [63:0] time_out,
	output reg time_out_en,
	input wire timesync_latch_in,

	output reg shutdown = 0
);

reg timesync_latch = 0;
reg timesync_latch1 = 0;
/* sync timesync_latch into our clock domain */
always @(posedge clk) begin
	timesync_latch <= timesync_latch1;
	timesync_latch1 <= timesync_latch_in;
end
localparam PS_IDLE = 0;
localparam PS_GET_VERSION_1 = 1;
localparam PS_GET_VERSION_2 = 2;
localparam PS_GET_VERSION_3 = 3;
localparam PS_SYNC_TIME_1 = 4;
localparam PS_GET_TIME_1  = 5;
localparam PS_GET_TIME_2  = 6;
localparam PS_MAX = 6;
localparam PS_BITS = $clog2(PS_MAX + 1);

reg [63:0] latched_time = 0;
reg latched = 0;
reg prev_latch = 0;
reg prev_pulse = 0;
reg [31:0] temp_data = 0;
reg [3:0] state = PS_IDLE;
/* just keep asserted, we'll read one arg per clock */
assign arg_advance = 1;
always @(posedge clk) begin
	if (cmd_done)
		cmd_done <= 0;
	if (time_out_en)
		time_out_en <= 0;
	if (state == PS_IDLE && cmd_ready) begin
		if (cmd == CMD_GET_VERSION) begin
			param_data <= VERSION;
			param_write <= 1;
			state <= PS_GET_VERSION_1;
		end else if (cmd == CMD_SYNC_TIME) begin
			state <= PS_SYNC_TIME_1;
			temp_data <= arg_data;
		end else if (cmd == CMD_GET_TIME) begin
			temp_data <= time_in[63:32];
			param_data <= time_in[31:0];
			param_write <= 1;
			state <= PS_GET_TIME_1;
		end else if (cmd == CMD_SHUTDOWN) begin
			shutdown <= 1;
			cmd_done <= 1;
			state <= PS_IDLE;
		end
	end else if (state == PS_GET_VERSION_1) begin
		param_data[31:24] <= NGPIO;
		param_data[23:16] <= NPWM;
		param_data[15:8] <= NSTEPDIR;
		param_data[7:0] <= NENDSTOP;
		state <= PS_GET_VERSION_2;
	end else if (state == PS_GET_VERSION_2) begin
		param_data[31:24] <= NUART;
		param_data[15:0] <= MOVE_COUNT;
		state <= PS_GET_VERSION_3;
	end else if (state == PS_GET_VERSION_3) begin
		cmd_done <= 1;
		param_write <= 0;
		param_data <= RSP_GET_VERSION;
		state <= PS_IDLE;
	end else if (state == PS_SYNC_TIME_1) begin
		/*
		 * +4, because we need 2 cycles to sync the pulse into our
		 * domain, one to latch the clock  and one, because system
		 * time will only get set on the next clk
		 */
		time_out <= time_in - latched_time + { arg_data, temp_data } + 4;
		time_out_en <= 1;
		latched <= 0;
		state <= PS_IDLE;
		cmd_done <= 1;
	end else if (state == PS_GET_TIME_1) begin
		param_data <= temp_data;
		state <= PS_GET_TIME_2;
	end else if (state == PS_GET_TIME_2) begin
		cmd_done <= 1;
		param_write <= 0;
		param_data <= RSP_GET_TIME;
		state <= PS_IDLE;
	end

	// latch time on falling edge of timesync_latch
	if (timesync_latch == 0 && prev_latch == 1) begin
		latched_time <= time_in;
	end
	prev_latch <= timesync_latch;
end

endmodule
