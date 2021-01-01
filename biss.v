`timescale 1ns / 1ps
`default_nettype none

module biss #(
	parameter HZ = 0,
	parameter CMD_BITS = 0,
	parameter NBISS = 0,
	parameter CMD_CONFIG_BISS = 0,
	parameter CMD_BISS_FRAME = 0,
	parameter RSP_BISS_FRAME = 0
) (
	input wire clk,
	input wire [63:0] systime,

	input wire [31:0] arg_data,
	output wire arg_advance,
	input wire [CMD_BITS-1:0] cmd,
	input wire cmd_ready,
	output reg cmd_done = 0,

	output reg [32:0] param_data = 0,
	output reg param_write = 0,

	output reg invol_req = 0,
	input wire invol_grant,

	output reg [NBISS-1:0] biss_ma = { NBISS { 1'b1 }},
	output reg [NBISS-1:0] biss_mo = { NBISS { 1'b0 }},
	input wire [NBISS-1:0] biss_mi,

	output wire [19:0] debug
);

/*
	CMD_CONFIG_BISS in <channel> <divider> <timeout>
	CMD_BISS_FRAME in <channel> <cdm>
	RSP_BISS_FRAME <channel> <status> <cds> <data(str)>
*/

localparam NBISS_BITS = $clog2(NBISS + 1);
reg [NBISS_BITS-1:0] channel = 0;

/* just keep asserted, we'll read one arg per clock */
assign arg_advance = 1;

localparam PS_IDLE		= 0;
localparam PS_CONFIG_BISS_1	= 1;
localparam PS_CONFIG_BISS_2	= 2;
localparam PS_FRAME_1		= 3;
localparam PS_FRAME_2		= 4;
localparam PS_FRAME_3		= 5;
localparam PS_FRAME_4		= 6;
localparam PS_FRAME_CMD_END	= 7;
localparam PS_FRAME_END		= 8;
localparam PS_FRAME_END_2	= 9;
localparam PS_MAX		= 9;

localparam PS_BITS= $clog2(PS_MAX + 1);
reg [PS_BITS-1:0] state = PS_IDLE;

localparam MAX_FRAME		= 64 * 8;
localparam MAX_FRAME_BITS = $clog2(MAX_FRAME);
localparam MAX_TIMEOUT		= HZ / 20000; /* 50us */
localparam MAX_TIMEOUT_BITS = $clog2(MAX_TIMEOUT);

reg [MAX_TIMEOUT_BITS-1:0] freq_divider[NBISS];
reg [MAX_TIMEOUT_BITS-1:0] timeout[NBISS];

reg [2:0] in_cnt = 0;
reg [MAX_FRAME_BITS-1:0] bit_cnt;
reg [MAX_TIMEOUT_BITS-1:0] ma_cnt;
reg cdm;

/*
 * TODO: add delay compensation (probably only needed at high speed)
 * TODO: add mo handling (for actuators)
 */

always @(posedge clk) begin
	cmd_done <= 0;
	param_write <= 0;

	if (ma_cnt != 0)
		ma_cnt <= ma_cnt - 1;

	if (state == PS_IDLE && cmd_ready) begin
		// common to all cmds
		channel <= arg_data[NBISS_BITS-1:0];
		if (cmd == CMD_CONFIG_BISS) begin
			state <= PS_CONFIG_BISS_1;
		end else if (cmd == CMD_BISS_FRAME) begin
			state <= PS_FRAME_1;
		end else begin
			cmd_done <= 1;
		end
	end else if (state == PS_CONFIG_BISS_1) begin
		/* freq divider, timeout, delay */
		freq_divider[channel] <= arg_data - 1;
		state <= PS_CONFIG_BISS_2;
	end else if (state == PS_CONFIG_BISS_2) begin
		timeout[channel] <= arg_data - 1;
		cmd_done <= 1;
		state <= PS_IDLE;
	end else if (state == PS_FRAME_1) begin
		cdm <= arg_data;
		param_data <= channel;
		param_write <= 1;
		state <= PS_FRAME_2;
	end else if (state == PS_FRAME_2) begin
		bit_cnt <= arg_data;
		in_cnt <= 7;
		param_data <= (arg_data >> 3) + (arg_data[2:0] ? 1 : 0); /* len of rsp string */
		param_write <= 1;
		state <= PS_FRAME_3;
	end else if (state == PS_FRAME_3 && ma_cnt == 0) begin
		if (bit_cnt != 0) begin
			biss_ma[channel] <= 0;
			state <= PS_FRAME_4;
			bit_cnt <= bit_cnt - 1;
			ma_cnt <= freq_divider[channel];
		end else begin
			biss_ma[channel] <= cdm;
			ma_cnt <= timeout[channel];
			if (in_cnt != 7) begin
				param_data[32] <= 1; /* send as raw byte */
				param_data[31:8] <= 0;
				param_write <= 1;
			end
			state <= PS_FRAME_CMD_END;
		end
	end else if (state == PS_FRAME_4 && ma_cnt == 0) begin
		biss_ma[channel] <= 1;
		if (in_cnt == 7)
			param_data[7:0] <= biss_mi[channel];
		else
			param_data[7:0] <= { param_data[6:0], biss_mi[channel] };
		if (in_cnt == 0) begin
			param_data[32] <= 1; /* send as raw byte */
			param_data[31:8] <= 0;
			param_write <= 1;
			in_cnt <= 7;
		end else begin
			in_cnt <= in_cnt - 1;
		end
		ma_cnt <= freq_divider[channel];
		state <= PS_FRAME_3;
	end else if (state == PS_FRAME_CMD_END) begin
		cmd_done <= 1;
		param_data <= RSP_BISS_FRAME;
		state <= PS_FRAME_END;
	end else if (state == PS_FRAME_END && ma_cnt == 0) begin
		biss_ma[channel] <= 1;
		ma_cnt <= freq_divider[channel];
		state <= PS_FRAME_END_2;
	end else if (state == PS_FRAME_END_2 && ma_cnt == 0) begin
		state <= PS_IDLE;
	end
end

assign debug[3:0] = state;
assign debug[19:4] = 0;

endmodule
