`timescale 1ns / 1ps
`default_nettype none

module sd #(
	parameter HZ = 0,
	parameter CMD_BITS = 0,
	parameter NSD = 0,
	parameter CMD_SD_QUEUE = 0,
	parameter RSP_SD_CMDQ = 0,
	parameter RSP_SD_DATQ = 0
) (
	input wire clk,
	input wire [31:0] systime,

	input wire [31:0] arg_data,
	output reg arg_advance,
	input wire [CMD_BITS-1:0] cmd,
	input wire cmd_ready,
	output reg cmd_done = 0,

	output reg [32:0] param_data = 0,
	output reg param_write = 0,

	output reg invol_req = 0,
	input wire invol_grant,

	output wire [NSD-1:0] sd_clk,
	output wire [NSD-1:0] sd_cmd_en,
	input wire [NSD-1:0] sd_cmd_in,
	output wire [NSD-1:0] sd_cmd_r,
	output wire [NSD-1:0] sd_dat_en,
	input wire [NSD-1:0] sd_dat0_in,
	input wire [NSD-1:0] sd_dat1_in,
	input wire [NSD-1:0] sd_dat2_in,
	input wire [NSD-1:0] sd_dat3_in,
	output wire [NSD-1:0] sd_dat0_r,
	output wire [NSD-1:0] sd_dat1_r,
	output wire [NSD-1:0] sd_dat2_r,
	output wire [NSD-1:0] sd_dat3_r,

	output wire [15:0] debug,

	input wire shutdown	/* not used */
);

/*
	CMD_SD_QUEUE <channel> <data>(str)
	RSP_SD_CMDQ <channel> <data>(str)
	RSP_SD_DATQ <channel> <data>(str)
*/

localparam NSD_BITS = $clog2(NSD) ? $clog2(NSD) : 1;
localparam DOUT_ADDR_BITS = 11; /* one BRAM */
reg [NSD_BITS-1:0] channel = 0;

wire [7:0] sd_cmd_data = arg_data[7:0];
reg [NSD-1:0] sd_cmd_valid = 0;
wire [NSD-1:0] sd_cmd_full;
wire [7:0] sd_output_data[NSD];
wire [DOUT_ADDR_BITS-1:0] sd_output_elemcnt[NSD];
reg [NSD-1:0] sd_output_advance = 0;
wire [NSD-1:0] sd_output_start;
genvar gi;
generate
	for (gi = 0; gi < NSD; gi = gi + 1) begin : gensd
		sdc #(
			.HZ(HZ),
			.DOUT_ADDR_BITS(DOUT_ADDR_BITS)
		) u_sdc (
			.clk(clk),
			.payload_data(),
			.payload_valid(),
			.cmd_data(sd_cmd_data),
			.cmd_valid(sd_cmd_valid[gi]),
			.cmd_full(sd_cmd_full[gi]),
			.output_data(sd_output_data[gi]),
			.output_elemcnt(sd_output_elemcnt[gi]),
			.output_start(sd_output_start[gi]),
			.output_advance(sd_output_advance[gi]),

			/* SD card signals */
			.sd_clk(sd_clk[gi]),
			.sd_cmd_en(sd_cmd_en[gi]),
			.sd_cmd_in(sd_cmd_in[gi]),
			.sd_cmd_r(sd_cmd_r[gi]),
			.sd_dat_en(sd_dat_en[gi]),
			.sd_dat0_in(sd_dat0_in[gi]),
			.sd_dat1_in(sd_dat1_in[gi]),
			.sd_dat2_in(sd_dat2_in[gi]),
			.sd_dat3_in(sd_dat3_in[gi]),
			.sd_dat0_r(sd_dat0_r[gi]),
			.sd_dat1_r(sd_dat1_r[gi]),
			.sd_dat2_r(sd_dat2_r[gi]),
			.sd_dat3_r(sd_dat3_r[gi])
		);
	end
endgenerate

localparam PS_IDLE			= 0;
localparam PS_SD_QUEUE_1		= 1;
localparam PS_SD_QUEUE_2		= 2;
localparam PS_WAIT_GRANT		= 3;
localparam PS_SD_OUT_1			= 4;
localparam PS_SD_OUT_2			= 5;
localparam PS_SD_OUT_3			= 6;
localparam PS_SD_OUT_4			= 7;
localparam PS_SD_OUT_5			= 8;
localparam PS_SD_OUT_6			= 9;
localparam PS_MAX			= 9;

localparam PS_BITS= $clog2(PS_MAX + 1);
reg [PS_BITS-1:0] state = 0;
reg [6:0] cmd_len; /* counter for string len */
reg [DOUT_ADDR_BITS-1:0] out_cnt;

integer i;
always @(posedge clk) begin
	cmd_done <= 0;
	sd_cmd_valid <= 0;
	arg_advance <= 1;
	sd_output_advance <= 0;
	param_write <= 0;
	if (state == PS_IDLE && cmd_ready) begin
		// common to all cmds
		channel <= arg_data[NSD_BITS-1:0];
		if (cmd == CMD_SD_QUEUE) begin
			state <= PS_SD_QUEUE_1;
		end else begin
			cmd_done <= 1;
		end
	end else if (state == PS_SD_QUEUE_1) begin
		cmd_len <= arg_data;
		arg_advance <= 0;
		state <= PS_SD_QUEUE_2;
	end else if (state == PS_SD_QUEUE_2) begin
		if (cmd_len == 0) begin
			cmd_done <= 1;
			state <= PS_IDLE;
		end else begin
			cmd_len <= cmd_len - 1;
			sd_cmd_valid[channel] <= 1;
		end
	end else if (state == PS_IDLE && sd_output_start) begin
		invol_req <= 1;
		state <= PS_WAIT_GRANT;
	end else if (state == PS_WAIT_GRANT && invol_grant) begin
		invol_req <= 0;
		for (i = 0; i < NSD; i = i + 1) begin
			if (sd_output_start[i]) begin
				channel <= i;
				state <= PS_SD_OUT_1;
				i = NSD;	/* break */
			end
		end
	end else if (state == PS_SD_OUT_1) begin
		param_data <= channel;
		param_write <= 1;
		if (sd_output_elemcnt[channel] > 64)
			out_cnt <= 64;
		else
			out_cnt <= sd_output_elemcnt[channel];
		state <= PS_SD_OUT_2;
	end else if (state == PS_SD_OUT_2) begin
		param_data <= { 1'b1, { 32 - DOUT_ADDR_BITS { 1'b0 }}, out_cnt };
		param_write <= 1;
		state <= PS_SD_OUT_3;
	end else if (state == PS_SD_OUT_3) begin
		/* wait for data */
		state <= PS_SD_OUT_4;
	end else if (state == PS_SD_OUT_4) begin
		/* wait for data */
		state <= PS_SD_OUT_5;
	end else if (state == PS_SD_OUT_5) begin
		if (out_cnt == 0) begin
			state <= PS_SD_OUT_6;
		end else begin
			param_data <= { 25'h1000000, sd_output_data[channel] };
			param_write <= 1;
			sd_output_advance[channel] <= 1;
			out_cnt <= out_cnt - 1;
			state <= PS_SD_OUT_3;
		end
	end else if (state == PS_SD_OUT_6) begin
		cmd_done <= 1;
		param_data <= RSP_SD_CMDQ;
		state <= PS_IDLE;
	end
end

`ifdef notyet
assign debug[0] = data_valid[0];
assign debug[1] = data_ack[0];
assign debug[2] = dr_state[0];
assign debug[4:3] = dr_state[0];
assign debug[7:5] = state;
assign debug[8] = data[0][7:0];
assign debug[15:9] = 0;
`endif

endmodule
