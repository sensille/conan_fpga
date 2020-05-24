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
reg [NSD_BITS-1:0] channel = 0;

wire [7:0] sd_cmd_data = arg_data[7:0];
reg [NSD-1:0] sd_cmd_valid = 0;
wire [NSD-1:0] sd_cmd_full;
wire [7:0] sd_output_data[NSD];
wire [NSD-1:0] sd_output_empty;
reg [NSD-1:0] sd_output_advance = 0;
genvar gi;
generate
	for (gi = 0; gi < NSD; gi = gi + 1) begin : gensd
		sdc #(
			.HZ(HZ)
		) u_sdc (
			.clk(clk),
			.payload_data(),
			.payload_valid(),
			.cmd_data(sd_cmd_data),
			.cmd_valid(sd_cmd_valid[gi]),
			.cmd_full(sd_cmd_full[gi]),
			.output_data(sd_output_data[gi]),
			.output_empty(sd_output_empty[gi]),
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
localparam PS_MAX			= 2;

localparam PS_BITS= $clog2(PS_MAX + 1);
reg [PS_BITS-1:0] state = 0;
reg [6:0] cmd_len; /* counter for string len */

always @(posedge clk) begin
	cmd_done <= 0;
	sd_cmd_valid <= 0;
	arg_advance = 1;
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
		state <= PS_SD_QUEUE_2;
	end else if (state == PS_SD_QUEUE_2) begin
		if (cmd_len == 0) begin
			cmd_done <= 1;
			state <= PS_IDLE;
		end else begin
			cmd_len <= cmd_len - 1;
			sd_cmd_valid[channel] <= 1;
		end
`ifdef notyet
	end else if (state == PS_IDLE && data_valid) begin
		invol_req <= 1;
		state <= PS_WAIT_GRANT;
	end else if (state == PS_WAIT_GRANT && invol_grant) begin
		invol_req <= 0;
		for (i = 0; i < NSD; i = i + 1) begin
			if (data_valid[i]) begin
				channel <= i;
				state <= PS_SD_DATA_1;
				i = NSD;
			end
		end
	end else if (state == PS_SD_DATA_1) begin
		param_data <= channel;
		param_write <= 1;
		state <= PS_SD_DATA_2;
	end else if (state == PS_SD_DATA_2) begin
		param_data <= starttime[channel];
		state <= PS_SD_DATA_3;
	end else if (state == PS_SD_DATA_3) begin
		param_data <= data[channel];
		state <= PS_SD_DATA_4;
	end else if (state == PS_SD_DATA_4) begin
		param_data <= bits[channel];
		state <= PS_SD_DATA_5;
		data_ack[channel] <= 1;
	end else if (state == PS_SD_DATA_5) begin
		cmd_done <= 1;
		param_write <= 0;
		param_data <= RSP_SD_DATA;
		state <= PS_IDLE;
`endif
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
