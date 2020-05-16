`timescale 1ns / 1ps
`default_nettype none

module sd #(
	parameter HZ = 0,
	parameter CMD_BITS = 0,
	parameter NSD = 0,
	parameter CMD_CONFIG_SD = 0,
	parameter CMD_SD_INIT = 0,
	parameter CMD_SD_CMD = 0,
	parameter CMD_SD_DATA = 0,
	parameter RSP_SD_CMD = 0,
	parameter RSP_SD_DATA = 0
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

	output reg [NSD-1:0] sd_clk,
	inout wire [NSD-1:0] sd_cmd,
	inout wire [NSD-1:0] sd_dat0,
	inout wire [NSD-1:0] sd_dat1,
	inout wire [NSD-1:0] sd_dat2,
	inout wire [NSD-1:0] sd_dat3,

	output wire [15:0] debug,

	input wire shutdown	/* not used */
);

/*
	CMD_CONFIG_SD <channel> <clkdiv>
	CMD_SD_CMD <channel> <flags> <cmd>(str)
	RSP_SD_CMD <channel> <rsp>(str)
	CMD_SD_DATA <channel> <flags> <offset> <data>(str)
	RSP_SD_DATA <channel> <offset> <data>(str)
	clkdiv 0 means disable

	config: set clkdiv, generate 74 clocks

	command has always 48 bits of data
	command without response
	command with 48 bits response
	command with 136 bits response
	command with 48 bits response and reading 512 byte on DAT
	command with 48 bits response and writing 512 byte on DAT (optional wait for busy)
	command with 48 bits response and reading 512 bits on DAT (status)

	command <channel> <flags> <data>
		flags: rsp: none - 48 bits - 136 bits - 512 bits on DAT
                            wait for N packets of 512 byte on DAT
			    N
			    write N packets of 512 byte on DAT
			    wait for non-busy before command
			    queue without notify
			    queue with notify
			    will with own data
	write <channel> <data (up to 48 byte chunk)>	consecutive writes must fill the block exactly
	rsp read <channel> <data (up to 48 byte chunk)>	
*/

localparam NSD_BITS = $clog2(NSD) ? $clog2(NSD) : 1;
reg [NSD_BITS-1:0] channel = 0;

localparam TIMEOUT_BITS = $clog2(HZ);	/* max timeout 1s */

localparam PS_IDLE		= 0;
localparam PS_CONFIG_SD_1	= 1;
localparam PS_SD_INIT_1		= 2;
localparam PS_SD_INIT_2		= 3;
localparam PS_MAX		= 3;

localparam PS_BITS= $clog2(PS_MAX + 1);
reg [PS_BITS-1:0] state = 0;

reg [NSD-1:0] sd_cmd_r;
reg [NSD-1:0] sd_cmd_en = 0;
reg [NSD-1:0] sd_dat_r[4];
reg [NSD-1:0] sd_dat_en;
reg [NSD-1:0] sd_dat_in[4];

reg [11:0] clkdiv[NSD];
reg [11:0] clkcnt[NSD];
reg [7:0] initcnt;

always @(*) begin: sdmux
	integer i;

	for (i = 0; i < NSD; i++) begin
		if (sd_cmd_en[i])
			sd_cmd = sd_cmd_r;
		else
			sd_cmd = 1'bZ;
		if (sd_dat_en[i])
			{ sd_dat0[i], sd_dat1[i], sd_dat2[i], sd_dat3[i] }  = sd_cmd_r;
		else
			{ sd_dat0[i], sd_dat1[i], sd_dat2[i], sd_dat3[i] }  = 4'bZ;
		sd_dat_in[i] = { sd_dat0[i], sd_dat1[i], sd_dat2[i], sd_dat3[i] };
	end
end

always @(posedge clk) begin
	if (cmd_done)
		cmd_done <= 0;
	if (state == PS_IDLE && cmd_ready) begin
		// common to all cmds
		arg_advance = 1;
		channel <= arg_data[NSD_BITS-1:0];
		if (cmd == CMD_CONFIG_SD) begin
			state <= PS_CONFIG_SD_1;
		end else if (cmd == CMD_SD_INIT) begin
			state <= PS_SD_INIT_1;
		end else if (cmd == CMD_SD_CMD) begin
			state <= PS_SD_CMD_1;
		end else begin
			cmd_done <= 1;
		end
	end else if (state == PS_CONFIG_SD_1) begin
		clkdiv[channel] <= arg_data;
		sd_clk[channel] <= 1;
		sd_cmd_en[channel] <= 0;
		sd_dat_en[channel] <= 0;
		sd_cmd_r[channel] <= 1;
		sd_dat_r[channel] <= 4'b1111;
		state <= PS_IDLE;
		cmd_done <= 1;
	end else if (state == PS_SD_INIT_1) begin
		clkcnt[channel] <= clkdiv[channel];
		initcnt <= 2 * 74 - 1;	/* generate 74 clocks for startup */
		sd_clk[channel] <= 1;
		state <= PS_SD_INIT_2;
	end else if (state == PS_SD_INIT_2) begin
		if (clkcnt[channel] == 0) begin
			if (initcnt == 0) begin
				state <= PS_IDLE;
				cmd_done <= 1;
			end else begin
				sd_clk[channel] <= !sd_clk[channel];
				clkcnt[channel] <= clkdiv[channel];
				initcnt <= initcnt - 1;
			end
		end else begin
			clkcnt[channel] <= clkcnt[channel] - 1;
		end
	end else if (state == PS_SD_CMD_1) begin
		cmd_flags <= arg_data;
		state <= PS_SD_CMD_2;
	end else if (state == PS_SD_CMD_2) begin
		if (arg_data != 6) begin
			/* error */
			cmd_done <= 1;
			state <= PS_IDLE;
		end else begin
			state <= PS_SD_CMD_3;
		end
	end else if (state == PS_SD_CMD_3) begin
		send_enable <= 1;
		send_data <= arg_data;
		
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
