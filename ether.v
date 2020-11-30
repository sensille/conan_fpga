`timescale 1ns / 1ps
`default_nettype none

module ether #(
	parameter HZ = 0,
	parameter CMD_BITS = 0,
	parameter NETHER = 0,
	parameter NDAQ = 0,
	parameter MAC_PACKET_BITS = 0,
	parameter CMD_CONFIG_ETHER = 0,
	parameter CMD_ETHER_MD_READ = 0,
	parameter CMD_ETHER_MD_WRITE = 0,
	parameter RSP_ETHER_MD_READ = 0
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

	output wire [NETHER-1:0] eth_tx0,
	output wire [NETHER-1:0] eth_tx1,
	output wire [NETHER-1:0] eth_tx_en,
	input wire [NETHER-1:0] eth_rx_clk,
	output reg [NETHER-1:0] eth_mdc = 0,
	input wire [NETHER-1:0] eth_mdio_in,
	output reg [NETHER-1:0] eth_mdio_out = 0,
	output reg [NETHER-1:0] eth_mdio_en = 0,

	input wire [31:0] daqo_data,
	output wire daqo_data_rd_en,
	input wire [MAC_PACKET_BITS-1:0] daqo_len,
	input wire daqo_len_ready,
	output wire daqo_len_rd_en,

	output reg [15:0] debug,

	input wire shutdown	/* not used */
);

reg [47:0] src_mac = 48'hffffffffffff;
reg [47:0] dst_mac = 48'hffffffffffff;
mac #(
	.HZ(HZ),
	.MAC_PACKET_BITS(MAC_PACKET_BITS)
) u_mac (
	.clk(clk),
	.tx0(eth_tx0[0]),
	.tx1(eth_tx1[0]),
	.tx_en(eth_tx_en[0]),
	.rx_clk_in(eth_rx_clk[0]),

	.daqo_data(daqo_data),
	.daqo_data_rd_en(daqo_data_rd_en),
	.daqo_len(daqo_len),
	.daqo_len_ready(daqo_len_ready),
	.daqo_len_rd_en(daqo_len_rd_en),

	.src_mac(src_mac),
	.dst_mac(dst_mac),

	.debug(debug)
);

/*
	parameter CMD_CONFIG_ETHER = 0,
	CMD_ETHER_MD_READ <channel> <phy> <register>
	CMD_ETHER_MD_WRITE <channel> <phy> <register> <data>
	RSP_ETHER_MD_READ <channel> <data>
*/

/*
 * We go for a fixed 250k at the md interface
 */
localparam BAUD = 250000;
localparam BITTIME = HZ / BAUD / 2;
localparam BITTIME_BITS = $clog2(BITTIME);

localparam NETHER_BITS = $clog2(NETHER + 1); /* +1 to make the case NETHER=1 work */
reg [NETHER_BITS-1:0] channel = 0;

localparam PS_IDLE			= 0;
localparam PS_CONFIG_ETHER_1		= 1;
localparam PS_CONFIG_ETHER_2		= 2;
localparam PS_CONFIG_ETHER_3		= 3;
localparam PS_ETHER_MD_1		= 4;
localparam PS_ETHER_MD_2		= 5;
localparam PS_ETHER_MD_3		= 6;
localparam PS_ETHER_MD_4		= 7;
localparam PS_ETHER_MD_READING		= 8;
localparam PS_ETHER_MD_PREAMBLE		= 9;
localparam PS_ETHER_MD_FRAME		= 10;
localparam PS_ETHER_MD_DONE		= 11;
localparam PS_ETHER_MD_RESPOND		= 12;
localparam PS_ETHER_MD_RESPOND_1	= 13;
localparam PS_ETHER_MD_RESPOND_2	= 14;
localparam PS_MAX			= 14;
localparam PS_BITS = $clog2(PS_MAX + 1);
reg [PS_BITS-1:0] state = PS_IDLE;

/* just keep asserted, we'll read one arg per clock */
assign arg_advance = 1;

reg rdwr = 0;	/* read or write command */
reg [4:0] phy = 0;
reg [4:0] register = 0;
reg [15:0] data = 0;
reg [BITTIME_BITS-1:0] bittime = 0;
reg [6:0] bitcnt = 0;
reg [31:0] bitdata = 0;
reg mclk;

always @(posedge clk) begin
	if (cmd_done)
		cmd_done <= 0;

	if (bittime != 0)
		bittime <= bittime - 1;

	if (state == PS_IDLE && cmd_ready) begin
		// common to all cmds
		channel <= arg_data[NETHER_BITS-1:0];
		if (cmd == CMD_CONFIG_ETHER) begin
			state <= PS_CONFIG_ETHER_1;
		end else if (cmd == CMD_ETHER_MD_WRITE) begin
			state <= PS_ETHER_MD_1;
			rdwr <= 0;
		end else if (cmd == CMD_ETHER_MD_READ) begin
			state <= PS_ETHER_MD_1;
			rdwr <= 1;
		end else begin
			cmd_done <= 1;
		end
	end else if (state == PS_CONFIG_ETHER_1) begin
		src_mac[47:16] <= arg_data;
		state <= PS_CONFIG_ETHER_2;
	end else if (state == PS_CONFIG_ETHER_2) begin
		src_mac[15:0] <= arg_data[31:16];
		dst_mac[47:32] <= arg_data[15:0];
		state <= PS_CONFIG_ETHER_3;
	end else if (state == PS_CONFIG_ETHER_3) begin
		dst_mac[31:0] <= arg_data;
		cmd_done <= 1;
		state <= PS_IDLE;
	end else if (state == PS_ETHER_MD_1) begin
		/* CMD_ETHER_MD_READ in <channel> <phy> <register> */
		/* CMD_ETHER_MD_WRITE in <channel> <phy> <register> <data> */
		phy <= arg_data;
		state <= PS_ETHER_MD_2;
		eth_mdc[channel] <= 0;
		/*
		 * for the first cycle mclk will be != mdc. This is so
		 * that the state machine can push out the first bit
		 * before the first real mdc edge
		 */
		mclk <= 1;
	end else if (state == PS_ETHER_MD_2) begin
		register <= arg_data;
		if (rdwr)
			state <= PS_ETHER_MD_4;
		else
			state <= PS_ETHER_MD_3;
		/* turn line to output, start preamble */
		eth_mdio_out[channel] <= 1;
		eth_mdio_en[channel] <= 1;
	end else if (state == PS_ETHER_MD_3) begin
		data <= arg_data;
		state <= PS_ETHER_MD_4;
	end else if (state == PS_ETHER_MD_4) begin
		/* start transfer */
		bittime <= BITTIME;
		bitcnt <= 32;
		bitdata <= 32'hffffffff;
		state <= PS_ETHER_MD_PREAMBLE;
	end else if (bittime != 0) begin
		bittime <= bittime - 1;
	end else if (bitcnt != 0) begin
		mclk <= !mclk;
		eth_mdc[channel] <= !mclk;
		bittime <= BITTIME;
		if (mclk == 0 && state == PS_ETHER_MD_READING) begin
			bitdata <= { bitdata[30:0], eth_mdio_in[channel] };
			bitcnt <= bitcnt - 1;
			if (bitcnt != 1)
				bitcnt <= bitcnt - 1;
			else
				state <= PS_ETHER_MD_RESPOND;
		end else if (mclk == 1 && state != PS_ETHER_MD_READING) begin
			eth_mdio_out[channel] <= bitdata[31];
			bitdata <= { bitdata[30:0], 1'b0 };
			bitcnt <= bitcnt - 1;
			if (bitcnt == 1) begin
				if (state == PS_ETHER_MD_PREAMBLE) begin
					bitdata <= { 2'b01, rdwr, !rdwr, phy, register, 2'b11, data };
					if (rdwr)
						bitcnt <= 15;
					else
						bitcnt <= 33;	/* +1 to push the last bit out */
					state <= PS_ETHER_MD_FRAME;
				end else if (rdwr) begin
					bitcnt <= 18;
					eth_mdio_en[channel] <= 0;
					state <= PS_ETHER_MD_READING;
				end else begin
					eth_mdio_en[channel] <= 0;
					cmd_done <= 1;
					state <= PS_IDLE;
				end
			end
		end
	end else if (state == PS_ETHER_MD_RESPOND) begin
		param_data <= channel;
		param_write <= 1;
		state <= PS_ETHER_MD_RESPOND_1;
	end else if (state == PS_ETHER_MD_RESPOND_1) begin
		param_data <= bitdata[15:0];
		param_write <= 1;
		state <= PS_ETHER_MD_RESPOND_2;
	end else if (state == PS_ETHER_MD_RESPOND_2) begin
		param_data <= RSP_ETHER_MD_READ;
		param_write <= 0;
		cmd_done <= 1;
		state <= PS_IDLE;
	end
end

endmodule
