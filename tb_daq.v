`timescale 1ns / 1ps
`default_nettype none

module tb_daq #(
	parameter NDAQ = 5
) (
	input wire clk,
	input wire [31:0] systime,

	output wire eth_tx0,
	output wire eth_tx1,
	output wire eth_tx_en,
	input wire eth_rx_clk,

	input wire [31:0] daq_data[NDAQ],
	input wire [NDAQ-1:0] daq_valid,
	input wire [NDAQ-1:0] daq_end,
	input wire [NDAQ-1:0] daq_req,
	output wire [NDAQ-1:0] daq_grant,

	input wire [1:0] enable,

	output wire [31:0] sig_data,
	output reg sig_end,
	output reg sig_valid = 0,
	output reg sig_req = 0,
	input wire sig_grant,

	input wire [31:0] s_arg_data,
	output wire s_arg_advance,
	input wire [CMD_BITS-1:0] s_cmd,
	input wire s_cmd_ready,
	output wire s_cmd_done,
	output wire [32:0] s_param_data,
	output wire s_param_write,
	output wire s_invol_req,
	input wire s_invol_grant,

	input [SIG_WIDTH-1:0] signal
);

localparam CMD_BITS = 8;
localparam MAC_PACKET_BITS = 9; /* 2^9 * 4 bytes > 1500 */
localparam HZ = 48000000;

wire [31:0] daqo_data;
wire daqo_data_rd_en;
wire [MAC_PACKET_BITS-1:0] daqo_len;
wire daqo_len_ready;
wire daqo_len_rd_en;
/* no system verilog: flatten daq_data */
wire [(32 * NDAQ)-1:0] _daq_data;
genvar gi;
generate
	for (gi = 0; gi < NDAQ; gi = gi + 1)
		assign _daq_data[((gi+1) * 32)-1:(gi * 32)] = daq_data[gi];
endgenerate

daq #(
	.NDAQ(NDAQ),
	.MAC_PACKET_BITS(MAC_PACKET_BITS)
) u_daq (
	.clk(clk),
	.systime(systime),

	.daq_data_in(_daq_data),
	.daq_end(daq_end),
	.daq_valid(daq_valid),
	.daq_req(daq_req),
	.daq_grant(daq_grant),

	.daqo_data(daqo_data),
	.daqo_data_rd_en(daqo_data_rd_en),
	.daqo_len(daqo_len),
	.daqo_len_ready(daqo_len_ready),
	.daqo_len_rd_en(daqo_len_rd_en),

	.debug()
);

reg [47:0] src_mac = 48'h123456789abc;
reg [47:0] dst_mac = 48'h665544332211;
mac #(
	.HZ(HZ),
	.MAC_PACKET_BITS(MAC_PACKET_BITS),
	.PACKET_WAIT_FRAC(100),
	.ES_QUEUE(0),
	.ES_DISCARD(1),
	.ES_RUNNING(2)
) u_mac (
	.clk(clk),

	.tx0(eth_tx0),
	.tx1(eth_tx1),
	.tx_en(eth_tx_en),
	.rx_clk_in(eth_rx_clk),

	.daqo_data(daqo_data),
	.daqo_data_rd_en(daqo_data_rd_en),
	.daqo_len(daqo_len),
	.daqo_len_ready(daqo_len_ready),
	.daqo_len_rd_en(daqo_len_rd_en),

	.src_mac(src_mac),
	.dst_mac(dst_mac),

	.enable_in(enable),

	.debug()
);

localparam SIG_WIDTH	= 18;

reg [31:0] s_daq_data;
reg s_daq_end;
reg s_daq_valid = 0;
reg s_daq_req = 0;
wire s_daq_grant;

signal #(
	.HZ(HZ),
	.SIG_WIDTH(SIG_WIDTH),
	.CMD_BITS(CMD_BITS),
	.CMD_CONFIG_SIGNAL(28),
	.SIG_WAIT_FRAC(1000),
	.RLE_BITS(12),
	.FLUSH_FREQ(10000)
) u_signal (
	.clk(clk),
	.systime(systime),

	.daq_data(sig_data),
	.daq_end(sig_end),
	.daq_valid(sig_valid),
	.daq_req(sig_req),
	.daq_grant(sig_grant),

	.arg_data(s_arg_data),
	.arg_advance(s_arg_advance),
	.cmd(s_cmd),
	.cmd_ready(s_cmd_ready),
	.cmd_done(s_cmd_done),
	.param_data(s_param_data),
	.param_write(s_param_write),
	.invol_req(s_invol_req),
	.invol_grant(s_invol_grant),

	.signal(signal)
);

endmodule
