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

	input wire [1:0] enable
);

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
	.daqo_len_rd_en(daqo_len_rd_en)
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

endmodule
