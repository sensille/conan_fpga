`timescale 1ns / 1ps
`default_nettype none

module mac #(
	parameter HZ = 0
) (
	input wire clk,

	output reg tx0 = 0,
	output reg tx1 = 0,
	output reg tx_en = 0,
	input wire rx_clk_in,

	input wire enable,
	input wire send_test_frame,

	output reg [15:0] debug
);

`ifdef VERILATOR
wire rx_clk = clk;
`else
wire rx_clk = rx_clk_in;
`endif

/*
Preamble: aa aa aa aa aa aa aa
SFD:      ab
dst mac:  6 bytes
src mac:  6 bytes
ether type: 2 bytes
payload: 46-1500 bytes
FCS:      4 bytes
interpacket gap: 12 byte

FCS: from src-mac to end of payload
left shifting CRC32 BZIP2 (poly = 0x4C11DB7, initial CRC = 0xFFFFFFFF, CRC is post complemented, verify value = 0x38FB2284)
payload lsb first, fcs msb first
ether type 0x800 == IP
*/


reg [7:0] testframe[72];
initial $readmemh("testframe.hex", testframe);

reg stf_1 = 0;
reg stf_2 = 0;

reg [1:0] dicnt;
reg [6:0] rptr;
reg [7:0] bout;
localparam MS_IDLE = 0;
localparam MS_START = 1;
localparam MS_RUNNING = 2;
reg [2:0] state = 0;
always @(posedge rx_clk) begin
	stf_1 <= send_test_frame;
	stf_2 <= stf_1;
	if (state == MS_IDLE && stf_2 == 1 && stf_1 == 0) begin
		state <= MS_START;
		rptr <= 0;
		dicnt <= 0;
	end else if (state == MS_START) begin
		bout <= testframe[rptr];
		state <= MS_RUNNING;
	end else if (state == MS_RUNNING) begin
		tx_en <= 1;
		tx0 <= bout[0];
		tx1 <= bout[1];
		bout <= { 2'b00, bout[7:2] };
		dicnt <= dicnt + 1;
		if (dicnt == 2) begin
			rptr <= rptr + 1;
		end else if (dicnt == 3) begin
			bout <= testframe[rptr];
		end else if (dicnt == 0) begin
			if (rptr == 72) begin
				tx_en <= 0;
				tx0 <= 0;
				tx1 <= 0;
				state <= MS_IDLE;
			end
		end
	end
end
endmodule
