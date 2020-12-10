`timescale 1ns / 1ps
`default_nettype none

/*
 * simple fifo. The next data is always available
 * at dout, but will only become available with
 * the clock after rd_en going high
 * If rd_en is kept high, data will only be available
 * with one clock delay.
 */
module fifo #(
	parameter DATA_WIDTH = 0,
	parameter ADDR_WIDTH = 0
) (
	input wire clk,
	input wire clr,
	// write side
	input wire [DATA_WIDTH - 1 : 0] din,
	input wire wr_en,
	output wire full,
	// read side
	output reg [DATA_WIDTH - 1 : 0] dout,
	input wire rd_en,
	output reg empty = 1,

	// status
	output reg [ADDR_WIDTH - 1 : 0] elemcnt
);

localparam ADDRS = 1 << ADDR_WIDTH;
reg [DATA_WIDTH - 1 : 0] ram[ADDRS - 1 : 0];

reg [ADDR_WIDTH - 1 : 0] rdptr = 0;
reg [ADDR_WIDTH - 1 : 0] wrptr = 0;

wire [ADDR_WIDTH - 1 : 0] next_rdptr = rdptr + 1;
wire [ADDR_WIDTH - 1 : 0] next_wrptr = wrptr + 1;

wire _empty = wrptr == rdptr;
assign full = next_wrptr == rdptr;
wire [ADDR_WIDTH - 1 : 0] _elemcnt = wrptr - rdptr;

always @(posedge clk) begin
	dout <= ram[rdptr];
	if (clr) begin
		rdptr <= 0;
		wrptr <= 0;
	end else begin
		if (rd_en && !_empty) begin
			rdptr <= next_rdptr;
		end
		if (wr_en && !full) begin
			ram[wrptr] <= din;
			wrptr <= next_wrptr;
		end
	end
	/* delay one slot to be in sync with output data */
	empty <= _empty;
	elemcnt <= _elemcnt;
end

endmodule
