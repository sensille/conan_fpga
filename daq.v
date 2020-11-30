`timescale 1ns / 1ps
`default_nettype none

module daq #(
	parameter NDAQ = 0,
	parameter MAC_PACKET_BITS = 0
) (
	input wire clk,
	input wire [31:0] systime,

	input wire [(32 * NDAQ) - 1:0] daq_data_in,
	input wire [NDAQ-1:0] daq_end,
	input wire [NDAQ-1:0] daq_valid,
	input wire [NDAQ-1:0] daq_req,
	output reg [NDAQ-1:0] daq_grant = 0,

	output reg [31:0] daqo_data,
	input wire daqo_data_rd_en,
	output wire [MAC_PACKET_BITS-1:0] daqo_len,
	output wire daqo_len_ready,
	input wire daqo_len_rd_en
);

localparam NDAQ_BITS = $clog2(NDAQ);
localparam BUFFER_DEPTH = 4096;
localparam BUFFER_BITS = $clog2(BUFFER_DEPTH);

wire [31:0] daq_data[NDAQ];
/* unpack daq_data due to lack of SV */
genvar gi;
generate
	for (gi = 0; gi < NDAQ; gi = gi + 1)
		assign daq_data[gi] = daq_data_in[((gi+1) * 32)-1:(gi * 32)];
endgenerate

/* instantiate fifos */
reg [MAC_PACKET_BITS-1:0] len_fifo_data = 0;
wire len_fifo_empty;
reg len_fifo_wr_en = 0;
fifo #(
	.DATA_WIDTH(MAC_PACKET_BITS),
	.ADDR_WIDTH(BUFFER_BITS) /* each entry could be a single packet */
) len_fifo (
	.clk(clk),
	.clr(1'b0),

	// write side
	.din(len_fifo_data),
	.wr_en(len_fifo_wr_en),
	.full(),

	// read side
	.dout(daqo_len),
	.rd_en(daqo_len_rd_en),
	.empty(len_fifo_empty),

	// status
	.elemcnt()
);

reg [31:0] data_fifo_data;
reg data_fifo_wr_en;
wire data_fifo_full;
fifo #(
	.DATA_WIDTH(32),
	.ADDR_WIDTH(BUFFER_BITS)
) data_fifo (
	.clk(clk),
	.clr(1'b0),

	// write side
	.din(data_fifo_data),
	.wr_en(data_fifo_wr_en),
	.full(data_fifo_full),

	// read side
	.dout(daqo_data),
	.rd_en(daqo_data_rd_en),
	.empty(),

	// status
	.elemcnt()
);
assign daqo_len_ready = !len_fifo_empty;

localparam DA_IDLE	= 0;
localparam DA_GRANTED	= 1;
localparam DA_MAX	= 1;
localparam DA_BITS = $clog2(DA_MAX + 1);
reg [DA_BITS-1:0] state = DA_IDLE;
integer i;
reg [NDAQ_BITS-1:0] daq;
always @(posedge clk) begin
	data_fifo_wr_en <= 0;
	len_fifo_wr_en <= 0;
	for (i = 0; i < NDAQ; i = i + 1) begin
		if (daq_grant[i])
			daq_grant[i] <= 0;
	end
	/* for now just use a priority encoder */
	if (state == DA_IDLE) begin
		for (i = 0; i < NDAQ; i = i + 1) begin
			if (daq_req[i]) begin
				daq <= i;
				daq_grant[i] <= 1;
				len_fifo_data <= 0;
				state <= DA_GRANTED;
				i = NDAQ;
			end
		end
	end else if (state == DA_GRANTED) begin
		if (daq_valid[daq]) begin
			data_fifo_data <= daq_data[daq];
			data_fifo_wr_en <= 1;
			len_fifo_data <= len_fifo_data + 1;
		end
		if (daq_end[daq]) begin
			len_fifo_wr_en <= 1;
			state <= DA_IDLE;
		end
	end
end

endmodule
