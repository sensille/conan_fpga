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
	input wire daqo_len_rd_en,

	output wire [15:0] debug
);

localparam NDAQ_BITS = $clog2(NDAQ);
/* hopefully enough to fully hold the startup */
localparam BUFFER_DEPTH = 8192;	/* hopefully enough to fully hold the startup */
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

reg [31:0] data_ring[BUFFER_DEPTH];
reg [BUFFER_BITS-1:0] rptr;
reg [BUFFER_BITS-1:0] wptr;
reg [BUFFER_BITS-1:0] saved_wptr;
reg inited = 0;
wire data_ring_empty = rptr == wptr;
wire data_ring_full = rptr == wptr + 1;

always @(posedge clk) begin
	if (!inited) begin
		rptr <= 0;
		inited <= 1;
	end
	daqo_data <= data_ring[rptr];
	if (daqo_data_rd_en && !data_ring_empty)
		rptr <= rptr + 1;
end

assign daqo_len_ready = !len_fifo_empty;

localparam DA_IDLE	= 0;
localparam DA_GRANTED	= 1;
localparam DA_MAX	= 1;
localparam DA_BITS = $clog2(DA_MAX + 1);
reg [DA_BITS-1:0] state = DA_IDLE;
integer i;
reg discard = 0;
reg [NDAQ_BITS-1:0] daq;
reg [23:0] discarded_pkts = 0;
reg loop_done;
always @(posedge clk) begin
	if (!inited)
		wptr <= 0;
	len_fifo_wr_en <= 0;
	daq_grant <= 0;

	if (state == DA_IDLE && discarded_pkts && !data_ring_full) begin
		data_ring[wptr] <= { 8'hfe, discarded_pkts };
		wptr <= wptr + 1;
		discarded_pkts <= 0;
		len_fifo_data <= 1;
		len_fifo_wr_en <= 1;
	end else if (state == DA_IDLE) begin
		/* for now just use a priority encoder */
		/* verilator lint_off BLKSEQ */
		loop_done = 0;
		for (i = 0; i < NDAQ; i = i + 1) begin
			if (!loop_done && daq_req[i]) begin
				daq <= i;
				daq_grant[i] <= 1;
				len_fifo_data <= 0;
				saved_wptr <= wptr;
				state <= DA_GRANTED;
				loop_done = 1;
			end
		end
		/* verilator lint_on BLKSEQ */
	end else if (state == DA_GRANTED) begin
		if (daq_valid[daq]) begin
			if (!discard) begin
				if (data_ring_full) begin
					/* revert wptr */
					wptr <= saved_wptr;
					discard <= 1;
				end else begin
					data_ring[wptr] <= daq_data[daq];
					wptr <= wptr + 1;
					len_fifo_data <= len_fifo_data + 1;
				end
			end
		end
		if (daq_end[daq]) begin
			if (!discard) begin
				len_fifo_wr_en <= 1;
			end else begin
				if (discarded_pkts != 24'hffffff)
					discarded_pkts <= discarded_pkts + 1;
			end
			discard <= 0;
			state <= DA_IDLE;
		end
	end
end

assign debug[0] = state;
assign debug[4:1] = daq;
assign debug[9:5] = daq_req;
assign debug[10] = len_fifo_empty;
assign debug[15:11] = 0;

endmodule
