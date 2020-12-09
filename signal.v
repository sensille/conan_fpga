`timescale 1ns / 1ps
`default_nettype none

module signal #(
	parameter HZ = 0,
	parameter SIG_WIDTH = 18,
	parameter CMD_BITS = 8,
	parameter CMD_CONFIG_SIGNAL = 0,
	parameter SIG_WAIT_FRAC = 10,
	parameter RLE_BITS = 20
) (
	input wire clk,
	input wire [63:0] systime,

	output reg [31:0] daq_data,
	output reg daq_end,
	output reg daq_valid = 0,
	output reg daq_req = 0,
	input wire daq_grant,

	input wire [31:0] arg_data,
	output wire arg_advance,
	input wire [CMD_BITS-1:0] cmd,
	input wire cmd_ready,
	output reg cmd_done = 0,
	output reg [32:0] param_data = 0,
	output reg param_write = 0,
	output reg invol_req = 0,
	input wire invol_grant,


	input wire [SIG_WIDTH-1:0] signal
);

/* XXX SIG_WIDTH currently must be <= 29 */

localparam DAQT_SIGNAL_DATA	= 64;
localparam DAQ_PACKET_SIZE	= 100;
localparam DAQ_PACKET_BITS	= $clog2(DAQ_PACKET_SIZE);
localparam NSLOTS		= 7;
localparam SLOT_BITS = $clog2(NSLOTS + 1);

reg [SLOT_BITS-1:0] slot = 0;
reg [SIG_WIDTH-1:0] pipeline[NSLOTS];
reg [SIG_WIDTH-1:0] recv = 0;
integer i;
initial begin
	for (i = 0; i < NSLOTS; i = i + 1) begin
		pipeline[i] = 0;
	end
end

reg [SIG_WIDTH-1:0] mask = 0;
reg enabled = 0;
reg prev_enabled = 0;

/*
 * it might be worth to optimize it by raising the
 * clock to 8x clk
 */

/*
 * LRU input -> index lookup
 */
integer j;
always @(posedge clk) begin
	recv <= signal & mask;
	prev_enabled <= enabled;

	if (enabled == 0) begin
		/* do nothing */
	end else if (prev_enabled == 0) begin
		pipeline[0] <= 0;
		pipeline[1] <= 0;
		pipeline[2] <= 0;
		pipeline[3] <= 0;
		pipeline[4] <= 0;
		pipeline[5] <= 0;
		pipeline[6] <= 0;
	end else if (recv == pipeline[0]) begin
		slot <= 1;
	end else if (recv == pipeline[1]) begin
		slot <= 2;
		pipeline[0] <= recv;
		pipeline[1] <= pipeline[0];
	end else if (recv == pipeline[2]) begin
		slot <= 3;
		pipeline[0] <= recv;
		pipeline[1] <= pipeline[0];
		pipeline[2] <= pipeline[1];
	end else if (recv == pipeline[3]) begin
		slot <= 4;
		pipeline[0] <= recv;
		pipeline[1] <= pipeline[0];
		pipeline[2] <= pipeline[1];
		pipeline[3] <= pipeline[2];
	end else if (recv == pipeline[4]) begin
		slot <= 5;
		pipeline[0] <= recv;
		pipeline[1] <= pipeline[0];
		pipeline[2] <= pipeline[1];
		pipeline[3] <= pipeline[2];
		pipeline[4] <= pipeline[3];
	end else if (recv == pipeline[5]) begin
		slot <= 6;
		pipeline[0] <= recv;
		pipeline[1] <= pipeline[0];
		pipeline[2] <= pipeline[1];
		pipeline[3] <= pipeline[2];
		pipeline[4] <= pipeline[3];
		pipeline[5] <= pipeline[4];
	end else if (recv == pipeline[6]) begin
		slot <= 7;
		pipeline[0] <= recv;
		pipeline[1] <= pipeline[0];
		pipeline[2] <= pipeline[1];
		pipeline[3] <= pipeline[2];
		pipeline[4] <= pipeline[3];
		pipeline[5] <= pipeline[4];
		pipeline[6] <= pipeline[5];
	end else begin
		slot <= 0;
		pipeline[0] <= recv;
		pipeline[1] <= pipeline[0];
		pipeline[2] <= pipeline[1];
		pipeline[3] <= pipeline[2];
		pipeline[4] <= pipeline[3];
		pipeline[5] <= pipeline[4];
		pipeline[6] <= pipeline[5];
	end
end

/*
 * RLE on slot
 */
localparam PLEN_BITS = $clog2(SIG_WIDTH + 3);
reg [RLE_BITS-1:0] slot_cnt;
reg [SIG_WIDTH-1:0] deferred;
reg first_loop = 1;
reg [SIG_WIDTH+2:0] push_data;
reg [PLEN_BITS-1:0] push_len = 0;
reg [RLE_BITS-1:0] push_clks;
reg [SLOT_BITS-1:0] prev_slot;
always @(posedge clk) begin
	push_len <= 0;
	if (enabled == 0) begin
		/* do nothing */
	end else if (prev_enabled == 0) begin
		prev_slot <= 0;
		first_loop <= 1;
	end else if (slot == 0 && prev_slot != 0) begin
		if (slot_cnt[RLE_BITS-1:4] == 0) begin
			push_len <= SLOT_BITS + 5;
			push_data <= { prev_slot, 1'b0, slot_cnt[3:0] };
		end else if (slot_cnt[RLE_BITS-1:8] == 0) begin
			push_len <= SLOT_BITS + 10;
			push_data <= { prev_slot, 2'b10, slot_cnt[7:0] };
		end else begin
			push_len <= SLOT_BITS + 3 + RLE_BITS;
			push_data <= { prev_slot, 3'b110, slot_cnt };
		end
		push_clks <= slot_cnt;
		deferred <= pipeline[0];
	end else if (slot == 0 && prev_slot == 0) begin
		if (!first_loop) begin
			push_len <= SIG_WIDTH + 3;
			push_data <= { 3'b000, deferred };
			push_clks <= 1;
		end
		deferred <= pipeline[0];
	end else if (slot != 0 && prev_slot == 0) begin
		if (!first_loop) begin
			push_len <= SIG_WIDTH + 3;
			push_data <= { 3'b000, deferred };
			push_clks <= 1;
		end
		slot_cnt <= 1;
	end else if (slot == prev_slot && slot_cnt != (1 << RLE_BITS) - 1) begin
		slot_cnt <= slot_cnt + 1;
	end else begin
		if (slot_cnt[RLE_BITS-1:4] == 0) begin
			push_len <= SLOT_BITS + 5;
			push_data <= { prev_slot, 1'b0, slot_cnt[3:0] };
		end else if (slot_cnt[RLE_BITS-1:8] == 0) begin
			push_len <= SLOT_BITS + 10;
			push_data <= { prev_slot, 2'b10, slot_cnt[7:0] };
		end else begin
			push_len <= SLOT_BITS + 3 + RLE_BITS;
			push_data <= { prev_slot, 3'b110, slot_cnt };
		end
		push_clks <= slot_cnt;
		slot_cnt <= 1;
	end
	prev_slot <= slot;
	first_loop <= 0;
end

/*
 * push data into bitstream
 */
reg [31:0] pbuf = 0;
reg [4:0] pbuflen = 0;
reg [(32 + 5 + RLE_BITS + 1) - 1:0] stream_data;
reg stream_data_valid = 0;
reg [4:0] next_mark = 0;
/*
 * assuming no 2 packets with full RLE bits encoding
 * fit into 32 bits, we only need room for one full
 * RLE count and a few
 */
reg [RLE_BITS + 1:0] stream_clks = 0;
always @(posedge clk) begin
	stream_data_valid <= 0;
	if (push_len) begin
		stream_clks <= stream_clks + push_clks;
		if (pbuflen + push_len >= 32) begin
			pbuflen <= pbuflen + push_len - 32;
			next_mark <= pbuflen + push_len - 32;
			stream_data[31:0] <= (pbuf << (32 - pbuflen)) | (push_data >> (push_len - (32 - pbuflen)));
			stream_data[36:32] <= next_mark; /* position mark */
			stream_data_valid <= 1;
			stream_data[37 + RLE_BITS:37] <= stream_clks + push_clks;
			stream_clks <= 0;
		end else begin
			pbuflen <= pbuflen + push_len;
		end
		pbuf <= (pbuf << push_len) | push_data;
	end
end

/*
 * fifo to output
 */
wire [(32 + 5 + RLE_BITS):0] fifo_out;
reg fifo_out_rd_en = 0;
wire [7:0] fifo_elemcnt;
fifo #(
	.DATA_WIDTH(32 + 5 + RLE_BITS + 1),
	.ADDR_WIDTH(8)
) u_fifo (
	.clk(clk),
	.clr(1'b0),

	// write side
	.din(stream_data),
	.wr_en(stream_data_valid),
	.full(),

	// read side
	.dout(fifo_out),
	.rd_en(fifo_out_rd_en),
	.empty(),

	// status
	.elemcnt(fifo_elemcnt)
);

localparam DAQ_TIMEOUT	= HZ/SIG_WAIT_FRAC;
localparam TIMEOUT_BITS = $clog2(DAQ_TIMEOUT);
reg [TIMEOUT_BITS-1:0] st_timer = 0;
reg [31:0] recovered_systime = 0;
reg [31:0] latched_systime;

localparam ST_IDLE	= 0;
localparam ST_WAIT_GRANT= 1;
localparam ST_HEADER_2	= 2;
localparam ST_SEND	= 3;
localparam ST_MAX	= 3;
localparam ST_BITS = $clog2(ST_MAX + 1);
reg [ST_BITS-1:0] st_state = ST_IDLE;
reg [DAQ_PACKET_BITS-1:0] st_len = 0;

always @(posedge clk) begin
	daq_valid <= 0;
	daq_end <= 0;
	fifo_out_rd_en <= 0;

	if (enabled == 0) begin
		/* do nothing */
	end else if (prev_enabled == 0) begin
		recovered_systime <= systime;
	end else if (st_state == ST_IDLE) begin
		if (fifo_elemcnt >= DAQ_PACKET_SIZE ||
		    (fifo_elemcnt && st_timer == 1)) begin
			latched_systime <= recovered_systime;
			st_timer <= 0;
			daq_req <= 1;
			if (fifo_elemcnt > DAQ_PACKET_SIZE)
				st_len <= DAQ_PACKET_SIZE;
			else
				st_len <= fifo_elemcnt;
			st_state <= ST_WAIT_GRANT;
		end else if (st_timer) begin
			st_timer <= st_timer - 1;
		end else if (fifo_elemcnt) begin
			st_timer <= DAQ_TIMEOUT;
		end
	end else if (st_state == ST_WAIT_GRANT && daq_grant) begin
		daq_req <= 0;
		daq_data[31:24] <= DAQT_SIGNAL_DATA;
		daq_data[23:16] <= fifo_out[36:32];
		daq_data[15:8] <= st_len;
		daq_data[7:0] <= SIG_WIDTH;
		daq_valid <= 1;
		fifo_out_rd_en <= 1; /* delayed, request two slots in advance */
		st_state <= ST_HEADER_2;
	end else if (st_state == ST_HEADER_2) begin
		daq_data <= latched_systime[31:0];
		daq_valid <= 1;
		if (st_len != 1)
			fifo_out_rd_en <= 1;
		st_state <= ST_SEND;
	end else if (st_state == ST_SEND) begin
		/* elemcnt is delayed by one cycle */
		if (st_len == 0) begin
			daq_end <= 1;
			st_state <= ST_IDLE;
		end else begin
			daq_data <= fifo_out[31:0];
			daq_valid <= 1;
			st_len <= st_len - 1;
			if (st_len > 2)
				fifo_out_rd_en <= 1;
			recovered_systime <= recovered_systime + fifo_out[37 + RLE_BITS - 1:37];
		end
	end
end

localparam PS_IDLE		= 0;
localparam PS_CONFIG_SIGNAL_1	= 1;
localparam PS_MAX		= 1;

localparam PS_BITS= $clog2(PS_MAX + 1);
reg [PS_BITS-1:0] state = PS_IDLE;

/* just keep asserted, we'll read one arg per clock */
assign arg_advance = 1;

always @(posedge clk) begin
	if (cmd_done)
		cmd_done <= 0;

	if (state == PS_IDLE && cmd_ready) begin
		if (cmd == CMD_CONFIG_SIGNAL) begin
			enabled <= arg_data[0];
			state <= PS_CONFIG_SIGNAL_1;
		end else begin
			cmd_done <= 1;
		end
	end else if (state == PS_CONFIG_SIGNAL_1) begin
		mask <= arg_data;
		cmd_done <= 1;
		state <= PS_IDLE;
	end
end
endmodule
