`timescale 1ns / 1ps
`default_nettype none

module framing #(
	parameter BAUD = 0,
	parameter RING_BITS = 0,
	parameter LEN_BITS = 0,
	parameter LEN_FIFO_BITS = 0,
	parameter HZ = 0
) (
	input wire clk,

	input wire rx,
	output wire tx,
	output wire cts,

	/*
	 * receive side
	 */
	output wire [7:0] msg_data,
	output wire msg_ready,
	input wire msg_rd_en,

	/*
	 * send side
	 */
	input wire send_fifo_wr_en,
	input wire [LEN_BITS-1:0] send_fifo_data,
	output wire send_fifo_full,

	/* ring buffer input */
	input wire [7:0] send_ring_data,
	input wire send_ring_wr_en,
	output wire send_ring_full,

	/* error, reset */
	output reg error = 0,
	input wire clr
);

localparam RING_SIZE = 1 << RING_BITS;

/*
 * sync rx into our clk domain
 * Initialize to one (idle)
 */
reg rx_s1 = 1;
reg rx_sync = 1;
always @(posedge clk) begin
	rx_s1 <= rx;
	rx_sync <= rx_s1;
end

/*
 * UART instantiation
 */
wire [7:0] rx_data;
wire rx_ready;
reg [7:0] tx_data = 0;
reg tx_en = 0;
wire tx_transmitting;

localparam CLOCK_DIVIDE = HZ / BAUD / 4; /* 4 phases per bit */
uart #(
        .CLOCK_DIVIDE(CLOCK_DIVIDE)
) uart_u (
	.clk(clk),
	.rst(1'b0),
	.rx(rx_sync),
	.tx(tx),
	.transmit(tx_en),
	.tx_byte(tx_data),
	.received(rx_ready),
	.rx_byte(rx_data),
	.is_receiving(),
	.is_transmitting(tx_transmitting),
	.recv_error()
);

reg [7:0] recv_ring [RING_SIZE-1:0];
reg [RING_BITS-1:0] recv_temp_wptr;
reg [RING_BITS-1:0] recv_wptr;
reg [RING_BITS-1:0] recv_rptr;
assign msg_data = recv_ring[recv_rptr];
assign msg_ready = recv_rptr != recv_wptr;

localparam RECV_EOF_CHAR = 8'h7e;

reg [7:0] crc16_in = 0;
reg [3:0] crc16_cnt = 0;
reg [15:0] crc16 = 0;
reg [7:0] recv_crc1 = 0;
reg [7:0] recv_crc2 = 0;
reg [7:0] recv_len = 0;
reg [7:0] recv_seq = 0;
reg [3:0] recv_next_seq = 0;
localparam RST_SOF = 3'd0;	/* start of frame (== len byte) */
localparam RST_SEQ = 3'd1;	/* read sequence byte */
localparam RST_DATA = 3'd2;	/* read data */
localparam RST_CRC1 = 3'd3;	/* read crc1 */
localparam RST_CRC2 = 3'd4;	/* read crc2 */
localparam RST_EOF = 3'd5;	/* read sync byte (end of frame) */
localparam RST_ERROR = 3'd6;	/* error state, set clr signal to get out */
localparam RST_INIT = 3'd7;	/* startup */
reg [2:0] recv_state = RST_INIT;

assign cts = (recv_rptr != recv_temp_wptr + 1'b1);

/*
 * packet receive state machine
 */
always @(posedge clk) begin
	if (recv_state == RST_INIT) begin
		recv_state <= RST_SOF;
		recv_rptr <= 0;
		recv_wptr <= 0;
		recv_temp_wptr <= 0;
	end else if (rx_ready) begin
		if (recv_state == RST_SOF) begin 
			if (rx_data == RECV_EOF_CHAR) begin
				/*
				 * additional sync bytes at start of frame
				 * are ignored
				 */
			end else if (rx_data < 5 || rx_data >= 64) begin
				/* bad len, go to error state */
				recv_state <= RST_ERROR;
			end else begin
				recv_state <= RST_SEQ;
				recv_len <= rx_data;
				crc16_in <= rx_data;
				crc16_cnt <= 8;
				crc16 <= 16'hffff;
				recv_temp_wptr <= recv_wptr;
			end
		end else if (recv_state == RST_SEQ) begin
			if (recv_len == 5)
				recv_state <= RST_CRC1; /* no data phase */
			else
				recv_state <= RST_DATA;
			recv_seq <= rx_data;
			crc16_in <= rx_data;
			crc16_cnt <= 8;
		end else if (recv_state == RST_DATA) begin
			recv_temp_wptr <= recv_temp_wptr + 1'b1;
			recv_ring[recv_temp_wptr] <= rx_data;
			recv_len <= recv_len - 1;
			crc16_in <= rx_data;
			crc16_cnt <= 8;
			if (recv_len == 6) begin
				recv_state <= RST_CRC1;
			end
		end else if (recv_state == RST_CRC1) begin
			recv_crc1 <= rx_data;
			recv_state <= RST_CRC2;
		end else if (recv_state == RST_CRC2) begin
			recv_crc2 <= rx_data;
			recv_state <= RST_EOF;
		end else if (recv_state == RST_EOF &&
		             rx_data == RECV_EOF_CHAR) begin
			recv_state <= RST_ERROR;
			if (crc16 != { recv_crc1, recv_crc2 }) begin
				/* bad crc, error */
			end else if (recv_seq[3:0] != recv_next_seq) begin
				/* bad seq, got to error state frame */
			end else if (recv_seq[7:4] != 4'b0001) begin
				/* invalid upper bits of seq, error */
			end else begin
				/*
				 * commit the write pointer to make the frame
				 * available to the reader
				 */
				recv_wptr <= recv_temp_wptr;
				recv_next_seq <= recv_seq + 1'b1;
				recv_state <= RST_SOF;
			end
		end else if (recv_state == RST_EOF) begin
			/* missing EOF, error */
			recv_state <= RST_ERROR;
			recv_temp_wptr <= recv_wptr;
		end else if (recv_state == RST_ERROR) begin
			error <= 1'b1;
		end
	end
	if (crc16_cnt != 0) begin
		/* crc16 CCITT, reflect in, reflect out, init 0xffff */
		crc16 <= { 1'b0, crc16[15:1] };
		crc16[3] <= crc16[4] ^ crc16_in[0] ^ crc16[0];
		crc16[10] <= crc16[11] ^ crc16_in[0] ^ crc16[0];
		crc16[15] <= crc16_in[0] ^ crc16[0];
		crc16_cnt <= crc16_cnt - 1'b1;
		crc16_in <= { 1'b0, crc16_in[7:1] };
	end

	/* read interface */
	if (msg_rd_en && msg_ready) begin
		recv_rptr <= recv_rptr + 1'b1;
	end

	/* reset */
	if (clr) begin
		recv_wptr <= 0;
		error <= 1'b0;
	end
end

/*
 * send side
 */
wire [LEN_BITS-1:0] send_fifo_rd_data;
reg send_fifo_rd_en = 0;
wire send_fifo_empty;
fifo #(
	.DATA_WIDTH(LEN_BITS),
	.ADDR_WIDTH(LEN_FIFO_BITS)
) send_len_fifo (
	.clk(clk),
	.clr(1'b0),

	// write side
	.din(send_fifo_data),
	.wr_en(send_fifo_wr_en),
	.full(send_fifo_full),

	// read side
	.dout(send_fifo_rd_data),
	.rd_en(send_fifo_rd_en),
	.empty(send_fifo_empty),

	// status
	.elemcnt()
);

reg [7:0] send_ring [RING_SIZE-1:0];
reg [RING_BITS-1:0] send_rptr;
reg [RING_BITS-1:0] send_wptr;
assign send_ring_full = (send_wptr + 1'b1) == send_rptr;

always @(posedge clk) begin
	if (send_ring_wr_en && !send_ring_full) begin
		send_ring[send_wptr] <= send_ring_data;
		send_wptr <= send_wptr + 1'b1;
	end
end

reg [7:0] send_crc16_in = 0;
reg [3:0] send_crc16_cnt = 0;
reg [15:0] send_crc16 = 0;
reg [7:0] send_crc1 = 0;
reg [7:0] send_crc2 = 0;
reg [LEN_BITS-1:0] send_len = 0;
reg [3:0] send_seq = 0;
localparam SST_IDLE = 3'd0;
localparam SST_SOF = 3'd1;	/* start of frame (== len byte) */
localparam SST_SEQ = 3'd2;	/* read sequence byte */
localparam SST_DATA = 3'd3;	/* read data */
localparam SST_CRC1 = 3'd4;	/* read crc1 */
localparam SST_CRC2 = 3'd5;	/* read crc2 */
localparam SST_EOF = 3'd6;	/* read sync byte (end of frame) */
localparam SST_INIT = 3'd7;	/* read sync byte (end of frame) */
reg [2:0] send_state = SST_INIT;

/*
 * send state machine
 */
always @(posedge clk) begin
	if (send_state == SST_INIT) begin
		send_state <= SST_IDLE;
		send_rptr <= 0;
		send_wptr <= 0;
	end else if (!tx_transmitting && !tx_en) begin
		if (send_state == SST_IDLE && !send_fifo_empty) begin
			send_state <= SST_SOF;
			send_len <= send_fifo_rd_data + 8'd5;
			send_fifo_rd_en <= 1;
		end else if (send_state == SST_SOF) begin
			send_state <= SST_SEQ;
			tx_data <= send_len;
			tx_en <= 1;
			send_crc16_in <= send_len;
			send_crc16_cnt <= 8;
			send_crc16 <= 16'hffff;
		end else if (send_state == SST_SEQ) begin
			tx_data <= { 4'b001, send_seq };
			tx_en <= 1;
			send_crc16_in <= { 4'b001, send_seq };
			send_crc16_cnt <= 8;
			send_seq <= send_seq + 1;
			if (send_len != 5) begin
				send_state <= SST_DATA;
			end else begin
				send_state <= SST_CRC1;
			end
		end else if (send_state == SST_DATA) begin
			tx_data <= send_ring[send_rptr];
			tx_en <= 1;
			send_crc16_in <= send_ring[send_rptr];
			send_crc16_cnt <= 8;
			send_rptr <= send_rptr + 1;
			send_len <= send_len - 1;
			if (send_len == 6) begin
				send_state <= SST_CRC1;
			end
		end else if (send_state == SST_CRC1) begin
			tx_data <= send_crc16[15:8];
			tx_en <= 1;
			send_state <= SST_CRC2;
		end else if (send_state == SST_CRC2) begin
			tx_data <= send_crc16[7:0];
			tx_en <= 1;
			send_state <= SST_EOF;
		end else if (send_state == SST_EOF) begin
			tx_data <= RECV_EOF_CHAR;
			tx_en <= 1;
			send_state <= SST_IDLE;
		end
	end
	if (tx_en) begin
		tx_en <= 0;
	end
	if (send_fifo_rd_en) begin
		send_fifo_rd_en <= 0;
	end
	if (send_crc16_cnt != 0) begin
		/* crc16 CCITT, reflect in, reflect out, init 0xffff */
		send_crc16 <= { 1'b0, send_crc16[15:1] };
		send_crc16[3] <= send_crc16[4] ^ send_crc16_in[0] ^
			send_crc16[0];
		send_crc16[10] <= send_crc16[11] ^ send_crc16_in[0] ^
			send_crc16[0];
		send_crc16[15] <= send_crc16_in[0] ^ send_crc16[0];
		send_crc16_cnt <= send_crc16_cnt - 1'b1;
		send_crc16_in <= { 1'b0, send_crc16_in[7:1] };
	end
end

endmodule
