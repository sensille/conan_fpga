`timescale 1ns / 1ps
`default_nettype none

module mac #(
	parameter HZ = 0,
	parameter MAC_PACKET_BITS = 0,
	parameter PACKET_WAIT_FRAC = 0,
	parameter ES_QUEUE = 0,
	parameter ES_DISCARD = 0,
	parameter ES_RUNNING = 0
) (
	input wire clk,

	output reg tx0 = 0,
	output reg tx1 = 0,
	output reg tx_en = 0,
	input wire rx_clk_in,

	input wire [31:0] daqo_data,
	output reg daqo_data_rd_en = 0,
	input wire [MAC_PACKET_BITS-1:0] daqo_len,
	input wire daqo_len_ready,
	output reg daqo_len_rd_en = 0,

	input wire [47:0] src_mac,
	input wire [47:0] dst_mac,

	input wire [1:0] enable_in,

	output wire [15:0] debug
);

`ifdef VERILATOR
wire rx_clk = clk;
`else
wire rx_clk = rx_clk_in;
`endif

/*
Preamble: 55 55 55 55 55 55 55
SFD:      d5
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

/* sync ready in */
reg len_ready_sync_1 = 0;
reg len_ready = 0;
always @(posedge rx_clk) begin
	len_ready_sync_1 <= daqo_len_ready;
	len_ready <= len_ready_sync_1;
end
/* sync rd_en out */
reg data_rd_en_sync_1 = 0;
reg data_rd_en_sync_2 = 0;
reg data_rd_en = 0;
reg len_rd_en_sync_1 = 0;
reg len_rd_en_sync_2 = 0;
reg len_rd_en = 0;
always @(posedge clk) begin
	data_rd_en_sync_1 <= data_rd_en;
	data_rd_en_sync_2 <= data_rd_en_sync_1;
	daqo_data_rd_en <= data_rd_en_sync_1 != data_rd_en_sync_2;
	len_rd_en_sync_1 <= len_rd_en;
	len_rd_en_sync_2 <= len_rd_en_sync_1;
	daqo_len_rd_en <= len_rd_en_sync_1 != len_rd_en_sync_2;
end

reg [1:0] enable = 0;
reg [1:0] enable_sync_1 = 0;
reg [1:0] enable_sync_2 = 0;
always @(posedge rx_clk) begin
	enable_sync_1 <= enable_in;
	enable_sync_2 <= enable_sync_1;
	if (enable_sync_2 == enable_sync_1)
		enable <= enable_sync_2;
end

/*
 * we build frames always in multiples of 4 bytes
 * preamble is 8 bytes
 * src + dst mac is 12 bytes
 * ether type is 2 bytes, we add 2 bytes magic as alignment
 * data is always in multples of 4
 */
localparam MS_IDLE		= 0;
localparam MS_START		= 1;
localparam MS_PREAMBLE_1	= 2;
localparam MS_MAC_1		= 3;
localparam MS_MAC_2		= 4;
localparam MS_MAC_3		= 5;
localparam MS_ETHERTYPE		= 6;
localparam MS_PAYLOAD		= 7;
localparam MS_STUFF		= 8;
localparam MS_FCS		= 9;
localparam MS_WAIT_GAP		= 10;
localparam MS_GAP		= 11;
localparam MS_MAX		= 11;
localparam MS_BITS = $clog2(MS_MAX + 1);

localparam MAX_PACKET		= 375;

reg [3:0] len_wait = 0;
reg [MAC_PACKET_BITS-1:0] data_len = 0;
reg [MAC_PACKET_BITS-1:0] next_len = 0;
reg [MAC_PACKET_BITS-1:0] discard_len = 0;
reg crc_data_ready;
reg init_crc = 0;
reg [31:0] next_data;
reg next_eof = 0;
reg tx_start = 0;
reg [MS_BITS-1:0] state = MS_IDLE;
reg [7:0] idle_wait = 0;
reg next_data_wanted = 0;
reg [3:0] stuff_len;
localparam PACKET_WAIT_TIME = HZ / PACKET_WAIT_FRAC;
reg [$clog2(PACKET_WAIT_TIME) - 1:0] packet_wait = 0;
reg [15:0] ether_seq = 0;
always @(posedge rx_clk) begin
	crc_data_ready <= 0;
	tx_start <= 0;
	init_crc <= 0;

	if (len_wait)
		len_wait <= len_wait - 1;
	if (packet_wait)
		packet_wait <= packet_wait - 1;
	if (discard_len != 0) begin
		if (len_wait == 0) begin
			/* we use len_wait also for the data queue here */
			len_wait <= 12;
			data_rd_en <= !data_rd_en;	/* toggle triggers one read */
			discard_len <= discard_len - 1;
		end
	end else if (len_wait == 0 && len_ready) begin
		if (enable == ES_DISCARD) begin
			discard_len <= daqo_len;
			len_rd_en <= !len_rd_en; /* toggle triggers one read */
		end else if (enable == ES_QUEUE) begin
			/* just leave the packet in the queue */
		end else if (next_len + daqo_len <= MAX_PACKET) begin
			next_len <= next_len + daqo_len;
			len_rd_en <= !len_rd_en; /* toggle triggers one read */
			/*
			 * give it 12 clocks to sync the next len in and out.
			 * should be plenty
			 */
			len_wait <= 12;
			/* wait timer */
			if (next_len == 0)
				packet_wait <= PACKET_WAIT_TIME;
		end else if (state == MS_IDLE) begin
			/* packet is full, send immediately */
			data_len <= next_len;
			stuff_len <= 0;
			next_len <= 0;
			state <= MS_START;
		end
	end else if (state == MS_IDLE && next_len && packet_wait == 0) begin
		data_len <= next_len;
		if (next_len < 11)
			stuff_len <= 11 - next_len;
		else
			stuff_len <= 0;
		next_len <= 0;
		state <= MS_START;
	end

	if (state == MS_START) begin
		next_data <= 32'h55555555;
		tx_start <= 1;
		init_crc <= 1;
		state <= MS_PREAMBLE_1;
	end else if (state == MS_PREAMBLE_1 && next_data_wanted) begin
		next_data <= 32'h555555d5;
		state <= MS_MAC_1;
	end else if (state == MS_MAC_1 && next_data_wanted) begin
		next_data <= dst_mac[47:16];
		crc_data_ready <= 1;
		state <= MS_MAC_2;
	end else if (state == MS_MAC_2 && next_data_wanted) begin
		next_data <= { dst_mac[15:0], src_mac[47:32] };
		crc_data_ready <= 1;
		state <= MS_MAC_3;
	end else if (state == MS_MAC_3 && next_data_wanted) begin
		next_data <= src_mac[31:0];
		crc_data_ready <= 1;
		state <= MS_ETHERTYPE;
	end else if (state == MS_ETHERTYPE && next_data_wanted) begin
		next_data <= { 32'h5139, ether_seq };
		ether_seq <= ether_seq + 1;
		crc_data_ready <= 1;
		state <= MS_PAYLOAD;
	end else if (state == MS_PAYLOAD && next_data_wanted) begin
		next_data <= daqo_data;
		crc_data_ready <= 1;
		data_rd_en <= !data_rd_en;	/* toggle triggers one read */
		if (data_len == 1) begin
			if (stuff_len) begin
				state <= MS_STUFF;
			end else begin
				state <= MS_FCS;
			end
		end else begin
			data_len <= data_len - 1;
		end
	end else if (state == MS_STUFF && next_data_wanted) begin
		next_data <= 32'hffffffff;
		crc_data_ready <= 1;
		if (stuff_len == 1) begin
			state <= MS_FCS;
		end else begin
			stuff_len <= stuff_len - 1;
		end
	end else if (state == MS_FCS && next_data_wanted) begin
		next_data <= ~{ crc[7:0], crc[15:8], crc[23:16], crc[31:24] };
		next_eof <= 1;
		state <= MS_WAIT_GAP;
	end else if (state == MS_WAIT_GAP && next_data_wanted) begin
		next_eof <= 0;
		idle_wait <= 160; /* drain all data + interpacket gap */
		state <= MS_GAP;
	end else if (state == MS_GAP) begin
		if (idle_wait == 0) begin
			state <= MS_IDLE;
		end else begin
			idle_wait <= idle_wait - 1;
		end
	end
end

reg [3:0] crc_cnt = 0;
reg [31:0] crc;
reg [31:0] crc_data;
localparam POLY = 32'hedb88320;
always @(posedge rx_clk) begin
	if (init_crc) begin
		crc <= 32'hffffffff;
	end
	if (crc_data_ready) begin
		//crc_data <= next_data;
		crc_data <= { next_data[7:0], next_data[15:8], next_data[23:16], next_data[31:24] };
		crc_cnt <= 8;
	end
	if (crc_cnt) begin
		crc <= (crc >> 4) ^
		       ((crc[0] ^ crc_data[0]) ? (POLY >> 3) : 0) ^
		       ((crc[1] ^ crc_data[1]) ? (POLY >> 2) : 0) ^
		       ((crc[2] ^ crc_data[2]) ? (POLY >> 1) : 0) ^
		       ((crc[3] ^ crc_data[3]) ? POLY : 0) ;
		crc_data <= crc_data >> 4;
		crc_cnt <= crc_cnt - 1;
	end
end

localparam TX_IDLE	= 0;
localparam TX_RUNNING	= 1;
reg [3:0] tx_dicnt;
reg tx_state = TX_IDLE;
reg [31:0] tx_data;
reg tx_eof = 0;
reg tx_pre_eof = 0;
always @(posedge rx_clk) begin
	next_data_wanted <= 0;
	if (tx_state == TX_IDLE && tx_start) begin
		tx_data <= { next_data[7:0], next_data[15:8], next_data[23:16], next_data[31:24] };
		next_data_wanted <= 1;
		tx_dicnt <= 0;
		tx_state <= TX_RUNNING;
	end else if (tx_state == TX_RUNNING) begin
		tx_en <= 1;
		tx0 <= tx_data[0];
		tx1 <= tx_data[1];
		tx_data <= { 2'b00, tx_data[31:2] };
		tx_dicnt <= tx_dicnt + 1;
		if (tx_dicnt == 15) begin
			/* latch next data and swap endianess */
			tx_data <= { next_data[7:0], next_data[15:8], next_data[23:16], next_data[31:24] };
			/*
			 * delay the eof info by one data word, as we still
			 * need to push the newly received data out
			 */
			tx_pre_eof <= next_eof;
			tx_eof <= tx_pre_eof;
			next_data_wanted <= 1;
		end else if (tx_dicnt == 0) begin
			if (tx_eof) begin
				tx_en <= 0;
				tx0 <= 0;
				tx1 <= 0;
				tx_eof <= 0;
				tx_state <= TX_IDLE;
			end
		end
	end
end

assign debug[0] = tx_state;
assign debug[5:1] = state;
assign debug[6] = daqo_len_ready;
assign debug[6] = len_ready;
assign debug[13:8] = next_len;
assign debug[15:14] = enable;

endmodule
