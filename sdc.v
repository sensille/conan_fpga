`timescale 1ns / 1ps
`default_nettype none

/*
 * SD Card controller base
 */
module sd #(
	parameter HZ = 0,
) (
	input wire clk,

	/* payload input queue */
	input wire [7:0] payload_data,
	input wire payload_valid,

	/* cmd input queue */
	input wire [8:0] cmd_data,
	input wire cmd_valid,
	outpu wire cmd_full,

	/* output queue */
	output wire [8:0] output_data,
	output wire output_empty,
	input wire output_advance,

	/* SD card signals */
	output reg [NSD-1:0] sd_clk,
	inout wire [NSD-1:0] sd_cmd,
	inout wire [NSD-1:0] sd_dat0,
	inout wire [NSD-1:0] sd_dat1,
	inout wire [NSD-1:0] sd_dat2,
	inout wire [NSD-1:0] sd_dat3,
);
/*
 * no cmd+data write in parallel
 */

/*
 * clk generation
 *
 * interface: clken, clkdiv
 */
reg clken = 0;
reg [11:0] clkdiv;

reg [11:0] clkcnt = 0;
always @(posedge clk) begin
	sd_clk_out <= 0;
	sd_clk_sample <= 0;
	if (clken == 0) begin
		/* disabled, do nothing */
	end else if (clk_cnt == 0) begin
		sd_clk <= !sd_clk;
		if (sd_clk == 0)
			sd_clk_out <= 1;
		else
			sd_clk_sample <= 1;
		clk_cnt <= clkdiv;
	end else begin
		clk_cnt <= clk_cnt - 1;
	end
		
end

/*
 * DAT reception
 *
 * interfaced by:
 * dat_recv_start_64 (in)
 * dat_recv_start_512 (in)
 * dat_buf_ready (out) (single cycle)
 * dat_status (out) 01 = 512 bit, 10 = 512 byte, 11 = timeout
 * dat_buf (out)
 */
localparam DAT_BUF_SIZE		= 512 + 2
localparam DAT_STATUS_64	= 1
localparam DAT_STATUS_512	= 2
localparam DAT_STATUS_TO	= 3
localparam DAT_STATUS_BITS	= 2
reg dat_recv_start_64 = 0;
reg dat_recv_start_512 = 0;
reg dat_buf_ready = 0;
reg [DAT_STATUS_BITS-1:0] dat_status;
reg [7:0] dat_buf [DAT_BUF_SIZE];
reg [TIMEOUT_BITS-1:0] dat_timeout;

localparam DS_IDLE		= 0
localparam DS_WAIT_FOR_START	= 1
localparam DS_RECV		= 2
localparam DS_MAX		= 2
localparam DS_STATE_BITS = $clogs2(DS_MAX + 1);
reg [DS_STATE_BITS-1:0] dat_state = DS_IDLE;
reg ds_recv_first;
always @(posedge clk) begin
	dat_buf_ready <= 0;
	dat_buf_timeout <= 0;
	if (ds_state == DS_IDLE && (dat_recv_start_64 | dat_recv_start_512)) begin
		if (dat_recv_start_64)
			ds_recv_cnt <= 64 + 8;	/ * incl. crc */
		else
			ds_recv_cnt <= 512 + 8;
		ds_state <= DS_WAIT_FOR_START;
		ds_timeout <= dat_timeout;
	end else if (ds_state == DS_WAIT_FOR_START && sd_clk_sample) begin
		if (sd_dat0 == 0) begin
			ds_state <= DS_RECV;
			ds_recv_first <= 1;
			ds_wptr <= 0;
		end else if (ds_timeout == 0) begin
			ds_state <= DS_IDLE;
			dat_buf_timeout <= 1;
		end
		ds_timeout <= ds_timeout - 1;
	end else if (ds_state == DS_RECV && sd_clk_sample && sd_dat0 == 0) begin
		ds_wdata <= { ds_rtmp[3:0], sd_dat3, sd_dat2, sd_dat1, sd_dat0 };
		if (ds_recv_cnt[0] == 0 && ds_recv_first == 0) begin
			dat_buf[ds_wptr] <= ds_wdata;
			ds_wptr <= ds_wptr + 1;
		end
		if (ds_recv_cnt == 0) begin
			dat_buf_ready <= 1;
			ds_state <= DS_IDLE;
		end
		ds_recv_cnt <= ds_recv_cnt - 1;
		ds_recv_first <= 0;
	end
end

/*
 * commands in queue
 *
 * start clk with clkdiv 1.1xxxxxxx
 * stop clk		 1.10000000
 * send cmd              1.0001xxxx
 *     xxxx = 0000 no response
 *     xxxx = 0001 48 bit response
 *     xxxx = 0010 136 bit response
 *     6 byte data following
 *     checksum is pre-calculated
 * send dat              1.00100000
 *     512 byte data following (no crc)
 *     send is in foreground
 * send payload          1.00110000
 *     512 byte data from payload queue (no crc)
 * recv dat              1.0100xxxx
 *     xxxx = 0001 512 bit
 *     xxxx = 0010 512 byte + crc
 *     receive is in background
 * notify                1.1001xxxx
 *     xxxx are echoed in notify
 * block for ready       1.10100000
 * 
 * responses in queue
 * recv response
 *     xxxx = 0001 48 bit
 *     xxxx = 0010 136 bit
 *     xxxx = 1111 timeout
 *     data following
 * recv dat
 *     xxxx = 0001 512 bit
 *     xxxx = 0010 512 byte + crc
 *     xxxx = 1111 timeout
 *     data following
 * notify
 *     xxxx echoed from cmd
 * set cmd timeout
 *     xxxx timeout (logarithmic)
 * set dat timeout
 *     xxxx timeout (logarithmic)
 *
 * as DAT receive is in background, it needs to be buffered
 * and copied to queue after receiption. during copy, next
 * data can already be received. Assumption is that it is 
 * faster to copy the data to the queue than to fill with new
 * data. So DAT reception is in a different process.
 */

/* queue */
localparam CMDQ_ADDR_BITS	11
wire [8:0] cmdq_dout;
wire cmdq_rd_en;
wire cmdq_empty;
wire [CMDQ_ADDR_BITS-1:0] cmdq_elemcnt;
fifo #(
	.DATA_WIDTH(9),
	.ADDR_WIDTH(CMDQ_ADDR_BITS)
) u_sd_cmd_fifo (
	.clk(clk),
	.clr(0),

	/* write side */
	.din(cmd_data),
	.wr_en(cmd_valid),
	.full(cmd_full),

	/* read side */
	.dout(cmdq_dout),
	.rd_en(cmdq_rd_en),
	.empty(cmdq_empty),

	/* status */
	.elemcnt(cmdq_elemcnt)
);

CQ_DILE
cq_state
always @(posedge clk) begin
	if (cq_state == CQ_IDLE && !cmdq_empty) begin
		case (cmdq_dout[7:4]) 
end

endmodule
