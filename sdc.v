`timescale 1ns / 1ps
`default_nettype none

/*
 * SD Card controller base
 */
module sdc #(
	parameter HZ = 0
) (
	input wire clk,

	/* payload input queue */
	input wire [7:0] payload_data,
	input wire payload_valid,

	/* cmd input queue */
	input wire [7:0] cmd_data,
	input wire cmd_valid,
	output wire cmd_full,

	/* output queue */
	output wire [7:0] output_data,
	output wire output_empty,
	input wire output_advance,

	/* SD card signals */
	output reg sd_clk,
	output reg sd_cmd_en = 0,
	output reg sd_cmd_r,
	input wire sd_cmd_in,
	output reg sd_dat_en = 0,
	output reg sd_dat0_r,
	output reg sd_dat1_r,
	output reg sd_dat2_r,
	output reg sd_dat3_r,
	input wire sd_dat0_in,
	input wire sd_dat1_in,
	input wire sd_dat2_in,
	input wire sd_dat3_in
);

/*
 * clk generation
 *
 * interface: clken, clkdiv
 */
reg clken = 0;
reg [11:0] clkdiv;
reg sd_clk_out = 0;
reg sd_clk_sample = 0;
reg [11:0] clk_cnt = 0;
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
localparam DAT_BUF_SIZE		= 512 + 8;
localparam DAT_STATUS_64	= 1;
localparam DAT_STATUS_512	= 2;
localparam DAT_STATUS_TO	= 3;
localparam DAT_STATUS_BITS	= 2;
localparam TIMEOUT_BITS		= 11;
localparam DAT_BUF_BITS		= $clog2(DAT_BUF_SIZE);
reg dat_recv_start_64 = 0;
reg dat_recv_start_512 = 0;
reg dat_buf_ready = 0;
reg [DAT_STATUS_BITS-1:0] dat_status;
reg [7:0] dat_buf [DAT_BUF_SIZE];
reg [TIMEOUT_BITS-1:0] dat_timeout;

localparam DS_IDLE		= 0;
localparam DS_WAIT_FOR_START	= 1;
localparam DS_RECV		= 2;
localparam DS_MAX		= 2;
localparam DS_STATE_BITS = $clog2(DS_MAX + 1);
reg [DS_STATE_BITS-1:0] ds_state = DS_IDLE;
reg ds_recv_first;
reg dat_buf_timeout = 0;
reg [9:0] ds_recv_cnt;
reg [TIMEOUT_BITS-1:0] ds_timeout;
reg [DAT_BUF_BITS-1:0] ds_wptr;
reg [7:0] ds_rtmp;
always @(posedge clk) begin
	dat_buf_ready <= 0;
	dat_buf_timeout <= 0;
	if (ds_state == DS_IDLE && (dat_recv_start_64 | dat_recv_start_512)) begin
		if (dat_recv_start_64)
			ds_recv_cnt <= 64 + 8;	/* incl. crc */
		else
			ds_recv_cnt <= 512 + 8;
		ds_state <= DS_WAIT_FOR_START;
		ds_timeout <= dat_timeout;
	end else if (ds_state == DS_WAIT_FOR_START && sd_clk_sample) begin
		if (sd_dat0_in == 0) begin
			ds_state <= DS_RECV;
			ds_recv_first <= 1;
			ds_wptr <= 0;
		end else if (ds_timeout == 0) begin
			ds_state <= DS_IDLE;
			dat_buf_timeout <= 1;
		end
		ds_timeout <= ds_timeout - 1;
	end else if (ds_state == DS_RECV && sd_clk_sample && sd_dat0_in == 0) begin
		ds_rtmp <= { ds_rtmp[3:0], sd_dat0_in, sd_dat1_in, sd_dat2_in, sd_dat3_in };
		if (ds_recv_cnt[0] == 0 && ds_recv_first == 0) begin
			dat_buf[ds_wptr] <= ds_rtmp;
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
 * set clkdiv            1000xxxx
 *     following byte    xxxxxxxx
 *     12'x clkdiv
 * start clk             10010000
 * stop clk              10100000
 * send cmd              0001xxxx
 *     xxxx = 0000 no response
 *     xxxx = 0001 48 bit response
 *     xxxx = 0010 136 bit response
 *     6 byte data following
 *     checksum is pre-calculated
 * send dat              00100000
 *     512 byte data following (no crc)
 *     send is in foreground
 * send payload          00110000
 *     512 byte data from payload queue (no crc)
 * recv dat              0100xxxx
 *     xxxx = 0001 512 bit
 *     xxxx = 0010 512 byte + crc
 *     receive is in background
 * notify                1001xxxx
 *     xxxx are echoed in notify
 * block for ready       10100000
 * 
 * responses in queue
 * recv response         0001xxxx
 *     xxxx = 0001 48 bit
 *     xxxx = 0010 136 bit
 *     xxxx = 1111 timeout
 *     data following
 * recv dat              0010xxxx
 *     xxxx = 0001 512 bit
 *     xxxx = 0010 512 byte + crc
 *     xxxx = 1111 timeout
 *     data following
 * notify                1001xxxx
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
localparam CMDQ_ADDR_BITS	= 11;
wire [7:0] cmdq_dout;
reg cmdq_rd_en = 0;
wire cmdq_empty;
wire [CMDQ_ADDR_BITS-1:0] cmdq_elemcnt;
fifo #(
	.DATA_WIDTH(8),
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

localparam CQ_IDLE		= 0;
localparam CQ_SET_CLKDIV_1	= 1;
localparam CQ_SET_CLKDIV_2	= 2;
localparam CQ_SEND_CMD_0	= 3;
localparam CQ_SEND_CMD_1	= 4;
localparam CQ_SEND_CMD_2	= 5;
localparam CQ_SEND_CMD_3	= 6;
localparam CQ_SEND_CMD_4	= 7;
localparam CQ_SEND_CMD_5	= 8;
localparam CQ_SEND_CMD_6	= 9;
localparam CQ_MAX		= 9;
localparam CQ_BITS = $clog2(CQ_MAX + 1);
reg [CQ_BITS-1:0] cq_state = CQ_IDLE;
reg [3:0] cq_xxxx; /* generic register to save lower half of cmd */
reg [2:0] cq_bit_cnt;
reg [9:0] cq_byte_cnt;
reg [7:0] co_data;
reg co_wr_en;
reg [7:0] cq_curr_byte;
reg [TIMEOUT_BITS-1:0] cq_timeout;
always @(posedge clk) begin
	cmdq_rd_en <= 0;
	co_wr_en <= 0;
	if (cq_state == CQ_IDLE && !cmdq_empty) begin
		cmdq_rd_en <= 1;
		cq_xxxx <= cmdq_dout[3:0];
		case (cmdq_dout[7:4])
		4'b0001: begin
			/*
			 * send cmd              0001xxxx
			 *     xxxx = 0000 no response
			 *     xxxx = 0001 48 bit response
			 *     xxxx = 0010 136 bit response
			 *     6 byte data following
			 *     checksum is pre-calculated
			 */
			cq_state <= CQ_SEND_CMD_0;
			cq_byte_cnt <= 5;
			sd_cmd_en <= 0;
		end
		4'b0010: begin
			/*
			 * send dat              00100000
			 *     512 byte data following (no crc)
			 *     send is in foreground
			 */
		end
		4'b0011: begin
			/*
			 * send payload          00110000
			 *     512 byte data from payload queue (no crc)
			 */
		end
		4'b0100: begin
			/*
			 * recv dat              0100xxxx
			 *     xxxx = 0001 512 bit
			 *     xxxx = 0010 512 byte + crc
			 *     receive is in background
			 */
		end
		4'b0101: begin
			/*
			 * block for ready       10100000
			 */
		end
		4'b0110: begin
			/*
			 * notify                1001xxxx
			 *     xxxx are echoed in notify
			 */
		end
		4'b1000: begin
			/*
			 * set clkdiv            1000xxxx
			 *     following byte    xxxxxxxx
			 *     12'x clkdiv
			 */
			clkdiv[11:8] <= cmdq_dout[3:0];
			cq_state <= CQ_SET_CLKDIV_1;
		end
		4'b1001: begin
			/*
			 * start clk             10010000
			 */
			clken <= 1;
		end
		4'b1010: begin
			/*
			 * stop clk              10100000
			 */
			clken <= 0;
		end
		default: begin
			/*
			 * can't recover from unknown command, so
			 * we just stall here
			 */
			cmdq_rd_en <= 0;
		end
		endcase
	end else if (cq_state == CQ_SET_CLKDIV_1) begin
		/* delay state for new data */
		cq_state <= CQ_SET_CLKDIV_2;
	end else if (cq_state == CQ_SET_CLKDIV_2) begin
		clkdiv[7:0] <= cmdq_dout;
		cq_state <= CQ_IDLE;
	end else if (cq_state == CQ_SEND_CMD_0) begin
		/* wait for all data to be available */
		if (cmdq_elemcnt >= 7) begin
			cmdq_rd_en <= 1;
			cq_state <= CQ_SEND_CMD_1;
		end
	end else if (cq_state == CQ_SEND_CMD_1) begin
		/* delay slot to get data */
		cq_state <= CQ_SEND_CMD_2;
	end else if (cq_state == CQ_SEND_CMD_2) begin
		/* store data */
		cq_curr_byte <= cmdq_dout;
		cq_state <= CQ_SEND_CMD_3;
		cq_bit_cnt <= 7;
		cmdq_rd_en <= 1;
	end else if (cq_state == CQ_SEND_CMD_3 && sd_clk_out == 1) begin
		/* send one bit */
		sd_cmd_r <= cq_curr_byte[7];
		cq_curr_byte <= { cq_curr_byte[6:0], 1'b0 };
		if (cq_bit_cnt != 0) begin
			cq_bit_cnt <= cq_bit_cnt - 1;
		end else if (cq_byte_cnt == 0) begin
			cq_state <= CQ_SEND_CMD_4;
		end else begin
			cq_byte_cnt <= cq_byte_cnt - 1;
			cq_state <= CQ_SEND_CMD_2;
		end
	end else if (cq_state == CQ_SEND_CMD_4) begin
		/* send finished */
		sd_cmd_en <= 0;
		if (cq_xxxx == 4'b0001) begin
			cq_byte_cnt <= 5;
			cq_timeout <= 100; /* XXX */
			cq_state <= CQ_SEND_CMD_5;
		end else if (cq_xxxx == 4'b0010) begin
			cq_byte_cnt <= 16;
			cq_timeout <= 100; /* XXX */
			cq_state <= CQ_SEND_CMD_5;
		end else begin
			/* no response */
			cq_state <= CQ_IDLE;
			cmdq_rd_en <= 1;
		end
	end else if (cq_state == CQ_SEND_CMD_5 && sd_clk_sample) begin
		/* wait for start bit */
		if (cq_timeout == 0) begin
			co_data <= 8'b00011111;
			co_wr_en <= 1;
			cq_state <= CQ_IDLE;
		end else if (sd_cmd_in == 0) begin
			cq_bit_cnt <= 6;
			cq_curr_byte[0] <= 0;
			co_data <= { 4'b0001, cq_xxxx };
			co_wr_en <= 1;
			cq_state <= CQ_SEND_CMD_6;
		end else begin
			cq_timeout <= cq_timeout - 1;
		end
	end else if (cq_state == CQ_SEND_CMD_6 && sd_clk_sample) begin
		/* receive bit */
		cq_curr_byte <= { cq_curr_byte[6:0], sd_cmd_in };
		if (cq_bit_cnt != 0) begin
			cq_bit_cnt <= cq_bit_cnt - 1;
		end else begin
			cq_bit_cnt <= 7;
			co_data <= cq_curr_byte;
			co_wr_en <= 1;
			cq_byte_cnt <= cq_byte_cnt - 1;
			if (cq_byte_cnt == 0) begin
				cq_state <= CQ_IDLE;
			end
		end
	end
end

endmodule
