`timescale 1ns / 1ps
`default_nettype none

module tmcuart #(
	parameter HZ = 0,
	parameter CMD_BITS = 0,
	parameter NUART = 0,
	parameter CMD_TMCUART_WRITE = 0,
	parameter CMD_TMCUART_READ = 0,
	parameter RSP_TMCUART_READ = 0
) (
	input wire clk,
	input wire [31:0] systime,

	input wire [31:0] arg_data,
	output wire arg_advance,
	input wire [CMD_BITS-1:0] cmd,
	input wire cmd_ready,
	output reg cmd_done = 0,

	output reg [32:0] param_data = 0,
	output reg param_write = 0,

	output reg invol_req = 0,
	input wire invol_grant,

	input wire [NUART-1:0] uart_in,
	output reg [NUART-1:0] uart_out,
	output reg [NUART-1:0] uart_en,

	input wire shutdown	/* not used */
);

/*
        "config_tmcuart oid=%c rx_pin=%u pull_up=%c tx_pin=%u bit_time=%u": 59,
        "tmcuart_send oid=%c write=%*s read=%c": 39,
        "tmcuart_response oid=%c read=%*s": 79,

	CMD_TMCUART_WRITE in <channel> <slave> <register> <data>
	CMD_TMCUART_READ in <channel> <slave> <register>
*/

/*
 * at 12 MHz, max rate is 750k. We go for a fixed 250k as on the host interfac3
 */
localparam BAUD = 250000;
localparam BITTIME = HZ / BAUD;
wire uart_rx;
wire uart_tx;
reg uart_tx_en = 0;
reg [7:0] uart_tx_data = 0;
wire uart_rx_ready;
wire [7:0] uart_rx_data;
wire uart_transmitting;
reg wire_en = 1;

uart #(
        .CLOCK_DIVIDE(BITTIME / 4)
) uart_u (
	.clk(clk),
	.rst(1'b0),
	.rx(uart_rx),
	.tx(uart_tx),
	.transmit(uart_tx_en),
	.tx_byte(uart_tx_data),
	.received(uart_rx_ready),
	.rx_byte(uart_rx_data),
	.is_receiving(),
	.is_transmitting(uart_transmitting),
	.recv_error()
);

always @(*) begin: uartmux
	integer i;

	for (i = 0; i < NUART; i++) begin
		if (i == channel) begin
			uart_out[i] = uart_tx;
			uart_en[i] = wire_en;
		end else begin
			uart_out[i] = 1'b1;
			uart_en[i] = 1'b1;
		end
	end
end
assign uart_rx = wire_en ? 1'b1 : uart_in[channel];

localparam NUART_BITS = $clog2(NUART);
reg [NUART_BITS-1:0] channel = NUART;	/* point to unpopulated channel when idle */

localparam PS_IDLE			= 0;
localparam PS_TMCUART_1			= 1;
localparam PS_TMCUART_2			= 2;
localparam PS_TMCUART_3			= 3;
localparam PS_TMCUART_4			= 4;
localparam PS_TMCUART_5			= 5;
localparam PS_TMCUART_6			= 6;
localparam PS_TMCUART_7			= 7;
localparam PS_TMCUART_8			= 8;
localparam PS_TMCUART_9			= 9;
localparam PS_TMCUART_10		= 10;
localparam PS_TMCUART_11		= 11;
localparam PS_TMCUART_12		= 12;
localparam PS_TMCUART_CRC		= 13;
localparam PS_TMCUART_RECV_1		= 14;
localparam PS_TMCUART_RECV_2		= 15;
localparam PS_TMCUART_RECV_3		= 16;
localparam PS_TMCUART_RECV_4		= 17;
localparam PS_TMCUART_RECV_5		= 18;
localparam PS_TMCUART_RECV_6		= 19;
localparam PS_TMCUART_RECV_7		= 20;
localparam PS_TMCUART_RECV_8		= 21;
localparam PS_TMCUART_RESPOND		= 22;
localparam PS_TMCUART_RESPOND_1		= 23;
localparam PS_TMCUART_RESPOND_2		= 24;
localparam PS_TMCUART_RESPOND_3		= 25;
localparam PS_TMCUART_DONE		= 26;
localparam PS_TMCUART_END		= 27;
localparam PS_TMCUART_RECV_ERROR	= 28;
localparam PS_TMCUART_WRITE_DONE	= 29;
localparam PS_MAX			= 29;
localparam PS_BITS = $clog2(PS_MAX + 1);
reg [PS_BITS-1:0] state = PS_IDLE;

localparam RE_OK		= 0;
localparam RE_TIMEOUT		= 1;
localparam RE_SYNC		= 2;
localparam RE_MASTER_ADDR	= 3;
localparam RE_REGISTER		= 4;
localparam RE_CRC		= 5;
localparam RE_MAX		= 5;
localparam RE_BITS = $clog2(RE_MAX + 1);
reg [RE_BITS-1:0] status = RE_OK;

/* just keep asserted, we'll read one arg per clock */
assign arg_advance = 1;

reg rdwr = 0;	/* read or write command */
reg [7:0] slave = 0;
reg [6:0] register = 0;
reg [31:0] data = 0;

localparam BTT_TIMEOUT = 700;
localparam BTT_BITS = $clog2(BITTIME * BTT_TIMEOUT);
reg [BTT_BITS-1:0] delay = 0;

reg [7:0] crc = 0;
reg [7:0] crc_in = 0;
reg crc_in_en = 0;
reg [2:0] crc_count = 0;
reg receiving = 0;

always @(posedge clk) begin
	if (cmd_done)
		cmd_done <= 0;
	uart_tx_en <= 0;
	crc_in_en <= 0;

	/* count above the state machine, so it can override the delay */
	if (delay)
		delay <= delay - 1;

	if (state == PS_IDLE && cmd_ready) begin
		// common to all cmds
		channel <= arg_data[NUART_BITS-1:0];
		if (cmd == CMD_TMCUART_WRITE) begin
			state <= PS_TMCUART_1;
			rdwr <= 0;
		end else if (cmd == CMD_TMCUART_READ) begin
			state <= PS_TMCUART_1;
			rdwr <= 1;
		end else begin
			cmd_done <= 1;
		end
	end else if (state == PS_TMCUART_1) begin
		/* CMD_TMCUART_SEND in <channel> <slave> <register> <data> */
		slave <= arg_data;
		state <= PS_TMCUART_2;
		/* turn line to output */
		wire_en <= 1;
	end else if (state == PS_TMCUART_2) begin
		register <= arg_data;
		crc <= 0;
		if (rdwr)
			state <= PS_TMCUART_4;
		else
			state <= PS_TMCUART_3;
	end else if (state == PS_TMCUART_3) begin
		data <= arg_data;
		state <= PS_TMCUART_4;
	end else if (state == PS_TMCUART_4) begin
		/* start transfer */
		uart_tx_data <= 8'b00000101;
		uart_tx_en <= 1;
		crc_in <= 8'b00000101;
		crc_in_en <= 1;
		state <= PS_TMCUART_5;
	end else if (state == PS_TMCUART_5 && !uart_tx_en && !uart_transmitting) begin
		uart_tx_data <= slave;
		uart_tx_en <= 1;
		crc_in <= slave;
		crc_in_en <= 1;
		state <= PS_TMCUART_6;
	end else if (state == PS_TMCUART_6 && !uart_tx_en && !uart_transmitting) begin
		uart_tx_data <= { !rdwr, register };
		uart_tx_en <= 1;
		crc_in <= { !rdwr, register };
		crc_in_en <= 1;
		if (rdwr)
			state <= PS_TMCUART_CRC;
		else
			state <= PS_TMCUART_7;
	end else if (state == PS_TMCUART_7 && !uart_tx_en && !uart_transmitting) begin
		/* write data */
		uart_tx_data <= data[31:24];
		uart_tx_en <= 1;
		crc_in <= data[31:24];
		crc_in_en <= 1;
		state <= PS_TMCUART_8;
	end else if (state == PS_TMCUART_8 && !uart_tx_en && !uart_transmitting) begin
		/* write data */
		uart_tx_data <= data[23:16];
		uart_tx_en <= 1;
		crc_in <= data[23:16];
		crc_in_en <= 1;
		state <= PS_TMCUART_9;
	end else if (state == PS_TMCUART_9 && !uart_tx_en && !uart_transmitting) begin
		/* write data */
		uart_tx_data <= data[15:8];
		uart_tx_en <= 1;
		crc_in <= data[15:8];
		crc_in_en <= 1;
		state <= PS_TMCUART_10;
	end else if (state == PS_TMCUART_10 && !uart_tx_en && !uart_transmitting) begin
		/* write data */
		uart_tx_data <= data[7:0];
		uart_tx_en <= 1;
		crc_in <= data[7:0];
		crc_in_en <= 1;
		state <= PS_TMCUART_CRC;
	end else if (state == PS_TMCUART_CRC && !uart_tx_en && !uart_transmitting) begin
		/* write data */
		uart_tx_data <= crc;
		uart_tx_en <= 1;
		if (rdwr)
			state <= PS_TMCUART_11;
		else
			state <= PS_TMCUART_WRITE_DONE;
	end else if (state == PS_TMCUART_WRITE_DONE && !uart_tx_en && !uart_transmitting) begin
		/* wait in this state until transmit finishes */
		state <= PS_TMCUART_DONE;
	end else if (state == PS_TMCUART_11 && !uart_tx_en && !uart_transmitting) begin
		/*
		 * send done. Wait 2 bit times before switching to receiving.
		 * The sender will wait 8 bit times
		 */
		delay <= BITTIME * 2;
		state <= PS_TMCUART_12;
	end else if (state == PS_TMCUART_12 && delay == 0) begin
		/* switch line to recv */
		wire_en <= 0;
		crc <= 0;
		/*
		 * receive must be finished within 640 + 6 bits. We wait
		 * 700.
		 */
		delay <= BTT_TIMEOUT * BITTIME;
		receiving <= 1;
		data <= 0;
		state <= PS_TMCUART_RECV_1;
	end else if (state == PS_TMCUART_RECV_1 && uart_rx_ready) begin
		if (uart_rx_data[3:0] != 4'b0101) begin
			status <= RE_SYNC;
			state <= PS_TMCUART_RECV_ERROR;
		end
		crc_in <= uart_rx_data;
		crc_in_en <= 1;
		state <= PS_TMCUART_RECV_2;
	end else if (state == PS_TMCUART_RECV_2 && uart_rx_ready) begin
		if (uart_rx_data != 8'hff) begin
			status <= RE_MASTER_ADDR;
			state <= PS_TMCUART_RECV_ERROR;
		end
		crc_in <= uart_rx_data;
		crc_in_en <= 1;
		state <= PS_TMCUART_RECV_3;
	end else if (state == PS_TMCUART_RECV_3 && uart_rx_ready) begin
		if (uart_rx_data != { 1'b0, register }) begin
			status <= RE_REGISTER;
			state <= PS_TMCUART_RECV_ERROR;
		end
		crc_in <= uart_rx_data;
		crc_in_en <= 1;
		state <= PS_TMCUART_RECV_4;
	end else if (state == PS_TMCUART_RECV_4 && uart_rx_ready) begin
		data[31:24] <= uart_rx_data;
		crc_in <= uart_rx_data;
		crc_in_en <= 1;
		state <= PS_TMCUART_RECV_5;
	end else if (state == PS_TMCUART_RECV_5 && uart_rx_ready) begin
		data[23:16] <= uart_rx_data;
		crc_in <= uart_rx_data;
		crc_in_en <= 1;
		state <= PS_TMCUART_RECV_6;
	end else if (state == PS_TMCUART_RECV_6 && uart_rx_ready) begin
		data[15:8] <= uart_rx_data;
		crc_in <= uart_rx_data;
		crc_in_en <= 1;
		state <= PS_TMCUART_RECV_7;
	end else if (state == PS_TMCUART_RECV_7 && uart_rx_ready) begin
		data[7:0] <= uart_rx_data;
		crc_in <= uart_rx_data;
		crc_in_en <= 1;
		state <= PS_TMCUART_RECV_8;
	end else if (state == PS_TMCUART_RECV_8 && uart_rx_ready) begin
		if (uart_rx_data != crc) begin
			status <= RE_CRC;
			state <= PS_TMCUART_RECV_ERROR;
		end
		status <= RE_OK;
		state <= PS_TMCUART_RESPOND;
	end else if (receiving == 1 && delay == 0) begin
		status <= RE_TIMEOUT;
		receiving <= 0;
		state <= PS_TMCUART_RESPOND;
	end else if (state == PS_TMCUART_RECV_ERROR && delay == 0) begin
		state <= PS_TMCUART_RESPOND;
	end else if (state == PS_TMCUART_RESPOND) begin
		param_data <= channel;
		param_write <= 1;
		state <= PS_TMCUART_RESPOND_1;
	end else if (state == PS_TMCUART_RESPOND_1) begin
		param_data <= status;
		param_write <= 1;
		state <= PS_TMCUART_RESPOND_2;
	end else if (state == PS_TMCUART_RESPOND_2) begin
		param_data <= data;
		param_write <= 1;
		state <= PS_TMCUART_RESPOND_3;
	end else if (state == PS_TMCUART_RESPOND_3) begin
		param_data <= RSP_TMCUART_READ;
		param_write <= 0;
		cmd_done <= 1;
		state <= PS_TMCUART_DONE;
	end else if (state == PS_TMCUART_DONE) begin
		/*
		 * delay 8 bit times for good measure. after a read, we
		 * need at least 4
		 */
		delay <= BITTIME * 8;
		state <= PS_TMCUART_END;
		receiving <= 0;
	end else if (state == PS_TMCUART_END && delay == 0) begin
		wire_en <= 1;
		channel <= NUART;
		cmd_done <= 1;
		state <= PS_IDLE;
	end

	/*
	 * CRC calculation. Assumption: receiving/transmitting a byte takes
	 * more than 10 cycles
	 */
	if (crc_in_en)
		crc_count <= 7;
	if (crc_count || crc_in_en) begin
		if (crc[7] ^ crc_in[0])
			crc <= { crc[6:0], 1'b0 } ^ 8'h07;
		else
			crc <= { crc[6:0], 1'b0 };
		crc_in <= { 1'b0, crc_in[7:1] };
		crc_count <= crc_count - 1;
	end
end

endmodule
