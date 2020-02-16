#include <stdlib.h>
#include "Vconan.h"
#include "verilated.h"

/*
 * UART send
 */
typedef struct {
	vluint8_t	*tx;
	int		divider;
	int		cnt;		/* clocks per bit */
	int		bit;		/* current bit */
	uint8_t		byte;		/* current byte */
	int		pos;		/* pos in buffer */
	int		len;		/* buffer len */
	uint8_t		*buf;
} uart_send_t;

static uart_send_t *
uart_send_init(vluint8_t *tx, int divider)
{
	uart_send_t *usp = (uart_send_t *)calloc(1, sizeof(*usp));

	usp->tx = tx;
	usp->divider = divider;

	*tx = 1;

	return usp;
}

static int
uart_send_tick(uart_send_t *usp, int *want_dump)
{
	if (usp->len == 0)
		return 1;

	if (--usp->cnt != 0)
		return 0;

	if (usp->bit == 0) {
		/* start bit */
		*usp->tx = 0;
		usp->byte = usp->buf[usp->pos];
	} else if (usp->bit < 9) {
		*usp->tx = usp->byte & 1;
		usp->byte >>= 1;
	} else {
		*usp->tx = 1;
	}
	if (++usp->bit == 10) {
		usp->bit = 0;
		++usp->pos;
		--usp->len;
	}
	usp->cnt = usp->divider;

	*want_dump = 1;

	return 0;
}

static void
uart_send(uart_send_t *usp, uint8_t *buf, int len)
{
	usp->cnt = 1;
	usp->bit = 0;
	usp->pos = 0;
	usp->len = len;
	usp->buf = buf;
}

/*
 * UART receive
 */
#define RXBUF	128
typedef struct {
	vluint8_t	*rx;
	int		divider;
	int		cnt;		/* clocks per bit */
	int		bit;		/* current bit */
	uint8_t		byte;		/* current byte */
	int		pos;		/* pos in buffer */
	uint8_t		buf[RXBUF];
} uart_recv_t;

static uart_recv_t *
uart_recv_init(vluint8_t *rx, int divider)
{
	uart_recv_t *urp = (uart_recv_t *)calloc(1, sizeof(*urp));

	urp->rx = rx;
	urp->divider = divider;

	return urp;
}

static int
uart_recv_tick(uart_recv_t *urp, int *want_dump)
{
	int b = *urp->rx;

	if (urp->bit == 0) { 	/* waiting for start bit */
		if (b == 1)
			return 0;
		urp->cnt = urp->divider + urp->divider / 2;
		urp->byte = 0;
		++urp->bit;
		*want_dump = 1;

		return 0;
	}
	if (--urp->cnt != 0)
		return 0;

	if (urp->bit < 9) { /* data bit */
		urp->byte >>= 1;
		urp->byte |= b << 7;
		++urp->bit;
		urp->cnt = urp->divider;
	} else if (urp->bit == 10) {
		if (b != 1) {
			printf("no stop bit received\n");
			exit(1);
		}
		urp->cnt = urp->divider / 4;
		++urp->bit;
	} else {
		urp->bit = 0;
		if (urp->pos == RXBUF) {
			printf("receive buffer overflow\n");
			exit(1);
		}
		urp->buf[urp->pos++] = urp->byte;
printf("received 0x%02x\n", urp->byte);
	}
		
	*want_dump = 1;
		
	return urp->bit == 0;
}

typedef struct {
		vluint8_t	fpga1;
		vluint8_t	fpga2;
		vluint8_t	uart_rx;
		vluint8_t	uart_tx;
		vluint8_t	uart_transmit;
		vluint8_t	uart_tx_byte;
		vluint8_t	uart_received;
		vluint8_t	uart_rx_byte;
		vluint8_t	uart_is_receiving;
		vluint8_t	uart_is_transmitting;
		vluint8_t	uart_rcv_error;
		vluint8_t	send_fifo_wr_en;
		vluint8_t	send_fifo_data;
		vluint8_t	send_fifo_full;
		vluint8_t	send_fifo_empty;
		vluint8_t	send_state;
} watchlist_t;
typedef struct {
	Vconan		*tb;
	uart_recv_t	*urp;
	uart_send_t	*usp;
	uint64_t	last_change;
	watchlist_t	w;
} sim_t;

/*
 * main tick loop
 */
static sim_t *
init(Vconan *tb)
{
	sim_t *sp = (sim_t *)calloc(1, sizeof(*sp));
	uint64_t d = 24000000 / 250000;	/* uart divider */

	sp->tb = tb;
	sp->urp = uart_recv_init(&tb->fpga2, d);
	sp->usp = uart_send_init(&tb->fpga1, d);
	sp->last_change = 0;

	return sp;
}

static void
step(sim_t *sp, uint64_t cycle)
{
	int ret;
	Vconan *tb = sp->tb;
	int want_dump = 0;

	uart_recv_tick(sp->urp, &want_dump);
	ret = uart_send_tick(sp->usp, &want_dump);

	if (cycle == 100) {
		uart_send(sp->usp, (uint8_t *)"test", 4);
	}

	watchlist_t old_w = sp->w;	/* make copy */
        sp->w.uart_received = tb->conan__DOT__u_framing__DOT__uart_u__DOT__received;
        sp->w.uart_transmit = tb->conan__DOT__u_framing__DOT__uart_u__DOT__transmit;
	sp->w.send_fifo_wr_en = tb->conan__DOT__u_framing__DOT__send_fifo_wr_en;
	sp->w.send_fifo_data = tb->conan__DOT__u_framing__DOT__send_fifo_data;
	sp->w.send_fifo_full = tb->conan__DOT__u_framing__DOT__send_fifo_full;
	sp->w.send_fifo_empty = tb->conan__DOT__u_framing__DOT__send_fifo_empty;
	sp->w.send_state = tb->conan__DOT__u_framing__DOT__send_state;
#if 0
	sp->w.fpga1 = tb->fpga1;
	sp->w.fpga2 = tb->fpga2;
        sp->w.uart_tx_byte = tb->conan__DOT__u_framing__DOT__uart_u__DOT__tx_byte;
        sp->w.uart_rx_byte = tb->conan__DOT__u_framing__DOT__uart_u__DOT__rx_byte;
        sp->w.uart_rx = tb->conan__DOT__u_framing__DOT__uart_u__DOT__rx;
        sp->w.uart_tx = tb->conan__DOT__u_framing__DOT__uart_u__DOT__tx;
        sp->w.uart_is_receiving = tb->conan__DOT__u_framing__DOT__uart_u__DOT__is_receiving;
        sp->w.uart_is_transmitting = tb->conan__DOT__u_framing__DOT__uart_u__DOT__is_transmitting;
        sp->w.uart_rcv_error = tb->conan__DOT__u_framing__DOT__uart_u__DOT__recv_error;
#endif
	int cmp = memcmp(&sp->w, &old_w, sizeof(old_w));

	if (cmp || (0 && want_dump)) {
		/* something has changed */
		printf("% 10d tx %d rx %d u_rx %d utx %d u_transmit %d u_tx_byte %d u_received %d u_rx_byte %d u_is_recieving %d u_is_transmitting %d u_rcv_error %d\n",
			cycle,
			tb->fpga1,
			tb->fpga2,
			tb->conan__DOT__u_framing__DOT__uart_u__DOT__rx,
			tb->conan__DOT__u_framing__DOT__uart_u__DOT__tx,
			tb->conan__DOT__u_framing__DOT__uart_u__DOT__transmit,
			tb->conan__DOT__u_framing__DOT__uart_u__DOT__tx_byte,
			tb->conan__DOT__u_framing__DOT__uart_u__DOT__received,
			tb->conan__DOT__u_framing__DOT__uart_u__DOT__rx_byte,
			tb->conan__DOT__u_framing__DOT__uart_u__DOT__is_receiving,
			tb->conan__DOT__u_framing__DOT__uart_u__DOT__is_transmitting,
			tb->conan__DOT__u_framing__DOT__uart_u__DOT__recv_error);
		printf("% 10d send_fifo: wr_en %d data %02x full %d empty %d state %d\n", cycle,
			tb->conan__DOT__u_framing__DOT__send_fifo_wr_en,
			tb->conan__DOT__u_framing__DOT__send_fifo_data,
			tb->conan__DOT__u_framing__DOT__send_fifo_full,
			tb->conan__DOT__u_framing__DOT__send_fifo_empty,
			tb->conan__DOT__u_framing__DOT__send_state);

		fflush(stdout);
		sp->last_change = cycle;
	}

	if (cycle - sp->last_change > 10000) {
		printf("% 10d finish due to inactivity\n", cycle);
		fflush(stdout);
		exit(0);
	}
}

int
main(int argc, char **argv) {
	// Initialize Verilators variables
	Verilated::commandArgs(argc, argv);
	uint64_t cycle = 0;
	sim_t *ctx;

	// Create an instance of our module under test
	Vconan *tb = new Vconan;

	ctx = init(tb);

	// Tick the clock until we are done
	while(!Verilated::gotFinish()) {
		tb->clk_48mhz = 1;
		tb->eval();
		tb->clk_48mhz = 0;
		tb->eval();
		++cycle;
		step(ctx, cycle);
	} exit(EXIT_SUCCESS);
}
