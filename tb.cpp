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
#if 0
printf("divider %d cnt %d bit %d byte %d pos %d len %d\n",
	usp->divider, usp->cnt, usp->bit, usp->byte, usp->pos, usp->len);
#endif

	if (usp->len == 0)
		return 1;

	if (--usp->cnt != 0)
		return 0;

	if (usp->bit == 0) {
		/* start bit */
		*usp->tx = 0;
		usp->byte = usp->buf[usp->pos];
	} else if (usp->bit < 9) {
		*usp->tx = (usp->byte >> 7) & 1;
		usp->byte <<= 1;
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
uart_recv_tick(uart_recv_t *usp, int *want_dump)
{
	return 0;
}

typedef struct {
		vluint8_t	fpga1;
		vluint8_t	fpga2;
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
	sp->w.fpga1 = tb->fpga1;
	sp->w.fpga2 = tb->fpga2;
	int cmp = memcmp(&sp->w, &old_w, sizeof(old_w));
	if (cmp || want_dump) {
		/* something has changed */
		printf("% 10d tx %d rx %d\n",
			cycle, tb->fpga1, tb->fpga2);
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
