#include <stdlib.h>
#include <stddef.h>
#include <setjmp.h>
#include <pcre.h>
#include "Vconan.h"
#include "verilated.h"
#include "vsyms.h"

#define CMD_GET_VERSION	0
#define CMD_CONFIG_PWM	5

#define RSP_GET_VERSION	2

#define WF_WATCH	1
#define WF_PRINT	2
#define WF_ALL		(WF_WATCH | WF_PRINT)
 
#define FORM_BIN	1
#define FORM_HEX	2
#define FORM_DEC	3

#define COLOR_NONE	1
#define COLOR_CHANGED	2
#define COLOR_ALWAYS	3

typedef struct _watch_entry {
	int		sig;	/* signal number */
	char		*nick;
	int		flags;
	void		*val;
	uint64_t	*mask;
	int		format;
	int		cmp;
} watch_entry_t;

typedef struct _watch {
	watch_entry_t	*we;
	int		n;
	Vconan		*tb;
} watch_t;

watch_t *
watch_init(Vconan *tb)
{
	watch_t *wp = (watch_t *)calloc(sizeof(*wp), 1);
	wp->we = (watch_entry_t *)calloc(sizeof(*wp->we), NSIGS * 10);
	wp->tb = tb;

	return wp;
}

static int
sig_to_len(int sig)
{
	int len;

	switch (vsigs[sig].type) {
	case sigC: len = 1; break;
	case sigS: len = 2; break;
	case sigI: len = 4; break;
	case sigQ: len = 8; break;
	case sigW: len = 4; break;
	default:
		printf("inval sig type\n");
		exit(1);
	}

	return len * vsigs[sig].num;
}

static void *
watch_get_value(watch_t *wp, int sig)
{
	int len = sig_to_len(sig);
	Vconan *tb = wp->tb;
	void *v = malloc(len);

	memcpy(v, (uint8_t *)tb + vsigs[sig].offset, len);

	return v;
}

static int
watch_cmp_value(int sig, void *v1, void *v2, uint64_t *mask)
{
	int len = sig_to_len(sig);

	return memcmp(v1, v2, len);
}

static int
find_signal(const char *name, int *start)
{
	int i;
	const char *err_str;
	int err_off;
	int ret;
	pcre *comp = pcre_compile(name, 0, &err_str, &err_off, NULL);

	if (comp == NULL) {
		printf("name %s not a valid regexp: %s\n", name, err_str);
		exit(1);
	}

	for (i = *start; i < NSIGS; ++i) {
		ret = pcre_exec(comp, NULL, vsigs[i].name,
			strlen(vsigs[i].name), 0, 0, NULL, 0);
		if (ret == PCRE_ERROR_NOMATCH)
			continue;
		if (ret < 0) {
			printf("pcre match failed with %d\n", ret);
			exit(1);
		}
		*start = i + 1;

		return i;
	}

	return -1;
}

static void
watch_add(watch_t *wp, const char *name, const char *nick, uint64_t *mask,
	int format, int flags)
{
	int _r = 0;
	int at_least_one = 0;
	int sig;

	while ((sig = find_signal(name, &_r)) >= 0) {
		printf("watch: adding %s as %s\n", vsigs[sig].name, nick);
		wp->we[wp->n].sig = sig;
		wp->we[wp->n].nick = strdup(nick);
		wp->we[wp->n].flags = flags;
		wp->we[wp->n].val = watch_get_value(wp, sig);
		/* TODO: save mask, format */
		++wp->n;
		if (wp->n == NSIGS * 10) {
			printf("too many signals in watchlist\n");
			exit(1);
		}
		at_least_one = 1;
	}
	if (!at_least_one) {
		printf("name %s doesn't match any signal\n", name);
		exit(0);
	}
}

static void
watch_remove(watch_t *wp, const char *name)
{
	int _r = 0;
	int sig;
	int i;

	while ((sig = find_signal(name, &_r)) >= 0) {
		for (i = 0; i < wp->n; ++i) {
			if (wp->we[i].sig == sig) {
				free(wp->we[i].nick);
				free(wp->we[i].val);
				memmove(wp->we + i, wp->we + i + 1, sizeof(*wp->we) * (wp->n - i));
				--wp->n;
				break;
			}
		}
	}
}

static void
watch_clear(watch_t *wp)
{
	int i;

	for (i = 0; i < wp->n; ++i) {
		free(wp->we[i].nick);
		free(wp->we[i].val);
	}
	wp->n = 0;
}

#define C_BLACK "\e[30m"
#define C_RED "\e[31m"
#define C_GREEN "\e[32m"
#define C_YELLOW "\e[33m"
#define C_BLUE "\e[34m"
#define C_MAGENTA "\e[35m"
#define C_CYAN "\e[36m"
#define C_WHITE "\e[37m"
#define C_RESET "\e[0m"
static void
print_value(watch_entry_t *we, int do_color)
{
	int len = sig_to_len(we->sig);
	const char *c = NULL;
	char outbuf[1000];
	uint64_t v;

	if (do_color == COLOR_CHANGED && we->cmp)
		c = C_RED;
	else if (do_color == COLOR_ALWAYS)
		c = C_GREEN;
	printf(" %s %s", we->nick, c ? c : "");

	printf("%x", *(uint8_t *)we->val);

	if (c)
		printf("%s", C_RESET);
}

static void
do_watch(watch_t *wp, uint64_t cycle)
{
	int i;
	int same = 1;
	int header_done;
	void *v;

	/*
	 * compare signals
	 */
	for (i = 0; i < wp->n; ++i) {
		watch_entry_t *we = wp->we + i;
		int ret = 0;
		void *v = NULL;

		v = watch_get_value(wp, we->sig);
		ret = watch_cmp_value(we->sig, we->val, v, we->mask);
		if (ret != 0)
			same = 0;
		we->cmp = ret;
		free(we->val);
		we->val = v;
	}

	if (same)
		return;

	/* print all fixed values */
	header_done = 0;
	for (i = 0; i < wp->n; ++i) {
		watch_entry_t *we = wp->we + i;
		int flags = we->flags;

		if (flags & WF_PRINT) {
			if (!header_done) {
				printf("% 10d", cycle);
				header_done = 1;
			}
			print_value(we, COLOR_CHANGED);
		}
	}
	if (header_done)
		printf("\n");

	header_done = 0;
	/* print watch-only first */
	for (i = 0; i < wp->n; ++i) {
		watch_entry_t *we = wp->we + i;
		int flags = we->flags;

		if (flags & WF_WATCH && we->cmp && ~flags & WF_PRINT) {
			if (!header_done) {
				printf("% 10d changed", cycle);
				header_done = 1;
			}
			print_value(we, COLOR_NONE);
		}
	}
	if (header_done)
		printf("\n");
}

uint16_t
crc16_ccitt(uint8_t *buf, uint_fast8_t len)
{
	uint16_t crc = 0xffff;
	while (len--) {
		uint8_t data = *buf++;
		data ^= crc & 0xff;
		data ^= data << 4;
		crc = ((((uint16_t)data << 8) | (crc >> 8)) ^ (uint8_t)(data >> 4)
		       ^ ((uint16_t)data << 3));
	}
	return crc;
}

// Encode an integer as a variable length quantity (vlq)
static uint8_t *
encode_int(uint8_t *p, uint32_t v)
{
	int32_t sv = v;

	if (sv < (3L<<5)  && sv >= -(1L<<5))
		goto f4;
	if (sv < (3L<<12) && sv >= -(1L<<12))
		goto f3;
	if (sv < (3L<<19) && sv >= -(1L<<19))
		goto f2;
	if (sv < (3L<<26) && sv >= -(1L<<26))
		goto f1;
	*p++ = (v>>28) | 0x80;
f1:	*p++ = ((v>>21) & 0x7f) | 0x80;
f2:	*p++ = ((v>>14) & 0x7f) | 0x80;
f3:	*p++ = ((v>>7) & 0x7f) | 0x80;
f4:	*p++ = v & 0x7f;

	return p;
}

#define MAXPACKET	128

// Parse an integer that was encoded as a "variable length quantity"
static int
parse_int(const uint8_t *buf, int pos, int len, uint32_t *val)
{
	int last = pos + len;
	uint8_t c;
	uint32_t v;

	if (pos >= last)
		goto err;

	c = buf[pos++];
	v = c & 0x7f;

	if ((c & 0x60) == 0x60)
		v |= -0x20;
	while (c & 0x80) {
		if (pos >= last)
			goto err;
		c = buf[pos++];
		v = (v<<7) | (c & 0x7f);
	}

	*val = v;

	return pos - (last - len);

err:
	printf("parse_int failed, buffer too short\n");
	exit(1);
}

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
	int		seq;		/* next seq to send */
	uint8_t		buf[MAXPACKET];
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

static int
uart_send_done(uart_send_t *usp)
{
	return usp->len == 0;
}

static void
uart_send(uart_send_t *usp, uint8_t *buf, int len)
{
	usp->cnt = 1;
	usp->bit = 0;
	usp->pos = 0;
	usp->len = len;
	memcpy(usp->buf, buf, len);

	printf("uart_send:");
	for (int i = 0; i < len; ++i)
		printf(" %02x", buf[i]);
	printf("\n");
}

static void
uart_send_packet(uart_send_t *usp, uint8_t *data, int len)
{
	uint16_t crc;
	uint8_t packet[len + 5];

	packet[0] = len + 5;
	packet[1] = 0x10 | usp->seq;
	usp->seq = (usp->seq + 1) & 0x0f;
	memcpy(packet + 2, data, len);
	crc = crc16_ccitt(packet, len + 2);
	packet[len + 2] = crc >> 8;
	packet[len + 3] = crc & 0xff;
	packet[len + 4] = 0x7e;
	uart_send(usp, packet, len + 5);
}

static void
uart_send_vlq(uart_send_t *usp, uint32_t *args, int num)
{
	uint8_t buf[num * 5];
	uint8_t *p = buf;
	int i;

	for (i = 0; i < num; ++i)
		p = encode_int(p, args[i]);

	uart_send_packet(usp, buf, p - buf);
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
	uint8_t		buf[MAXPACKET];
	/* packet level */
	int		expected_seq;
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

static int
uart_frame_done(uart_recv_t *urp)
{
	int len;

	if (urp->pos < 5)
		return 0;

	len = urp->buf[0];

	if (urp->pos < len)
		return 0;

	if (urp->pos > len) {
		printf("frame longer than header indicates\n");
		exit(1);
	}
	if (urp->buf[len - 1] != 0x7e) {
		printf("frame not properly terminated\n");
		exit(1);
	}
	uint16_t crc = crc16_ccitt(urp->buf, len - 3);
	if ((urp->buf[len - 3] != (crc >> 8)) ||
	    (urp->buf[len - 2] != (crc & 0xff))) {
		printf("received bad crc\n");
		exit(1);
	}
	if (urp->buf[1] != (0x10 + urp->expected_seq)) {
		printf("received bad seq\n");
		exit(1);
	}

	return 1;
}

typedef struct {
	Vconan		*tb;
	uart_recv_t	*urp;
	uart_send_t	*usp;
	uint64_t	last_change;
	watch_t		*wp;
	uint64_t	cycle;
	jmp_buf		main_jb;
	jmp_buf		test_jb;
	vluint8_t	*signal8p;
	vluint8_t	signal8v;
	uint64_t	delay_until;
} sim_t;

/*
 * main tick loop
 */
static void test(sim_t *sp);
static sim_t *
init(Vconan *tb)
{
	sim_t *sp = (sim_t *)calloc(1, sizeof(*sp));
	uint64_t d = 24000000 / 250000;	/* uart divider */

	sp->tb = tb;
	sp->urp = uart_recv_init(&tb->fpga2, d);
	sp->usp = uart_send_init(&tb->fpga1, d);
	sp->last_change = 0;
	sp->cycle = 0;

	sp->wp = watch_init(tb);

	return sp;
}

static void
yield(sim_t *sp)
{
	if (setjmp(sp->test_jb) == 0)
		longjmp(sp->main_jb, 1);
}

static void
step(sim_t *sp, uint64_t cycle)
{
	int ret;
	Vconan *tb = sp->tb;
	int want_dump = 0;

	uart_recv_tick(sp->urp, &want_dump);
	ret = uart_send_tick(sp->usp, &want_dump);

	/* continue test procedure */
	sp->cycle = cycle;
	ret = setjmp(sp->main_jb);
	if (ret == 0)
		longjmp(sp->test_jb, 1);
	if (ret == 1)
		want_dump = 1;

	do_watch(sp->wp, cycle);
	fflush(stdout);
}

static void
delay(sim_t *sp, uint64_t ticks)
{
	sp->delay_until = sp->cycle + ticks;

	while (sp->cycle < sp->delay_until)
		yield(sp);
}

static void
wait_for_uart_send(sim_t *sp)
{
	while (!uart_send_done(sp->usp))
		yield(sp);
}

static void
wait_for_uart_recv(sim_t *sp)
{
	while (!uart_frame_done(sp->urp))
		yield(sp);
}

static void
wait_for_uart_vlq(sim_t *sp, int n, uint32_t *vlq)
{
	int ret;
	int i;
	int pos = 2;
	int len;

	wait_for_uart_recv(sp);
	len = sp->urp->pos - 5;
	for (i = 0; i < n; ++i) {
		ret = parse_int(sp->urp->buf, pos, len, vlq + i);
		pos += ret;
		len -= ret;
	}
	if (len != 0) {
		printf("parsing recv buffer has leftover\n");
		exit(1);
	}
	/* clear recv buffer */
	sp->urp->pos = 0;
	printf("received {");
	for (i = 0; i < n; ++i)
		printf(" %d", vlq[i]);
	printf(" }\n");
}

static void
wait_for_signal8(sim_t *sp, vluint8_t *signal, vluint8_t val)
{
	sp->signal8p = signal; 
	sp->signal8v = val; 
	while (*sp->signal8p != sp->signal8v)
		yield(sp);
}

static void
fail(const char *msg, ...)
{
	va_list ap;
	va_start(ap, msg);
	printf("test failed: ");
	vprintf(msg, ap);
	exit(1);
}

static void
test_version(sim_t *sp)
{
	uart_send_t *usp = sp->usp;
	uart_recv_t *urp = sp->urp;
	Vconan *tb = sp->tb;
	int i;
	uint64_t curr;
	watch_t *wp = sp->wp;

	watch_add(wp, "u_framing.tx$", "tx", NULL, FORM_BIN, WF_ALL);
	watch_add(wp, "u_framing.rx$", "rx", NULL, FORM_BIN, WF_ALL);
	watch_add(wp, "u_system.cmd_ready", "rdy", NULL, FORM_BIN, WF_ALL);
	watch_add(wp, "u_command.unit$", "unit", NULL, FORM_BIN, WF_ALL);
	watch_add(wp, "u_command.msg_state", "cstate", NULL, FORM_BIN, WF_ALL);

	
	/* send version request */
	uint32_t cmd[] = { 1, CMD_GET_VERSION };
	uint32_t rsp[2];
	uart_send_vlq(usp, cmd, sizeof(cmd) / sizeof(*cmd));
	wait_for_uart_vlq(sp, 2, rsp);

	if (rsp[0] != 2 || rsp[1] != 0x42) {
		printf("received incorrect version rsp\n");
		exit(1);
	}
}

static void
test_pwm(sim_t *sp)
{
	uart_send_t *usp = sp->usp;
	uart_recv_t *urp = sp->urp;
	Vconan *tb = sp->tb;
	int i;
	uint64_t curr;

	watch_add(sp->wp, "pwm1$", "p1", NULL, FORM_BIN, WF_ALL);

	/* CONFIGURE_PWM, channel,  cycle_ticks, on_ticks, default_value, max_duration */
	uint32_t cmd[] = { CMD_CONFIG_PWM, 0, 1000, 100, 0, 5000 };
	uart_send_vlq(usp, cmd, sizeof(cmd) / sizeof(*cmd));
	wait_for_uart_send(sp);
	delay(sp, 100);
	wait_for_signal8(sp, &tb->conan__DOT__pwm1, 0);
	wait_for_signal8(sp, &tb->conan__DOT__pwm1, 1);
	for (i = 0; i < 3; ++i) {
		curr = sp->cycle;
		wait_for_signal8(sp, &tb->conan__DOT__pwm1, 0);
		if (sp->cycle - curr != 900)
			fail("pwm 0 signal period mismatch, expected 900, got %d\n", sp->cycle - curr);
		wait_for_signal8(sp, &tb->conan__DOT__pwm1, 1);
		if (sp->cycle - curr != 1000)
			fail("pwm period mismatch, expected 1000, got %d\n", sp->cycle - curr);
	}
}

static void
test(sim_t *sp)
{
	delay(sp, 1);	/* pass back control after initialization */

	test_version(sp);

	/* TODO always sync time */
	test_pwm(sp);

	printf("test succeeded after %d cycles\n", sp->cycle);

	exit(0);
}

int
main(int argc, char **argv) {
	// Initialize Verilators variables
	Verilated::commandArgs(argc, argv);
	uint64_t cycle = 0;
	sim_t *sp;

	// Create an instance of our module under test
	Vconan *tb = new Vconan;

	sp = init(tb);

	if (setjmp(sp->main_jb) == 0)
		test(sp);	/* initialize test procedure */
	/*
	 * hack: this alloc reserves 64k of stack for test().
	 * it prevents the region where the stack frame from test
	 * resides from being overwritten
	 */
	alloca(65536);

	// Tick the clock until we are done
	while(!Verilated::gotFinish()) {
		tb->clk_48mhz = 1;
		tb->eval();
		tb->clk_48mhz = 0;
		tb->eval();
		++cycle;
		step(sp, cycle);
	}
	exit(0);
}
