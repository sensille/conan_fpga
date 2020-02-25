#include <stdlib.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <pcre.h>
#include "Vconan.h"
#include "verilated.h"
#include "vsyms.h"

#define CMD_GET_VERSION		0
#define CMD_SYNC_TIME		1
#define CMD_GET_TIME		2
#define CMD_CONFIG_PWM		3
#define CMD_SCHEDULE_PWM	4
#define CMD_CONFIG_STEPPER	5
#define CMD_QUEUE_STEP		6
#define CMD_SET_NEXT_STEP_DIR	7
#define CMD_RESET_STEP_CLOCK	8
#define CMD_STEPPER_GET_POS	9
#define CMD_ENDSTOP_SET_STEPPER	10
#define CMD_ENDSTOP_QUERY	11
#define CMD_ENDSTOP_HOME	12
#define CMD_TMCUART_WRITE	13
#define CMD_TMCUART_READ	14

#define RSP_GET_VERSION		0
#define RSP_GET_TIME		1
#define RSP_STEPPER_GET_POS	2
#define RSP_ENDSTOP_STATE	3
#define RSP_TMCUART_READ	4

#define HZ 48000000
#define NUART 6

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
	uint64_t	last_cycle;
} watch_t;

#define MAXPACKET	128

typedef struct {
	vluint8_t	*tx;
	int		divider;
	int		cnt;		/* clocks per bit */
	int		bit;		/* current bit */
	uint8_t		byte;		/* current byte */
	int		pos;		/* pos in buffer */
	int		len;		/* buffer len */
	int		seq;		/* next seq to send */
	const char	*name;
	uint8_t		buf[MAXPACKET];
} uart_send_t;

#define RXBUF	128
typedef struct {
	vluint8_t	*rx;
	int		divider;
	int		cnt;		/* clocks per bit */
	int		bit;		/* current bit */
	uint8_t		byte;		/* current byte */
	int		pos;		/* pos in buffer */
	const char	*name;
	uint8_t		buf[MAXPACKET];
	/* packet level */
	int		expected_seq;
} uart_recv_t;

typedef struct {
	uart_send_t *usp;
	uart_recv_t *urp;

	uint8_t uart_in;
	uint8_t uart_out;

	int state;
	int delay;
	int last_pos;
	uint64_t last_change;

	vluint8_t *line_in;
	vluint8_t *line_out;

	uint32_t regs[128];
} tmcuart_t;
#define TU_READ		1
#define TU_WRITE	2
#define TU_IGNORE	3
#define TU_TURNAROUND	4
#define TU_TURNBACK	5

#define IOIN	2
#define IFCNT	1

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
	tmcuart_t	*tmcuart[NUART];
} sim_t;

static void tmcuart_tick(sim_t *sp);
static void wait_for_uart_send(sim_t *sp);
static void fail(const char *msg, ...);

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
		if (nick)
			printf("watch: adding %s as %s\n", vsigs[sig].name, nick);
		else
			printf("watch: adding %s as <unnamed>\n", vsigs[sig].name);
		wp->we[wp->n].sig = sig;
		wp->we[wp->n].nick = nick ? strdup(nick) : NULL;
		wp->we[wp->n].flags = flags;
		wp->we[wp->n].format = format;
		wp->we[wp->n].val = watch_get_value(wp, sig);
		/* TODO: save mask */
		++wp->n;
		if (wp->n == NSIGS * 10) {
			printf("too many signals in watchlist\n");
			exit(1);
		}
		at_least_one = 1;
		nick = NULL;
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
	int type = vsigs[we->sig].type;

	if (do_color == COLOR_CHANGED && we->cmp)
		c = C_RED;
	else if (do_color == COLOR_ALWAYS)
		c = C_GREEN;
	if (we->nick)
		printf(" %s %s", we->nick, c ? c : "");
	else
		printf(" %s", c ? c : "");

	for (int i = 0; i < vsigs[we->sig].num; ++i) {
		uint64_t val;

		if (i > 0)
			printf("/");
		if (type == sigC)
			val = ((uint8_t *)we->val)[i];
		else if (type == sigS)
			val = ((uint16_t *)we->val)[i];
		else if (type == sigI)
			val = ((uint32_t *)we->val)[i];
		else if (type == sigQ)
			val = ((uint64_t *)we->val)[i];
		else if (type == sigW)
			val = ((uint32_t *)we->val)[i];

		if (we->format == FORM_HEX) {
			printf("%lx", val);
		} else if (we->format == FORM_DEC) {
			printf("%ld", val);
		} else if (we->format == FORM_BIN) {
			int first = 0;
			int i;

			for (i = 63; i >= 0; --i) {
				int bit = (val & (1ul << 63)) != 0;

				val <<= 1;

				if (bit == 0 && !first)
					continue;
				first = 1;
				printf("%d", bit);
			}
			if (!first)
				printf("0");
		} else {
			printf("invalid format %d\n", we->format);
			exit(1);
		}
	}

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

	if (cycle - wp->last_cycle > 1000000)
		fail("abort due to inactivity\n");

	/*
	 * compare signals
	 */
	for (i = 0; i < wp->n; ++i) {
		watch_entry_t *we = wp->we + i;
		int ret = 0;
		void *v = NULL;

		v = watch_get_value(wp, we->sig);
		ret = watch_cmp_value(we->sig, we->val, v, we->mask);
		if (ret != 0 && (we->flags & WF_WATCH))
			same = 0;
		we->cmp = ret;
		free(we->val);
		we->val = v;
	}

	if (same)
		return;

	uint64_t diff = cycle - wp->last_cycle;

	/* print all fixed values */
	header_done = 0;
	for (i = 0; i < wp->n; ++i) {
		watch_entry_t *we = wp->we + i;
		int flags = we->flags;

		if (flags & WF_PRINT) {
			if (!header_done) {
				printf("% 10d % 6d", cycle, diff);
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
				printf("% 10d % 6d changed", cycle, diff);
				header_done = 1;
			}
			print_value(we, COLOR_NONE);
		}
	}
	if (header_done)
		printf("\n");

	wp->last_cycle = cycle;
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
static uart_send_t *
uart_send_init(vluint8_t *tx, int divider, const char *name)
{
	uart_send_t *usp = (uart_send_t *)calloc(1, sizeof(*usp));

	usp->tx = tx;
	usp->divider = divider;
	usp->name = name;

	*tx = 1;

	return usp;
}

static void
uart_send_free(uart_send_t *usp)
{
	free(usp);
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
	if (++usp->bit == 11) {
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

	printf("uart_send (%s):", usp->name);
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
uart_send_vlq(sim_t *sp, int num, ...)
{
	va_list ap;
	va_start(ap, num);

	uint8_t buf[num * 5];
	uint8_t *p = buf;
	int i;

	for (i = 0; i < num; ++i)
		p = encode_int(p, va_arg(ap, uint32_t));

	uart_send_packet(sp->usp, buf, p - buf);
}

static void
uart_send_vlq_and_wait(sim_t *sp, int num, ...)
{
	va_list ap;
	va_start(ap, num);

	uint8_t buf[num * 5];
	uint8_t *p = buf;
	int i;

	for (i = 0; i < num; ++i)
		p = encode_int(p, va_arg(ap, uint32_t));

	uart_send_packet(sp->usp, buf, p - buf);

	wait_for_uart_send(sp);
}

/*
 * UART receive
 */
static uart_recv_t *
uart_recv_init(vluint8_t *rx, int divider, const char *name)
{
	uart_recv_t *urp = (uart_recv_t *)calloc(1, sizeof(*urp));

	urp->rx = rx;
	urp->divider = divider;
	urp->name = name;

	return urp;
}

static void
uart_recv_free(uart_recv_t *urp)
{
	free(urp);
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
printf("received (%s) 0x%02x\n", urp->name, urp->byte);
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

/*
 * main tick loop
 */
static void test(sim_t *sp);
static sim_t *
init(Vconan *tb)
{
	sim_t *sp = (sim_t *)calloc(1, sizeof(*sp));
	uint64_t d = HZ / 250000;	/* uart divider */

	sp->tb = tb;
	sp->urp = uart_recv_init(&tb->fpga2, d, "conan");
	sp->usp = uart_send_init(&tb->fpga1, d, "conan");
	sp->last_change = 0;
	sp->cycle = 0;

	sp->wp = watch_init(tb);
	tb->fpga5 = 0;
	tb->fpga6 = 0;

	return sp;
}

static void
yield(sim_t *sp)
{
	if (setjmp(sp->test_jb) == 0)
		longjmp(sp->main_jb, 1);
}

static void
timer_tick(sim_t *sp)
{
	Vconan *tb = sp->tb;

	if ((sp->cycle % 65536) != 0)
		return;
	tb->fpga5 = !tb->fpga5;
}

static void
step(sim_t *sp, uint64_t cycle)
{
	int ret;
	Vconan *tb = sp->tb;
	int want_dump = 0;

	sp->cycle = cycle;

	uart_recv_tick(sp->urp, &want_dump);
	uart_send_tick(sp->usp, &want_dump);
	timer_tick(sp);
	tmcuart_tick(sp);

	/* watch output before test, so we might see failure reasons */
	do_watch(sp->wp, cycle);

	/* continue test procedure */
	ret = setjmp(sp->main_jb);
	if (ret == 0)
		longjmp(sp->test_jb, 1);
	if (ret == 1)
		want_dump = 1;

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
	/* clear recv buffer, bump seq */
	sp->urp->pos = 0;
	sp->urp->expected_seq = (sp->urp->expected_seq + 1) & 0x0f;
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
	watch_add(wp, "u_command.msg_state", "cstate", NULL, FORM_DEC, WF_ALL);

	/* send version request */
	uint32_t rsp[2];
	uart_send_vlq(sp, 1, CMD_GET_VERSION);
	wait_for_uart_vlq(sp, 2, rsp);

	if (rsp[0] != 0 || rsp[1] != 0x42) {
		printf("received incorrect version rsp\n");
		exit(1);
	}
	watch_clear(wp);
}

static void
test_time(sim_t *sp)
{
	uart_send_t *usp = sp->usp;
	uart_recv_t *urp = sp->urp;
	Vconan *tb = sp->tb;
	int i;
	uint64_t curr;
	watch_t *wp = sp->wp;

	watch_add(wp, "u_system.cmd_ready", "rdy", NULL, FORM_BIN, WF_ALL);
	watch_add(wp, "u_system.latched_time$", "latch", NULL, FORM_DEC, WF_ALL);
	watch_add(wp, "u_system.latched$", "latched", NULL, FORM_BIN, WF_ALL);
	watch_add(wp, "u_system.state", "state", NULL, FORM_DEC, WF_ALL);
	watch_add(wp, "u_command.msg_state", "cstate", NULL, FORM_DEC, WF_ALL);
	watch_add(wp, "^systime$", "time", NULL, FORM_DEC, WF_PRINT);
	watch_add(wp, "systime_set$", "time_set", NULL, FORM_DEC, WF_ALL);
	watch_add(wp, "systime_set_en$", "time_set_en", NULL, FORM_BIN, WF_ALL);
	watch_add(wp, "fpga5", "fpga5", NULL, FORM_BIN, WF_ALL);
	watch_add(wp, "fpga6", "fpga6", NULL, FORM_BIN, WF_ALL);

	/* send version request */
	uint32_t rsp[3];
	uart_send_vlq(sp, 1, CMD_GET_TIME);
	wait_for_uart_vlq(sp, 3, rsp);
	if (rsp[0] != 1) {
		printf("received incorrect version rsp\n");
		exit(1);
	}
	printf("received time %ld\n", rsp[1] + rsp[2] * (1ull << 32));

	/* send a latch signal. wait for a time pulse and set latch 100 cycles after that */
	uint64_t sync_cycle = ((sp->cycle + 65535) & ~0xffffull);
	printf("delaying %d to get to sync cycle %d = 0x%x\n",
		sync_cycle - sp->cycle, sync_cycle, sync_cycle);
	delay(sp, sync_cycle - sp->cycle + 100);

	tb->fpga6 = 1;
	delay(sp, 10);
	tb->fpga6 = 0;
	delay(sp, 10);

	uart_send_vlq_and_wait(sp, 3, CMD_SYNC_TIME, (uint32_t)(sync_cycle & 0xffffffffull),
		(uint32_t)(sync_cycle >> 32));

	delay(sp, 100);

	/* by now, systime and our cycles have to be equal */
	if (tb->conan__DOT__systime != sp->cycle) {
		printf("time sync failed: %ld != %ld\n",
			tb->conan__DOT__systime, sp->cycle);
		exit(1);
	}
	watch_clear(wp);
}

static void
test_pwm_check_cycle(sim_t *sp, int period, int duty)
{
	Vconan *tb = sp->tb;
	int i;
	uint64_t curr;

	wait_for_signal8(sp, &tb->conan__DOT__pwm1, 0);
	wait_for_signal8(sp, &tb->conan__DOT__pwm1, 1);
	for (i = 0; i < 3; ++i) {
		curr = sp->cycle;
		wait_for_signal8(sp, &tb->conan__DOT__pwm1, 0);
		if (sp->cycle - curr != duty)
			fail("pwm duty period mismatch, expected %d, got %d\n", duty, sp->cycle - curr);
		wait_for_signal8(sp, &tb->conan__DOT__pwm1, 1);
		if (sp->cycle - curr != period)
			fail("pwm period mismatch, expected %d got %d\n", period, sp->cycle - curr);
	}
}

static void
test_pwm(sim_t *sp)
{
	uart_send_t *usp = sp->usp;
	uart_recv_t *urp = sp->urp;
	Vconan *tb = sp->tb;
	int i;

	watch_add(sp->wp, "pwm1$", "p1", NULL, FORM_BIN, WF_ALL);

	/* CONFIGURE_PWM, channel,  cycle_ticks, on_ticks, default_value, max_duration */
	uart_send_vlq_and_wait(sp, 6, CMD_CONFIG_PWM, 0, 1000, 1000 - 100, 0, 50000);
	delay(sp, 100);
	test_pwm_check_cycle(sp, 1000, 100);

	/* give it 100000 cycles to process the message */
	uint32_t sched = sp->cycle + 100000;
	printf("schedule for %d\n", sched);
	uart_send_vlq(sp, 4, CMD_SCHEDULE_PWM, 0, (uint32_t)(sp->cycle + 100000), 1000 - 555);
	delay(sp, 50000);
	/* see that it's not yet scheduled */
	if (sp->cycle > sched - 5000)
		fail("waited too long for test");
	printf("test it's still the old cycle\n");
	test_pwm_check_cycle(sp, 1000, 100);
	delay(sp, 50000);
	if (sp->cycle < sched)
		fail("waited not long enough");
	printf("test it's the new cycle\n");
	test_pwm_check_cycle(sp, 1000, 555);

	/* wait until max duration (+one cycle) is over. pwm should be set to default */
	delay(sp, 152000 - (sp->cycle - sched));
	printf("test it's the default of 0\n");
	for (i = 0; i < 2000; ++i) {
		if (tb->conan__DOT__pwm1 != 0)
			fail("pwm failed to fall back to default");
	}
}

static void
_check_stepdir(sim_t *sp, int interval, int count, int add, int dir, int *step, int *pos, int first)
{
	int i;
	int j;
	Vconan *tb = sp->tb;

	for (i = 0; i < count; ++i) {
		for (j = 0; j < interval; ++j) {
			int exp;

			if (step)
				exp = *step;
			else if (first && i == 0)
				exp = 0;
			else
				exp = j < 2;

			if (tb->step1 != exp)
				fail("step line does not match expectation: %d != %d at i %d j %d\n",
					tb->step1, exp, i, j);
			if (tb->dir1 != dir)
				fail("dir line does not match expectation: %d != %d at i %d j %d\n",
					tb->dir1, dir, i, j);
			yield(sp);
		}
		
		if (step)
			*step = !*step;
		if (pos)
			*pos += dir ? 1 : -1;
		interval += add;
	}
}

static void
test_stepper(sim_t *sp)
{
	Vconan *tb = sp->tb;
	int i;

	/*
         * CMD_CONFIG_STEPPER in: <channel> <dedge>
         * CMD_QUEUE_STEP in: <channel> <interval> <count> <add>
         * CMD_SET_NEXT_STEP_DIR in: <channel> <dir>
         * CMD_RESET_STEP_CLOCK in: <channel> <time>
         * CMD_STEPPER_GET_POS in: <channel>
         * CMD_ENDSTOP_SET_STEPPER in: <endstop-channel> <stepper-channel>
         * CMD_ENDSTOP_QUERY in: <channel>
         * CMD_ENDSTOP_HOME in: <endstop-channel> <time> <sample_count> <pin_value>
	 *
	 * RSP_STEPPER_GET_POS out: <position>
	 * RSP_ENDSTOP_QUERY out: <homing> <pin_value>
	 */

	watch_add(sp->wp, "step1", "step", NULL, FORM_BIN, WF_ALL);
	watch_add(sp->wp, "dir1", "dir", NULL, FORM_BIN, WF_ALL);
	watch_add(sp->wp, "step_start$", "start", NULL, FORM_BIN, WF_ALL);
	watch_add(sp->wp, "\\.step_reset", "reset", NULL, FORM_BIN, WF_ALL);
#if 0
	watch_add(sp->wp, "endstop_step_reset", "esreset", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "endstop_stepper", "esst", NULL, FORM_BIN, WF_ALL);
#endif
	watch_add(sp->wp, "step_start_pending", "pending", NULL, FORM_BIN, WF_ALL);
	watch_add(sp->wp, "step_start_time", "time", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_stepper.state", "state", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_stepper.cmd_ready", "rdy", NULL, FORM_BIN, WF_ALL);
	watch_add(sp->wp, "u_command.msg_state", "msg_state", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "^endstop2$", "es", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "0..u_stepdir.reset", "reset", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "unit_invol_req", "ivrq", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "unit_invol_grant", "ivgr", NULL, FORM_DEC, WF_ALL);
#if 0
	watch_add(sp->wp, "0..u_stepdir.curr_interval", "curr_iv", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "0..u_stepdir.interval", "iv", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "0..u_stepdir.count", "count", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "0..u_stepdir.add", "add", NULL, FORM_DEC, WF_ALL);
#endif
#if 0
	watch_add(sp->wp, "u_command.msg_data", "msg_data", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_command.msg_ready", "msg_ready", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_framing.recv_state", "recv_state", NULL, FORM_DEC, WF_ALL);
#endif

	uart_send_vlq_and_wait(sp, 3, CMD_CONFIG_STEPPER, 0, 1); /* dedge */
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 1000, 10, -10);
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 100, 10, 0);
	uart_send_vlq_and_wait(sp, 3, CMD_SET_NEXT_STEP_DIR, 0, 1);
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 200, 10, 10);
	uint32_t start = sp->cycle + 50000;
	printf("schedule start for %d\n", start);
	uart_send_vlq_and_wait(sp, 3, CMD_RESET_STEP_CLOCK, 0, start);

	/* wait for the scheduled time to arrive */
	delay(sp, start - sp->cycle);

	/* check step/dir each cycle until the program ends */
	int step1 = 0;
	int steppos = 0;
	_check_stepdir(sp, 1000, 10, -10, 0, &step1, &steppos, 1);
	_check_stepdir(sp, 100, 10, 0, 0, &step1, &steppos, 0);
	_check_stepdir(sp, 200, 10, 10, 1, &step1, &steppos, 0);
	/* check step keeps low for another 2000 cycles */
	_check_stepdir(sp, 2000, 1, 0, 1, &step1, NULL, 0);

	/* check position */
	uart_send_vlq(sp, 2, CMD_STEPPER_GET_POS, 0);
	uint32_t rsp[3];
	wait_for_uart_vlq(sp, 2, rsp);
	if (rsp[0] != RSP_STEPPER_GET_POS)
		fail("received incorrect rsp to STEPPER_GET_POS\n");
	if (rsp[1] != steppos)
		fail("stepper pos does not match: %d != %d\n", rsp[1], steppos);

	tb->endstop2 = 0;
	uart_send_vlq(sp, 2, CMD_ENDSTOP_QUERY, 1);
	wait_for_uart_vlq(sp, 3, rsp);
	if (rsp[0] != RSP_ENDSTOP_STATE)
		fail("received incorrect rsp to ENDSTOP_QUERY\n");
	if (rsp[1] != 0)
		fail("endstop state homing\n");
	if (rsp[2] != 0)
		fail("endstop state not 0\n");

	tb->endstop2 = 1;
	uart_send_vlq(sp, 2, CMD_ENDSTOP_QUERY, 1);
	wait_for_uart_vlq(sp, 3, rsp);
	if (rsp[0] != RSP_ENDSTOP_STATE)
		fail("received incorrect rsp to ENDSTOP_QUERY\n");
	if (rsp[1] != 0)
		fail("endstop state homing\n");
	if (rsp[2] != 1)
		fail("endstop state not 1\n");

	/*
	 * test homing
	 */
	uart_send_vlq_and_wait(sp, 3, CMD_ENDSTOP_SET_STEPPER, 1, 0);
	uart_send_vlq_and_wait(sp, 3, CMD_CONFIG_STEPPER, 0, 1); /* dedge */
	uart_send_vlq_and_wait(sp, 3, CMD_SET_NEXT_STEP_DIR, 0, 0);
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 1000, 100, 0);
	start = sp->cycle + 50000;
	printf("schedule start for %d\n", start);
	/* CMD_ENDSTOP_HOME in: <endstop-channel> <time> <sample_count> <pin_value> */
	uart_send_vlq_and_wait(sp, 5, CMD_ENDSTOP_HOME, 1, start, 10, 0);
	uart_send_vlq_and_wait(sp, 3, CMD_RESET_STEP_CLOCK, 0, start);

	/* see that endstop is reported as homing */
	uart_send_vlq(sp, 2, CMD_ENDSTOP_QUERY, 1);
	wait_for_uart_vlq(sp, 3, rsp);
	if (rsp[0] != RSP_ENDSTOP_STATE)
		fail("received incorrect rsp to ENDSTOP_QUERY\n");
	if (rsp[1] != 1)
		fail("endstop state homing\n");
	if (rsp[2] != 1)
		fail("endstop state not 1\n");

	/* wait for the scheduled time to arrive */
	delay(sp, start - sp->cycle);

	delay(sp, 5555);
	tb->endstop2 = 0;
	delay(sp, 5);
	tb->endstop2 = 1;
	delay(sp, 5);
	if (tb->conan__DOT__u_command__DOT__u_stepper__DOT__genstepdir__BRA__0__KET____DOT__u_stepdir__DOT__reset)
		fail("spike triggered homing end\n");
	tb->endstop2 = 0;
	delay(sp, 11);
	if (tb->conan__DOT__u_command__DOT__u_stepper__DOT__genstepdir__BRA__0__KET____DOT__u_stepdir__DOT__reset)
		fail("homing end not triggered after 10 cycles\n");
	delay(sp, 5000);
	if (tb->step1 != !step1)
		fail("step reset by homing\n");
	
	wait_for_uart_vlq(sp, 3, rsp);
	if (rsp[0] != RSP_ENDSTOP_STATE)
		fail("received incorrect rsp to ENDSTOP_QUERY\n");
	if (rsp[1] != 0)
		fail("endstop state homing\n");
	if (rsp[2] != 0)
		fail("endstop state not 0\n");

	/* reset has to be cleared */
	if (tb->conan__DOT__u_command__DOT__u_stepper__DOT__genstepdir__BRA__0__KET____DOT__u_stepdir__DOT__reset)
		fail("reset still on after homing\n");

	/* check step keeps state for another 2000 cycles */
	step1 = tb->step1;
	_check_stepdir(sp, 2000, 1, 0, 0, &step1, NULL, 0);
	/*
	 * test homing abort
	 */
	printf("test homing abort\n");
	tb->endstop2 = 1;
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 1000, 100, 0);
	start = sp->cycle + 50000;
	printf("schedule start for %d\n", start);
	uart_send_vlq_and_wait(sp, 5, CMD_ENDSTOP_HOME, 1, start, 10, 0);
	uart_send_vlq_and_wait(sp, 3, CMD_RESET_STEP_CLOCK, 0, start);

	/* wait for the scheduled time to arrive */
	delay(sp, start - sp->cycle);
	/* abort homing */
	uart_send_vlq_and_wait(sp, 5, CMD_ENDSTOP_HOME, 1, 0, 0, 0);
	
	uart_send_vlq(sp, 2, CMD_ENDSTOP_QUERY, 1);
	wait_for_uart_vlq(sp, 3, rsp);
	if (rsp[0] != RSP_ENDSTOP_STATE || rsp[1] != 0 || rsp[2] != 1)
		fail("homing abort failed\n");

	/* wait for move to finish */
	delay(sp, start + 100000 + 10 - sp->cycle);
	step1 = tb->step1;
	_check_stepdir(sp, 2000, 1, 0, 0, &step1, NULL, 0);

	/*
	 * test regular move after homing again, this time without dedge
	 */
	printf("test move without dedge\n");
	uart_send_vlq_and_wait(sp, 3, CMD_CONFIG_STEPPER, 0, 0);
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 1000, 10, -10);
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 500, 10, 10);
	start = sp->cycle + 50000;
	printf("schedule start for %d\n", start);
	uart_send_vlq_and_wait(sp, 3, CMD_RESET_STEP_CLOCK, 0, start);

	/* wait for the scheduled time to arrive */
	delay(sp, start - sp->cycle);

	/* check step/dir each cycle until the program ends */
	_check_stepdir(sp, 1000, 10, -10, 0, NULL, &steppos, 1);
	_check_stepdir(sp, 500, 10, 10, 0, NULL, &steppos, 0);
	delay(sp, 2);	/* final step pulse */
	/* check step keeps low for another 2000 cycles */
	for (i = 0; i < 2000; ++i)
		if (tb->step1 != 0)
			fail("stepper moved after finish\n");

	watch_clear(sp->wp);
}

/*
 * handler to simulate tmc2209s
 */
static tmcuart_t *
tmcuart_init(sim_t *sp, vluint8_t *in, vluint8_t *out, const char *name)
{
	tmcuart_t *tu = (tmcuart_t *)calloc(sizeof(*tu), 1);
	uint64_t d = HZ / 200000;	/* uart divider */

	tu->urp = uart_recv_init(&tu->uart_in, d, name);
	tu->usp = uart_send_init(&tu->uart_out, d, name);
	tu->line_in = in;
	tu->line_out = out;
	*out = 1;
	tu->state = TU_READ;

	tu->regs[IFCNT] = 0;
	tu->regs[IOIN] = 0x21000000;	/* version */
}

static void
tmcuart_free(tmcuart_t *tu)
{
	uart_recv_free(tu->urp);
	uart_send_free(tu->usp);
	free(tu);
}

static uint8_t
tmcuart_crc(uint8_t *b, int l)
{
	int i;
	int j;
	uint8_t crc = 0;
	uint8_t currentByte;

	for (i = 0; i < l; i++) {
		currentByte = b[i];
		for (j = 0; j < 8; j++) {
			if ((crc >> 7) ^ (currentByte & 0x01))
				crc = (crc << 1) ^ 0x07;
			else
				crc = (crc << 1);
			currentByte = currentByte >> 1;
		}
	}

	return crc;
}

static void
tmcuart_reset(tmcuart_t *tu)
{
	tu->state = TU_READ;
	tu->urp->pos = 0;
	tu->delay = 0;
	tu->last_pos = 0;
}

static void
tmcuart_tick(sim_t *sp)
{
	int i;
	uint8_t crc;

	for (i = 0; i < NUART; ++i) {
		int want_dump = 0; /* dummy */
		tmcuart_t *tu = sp->tmcuart[i];

		if (tu == NULL)
			continue;

		uart_send_t *usp = tu->usp;
		uart_recv_t *urp = tu->urp;

		if (tu->state == TU_READ) {
			tu->uart_in = *tu->line_in;
		} else if (tu->state == TU_WRITE) {
			*tu->line_out = tu->uart_out;
		}
		uart_send_tick(usp, &want_dump);
		uart_recv_tick(urp, &want_dump);

		if (tu->state == TU_READ && urp->pos) {
			if (urp->pos != tu->last_pos)
				tu->last_change = sp->cycle;
			if (tu->last_change && (sp->cycle - tu->last_change) >
			    HZ / 200000 * 63) {
				printf("tmcuart reset\n");
				tmcuart_reset(tu);
			}
		}

		if (tu->state == TU_IGNORE) {
			/* already examined the packet, it's bad */
		} else if (tu->state == TU_READ && urp->pos == 4) {
			if ((urp->buf[0] & 0xf0) == 0xa0 &&
			    urp->buf[1] == 0x00 &&
			    (urp->buf[2] & 1) == 0) {
				/* read request, check crc */
				crc = tmcuart_crc(urp->buf, 3);
				if (urp->buf[3] == crc) {
					tu->state = TU_TURNAROUND;
					tu->delay = HZ / 200000 * 8; /* 8 bit times turnaround */
				} else {
					printf("crc mismatch, ignore: %02x != %02x\n", urp->buf[3], crc);
					tu->state = TU_IGNORE;
				}
			}
		} else if (tu->state == TU_READ && urp->pos == 8) {
			/* check write */
			if ((urp->buf[0] & 0xf0) == 0xa0 &&
			    urp->buf[1] == 0x00 &&
			    (urp->buf[2] & 1) == 1) {
				/* read request, check crc */
				crc = tmcuart_crc(urp->buf, 7);
				if (urp->buf[7] == crc) {
					int reg = urp->buf[2] >> 1;
					uint32_t data = (urp->buf[3] << 24) | (urp->buf[4] << 16) |
							(urp->buf[5] << 8) | urp->buf[6];
					tu->regs[reg] = data;
					printf("writing %x to reg %d\n", data, reg);
					++tu->regs[IFCNT];
					tmcuart_reset(tu);
				} else {
					printf("crc mismatch, ignore: %02x != %02x\n", urp->buf[7], crc);
					tu->state = TU_IGNORE;
				}
			} else {
				tmcuart_reset(tu);
			}
		} else if (tu->state == TU_TURNAROUND && --tu->delay == 0) {
			uint8_t outbuf[8];
			int reg = urp->buf[2] >> 1;

			printf("received valid read for reg %d\n", reg);
			outbuf[0] = 0xa0;
			outbuf[1] = 0xff;
			outbuf[2] = reg << 1;
			outbuf[3] = tu->regs[reg] >> 24;
			outbuf[4] = (tu->regs[reg] >> 16) & 0xff;
			outbuf[5] = (tu->regs[reg] >> 8) & 0xff;
			outbuf[6] = tu->regs[reg] & 0xff;
			outbuf[7] = tmcuart_crc(outbuf, 7);
			uart_send(usp, outbuf, 8);
			urp->pos = 0;
			tu->state = TU_WRITE;
		} else if (tu->state == TU_WRITE) {
			if (uart_send_done(usp)) {
				tu->state = TU_TURNBACK;
				tu->delay = HZ / 200000 * 12;
			}
		} else if (tu->state == TU_TURNBACK && --tu->delay == 0) {
			tu->state = TU_READ;
			printf("tmcuart(%s) ready to read again\n", tu->usp->name);
		}
	}
}

static void
test_tmcuart(sim_t *sp)
{
	Vconan *tb = sp->tb;
	int i;

	/*
	 * CMD_TMCUART_WRITE in <channel> <slave> <register> <data>
	 * CMD_TMCUART_READ in <channel> <slave> <register>
	 * RSP_TMCUART_READ in <status> <data>
	 */

	sp->tmcuart[0] = tmcuart_init(sp, &tb->uart1, &tb->uart1_in, "uart1");
	sp->tmcuart[1] = tmcuart_init(sp, &tb->uart2, &tb->uart2_in, "uart2");
	sp->tmcuart[2] = tmcuart_init(sp, &tb->uart3, &tb->uart3_in, "uart3");
	sp->tmcuart[3] = tmcuart_init(sp, &tb->uart4, &tb->uart4_in, "uart4");
	sp->tmcuart[4] = tmcuart_init(sp, &tb->uart5, &tb->uart5_in, "uart5");
	sp->tmcuart[5] = tmcuart_init(sp, &tb->uart6, &tb->uart6_in, "uart6");
#if 0
tmcuart_init(sim_t *sp, vluint8_t *in, vluint8_t *out, vluint8_t *en, int mask)
        CData/*5:0*/ conan__DOT__u_command__DOT__u_tmcuart__DOT__uart__out__out0;
        CData/*5:0*/ conan__DOT__u_command__DOT__u_tmcuart__DOT__uart__out__en0;
        CData/*5:0*/ conan__DOT__u_command__DOT__u_tmcuart__DOT__uart;
#endif

	watch_add(sp->wp, "^uart.$", "ua", NULL, FORM_BIN, WF_ALL);
	watch_add(sp->wp, "tmcuart.state", "state", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "tmcuart.uart_tx_en", "tx_en", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "tmcuart.uart_tx_data", "tx_data", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "tmcuart.uart_rx_ready", "rx_rdy", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "tmcuart.uart_rx_data", "rx_data", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "tmcuart.uart_rx$", "rx", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "tmcuart.uart_tx$", "tx", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "tmcuart.uart_in$", "in", NULL, FORM_BIN, WF_ALL);
	watch_add(sp->wp, "tmcuart.uart_out$", "out", NULL, FORM_BIN, WF_ALL);
	watch_add(sp->wp, "tmcuart.uart_en$", "en", NULL, FORM_BIN, WF_ALL);
	watch_add(sp->wp, "tmcuart.wire_en$", "wen", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "tmcuart.channel$", "ch", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "tmcuart.uart_transmitting", "txing", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_tmcuart.cmd_ready", "rdy", NULL, FORM_BIN, WF_ALL);
	watch_add(sp->wp, "u_command.msg_state", "msg_state", NULL, FORM_DEC, WF_ALL);
#if 0
	watch_add(sp->wp, "tmcuart.status", "status", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "tmcuart.receiving", "rcving", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "tmcuart.delay", "delay", NULL, FORM_DEC, WF_PRINT);
#endif
	watch_add(sp->wp, "tmcuart.crc$", "crc", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "tmcuart.crc_in$", "in", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "tmcuart.crc_in_en", "en", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "tmcuart.crc_count", "cnt", NULL, FORM_HEX, WF_ALL);

	/* read version */
	uint32_t rsp[3];
	uart_send_vlq(sp, 4, CMD_TMCUART_READ, 2, 0, IOIN);
	wait_for_uart_vlq(sp, 3, rsp);
	if (rsp[0] != RSP_TMCUART_READ)
		fail("tmcuart read version failed\n");
	if (rsp[1] != 0)
		fail("tmcuart read status %d\n", rsp[1]);
	if (rsp[2] != 0x21000000)
		fail("tmcuart read bad version %x\n", rsp[2]);

	/* timeout on a different slave */
	uart_send_vlq(sp, 4, CMD_TMCUART_READ, 2, 1, IOIN);
	wait_for_uart_vlq(sp, 3, rsp);
	if (rsp[0] != RSP_TMCUART_READ)
		fail("tmcuart test timeout\n");
	if (rsp[1] != 1)
		fail("tmcuart read status %d\n", rsp[1]);
	if (rsp[2] != 0)
		fail("tmcuart read bad version %x\n", rsp[2]);
	/* reset receiver, it received zeros due to problems with verilator and 'z' */
	tmcuart_reset(sp->tmcuart[2]);

	/* read version again */
	printf("read version again\n");
	uart_send_vlq(sp, 4, CMD_TMCUART_READ, 2, 0, IOIN);
	wait_for_uart_vlq(sp, 3, rsp);
	if (rsp[0] != RSP_TMCUART_READ)
		fail("tmcuart read version failed\n");
	if (rsp[1] != 0)
		fail("tmcuart read status %d\n", rsp[1]);
	if (rsp[2] != 0x21000000)
		fail("tmcuart read bad version %x\n", rsp[2]);

	/* write */
	uart_send_vlq_and_wait(sp, 5, CMD_TMCUART_WRITE, 2, 0, 10, 0x1234);

	/* read */
	uart_send_vlq(sp, 5, CMD_TMCUART_READ, 2, 0, 10);
	wait_for_uart_vlq(sp, 3, rsp);
	if (rsp[0] != RSP_TMCUART_READ)
		fail("tmcuart read version failed\n");
	if (rsp[1] != 0)
		fail("tmcuart read status %d\n", rsp[1]);
	if (rsp[2] != 0x1234)
		fail("tmcuart read bad register content %x\n", rsp[2]);

	watch_clear(sp->wp);
	for (i = 0; i < 6; ++i) {
		free(sp->tmcuart[i]);
		sp->tmcuart[i] = NULL;
	}
}

static void
test(sim_t *sp)
{
	delay(sp, 1);	/* pass back control after initialization */

	test_time(sp);	/* always needed as time sync */
	test_version(sp);
	test_pwm(sp);
	test_stepper(sp);
	test_tmcuart(sp);

	printf("test succeeded after %d cycles\n", sp->cycle);

	exit(0);
}

int
main(int argc, char **argv) {
	// Initialize Verilators variables
	Verilated::commandArgs(argc, argv);
	uint64_t cycle = 100000;
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
