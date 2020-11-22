#include <stdlib.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <pcre.h>
#include "Vconan.h"
#include "verilated.h"
#include "vsyms.h"

static int color_disabled = 1;

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
#define CMD_SET_DIGITAL_OUT	15
#define CMD_CONFIG_DIGITAL_OUT	16
#define CMD_SCHEDULE_DIGITAL_OUT 17
#define CMD_UPDATE_DIGITAL_OUT	18
#define CMD_SHUTDOWN		19
#define CMD_STEPPER_GET_NEXT	20
#define CMD_CONFIG_DRO		21
#define CMD_CONFIG_AS5311	22
#define CMD_SD_QUEUE		23

#define RSP_GET_VERSION		0
#define RSP_GET_TIME		1
#define RSP_STEPPER_GET_POS	2
#define RSP_ENDSTOP_STATE	3
#define RSP_TMCUART_READ	4
#define RSP_SHUTDOWN		5
#define RSP_STEPPER_GET_NEXT	6
#define RSP_DRO_DATA		7
#define RSP_AS5311_DATA		8
#define RSP_SD_CMDQ		9
#define RSP_SD_DATQ		10

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

#define AS_IDLE		0
#define AS_CLK_HI	1
#define AS_CLK_LO	2
#define AS_WAIT_CS	3

#define NAS5311		3

typedef struct {
	int		state;
	uint32_t	data;
	int		cnt;
	uint16_t	magnet;
	uint16_t	sensor;
	vluint8_t	*cs;
	vluint8_t	*clk;
	vluint8_t	*dout;
} as5311_t;

#define SDC_IDLE	0
#define SDC_RCV		1
#define SDC_SND_DELAY	2
#define SDC_SND		3
typedef struct {
	vluint8_t	*clk;
	vluint8_t	*cmd_in;
	vluint8_t	*cmd_out;
	vluint8_t	*dat0_in;
	vluint8_t	*dat1_in;
	vluint8_t	*dat2_in;
	vluint8_t	*dat3_in;
	vluint8_t	*dat0_out;
	vluint8_t	*dat1_out;
	vluint8_t	*dat2_out;
	vluint8_t	*dat3_out;
	vluint8_t	*cmd_en;
	vluint8_t	*dat_en;
	int		last_clk;
	int		bitcnt;
	int		cmd_state;
	uint8_t		cmd_rcv[6];
	int		cmd_rcv_ready;
	int		sndbits;
	uint8_t		sndbuf[17];
} sd_t;

typedef struct {
	Vconan		*tb;
	uart_recv_t	*urp;
	uart_send_t	*usp;
	as5311_t	*as5311[NAS5311];
	sd_t		*sd;
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
static void as5311_tick(sim_t *sp);
static void sd_tick(sim_t *sp);
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

	if (color_disabled)
		do_color = COLOR_NONE;
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

#if 0
	if (cycle - wp->last_cycle > 1000000)
		fail("abort due to inactivity\n");
#endif

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

/*
 * num positive: number of parameters
 * num negative: number of parameters of which the last one is a string
 */
static void
uart_send_vlq_va(sim_t *sp, int num, va_list ap)
{
	uint8_t buf[256]; 	/* just enough */
	uint8_t *p = buf;
	uint32_t arg;
	int i;
	int n = num < 0 ? -num : num;

printf("num %d n %d\n", num, n);
	for (i = 0; i < n; ++i) {
		arg = va_arg(ap, uint32_t);
printf("arg %d\n", arg);
		p = encode_int(p, arg);
	}
	if (num < 0) {
printf("add string len %d\n", arg);
		/* arg, the value of the last parameter, is the length of the following string */
		for (i = 0; i < arg; ++i)
			*p++ = (uint8_t)va_arg(ap, uint32_t);
	}

	uart_send_packet(sp->usp, buf, p - buf);
}

static void
uart_send_vlq(sim_t *sp, int num, ...)
{
	va_list ap;
	va_start(ap, num);

	uart_send_vlq_va(sp, num, ap);

        va_end(ap);
}

static void
uart_send_vlq_and_wait(sim_t *sp, int num, ...)
{
	va_list ap;
	va_start(ap, num);

	uart_send_vlq_va(sp, num, ap);

        va_end(ap);

	uint8_t buf[num * 5];

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
	as5311_tick(sp);
	sd_tick(sp);

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
wait_for_uart_vlq(sim_t *sp, int _n, uint32_t *vlq)
{
	int ret;
	int i;
	int pos = 2;
	int len;
	int n = _n > 0 ? _n : -_n ;

	wait_for_uart_recv(sp);
	len = sp->urp->pos - 5;
	for (i = 0; i < n; ++i) {
		ret = parse_int(sp->urp->buf, pos, len, vlq + i);
		pos += ret;
		len -= ret;
	}
	if (_n < 0) {
		for (i = 0; i < vlq[n - 1]; ++i) {
			vlq[i + n] = sp->urp->buf[pos++];
			len -= 1;
		}
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
	uint32_t rsp[4];
	uart_send_vlq(sp, 1, CMD_GET_VERSION);
	wait_for_uart_vlq(sp, 4, rsp);

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

	watch_add(wp, "u_command.tmp_arg", "tmp_arg", NULL, FORM_HEX, WF_ALL);
	watch_add(wp, "u_command.args", "args", NULL, FORM_HEX, WF_ALL);
	watch_add(wp, "u_command.curr_arg", "curr_arg", NULL, FORM_HEX, WF_ALL);
	watch_add(wp, "u_system.arg_data", "arg_data", NULL, FORM_HEX, WF_ALL);
	watch_add(wp, "u_system.temp_data", "temp_data", NULL, FORM_HEX, WF_ALL);
	watch_add(wp, "u_system.cmd_ready", "rdy", NULL, FORM_BIN, WF_ALL);
	watch_add(wp, "u_system.latched_time$", "latch", NULL, FORM_DEC, WF_ALL);
	watch_add(wp, "u_system.latched$", "latched", NULL, FORM_BIN, WF_ALL);
	watch_add(wp, "u_system.state", "state", NULL, FORM_DEC, WF_ALL);
	watch_add(wp, "u_command.msg_state", "cstate", NULL, FORM_DEC, WF_ALL);
	watch_add(wp, "^systime$", "time", NULL, FORM_DEC, WF_PRINT);
	watch_add(wp, "systime_set$", "time_set", NULL, FORM_DEC, WF_ALL);
	watch_add(wp, "systime_set_en$", "time_set_en", NULL, FORM_BIN, WF_ALL);
	watch_add(wp, "fpga5", "fpga5", NULL, FORM_BIN, WF_ALL);

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
	watch_add(sp->wp, "on_ticks$", "on", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "off_ticks$", "off", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "scheduled$", "sched", NULL, FORM_BIN, WF_ALL);
	watch_add(sp->wp, "next_time$", "next", NULL, FORM_DEC, WF_ALL);

	if (tb->conan__DOT__pwm1 != 0)
		fail("wrong pwm startup value\n");

	/* CONFIGURE_PWM, channel, value, default_value, max_duration */
	uart_send_vlq_and_wait(sp, 5, CMD_CONFIG_PWM, 0, 1, 0, 200000);
	delay(sp, 100);

	for (i = 0; i < 10000; ++i) {
		if (tb->conan__DOT__pwm1 != 1)
			fail("wrong pwm initial value\n");
		yield(sp);
	}

	uart_send_vlq(sp, 5, CMD_SCHEDULE_PWM, 0, (uint32_t)(sp->cycle + 100000), 100, 900);
	delay(sp, 100);
	test_pwm_check_cycle(sp, 1000, 100);

	/* give it 100000 cycles to process the message */
	uint32_t sched = sp->cycle + 100000;
	printf("schedule for %d\n", sched);
	uart_send_vlq(sp, 5, CMD_SCHEDULE_PWM, 0, (uint32_t)(sp->cycle + 100000), 555, 445);
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
	delay(sp, 200000 - (sp->cycle - sched));
	printf("test it's the default of 0\n");
	for (i = 0; i < 2000; ++i) {
		if (tb->conan__DOT__pwm1 != 0)
			fail("pwm failed to fall back to default\n");
		yield(sp);
	}
	sched = sp->cycle + 50000;
	uart_send_vlq(sp, 5, CMD_SCHEDULE_PWM, 0, sched, 111, 889);
	delay(sp, 50000);
	test_pwm_check_cycle(sp, 1000, 111);
	sched = sp->cycle + 50000;
	uart_send_vlq(sp, 5, CMD_SCHEDULE_PWM, 0, sched, 0, 1);	/* always on */
	delay(sp, 51000);
	for (i = 0; i < 10000; ++i) {
		if (tb->conan__DOT__pwm1 != 1)
			fail("pwm not always on\n");
		yield(sp);
	}
	sched = sp->cycle + 50000;
	uart_send_vlq(sp, 5, CMD_SCHEDULE_PWM, 0, sched, 1, 0);	/* always off */
	delay(sp, 51000);
	for (i = 0; i < 10000; ++i) {
		if (tb->conan__DOT__pwm1 != 0)
			fail("pwm not always off\n");
		yield(sp);
	}
	sched = sp->cycle + 50000;
	uart_send_vlq(sp, 5, CMD_SCHEDULE_PWM, 0, sched, 1, 0);	/* same again */
	delay(sp, 49000);
	for (i = 0; i < 10000; ++i) {
#if 0
		if (tb->conan__DOT__pwm1 != 0)
			fail("pwm not always off (2nd schedule)\n");
#endif
		yield(sp);
	}
	sched = sp->cycle + 50000;
	uart_send_vlq(sp, 5, CMD_SCHEDULE_PWM, 0, sched, 222, 778);
	delay(sp, 50000);
	test_pwm_check_cycle(sp, 1000, 222);

	struct _testv {
		uint32_t clock;
		uint32_t on;
		uint32_t off;
	} testv[] = {
		{ 3328252160, 4800000, 0 },
		{ 3529852160, 4800000, 0 },
		{ 3587452160, 4549796, 250204 },
		{ 3630652160, 4208293, 591707 },
		{ 3673852160, 3893000, 907000 },
		{ 3731452160, 3604654, 1195346 },
		{ 3789052160, 3252107, 1547893 },
		{ 3904252160, 2988908, 1811092 },
		{ 3990652160, 3260292, 1539708 },
		{ 4033852160, 3845186, 954814 },
		{ 4077052160, 4191617, 608383 },
		{ 4120252160, 4522369, 277631 },
		{ 4293052160, 4775595, 24405 },
		{ 199684864, 4800000, 0 },
		{ 401284864, 4800000, 0 },
	};
	uint32_t base = testv[0].clock;
	uart_send_vlq_and_wait(sp, 5, CMD_CONFIG_PWM, 0, 1, 0, 24000000);
	delay(sp, 50000);
	sched = sp->cycle + 50000;
	for (i = 0; i < sizeof(testv) / sizeof(struct _testv); ++i) {
		uint32_t on = testv[i].on;
		uint32_t off = testv[i].off;
		uint32_t diff = testv[i].clock - testv[i - 1].clock;
		if (on == 0) {
			on = 1;
			off = 0;
		} else if (off == 1) {
			on = 0;
			off = 1;
		}

		printf("diff: %u\n", testv[i].clock - testv[i - 1].clock);
		uart_send_vlq(sp, 5, CMD_SCHEDULE_PWM, 0, sched, on, off);
		delay(sp, diff);
	}

	watch_clear(sp->wp);
}

static void
_check_stepdir(sim_t *sp, int interval, int count, int add, int dir, int *step, int *pos, int first)
{
	int i;
	int j;
	Vconan *tb = sp->tb;
	int init_dir = tb->dir1;
	int dircnt = 0;

	for (i = 0; i < count; ++i) {
		for (j = 0; j < interval; ++j) {
			int exp;
			int dir_exp;

			if (step)
				exp = *step;
			else if (first && i == 0)
				exp = 0;
			else
				exp = j < 8;

			if (tb->step1 != exp)
				fail("%d: step line does not match expectation: %d != %d at i %d j %d\n",
					sp->cycle, tb->step1, exp, i, j);
			dir_exp = dircnt < 8 ? init_dir : dir;
			if (tb->dir1 != dir_exp)
				fail("%d: dir line does not match expectation: %d != %d at i %d j %d\n",
					sp->cycle, tb->dir1, dir_exp, i, j);
			++dircnt;
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
#if 0
	watch_add(sp->wp, "endstop_step_reset", "esreset", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "endstop_stepper", "esst", NULL, FORM_BIN, WF_ALL);
#endif
	watch_add(sp->wp, "u_stepper.state", "state", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_stepper.cmd_ready", "rdy", NULL, FORM_BIN, WF_ALL);
	watch_add(sp->wp, "u_command.msg_state", "msg_state", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "^endstop2$", "es", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "0..u_stepdir.reset", "reset", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "unit_invol_req", "ivrq", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "unit_invol_grant", "ivgr", NULL, FORM_DEC, WF_ALL);

#if 0
	watch_add(sp->wp, "0..u_stepdir.next_step", "next_step", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "0..u_stepdir.q_interval", "q_iv", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "0..u_stepdir.interval", "iv", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "0..u_stepdir.count", "count", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "0..u_stepdir.add", "add", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "0..u_stepdir.queue_rd_data", "q", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "0..u_stepdir.queue_empty", "queue_empty", NULL, FORM_HEX, WF_ALL);
#endif
#if 0
	watch_add(sp->wp, "u_command.msg_data", "msg_data", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_command.msg_ready", "msg_ready", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_framing.recv_state", "recv_state", NULL, FORM_DEC, WF_ALL);
#endif

	uint32_t start = sp->cycle;
	uart_send_vlq_and_wait(sp, 3, CMD_CONFIG_STEPPER, 0, 1); /* dedge */
	uart_send_vlq_and_wait(sp, 3, CMD_RESET_STEP_CLOCK, 0, start);
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 250000, 1, 0);
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 1000, 10, -10);
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 100, 10, 0);
	uart_send_vlq_and_wait(sp, 3, CMD_SET_NEXT_STEP_DIR, 0, 1);
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 200, 10, 10);

	start += 250000 - 10;

	printf("schedule start for %d\n", start);
	/* wait for the scheduled time to arrive */
	delay(sp, start - sp->cycle);

	/* check step/dir each cycle until the program ends */
	int step1 = 0;
	int steppos = 0;
	_check_stepdir(sp, 10, 1, 0, 0, &step1, &steppos, 1);
	_check_stepdir(sp, 1000, 10, -10, 0, &step1, &steppos, 0);
	_check_stepdir(sp, 100, 10, 0, 0, &step1, &steppos, 0);
	_check_stepdir(sp, 200, 10, 10, 1, &step1, &steppos, 0);
	/* check step keeps low for another 2000 cycles */
	_check_stepdir(sp, 2000, 1, 0, 1, &step1, NULL, 0);

	/* check position */
	uart_send_vlq(sp, 2, CMD_STEPPER_GET_POS, 0);
	uint32_t rsp[4];
	wait_for_uart_vlq(sp, 3, rsp);
	if (rsp[0] != RSP_STEPPER_GET_POS)
		fail("received incorrect rsp to STEPPER_GET_POS\n");
	if (rsp[1] != 0)
		fail("received bad channel\n");
	if (rsp[2] != steppos)
		fail("stepper pos does not match: %d != %d\n", rsp[1], steppos);

	tb->endstop2 = 0;
	uart_send_vlq(sp, 2, CMD_ENDSTOP_QUERY, 1);
	wait_for_uart_vlq(sp, 4, rsp);
	if (rsp[0] != RSP_ENDSTOP_STATE)
		fail("received incorrect rsp to ENDSTOP_QUERY\n");
	if (rsp[1] != 1)
		fail("received bad channel\n");
	if (rsp[2] != 0)
		fail("endstop state homing\n");
	if (rsp[3] != 0)
		fail("endstop state not 0\n");

	tb->endstop2 = 1;
	uart_send_vlq(sp, 2, CMD_ENDSTOP_QUERY, 1);
	wait_for_uart_vlq(sp, 4, rsp);
	if (rsp[0] != RSP_ENDSTOP_STATE)
		fail("received incorrect rsp to ENDSTOP_QUERY\n");
	if (rsp[1] != 1)
		fail("received bad channel\n");
	if (rsp[2] != 0)
		fail("endstop state homing\n");
	if (rsp[3] != 1)
		fail("endstop state not 1\n");

	/*
	 * test homing
	 */
	uart_send_vlq_and_wait(sp, 3, CMD_ENDSTOP_SET_STEPPER, 1, 0);
	uart_send_vlq_and_wait(sp, 3, CMD_CONFIG_STEPPER, 0, 1); /* dedge */
	uart_send_vlq_and_wait(sp, 3, CMD_SET_NEXT_STEP_DIR, 0, 0);
	start = sp->cycle;
	uart_send_vlq_and_wait(sp, 3, CMD_RESET_STEP_CLOCK, 0, start);
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 200000, 1, 0);
	/* during this move we'll stop the homing */
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 1000, 100, 0);
	/* this move has to be flushed */
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 888, 88, 0);
	start += 200000;
	printf("schedule start for %d (test homing)\n", start);
	/* CMD_ENDSTOP_HOME in: <endstop-channel> <time> <sample_count> <pin_value> */
	uart_send_vlq_and_wait(sp, 5, CMD_ENDSTOP_HOME, 1, start, 10, 0);

	/* see that endstop is reported as homing */
	uart_send_vlq(sp, 2, CMD_ENDSTOP_QUERY, 1);
	wait_for_uart_vlq(sp, 4, rsp);
	if (rsp[0] != RSP_ENDSTOP_STATE)
		fail("received incorrect rsp to ENDSTOP_QUERY\n");
	if (rsp[1] != 1)
		fail("received bad channel\n");
	if (rsp[2] != 1)
		fail("endstop state homing\n");
	if (rsp[3] != 1)
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

	wait_for_uart_vlq(sp, 4, rsp);
	if (rsp[0] != RSP_ENDSTOP_STATE)
		fail("received incorrect rsp to ENDSTOP_QUERY\n");
	if (rsp[1] != 1)
		fail("received bad channel\n");
	if (rsp[2] != 0)
		fail("endstop state homing\n");
	if (rsp[3] != 0)
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
	start = sp->cycle;
	uart_send_vlq_and_wait(sp, 3, CMD_RESET_STEP_CLOCK, 0, start);
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 150000, 1, 0);
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 1000, 100, 0);
	start += 150000;
	printf("schedule start for %d (homing abort)\n", start);
	uart_send_vlq_and_wait(sp, 5, CMD_ENDSTOP_HOME, 1, start, 10, 0);

	/* wait for the scheduled time to arrive */
	delay(sp, start - sp->cycle);
	/* abort homing */
	uart_send_vlq_and_wait(sp, 5, CMD_ENDSTOP_HOME, 1, 0, 0, 0);

	uart_send_vlq(sp, 2, CMD_ENDSTOP_QUERY, 1);
	wait_for_uart_vlq(sp, 4, rsp);
	if (rsp[0] != RSP_ENDSTOP_STATE || rsp[1] != 1 || rsp[2] != 0 || rsp[3] != 1)
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
	start = sp->cycle;
	uart_send_vlq_and_wait(sp, 3, CMD_RESET_STEP_CLOCK, 0, start);
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 150000, 1, 0);
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 1000, 10, -10);
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 500, 10, 10);

	start += 150000 - 10;

	printf("schedule start for %d (no dedge)\n", start);

	/* wait for the scheduled time to arrive */
	delay(sp, start - sp->cycle);

	/* check step/dir each cycle until the program ends */
	_check_stepdir(sp, 10, 1, 0, 0, NULL, &steppos, 1);
	_check_stepdir(sp, 1000, 10, -10, 0, NULL, &steppos, 0);
	_check_stepdir(sp, 500, 10, 10, 0, NULL, &steppos, 0);
	delay(sp, 8);	/* final step pulse */
	/* check step keeps low for another 2000 cycles */
	for (i = 0; i < 2000; ++i)
		if (tb->step1 != 0)
			fail("stepper moved after finish\n");

	start = sp->cycle;
	uart_send_vlq_and_wait(sp, 3, CMD_RESET_STEP_CLOCK, 0, start);
	/* send a command in the past */
	uart_send_vlq_and_wait(sp, 5, CMD_QUEUE_STEP, 0, 1000, 1, 0);
	wait_for_uart_vlq(sp, 3, rsp);
	if (rsp[0] != RSP_SHUTDOWN || rsp[1] != 1)
		fail("failed to shutdown\n");

	watch_clear(sp->wp);
}

/*
 * handler to simulate tmc2209s
 */
static tmcuart_t *
tmcuart_init(sim_t *sp, vluint8_t *in, vluint8_t *out, const char *name)
{
	tmcuart_t *tu = (tmcuart_t *)calloc(sizeof(*tu), 1);
	uint64_t d = HZ / 250000;	/* uart divider */

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
			    HZ / 250000 * 63) {
				printf("tmcuart reset\n");
				tmcuart_reset(tu);
			}
		}

		if (tu->state == TU_IGNORE) {
			/* already examined the packet, it's bad */
		} else if (tu->state == TU_READ && urp->pos == 4) {
			if ((urp->buf[0] & 0x0f) == 0x05 &&
			    urp->buf[1] == 0x00 &&
			    (urp->buf[2] & 0x80) == 0) {
				/* read request, check crc */
				crc = tmcuart_crc(urp->buf, 3);
				if (urp->buf[3] == crc) {
					tu->state = TU_TURNAROUND;
					tu->delay = HZ / 250000 * 8; /* 8 bit times turnaround */
				} else {
					printf("crc mismatch, ignore: %02x != %02x\n", urp->buf[3], crc);
					tu->state = TU_IGNORE;
				}
			}
		} else if (tu->state == TU_READ && urp->pos == 8) {
			/* check write */
			if ((urp->buf[0] & 0x0f) == 0x05 &&
			    urp->buf[1] == 0x00 &&
			    (urp->buf[2] & 0x80) == 0x80) {
				/* read request, check crc */
				crc = tmcuart_crc(urp->buf, 7);
				if (urp->buf[7] == crc) {
					int reg = urp->buf[2] & 0x7f;
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
			int reg = urp->buf[2] & 0x7f;

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
				tu->delay = HZ / 250000 * 12;
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
	uint32_t rsp[4];
	uart_send_vlq(sp, 4, CMD_TMCUART_READ, 2, 0, IOIN);
	wait_for_uart_vlq(sp, 4, rsp);
	if (rsp[0] != RSP_TMCUART_READ)
		fail("tmcuart read version failed\n");
	if (rsp[1] != 2)
		fail("received bad channel\n");
	if (rsp[2] != 0)
		fail("tmcuart read status %d\n", rsp[1]);
	if (rsp[3] != 0x21000000)
		fail("tmcuart read bad version %x\n", rsp[2]);

	/* timeout on a different slave */
	uart_send_vlq(sp, 4, CMD_TMCUART_READ, 2, 1, IOIN);
	wait_for_uart_vlq(sp, 4, rsp);
	if (rsp[0] != RSP_TMCUART_READ)
		fail("tmcuart test timeout\n");
	if (rsp[1] != 2)
		fail("received bad channel\n");
	if (rsp[2] != 1)
		fail("tmcuart read status %d\n", rsp[1]);
	if (rsp[3] != 0)
		fail("tmcuart read bad version %x\n", rsp[2]);
	/* reset receiver, it received zeros due to problems with verilator and 'z' */
	tmcuart_reset(sp->tmcuart[2]);

	/* read version again */
	printf("read version again\n");
	uart_send_vlq(sp, 4, CMD_TMCUART_READ, 2, 0, IOIN);
	wait_for_uart_vlq(sp, 4, rsp);
	if (rsp[0] != RSP_TMCUART_READ)
		fail("tmcuart read version failed\n");
	if (rsp[1] != 2)
		fail("received bad channel\n");
	if (rsp[2] != 0)
		fail("tmcuart read status %d\n", rsp[1]);
	if (rsp[3] != 0x21000000)
		fail("tmcuart read bad version %x\n", rsp[2]);

	/* write */
	uart_send_vlq_and_wait(sp, 5, CMD_TMCUART_WRITE, 2, 0, 10, 0x1234);

	/* read */
	uart_send_vlq(sp, 4, CMD_TMCUART_READ, 2, 0, 10);
	wait_for_uart_vlq(sp, 4, rsp);
	if (rsp[0] != RSP_TMCUART_READ)
		fail("tmcuart read version failed\n");
	if (rsp[1] != 2)
		fail("received bad channel\n");
	if (rsp[2] != 0)
		fail("tmcuart read status %d\n", rsp[1]);
	if (rsp[3] != 0x1234)
		fail("tmcuart read bad register content %x\n", rsp[2]);

	delay(sp, 1000);
	watch_clear(sp->wp);
	for (i = 0; i < 6; ++i) {
		free(sp->tmcuart[i]);
		sp->tmcuart[i] = NULL;
	}
}

static void
test_gpio(sim_t *sp)
{
	Vconan *tb = sp->tb;
	int i;

	watch_add(sp->wp, "^gpio", "io", NULL, FORM_BIN, WF_ALL);
	watch_add(sp->wp, "gpio.state", "state", NULL, FORM_BIN, WF_ALL);
	watch_add(sp->wp, "u_command.msg_state", "msg_state", NULL, FORM_DEC, WF_ALL);

	/* directly set gpio */
	uart_send_vlq_and_wait(sp, 3, CMD_SET_DIGITAL_OUT, 2, 1);
	delay(sp, 100);
	if (!(tb->conan__DOT__gpio & 4))
		fail("failed to set gpio 2\n");

	uart_send_vlq_and_wait(sp, 3, CMD_SET_DIGITAL_OUT, 2, 0);
	delay(sp, 100);
	if ((tb->conan__DOT__gpio & 4))
		fail("failed to reset gpio 2\n");

	/* set via update */
	uart_send_vlq_and_wait(sp, 3, CMD_UPDATE_DIGITAL_OUT, 2, 1);
	delay(sp, 100);
	if (!(tb->conan__DOT__gpio & 4))
		fail("failed to set gpio 2 via update\n");

	uart_send_vlq_and_wait(sp, 3, CMD_UPDATE_DIGITAL_OUT, 2, 0);
	delay(sp, 100);
	if ((tb->conan__DOT__gpio & 4))
		fail("failed to clr gpio 2 via update\n");

	/* CONFIGURE_GPIO, channel,  value, default_value, max_duration */
	uart_send_vlq_and_wait(sp, 5, CMD_CONFIG_DIGITAL_OUT, 2, 1, 1, 50000);
	delay(sp, 100);
	if (!(tb->conan__DOT__gpio & 4))
		fail("failed to set gpio 2 via configure\n");

	/* give it 100000 cycles to process the message */
	uint32_t sched = sp->cycle + 100000;
	printf("schedule for %d\n", sched);
	uart_send_vlq(sp, 4, CMD_SCHEDULE_DIGITAL_OUT, 2, (uint32_t)(sp->cycle + 100000), 0);
	delay(sp, 50000);
	/* see that it's not yet scheduled */
	if (sp->cycle > sched - 5000)
		fail("waited too long for test");
	if (!(tb->conan__DOT__gpio & 4))
		fail("digital out reset prematurely\n");
	delay(sp, 50001);
	if (sp->cycle < sched)
		fail("waited not long enough");
	if ((tb->conan__DOT__gpio & 4))
		fail("digital out not cleared on time\n");

	/* wait until max duration (+one cycle) is over. pwm should be set to default */
	delay(sp, 152000 - (sp->cycle - sched));
	printf("test it's the default of 1\n");
	if (!(tb->conan__DOT__gpio & 4))
		fail("digital out now reset by duration\n");
}

static uint32_t
dro_send(sim_t *sp, uint32_t val, int bits, uint32_t idle)
{
	Vconan *tb = sp->tb;
	int setuptime = idle / 40;
	int bittime = idle / 20;
	int i;
	uint32_t starttime = 0;

	/* discard unused upper bits */
	val <<= 32 - bits;

	tb->chain_out_out2 = 1;	/* clk */
	tb->chain_out_out1 = 1; /* do */

	delay(sp, idle);

	for (i = 0; i < bits; ++i) {
		tb->chain_out_out1 = !!(val & (1 << 31));
		delay(sp, setuptime);
		tb->chain_out_out2 = 0;
		if (starttime == 0)
			starttime = sp->cycle;
		delay(sp, bittime);
		tb->chain_out_out2 = 1;
		delay(sp, bittime);
		val <<= 1;
	}

	delay(sp, idle);

	return starttime + HZ / 1000000 + 4;
}

static void
test_dro(sim_t *sp)
{
	Vconan *tb = sp->tb;
	int i;
	uint32_t rsp[5];
	uint32_t starttime;
	uint32_t idle = HZ / 1000; /* 1ms */

	tb->chain_out_out2 = 1;	/* clk */

	watch_add(sp->wp, "u_dro.state", "state", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_dro.dr_state", "dr_state", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_dro.dclk", "dclk", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_dro.ddo", "ddo", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "chain_out_out2", "dclk_in", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "chain_out_out1", "ddo_in", NULL, FORM_DEC, WF_ALL);

	uart_send_vlq_and_wait(sp, 3, CMD_CONFIG_DRO, 0, idle);

	starttime = dro_send(sp, 0x000000, 24, idle);

	wait_for_uart_vlq(sp, 5, rsp);
	if (rsp[0] != RSP_DRO_DATA)
		fail("read dro data\n");
	if (rsp[1] != 0)
		fail("dro data bad channel %d\n", rsp[1]);
	if (rsp[2] != starttime)
		fail("dro data bad starttime %d != %d\n", rsp[2], starttime);
	if (rsp[3] != 0x000000)
		fail("dro data bad data %x\n", rsp[3]);
	if (rsp[4] != 24)
		fail("dro data bad bits %x\n", rsp[4]);

	starttime = dro_send(sp, 0x349876, 24, idle);

	wait_for_uart_vlq(sp, 5, rsp);
	if (rsp[0] != RSP_DRO_DATA)
		fail("read dro data\n");
	if (rsp[1] != 0)
		fail("dro data bad channel %d\n", rsp[1]);
	if (rsp[2] != starttime)
		fail("dro data bad starttime %d != %d\n", rsp[2], starttime);
	if (rsp[3] != 0x349876)
		fail("dro data bad data %x\n", rsp[3]);
	if (rsp[4] != 24)
		fail("dro data bad bits %x\n", rsp[4]);

	uart_send_vlq_and_wait(sp, 3, CMD_CONFIG_DRO, 0, 0);

	watch_clear(sp->wp);
}

static void
as5311_tick(sim_t *sp)
{
	int i;
	as5311_t *as;

	for (i = 0; i < NAS5311; ++i) {
		as = sp->as5311[i];

		if (as == NULL)
			continue;

		if (as->state == AS_IDLE && *as->cs == 0) {
			if (*as->clk == 0) {
				/* magnet data */
				as->data = (as->magnet++ << 6) | 0x25;
				as->state = AS_CLK_LO;
			} else {
				/* sensor data */
				as->data = (as->sensor++ << 6) | 0x25;
				as->state = AS_CLK_HI;
			}
			as->cnt = 18;
		} else if (as->state == AS_CLK_HI && *as->clk == 0) {
			as->state = AS_CLK_LO;
		} else if (as->state == AS_CLK_LO && *as->clk == 1) {
			*as->dout = (as->data >> 17) & 1;
			as->data <<= 1;
			if (--as->cnt == 0)
				as->state = AS_WAIT_CS;
			else
				as->state = AS_CLK_HI;
		} else if (as->state == AS_WAIT_CS && *as->cs == 1) {
			as->state = AS_IDLE;
		} else if (as->state != AS_IDLE && *as->cs == 1) {
			fail("as5311 bad timing at cnt=%d\n", as->cnt);
		}
	}
}

static void
test_as5311(sim_t *sp)
{
	Vconan *tb = sp->tb;
	int i;
	uint32_t rsp[5];
	int vm = 0x321;
	int vs = 0x811;
	as5311_t as = { 0 };

	as.magnet = vm;
	as.sensor = vs;
	as.clk = &tb->exp1_1;
	as.cs = &tb->exp1_2;
	as.dout = &tb->exp1_3;

	sp->as5311[0] = &as;

	watch_add(sp->wp, "exp1_1$", "clk", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "exp1_2$", "cs", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "exp1_3$", "dout", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "as_state", "as_state", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_as5311.data$", "data", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_as5311.data_valid", "valid", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_as5311.state", "state", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_command.msg_state$", "msg_state", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_as5311.data_pending", "d_pend", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_as5311.mag_pending", "m_pend", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_as5311.next_data", "n_data", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_as5311.next_mag", "n_mag", NULL, FORM_DEC, WF_ALL);

	/* channel, divider, data interval, mag interval */
	uart_send_vlq_and_wait(sp, 5, CMD_CONFIG_AS5311, 0, 10, 100000, 300000);

	/* rsp: RSP_AS5311_DATA, channel, starttime, data, type (1=data, 0=mag) */
	for (i = 0; i < 5; ++i) {
		wait_for_uart_vlq(sp, 5, rsp);

		if (rsp[0] != RSP_AS5311_DATA)
			fail("read as5311 data\n");

		printf("channel %d starttime %d data %x/%x type %d\n",
			rsp[1], rsp[2], rsp[3] >> 6, rsp[3] & 0x3f, rsp[4]);

		switch (i) {
		case 0:
		case 1:
		case 2:
		case 4:
			if (rsp[4] != 1)
				fail("as5311 bad type\n");
			if ((rsp[3] >> 6) != vs++)
				fail("as5311 bad sensor data\n");
			break;
		case 3:
			if (rsp[4] != 0)
				fail("as5311 bad type\n");
			if ((rsp[3] >> 6) != vm++)
				fail("as5311 bad magnet data\n");
			break;
		}
		if ((rsp[3] & 0x3f) != 0x25)
			fail("as5311 bad status data\n");
	}

	/* disable: channel, divider, data interval, mag interval */
	uart_send_vlq_and_wait(sp, 5, CMD_CONFIG_AS5311, 0, 0, 0, 0);

	sp->as5311[0] = NULL;
}

static void
sd_tick(sim_t *sp)
{
	sd_t *sd = sp->sd;

	if (sd == NULL)
		return;

	if (sd->last_clk == *sd->clk)
		return;
	sd->last_clk = *sd->clk;

	if (sd->cmd_state == SDC_IDLE && *sd->clk == 1 && (*sd->cmd_en == 0 || *sd->cmd_out == 1))
		return;

	if (sd->cmd_state == SDC_IDLE && *sd->clk == 1) {
		sd->bitcnt = 0;
		memset(sd->cmd_rcv, 0, sizeof(sd->cmd_rcv));
		sd->cmd_state = SDC_RCV;
	}

	if (sd->cmd_state == SDC_RCV && *sd->clk == 1) {
		if (*sd->cmd_en == 0) {
			printf("sd: cmd recv buf so far: %02x%02x%02x%02x%02x%02x\n",
				sd->cmd_rcv[0], sd->cmd_rcv[1], sd->cmd_rcv[2],
				sd->cmd_rcv[3], sd->cmd_rcv[4], sd->cmd_rcv[5]);
			fail("sd: cmd line disabled during receive at bit %d\n", sd->bitcnt);
		}

		printf("sd: received cmd bit %d\n", *sd->cmd_out);
		if (*sd->cmd_out)
			sd->cmd_rcv[sd->bitcnt / 8] |= 1 << (7 - (sd->bitcnt & 7));

		if (++sd->bitcnt == 48) {
			printf("sd: received cmd %02x%02x%02x%02x%02x%02x\n",
				sd->cmd_rcv[0], sd->cmd_rcv[1], sd->cmd_rcv[2],
				sd->cmd_rcv[3], sd->cmd_rcv[4], sd->cmd_rcv[5]);

			if (sd->cmd_rcv[0] == 0x01 || sd->cmd_rcv[0] == 0x02) {
				if (sd->cmd_rcv[0] == 0x01) {
					sd->sndbits = 48;
					memcpy(sd->sndbuf, "\x31\x42\x53\x64\x75\x86", 6);
				} else {
					sd->sndbits = 136;
					memcpy(sd->sndbuf,
						"\x13\x24\x35\x46\x57\x68\x79\x8a\x9b\xac\xbd\xce\xdf\xe0\xf1\x02\x13",
						17);
				}
				sd->cmd_state = SDC_SND_DELAY;
				sd->bitcnt = 0;
			} else {
				sd->cmd_state = SDC_IDLE;
				sd->cmd_rcv_ready = 1;
			}
		}
	} else if (sd->cmd_state == SDC_SND_DELAY && *sd->clk == 0) {
		if (++sd->bitcnt == 8) {
			sd->cmd_state = SDC_SND;
			sd->bitcnt = 0;
		}
	} else if (sd->cmd_state == SDC_SND && *sd->clk == 0) {
		if (sd->bitcnt == sd->sndbits) {
			sd->cmd_state = SDC_IDLE;
			*sd->cmd_in = 1;
		} else {
#if 0
printf("send bit %d at cnt %d sndbuf[0]=%02x\n", !!(sd->sndbuf[sd->bitcnt / 8] & (1 << (7 - (sd->bitcnt & 7)))), sd->bitcnt, sd->sndbuf[0]);
#endif
			*sd->cmd_in = !!(sd->sndbuf[sd->bitcnt / 8] & (1 << (7 - (sd->bitcnt & 7))));
			++sd->bitcnt;
		}
	}
}

static void
test_sd(sim_t *sp)
{
	Vconan *tb = sp->tb;
	int i;
	int j;
	uint32_t rsp[100];
	sd_t sd = { 0 };

	sd.clk = &tb->esp_gpio2;
	sd.cmd_in = &tb->sd_cmd_in;
	sd.cmd_out = &tb->esp_rst;
	sd.dat0_in = &tb->sd_dat0_in;
	sd.dat1_in = &tb->sd_dat1_in;
	sd.dat2_in = &tb->sd_dat2_in;
	sd.dat3_in = &tb->sd_dat3_in;
	sd.dat0_out = &tb->esp_en;
	sd.dat1_out = &tb->esp_tx;
	sd.dat2_out = &tb->esp_rx;
	sd.dat3_out = &tb->esp_flash;
	sd.cmd_en = &tb->sd_cmd_en_v;
	sd.dat_en = &tb->sd_dat_en_v;

	*sd.cmd_in = 1;
	*sd.dat0_in = 1;
	*sd.dat1_in = 1;
	*sd.dat2_in = 1;
	*sd.dat3_in = 1;

#if 0
	watch_add(sp->wp, "u_command.msg_state$", "msg_state", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_command.str_len$", "str_len", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_command.msg_ready$", "msg_ready", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_command.unit$", "unit", NULL, FORM_DEC, WF_ALL);
#endif
#if 1
	watch_add(sp->wp, "\\.output_start$", "output_start", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "\\.output_elemcnt$", "output_elemcnt", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "unit_invol_req", "ivrq", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "unit_invol_grant", "ivgr", NULL, FORM_DEC, WF_ALL);
#endif

	watch_add(sp->wp, "u_sd.state$", "sd_state", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_sd.cmd_ready$", "cmdr", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_sd.cmd_len$", "cmd_len", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_sdc.cmdq_empty$", "cq_empty", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_sdc.cmdq_dout$", "cq_dout", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.cmdq_rd_en$", "cq_rd_en", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.cmdq_elemcnt$", "cq_elcnt", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.cq_curr_byte$", "cq_c_byte", NULL, FORM_HEX, WF_ALL);
#if 0
	watch_add(sp->wp, "u_sd_cmd_fifo.ram$", "ram", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.payload_data", "pl_data", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.payload_valid", "pl_valid", NULL, FORM_DEC, WF_ALL);
#endif
#if 1
	watch_add(sp->wp, "u_sdc.cmd_data", "c_data", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.cmd_valid", "c_valid", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.cmd_full", "c_full", NULL, FORM_HEX, WF_ALL);
#endif
#if 1
	watch_add(sp->wp, "u_sdc.output_data", "o_data", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.output_advance", "o_adv", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.output_elemcnt", "o_elcnt", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sd.out_cnt", "o_cnt", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_sd.param_data", "pd", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sd.param_write", "pw", NULL, FORM_HEX, WF_ALL);
#endif
	watch_add(sp->wp, "u_sdc.sd_clk", "clk", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.sd_cmd_en", "cmd_en", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.sd_cmd_r", "cmd_r", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.sd_cmd_in", "cmd_in", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.sd_dat_en", "dat_en", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.sd_dat._r", "dat_r", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.sd_dat._in", "dat_in", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.cq_state", "cq_state", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_sdc.clkdiv", "clkdiv", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.co_data", "co_data", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_sdc.co_wr_en", "co_wr_en", NULL, FORM_HEX, WF_ALL);
#if 0
	watch_add(sp->wp, "u_sdc.u_sd_out_fifo.ram", "ramout", NULL, FORM_HEX, WF_ALL);
#endif

	sp->sd = &sd;

	/* set clkdiv and enable clock */
	uart_send_vlq_and_wait(sp, -3, CMD_SD_QUEUE, 0, 3, 0x83, 0x21, 0x90);
	delay(sp, 100);
        if (tb->conan__DOT__u_command__DOT__u_sd__DOT__gensd__BRA__0__KET____DOT__u_sdc__DOT__clkdiv != 0x321)
		fail("failed to set clkdiv\n");
	/* TODO: test clock divider */
	/* disable clock, set clkdiv to 120 and enable clock */
	uart_send_vlq_and_wait(sp, -3, CMD_SD_QUEUE, 0, 4, 0xa0, 0x80, 120, 0x90);
	delay(sp, 100);
        if (tb->conan__DOT__u_command__DOT__u_sd__DOT__gensd__BRA__0__KET____DOT__u_sdc__DOT__clkdiv != 120)
		fail("failed to set clkdiv to 120\n");

	for (i = 0; i < 2; ++i) {
		uart_send_vlq_and_wait(sp, -3, CMD_SD_QUEUE, 0, 7, 0x10, 0x11, 0x22, 0x33, 0x44, 0x55, 0x67);

		while (sd.cmd_rcv_ready == 0)
			yield(sp);
		sd.cmd_rcv_ready = 1;

		if (memcmp(sd.cmd_rcv, "\x11\x22\x33\x44\x55\x67", 6) != 0)
			fail("failed to received correct command\n");
	}

	for (i = 0; i < 2; ++i) {
		/* send with 48 bit response */
		uart_send_vlq_and_wait(sp, -3, CMD_SD_QUEUE, 0, 7, 0x11, 0x01, 0x22, 0x33, 0x44, 0x55, 0x67);

		wait_for_uart_vlq(sp, -3, rsp);

		for (j = 0; j < 10; ++j)
			printf("%x ", rsp[j]);
		printf("\n");

		if (rsp[0] != RSP_SD_CMDQ)
			fail("read sd 48 bit cmd\n");
		if (rsp[1] != 0)
			fail("rsp len not 48 bit (signalled)\n");
		if (rsp[2] != 7)
			fail("bad rsp strlen\n");
		if (rsp[3] != 0x11)
			fail("bad rsp cmd\n");
		if (rsp[4] != 0x31 || rsp[5] != 0x42 || rsp[6] != 0x53 ||
		    rsp[7] != 0x64 || rsp[8] != 0x75 || rsp[9] != 0x86)
			fail("bad cmd data received\n");

		delay(sp, 1000);
	}

	for (i = 0; i < 2; ++i) {
		/* send with 136 bit response */
		uart_send_vlq_and_wait(sp, -3, CMD_SD_QUEUE, 0, 7, 0x12, 0x02, 0x22, 0x33, 0x44, 0x55, 0x67);

		delay(sp, 1000);
		wait_for_uart_vlq(sp, -3, rsp);

		for (j = 0; j < 20; ++j)
			printf("%x ", rsp[j]);
		printf("\n");

		if (rsp[0] != RSP_SD_CMDQ)
			fail("read sd 48 bit cmd\n");
		if (rsp[1] != 0)
			fail("rsp len not 48 bit (signalled)\n");
		if (rsp[2] != 18)
			fail("bad rsp strlen\n");
		if (rsp[3] != 0x12)
			fail("bad rsp cmd\n");
		if (rsp[4] != 0x13 || rsp[5] != 0x24 || rsp[6] != 0x35 ||
		    rsp[7] != 0x46 || rsp[8] != 0x57 || rsp[9] != 0x68 ||
		    rsp[10] != 0x79 || rsp[11] != 0x8a || rsp[12] != 0x9b ||
		    rsp[13] != 0xac || rsp[14] != 0xbd || rsp[15] != 0xce ||
		    rsp[16] != 0xdf || rsp[17] != 0xe0 || rsp[18] != 0xf1 ||
		    rsp[19] != 0x02 || rsp[20] != 0x13)
			fail("bad cmd data received (2)\n");

		delay(sp, 1000);
	}

	for (i = 0; i < 2; ++i) {
		uart_send_vlq_and_wait(sp, -3, CMD_SD_QUEUE, 0, 7, 0x11, 0x11, 0x22, 0x33, 0x44, 0x55, 0x67);

		delay(sp, 1000);
		wait_for_uart_vlq(sp, -3, rsp);

		for (j = 0; j < 4; ++j)
			printf("%x ", rsp[j]);
		printf("\n");

		if (rsp[0] != RSP_SD_CMDQ)
			fail("read sd timeout cmd\n");
		if (rsp[1] != 0)
			fail("rsp len not 48 bit (signalled)\n");
		if (rsp[2] != 1)
			fail("bad rsp strlen\n");
		if (rsp[3] != 0x1f)
			fail("bad rsp cmd (not timeout)\n");
	}

	sp->sd = NULL;
}

static void
test(sim_t *sp)
{
	delay(sp, 1);	/* pass back control after initialization */

	test_time(sp);	/* always needed as time sync */
	test_version(sp);
#if 0
	test_sd(sp);
#endif
	test_pwm(sp);
#if 0
	test_tmcuart(sp);
	test_gpio(sp);
	test_dro(sp);
	test_as5311(sp);
	/* must be last, as it ends with a shutdown */
	test_stepper(sp);
#endif

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
