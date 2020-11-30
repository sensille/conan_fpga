#include <stdlib.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <pcre.h>
#include <arpa/inet.h>

#include "Vtb_daq.h"
#include "verilated.h"
#include "vsyms.h"

static int color_disabled = 0;

#define HZ 48000000

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
	Vtb_daq		*tb;
	uint64_t	last_cycle;
} watch_t;

typedef struct {
	Vtb_daq		*tb;
	uint64_t	last_change;
	watch_t		*wp;
	uint64_t	cycle;
	jmp_buf		main_jb;
	jmp_buf		test_jb;
	vluint8_t	*signal8p;
	vluint8_t	signal8v;
	uint64_t	delay_until;
} sim_t;

static void fail(const char *msg, ...);

watch_t *
watch_init(Vtb_daq *tb)
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
	Vtb_daq *tb = wp->tb;
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

/*
 * main tick loop
 */
static void test(sim_t *sp);
static sim_t *
init(Vtb_daq *tb)
{
	sim_t *sp = (sim_t *)calloc(1, sizeof(*sp));
	uint64_t d = HZ / 250000;	/* uart divider */

	sp->tb = tb;
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
	Vtb_daq *tb = sp->tb;
	int want_dump = 0;

	sp->cycle = cycle;

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
check_packet(sim_t *sp, uint32_t *exp_data, int exp_len, int *seq)
{
	Vtb_daq *tb = sp->tb;
	int i;
	int j;
	int plen;
	uint32_t buf[500];
	uint8_t *p = (uint8_t *)buf;

	/* wait for packet */
	for (i = 0; i < 481000; ++i) {
		if (tb->eth_tx_en)
			break;
		yield(sp);
	}
	if (i == 481000)
		fail("no packet found\n");

	/* first receive all bits */
	for (i = 0; i < 1530; ++i) {
		uint8_t b = 0;

		if (tb->eth_tx_en == 0)
			break;
		for (j = 0; j < 4; ++j) {
			if (tb->eth_tx_en == 0)
				fail("tx_en deasserted mid-octet\n");
			b = (b >> 2) | (tb->eth_tx0 << 6) | (tb->eth_tx1 << 7);
			yield(sp);
		}
		p[i] = b;
	}
	if (i > 1522)
		fail("packet too long\n");
	plen = i;

	printf("PACKET: ");
	for (i = 0; i < plen; ++i)
		printf("%02x ", p[i]);
	printf("\n");

	/* check IPG */
	for (i = 0; i < 12 * 4; ++i) {
		if (tb->eth_tx0 || tb->eth_tx1)
			fail("activity during inter packet gap\n");
		yield(sp);
	}

	if (plen < 68)
		fail("frame too short\n");
	if ((plen % 4) != 0)
		fail("frame length not multiple of 4\n");

	if (ntohl(buf[0]) != 0x55555555)
		fail("bad preamble1\n");
	if (ntohl(buf[1]) != 0x555555d5)
		fail("bad preamble2\n");
	if (ntohl(buf[2]) != 0x66554433)
		fail("bad dst mac\n");
	if (ntohl(buf[3]) != 0x22111234)
		fail("bad dst/src mac\n");
	if (ntohl(buf[4]) != 0x56789abc)
		fail("bad src mac\n");
	if ((ntohl(buf[5]) & 0xffff0000) != 0x51390000)
		fail("ether type/fill %08x\n", ntohl(buf[5]));
	if ((ntohl(buf[5]) & 0xfff) != *seq)
		fail("seq %d != %d\n", ntohl(buf[5]) & 0xffff, *seq);
	(*seq)++;

	/* calculate crc */
	uint32_t crc = 0xffffffff;
	for (i = 8; i < plen - 4; ++i) {
		unsigned int byte, mask;

		byte = p[i];
		crc = crc ^ byte;
		for (j = 7; j >= 0; j--) {
			mask = -(crc & 1);
			crc = (crc >> 1) ^ (0xEDB88320 & mask);
		}
	}
	crc = ~crc;
	uint32_t recv_crc = buf[plen / 4 - 1]; /* no bswap */
	if (recv_crc != crc)
		fail("crc differ: recv %08x calc %08x\n", recv_crc, crc);

	if (plen != 28 + (exp_len < 11 ? 11 : exp_len) * 4)
		fail("bad packet length %d\n", plen);
	for (i = 0; i < exp_len; ++i) {
		if (ntohl(buf[6 + i]) != exp_data[i])
			fail("bad packet contents at %d\n", i);
	}
	for (i = exp_len; i < 11; ++i)
		if (buf[6 + i] != 0xffffffff)
			fail("bad packet filler at %i\n", i);
}

void
send_packet(sim_t *sp, int channel, uint32_t *buf, int len)
{
	Vtb_daq *tb = sp->tb;
	int i;
	int mask = 1 << channel;

	tb->daq_req |= mask;

	delay(sp, 1);
	for (i = 0; i < 1000; ++i) {
		if (tb->daq_grant & mask)
			break;
	}
	if (i == 1000)
		fail("grant not set\n");
	tb->daq_req &= ~mask;
	delay(sp, 1);

	tb->daq_valid = mask;
	for (i = 0; i < len; ++i) {
		tb->daq_data[channel] = buf[i];
		if (i == len - 1)
			tb->daq_end = mask;
		delay(sp, 1);
	}
	tb->daq_valid &= ~mask;
	tb->daq_end &= ~mask;
}

static void
test_daq(sim_t *sp)
{
	Vtb_daq *tb = sp->tb;
	int i;
	uint32_t starttime;
	uint32_t buf[1000];
	int len;
	int seq = 0;

	tb->daq_req = 0;

	watch_add(sp->wp, "eth_tx0", "tx0", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "eth_tx1", "tx1", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "eth_tx_en", "tx_en", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "^daq_req$", "req", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "^daq_grant$", "grant", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "^daq_valid$", "valid", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "^daq_end$", "end", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_daq.state$", "d_state", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "^daqo_len_ready$", "len_ready", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "^daqo_len$", "len", NULL, FORM_DEC, WF_ALL);
#if 0
	watch_add(sp->wp, "^daqo_data$", "data", NULL, FORM_HEX, WF_ALL);
#endif
	watch_add(sp->wp, "^daqo_len_rd_en$", "len_rd_en", NULL, FORM_BIN, WF_ALL);
	watch_add(sp->wp, "^daqo_data_rd_en$", "data_rd_en", NULL, FORM_BIN, WF_ALL);
	watch_add(sp->wp, "u_mac.state$", "m_state", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_mac.next_len$", "next_len", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_mac.data_len$", "data_len", NULL, FORM_DEC, WF_ALL);
#if 1
	watch_add(sp->wp, "u_mac.tx_state$", "t_state", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_mac.tx_dicnt$", "dicnt", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_mac.next_eof$", "n_eof", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_mac.tx_eof$", "t_eof", NULL, FORM_DEC, WF_ALL);
	watch_add(sp->wp, "u_mac.tx_data$", "t_data", NULL, FORM_HEX, WF_ALL);
#endif
	watch_add(sp->wp, "u_mac.next_data$", "n_data", NULL, FORM_HEX, WF_ALL);
#if 0
	watch_add(sp->wp, "data_fifo_data", "df_data", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "data_fifo_wr_en", "df_wr_en", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_daq.daq_data", "daq_data", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_daq.daq_valid", "daq_valid", NULL, FORM_BIN, WF_ALL);

	watch_add(sp->wp, "u_mac.crc$", "crc", NULL, FORM_HEX, WF_ALL);
	watch_add(sp->wp, "u_mac.crc_data", "crc_data", NULL, FORM_HEX, WF_ALL);
#endif

	delay(sp, 1000);

	tb->daq_data[0] = 0x11223344;
	tb->daq_data[1] = 0xaabbccdd;
	tb->daq_data[2] = 0x01234567;
	tb->daq_data[3] = 0x456789ab;
	tb->daq_data[4] = 0xffeeddcc;
	tb->daq_valid = 0;
	tb->daq_end = 0;

	delay(sp, 10);

	/*
	 * send a single packet and wait for it to be sent
	 * by timeout
	 */
	buf[0] = 0x18293a4b;
	buf[1] = 0x33aa1122;
	send_packet(sp, 1, buf, 2);

	/* wait for the packet */
	for (i = 0; i < 481000; ++i) {
		if (tb->eth_tx_en == 1)
			break;
		delay(sp, 1);
	}
	if (i < 470000)
		fail("packet too early at %d\n", i);
	if (i == 481000)
		fail("no packet\n");

	check_packet(sp, buf, 2, &seq);

	/*
	 * send two packets with a short gap, should arrive as one
	 */
	buf[0] = 0xa124512f;
	buf[1] = 0xda324a9c;
	buf[2] = 0xcc10afec;
	buf[3] = 0x99ab2100;

	send_packet(sp, 1, buf, 3);
	delay(sp, 5000);
	send_packet(sp, 1, buf + 3, 1);
	delay(sp, 1000);

	check_packet(sp, buf, 4, &seq);

	/*
	 * send enough data to fill 1 1/2 packets. the first one has to arrive
	 * full
	 * spread over some channels
	 */
	srand(0);
	for (i = 0; i < 500; ++i)
		buf[i] = rand();
	
	for (i = 0; i < 500;) {
		int len = rand() % 9 + 1;
		int channel = rand() % 5;

		if (len > 500 - i)
			len = 500 - i;

		send_packet(sp, channel, buf + i, len);
		delay(sp, 1 + rand() % 20);
		i += len;
	}

#if 0
	check_packet(sp, buf, 375, &seq);
	check_packet(sp, buf + 375, 500 - 375, &seq);
#else
	delay(sp, 50000);
#endif
	
#if 0
printf("sleep\n"); sleep(1000);
#endif
	watch_clear(sp->wp);
}

static void
test(sim_t *sp)
{
	delay(sp, 1);	/* pass back control after initialization */

	test_daq(sp);

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
	Vtb_daq *tb = new Vtb_daq;

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
		tb->clk = 1;
		tb->eval();
		tb->clk = 0;
		tb->eval();
		++cycle;
		step(sp, cycle);
		/* push in values changed by step() */
		tb->eval();
	}
	exit(0);
}