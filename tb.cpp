#include <stdlib.h>
#include <setjmp.h>
#include "Vconan.h"
#include "verilated.h"
 
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
static uint32_t
parse_int(uint8_t **pp)
{
	uint8_t *p = *pp, c = *p++;
	uint32_t v = c & 0x7f;
	if ((c & 0x60) == 0x60)
		v |= -0x20;
	while (c & 0x80) {
		c = *p++;
		v = (v<<7) | (c & 0x7f);
	}
	*pp = p;

	return v;
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
		vluint8_t	framing_rx_ready;
		vluint8_t	framing_recv_state;
		vluint8_t	framing_msg_data;
		vluint8_t	framing_msg_ready;
		vluint8_t	framing_msg_rd_en;
		vluint8_t	command_unit_arg_ptr;
		vluint32_t	command_unit_arg_data;
		vluint8_t	command_unit_arg_advance;
		vluint8_t	command_unit_cmd;
		vluint8_t	command_unit_cmd_ready;
		vluint8_t	command_unit_cmd_done;
		vluint8_t	command_unit_param_write;
#if 0
		vluint8_t	command_unit_rsp[6];
#endif
		vluint8_t	command_unit_invol_req;
		vluint8_t	command_unit_invol_grant;
		vluint8_t	command_msg_state;
		vluint8_t	pwm1;
		vluint8_t	pwm2;
		vluint8_t	pwm3;
		vluint8_t	pwm4;
		vluint8_t	pwm5;
		vluint8_t	pwm6;
		vluint8_t	pwm7;
		vluint8_t	pwm8;
		vluint8_t	pwm9;
		vluint8_t	pwm10;
		vluint8_t	pwm11;
		vluint8_t	pwm12;
		vluint32_t	pwm_cycle_ticks[12];
		vluint32_t	pwm_on_ticks[12];
		vluint32_t	pwm_next_on_ticks[12];
		vluint32_t	pwm_next_time[12];
		vluint32_t	pwm_scheduled[12];
		vluint32_t	pwm_default_value[12];
		vluint32_t	pwm_max_duration[12];
		vluint32_t	pwm_duration[12];
		vluint32_t	pwm_cycle_cnt[12];
		vluint8_t	pwm_state;
		vluint8_t	pwm_channel;
} watchlist_t;
typedef struct {
	Vconan		*tb;
	uart_recv_t	*urp;
	uart_send_t	*usp;
	uint64_t	last_change;
	watchlist_t	w;
	uint64_t	cycle;
	jmp_buf		main_jb;
	jmp_buf		test_jb;
	vluint8_t	*signal8p;
	vluint8_t	signal8v;
	uint64_t	delay_until;
} sim_t;

void
watch(Vconan *tb, sim_t *sp, uint64_t cycle, int want_dump)
{
	watchlist_t old_w = sp->w;	/* make copy */

        sp->w.uart_received = tb->conan__DOT__u_framing__DOT__uart_u__DOT__received;
        sp->w.uart_transmit = tb->conan__DOT__u_framing__DOT__uart_u__DOT__transmit;
	sp->w.send_fifo_wr_en = tb->conan__DOT__u_framing__DOT__send_fifo_wr_en;
	sp->w.send_fifo_data = tb->conan__DOT__u_framing__DOT__send_fifo_data;
	sp->w.send_fifo_full = tb->conan__DOT__u_framing__DOT__send_fifo_full;
	sp->w.send_fifo_empty = tb->conan__DOT__u_framing__DOT__send_fifo_empty;
	sp->w.send_state = tb->conan__DOT__u_framing__DOT__send_state;
	sp->w.framing_rx_ready = tb->conan__DOT__u_framing__DOT__rx_ready;
	sp->w.framing_recv_state = tb->conan__DOT__u_framing__DOT__recv_state;
	sp->w.framing_msg_data = tb->conan__DOT__u_framing__DOT__msg_data;
	sp->w.framing_msg_ready = tb->conan__DOT__u_framing__DOT__msg_ready;
	sp->w.framing_msg_rd_en = tb->conan__DOT__u_framing__DOT__msg_rd_en;
	sp->w.command_unit_arg_ptr = tb->conan__DOT__u_command__DOT__unit_arg_ptr;
	sp->w.command_unit_arg_data = tb->conan__DOT__u_command__DOT__unit_arg_data;
	sp->w.command_unit_arg_advance = tb->conan__DOT__u_command__DOT__unit_arg_advance;
	sp->w.command_unit_cmd = tb->conan__DOT__u_command__DOT__unit_cmd;
	sp->w.command_unit_cmd_ready = tb->conan__DOT__u_command__DOT__unit_cmd_ready;
	sp->w.command_unit_cmd_done = tb->conan__DOT__u_command__DOT__unit_cmd_done;
	sp->w.command_unit_param_write = tb->conan__DOT__u_command__DOT__unit_param_write;
	sp->w.command_msg_state = tb->conan__DOT__u_command__DOT__msg_state;
	sp->w.pwm1 = tb->conan__DOT__pwm1;
	sp->w.pwm2 = tb->conan__DOT__pwm2;
	sp->w.pwm3 = tb->conan__DOT__pwm3;
	sp->w.pwm4 = tb->conan__DOT__pwm4;
	sp->w.pwm5 = tb->conan__DOT__pwm5;
	sp->w.pwm6 = tb->conan__DOT__pwm6;
	sp->w.pwm7 = tb->conan__DOT__pwm7;
	sp->w.pwm8 = tb->conan__DOT__pwm8;
	sp->w.pwm9 = tb->conan__DOT__pwm9;
	sp->w.pwm10 = tb->conan__DOT__pwm10;
	sp->w.pwm11 = tb->conan__DOT__pwm11;
	sp->w.pwm12 = tb->conan__DOT__pwm12;
	for (int i = 0; i < 12; ++i) {
		sp->w.pwm_cycle_ticks[i] = tb->conan__DOT__u_command__DOT__u_pwm__DOT__cycle_ticks[i];
		sp->w.pwm_on_ticks[i] = tb->conan__DOT__u_command__DOT__u_pwm__DOT__on_ticks[i];
		sp->w.pwm_next_on_ticks[i] = tb->conan__DOT__u_command__DOT__u_pwm__DOT__next_on_ticks[i];
		sp->w.pwm_next_time[i] = tb->conan__DOT__u_command__DOT__u_pwm__DOT__next_time[i];
		sp->w.pwm_scheduled[i] = tb->conan__DOT__u_command__DOT__u_pwm__DOT__scheduled[i];
		sp->w.pwm_default_value[i] = tb->conan__DOT__u_command__DOT__u_pwm__DOT__default_value[i];
		sp->w.pwm_max_duration[i] = tb->conan__DOT__u_command__DOT__u_pwm__DOT__max_duration[i];
		sp->w.pwm_duration[i] = tb->conan__DOT__u_command__DOT__u_pwm__DOT__duration[i];
		sp->w.pwm_cycle_cnt[i] = tb->conan__DOT__u_command__DOT__u_pwm__DOT__cycle_cnt[i];
	}
	sp->w.pwm_state = tb->conan__DOT__u_command__DOT__u_pwm__DOT__state;
	sp->w.pwm_channel = tb->conan__DOT__u_command__DOT__u_pwm__DOT__channel;
#if 0
	sp->w.command_unit_rsp = tb->conan__DOT__u_command__DOT__unit_rsp[6];
#endif
	sp->w.command_unit_invol_req = tb->conan__DOT__u_command__DOT__unit_invol_req;
	sp->w.command_unit_invol_grant = tb->conan__DOT__u_command__DOT__unit_invol_grant;
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
		printf("% 10d tx %d rx %d u_rx %d utx %d u_transmit %d u_tx_byte %02x u_rcvd %d u_rx_byte %02x u_is_rx %d u_is_tx %d u_rcv_err %d\n",
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
		printf("% 10d framing: frame_error %d rx_ready %d recv_state %d msg_data %02x msg_ready %d msg_rd_en %d\n", cycle,
			tb->fpga3,
			tb->conan__DOT__u_framing__DOT__rx_ready,
			tb->conan__DOT__u_framing__DOT__recv_state,
			tb->conan__DOT__u_framing__DOT__msg_data,
			tb->conan__DOT__u_framing__DOT__msg_ready,
			tb->conan__DOT__u_framing__DOT__msg_rd_en);
		printf("% 10d command_unit: arg_ptr %d arg_data %d(0x%x) arg_advance 0x%x cmd %d cmd_ready 0x%x cmd_done 0x%x param_write 0x%x invol_req 0x%x invol_grant 0x%x\n", cycle,

			tb->conan__DOT__u_command__DOT__unit_arg_ptr,
			tb->conan__DOT__u_command__DOT__unit_arg_data,
			tb->conan__DOT__u_command__DOT__unit_arg_data,
			tb->conan__DOT__u_command__DOT__unit_arg_advance,
			tb->conan__DOT__u_command__DOT__unit_cmd,
			tb->conan__DOT__u_command__DOT__unit_cmd_ready,
			tb->conan__DOT__u_command__DOT__unit_cmd_done,
			tb->conan__DOT__u_command__DOT__unit_param_write,
#if 0
			tb->conan__DOT__u_command__DOT__unit_rsp[6],
#endif
			tb->conan__DOT__u_command__DOT__unit_invol_req,
			tb->conan__DOT__u_command__DOT__unit_invol_grant);
		printf("% 10d command state %d args:", cycle,
			tb->conan__DOT__u_command__DOT__msg_state);
		for (int i = 0; i < 8; ++i)
			printf(" %d", tb->conan__DOT__u_command__DOT__args[i]);
		printf("\n");
		printf("% 10d pwm %d %d %d %d %d %d %d %d %d %d %d %d channel %d state %d\n", cycle,
			tb->conan__DOT__pwm1,
			tb->conan__DOT__pwm2,
			tb->conan__DOT__pwm3,
			tb->conan__DOT__pwm4,
			tb->conan__DOT__pwm5,
			tb->conan__DOT__pwm6,
			tb->conan__DOT__pwm7,
			tb->conan__DOT__pwm8,
			tb->conan__DOT__pwm9,
			tb->conan__DOT__pwm10,
			tb->conan__DOT__pwm11,
			tb->conan__DOT__pwm12,
			tb->conan__DOT__u_command__DOT__u_pwm__DOT__channel,
			tb->conan__DOT__u_command__DOT__u_pwm__DOT__state);
		for (int i = 0; i < 12; ++i) {
			printf("% 10d pwm%d cycle_ticks %d on_ticks %d next_on_ticks %d next_time %d scheduled %d default_value %d max_duration %d duration %d cycle_cnt %d\n",
				cycle, i,
				tb->conan__DOT__u_command__DOT__u_pwm__DOT__cycle_ticks[i],
				tb->conan__DOT__u_command__DOT__u_pwm__DOT__on_ticks[i],
				tb->conan__DOT__u_command__DOT__u_pwm__DOT__next_on_ticks[i],
				tb->conan__DOT__u_command__DOT__u_pwm__DOT__next_time[i],
				tb->conan__DOT__u_command__DOT__u_pwm__DOT__scheduled[i],
				tb->conan__DOT__u_command__DOT__u_pwm__DOT__default_value[i],
				tb->conan__DOT__u_command__DOT__u_pwm__DOT__max_duration[i],
				tb->conan__DOT__u_command__DOT__u_pwm__DOT__duration[i],
				tb->conan__DOT__u_command__DOT__u_pwm__DOT__cycle_cnt[i]);
		}
		fflush(stdout);
		sp->last_change = cycle;
	}

	if (cycle - sp->last_change > 10000) {
		printf("% 10d finish due to inactivity\n", cycle);
		fflush(stdout);
		exit(0);
	}
}

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

	watch(tb, sp, cycle, want_dump);
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

	/* send version request */
	uint32_t cmd[] = { 5, 0, 1000, 100, 0, 5000 };
	uart_send_vlq(usp, cmd, 6);
#if 0
	wait_for_uart_vlq(sp);
#endif
}

static void
test_pwm(sim_t *sp)
{
	uart_send_t *usp = sp->usp;
	uart_recv_t *urp = sp->urp;
	Vconan *tb = sp->tb;
	int i;
	uint64_t curr;

	/* CONFIGURE_PWM, channel,  cycle_ticks, on_ticks, default_value, max_duration */
	uint32_t cmd[] = { 5, 0, 1000, 100, 0, 5000 };
	uart_send_vlq(usp, cmd, 6);
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

#if 0
	test_version(sp);
printf("test 1 success\n"); sleep(3);
#endif

	/* TODO always sync time */

	test_pwm(sp);

	printf("test succeeded\n");
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
