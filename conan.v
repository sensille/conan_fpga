`timescale 1ns / 1ps
`default_nettype none

module conan #(
	parameter BAUD = 250000,
	parameter LEN_BITS = 7,
	parameter LEN_FIFO_BITS = 5,
	parameter MOVE_COUNT = 1024,
	parameter NGPIO = 9,
	parameter NPWM = 12,
	parameter NSTEPDIR = 6,
	parameter NENDSTOP = 8,
	parameter NUART = 6,
	parameter NDRO = 2,
	parameter NAS5311 = 3,
	parameter NSD = 1,
	parameter NETHER = 1,
	parameter VERSION = 66,
	parameter PACKET_WAIT_FRAC = 10
) (
	input wire clk_50mhz,
	input wire clk_48mhz,
	output wire led1,
	output wire led2,
	output wire led3,
	output wire led4,
	output wire led5,
	output wire led6,
	output wire led7,
	output wire led8,

	output wire exp2_17,
	output wire exp2_16,
	output wire exp2_15,

	output wire exp2_14,
	output wire exp2_13,
	output wire exp2_12,
	output wire exp2_11,
	output wire exp2_10,
	output wire exp2_9,
	input wire exp2_8,
	input wire exp2_7,
	input wire exp2_6,
	input wire exp2_5,
	input wire exp2_4,
	input wire exp2_3,
	output wire exp2_2,
	output wire exp2_1,

	input wire exp1_17,
	input wire exp1_16,
	input wire exp1_15,
	input wire exp1_14,
	input wire exp1_13,
	input wire exp1_12,
	input wire exp1_11,
	input wire exp1_10,
	input wire exp1_9,
	input wire exp1_8,
	input wire exp1_7,
	input wire exp1_6,
	output wire exp1_5,
	output wire exp1_4,
	input wire exp1_3,
	output wire exp1_2,
	output wire exp1_1,

	input wire pmod1_1,
	input wire pmod1_2,
	output wire pmod1_3,
	output wire pmod1_4,

	output wire pmod2_1,
	input wire pmod2_2,
	output wire pmod2_3,
	inout wire pmod2_4,
`ifdef VERILATOR
	input wire eth_mdio_in1,
	output wire eth_mdio_en_v,
`endif
	output wire display1,
	input wire display2,
	output wire display3,
	input wire display4,
	input wire display5,
	input wire display6,
	input wire display7,
	input wire display8,
	input wire display9,
	input wire display10,
	input wire display11,
	input wire display12,
	input wire display13,

	input wire chain_in_in1,
	input wire chain_in_in2,
	input wire chain_in_in3,
	input wire chain_in_out1,
	input wire chain_in_out2,

	input wire chain_out_out1,
	input wire chain_out_out2,
	input wire chain_out_out3,
	input wire chain_out_in1,
	input wire chain_out_in2,

	inout reg esp_tx,
	inout reg esp_en,
	inout reg esp_rst,
	output reg esp_gpio2,
	inout reg esp_flash,
	inout reg esp_rx,
`ifdef VERILATOR
	input wire sd_cmd_in,
	input wire sd_dat0_in,
	input wire sd_dat1_in,
	input wire sd_dat2_in,
	input wire sd_dat3_in,
	output wire sd_cmd_en_v,
	output wire sd_dat_en_v,
`endif

	output wire drclk,
	output wire enn,
	inout wire uart1,
	inout wire uart2,
	inout wire uart3,
	inout wire uart4,
	inout wire uart5,
	inout wire uart6,
`ifdef VERILATOR
	input wire uart1_in,
	input wire uart2_in,
	input wire uart3_in,
	input wire uart4_in,
	input wire uart5_in,
	input wire uart6_in,
`endif
	output wire step1,
	output wire step2,
	output wire step3,
	output wire step4,
	output wire step5,
	output wire step6,
	output wire dir1,
	output wire dir2,
	output wire dir3,
	output wire dir4,
	output wire dir5,
	output wire dir6,
	input wire diag1,
	input wire diag2,
	input wire diag3,
	input wire diag4,
	input wire diag5,
	input wire diag6,
	input wire index1,
	input wire index2,
	input wire index3,
	input wire index4,
	input wire index5,
	input wire index6,

	input wire endstop1,
	input wire endstop2,
	input wire endstop3,
	input wire endstop4,
	input wire endstop5,
	input wire endstop6,
	input wire endstop7,
	input wire endstop8,

	output wire pwm1,
	output wire pwm2,
	output wire pwm3,
	output wire pwm4,
	output wire pwm5,
	output wire pwm6,
	output wire pwm7,
	output wire pwm8,
	output wire pwm9,
	output wire pwm10,
	output wire pwm11,
	output wire pwm12,

	output wire wdi,
	output wire wden,

	input wire fpga1,
	output wire fpga2,
	output wire fpga3,
	input wire fpga4,
	input wire fpga5,
	input wire fpga6
);

/*
 * PLL
 *
 * 48MHz in -> 24MHz system clock
 *          -> 12MHz stepper driver clock
 */
localparam HZ = 48000000;
wire clk;
`ifdef VERILATOR
assign clk = clk_48mhz;
reg _drclk = 0;
always @(posedge clk)
	_drclk <= !_drclk;
assign drclk = _drclk;
`else
`ifdef yosys
pll u_pll(
	.clkin(clk_48mhz),
	.clkout0(clk),		// 24 MHz
	.clkout1(drclk)		// 12 MHz
);
`else
assign clk = clk_48mhz;
reg [1:0] _drclk = 0;
always @(posedge clk)
	_drclk <= _drclk + 1;
assign drclk = _drclk[1];
`endif
`endif

/*
 * communication interface to MCU
 */
wire [7:0] msg_data;
wire msg_ready;
wire msg_rd_en;

/* max length of a packet MCU -> host */
/* address width of fifo */
/* size of receive ring (2^x) */
localparam RING_BITS = 9;
wire [LEN_BITS-1:0] send_fifo_data;
wire send_fifo_wr_en;
wire [7:0] send_ring_data;
wire send_ring_wr_en;
wire frame_reset = 1'b0;
wire frame_error;
assign fpga3 = frame_error;
framing #(
	.BAUD(BAUD),
	.RING_BITS(RING_BITS),
	.LEN_BITS(LEN_BITS),
	.LEN_FIFO_BITS(LEN_FIFO_BITS),
	.HZ(HZ)
) u_framing (
	.clk(clk),

	.rx(fpga1),
	.tx(fpga2),
	.cts(),

	/*
	 * receive side
	 */
	.msg_data(msg_data),
	.msg_ready(msg_ready),
	.msg_rd_en(msg_rd_en),

	/*
	 * send side
	 */
	.send_fifo_wr_en(send_fifo_wr_en),
	.send_fifo_data(send_fifo_data),
	.send_fifo_full(),

	/* ring buffer input */
	.send_ring_data(send_ring_data),
	.send_ring_wr_en(send_ring_wr_en),
	.send_ring_full(),

	/* reset */
	.error(frame_error),
	.clr(frame_reset)
/* verilator lint_on PINCONNECTEMPTY */
);

wire [52:0] cmd_debug;
wire [15:0] step_debug;

/*
 * on-board LEDs
 */
reg [63:0] systime = 0;
wire [63:0] systime_set;
wire systime_set_en;
reg [7:0] leds = 8'b00000001;
assign { led8, led7, led6, led5, led4, led3, led2, led1 } = leds;
always @(posedge clk) begin
	if (systime_set_en)
		systime <= systime_set;
	else
		systime <= systime + 1;
	if (systime[21:0] == 0) begin
		leds <= { leds[6:0], leds[7] };
	end
end

wire [NUART-1:0] uart_in;
wire [NUART-1:0] uart_out;
wire [NUART-1:0] uart_en;

wire [NGPIO-1:0] gpio;

wire [NDRO-1:0] dro_clk;
wire [NDRO-1:0] dro_do;

wire [NAS5311-1:0] as5311_clk;
wire [NAS5311-1:0] as5311_cs;
wire [NAS5311-1:0] as5311_do;

wire [NSD-1:0] sd_clk;
wire [NSD-1:0] sd_cmd_en;
wire [NSD-1:0] sd_dat_en;
`ifndef VERILATOR
reg [NSD-1:0] sd_cmd_in;
reg [NSD-1:0] sd_dat0_in;
reg [NSD-1:0] sd_dat1_in;
reg [NSD-1:0] sd_dat2_in;
reg [NSD-1:0] sd_dat3_in;
`endif
wire [NSD-1:0] sd_cmd_r;
wire [NSD-1:0] sd_dat0_r;
wire [NSD-1:0] sd_dat1_r;
wire [NSD-1:0] sd_dat2_r;
wire [NSD-1:0] sd_dat3_r;

wire [NETHER-1:0] eth_tx0;
wire [NETHER-1:0] eth_tx1;
wire [NETHER-1:0] eth_tx_en;
wire [NETHER-1:0] eth_rx_clk;
wire [NETHER-1:0] eth_mdc;
wire [NETHER-1:0] eth_mdio_in;
wire [NETHER-1:0] eth_mdio_out;
wire [NETHER-1:0] eth_mdio_en;

reg req_shutdown = 0;

command #(
	.HZ(HZ),
	.LEN_BITS(LEN_BITS),
	.LEN_FIFO_BITS(LEN_FIFO_BITS),
	.MOVE_COUNT(MOVE_COUNT),
	.NGPIO(NGPIO),
	.NPWM(NPWM),
	.NSTEPDIR(NSTEPDIR),
	.NENDSTOP(NENDSTOP),
	.NUART(NUART),
	.NDRO(NDRO),
	.NAS5311(NAS5311),
	.NSD(NSD),
	.NETHER(NETHER),
	.VERSION(VERSION),
	.PACKET_WAIT_FRAC(PACKET_WAIT_FRAC)
) u_command (
	.clk(clk),
	.systime(systime[31:0]),

	/*
	 * receive side
	 */
	.msg_data(msg_data),
	.msg_ready(msg_ready),
	.msg_rd_en(msg_rd_en),

	/*
	 * send side
	 */
	.send_fifo_wr_en(send_fifo_wr_en),
	.send_fifo_data(send_fifo_data),
	.send_fifo_full(),

	/* ring buffer input */
	.send_ring_data(send_ring_data),
	.send_ring_wr_en(send_ring_wr_en),
	.send_ring_full(),

	/* I/O */
	.gpio(gpio),
	.pwm({ pwm12, pwm11, pwm10, pwm9, pwm8, pwm7, pwm6, pwm5, pwm4, pwm3, pwm2, pwm1 }),
	.step({ step6, step5, step4, step3, step2, step1 }),
	.dir({ dir6, dir5, dir4, dir3, dir2, dir1 }),
	.endstop({ endstop8, endstop7, endstop6, endstop5, endstop4, endstop3, endstop2, endstop1 }),
	.uart_in(uart_in),
	.uart_out(uart_out),
	.uart_en(uart_en),

	.dro_clk(dro_clk),
	.dro_do(dro_do),

	.as5311_clk(as5311_clk),
	.as5311_cs(as5311_cs),
	.as5311_do(as5311_do),

	.sd_clk(sd_clk),
	.sd_cmd_en(sd_cmd_en),
	.sd_cmd_in(sd_cmd_in),
	.sd_cmd_r(sd_cmd_r),
	.sd_dat_en(sd_dat_en),
	.sd_dat0_in(sd_dat0_in),
	.sd_dat1_in(sd_dat1_in),
	.sd_dat2_in(sd_dat2_in),
	.sd_dat3_in(sd_dat3_in),
	.sd_dat0_r(sd_dat0_r),
	.sd_dat1_r(sd_dat1_r),
	.sd_dat2_r(sd_dat2_r),
	.sd_dat3_r(sd_dat3_r),

	.eth_tx0(eth_tx0),
	.eth_tx1(eth_tx1),
	.eth_tx_en(eth_tx_en),
	.eth_rx_clk(eth_rx_clk),
	.eth_mdc(eth_mdc),
	.eth_mdio_out(eth_mdio_out),
	.eth_mdio_in(eth_mdio_in),
	.eth_mdio_en(eth_mdio_en),

	.time_in(systime),
	.time_out(systime_set),
	.time_out_en(systime_set_en),
	.timesync_latch_in(fpga5),

	.req_shutdown(req_shutdown),
	.debug(cmd_debug),
	.step_debug(step_debug)
);

`ifdef VERILATOR
assign uart_in = { uart6_in, uart5_in, uart4_in, uart3_in, uart2_in, uart1_in };
`else
assign uart_in = { uart6, uart5, uart4, uart3, uart2, uart1 };
`endif
assign uart1 = uart_en[0] ? uart_out[0] : 1'bz;
assign uart2 = uart_en[1] ? uart_out[1] : 1'bz;
assign uart3 = uart_en[2] ? uart_out[2] : 1'bz;
assign uart4 = uart_en[3] ? uart_out[3] : 1'bz;
assign uart5 = uart_en[4] ? uart_out[4] : 1'bz;
assign uart6 = uart_en[5] ? uart_out[5] : 1'bz;

`ifdef VERILATOR
assign eth_mdio_in[0] = eth_mdio_in1;
assign eth_mdio_en_v = eth_mdio_en[0];
`else
assign eth_mdio_in[0] = pmod2_4;
`endif
assign pmod2_4 = eth_mdio_en[0] ? eth_mdio_out[0] : 1'bz;
assign pmod2_3 = eth_mdc[0];
assign eth_rx_clk[0] = pmod2_2;
assign pmod2_1 = eth_tx_en[0];
assign pmod1_4 = eth_tx0[0];
assign pmod1_3 = eth_tx1[0];

assign dro_do = { chain_out_out3, chain_out_out1 };
assign dro_clk = { chain_out_in1, chain_out_out2 };

assign exp1_1 = as5311_clk[0];
assign exp1_2 = as5311_cs[0];
assign as5311_do[0] = exp1_3;
assign exp1_4 = as5311_clk[1];
assign exp1_5 = as5311_cs[1];
assign as5311_do[1] = exp1_6;

assign exp2_1 = as5311_clk[2];
assign exp2_2 = as5311_cs[2];
assign as5311_do[2] = exp2_3;
/*
 * Stepper driver
 */
assign enn = gpio[0];

/*
 * watchdog
 */
assign wden = 1'b0;
assign wdi = 1'b0;

/*
 * LED matrix debugging
 */
wire [255:0] ldata;
led7219 u_led7219(
	.clk(clk),
	.data(ldata),
	.leds_out(exp2_17),
	.leds_cs(exp2_16),
	.leds_clk(exp2_15)
);

assign ldata[255] = exp2_14;
assign ldata[254] = exp2_13;
assign ldata[253] = exp2_12;
assign ldata[252] = exp2_11;
assign ldata[251] = exp2_10;
assign ldata[250] = exp2_9;
assign ldata[249] = exp2_8;
assign ldata[248] = exp2_7;
assign ldata[247] = exp2_6;
assign ldata[246] = exp2_5;
assign ldata[245] = exp2_4;
assign ldata[244] = exp2_3;
assign ldata[243] = exp2_2;
assign ldata[242] = exp2_1;

assign ldata[241] = exp1_17;
assign ldata[240] = exp1_16;
assign ldata[239] = exp1_15;
assign ldata[238] = exp1_14;
assign ldata[237] = exp1_13;
assign ldata[236] = exp1_12;
assign ldata[235] = exp1_11;
assign ldata[234] = exp1_10;
assign ldata[233] = exp1_9;
assign ldata[232] = exp1_8;
assign ldata[231] = exp1_7;
assign ldata[230] = exp1_6;
assign ldata[229] = exp1_5;
assign ldata[228] = exp1_4;
assign ldata[227] = exp1_3;
assign ldata[226] = exp1_2;
assign ldata[225] = exp1_1;

assign ldata[224] =  pmod1_1;
assign ldata[223] =  pmod1_2;
assign ldata[222] =  pmod1_3;
assign ldata[221] =  pmod1_4;

assign ldata[220] =  pmod2_1;
assign ldata[219] =  pmod2_2;
assign ldata[218] =  pmod2_3;
assign ldata[217] =  pmod2_4;

assign ldata[216] =  display1;
assign ldata[215] =  display2;
assign ldata[214] =  display3;
assign ldata[213] =  display4;
assign ldata[212] =  display5;
assign ldata[211] =  display6;
assign ldata[210] =  display7;
assign ldata[209] =  display8;
assign ldata[208] =  display9;
assign ldata[207] =  display10;
assign ldata[206] =  display11;
assign ldata[205] =  display12;
assign ldata[204] =  display13;

assign ldata[203] =  chain_in_in1;
assign ldata[202] =  chain_in_in2;
assign ldata[201] =  chain_in_in3;
assign ldata[200] =  chain_in_out1;
assign ldata[199] =  chain_in_out2;

assign ldata[198] =  chain_out_out1;
assign ldata[197] =  chain_out_out2;
assign ldata[196] =  chain_out_out3;
assign ldata[195] =  chain_out_in1;
assign ldata[194] =  chain_out_in2;

assign ldata[193] =  esp_tx;
assign ldata[192] =  esp_en;
assign ldata[191] =  esp_rst;
assign ldata[190] =  esp_gpio2;
assign ldata[189] =  esp_flash;
assign ldata[188] =  esp_rx;

assign ldata[187] = step1;
assign ldata[186] = dir1;
assign ldata[185] = uart1;
assign ldata[184] = index1;
assign ldata[183] = diag1;
assign ldata[182] = step2;
assign ldata[181] = dir2;
assign ldata[180] = uart2;
assign ldata[179] = index2;
assign ldata[178] = diag2;
assign ldata[177] = step3;
assign ldata[176] = dir3;
assign ldata[175] = uart3;
assign ldata[174] = index3;
assign ldata[173] = diag3;
assign ldata[172] = step4;
assign ldata[171] = dir4;
assign ldata[170] = uart4;
assign ldata[169] = index4;
assign ldata[168] = diag4;
assign ldata[167] = step5;
assign ldata[166] = dir5;
assign ldata[165] = uart5;
assign ldata[164] = index5;
assign ldata[163] = diag5;
assign ldata[162] = step6;
assign ldata[161] = dir6;
assign ldata[160] = uart6;
assign ldata[159] = index6;
assign ldata[158] = diag6;
assign ldata[157] = enn;
assign ldata[156] = drclk;

assign ldata[155] = endstop1;
assign ldata[154] = endstop2;
assign ldata[153] = endstop3;
assign ldata[152] = endstop4;
assign ldata[151] = endstop5;
assign ldata[150] = endstop6;
assign ldata[149] = endstop7;
assign ldata[148] = endstop8;

assign ldata[147] = pwm1;
assign ldata[146] = pwm2;
assign ldata[145] = pwm3;
assign ldata[144] = pwm4;
assign ldata[143] = pwm5;
assign ldata[142] = pwm6;
assign ldata[141] = pwm7;
assign ldata[140] = pwm8;
assign ldata[139] = pwm9;
assign ldata[138] = pwm10;
assign ldata[137] = pwm11;
assign ldata[136] = pwm12;

assign ldata[135] = wdi;
assign ldata[134] = wden;

assign ldata[133] = fpga1;
assign ldata[132] = fpga2;
assign ldata[131] = fpga3;
assign ldata[130] = fpga4;
assign ldata[129] = fpga5;
assign ldata[128] = fpga6;

assign ldata[127] = clk_48mhz;
assign ldata[126] = clk_50mhz;

assign ldata[125:117] = gpio;
assign ldata[116:64] = cmd_debug;

assign ldata[31:0] = systime[43:12];

/* step watcher */
wire [5:0] d_step = { step6, step5, step4, step3, step2, step1 };
reg [5:0] prev_d_step = 0;
reg [5:0] d_alert = 0;
reg [31:0] d_idle[6];
reg armed = 0;
integer i;
initial
	for (i = 0; i < 6; i = i + 1)
		d_idle[i] = 0;

always @(posedge clk) begin
	for (i = 0; i < 6; i = i + 1) begin
		d_alert[i] <= 0;
		if (d_idle[i] != HZ * 10) begin
			d_idle[i] <= d_idle[i] + 1;
		end else begin
			d_alert[i] <= 1;
			if (i == 5 && armed)
				req_shutdown <= 1;
		end
		if (prev_d_step[i] != d_step[i]) begin
			d_idle[i] <= 0;
		end
	end
	prev_d_step <= d_step;
	if (endstop5 == 0)
		armed <= 1;
end
assign ldata[37:32] = d_alert;
assign ldata[38] = armed;
assign ldata[39] = 0;
assign ldata[47:40] = d_idle[5][31:24];
assign ldata[63:48] = step_debug;

/*
 * sd card tri-state
 */
always @(*) begin: sdmux
	esp_gpio2 = sd_clk[0];
	esp_rst = sd_cmd_en[0] ? sd_cmd_r[0] : 1'bZ;
	esp_en = sd_dat_en[0] ? sd_dat0_r[0] : 1'bZ;
	esp_tx = sd_dat_en[0] ? sd_dat1_r[0] : 1'bZ;
	esp_rx = sd_dat_en[0] ? sd_dat2_r[0] : 1'bZ;
	esp_flash = sd_dat_en[0] ? sd_dat3_r[0] : 1'bZ;
`ifndef VERILATOR
	sd_cmd_in = esp_rst;
	sd_dat0_in = esp_en;
	sd_dat1_in = esp_tx;
	sd_dat2_in = esp_rx;
	sd_dat3_in = esp_flash;
`endif
end
`ifdef VERILATOR
assign sd_cmd_en_v = sd_cmd_en[0];
assign sd_dat_en_v = sd_dat_en[0];
`endif

/*
 * direct output for scope debugging
 */
`ifdef notanymore
assign esp_rx = fpga1;		/* rx */
assign esp_flash = fpga2;	/* tx */
assign esp_tx = fpga5;		/* timesync */
assign esp_gpio2 = pwm11;
assign exp2_9 = step6;
assign exp2_10 = dir6;
`endif

assign exp2_9 = esp_gpio2;
assign exp2_10 = esp_rst;
assign exp2_11 = esp_en;
assign exp2_12 = esp_tx;
assign exp2_13 = esp_rx;
assign exp2_14 = esp_flash;

assign display1 = fpga1;
assign display3 = fpga2;

endmodule
