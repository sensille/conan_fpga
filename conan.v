`timescale 1ns / 1ps
`default_nettype none

module conan
(
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

	input wire exp2_14,
	input wire exp2_13,
	input wire exp2_12,
	input wire exp2_11,
	input wire exp2_10,
	input wire exp2_9,
	input wire exp2_8,
	input wire exp2_7,
	input wire exp2_6,
	input wire exp2_5,
	input wire exp2_4,
	input wire exp2_3,
	input wire exp2_2,
	input wire exp2_1,

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
	input wire exp1_5,
	input wire exp1_4,
	input wire exp1_3,
	input wire exp1_2,
	input wire exp1_1,

	input wire pmod1_1,
	input wire pmod1_2,
	input wire pmod1_3,
	input wire pmod1_4,

	input wire pmod2_1,
	input wire pmod2_2,
	input wire pmod2_3,
	input wire pmod2_4,

	input wire display1,
	input wire display2,
	input wire display3,
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

	input wire esp_tx,
	input wire esp_en,
	input wire esp_rst,
	input wire esp_gpio2,
	output wire esp_flash,
	output wire esp_rx,

	output wire drclk,
	output wire enn,
	inout wire uart1,
	inout wire uart2,
	inout wire uart3,
	inout wire uart4,
	inout wire uart5,
	inout wire uart6,
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

	output reg wdi,
	output wire wden,

	input wire fpga1,
	output wire fpga2,
	input wire fpga3,
	input wire fpga4,
	input wire fpga5,
	input wire fpga6
);

wire clk; 
`ifdef VERILATOR
assign clk = clk_50mhz;
`else
pll u_pll(
	.clkin(clk_50mhz),
	.clkout0(clk)		// 20 MHz
);
`endif

wire [255:0] ldata;
led7219 u_led7219(
	.clk(clk),
	.data(ldata),
	.leds_out(exp2_17),
	.leds_cs(exp2_16),
	.leds_clk(exp2_15)
);

reg [63:0] clock = 0;
reg [7:0] leds = 8'b11111110;
assign { led8, led7, led6, led5, led4, led3, led2, led1 } = leds;
always @(posedge clk) begin
	clock <= clock + 1;
	if (clock[20:0] == 0) begin
		leds <= { leds[6:0], leds[7] };
	end
end

/*
 * TMC uart mux
 *
 * Idea: push every input null to output, after transition back to high
 *       push high for 4 clock, and then leave the rest to the pull-up
 *       Transmit every 0 that's not from the mcu itself to the mcu
 */
wire uart_in = fpga1;
reg uart_out = 1;
assign fpga2 = uart_out;
reg uart_tmc_out = 1;
reg uart_tmc_out_enable = 0;
wire uart_tmc_in;
reg [2:0] ustate = 0;
localparam US_IDLE = 0;
localparam US_TX = 1;
localparam US_PUSH1 = 2;
localparam US_PUSH2 = 3;
localparam US_PUSH3 = 4;
localparam US_PUSH4 = 5;


// just mirror config to all drivers
assign uart1 = uart_tmc_out_enable ? uart_tmc_out : 1'bz;
assign uart2 = uart_tmc_out_enable ? uart_tmc_out : 1'bz;
assign uart3 = uart_tmc_out_enable ? uart_tmc_out : 1'bz;
assign uart4 = uart_tmc_out_enable ? uart_tmc_out : 1'bz;
assign uart5 = uart_tmc_out_enable ? uart_tmc_out : 1'bz;
assign uart6 = uart_tmc_out_enable ? uart_tmc_out : 1'bz;

assign uart_tmc_in = uart1;

always @(posedge clk) begin
	if (ustate == US_IDLE) begin
		if (uart_in == 0) begin
			uart_tmc_out <= 1'b0;
			uart_tmc_out_enable <= 1'b1;
			uart_out <= 1'b1;
			ustate <= US_TX;
		end else begin
			uart_out <= uart_tmc_in;
		end
	end else if (ustate == US_TX) begin
		if (uart_in == 1) begin
			uart_tmc_out <= 1'b1;
			uart_tmc_out_enable <= 1'b1;
			ustate <= US_PUSH1;
		end else begin
			uart_tmc_out <= 1'b0;
			uart_tmc_out_enable <= 1'b1;
		end
	end else if (ustate == US_PUSH1) begin
		ustate <= US_PUSH2;
	end else if (ustate == US_PUSH2) begin
		ustate <= US_PUSH3;
	end else if (ustate == US_PUSH3) begin
		ustate <= US_PUSH4;
	end else if (ustate == US_PUSH4) begin
		uart_tmc_out <= 1'b1;
		uart_tmc_out_enable <= 1'b0;
		ustate <= US_IDLE;
	end
end
assign step1 = fpga3;
assign step2 = fpga3;
assign step3 = fpga3;
assign step4 = fpga3;
assign step5 = fpga3;
assign step6 = fpga3;
assign dir1 = fpga4;
assign dir2 = fpga4;
assign dir3 = fpga4;
assign dir4 = fpga4;
assign dir5 = fpga4;
assign dir6 = fpga4;

assign drclk = clock[0];
//assign drclk = 0;
assign enn = fpga5;

reg [20:0] pwm_clk = 0;
reg [7:0] pwm_thr = 0;
reg pwm_out = 0;
always @(posedge clk) begin
	pwm_clk <= pwm_clk + 1;
	if (pwm_clk[18:11] < pwm_thr)
		pwm_out <= 1'b0;
	else
		pwm_out <= 1'b1;
	if (pwm_clk[20:0] == 0)
		pwm_thr <= pwm_thr + 1;
end
assign pwm1 = !pwm_out;
assign pwm2 = pwm_out;
assign pwm3 = 1'b0;
assign pwm4 = 1'b0;
assign pwm5 = 1'b0;
assign pwm6 = 1'b1;
assign pwm7 = !pwm_out;
assign pwm8 = pwm_out;
assign pwm9 = 1'b0;
assign pwm10 = 1'b0;
assign pwm11 = 1'b0;
assign pwm12 = 1'b0;

assign wden = 1'b0;
always @(posedge clk) begin
	if (clock[32] == 0)
		wdi <= clock[20];
	else
		wdi <= 1'b0;
end

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

assign ldata[126:64] = 0;

assign ldata[63:0] = clock;

/* debug */
assign esp_rx = fpga1;
assign esp_flash = fpga3;

endmodule
