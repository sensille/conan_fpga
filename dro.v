`timescale 1ns / 1ps
`default_nettype none

module dro #(
	parameter HZ = 0,
	parameter CMD_BITS = 0,
	parameter NDRO = 0,
	parameter CMD_CONFIG_DRO = 0,
	parameter RSP_DRO_DATA = 0
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

	input wire [NDRO-1:0] dro_clk,
	input wire [NDRO-1:0] dro_do,

	output wire [31:0] daq_data,
	output reg daq_end,
	output reg daq_valid = 0,
	output reg daq_req = 0,
	input wire daq_grant,

	output wire [15:0] debug,

	input wire shutdown	/* not used */
);

/*
	CMD_CONFIG_DRO in <channel> <timeout>
	CMD_DRO_READ in <channel> <clock> <val1>
	timeout 0 means disable
*/

/*
 * we always set daq_data and param_data at once,
 * so both can be merged into one by synthesis
 */
assign daq_data = param_data[31:0];

localparam NDRO_BITS = $clog2(NDRO);
reg [NDRO_BITS-1:0] channel = 0;

localparam TIMEOUT_BITS = $clog2(HZ);	/* max timeout 1s */
reg [TIMEOUT_BITS-1:0] timeout [NDRO];
reg [TIMEOUT_BITS-1:0] timeout_cnt [NDRO];
reg [NDRO-1:0] enabled = 0;

/* just keep asserted, we'll read one arg per clock */
assign arg_advance = 1;

reg [31:0] data [NDRO];
reg [4:0] bits [NDRO];
reg [31:0] prev_data [NDRO];
reg [4:0] prev_bits [NDRO];
reg [31:0] starttime [NDRO];
reg [NDRO-1:0] data_valid = 0;
reg [NDRO-1:0] data_ack = 0;

/* intermediate sync stage */
reg [NDRO-1:0] dro_clk1 = 0;
reg [NDRO-1:0] dro_do1 = 0;
reg [NDRO-1:0] dro_clk2 = 0;
reg [NDRO-1:0] dro_do2 = 0;
/* final synced signals */
reg [NDRO-1:0] dclk = 0;
reg [NDRO-1:0] ddo = 0;
/* debounce regs */
localparam DEBOUNCE_CNT = HZ / 1000000;	/* 1us */
localparam DEBOUNCE_BITS = $clog2(DEBOUNCE_CNT + 1);
reg [DEBOUNCE_BITS-1:0] clk_deb_cnt [NDRO];
reg [DEBOUNCE_BITS-1:0] do_deb_cnt [NDRO];

localparam DAQT_DRO_DATA	= 48;

localparam PS_IDLE		= 0;
localparam PS_CONFIG_DRO_1	= 1;
localparam PS_CONFIG_DRO_2	= 2;
localparam PS_WAIT_GRANT	= 3;
localparam PS_DRO_DATA_1	= 4;
localparam PS_DRO_DATA_2	= 5;
localparam PS_DRO_DATA_3	= 6;
localparam PS_DRO_DATA_4	= 7;
localparam PS_DRO_DATA_5	= 8;
localparam PS_DRO_DAQ_1		= 9;
localparam PS_DRO_DAQ_2		= 10;
localparam PS_DRO_DAQ_3		= 11;
localparam PS_MAX		= 11;

localparam PS_BITS= $clog2(PS_MAX + 1);
reg [PS_BITS-1:0] state = 0;

localparam DR_IDLE		= 0;
localparam DR_CLK_LO		= 1;
localparam DR_CLK_HI		= 2;
localparam DR_WAIT_HI		= 3;
localparam DR_MAX		= 3;

localparam DR_BITS= $clog2(DR_MAX + 1);
reg [DR_BITS-1:0] dr_state [NDRO];
reg [NDRO-1:0] use_daq = 0;

integer i;
initial begin
	for (i = 0; i < NDRO; i = i + 1) begin
		data[i] = 0;
		prev_data[i] = 32'hffffffff;
		prev_bits[i] = 0;
		starttime[i] = 0;
		clk_deb_cnt[i] = 0;
		do_deb_cnt[i] = 0;
		dr_state[i] = DR_IDLE;
		timeout[i] = 0;
		timeout_cnt[i] = 0;
		bits[i] = 0;
	end

end

/* receive state machine */
always @(posedge clk) begin
	/*
	 * sync to our domain
	 */
	dro_clk1 <= dro_clk;
	dro_do1 <= dro_do;
	dro_clk2 <= dro_clk1;
	dro_do2 <= dro_do1;

	/*
	 * debounce
	 */
	for (i = 0; i < NDRO; i = i + 1) begin
		if (dclk[i] != dro_clk2[i]) begin
			if (clk_deb_cnt[i] == 0)
				clk_deb_cnt[i] <= DEBOUNCE_CNT;
			else if (clk_deb_cnt[i] == 1)
				dclk[i] <= dro_clk2[i];
			else
				clk_deb_cnt[i] <= clk_deb_cnt[i] - 1;
		end else begin
			clk_deb_cnt[i] <= 0;
		end
		if (ddo[i] != dro_do2[i]) begin
			if (do_deb_cnt[i] == 0)
				do_deb_cnt[i] <= DEBOUNCE_CNT;
			else if (do_deb_cnt[i] == 1)
				ddo[i] <= dro_do2[i];
			else
				do_deb_cnt[i] <= do_deb_cnt[i] - 1;
		end else begin
			do_deb_cnt[i] <= 0;
		end
	end

	/*
	 * parser
	 */
	for (i = 0; i < NDRO; i = i + 1) begin
		if (timeout_cnt[i])
			timeout_cnt[i] <= timeout_cnt[i] - 1;
		if (data_valid[i] && data_ack[i])
			data_valid[i] <= 0;
		if (!enabled[i]) begin
			dr_state[i] <= DR_IDLE;
			data_valid[i] <= 0;
			timeout_cnt[i] <= 0;
		end else if (dr_state[i] == DR_IDLE && dclk[i] == 0) begin
			data[i] <= 0;
			bits[i] <= 0;
			data_valid[i] <= 0;
			dr_state[i] <= DR_CLK_LO;
			starttime[i] <= systime;
			timeout_cnt[i] <= timeout[i];
		end else if (dr_state[i] == DR_CLK_LO && dclk[i] == 1) begin
			data[i] <= { data[i][30:0], ddo[i] };
			bits[i] <= bits[i] + 1'b1;
			timeout_cnt[i] <= timeout[i];
			dr_state[i] <= DR_CLK_HI;
		end else if (dr_state[i] == DR_CLK_HI && dclk[i] == 0) begin
			timeout_cnt[i] <= timeout[i];
			dr_state[i] <= DR_CLK_LO;
		end else if (dr_state[i] == DR_WAIT_HI && dclk[i] == 1) begin
			dr_state[i] <= DR_IDLE;
		end else if (timeout_cnt[i] == 1 && dclk[i] == 0) begin
			dr_state[i] <= DR_WAIT_HI;
			timeout_cnt[i] <= 0;
		end else if (timeout_cnt[i] == 1 && dclk[i] == 1) begin
			if (data[i] != prev_data[i] || bits[i] != prev_bits[i] || use_daq[i])
				data_valid[i] <= 1;
			prev_data[i] <= data[i];
			prev_bits[i] <= bits[i];
			dr_state[i] <= DR_IDLE;
			timeout_cnt[i] <= 0;
		end
	end
end

always @(posedge clk) begin
	if (cmd_done)
		cmd_done <= 0;
	data_ack <= 0;
	daq_end <= 0;
	daq_valid <= 0;

	if (state == PS_IDLE && cmd_ready) begin
		// common to all cmds
		channel <= arg_data[NDRO_BITS-1:0];
		if (cmd == CMD_CONFIG_DRO) begin
			state <= PS_CONFIG_DRO_1;
		end else begin
			cmd_done <= 1;
		end
	end else if (state == PS_CONFIG_DRO_1) begin
		timeout[channel] <= arg_data;
		if (arg_data == 0)
			enabled[channel] <= 0;
		else
			enabled[channel] <= 1;
		state <= PS_CONFIG_DRO_2;
	end else if (state == PS_CONFIG_DRO_2) begin
		use_daq[channel] <= arg_data;
		cmd_done <= 1;
		state <= PS_IDLE;
	end else if (state == PS_IDLE && data_valid) begin
		if (use_daq & data_valid)
			daq_req <= 1;
		else
			invol_req <= 1;
		state <= PS_WAIT_GRANT;
	end else if (state == PS_WAIT_GRANT && (invol_grant || daq_grant)) begin
		invol_req <= 0;
		daq_req <= 0;
		for (i = 0; i < NDRO; i = i + 1) begin
			if (data_valid[i]) begin
				channel <= i;
				if (use_daq[i] & daq_grant)
					state <= PS_DRO_DAQ_1;
				else if (~use_daq[i] & invol_grant)
					state <= PS_DRO_DATA_1;
				i = NDRO;
			end
		end
	end else if (state == PS_DRO_DATA_1) begin
		param_data <= channel;
		param_write <= 1;
		state <= PS_DRO_DATA_2;
	end else if (state == PS_DRO_DATA_2) begin
		param_data <= starttime[channel];
		state <= PS_DRO_DATA_3;
	end else if (state == PS_DRO_DATA_3) begin
		param_data <= data[channel];
		state <= PS_DRO_DATA_4;
	end else if (state == PS_DRO_DATA_4) begin
		param_data <= bits[channel];
		state <= PS_DRO_DATA_5;
		data_ack[channel] <= 1;
	end else if (state == PS_DRO_DATA_5) begin
		cmd_done <= 1;
		param_write <= 0;
		param_data <= RSP_DRO_DATA;
		state <= PS_IDLE;
	end else if (state == PS_DRO_DAQ_1) begin
		param_data[31:24] <= DAQT_DRO_DATA;
		param_data[23:16] <= channel;
		param_data[15:8] <= bits[channel];
		daq_valid <= 1;
		state <= PS_DRO_DAQ_2;
	end else if (state == PS_DRO_DAQ_2) begin
		param_data[31:0] <= data[channel];
		daq_valid <= 1;
		data_ack[channel] <= 1;
		state <= PS_DRO_DAQ_3;
	end else if (state == PS_DRO_DAQ_3) begin
		param_data <= starttime[channel];
		daq_valid <= 1;
		daq_end <= 1;
		state <= PS_IDLE;
	end
end

assign debug[0] = data_valid[0];
assign debug[1] = data_ack[0];
assign debug[2] = dr_state[0];
assign debug[4:3] = dr_state[0];
assign debug[7:5] = state;
assign debug[8] = data[0][7:0];
assign debug[15:9] = 0;

endmodule
