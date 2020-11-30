`timescale 1ns / 1ps
`default_nettype none

module as5311 #(
	parameter HZ = 0,
	parameter CMD_BITS = 0,
	parameter NAS5311 = 0,
	parameter CMD_CONFIG_AS5311 = 0,
	parameter RSP_AS5311_DATA = 0,
	parameter DAQT_AS5311_DAT = 0,
	parameter DAQT_AS5311_MAG = 0
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

	output reg [NAS5311-1:0] as5311_clk = { NAS5311 { 1'b1 }},
	output reg [NAS5311-1:0] as5311_cs = { NAS5311 { 1'b1 }},
	input wire [NAS5311-1:0] as5311_do,

	output wire [31:0] daq_data,
	output reg daq_end,
	output reg daq_valid = 0,
	output reg daq_req = 0,
	input wire daq_grant,

	output wire [15:0] debug,

	input wire shutdown	/* not used */
);

/*
	CMD_CONFIG_AS5311 in <channel> <timeout>
	CMD_AS5311_READ in <channel> <clock> <val1>
	timeout 0 means disable
*/

/*
 * we always set daq_data and param_data at once,
 * so both can be merged into one by synthesis
 */
assign daq_data = param_data[31:0];

localparam NAS5311_BITS = $clog2(NAS5311);
reg [NAS5311_BITS-1:0] channel = 0;

reg [NAS5311-1:0] enabled = 0;

/* just keep asserted, we'll read one arg per clock */
assign arg_advance = 1;

localparam BITSIZE		= 18;
localparam BITSIZE_BITS = $clog2(BITSIZE);

reg [BITSIZE-1:0] data [NAS5311];
reg [NAS5311-1:0] ftype = 0;
reg [BITSIZE-1:0] prev_data [NAS5311];
reg [BITSIZE-1:0] prev_magnet [NAS5311];
reg [BITSIZE_BITS-1:0] bitcnt [NAS5311];
reg [31:0] starttime [NAS5311];
reg [31:0] next_data [NAS5311];
reg [31:0] next_mag [NAS5311];
reg [31:0] data_interval [NAS5311];
reg [31:0] mag_interval [NAS5311];
reg [11:0] freq_divider [NAS5311];
reg [11:0] div [NAS5311];
reg [NAS5311-1:0] data_valid = 0;
reg [NAS5311-1:0] data_ack = 0;
reg [NAS5311-1:0] data_pending = 0;
reg [NAS5311-1:0] mag_pending = 0;

localparam PS_IDLE		= 0;
localparam PS_CONFIG_AS5311_1	= 1;
localparam PS_CONFIG_AS5311_2	= 2;
localparam PS_CONFIG_AS5311_3	= 3;
localparam PS_CONFIG_AS5311_4	= 4;
localparam PS_WAIT_GRANT	= 5;
localparam PS_AS5311_DATA_1	= 6;
localparam PS_AS5311_DATA_2	= 7;
localparam PS_AS5311_DATA_3	= 8;
localparam PS_AS5311_DATA_4	= 9;
localparam PS_AS5311_DATA_5	= 10;
localparam PS_AS5311_DAQ_1	= 11;
localparam PS_AS5311_DAQ_2	= 12;
localparam PS_MAX		= 12;

localparam PS_BITS= $clog2(PS_MAX + 1);
reg [PS_BITS-1:0] state = PS_IDLE;

localparam AS_IDLE		= 0;
localparam AS_CS_LO		= 1;
localparam AS_CLK_LO		= 2;
localparam AS_CLK_HI		= 3;
localparam AS_CS_HI		= 4;
localparam AS_END		= 5;
localparam AS_MAX		= 5;

localparam AS_BITS= $clog2(AS_MAX + 1);
reg [AS_BITS-1:0] as_state [NAS5311];
reg use_daq = 0;

integer i;
initial begin
	for (i = 0; i < NAS5311; i = i + 1) begin
		data[i] = 0;
		prev_data[i] = { BITSIZE { 1'b1 }};
		prev_magnet[i] = { BITSIZE { 1'b1 }};
		starttime[i] = 0;
		next_data[i] = 0;
		next_mag[i] = 0;
		as_state[i] = AS_IDLE;
		data_interval[i] = 0;
		mag_interval[i] = 0;
		freq_divider[i] = 0;
	end
end

/* receive state machine */
always @(posedge clk) begin
	for (i = 0; i < NAS5311; i = i + 1) begin
		if (data_interval[i] != 0 && next_data[i] == systime) begin
			data_pending[i] <= 1;
			next_data[i] <= data_interval[i] + systime;
		end
		if (mag_interval[i] != 0 && next_mag[i] == systime) begin
			mag_pending[i] <= 1;
			next_mag[i] <= mag_interval[i] + systime;
		end
		if (data_valid[i] && data_ack[i])
			data_valid[i] <= 0;
		if (freq_divider[i] == 0) begin
			/* disabled */
			as_state[i] <= AS_IDLE;
			data_valid[i] <= 0;
		end else if (as_state[i] == AS_IDLE && (data_pending[i] | mag_pending[i])) begin
			data[i] <= 0;
			data_valid[i] <= 0;
			as5311_clk[i] <= data_pending[i];
			ftype[i] <= data_pending[i];
			if (data_pending[i])
				data_pending[i] <= 0;
			else
				mag_pending[i] <= 0;
			as_state[i] <= AS_CS_LO;
			starttime[i] <= systime;
			div[i] <= freq_divider[i];
		end else if (as_state[i] == AS_CS_LO && div[i] == 1) begin
			as5311_cs[i] <= 0;
			div[i] <= freq_divider[i];
			bitcnt[i] <= BITSIZE;
			as_state[i] <= AS_CLK_LO;
		end else if (as_state[i] == AS_CLK_LO && div[i] == 1) begin
			as5311_clk[i] <= 0;
			div[i] <= freq_divider[i];
			data[i] <= { data[i][BITSIZE-2:0], as5311_do[i] };
			bitcnt[i] <= bitcnt[i] - 1;
			as_state[i] <= AS_CLK_HI;
		end else if (as_state[i] == AS_CLK_HI && div[i] == 1) begin
			as5311_clk[i] <= 1;
			div[i] <= freq_divider[i];
			if (bitcnt[i] == 0)
				as_state[i] <= AS_CS_HI;
			else
				as_state[i] <= AS_CLK_LO;
		end else if (as_state[i] == AS_CS_HI && div[i] == 1) begin
			as5311_cs[i] <= 1;
			div[i] <= freq_divider[i];
			data[i] <= { data[i][BITSIZE-2:0], as5311_do[i] };
			if (ftype[i]) begin
				if (data[i] != prev_data[i]) begin
					data_valid[i] <= 1;
					prev_data[i] <= data[i];
				end
			end else begin
				if (data[i] != prev_magnet[i]) begin
					data_valid[i] <= 1;
					prev_magnet[i] <= data[i];
				end
			end
			as_state[i] <= AS_END;
		end else if (as_state[i] == AS_END && div[i] == 1) begin
			as_state[i] <= AS_IDLE;
		end else if (div[i] != 0) begin
			div[i] <= div[i] - 1;
		end
	end
end

always @(posedge clk) begin
	cmd_done <= 0;
	daq_end <= 0;
	daq_valid <= 0;
	data_ack <= 0;

	if (state == PS_IDLE && cmd_ready) begin
		// common to all cmds
		channel <= arg_data[NAS5311_BITS-1:0];
		if (cmd == CMD_CONFIG_AS5311) begin
			state <= PS_CONFIG_AS5311_1;
		end else begin
			cmd_done <= 1;
		end
	end else if (state == PS_CONFIG_AS5311_1) begin
		/* freq divider, data interval, mag interval */
		freq_divider[channel] <= arg_data;
		state <= PS_CONFIG_AS5311_2;
	end else if (state == PS_CONFIG_AS5311_2) begin
		data_interval[channel] <= arg_data;
		next_data[channel] <= arg_data + systime;
		state <= PS_CONFIG_AS5311_3;
	end else if (state == PS_CONFIG_AS5311_3) begin
		mag_interval[channel] <= arg_data;
		next_mag[channel] <= arg_data + systime;
		state <= PS_CONFIG_AS5311_4;
	end else if (state == PS_CONFIG_AS5311_4) begin
		use_daq <= arg_data;
		cmd_done <= 1;
		state <= PS_IDLE;
	end else if (state == PS_IDLE && data_valid) begin
		/* XXX TODO make use_daq per channel */
		if (use_daq)
			daq_req <= 1;
		else
			invol_req <= 1;
		state <= PS_WAIT_GRANT;
	end else if (state == PS_WAIT_GRANT && (invol_grant || daq_grant)) begin
		invol_req <= 0;
		daq_req <= 0;
		for (i = 0; i < NAS5311; i = i + 1) begin
			if (data_valid[i]) begin
				channel <= i;
				if (invol_grant)
					state <= PS_AS5311_DATA_1;
				else
					state <= PS_AS5311_DAQ_1;
				i = NAS5311;
			end
		end
	end else if (state == PS_AS5311_DATA_1) begin
		param_data <= channel;
		param_write <= 1;
		state <= PS_AS5311_DATA_2;
	end else if (state == PS_AS5311_DATA_2) begin
		param_data <= starttime[channel];
		state <= PS_AS5311_DATA_3;
	end else if (state == PS_AS5311_DATA_3) begin
		param_data <= data[channel];
		state <= PS_AS5311_DATA_4;
	end else if (state == PS_AS5311_DATA_4) begin
		param_data <= ftype[channel];
		state <= PS_AS5311_DATA_5;
		data_ack[channel] <= 1;
	end else if (state == PS_AS5311_DATA_5) begin
		cmd_done <= 1;
		param_write <= 0;
		param_data <= RSP_AS5311_DATA;
		state <= PS_IDLE;
	end else if (state == PS_AS5311_DAQ_1) begin
		if (ftype[channel])
			param_data[31:24] <= DAQT_AS5311_DAT;
		else
			param_data[31:24] <= DAQT_AS5311_MAG;
		param_data[23:18] <= channel;
		param_data[17:0] <= data[channel];
		daq_valid <= 1;
		data_ack[channel] <= 1;
		state <= PS_AS5311_DAQ_2;
	end else if (state == PS_AS5311_DAQ_2) begin
		param_data <= starttime[channel];
		daq_valid <= 1;
		daq_end <= 1;
		state <= PS_IDLE;
	end
end

assign debug[3:0] = state;
assign debug[6:4] = as_state[0];
assign debug[15:7] = 0;

endmodule
