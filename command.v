`timescale 1ns / 1ps
`default_nettype none

module command #(
	parameter HZ = 0,
	parameter LEN_BITS = 0,
	parameter LEN_FIFO_BITS = 0,
	parameter MOVE_COUNT = 0,
	parameter NGPIO = 0,
	parameter NPWM = 0,
	parameter NSTEPDIR = 6,
	parameter NENDSTOP = 0,
	parameter NUART = 0,
	parameter VERSION = 0
) (
	input wire clk,

	/*
	 * receive side
	 */
	input wire [7:0] msg_data,
	input wire msg_ready,
	output reg msg_rd_en = 0,

	/*
	 * send side
	 */
	output reg send_fifo_wr_en = 0,
	output reg [LEN_BITS-1:0] send_fifo_data = 0,
	input wire send_fifo_full,

	/* ring buffer input */
	output reg [7:0] send_ring_data = 0,
	output reg send_ring_wr_en = 0,
	input wire send_ring_full,

	/* global time */
	input wire [63:0] systime,

	/*
	 * I/O
	 */
	output wire [NGPIO-1:0] gpio,
	output wire [NPWM-1:0] pwm,
	output wire [NSTEPDIR-1:0] step,
	output wire [NSTEPDIR-1:0] dir,
	input wire [NENDSTOP-1:0] endstop,
	input wire [NUART-1:0] uart_in,
	output wire [NUART-1:0] uart_out,
	output wire [NUART-1:0] uart_en,

	input wire [63:0] time_in,
	output wire [63:0] time_out,
	output wire time_out_en,
	input wire timesync_latch_in,
	input wire req_shutdown,

	/*
	 * debug
	 */
	output wire [52:0] debug,
	output wire [15:0] step_debug
);

/*
 * definitions for each command
 *
 * indexed by command id
 * values number of parameters, has string, unit to dispatch to, cmd sends response
 * strings are always last
 * 
 */
localparam MAX_ARGS = 8;
localparam ARGS_BITS = $clog2(MAX_ARGS);

localparam UNITS_BITS		= 4;
localparam UNIT_PWM		= 4'd0;
localparam UNIT_SYSTEM		= 4'd1;
localparam UNIT_STEPPER		= 4'd2;
localparam UNIT_TMCUART		= 4'd3;
localparam UNIT_GPIO		= 4'd4;
localparam NUNITS		= 4'd5;

localparam CMDTAB_SIZE = UNITS_BITS + ARGS_BITS + 1;
localparam CMD_GET_VERSION		= 0;
localparam CMD_SYNC_TIME		= 1;
localparam CMD_GET_TIME			= 2;
localparam CMD_CONFIG_PWM		= 3;
localparam CMD_SCHEDULE_PWM		= 4;
localparam CMD_CONFIG_STEPPER		= 5;
localparam CMD_QUEUE_STEP		= 6;
localparam CMD_SET_NEXT_STEP_DIR	= 7;
localparam CMD_RESET_STEP_CLOCK		= 8;
localparam CMD_STEPPER_GET_POS		= 9;
localparam CMD_ENDSTOP_SET_STEPPER	= 10;
localparam CMD_ENDSTOP_QUERY		= 11;
localparam CMD_ENDSTOP_HOME		= 12;
localparam CMD_TMCUART_WRITE		= 13;
localparam CMD_TMCUART_READ		= 14;
localparam CMD_SET_DIGITAL_OUT		= 15;
localparam CMD_CONFIG_DIGITAL_OUT	= 16;
localparam CMD_SCHEDULE_DIGITAL_OUT	= 17;
localparam CMD_UPDATE_DIGITAL_OUT	= 18;
localparam CMD_SHUTDOWN			= 19;
localparam CMD_STEPPER_GET_NEXT		= 20;
localparam NCMDS			= 21;
localparam CMD_BITS = $clog2(NCMDS);

localparam RSP_GET_VERSION	= 0;
localparam RSP_GET_TIME		= 1;
localparam RSP_STEPPER_GET_POS	= 2;
localparam RSP_ENDSTOP_STATE	= 3;
localparam RSP_TMCUART_READ	= 4;
localparam RSP_SHUTDOWN		= 5;
localparam RSP_STEPPER_GET_NEXT	= 6;

localparam MISSED_STEPPER	= 0;
localparam MISSED_ENDSTOP	= 1;
localparam MISSED_PWM		= 2;
localparam MISSED_GPIO		= 3;
localparam MISSED_REQ_SHUTDOWN	= 4;
localparam MISSED_BITS		= 5;

/* BAD HACK, should go away soon */
localparam [ARGS_BITS-1:0] ARGS_0 = 0;
localparam [ARGS_BITS-1:0] ARGS_1 = 1;
localparam [ARGS_BITS-1:0] ARGS_2 = 2;
localparam [ARGS_BITS-1:0] ARGS_3 = 3;
localparam [ARGS_BITS-1:0] ARGS_4 = 4;
localparam [ARGS_BITS-1:0] ARGS_5 = 5;
localparam [ARGS_BITS-1:0] ARGS_6 = 6;
localparam [ARGS_BITS-1:0] ARGS_7 = 7;

reg [CMDTAB_SIZE-1:0] cmdtab[NCMDS];
/* { unit, nargs, string_arg, cmd_has_response } */
initial begin
	cmdtab[CMD_GET_VERSION] = { UNIT_SYSTEM, ARGS_0, 1'b0, 1'b1 };
	cmdtab[CMD_SYNC_TIME] = { UNIT_SYSTEM, ARGS_2, 1'b0, 1'b0 };
	cmdtab[CMD_GET_TIME] = { UNIT_SYSTEM, ARGS_0, 1'b0, 1'b1 };
	cmdtab[CMD_CONFIG_PWM] = { UNIT_PWM, ARGS_5, 1'b0, 1'b0 };
	cmdtab[CMD_SCHEDULE_PWM] = { UNIT_PWM, ARGS_3, 1'b0, 1'b0 };
	cmdtab[CMD_CONFIG_STEPPER] = { UNIT_STEPPER, ARGS_2, 1'b0, 1'b0 };
	cmdtab[CMD_QUEUE_STEP] = { UNIT_STEPPER, ARGS_4, 1'b0, 1'b0 };
	cmdtab[CMD_SET_NEXT_STEP_DIR] = { UNIT_STEPPER, ARGS_2, 1'b0, 1'b0 };
	cmdtab[CMD_RESET_STEP_CLOCK] = { UNIT_STEPPER, ARGS_2, 1'b0, 1'b0 };
	cmdtab[CMD_STEPPER_GET_POS] = { UNIT_STEPPER, ARGS_1, 1'b0, 1'b1 };
	cmdtab[CMD_ENDSTOP_SET_STEPPER] = { UNIT_STEPPER, ARGS_2, 1'b0, 1'b0 };
	cmdtab[CMD_ENDSTOP_QUERY] = { UNIT_STEPPER, ARGS_1, 1'b0, 1'b1 };
	cmdtab[CMD_ENDSTOP_HOME] = { UNIT_STEPPER, ARGS_4, 1'b0, 1'b0 };
	cmdtab[CMD_TMCUART_WRITE] = { UNIT_TMCUART, ARGS_4, 1'b0, 1'b0 };
	cmdtab[CMD_TMCUART_READ] = { UNIT_TMCUART, ARGS_3, 1'b0, 1'b1 };
	cmdtab[CMD_SET_DIGITAL_OUT] = { UNIT_GPIO, ARGS_2, 1'b0, 1'b0 };
	cmdtab[CMD_CONFIG_DIGITAL_OUT] = { UNIT_GPIO, ARGS_4, 1'b0, 1'b0 };
	cmdtab[CMD_SCHEDULE_DIGITAL_OUT] = { UNIT_GPIO, ARGS_3, 1'b0, 1'b0 };
	cmdtab[CMD_UPDATE_DIGITAL_OUT] = { UNIT_GPIO, ARGS_2, 1'b0, 1'b0 };
	cmdtab[CMD_SHUTDOWN] = { UNIT_SYSTEM, ARGS_0, 1'b0, 1'b0 };
	cmdtab[CMD_STEPPER_GET_NEXT] = { UNIT_STEPPER, ARGS_1, 1'b0, 1'b1 };
end

wire shutdown; /* set by command, never cleared */
wire [MISSED_BITS-1:0] missed_clock;
assign missed_clock[MISSED_REQ_SHUTDOWN] = req_shutdown;

reg [ARGS_BITS-1:0] unit_arg_ptr = 0;
reg [31:0] unit_arg_data = 0;
wire [NUNITS-1:0] unit_arg_advance;
wire [CMD_BITS-1:0] unit_cmd;
reg [NUNITS-1:0] unit_cmd_ready = 0;
wire [NUNITS-1:0] unit_cmd_done;
wire [31:0] unit_param_data [NUNITS];
wire [NUNITS-1:0] unit_param_write;
wire [NUNITS-1:0] unit_invol_req;
reg [NUNITS-1:0] unit_invol_grant = 0;
pwm #(
	.NPWM(NPWM),
	.CMD_CONFIG_PWM(CMD_CONFIG_PWM),
	.CMD_SCHEDULE_PWM(CMD_SCHEDULE_PWM)
) u_pwm (
	.clk(clk),
	.systime(systime),

	.arg_data(unit_arg_data),
	.arg_advance(unit_arg_advance[UNIT_PWM]),
	.cmd(unit_cmd),
	.cmd_ready(unit_cmd_ready[UNIT_PWM]),
	.cmd_done(unit_cmd_done[UNIT_PWM]),

	.param_data(unit_param_data[UNIT_PWM]),
	.param_write(unit_param_write[UNIT_PWM]),

	.invol_req(unit_invol_req[UNIT_PWM]),
	.invol_grant(unit_invol_grant[UNIT_PWM]),

	.pwm(pwm),

	.shutdown(shutdown),

	.missed_clock(missed_clock[MISSED_PWM])
);

wire [$clog2(NSTEPDIR):0] step_queue_overflow;
system #(
	.CMD_GET_VERSION(CMD_GET_VERSION),
	.RSP_GET_VERSION(RSP_GET_VERSION),
	.CMD_SYNC_TIME(CMD_SYNC_TIME),
	.CMD_GET_TIME(CMD_GET_TIME),
	.RSP_GET_TIME(RSP_GET_TIME),
	.CMD_SHUTDOWN(CMD_SHUTDOWN),
	.RSP_SHUTDOWN(RSP_SHUTDOWN),
	.VERSION(VERSION),
	.MOVE_COUNT(MOVE_COUNT),
	.NGPIO(NGPIO),
	.NPWM(NPWM),
	.NSTEPDIR(NSTEPDIR),
	.NENDSTOP(NENDSTOP),
	.NUART(NUART),
	.MISSED_BITS(MISSED_BITS)
) u_system (
	.clk(clk),
	.systime(systime),

	.arg_data(unit_arg_data),
	.arg_advance(unit_arg_advance[UNIT_SYSTEM]),
	.cmd(unit_cmd),
	.cmd_ready(unit_cmd_ready[UNIT_SYSTEM]),
	.cmd_done(unit_cmd_done[UNIT_SYSTEM]),

	.param_data(unit_param_data[UNIT_SYSTEM]),
	.param_write(unit_param_write[UNIT_SYSTEM]),

	.invol_req(unit_invol_req[UNIT_SYSTEM]),
	.invol_grant(unit_invol_grant[UNIT_SYSTEM]),


	.time_in(time_in),
	.time_out(time_out),
	.time_out_en(time_out_en),
	.timesync_latch_in(timesync_latch_in),

	.shutdown(shutdown),
	.missed_clock(missed_clock),
	.step_queue_overflow(step_queue_overflow)
);

wire [28:0] stepper_debug;
stepper #(
	.NSTEPDIR(NSTEPDIR),
	.NENDSTOP(NENDSTOP),
	.MOVE_COUNT(MOVE_COUNT),
	.CMD_BITS(CMD_BITS),
	.CMD_CONFIG_STEPPER(CMD_CONFIG_STEPPER),
	.CMD_QUEUE_STEP(CMD_QUEUE_STEP),
	.CMD_SET_NEXT_STEP_DIR(CMD_SET_NEXT_STEP_DIR),
	.CMD_RESET_STEP_CLOCK(CMD_RESET_STEP_CLOCK),
	.CMD_STEPPER_GET_POS(CMD_STEPPER_GET_POS),
	.CMD_STEPPER_GET_NEXT(CMD_STEPPER_GET_NEXT),
	.CMD_ENDSTOP_SET_STEPPER(CMD_ENDSTOP_SET_STEPPER),
	.CMD_ENDSTOP_QUERY(CMD_ENDSTOP_QUERY),
	.CMD_ENDSTOP_HOME(CMD_ENDSTOP_HOME),
	.RSP_STEPPER_GET_POS(RSP_STEPPER_GET_POS),
	.RSP_ENDSTOP_STATE(RSP_ENDSTOP_STATE),
	.RSP_STEPPER_GET_NEXT(RSP_STEPPER_GET_NEXT)
) u_stepper (
	.clk(clk),
	.systime(systime),

	.arg_data(unit_arg_data),
	.arg_advance(unit_arg_advance[UNIT_STEPPER]),
	.cmd(unit_cmd),
	.cmd_ready(unit_cmd_ready[UNIT_STEPPER]),
	.cmd_done(unit_cmd_done[UNIT_STEPPER]),

	.param_data(unit_param_data[UNIT_STEPPER]),
	.param_write(unit_param_write[UNIT_STEPPER]),

	.invol_req(unit_invol_req[UNIT_STEPPER]),
	.invol_grant(unit_invol_grant[UNIT_STEPPER]),

	.step(step),
	.dir(dir),
	.endstop_in(endstop),

	.shutdown(shutdown),

	.step_missed_clock(missed_clock[MISSED_STEPPER]),
	.endstop_missed_clock(missed_clock[MISSED_ENDSTOP]),
	.queue_overflow(step_queue_overflow),

	.debug(stepper_debug),
	.step_debug(step_debug)
);

tmcuart #(
	.HZ(HZ),
	.NUART(NUART),
	.CMD_TMCUART_WRITE(CMD_TMCUART_WRITE),
	.CMD_TMCUART_READ(CMD_TMCUART_READ),
	.RSP_TMCUART_READ(RSP_TMCUART_READ)
) u_tmcuart (
	.clk(clk),
	.systime(systime),

	.arg_data(unit_arg_data),
	.arg_advance(unit_arg_advance[UNIT_TMCUART]),
	.cmd(unit_cmd),
	.cmd_ready(unit_cmd_ready[UNIT_TMCUART]),
	.cmd_done(unit_cmd_done[UNIT_TMCUART]),

	.param_data(unit_param_data[UNIT_TMCUART]),
	.param_write(unit_param_write[UNIT_TMCUART]),

	.invol_req(unit_invol_req[UNIT_TMCUART]),
	.invol_grant(unit_invol_grant[UNIT_TMCUART]),

	.uart_in(uart_in),
	.uart_out(uart_out),
	.uart_en(uart_en),

	.shutdown(shutdown)
);

gpio #(
	.NGPIO(NGPIO),
	.CMD_SET_DIGITAL_OUT(CMD_SET_DIGITAL_OUT),
	.CMD_CONFIG_DIGITAL_OUT(CMD_CONFIG_DIGITAL_OUT),
	.CMD_SCHEDULE_DIGITAL_OUT(CMD_SCHEDULE_DIGITAL_OUT),
	.CMD_UPDATE_DIGITAL_OUT(CMD_UPDATE_DIGITAL_OUT)
) u_gpio (
	.clk(clk),
	.systime(systime),

	.arg_data(unit_arg_data),
	.arg_advance(unit_arg_advance[UNIT_GPIO]),
	.cmd(unit_cmd),
	.cmd_ready(unit_cmd_ready[UNIT_GPIO]),
	.cmd_done(unit_cmd_done[UNIT_GPIO]),

	.param_data(unit_param_data[UNIT_GPIO]),
	.param_write(unit_param_write[UNIT_GPIO]),

	.invol_req(unit_invol_req[UNIT_GPIO]),
	.invol_grant(unit_invol_grant[UNIT_GPIO]),

	.gpio(gpio),

	.shutdown(shutdown),

	.missed_clock(missed_clock[MISSED_GPIO])
);

localparam MST_IDLE = 0;
localparam MST_PARSE_ARG_START = 1;
localparam MST_PARSE_ARG_CONT = 2;
localparam MST_STRING_START = 3;
localparam MST_STRING_ARG = 4;
localparam MST_DISPATCH = 5;
localparam MST_DISPATCH_WAIT_DONE = 6;
localparam MST_PARAM = 7;
localparam MST_PARAM_SKIP = 8;
localparam MST_PARAM_SEND = 9;
localparam MST_MAX = 9;
localparam MST_BITS = $clog2(MST_MAX + 1);

reg [MST_BITS-1:0] msg_state = 0;
reg [CMD_BITS-1:0] msg_cmd = 0;
assign unit_cmd = msg_cmd;

/* input args state */
reg [31:0] args[MAX_ARGS];
reg [ARGS_BITS-1:0] curr_arg = 0;
reg [ARGS_BITS-1:0] nargs = 0;
reg [UNITS_BITS-1:0] unit = 0;
reg cmd_has_response = 0;

/* output parameters state */
localparam MAX_PARAMS = 8;
localparam PARAM_BITS = $clog2(MAX_PARAMS);
reg [31:0] params [MAX_PARAMS];
reg [PARAM_BITS-1:0] nparams = 0;
reg [PARAM_BITS-1:0] curr_param = 0;
reg [34:0] rcv_param = 0;
reg [2:0] curr_cnt = 0;	/* counter for VLQ */
reg [7:0] rsp_len = 0;
reg string_arg = 0;
/* assume max string is 64 */
reg [5:0] str_pos = 0;
reg [5:0] str_len = 0;
reg [7:0] str_buf[64];

reg _arg_end;
integer i;
always @(posedge clk) begin
	if (msg_rd_en) begin
		msg_rd_en <= 0;
	end
	if (send_ring_wr_en) begin
		send_ring_wr_en <= 0;
	end
	if (send_fifo_wr_en) begin
		send_fifo_wr_en <= 0;
	end
	for (i = 0; i < NUNITS; i = i + 1) begin
		if (unit_invol_grant[i])
			unit_invol_grant[i] <= 0;
	end
	/*
	 * ------------------------------------
	 * stage 1, parse arguments, decode VLQ
	 * ------------------------------------
	 */
	if (msg_ready && !msg_rd_en) begin
		/* verilator lint_off BLKSEQ */
		_arg_end = 0;
		/* verilator lint_on BLKSEQ */
		msg_rd_en <= 1;
		if (msg_state == MST_IDLE) begin
			msg_cmd <= msg_data;
			curr_arg <= 0;
			{ unit, nargs, string_arg, cmd_has_response } <= cmdtab[msg_data];
			if (cmdtab[msg_data][ARGS_BITS+1:2] == 0)
				msg_state <= MST_DISPATCH;
			else
				msg_state <= MST_PARSE_ARG_START;
		end else if (msg_state == MST_PARSE_ARG_START) begin
			args[curr_arg] <= msg_data[6:0];
			if (msg_data[6:5] == 2'b11) begin
				/* negative value */
				args[curr_arg][31:7] <= 25'b1111111111111111111111111;
			end
			if (msg_data[7]) begin
				msg_state <= MST_PARSE_ARG_CONT;
			end else begin
				/* verilator lint_off BLKSEQ */
				_arg_end = 1;
				/* verilator lint_on BLKSEQ */
			end
		end else if (msg_state == MST_PARSE_ARG_CONT) begin
			args[curr_arg] <= { args[curr_arg][24:0], msg_data[6:0] };
			if (!msg_data[7]) begin
				/* verilator lint_off BLKSEQ */
				_arg_end = 1;
				/* verilator lint_on BLKSEQ */
			end
		end else if (msg_state == MST_STRING_START) begin
			string_arg <= 0;
			str_pos <= 0;
			str_len <= args[curr_arg - 1];
			msg_state <= MST_STRING_ARG;
			msg_rd_en <= 0;	/* no read in this clock */
		end else if (msg_state == MST_STRING_ARG) begin
			str_buf[str_pos] <= msg_data;
			str_pos <= str_pos + 1;
			if (str_len == str_pos + 1) begin
				msg_state <= MST_DISPATCH;
			end
		end else begin
			/* we're in some of the states below */
			msg_rd_en <= 0;	/* we're in some of the states below */
		end
		if (_arg_end) begin
			curr_arg <= curr_arg + 1;
			if (curr_arg + 1 == nargs) begin
				if (string_arg) begin
					msg_state <= MST_STRING_START;
				end else begin
					msg_state <= MST_DISPATCH;
				end
			end else begin
				msg_state <= MST_PARSE_ARG_START;
			end
		end
	end
	/*
	 * -------------------------
	 * stage 2, dispatch message
	 * -------------------------
	 */
	if (msg_state == MST_DISPATCH) begin
		unit_arg_data <= args[0];
		unit_arg_ptr <= 1;
		curr_param <= 0;
		nparams <= 0;
		unit_cmd_ready[unit] <= 1;
		msg_state <= MST_DISPATCH_WAIT_DONE;
	end else if (msg_state == MST_DISPATCH_WAIT_DONE) begin
		unit_cmd_ready[unit] <= 0;
		unit_invol_grant[unit] <= 0;
		if (unit_param_write[unit]) begin
			params[nparams] <= unit_param_data[unit];
			nparams <= nparams + 1;
		end
		if (unit_arg_advance[unit]) begin
			unit_arg_data <= args[unit_arg_ptr];
			unit_arg_ptr <= unit_arg_ptr + 1;
		end
		if (unit_cmd_done[unit]) begin
			if (cmd_has_response) begin
				send_ring_data <= unit_param_data[unit];
				send_ring_wr_en <= 1;
				rsp_len <= 1;
				msg_state <= MST_PARAM;
			end else begin
				msg_state <= MST_IDLE;
			end
		end
	/*
	 * ---------------------------------
	 * stage 3, encode and send response
	 * ---------------------------------
	 */
	end else if (msg_state == MST_PARAM) begin
		/*
		 * encode VLQ param
		 */
		/*
		 * < 00000060 && >= ffffffe0 length 1
		 * < 00003000 && >= fffff000 length 2
		 * < 00180000 && >= fff80000 length 3
		 * < 0c000000 && >= fc000000 length 4
		 * else length 5
		 */
		curr_cnt <= 4;
		/* extend by 3 bits, 32+3 == 5 * 7 */
		rcv_param <= { params[curr_param][31], params[curr_param][31],
				params[curr_param][31], params[curr_param] };
		msg_state <= MST_PARAM_SKIP;
	end else if (msg_state == MST_PARAM_SKIP) begin
		if (curr_cnt != 0 &&
		    (rcv_param[34:26] == 9'b111111111 ||
		     rcv_param[34:26] <  9'b000000011)) begin
			curr_cnt <= curr_cnt - 1;
			rcv_param <= { rcv_param[27:0], 7'b0 };
		end else begin
			msg_state <= MST_PARAM_SEND;
		end
	end else if (msg_state == MST_PARAM_SEND) begin
		if (curr_cnt == 0) begin
			if (curr_param + 1 != nparams) begin
				curr_param <= curr_param + 1;
				msg_state <= MST_PARAM;
			end else begin
				send_fifo_data <= rsp_len + 1;
				send_fifo_wr_en <= 1;
				msg_state <= MST_IDLE;
			end
			send_ring_data[7] <= 1'b0;
		end else begin
			send_ring_data[7] <= 1'b1;
		end
		send_ring_data[6:0] <= rcv_param[34:28];
		send_ring_wr_en <= 1;
		rsp_len <= rsp_len + 1;
		curr_cnt <= curr_cnt - 1;
		rcv_param <= { rcv_param[27:0], 7'b0 };
	/*
	 * ------------------------------
	 * stage 5, send involuntary data
	 * ------------------------------
	 *
	 * only send when state machine is idle and no msg has
	 * been pulled at stage 1
	 */
	end else if (!msg_ready && msg_state == MST_IDLE) begin
		for (i = 0; i < NUNITS; i = i + 1) begin
			if (unit_invol_req[i]) begin
				unit <= i;
				nparams <= 0;
				curr_param <= 0;
				cmd_has_response <= 1;
				unit_invol_grant[i] <= 1;
				msg_state <= MST_DISPATCH_WAIT_DONE;
				i = NUNITS;
			end
		end
	end
end

reg [3:0] msg_state_pre = 0;
reg [3:0] msg_state_pre_pre = 0;
reg [3:0] msg_state_pre_pre_pre = 0;
always @(posedge clk) begin
	if (msg_state != msg_state_pre) begin
		msg_state_pre_pre_pre <= msg_state_pre_pre;
		msg_state_pre_pre <= msg_state_pre;
		msg_state_pre <= msg_state;
	end
end
assign debug[3:0] = msg_state_pre;
assign debug[7:4] = msg_state_pre_pre;
assign debug[11:8] = msg_state_pre_pre_pre;
assign debug[16:12] = msg_cmd;
assign debug[UNITS_BITS+19:20] = unit;
assign debug[52:UNITS_BITS+20] = stepper_debug;

endmodule
