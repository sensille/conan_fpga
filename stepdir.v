`timescale 1ns / 1ps
`default_nettype none

module stepdir #(
	parameter MOVE_TYPE_KLIPPER = 3'b000,
	parameter MOVE_TYPE_BITS = 0,
	parameter STEP_INTERVAL_BITS = 0,
	parameter STEP_COUNT_BITS = 0,
	parameter STEP_ADD_BITS = 0,
	parameter MOVE_COUNT = 0
) (
	input wire clk,
	input wire [MOVE_TYPE_BITS + STEP_INTERVAL_BITS + STEP_COUNT_BITS + STEP_ADD_BITS + 1 - 1:0] queue_wr_data,
	input wire queue_wr_en,
	output wire queue_empty,

	input wire dedge,

	input wire do_reset_clock,
	input wire [31:0] reset_clock,
	input wire [31:0] clock,
	input wire reset,

	output reg step = 0,
	output reg dir = 0,

	output reg [31:0] position = 0,
	output wire [31:0] next_step_time,
	output reg missed_clock = 0,
	output wire queue_full,

	output wire [$clog2(MOVE_COUNT)-1:0] elemcnt,
	output wire [15:0] debug
);

localparam DATA_WIDTH = MOVE_TYPE_BITS + STEP_INTERVAL_BITS + STEP_COUNT_BITS + STEP_ADD_BITS + 1;
localparam MOVE_ADDR_BITS = $clog2(MOVE_COUNT);

wire [DATA_WIDTH-1:0] queue_rd_data;
reg queue_rd_en = 0;

fifo #(
	.DATA_WIDTH(DATA_WIDTH),
	.ADDR_WIDTH(MOVE_ADDR_BITS)
) u_fifo (
	.clk(clk),
	.clr(reset),
	.din(queue_wr_data),
	.wr_en(queue_wr_en),
	.full(queue_full),
	.dout(queue_rd_data),
	.rd_en(queue_rd_en),
	.empty(queue_empty),
	.elemcnt(elemcnt)
);

/*
 * queue is 100 bits wide:
 * <dir:1> <interval:32> <count:32> <add:32> <move type:3>
 *
 * move type has to be last, otherwise yosys won't infer block ram.
 * This has to do with the fact that we don't use the move_type yet.
 */
/* for convenient access */
wire [MOVE_TYPE_BITS-1:0] q_move_type;
wire [STEP_INTERVAL_BITS-1:0] q_interval;
wire q_dir;
wire [STEP_COUNT_BITS-1:0] q_count;
wire [STEP_ADD_BITS-1:0] q_add;
assign { q_dir, q_interval, q_count, q_add, q_move_type } = queue_rd_data;

reg [STEP_INTERVAL_BITS-1:0] interval = 0;
reg [STEP_COUNT_BITS-1:0] count = 0;
reg [STEP_ADD_BITS-1:0] add = 0;
wire [STEP_INTERVAL_BITS-1:0] signed_add = { {(STEP_INTERVAL_BITS - STEP_ADD_BITS) { add[STEP_ADD_BITS-1] }}, add };
reg [31:0] next_step = 0;
reg next_dir = 0;
reg delayed_reset = 0;
assign next_step_time = next_step;

reg [2:0] step_delay = 0;

always @(posedge clk) begin
	if (queue_rd_en)
		queue_rd_en <= 0;

	if (count == 0 && !queue_empty && !delayed_reset && !reset) begin
		/*
		 * currently this condition is only here to make use of all bits
		 * of the fifo. Otherwise yosys won't infer a block ram for it
		 */
		count <= q_count;
		add <= q_add;
		interval <= q_interval;
		next_step <= next_step + q_interval;
		next_dir <= q_dir;
		queue_rd_en <= 1;
		/* check if next_step + q_interval is behind sysclock. if so, set error flag */
		if (next_step + q_interval - clock >= 32'hc0000000)
			missed_clock <= 1;
	end else if (count != 0 && clock == next_step) begin
		count <= count - 1;
		if (count != 1) begin
			interval <= interval + signed_add;
			next_step <= next_step + interval + signed_add;
		end else if (!queue_empty) begin
			count <= q_count;
			add <= q_add;
			interval <= q_interval;
			next_step <= next_step + q_interval;
			next_dir <= q_dir;
			queue_rd_en <= 1;
		end
		if (dedge)
			step <= !step;
		else
			step <= 1;
		step_delay <= 7;
		if (dir)
			position <= position + 1;
		else
			position <= position - 1;
	end

	if (do_reset_clock)
		next_step <= reset_clock;

	if (step_delay) begin
		step_delay <= step_delay - 1;
	end else begin
		if (!dedge && step)
			step <= 0;
		dir <= next_dir;
	end

	if (reset) begin
		count <= 0;
		/*
		 * because queue_empty is delayed by one slot, we also need
		 * to delay our check of it
		 */
		delayed_reset <= 1;
	end
	if (delayed_reset)
		delayed_reset <= 0;
end

assign debug[8:0] = elemcnt;
assign debug[15:9] = next_step[31:25];

endmodule
