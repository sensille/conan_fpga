`timescale 1ns / 1ps
`default_nettype none

module stepdir #(
	parameter MOVE_TYPE_KLIPPER = 3'b000,
	parameter MOVE_TYPE_BITS = 3,
	parameter STEP_INTERVAL_BITS = 22,
	parameter STEP_COUNT_BITS = 26,
	parameter STEP_ADD_BITS = 20,
	parameter MOVE_COUNT = 512
) (
	input wire clk,
	input wire [71:0] queue_wr_data,
	input wire queue_wr_en,
	output wire queue_empty,
	output reg running = 0,

	input wire dedge,

	input wire start,
	input wire reset,

	output reg step = 0,
	output reg dir = 0,

	output reg [31:0] position = 0
);

localparam MOVE_ADDR_BITS = $clog2(MOVE_COUNT);

wire [71:0] queue_rd_data;
reg queue_rd_en = 0;

fifo #(
	.DATA_WIDTH(72),
	.ADDR_WIDTH(MOVE_ADDR_BITS)
) u_fifo (
	.clk(clk),
	.clr(reset),
	.din(queue_wr_data),
	.wr_en(queue_wr_en),
	.full(),
	.dout(queue_rd_data),
	.rd_en(queue_rd_en),
	.empty(queue_empty),
	.elemcnt()
);

/*
 * queue is 72 bits wide:
 * <dir:1> <interval:22> <count:26> <add:20> <move type:3>
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
reg [STEP_INTERVAL_BITS-1:0] curr_interval = 0;
reg [STEP_COUNT_BITS-1:0] count = 0;
reg [STEP_ADD_BITS-1:0] add = 0;
wire [STEP_INTERVAL_BITS-1:0] signed_add = { {(STEP_INTERVAL_BITS - STEP_ADD_BITS) { add[STEP_ADD_BITS-1] }}, add };

reg step_delay = 0;

always @(posedge clk) begin
	if (queue_rd_en)
		queue_rd_en <= 0;

	if (count == 0) begin
		if (start && !queue_empty) begin
			/*
			 * currently this condition is only here to make use of all bits
			 * of the fifo. Otherwise yosys won't infer a block ram for it
			 */
			running <= 1;
			count <= q_count;
			add <= q_add;
			interval <= q_interval;
			curr_interval <= q_interval - 1;
			dir <= q_dir;
			queue_rd_en <= 1;
		end
	end else begin
		if (curr_interval != 0) begin
			curr_interval <= curr_interval - 1;
		end else begin
			count <= count - 1;
			if (count != 1) begin
				interval <= interval + signed_add;
				curr_interval <= interval + signed_add - 1;
			end else if (!queue_empty) begin
				count <= q_count;
				add <= q_add;
				interval <= q_interval;
				curr_interval <= q_interval - 1;
				dir <= q_dir;
				queue_rd_en <= 1;
			end else begin
				running <= 0;
			end
			if (dedge) begin
				step <= !step;
			end else begin
				step <= 1;
				step_delay <= 1;
			end
			if (dir)
				position <= position + 1;
			else
				position <= position - 1;
		end
	end

	if (step_delay) begin
		step_delay <= 0;
	end else if (!dedge && step) begin
		step <= 0;
	end

	if (reset) begin
		running <= 0;
		count <= 0;
	end
end

endmodule
