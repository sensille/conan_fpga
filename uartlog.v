`timescale 1ns / 1ps
`default_nettype none

module uartlog #(
	parameter HZ = 0
) (
	input wire clk,
	input wire [63:0] systime,

	input wire [7:0] tx_data,
	input wire tx_en,
	input wire [7:0] rx_data,
	input wire rx_ready,

	output reg [31:0] daq_data,
	output reg daq_end,
	output reg daq_valid = 0,
	output reg daq_req = 0,
	input wire daq_grant
);

localparam DAQT_MCU_RX		= 8;
localparam DAQT_MCU_RX_LONG	= 9;
localparam DAQT_MCU_TX		= 10;
localparam DAQT_MCU_TX_LONG	= 11;
localparam RX = 0;
localparam TX = 1;
reg [1:0] latched;
reg [7:0] latched_data[2];
reg [47:0] latched_time[2];
reg ix;

localparam ST_IDLE		= 0;
localparam ST_WAIT_GRANT	= 1;
localparam ST_SEND		= 2;
localparam ST_SEND_TIME_HIGH	= 3;
localparam ST_MAX		= 3;
localparam ST_BITS = $clog2(ST_MAX + 1);
reg [ST_BITS-1:0] state = ST_IDLE;

reg [31:0] last_time_high = ~0;
always @(posedge clk) begin
	daq_valid <= 0;
	daq_end <= 0;
	if (tx_en) begin
		latched[TX] <= 1;
		latched_data[TX] <= tx_data;
		latched_time[TX] <= systime[47:0];
	end
	if (rx_ready) begin
		latched[RX] <= 1;
		latched_data[RX] <= rx_data;
		latched_time[RX] <= systime[47:0];
	end
	if (state == ST_IDLE && latched) begin
		state <= ST_WAIT_GRANT;
		daq_req <= 1;
	end else if (state == ST_WAIT_GRANT && daq_grant) begin
		daq_req <= 0;
		if (latched[RX])
			ix <= RX;
		else
			ix <= TX;
		state <= ST_SEND;
	end else if (state == ST_SEND) begin
		daq_data[23:16] <= latched_data[ix];
		daq_data[15:0] <= latched_time[ix][15:0];
		daq_valid <= 1;
		latched[ix] <= 0;
		if (last_time_high != latched_time[ix][47:16]) begin
			last_time_high <= latched_time[ix][47:16];
			if (ix == RX)
				daq_data[31:24] <= DAQT_MCU_RX_LONG;
			else
				daq_data[31:24] <= DAQT_MCU_TX_LONG;
			state <= ST_SEND_TIME_HIGH;
		end else begin
			if (ix == RX)
				daq_data[31:24] <= DAQT_MCU_RX;
			else
				daq_data[31:24] <= DAQT_MCU_TX;
			daq_end <= 1;
			state <= ST_IDLE;
		end
	end else if (state == ST_SEND_TIME_HIGH) begin
			daq_data <= latched_time[ix][47:16];
			daq_valid <= 1;
			daq_end <= 1;
			state <= ST_IDLE;
	end
end

endmodule
