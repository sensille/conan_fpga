#include <stdlib.h>
#include "Vconan.h"
#include "verilated.h"

int main(int argc, char **argv) {
	// Initialize Verilators variables
	Verilated::commandArgs(argc, argv);

	// Create an instance of our module under test
	Vconan *tb = new Vconan;

	// Tick the clock until we are done
	while(!Verilated::gotFinish()) {
		tb->clk_50mhz = 1;
		tb->eval();
		tb->clk_50mhz = 0;
		tb->eval();
		printf("leds_out: %d leds_clk: %d leds_cs: %d\n",
			tb->exp2_17,
			tb->exp2_15,
			tb->exp2_16);
	} exit(EXIT_SUCCESS);
}
