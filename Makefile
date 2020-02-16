TARGET = conan

all: $(TARGET).bit

VINC=/usr/local/share/verilator/include

SRC = led7219.v pll.v uart.v framing.v fifo.v command.v pwm.v $(TARGET).v

$(TARGET).json: $(SRC) $(TARGET).lpf Makefile
	yosys -p "synth_ecp5 -json $(TARGET).json" $(SRC)

%_out.config: %.json $(TARGET).lpf
	nextpnr-ecp5 --json $< --textcfg $@ --45k --package CABGA256 --lpf $(TARGET).lpf

%.bit: %_out.config
#	ecppack --idcode 0x21111043 --svf-rowsize 100000 --svf $(TARGET).svf $< $@
	ecppack --compress --svf-rowsize 100000 --svf $(TARGET).svf $< $@

verilate:
	verilator -Wall -cc --exe $(TARGET).v tb.cpp
	make -j 4 -C obj_dir -f V$(TARGET).mk
	obj_dir/V$(TARGET)

.PRECIOUS: $(TARGET).json $(TARGET)_out.config
