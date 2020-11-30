TARGET = conan
LDLIBS = -lpcre

all: $(TARGET).bit

VINC=/usr/local/share/verilator/include

SRC = led7219.v pll.v uart.v framing.v fifo.v command.v pwm.v system.v \
       stepper.v stepdir.v tmcuart.v gpio.v dro.v as5311.v sd.v sdc.v \
       mac.v ether.v daq.v $(TARGET).v

DAQ_SRC = mac.v ether.v daq.v tb_daq.v

$(TARGET).json: $(SRC) $(TARGET).lpf Makefile
	yosys -q -f "verilog -defer" -p "synth_ecp5 -top $(TARGET) -json $(TARGET).json" $(SRC)
	#yosys -f "verilog -defer" -p "synth_ecp5 -top $(TARGET) -json $(TARGET).json" $(SRC)

%_out.config: %.json $(TARGET).lpf
	nextpnr-ecp5 -q -l nextpnr.log --json $< --textcfg $@ --45k --package CABGA256 --lpf $(TARGET).lpf

%.bit: %_out.config
#	ecppack --idcode 0x21111043 --svf-rowsize 100000 --svf $(TARGET).svf $< $@
	ecppack --compress --svf-rowsize 100000 --svf $(TARGET).svf $< $@

verilate: vrun
v: vrun
test: vrun

obj_dir/$(TARGET).mk: $(SRC) Makefile
	verilator --public -Wall -CFLAGS -g --exe --cc $(TARGET).v verilator.vlt tb.cpp

obj_dir/V$(TARGET)__ALL.a: obj_dir/$(TARGET).mk
	make -j 4 -C obj_dir -f V$(TARGET).mk V$(TARGET)__ALL.a

obj_dir/vsyms.h: obj_dir/$(TARGET).mk
	./gensyms.pl $(TARGET) obj_dir/V$(TARGET).h obj_dir/vsyms.h
	touch obj_dir/vsyms.h

obj_dir/V$(TARGET): obj_dir/vsyms.h
	LDLIBS=$(LDLIBS) make -C obj_dir -f V$(TARGET).mk

# daq testbench
obj_dir_daq/tb_daq.mk: $(DAQ_SRC) Makefile
	verilator --public -Wall -Mdir obj_dir_daq -CFLAGS -g --exe --cc tb_daq.v verilator.vlt tb_daq.cpp

obj_dir_daq/Vtb_daq__ALL.a: obj_dir_daq/tb_daq.mk
	make -j 4 -C obj_dir_daq -f Vtb_daq.mk Vtb_daq__ALL.a

obj_dir_daq/vsyms.h: obj_dir_daq/tb_daq.mk
	./gensyms.pl tb_daq obj_dir_daq/Vtb_daq.h obj_dir_daq/vsyms.h
	touch obj_dir_daq/vsyms.h

obj_dir_daq/Vtb_daq: obj_dir_daq/vsyms.h
	LDLIBS=$(LDLIBS) make -C obj_dir_daq -f Vtb_daq.mk

vrun: obj_dir/Vconan obj_dir_daq/Vtb_daq
	obj_dir_daq/Vtb_daq
	obj_dir/V$(TARGET)

.PRECIOUS: $(TARGET).json $(TARGET)_out.config
