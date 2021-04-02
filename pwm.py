from nmigen import *
from cmdbus import *

class PWM(Elaboratable):
    def __init__(self, cmdbus, npwm=4, pwm_bits = 26):
        # Config
        self.npwm = npwm
        self.pwm_bits = pwm_bits

        # Ports
        self.pwm = Signal(npwm)
        self.missed_clock = Signal()

        device = cmdbus.register(self)
        self.cmd = device.signals()

        # Register our command
        self.CMD_CONFIG = device.define_cmd('CMD_CONFIG_PWM', nargs=4)
        self.CMD_SCHEDULE = device.define_cmd('CMD_SCHEDULE_PWM', nargs=4)

    def elaborate(self, platfrom):
        m = Module()
        npwm = self.npwm
        pwm_bits = self.pwm_bits
        pwm = self.pwm
        cb = self.cmd

        # State
        on_ticks = Array([Signal(pwm_bits) for _ in range(npwm)])
        off_ticks = Array([Signal(pwm_bits, reset = 1) for _ in range(npwm)])
        next_on_ticks = Array([Signal(pwm_bits) for _ in range(npwm)])
        next_off_ticks = Array([Signal(pwm_bits) for _ in range(npwm)])
        next_time = Array([Signal(32) for _ in range(npwm)])
        scheduled = Array([Signal(1) for _ in range(npwm)])
        default_value = Array([Signal(1) for _ in range(npwm)])
        max_duration = Array([Signal(32) for _ in range(npwm)])
        duration = Array([Signal(32) for _ in range(npwm)])
        toggle_cnt = Array([Signal(pwm_bits, reset = 1) for _ in range(npwm)])
        channel = Signal(range(npwm))

        m.d.comb += cb.arg_advance.eq(1)

        with m.If(cb.cmd_done == 1):
            m.d.sync += cb.cmd_done.eq(0)

        for i in range(npwm):
            with m.If(toggle_cnt[i] == 1):
                with m.If(pwm[i] == 0):
                    with m.If(on_ticks[i] != 0):
                        m.d.sync += toggle_cnt[i].eq(on_ticks[i])
                        m.d.sync += pwm[i].eq(1)
                    with m.Else():
                        m.d.sync += toggle_cnt[i].eq(off_ticks[i])
                with m.Else():
                    with m.If(off_ticks[i] != 0):
                        with m.If(off_ticks[i] != 0):
                            m.d.sync += toggle_cnt[i].eq(off_ticks[i])
                            m.d.sync += pwm[i].eq(0)
                    with m.Else():
                        m.d.sync += toggle_cnt[i].eq(on_ticks[i])
            with m.Else():
                m.d.sync += toggle_cnt[i].eq(toggle_cnt[i] - 1)

            with m.If(scheduled[i].bool() &
                    (next_time[i] == cb.systime[:32])):
                m.d.sync += [
                    on_ticks[i].eq(next_on_ticks[i]),
                    off_ticks[i].eq(next_off_ticks[i]),
                    duration[i].eq(max_duration[i]),
                    scheduled[i].eq(0),
                ]
            with m.Else():
                m.d.sync += duration[i].eq(duration[i] - 1)

        #
        # pwm duration safety feature. Must be before state
        # machine because pwm_duration is set on load.
        # duration = 0 turns this check off
        #
        for i in range(npwm):
            with m.If(cb.shutdown.bool() | (duration[i] == 1)):
                with m.If(default_value[i]):
                    m.d.sync += on_ticks[i].eq(1),
                    m.d.sync += off_ticks[i].eq(0),
                with m.Else():
                    m.d.sync += on_ticks[i].eq(0),
                    m.d.sync += off_ticks[i].eq(1),

        #
        # cmdbus state machine
        #
        with m.FSM():
            with m.State('IDLE'):
                with m.If(cb.cmd_ready):
                    # common to all cmds
                    m.d.sync += channel.eq(cb.arg_data)
                    with m.Switch(cb.cmd):
                        with m.Case(self.CMD_CONFIG):
                            m.next = 'CONFIG_1'
                        with m.Case(self.CMD_SCHEDULE):
                            m.next = 'SCHEDULE_1'
                        with m.Default():
                            m.d.sync += cb.cmd_done.eq(1)
            with m.State('CONFIG_1'):
                with m.If(cb.arg_data[0]):
                    m.d.sync += on_ticks[channel].eq(1)
                    m.d.sync += off_ticks[channel].eq(0)
                with m.Else():
                    m.d.sync += on_ticks[channel].eq(0)
                    m.d.sync += off_ticks[channel].eq(1)
                m.next = 'CONFIG_2'
            with m.State('CONFIG_2'):
                m.d.sync += default_value[channel].eq(cb.arg_data[0])
                m.next = 'CONFIG_3'
            with m.State('CONFIG_3'):
                m.d.sync += max_duration[channel].eq(cb.arg_data)
                m.d.sync += cb.cmd_done.eq(1)
                m.next = 'IDLE'
            with m.State('SCHEDULE_1'):
                m.d.sync += next_time[channel].eq(cb.arg_data)
                with m.If ((cb.arg_data - cb.systime >= 0xc0000000).bool() |
                           (cb.arg_data - cb.systime == 0x00000000).bool()):
                    m.d.sync += self.missed_clock.eq(1)
                m.next = 'SCHEDULE_2'
            with m.State('SCHEDULE_2'):
                m.d.sync += next_on_ticks[channel].eq(cb.arg_data)
                m.next = 'SCHEDULE_3'
            with m.State('SCHEDULE_3'):
                m.d.sync += next_off_ticks[channel].eq(cb.arg_data)
                m.d.sync += scheduled[channel].eq(1)
                m.d.sync += cb.cmd_done.eq(1)
                m.next = 'IDLE'

        return m

from nmigen.sim import *
from nmigen.back import verilog

if __name__ == "__main__":
    cmdbus = CmdBusMaster()
    npwm = 6
    pwm = PWM(cmdbus, npwm = npwm)

    def proc():
        for i in range(20):
            yield

        for i in range(npwm):
            assert (yield pwm.pwm[i]) == 0
            # channel, value, default_value, max_duration
            yield from cmdbus.sim_write('CMD_CONFIG_PWM', [i, 1, 0, 10000])
            # check pwm is one
            for _ in range(1000):
                assert (yield pwm.pwm[i]) == 1
                yield
        for i in range(20):
            yield

    sim = Simulator(cmdbus)
    sim.add_clock(1e-6)
    sim.add_sync_process(proc)
    cmdbus.sim_add_background(sim)

    with sim.write_vcd("pwm.vcd"):
        sim.run()
#    with open("gen_pwm.v", "w") as f:
#        #f.write(verilog.convert(dut, ports=[dut.pwm].extend(dut.cmd()))
#        f.write(verilog.convert(dut, ports=[dut.pwm]))
