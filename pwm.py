#!/usr/bin/env python3

from nmigen import *
from cmdbus import *
from movequeue import *

class PWM(Elaboratable):
    def __init__(self, cmdbus, movequeue, npwm=4, pwm_bits = 26):
        # Config
        self.npwm = npwm
        self.pwm_bits = pwm_bits

        # Ports
        self.pwm = Signal(npwm)
        self.missed_clock = Signal()

        device = cmdbus.register(self)
        self.cmd = device.signals()

        # Register our command
        self.CMD_CONFIG = device.define_cmd('CMD_CONFIG_PWM',
            ['channel', 'value', 'default_value', 'max_duration'])
        self.CMD_SCHEDULE = device.define_cmd('CMD_SCHEDULE_PWM',
            ['channel', 'clock', 'on_ticks', 'off_ticks'])

        if movequeue.width < 2 * pwm_bits + 32:
            raise RuntimeError('movequeue not wide enough for PWM module')
        self.mq = movequeue.register(channels=npwm)

    def elaborate(self, platfrom):
        m = Module()
        npwm = self.npwm
        pwm_bits = self.pwm_bits
        pwm = self.pwm
        cb = self.cmd
        mq = self.mq

        # State
        on_ticks = Array([Signal(pwm_bits) for _ in range(npwm)])
        off_ticks = Array([Signal(pwm_bits, reset = 1) for _ in range(npwm)])
        next_on_ticks = Array([Signal(pwm_bits) for _ in range(npwm)])
        next_off_ticks = Array([Signal(pwm_bits) for _ in range(npwm)])
        next_time = Array([Signal(32, name=f"next_time{i}")
            for i in range(npwm)])
        scheduled = Array([Signal(1, name=f"scheduled{i}")
            for i in range(npwm)])
        default_value = Array([Signal(1) for _ in range(npwm)])
        max_duration = Array([Signal(32) for _ in range(npwm)])
        duration = Array([Signal(32, name=f"duration{i}") for i in range(npwm)])
        toggle_cnt = Array([Signal(pwm_bits, reset = 1) for _ in range(npwm)])
        channel = Signal(range(npwm))

        next_time_in = Signal(32)
        next_on_ticks_in = Signal(pwm_bits)
        next_off_ticks_in = Signal(pwm_bits)

        m.d.comb += cb.arg_advance.eq(1)
        m.d.comb += cb.param_data.eq(0)
        m.d.comb += cb.param_write.eq(0)
        m.d.comb += cb.invol_req.eq(0)

        with m.If(cb.cmd_done == 1):
            m.d.sync += cb.cmd_done.eq(0)

        for i in range(npwm):
            with m.If(scheduled[i] == 0):
                with m.If(mq.out_valid.bit_select(i, 1)):
                    m.d.sync += mq.out_req.bit_select(i, 1).eq(0)
                    m.d.sync += Cat(next_on_ticks[i], next_off_ticks[i],
                        next_time[i]).eq(mq.out_data)
                    m.d.sync += scheduled[i].eq(1)
                with m.Else():
                    m.d.sync += mq.out_req.bit_select(i, 1).eq(1)

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

            with m.If(scheduled[i] & (next_time[i] == cb.systime[:32])):
                m.d.sync += [
                    on_ticks[i].eq(next_on_ticks[i]),
                    off_ticks[i].eq(next_off_ticks[i]),
                    duration[i].eq(max_duration[i]),
                    scheduled[i].eq(0),
                ]
            with m.Elif(duration[i] != 0):
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
        m.d.sync += mq.in_data.eq(0)
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
                m.d.sync += next_time_in.eq(cb.arg_data)
                #with m.If (((cb.arg_data - cb.systime[:32]) >= 0xc0000000) |
                #           ((cb.arg_data - cb.systime[:32]) == 0x00000000)):
                #    m.d.sync += self.missed_clock.eq(1)
                m.next = 'SCHEDULE_2'
            with m.State('SCHEDULE_2'):
                m.d.sync += next_on_ticks_in.eq(cb.arg_data)
                m.next = 'SCHEDULE_3'
            with m.State('SCHEDULE_3'):
                m.d.sync += next_off_ticks_in.eq(cb.arg_data)
                #m.d.sync += scheduled[channel].eq(1)
                m.d.sync += cb.cmd_done.eq(1)
                m.d.sync += mq.in_req.bit_select(channel, 1).eq(1)
                m.next = 'SCHEDULE_4'
            with m.State('SCHEDULE_4'):
                with m.If(mq.in_ack.any()):
                    m.d.sync += mq.in_data.eq(Cat(next_on_ticks_in,
                        next_off_ticks_in, next_time_in))
                    m.d.sync += mq.in_req.eq(0)
                    m.next = 'IDLE'

        return m

from nmigen.sim import *
from nmigen.back import verilog

if __name__ == "__main__":
    class Top(Elaboratable):
        def __init__(self, mods=[], ports=[]):
            self.mods = mods
            self.ports = ports
            for (port, name, dir) in ports:
                setattr(self, port.name, port)
        def elaborate(self, platform):
            m = Module()
            m.submodules += self.mods
            return m

    class PwmTop(Elaboratable):
        def __init__(self, pwm, mq):
            self.m_pwm = pwm
            self.m_mq = mq

            self.systime = Signal(64)
            self.arg_data = Signal(32)
            self.arg_advance = Signal()
            self.cmd = Signal(pwm.cmd.cmd.shape())
            self.cmd_ready = Signal()
            self.cmd_done = Signal()
            self.param_data = Signal(32)
            self.param_write = Signal()
            self.invol_req = Signal()
            self.invol_grant = Signal()
            self.pwm = Signal(pwm.npwm)
            self.shutdown = Signal()
            self.missed_clock = Signal()
            
        def elaborate(self, platform):
            pwm = self.m_pwm
            cmd = pwm.cmd

            m = Module()
            m.submodules += [self.m_pwm, self.m_mq]

            m.d.comb += [
                cmd.systime.eq(self.systime),
                cmd.arg_data.eq(self.arg_data),
                self.arg_advance.eq(cmd.arg_advance),
                cmd.cmd.eq(self.cmd),
                cmd.cmd_ready.eq(self.cmd_ready),
                self.cmd_done.eq(cmd.cmd_done),
                self.param_data.eq(cmd.param_data),
                self.param_write.eq(cmd.param_write),
                self.invol_req.eq(cmd.invol_req),
                cmd.invol_grant.eq(self.invol_grant),
                self.pwm.eq(pwm.pwm),
                cmd.shutdown.eq(self.shutdown),
                self.missed_clock.eq(pwm.missed_clock),
            ]
        
            return m

    cmdbus = CmdBusMaster(first_cmd=3)
    #mq = MoveQueue(width=72)
    mq = MoveQueue(width=2 * 26 + 32)
    npwm = 12
    pwm = PWM(cmdbus, mq, npwm = npwm)
    mq.configure()
    systime = cmdbus.systime_sig()

    top = Top(mods=[cmdbus, mq])
    pwmtop = PwmTop(pwm, mq)

    def fail(msg):
        raise RuntimeError(msg)

    def wait_for_signal(sig, val):
        i = 0
        while ((yield sig) != val):
            assert(i < 10000)
            if i == 10000:
                fail("signal never asserted")
            i = i + 1
            yield

    def test_pwm_check_cycle(systime, pwm, duty, period):
        yield from wait_for_signal(pwm, 0)
        print("0: now is %d" % (yield systime))
        yield from wait_for_signal(pwm, 1)
        print("1: now is %d" % (yield systime))
        for i in range(3):
            curr = yield systime
            yield from wait_for_signal(pwm, 0)
            print("c0: now is %d diff %d" % ((yield systime),
                (yield systime) - curr))
            if (yield systime) - curr != duty:
                fail("pwm duty period mismatch, expected %d, got %d" %
                    (duty, (yield systime) - curr))
            yield from wait_for_signal(pwm, 1)
            print("c1: now is %d diff %d" % ((yield systime), (yield systime) - curr))
            if (yield systime) - curr != period:
                fail("pwm period mismatch, expected %d, got %d" %
                    (period, (yield systime) - curr))
        print("passed %d/%d" % (duty, period))

    def sim_main():
        for i in range(20):
            yield

#        for i in range(npwm):
        for i in range(2):
            assert (yield pwm.pwm[i]) == 0
            # channel, value, default_value, max_duration
            yield from cmdbus.sim_write('CMD_CONFIG_PWM',
                channel = i, value = 1, default_value = 0, max_duration = 1000)

            # check pwm value and that it stays on it
            for _ in range(1000):
                assert (yield pwm.pwm[i]) == 1
                yield

            now = yield cmdbus.master.systime
            yield from cmdbus.sim_write('CMD_SCHEDULE_PWM',
                channel = i, clock = now + 50, on_ticks = 10, off_ticks = 90)

            yield from test_pwm_check_cycle(systime, pwm.pwm[i], 10, 100);

            sched = (yield systime) + 400
            yield from cmdbus.sim_write('CMD_SCHEDULE_PWM',
                channel = i, clock = sched, on_ticks = 55, off_ticks = 45)
            yield from test_pwm_check_cycle(systime, pwm.pwm[i], 10, 100);
            for _ in range(300):
                yield
            yield from test_pwm_check_cycle(systime, pwm.pwm[i], 55, 100);
            for _ in range(500):
                yield
            # wait until max duration (+one cycle) is over.
            # pwm should be set to default
            for _ in range(101):
                assert (yield pwm.pwm[i]) == 0
                yield

            sched = (yield systime) + 300
            yield from cmdbus.sim_write('CMD_SCHEDULE_PWM',
                channel = i, clock = sched, on_ticks = 11, off_ticks = 89)
            for _ in range(300):
                yield
            yield from test_pwm_check_cycle(systime, pwm.pwm[i], 11, 100);

            # test always on
            sched = (yield systime) + 300
            yield from cmdbus.sim_write('CMD_SCHEDULE_PWM',
                channel = i, clock = sched, on_ticks = 1, off_ticks = 0)
            for _ in range(300):
                yield
            for _ in range(101):
                assert (yield pwm.pwm[i]) == 1
                yield

            # test always off
            sched = (yield systime) + 300
            yield from cmdbus.sim_write('CMD_SCHEDULE_PWM',
                channel = i, clock = sched, on_ticks = 0, off_ticks = 1)
            for _ in range(300):
                yield
            for _ in range(101):
                assert (yield pwm.pwm[i]) == 0
                yield

            # same again
            sched = (yield systime) + 300
            yield from cmdbus.sim_write('CMD_SCHEDULE_PWM',
                channel = i, clock = sched, on_ticks = 0, off_ticks = 1)
            for _ in range(300):
                yield
            for _ in range(101):
                assert (yield pwm.pwm[i]) == 0
                yield

            sched = (yield systime) + 300
            yield from cmdbus.sim_write('CMD_SCHEDULE_PWM',
                channel = i, clock = sched, on_ticks = 22, off_ticks = 78)
            for _ in range(300):
                yield
            yield from test_pwm_check_cycle(systime, pwm.pwm[i], 22, 100);

    sim = Simulator(top)
    sim.add_clock(1e-6)
    sim.add_sync_process(sim_main)
    cmdbus.sim_add_background(sim)

    with open("gen_pwm.v", "w") as f:
        f.write(verilog.convert(pwmtop, name='gen_pwm', ports=[
            pwmtop.systime,
            pwmtop.arg_data,
            pwmtop.arg_advance,
            pwmtop.cmd,
            pwmtop.cmd_ready,
            pwmtop.cmd_done,
            pwmtop.param_data,
            pwmtop.param_write,
            pwmtop.invol_req,
            pwmtop.invol_grant,
            pwmtop.pwm,
            pwmtop.shutdown,
            pwmtop.missed_clock,
        ]))
    #with open("gen_pwm.v", "w") as f:
    #    ports = [pwm.pwm, pwm.missed_clock]
    #    ports.extend(pwm.cmd.as_value().parts)
    #    f.write(verilog.convert(pwm, name='gen_pwm', ports=ports))
    with sim.write_vcd("pwm.vcd"):
        sim.run()
