#!/usr/bin/env python3

from nmigen import *
from nmigen.lib.coding import Encoder
from cmdbus import *
from movequeue import *
import math;

class PWM(Elaboratable):
    def __init__(self, cmdbus, movequeue, npwm=4, pwm_bits = 26):
        # Config
        self.npwm = npwm
        self.pwm_bits = pwm_bits
        self.npwm_bits = int(math.log(npwm, 2))

        if (bin(npwm).count("1") != 1):
            raise ValueError("npwm must be power of 2")

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

        if movequeue.width < 2 * pwm_bits + 32 - self.npwm_bits:
            raise RuntimeError('movequeue not wide enough for PWM module')
        self.mq = movequeue.register(channels=npwm)

    def elaborate(self, platfrom):
        m = Module()
        npwm = self.npwm
        npwm_bits = self.npwm_bits
        if (2 ** npwm_bits != npwm):
            raise ValueError("math error with npwm_bits")
        pwm_bits = self.pwm_bits
        pwm = self.pwm
        cb = self.cmd
        mq = self.mq
        upper = npwm_bits

        class ConfLayout(Layout):
            def __init__(self):
                super().__init__([
                    ("max_duration", unsigned(32 - upper)),
                    ("default_value", unsigned(1)),
                ])

        class NextLayout(Layout):
            def __init__(self):
                super().__init__([
                    ("on_ticks", unsigned(pwm_bits)),
                    ("off_ticks", unsigned(pwm_bits)),
# XXX no need to save lower bits
                    ("time", unsigned(32)),
                ])

        class StateLayout(Layout):
            def __init__(self):
                super().__init__([
                    ("on_ticks", unsigned(pwm_bits)),
                    ("off_ticks", unsigned(pwm_bits)),
                    ("toggle_at", unsigned(32)),
                    ("duration", unsigned(32 - upper)),
                    ("val", unsigned(1)),
                    ("running", unsigned(1)),
                ])

        class ToggleLayout(Layout):
            def __init__(self):
                super().__init__([
                    ("thresh", unsigned(upper)),
                    ("val", unsigned(1)),
                    ("en", unsigned(1)),
                ])

        # Config
        # read in cycle loop, written from config
        conf = Record(ConfLayout())
        conf_mem = Memory(width=len(conf), depth=npwm)
        conf_r = conf_mem.read_port(domain='comb')
        conf_w = conf_mem.write_port()
        m.submodules += [conf_r, conf_w]

        # Next
        # read in cycle loop, written from movequeue fetcher
        next = Record(NextLayout())
        next_mem = Memory(width=len(next), depth=npwm)
        next_r = next_mem.read_port(domain='comb')
        next_w = next_mem.write_port()
        m.submodules += [next_r, next_w]

        # State
        # read and updated in cycle loop
        state = Record(StateLayout())
        state_mem = Memory(width=len(state), depth=npwm)
        state_r = state_mem.read_port(domain='comb')
        state_w = state_mem.write_port()
        m.submodules += [state_r, state_w]

        # one-shot: 
        toggle = Record(ToggleLayout())
        toggle_mem = Memory(width=len(toggle), depth=npwm)
        toggle_r = []
        toggle_w = toggle_mem.write_port()
        m.submodules += toggle_w
        for _ in range(self.npwm):
            r = toggle_mem.read_port(domain='comb')
            toggle_r.append(r)
            m.submodules += r
        for channel in range(npwm):
            m.d.comb += toggle_r[channel].addr.eq(channel)


        #
        # Fetch updates from movequeue
        #
        out_req = Signal(npwm, reset = 2 ** npwm - 1)
        m.d.comb += mq.out_req.eq(out_req)
        valid_ch = Signal(npwm_bits)
        valid_enc = Encoder(npwm)
        m.submodules += valid_enc
        m.d.comb += valid_enc.i.eq(mq.out_valid)
        m.d.comb += next_w.addr.eq(valid_enc.o)
        m.d.comb += next_w.data.eq(mq.out_data)
        with m.If(~valid_enc.n):
            m.d.comb += next_w.en.eq(1)
            m.d.sync += out_req.bit_select(valid_enc.o, 1).eq(0)

        #
        # cycle through all channels, advancing one per clock
        #
        curr = Signal(range(self.npwm))
        m.d.comb += curr.eq(systime[:upper])

        state_next = Record(StateLayout())
        m.d.comb += conf_r.addr.eq(curr)
        m.d.comb += conf.eq(conf_r.data)
        m.d.comb += next_r.addr.eq(curr)
        m.d.comb += next.eq(next_r.data)
        m.d.comb += state_r.addr.eq(curr)
        m.d.comb += state_w.addr.eq(curr)
        m.d.comb += state_w.en.eq(1)
        m.d.comb += state.eq(state_r.data)
        m.d.comb += state_w.data.eq(state_next)
        m.d.comb += toggle_w.addr.eq(curr)
        m.d.comb += toggle_w.data.eq(toggle)
        m.d.comb += toggle_w.en.eq(1)
        m.d.comb += state_next.eq(state)

        pwm_next = Signal(pwm.shape())
        m.d.comb += pwm_next.eq(pwm)

        base_toggle_at = Signal(state.toggle_at.shape())
        m.d.comb += base_toggle_at.eq(state.toggle_at)

        # check reload
        lookahead = Signal()
        with m.If(next.time[:upper] <= curr):
            m.d.comb += lookahead.eq(1)
        with m.If((out_req.bit_select(curr, 1) == 0) &
                  (next.time[upper:] == systime[upper:32] + lookahead)):
            m.d.comb += state_next.on_ticks.eq(next.on_ticks)
            m.d.comb += state_next.off_ticks.eq(next.off_ticks)
            m.d.comb += state_next.duration.eq(conf.max_duration)
            m.d.sync += out_req.bit_select(curr, 1).eq(1)
            with m.If(next.on_ticks == 0):
                m.d.comb += state_next.running.eq(0)
                m.d.comb += toggle.en.eq(1)
                m.d.comb += toggle.val.eq(0)
                m.d.comb += base_toggle_at.eq(next.time)
            with m.Elif(next.off_ticks == 0):
                m.d.comb += state_next.running.eq(0)
                m.d.comb += toggle.en.eq(1)
                m.d.comb += toggle.val.eq(1)
                m.d.comb += base_toggle_at.eq(next.time)
            with m.Elif(state.running == 0):
                m.d.comb += state_next.running.eq(1)
                # XXX
                m.d.comb += state_next.val.eq(pwm.bit_select(curr, 1))
                m.d.comb += base_toggle_at.eq(next.time)
                m.d.comb += state_next.toggle_at.eq(next.time)
        # check max_duration expired
        with m.Elif(state.duration == 1):
            m.d.comb += pwm_next.bit_select(curr, 1).eq(conf.default_value)
            m.d.comb += state_next.running.eq(0)
        with m.Else():
            m.d.comb += state_next.duration.eq(state.duration - 1)

        m.d.comb += toggle.thresh.eq(base_toggle_at[:upper])

        # PWM running
        lookahead2 = Signal()
        with m.If(base_toggle_at[:upper] <= curr):
            m.d.comb += lookahead2.eq(1)
        with m.If(state_next.running &
                (base_toggle_at[upper:] == (systime[upper:32] + lookahead2))):
            m.d.comb += toggle.en.eq(1)
            # reload toggle_cnt to on_ticks/off_ticks
            m.d.comb += state_next.val.eq(~state.val)
            m.d.comb += toggle.val.eq(state_next.val)
            with m.If(state.val):
                m.d.comb += state_next.toggle_at.eq(
                    base_toggle_at + state_next.off_ticks)
            with m.Else():
                m.d.comb += state_next.toggle_at.eq(
                    base_toggle_at + state_next.on_ticks)

        #
        # check all one-shot timers each cycle
        #
        for channel in range(self.npwm):
            t = Record(ToggleLayout())
            m.d.comb += t.eq(toggle_r[channel].data)
            with m.If(t.en):
                with m.If(t.thresh == systime[:upper]):
                    m.d.comb += pwm_next[channel].eq(t.val)
            
        m.d.sync += pwm.eq(pwm_next)

        next_time_in = Signal(32)
        next_on_ticks_in = Signal(pwm_bits)
        next_off_ticks_in = Signal(pwm_bits)

        m.d.comb += cb.arg_advance.eq(1)
        m.d.comb += cb.param_data.eq(0)
        m.d.comb += cb.param_write.eq(0)
        m.d.comb += cb.invol_req.eq(0)

        with m.If(cb.cmd_done == 1):
            m.d.sync += cb.cmd_done.eq(0)

        #
        # cmdbus state machine
        #
        channel = Signal(range(npwm))
        conf_next = Record(ConfLayout())
        m.d.comb += conf_w.addr.eq(channel)
        m.d.comb += conf_w.data.eq(conf_next)
        m.d.sync += conf_w.en.eq(0)
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
                m.d.sync += pwm.bit_select(channel, 1).eq(cb.arg_data[0])
                m.next = 'CONFIG_2'
            with m.State('CONFIG_2'):
                m.d.sync += conf_next.default_value.eq(cb.arg_data[0])
                m.next = 'CONFIG_3'
            with m.State('CONFIG_3'):
                m.d.sync += conf_next.max_duration.eq(cb.arg_data[upper:])
                m.d.sync += conf_w.en.eq(1)
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
                m.d.sync += mq.in_req.bit_select(channel, 1).eq(1)
                m.next = 'SCHEDULE_4'
            with m.State('SCHEDULE_4'):
                m.d.sync += cb.cmd_done.eq(1)
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
    npwm = 16
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

        for i in range(npwm):
#        for i in [4]:
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
                channel = i, clock = now + 50, on_ticks = 20, off_ticks = 80)

            yield from test_pwm_check_cycle(systime, pwm.pwm[i], 20, 100)

            sched = (yield systime) + 400
            print("writing second schedule at %d" % (yield systime))
            yield from cmdbus.sim_write('CMD_SCHEDULE_PWM',
                channel = i, clock = sched, on_ticks = 55, off_ticks = 45)
            yield from test_pwm_check_cycle(systime, pwm.pwm[i], 20, 100)
            for _ in range(300):
                yield
            yield from test_pwm_check_cycle(systime, pwm.pwm[i], 55, 100)
            for _ in range(500):
                yield
            # wait until max duration (+one cycle) is over.
            # pwm should be set to default
            for _ in range(101):
                assert (yield pwm.pwm[i]) == 0
                yield

            sched = (yield systime) + 300
            yield from cmdbus.sim_write('CMD_SCHEDULE_PWM',
                channel = i, clock = sched, on_ticks = 21, off_ticks = 79)
            for _ in range(300):
                yield
            yield from test_pwm_check_cycle(systime, pwm.pwm[i], 21, 100);

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
