#!/usr/bin/env python3
from nmigen import *
from nmigen.lib.fifo import SyncFIFOBuffered
from nmigen.lib.scheduler import RoundRobin
from random import randrange, seed

#
# taken from lib/scheduler and turned into an async version
#
class AsyncRoundRobin(Elaboratable):
    def __init__(self, *, count):
        if not isinstance(count, int) or count < 0:
            raise ValueError("Count must be a non-negative integer, not {!r}"
                             .format(count))
        self.count    = count

        self.requests   = Signal(count)
        self.grant      = Signal(range(count))
        self.prev_grant = Signal(range(count))
        self.valid      = Signal()

    def elaborate(self, platform):
        m = Module()

        m.d.comb += self.grant.eq(self.prev_grant)
        with m.Switch(self.prev_grant):
            for i in range(self.count):
                with m.Case(i):
                    for pred in reversed(range(i)):
                        with m.If(self.requests[pred]):
                            m.d.comb += self.grant.eq(pred)
                    for succ in reversed(range(i + 1, self.count)):
                        with m.If(self.requests[succ]):
                            m.d.comb += self.grant.eq(succ)

        m.d.comb += self.valid.eq(self.requests.any())

        return m

class MoveQueue(Elaboratable):
    def __init__(self, width=72, entries_bits=9, channels=32):
        # Config
        self.width = width
        self.entries_bits = entries_bits
        self.entries = 2 ** entries_bits
        self.channels = channels

        # Input side ports
        #
        # writer has to hold req with all data 0 until ack goes high
        # with ack high, it has to deassert req and output the date with
        # the next clock for exactly one clock
        self.in_data = Signal(width)
        self.in_req = Signal(channels)
        self.in_ack = Signal(channels)

        # Output side ports
        #
        # requestor has to hold req until valid goes high
        # with valid high, it has to latch the data with the next clock
        # and deassert req
        self.out_data = Signal(width)
        self.out_valid = Signal(channels)
        self.out_req = Signal(channels)

    def elaborate(self, platfrom):
        m = Module()

        entries = self.entries
        entries_bits = self.entries_bits
        channels = self.channels

        # State
        slots = Memory(width=self.width, depth=entries,
            init = [0xdeadbeef0000 + i for i in range(entries)])
        slots_r = slots.read_port()
        slots_w = slots.write_port()
        m.submodules += [ slots_r, slots_w]

        ptrs = Memory(width=entries_bits, depth=entries)
        ptrs_r = ptrs.read_port()
        ptrs_w = ptrs.write_port()
        m.submodules += [ ptrs_r, ptrs_w]

        # Free fifo
        free_fifo = SyncFIFOBuffered(width=entries_bits, depth=entries,
            init=[i for i in range(entries - 1)])
        m.submodules += free_fifo

        # Per-channel pointered list heads/tails
        heads = Memory(width=entries_bits, depth=channels)
        heads_r = heads.read_port(domain='comb')
        heads_w = heads.write_port()
        m.submodules += [ heads_r, heads_w ]
        tails = Memory(width=entries_bits, depth=channels)
        tails_r = tails.read_port(domain='comb')
        tails_w = tails.write_port()
        m.submodules += [ tails_r, tails_w ]

        # whether the channel list is empty
        empty = Signal(channels, reset=2 ** channels - 1)

        w_chan = Signal(range(channels))
        w_elem = Signal(entries_bits)
        n_r_chan = Signal(range(channels))
        r_chan = Signal(range(channels))
        n_r_elem = Signal(entries_bits)
        r_elem = Signal(entries_bits)

        w_sched = AsyncRoundRobin(count=channels)
        m.submodules += w_sched
        m.d.comb += w_sched.requests.eq(self.in_req)
        m.d.comb += w_sched.prev_grant.eq(w_chan)

        r_sched = AsyncRoundRobin(count=channels)
        m.submodules += r_sched
        m.d.comb += r_sched.requests.eq(self.out_req & ~empty)
        m.d.comb += r_sched.prev_grant.eq(r_chan)

        phase = Signal()
        r_active = Signal()
        w_active = Signal()

        #
        # The Grand Plan
        #
        # writer phase one:
        #     if (free.rdy && req)
        #         elem := free.pop
        #         slot[elem] := data
        #         ptrs[tail] := elem
        #         tail := elem     -- write and write in the same cycle, port
        #                          -- must not be transparent
        #         if (emtpy) head := elem
        #         empty := false
        # write phase two:
        #         nada
        # reader phase one:
        #     if (req && !empty)
        #         elem := head
        #         trigger read of ptrs[elem]
        #         trigger read of slot[elem]
        #         active := true
        # reader phase two:
        #     if active
        #         out := slot[elem]
        #         head := ptrs[elem] -- we expect the potential newly written
        #                            -- data from phase one here
        #         if (elem == tail) empty := true
        #         free.push(elem)
        #         

        # remark: the lines below are moved here, but also left in their
        # original place below, marked with (*)
        m.d.comb += slots_w.addr.eq(w_elem)
        m.d.comb += slots_w.data.eq(self.in_data)
        m.d.comb += ptrs_w.addr.eq(tails_r.data)
        m.d.comb += ptrs_w.data.eq(w_elem)
        m.d.comb += tails_w.addr.eq(w_chan)
        m.d.comb += tails_w.data.eq(w_elem)
        m.d.comb += heads_r.addr.eq(n_r_chan)
        m.d.comb += n_r_elem.eq(heads_r.data)
        m.d.sync += r_elem.eq(n_r_elem)
        m.d.comb += ptrs_r.addr.eq(n_r_elem)
        m.d.comb += slots_r.addr.eq(n_r_elem)
        m.d.comb += self.out_data.eq(slots_r.data)
        m.d.comb += free_fifo.w_data.eq(r_elem)

        with m.If(phase == 0):
            #
            # writer phase one
            #
            with m.If(w_active):
                # elem := free.pop
                m.d.comb += w_elem.eq(free_fifo.r_data)
                m.d.comb += free_fifo.r_en.eq(1)

                # slot[elem] := data
                # address and data already set up for slot
                #m.d.comb += slots_w.addr.eq(w_elem)       # (*)
                #m.d.comb += slots_w.data.eq(self.in_data) # (*)
                m.d.comb += slots_w.en.eq(1)
            
                # ptrs[tail] := elem
                m.d.comb += tails_r.addr.eq(w_chan)
                #m.d.comb += ptrs_w.addr.eq(tails_r.data)  # (*)
                #m.d.comb += ptrs_w.data.eq(w_elem)        # (*)
                m.d.comb += ptrs_w.en.eq(1)

                # tail := elem
                #m.d.comb += tails_w.addr.eq(w_chan)       # (*)
                #m.d.comb += tails_w.data.eq(w_elem)       # (*)
                m.d.comb += tails_w.en.eq(1)

                # if (emtpy) head := elem
                with m.If(empty & (1 << w_chan)):
                    m.d.comb += heads_w.addr.eq(w_chan)
                    m.d.comb += heads_w.data.eq(w_elem)
                    m.d.comb += heads_w.en.eq(1)

                # empty := false
                m.d.sync += empty.eq(empty & ~(1 << w_chan))
                m.d.sync += w_active.eq(0)

        with m.If(phase == 1):
            #
            # write phase two
            #
            with m.If(free_fifo.r_rdy & w_sched.valid):
                m.d.sync += w_chan.eq(w_sched.grant)
                m.d.comb += self.in_ack.eq(1 << w_sched.grant)
                m.d.sync += w_active.eq(1)

        with m.If(phase == 0):
            #
            # reader phase one
            #
            with m.If(r_sched.valid):
                m.d.comb += n_r_chan.eq(r_sched.grant)
                m.d.sync += r_chan.eq(n_r_chan)

                # elem := head
                #m.d.comb += heads_r.addr.eq(n_r_chan)       # (*)
                #m.d.comb += n_r_elem.eq(heads_r.data)     # (*)
                #m.d.sync += r_elem.eq(n_r_elem)           # (*)

                # trigger read of ptrs[elem]
                #m.d.comb += ptrs_r.addr.eq(n_r_elem)      # (*)

                # trigger read of slot[elem]
                #m.d.comb += slots_r.addr.eq(n_r_elem)     # (*)

                # active := true
                m.d.sync += r_active.eq(1)

        with m.If(phase == 1):
            #
            # reader phase two
            #
            with m.If(r_active):
                # out := slot[elem]
                #m.d.comb += self.out_data.eq(slots_r.data)  # (*)
                m.d.comb += self.out_valid.eq(1 << r_chan)

                # head := ptrs[elem]
                m.d.comb += heads_w.addr.eq(r_chan)
                m.d.comb += heads_w.data.eq(ptrs_r.data)
                m.d.comb += heads_w.en.eq(1)

                # if (elem == tail) empty := true
                m.d.comb += tails_r.addr.eq(r_chan)
                with m.If(r_elem == tails_r.data):
                    m.d.sync += empty.eq(empty | (1 << r_chan))

                # free.push(elem)
                #m.d.comb += free_fifo.w_data.eq(r_elem)   # (*)
                m.d.comb += free_fifo.w_en.eq(1)
                # we could assert free_fifo.w_rdy == 1

                m.d.sync += r_active.eq(0)

        m.d.sync += phase.eq(~phase)
        return m

from nmigen.sim import *
from nmigen.back import verilog

if __name__ == "__main__":
    channels = 6
    seed(0)
    mq = MoveQueue(width=72, entries_bits=9, channels=channels)

    def fail(msg):
        print(msg)
        exit(1)

    def wait_for_signal(sig, val):
        i = 0
        while ((yield sig) != val):
            if i == 2000:
                fail("signal never asserted")
            i = i + 1
            yield

    def wait(delay):
        for i in range(delay):
            yield

    def make_data(chan, seq):
        return (chan << 64) | (seq << 48) | (0xcafe << 32)

    # one process per channel
    #   send pseudo-random data with channel + seq in it
    #   delay 0-300 cycles
    #   repeat n times
    def producer_gen(in_req, in_ack, in_data, chan, n):
        def producer():
            for seq in range(n):
                data = make_data(chan, seq)
                yield in_req.eq(1)
                yield from wait_for_signal(in_ack, 1)
                yield in_data.eq(data)
                yield in_req.eq(0)
                yield
                yield in_data.eq(0)
                yield
                print("chan %d seq %x data %x" % (chan, seq, data))
                delay = randrange(1, 2 * (n - seq))
                yield from wait(delay)
        return producer

    # one process per channel
    #   receive + check data
    #   delay 0-200 cycles
    #   repeat n times
    def consumer_gen(out_req, out_valid, out_data, chan, n, initial):
        def consumer():
            for _ in range(initial):
                yield
            for seq in range(n):
                data = make_data(chan, seq)
                yield out_req.eq(1)
                yield from wait_for_signal(out_valid, 1)
                data = yield out_data
                if data != make_data(chan, seq):
                    fail("unexpected data %x on chan %d seq %d" %
                        (data, chan, seq))
                yield out_req.eq(0)
                yield
                delay = randrange(1, 2 * n)
                yield from wait(delay)
        return consumer

    sim = Simulator(mq)
    sim.add_clock(1e-6)
    channels = 6
    for ch in range(channels):
        sim.add_sync_process(producer_gen(mq.in_req[ch], mq.in_ack[ch],
            mq.in_data, ch, 750))
    for ch in range(channels):
        sim.add_sync_process(consumer_gen(mq.out_req[ch], mq.out_valid[ch],
            mq.out_data, ch, 750, 0))

    with sim.write_vcd("movequeue.vcd"):
        sim.run()
    with open("gen_movequeue.v", "w") as f:
        f.write(verilog.convert(mq, name='gen_movequeue', ports=[
            mq.in_data, mq.in_req, mq.in_ack,
            mq.out_data, mq.out_valid, mq.out_req]))
