#!/usr/bin/env python3

from nmigen import *
from nmigen.lib.fifo import SyncFIFOBuffered
from random import randrange, seed
from nmigen.hdl.rec import *

class Scheduler(Elaboratable):
    def __init__(self, *, width):
        self.width    = width
        self.requests = Signal(width)
        self.grant    = Signal(width)
        self.grant_enc= Signal(range(width))
        self.valid    = Signal()
        self.capture  = Signal(width)
        self.en       = Signal()

    def elaborate(self, platform):
        m = Module()

        capture = Signal(self.width)
        m.d.comb += capture.eq(self.capture)
        with m.If(self.capture.any() == 0):
            m.d.comb += capture.eq(self.requests)

        for i in range(self.width):
            with m.If(capture & (1 << i)):
                m.d.comb += self.grant.eq(1 << i)
                m.d.comb += self.grant_enc.eq(i)
                m.d.comb += self.valid.eq(1)
                with m.If(self.en):
                    m.d.sync += self.capture.eq(capture & ~(1 << i))

        return m

class MoveQueueLayout(Layout):
    def __init__(self, *, width, channels):
        super().__init__([
            #
            # Input side ports
            #
            # writer has to hold req with all data 0 until ack goes high
            # with ack high, it has to deassert req and output the date with
            # the next clock for exactly one clock
            #
            # initiator -> movequeue
            ("in_data", unsigned(width), DIR_FANIN),
            ("in_req", unsigned(channels), DIR_NONE),
            # movequeue -> initiator
            ("in_ack", unsigned(channels), DIR_NONE),

            #
            # Output side ports
            #
            # requestor has to hold req until valid goes high
            # with valid high, it has to latch the data with the next clock
            # and deassert req
            #
            # receptor -> movequeue
            ("out_req", unsigned(channels), DIR_NONE),
            # movequeue -> receptor
            ("out_data", unsigned(width), DIR_FANOUT),
            ("out_valid", unsigned(channels), DIR_NONE),
        ])

class MoveQueue(Elaboratable):
    def __init__(self, width=72, entries_bits=9):
        # Config
        self.width = width
        self.entries_bits = entries_bits
        self.entries = 2 ** entries_bits
        self.clients = []

    def configure(self):
        # call after all modules have registered
        channels = sum(map(lambda x: x['channels'], self.clients))
        self.bus = Record(MoveQueueLayout(width=self.width, channels=channels))

    def register(self, *, channels=1):
        bus = Record(MoveQueueLayout(width=self.width, channels=channels))
        self.clients.append({ 'bus': bus, 'channels': channels })

        return bus

    def elaborate(self, platform):
        m = Module()

        entries = self.entries
        entries_bits = self.entries_bits
        channels = sum(map(lambda x: x['channels'], self.clients))

        bus = self.bus
        # stitch up busses
        m.d.comb += bus.connect(*map(lambda x: x['bus'], self.clients),
            exclude=['in_req', 'in_ack', 'out_req', 'out_valid'])
        i = 0
        for c in self.clients:
            b = c['bus']
            for j in range(c['channels']):
                m.d.comb += bus.in_req[i].eq(b.in_req[j])
                m.d.comb += bus.out_req[i].eq(b.out_req[j])
                m.d.comb += b.in_ack[j].eq(bus.in_ack[i])
                m.d.comb += b.out_valid[j].eq(bus.out_valid[i])
                i = i + 1

        # State
        slots = Memory(width=self.width, depth=entries)
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

        w_sched = Scheduler(width=channels)
        m.submodules += w_sched
        m.d.comb += w_sched.requests.eq(bus.in_req)

        r_sched = Scheduler(width=channels)
        m.submodules += r_sched
        m.d.comb += r_sched.requests.eq(bus.out_req & ~empty)

        phase = Signal()
        r_active = Signal()
        w_active = Signal()

        #
        # The Grand Plan
        #
        # writer phase one:
        #     if (free.rdy && req)
        #         elem := free.pop
        #         if (!empty) slot[elem] := data
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
        m.d.comb += slots_w.data.eq(bus.in_data)
        m.d.comb += ptrs_w.addr.eq(tails_r.data)
        m.d.comb += ptrs_w.data.eq(w_elem)
        m.d.comb += tails_w.addr.eq(w_chan)
        m.d.comb += tails_w.data.eq(w_elem)
        m.d.comb += heads_r.addr.eq(n_r_chan)
        m.d.comb += n_r_elem.eq(heads_r.data)
        m.d.sync += r_elem.eq(n_r_elem)
        m.d.comb += ptrs_r.addr.eq(n_r_elem)
        m.d.comb += slots_r.addr.eq(n_r_elem)
        m.d.comb += bus.out_data.eq(slots_r.data)
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
                #m.d.comb += slots_w.data.eq(bus.in_data)  # (*)
                m.d.comb += slots_w.en.eq(1)

                # ptrs[tail] := elem
                m.d.comb += tails_r.addr.eq(w_chan)
                #m.d.comb += ptrs_w.addr.eq(tails_r.data)  # (*)
                #m.d.comb += ptrs_w.data.eq(w_elem)        # (*)
                with m.If((empty & (1 << w_chan)) == 0):
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
                m.d.sync += w_chan.eq(w_sched.grant_enc)
                m.d.comb += bus.in_ack.eq(w_sched.grant)
                m.d.sync += w_active.eq(1)
                m.d.comb += w_sched.en.eq(1)

        with m.If(phase == 0):
            #
            # reader phase one
            #
            with m.If(r_sched.valid):
                m.d.comb += n_r_chan.eq(r_sched.grant_enc)
                m.d.sync += r_chan.eq(n_r_chan)
                m.d.comb += r_sched.en.eq(1)

                # elem := head
                #m.d.comb += heads_r.addr.eq(n_r_chan)      # (*)
                #m.d.comb += n_r_elem.eq(heads_r.data)      # (*)
                #m.d.sync += r_elem.eq(n_r_elem)            # (*)

                # trigger read of ptrs[elem]
                #m.d.comb += ptrs_r.addr.eq(n_r_elem)       # (*)

                # trigger read of slot[elem]
                #m.d.comb += slots_r.addr.eq(n_r_elem)      # (*)

                # active := true
                m.d.sync += r_active.eq(1)

        with m.If(phase == 1):
            #
            # reader phase two
            #
            with m.If(r_active):
                # out := slot[elem]
                #m.d.comb += bus.out_data.eq(slots_r.data)  # (*)
                m.d.comb += bus.out_valid.eq(1 << r_chan)

                # head := ptrs[elem]
                m.d.comb += heads_w.addr.eq(r_chan)
                m.d.comb += heads_w.data.eq(ptrs_r.data)
                m.d.comb += heads_w.en.eq(1)

                # if (elem == tail) empty := true
                m.d.comb += tails_r.addr.eq(r_chan)
                with m.If(r_elem == tails_r.data):
                    m.d.sync += empty.eq(empty | (1 << r_chan))

                # free.push(elem)
                #m.d.comb += free_fifo.w_data.eq(r_elem)     # (*)
                m.d.comb += free_fifo.w_en.eq(1)
                # we could assert free_fifo.w_rdy == 1

                m.d.sync += r_active.eq(0)

        m.d.sync += phase.eq(~phase)

        # for invariant checker
        #self.heads = heads
        #self.tails = tails
        #self.slots = slots
        #self.fifo = free_fifo
        #self.empty = empty
        #self.ptrs = ptrs
        #self.in_data = in_data
        #self.in_req = in_req
        #self.in_ack = in_ack
        #self.out_data = out_data
        #self.out_valid = out_valid
        #self.out_req = out_req

        return m

from nmigen.sim import *
from nmigen.back import verilog

if __name__ == "__main__":
    channels = 30
    entries_bits = 9
    entries = 2 ** entries_bits
    seed(0)
    mq = MoveQueue(width=72, entries_bits=entries_bits)
    bus = mq.register(channels=channels)
    mq.configure()

    def fail(msg):
        print(msg)
        raise("simulation abort")

    def wait_for_signal(sig, val):
        i = 0
        while ((yield sig) != val):
            if i == 10000:
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
                #delay = randrange(1, 10)
                yield from wait(delay)
            print("producer %d done" % chan)
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
                #delay = randrange(1, 20)
                yield from wait(delay)
            print("consumer %d done" % chan)
        return consumer

    def invariant_checker():
        yield Passive()
        while (True):
            if (yield bus.in_ack[4]) == 0 and (yield bus.out_valid[4]) == 0:
                yield
                continue
            # check chain head->tail
            # check slots in chain: correct channel, in seq
            empty = yield mq.empty
            visited = {}
            #for ch in range(channels):
            for ch in [4]:
                if (empty & (1 << ch)):
                    continue
                p = yield mq.heads[ch]
                seq = None
                chain = []
                while (p != (yield mq.tails[ch])):
                    chain.append(p)
                    if (p in visited):
                        fail("node %d already visited (ch %d)" % (p, ch))
                    visited[p] = 1
                    s = yield mq.slots[p]
                    if ((s >> 64) != ch):
                        print(chain)
                        fail("data for wrong channel in chain %d, slot %d/%x" %
                            (ch, p, s))
                    rs = (s >> 48) & 65535
                    if (seq is not None):
                        if (rs != seq + 1):
                            print(chain)
                            fail("data not in sequence (%d/%d) in chain %d, "
                                "slot %d/%x" % (rs, seq + 1, ch, p, s))
                    seq = rs
                    p = yield mq.ptrs[p]
                print("chain 4: %s" % chain)
            yield

    sim = Simulator(mq)
    sim.add_clock(1e-6)
    channels = 6
    for ch in range(channels):
        sim.add_sync_process(producer_gen(bus.in_req[ch], bus.in_ack[ch],
            bus.in_data, ch, 750))
    for ch in range(channels):
        sim.add_sync_process(consumer_gen(bus.out_req[ch], bus.out_valid[ch],
            bus.out_data, ch, 750, 0))
    #sim.add_sync_process(invariant_checker)

    #with sim.write_vcd("movequeue.vcd"):
    #    sim.run()
    with open("gen_movequeue.v", "w") as f:
        f.write(verilog.convert(mq, name='gen_movequeue', ports=[
            mq.bus.in_data, mq.bus.in_req, mq.bus.in_ack,
            mq.bus.out_data, mq.bus.out_valid, mq.bus.out_req]))
