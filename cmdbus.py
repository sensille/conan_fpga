from nmigen import *
from nmigen.sim import *
from nmigen.hdl.rec import *
from collections import namedtuple

class CmdBusLayout(Layout):
    def __init__(self):
        super().__init__([
            # master -> device
            ("arg_data", unsigned(32), DIR_FANOUT),
            ("arg_advance", unsigned(1), DIR_NONE), # TODO
            ("cmd", unsigned(8), DIR_FANOUT),
            ("cmd_ready", unsigned(1), DIR_NONE),
            ("cmd_done", unsigned(1), DIR_FANIN),
            # device -> master
            ("param_data", unsigned(32), DIR_FANIN),
            ("param_write", unsigned(1), DIR_FANIN),
            # async data device -> master
            ("invol_req", unsigned(1), DIR_NONE),
            ("invol_grant", unsigned(1), DIR_NONE),
            # system state
            ("systime", unsigned(64), DIR_FANOUT),
            ("shutdown", unsigned(1), DIR_FANOUT),
        ])

class CmdBus(Record):
    def __init__(self):
        super().__init__(CmdBusLayout())

class CmdBusDevice:
    def __init__(self, cmdbus, unit):
        self.cmdbus = cmdbus
        self.unit = unit
        self.sigs = CmdBus()
    def signals(self):
        return self.sigs
    def define_cmd(self, name, nargs=0, string_arg=False, response=True):
        cmd = self.cmdbus.define_cmd(self, name, nargs, string_arg, response)
        return cmd

class CmdBusMaster(Elaboratable):
    def __init__(self, first_cmd=0, first_unit=0):
        self.next_cmd = first_cmd
        self.next_unit = 0
        self.submods = []
        self.cmds = {}
        self.cmds_by_name = {}
        self.signals = [None for _ in range(first_unit)]
        self.units = [None for _ in range(first_unit)]
        self.master = CmdBus()

    def register(self, m):
        dev = CmdBusDevice(self, self.next_unit)
        self.next_unit = self.next_unit + 1
        self.submods.append(m)
        self.signals.append(dev.signals())
        self.units.append(dev)
        return dev

    Command = namedtuple("commands",
        ['dev', 'name', 'nargs', 'string_arg', 'response'])

    def define_cmd(self, dev, name, nargs=0, string_arg=False, response=True):
        num = self.next_cmd
        self.next_cmd = self.next_cmd + 1
        print("define %s to %d" % (name, num))
        self.cmds[num] = self.Command(dev, name, nargs, string_arg, response)
        self.cmds_by_name[name] = num
        return num

    def elaborate(self, platform):
        m = Module()
        m.submodules += self.submods
        m.d.comb += self.master.connect(*self.signals,
            exclude=['cmd_ready', 'invol_grant', 'invol_req', 'arg_advance'])
        self.cmd_ready = []
        self.cmd_done = []
        self.invol_req = []
        self.invol_grant = []
        # TODO: convert to FANIN
        self.arg_advance = []
        for s in self.signals:
            self.cmd_ready.append(s.cmd_ready)
            self.cmd_done.append(s.cmd_done)
            self.invol_req.append(s.invol_req)
            self.invol_grant.append(s.invol_grant)
            self.arg_advance.append(s.arg_advance)
        return m

    #
    # simulation helper
    #
    def sim_drive_systime(self, sim):
        def proc():
            yield Passive()
            while (True):
                systime = yield self.master.systime
                yield self.master.systime.eq(systime + 1)
                yield
        return proc

    def sim_add_background(self, sim):
        sim.add_sync_process(self.sim_drive_systime(sim))

    def sim_write(self, cmd, args):
        num = self.cmds_by_name[cmd]
        c = self.cmds[num]
        unit = c.dev.unit
        assert c.nargs == len(args)

        yield self.master.cmd.eq(num)
        yield self.cmd_ready[unit].eq(1)
        yield self.master.arg_data.eq(args[0])
        yield
        yield self.cmd_ready[unit].eq(0)

        for arg in args[1:]:
            while ((yield self.arg_advance[unit]) == 0 and
                   (yield self.cmd_done[unit]) == 0):
                yield
            if (yield self.cmd_done[unit] == 1):
                break;
            yield self.master.arg_data.eq(arg)
            yield

        while ((yield self.cmd_done[unit]) == 0):
            yield
