#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex_boards.platforms import digilent_nexys_video

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.video import VideoS7HDMIPHY
from litex.soc.cores.led import LedChaser
from litex.soc.cores.jtag import XilinxJTAG

from litedram.modules import MT41K256M16
from litedram.phy import s7ddrphy

from liteeth.phy.s7rgmii import LiteEthPHYRGMII

from litex.tools.litex_json2dts_linux import generate_dts
import json

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, toolchain="vivado", with_video_pll=False):
        self.rst          = Signal()
        self.cd_sys       = ClockDomain()
        self.cd_sys4x     = ClockDomain()
        self.cd_sys4x_dqs = ClockDomain()
        self.cd_idelay    = ClockDomain()
        self.cd_hdmi      = ClockDomain()
        self.cd_hdmi5x    = ClockDomain()
        self.cd_clk100    = ClockDomain()
        self.cd_usb       = ClockDomain()

        # # #

        # Clk / Rst.
        clk100 = platform.request("clk100")
        rst_n  = platform.request("cpu_reset")

        # PLL.
        if toolchain == "vivado":
            self.pll = pll = S7MMCM(speedgrade=-1)
        else:
            self.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(~rst_n | self.rst)
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys,       sys_clk_freq, reset_buf="bufg")
        pll.create_clkout(self.cd_sys4x,     4*sys_clk_freq)
        pll.create_clkout(self.cd_sys4x_dqs, 4*sys_clk_freq, phase=90)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

        self.usb_pll = usb_pll = S7MMCM(speedgrade=-1)
        usb_pll.reset.eq(~rst_n | self.rst)
        usb_pll.register_clkin(clk100, 100e6)
        usb_pll.create_clkout(self.cd_idelay,    200e6)
        usb_pll.create_clkout(self.cd_usb, 48e6, margin=0)

        self.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

        # Video PLL.
        if with_video_pll:
            self.video_pll = video_pll = S7MMCM(speedgrade=-1)
            video_pll.reset.eq(~rst_n | self.rst)
            video_pll.register_clkin(clk100, 100e6)
            video_pll.create_clkout(self.cd_hdmi,   40e6)
            video_pll.create_clkout(self.cd_hdmi5x, 5*40e6)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    mem_map = {**SoCCore.mem_map, **{
        "usb_ohci":     0xE0000000,
    }}

    # DTS generation ---------------------------------------------------------------------------
    def generate_dts(self, board_name):
        json_src = os.path.join("build", board_name, "csr.json")
        dts = os.path.join("build", board_name, "{}.dts".format(board_name))

        with open(json_src) as json_file, open(dts, "w") as dts_file:
            dts_content = generate_dts(json.load(json_file), polling=False)
            dts_file.write(dts_content)

    # DTS compilation --------------------------------------------------------------------------
    def compile_dts(self, board_name, symbols=False):
        dts = os.path.join("build", board_name, "{}.dts".format(board_name))
        dtb = os.path.join("build", board_name, "{}.dtb".format(board_name))
        subprocess.check_call(
            "dtc {} -O dtb -o {} {}".format("-@" if symbols else "", dtb, dts), shell=True)

    def __init__(self, toolchain="vivado", sys_clk_freq=100e6,
        with_ethernet          = False,
        with_led_chaser        = True,
        with_sata              = False, sata_gen="gen2",
        vadj                   = "1.2V",
        with_video_terminal    = False,
        with_video_framebuffer = False,
        **kwargs):
        platform = digilent_nexys_video.Platform(toolchain=toolchain)

        # CRG --------------------------------------------------------------------------------------
        with_video_pll = (with_video_terminal or with_video_framebuffer)
        self.crg = _CRG(platform, sys_clk_freq, toolchain,
            with_video_pll       = with_video_pll
        )

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on Nexys Video", **kwargs)

        # DDR3 SDRAM -------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            self.ddrphy = s7ddrphy.A7DDRPHY(platform.request("ddram"),
                memtype      = "DDR3",
                nphases      = 4,
                sys_clk_freq = sys_clk_freq)
            self.add_sdram("sdram",
                phy           = self.ddrphy,
                module        = MT41K256M16(sys_clk_freq, "1:4"),
                l2_cache_size = kwargs.get("l2_size", 8192)
            )

        # Ethernet ---------------------------------------------------------------------------------
        if with_ethernet:
            self.ethphy = LiteEthPHYRGMII(
                clock_pads = self.platform.request("eth_clocks"),
                pads       = self.platform.request("eth"))
            self.add_ethernet(phy=self.ethphy)

        # SATA -------------------------------------------------------------------------------------
        if with_sata:
            from litex.build.generic_platform import Subsignal, Pins
            from litesata.phy import LiteSATAPHY

            # IOs
            _sata_io = [
                # AB09-FMCRAID / https://www.dgway.com/AB09-FMCRAID_E.html
                ("fmc2sata", 0,
                    Subsignal("clk_p", Pins("LPC:GBTCLK0_M2C_P")),
                    Subsignal("clk_n", Pins("LPC:GBTCLK0_M2C_N")),
                    Subsignal("tx_p",  Pins("LPC:DP0_C2M_P")),
                    Subsignal("tx_n",  Pins("LPC:DP0_C2M_N")),
                    Subsignal("rx_p",  Pins("LPC:DP0_M2C_P")),
                    Subsignal("rx_n",  Pins("LPC:DP0_M2C_N"))
                ),
            ]
            platform.add_extension(_sata_io)

            # RefClk, generate 150MHz from PLL.
            self.cd_sata_refclk = ClockDomain()
            self.crg.pll.create_clkout(self.cd_sata_refclk, 150e6)
            sata_refclk = ClockSignal("sata_refclk")
            platform.add_platform_command("set_property SEVERITY {{WARNING}} [get_drc_checks REQP-49]")

            # PHY
            self.sata_phy = LiteSATAPHY(platform.device,
                refclk     = sata_refclk,
                pads       = platform.request("fmc2sata"),
                gen        = sata_gen,
                clk_freq   = sys_clk_freq,
                data_width = 16)

            # Core
            self.add_sata(phy=self.sata_phy, mode="read+write")

        # Video ------------------------------------------------------------------------------------
        if with_video_terminal or with_video_framebuffer:
            self.videophy = VideoS7HDMIPHY(platform.request("hdmi_out"), clock_domain="hdmi")
            if with_video_terminal:
                self.add_video_terminal(phy=self.videophy, timings="800x600@60Hz", clock_domain="hdmi")
            if with_video_framebuffer:
                self.add_video_framebuffer(phy=self.videophy, timings="800x600@60Hz", clock_domain="hdmi")

        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq)

        # VADJ -------------------------------------------------------------------------------------
        vadj_map = {"1.2V": 0b00, "1.8V": 0b01, "2.5V": 0b10, "3.3V": 0b11}
        platform.request_all("vadj").eq(vadj_map[vadj])


        # OHCI
        from litex.soc.cores.usb_ohci import USBOHCI
        from litex.build.generic_platform import Subsignal, Pins, IOStandard
        from litex.soc.integration.soc import SoCRegion
        _usb_pmod_ios = [
            ("usb_pmodb", 0,
                Subsignal("dp", Pins("pmodb:0", "pmodb:1", "pmodb:2", "pmodb:3")),
                Subsignal("dm", Pins("pmodb:4", "pmodb:5", "pmodb:6", "pmodb:7")),
                IOStandard("LVCMOS33"),
            )
        ]
        platform.add_extension(_usb_pmod_ios)
        self.submodules.usb_ohci = USBOHCI(platform, platform.request("usb_pmodb"), usb_clk_freq=int(48e6))
        self.bus.add_slave("usb_ohci_ctrl", self.usb_ohci.wb_ctrl, region=SoCRegion(origin=self.mem_map["usb_ohci"], size=0x100000, cached=False)) # FIXME: Mapping.
        self.dma_bus.add_master("usb_ohci_dma", master=self.usb_ohci.wb_dma)

        self.comb += self.cpu.interrupt[16].eq(self.usb_ohci.interrupt)





        # Tracer
        from litex.build.generic_platform import Subsignal, Pins, Misc, IOStandard
        _tracer_io = [
            ("tracer", 0,
             Subsignal("tracer_valid", Pins("pmoda:4")),
             Subsignal("tracer_payload", Pins("pmoda:5", "pmoda:6", "pmoda:7")),
             Misc("SLEWRATE=FAST"),
             IOStandard("LVCMOS33"),
             )
        ]
        self.platform.add_extension(_tracer_io)

        tracer_pads = platform.request("tracer")
        self.comb += tracer_pads.tracer_valid.eq(self.cpu.tracer_valid)
        self.comb += tracer_pads.tracer_payload.eq(self.cpu.tracer_payload)




        #  from litex.build.generic_platform import Subsignal, Pins, Misc, IOStandard
        #  _pmodadd_io = [
        #      ("pmodadd", 0,
        #          Subsignal("userpin",  Pins("pmoda:5")),
        #          Misc("SLEWRATE=FAST"),
        #           IOStandard("LVCMOS33"),
        #      )
        # ]
        # self.platform.add_extension(_pmodadd_io)

        # pmodadd_pads = platform.request("pmodadd")

        # spisd = platform.request("spisdcard")
        # self.comb += pmodadd_pads.userpin.eq(spisd.clk)

        # JTAG
        #jtag = XilinxJTAG("BSCANE2", 4)
        #self.submodules.jtag = jtag
        #self.comb += self.cpu.jtag_clk.eq(jtag.tck)
        #self.comb += self.cpu.jtag_enable.eq(True)
        #self.comb += self.cpu.jtag_capture.eq(jtag.capture)
        #self.comb += self.cpu.jtag_shift.eq(jtag.shift)
        #self.comb += self.cpu.jtag_update.eq(jtag.update)
        #self.comb += self.cpu.jtag_reset.eq(jtag.reset)
        #self.comb += self.cpu.jtag_tdi.eq(jtag.tdi)
        #self.comb += jtag.tdo.eq(self.cpu.jtag_tdo)

        from litex.build.generic_platform import Subsignal, Pins, Misc, IOStandard
        _jtag_io = [
            ("jtag", 0,
             Subsignal("tck", Pins("pmoda:0"), Misc("PULLMODE=UP")),
             Subsignal("tdi", Pins("pmoda:1")),
             Subsignal("tdo", Pins("pmoda:2")),
             Subsignal("tms", Pins("pmoda:3")),
             Misc("SLEWRATE=FAST"),
             IOStandard("LVCMOS33"),
             )
        ]
        self.platform.add_extension(_jtag_io)


        
        jtag_pads = platform.request("jtag")
        self.comb += self.cpu.jtag_tck.eq(jtag_pads.tck)
        self.comb += self.cpu.jtag_tms.eq(jtag_pads.tms)
        self.comb += self.cpu.jtag_tdi.eq(jtag_pads.tdi)
        self.comb += jtag_pads.tdo.eq(self.cpu.jtag_tdo)
        platform.add_platform_command("set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets jtag_tck_IBUF]")
        platform.add_platform_command("create_clock -period 10.000 -name jtag_tck [get_nets jtag_tck_IBUF]")
        
        platform.add_platform_command("set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets jtag_tckbuf_OBUF]")

        platform.add_platform_command("create_clock -name jtag_tck -period 20.0 [get_nets jtag_tck]")
        # platform.add_platform_command("set_clock_groups -group [get_clocks -include_generated_clocks -of [get_nets sys_clk]] -group [get_clocks -include_generated_clocks -of [get_nets jtag_tck]] -asynchronous")


        platform.add_false_path_constraints(self.crg.cd_sys.clk, jtag_pads.tck)

        # create_clock -period 32.000 -name clk_f2 -waveform {0.000 16.000} [get_nets *clk_f2*]
        # create_clock -period 27.000 -name clk_f1 -waveform {0.000 13.500} [get_nets *clk_f1*]
        # set_clock_groups -asynchronous -group clk_f1 -group clk_f2 -group sysclk
        #
        #     platform.add_platform_command("set_clock_groups -asynchronous -group jtag_tck -group crg_s7mmcm0_clkout0")
        #     platform.add_platform_command("set_clock_groups -asynchronous -group crg_s7mmcm0_clkout0 -group jtag_tck")


# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=digilent_nexys_video.Platform, description="LiteX SoC on Nexys Video.")
    parser.add_target_argument("--sys-clk-freq",  default=100e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--with-ethernet", action="store_true",       help="Enable Ethernet support.")
    sdopts = parser.target_group.add_mutually_exclusive_group()
    sdopts.add_argument("--with-spi-sdcard", action="store_true", help="Enable SPI-mode SDCard support.")
    sdopts.add_argument("--with-sdcard",     action="store_true", help="Enable SDCard support.")
    parser.add_target_argument("--with-sata",            action="store_true", help="Enable SATA support (over FMCRAID).")
    parser.add_target_argument("--sata-gen",             default="2",         help="SATA Gen.", choices=["1", "2"])
    parser.add_target_argument("--vadj",                 default="1.2V",      help="FMC VADJ value.", choices=["1.2V", "1.8V", "2.5V", "3.3V"])
    viopts = parser.target_group.add_mutually_exclusive_group()
    viopts.add_argument("--with-video-terminal",    action="store_true", help="Enable Video Terminal (HDMI).")
    viopts.add_argument("--with-video-framebuffer", action="store_true", help="Enable Video Framebuffer (HDMI).")
    args = parser.parse_args()

    soc = BaseSoC(
        toolchain              = args.toolchain,
        sys_clk_freq           = args.sys_clk_freq,
        with_ethernet          = args.with_ethernet,
        with_sata              = args.with_sata,
        sata_gen               = "gen" + args.sata_gen,
        vadj                   = args.vadj,
        with_video_terminal    = args.with_video_terminal,
        with_video_framebuffer = args.with_video_framebuffer,
        **parser.soc_argdict
    )
    if args.with_spi_sdcard:
        soc.add_spi_sdcard()
    if args.with_sdcard:
        soc.add_sdcard()
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    board_name = "digilent_nexys_video"
    # soc.generate_dts(board_name)
    # soc.compile_dts(board_name)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
