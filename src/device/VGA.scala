package ysyx

import chisel3._
import chisel3.util._

import freechips.rocketchip.amba.apb._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

class VGAIO extends Bundle {
  val r     = Output(UInt(8.W))
  val g     = Output(UInt(8.W))
  val b     = Output(UInt(8.W))
  val hsync = Output(Bool())
  val vsync = Output(Bool())
  val valid = Output(Bool())
}

class VgaSignalGen extends Module {
  class Port extends Bundle {
    val pixel = Input(UInt(24.W))
    val hAddr = Output(UInt(10.W))
    val vAddr = Output(UInt(10.W))
    val vga   = new VGAIO
  }

  val io = IO(new Port)

  private val FrontPorchH = 96
  private val ActiveH     = 144
  private val BackPorchH  = 784
  private val TotalH      = 800
  private val FrontPorchV = 2
  private val ActiveV     = 35
  private val BackPorchV  = 515
  private val TotalV      = 525

  private val xCnt = RegInit(1.U(10.W))
  private val yCnt = RegInit(1.U(10.W))

  xCnt := Mux(xCnt === TotalH.U, 1.U, xCnt + 1.U)
  yCnt := MuxLookup(Cat(xCnt === TotalH.U, yCnt === TotalV.U), yCnt)(
    Seq(
      "b11".U -> 1.U,
      "b10".U -> (yCnt + 1.U)
    )
  )

  private val hValid = (xCnt > ActiveH.U) & (xCnt <= BackPorchH.U)
  private val vValid = (yCnt > ActiveV.U) & (yCnt <= BackPorchV.U)

  io.vga.hsync := xCnt > FrontPorchH.U
  io.vga.vsync := yCnt > FrontPorchV.U
  io.vga.valid := hValid & vValid
  io.vga.r     := io.pixel(23, 16)
  io.vga.g     := io.pixel(15, 8)
  io.vga.b     := io.pixel(7, 0)
  io.hAddr     := Mux(hValid, xCnt - (ActiveH + 1).U, 0.U)
  io.vAddr     := Mux(vValid, yCnt - (ActiveV + 1).U, 0.U)
}

class VgaCtrl extends Module {
  class Port extends Bundle {
    val in  = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
    val vga = new VGAIO
  }

  val io = IO(new Port)

  private val addrBad = ~(BitPat("b00100001_000?????_????????_??????00") === io.in.paddr)
  private val strbBad = ~(io.in.pstrb === "b1111".U)
  private val bad     = ~io.in.pwrite | addrBad | strbBad

  private object State extends ChiselEnum {
    val S_Idle  = Value
    val S_Setup = Value
    val S_Wr    = Value
    val S_Done  = Value
  }
  import State._
  private val y = RegInit(S_Idle)
  y := MuxLookup(y, S_Idle)(
    Seq(
      S_Idle -> Mux(io.in.psel, S_Setup, S_Idle),
      S_Setup -> MuxCase(
        S_Setup,
        Seq(
          (~io.in.penable) -> S_Setup,
          (addrBad)        -> S_Done,
          (io.in.pwrite)   -> S_Wr,
          (~io.in.pwrite)  -> S_Done
        )
      ),
      S_Wr   -> S_Done,
      S_Done -> S_Idle
    )
  )

  private val fb = Mem(640 * 480, UInt(24.W))

  private val signalGen = Module(new VgaSignalGen)
  io.vga <> signalGen.io.vga

  signalGen.io.pixel := fb(signalGen.io.vAddr * 640.U + signalGen.io.hAddr)

  fb(io.in.paddr(20, 2)) := Mux(y === S_Wr & ~bad, io.in.pwdata(23, 0), fb(io.in.paddr(20, 2)))

  io.in.prdata  := DontCare
  io.in.pslverr := y === S_Done & bad
  io.in.pready  := y === S_Done
}

class VGACtrlIO extends Bundle {
  val clock = Input(Clock())
  val reset = Input(Bool())
  val in    = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
  val vga   = new VGAIO
}

class vga_top_apb extends BlackBox {
  val io = IO(new VGACtrlIO)
}

class vgaChisel extends RawModule {
  val io = IO(new VGACtrlIO)

  private val ctrl = withClockAndReset(io.clock, io.reset) { Module(new VgaCtrl) }
  ctrl.io.in  <> io.in
  ctrl.io.vga <> io.vga
}

class APBVGA(address: Seq[AddressSet])(implicit p: Parameters) extends LazyModule {
  val node = APBSlaveNode(
    Seq(
      APBSlavePortParameters(
        Seq(
          APBSlaveParameters(
            address       = address,
            executable    = false,
            supportsRead  = false,
            supportsWrite = true
          )
        ),
        beatBytes = 4
      )
    )
  )

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val (in, _)    = node.in(0)
    val vga_bundle = IO(new VGAIO)

    val mvga = Module(new vgaChisel)
    mvga.io.clock := clock
    mvga.io.reset := reset
    mvga.io.in    <> in
    vga_bundle    <> mvga.io.vga
  }
}
