package ysyx

import chisel3._
import chisel3.util._
import chisel3.util.experimental.decode._

import freechips.rocketchip.amba.apb._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

class Bcd7SegDecoder(n: Int, hex: Boolean = false) extends Module {
  class Port extends Bundle {
    val bcd = Input(Vec(n, UInt(4.W)))
    val out = Output(Vec(n, UInt(8.W)))
  }

  val io = IO(new Port)

  private val table = TruthTable(
    Seq(
      BitPat(0x0.U(4.W)) -> BitPat("b11111100"),
      BitPat(0x1.U(4.W)) -> BitPat("b01100000"),
      BitPat(0x2.U(4.W)) -> BitPat("b11011010"),
      BitPat(0x3.U(4.W)) -> BitPat("b11110010"),
      BitPat(0x4.U(4.W)) -> BitPat("b01100110"),
      BitPat(0x5.U(4.W)) -> BitPat("b10110110"),
      BitPat(0x6.U(4.W)) -> BitPat("b10111110"),
      BitPat(0x7.U(4.W)) -> BitPat("b11100000"),
      BitPat(0x8.U(4.W)) -> BitPat("b11111110"),
      BitPat(0x9.U(4.W)) -> BitPat("b11110110")
    ) ++ (if (hex)
            Seq(
              BitPat(0xa.U(4.W)) -> BitPat("b11101110"),
              BitPat(0xb.U(4.W)) -> BitPat("b00111110"),
              BitPat(0xc.U(4.W)) -> BitPat("b00011010"),
              BitPat(0xd.U(4.W)) -> BitPat("b01111010"),
              BitPat(0xe.U(4.W)) -> BitPat("b10011110"),
              BitPat(0xf.U(4.W)) -> BitPat("b10001110")
            )
          else Seq()),
    BitPat("b00000000")
  )

  for (i <- 0 until n) {
    io.out(i) := ~decoder(io.bcd(i), table)
  }
}

class GPIOIO extends Bundle {
  val out = Output(UInt(16.W))
  val in  = Input(UInt(16.W))
  val seg = Output(Vec(8, UInt(8.W)))
}

class GPIOCtrlIO extends Bundle {
  val clock = Input(Clock())
  val reset = Input(Reset())
  val in    = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
  val gpio  = new GPIOIO
}

class GpioCtrl extends Module {
  class Port extends Bundle {
    val in   = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
    val gpio = new GPIOIO
  }

  val io = IO(new Port)

  private val addrBad = ~(BitPat("b00010000_00000000_00100000_0000????") === io.in.paddr)

  private val regs = RegInit(VecInit(Seq.fill(4)(0.U(32.W))))

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

  private val regIdx1H   = UIntToOH(io.in.paddr(3, 2), 4)
  private val regSelData = Mux1H(regs.zipWithIndex.map { case (r, i) => regIdx1H(i) -> r })

  private val wrEn   = y === S_Wr
  private val wrMask = FillInterleaved(8, io.in.pstrb)

  private val wrData = (regSelData & ~wrMask) | (io.in.pwdata & wrMask)
  for (i <- 0 until regs.length) {
    if (i == 1) {
      // GPIO Input debounce
      regs(i) := Mux(y === S_Idle, Cat(0.U(16.W), io.gpio.in), regs(i))
    } else {
      regs(i) := Mux(wrEn & regIdx1H(i), wrData, regs(i))
    }
  }

  private val nSegs         = regs(2).getWidth / 4
  private val segComponents = Wire(Vec(nSegs, UInt(4.W)))
  for (i <- 0 until nSegs) {
    segComponents(i) := regs(2)(4 * (i + 1) - 1, 4 * i)
  }
  private val segDecoder = Module(new Bcd7SegDecoder(nSegs))
  segDecoder.io.bcd := segComponents

  io.gpio.out   := regs(0)
  io.gpio.seg   := segDecoder.io.out
  io.in.pready  := y === S_Done
  io.in.pslverr := io.in.penable & addrBad
  io.in.prdata  := regSelData
}

class gpio_top_apb extends BlackBox {
  val io = IO(new GPIOCtrlIO)
}

class gpioChisel extends RawModule {
  val io = IO(new GPIOCtrlIO)

  val ctrl = withClockAndReset(io.clock, io.reset) { Module(new GpioCtrl) }
  ctrl.io.gpio <> io.gpio
  ctrl.io.in   <> io.in
}

class APBGPIO(address: Seq[AddressSet])(implicit p: Parameters) extends LazyModule {
  val node = APBSlaveNode(
    Seq(
      APBSlavePortParameters(
        Seq(
          APBSlaveParameters(
            address       = address,
            executable    = true,
            supportsRead  = true,
            supportsWrite = true
          )
        ),
        beatBytes = 4
      )
    )
  )

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val (in, _)     = node.in(0)
    val gpio_bundle = IO(new GPIOIO)

    val mgpio = Module(new gpioChisel)
    mgpio.io.clock := clock
    mgpio.io.reset := reset
    mgpio.io.in    <> in
    gpio_bundle    <> mgpio.io.gpio
  }
}
