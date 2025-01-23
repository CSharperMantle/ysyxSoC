package ysyx

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.amba._
import freechips.rocketchip.amba.apb._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

class APBDelayerIO extends Bundle {
  val clock = Input(Clock())
  val reset = Input(Reset())
  val in    = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
  val out   = new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32))
}

class apb_delayer extends BlackBox {
  val io = IO(new APBDelayerIO)
}

class APBDelayerChisel(val factor: Double) extends Module {
  require(factor >= 1)

  private val r = 1024

  val io = IO(new APBDelayerIO)

  private val delayDone = Wire(Bool())

  object State extends ChiselEnum {
    val S_Idle     = Value
    val S_Setup    = Value
    val S_Access   = Value
    val S_DelayAdj = Value
    val S_Delay    = Value
    val S_Respond  = Value
  }
  import State._
  private val y = RegInit(S_Idle)
  y := MuxLookup(y, S_Idle)(
    Seq(
      S_Idle     -> Mux(io.in.psel, S_Setup, S_Idle),
      S_Setup    -> Mux(io.in.penable, S_Access, S_Setup),
      S_Access   -> Mux(io.out.pready, S_DelayAdj, S_Access),
      S_DelayAdj -> S_Delay,
      S_Delay    -> Mux(delayDone, S_Respond, S_Delay),
      S_Respond  -> Mux(io.in.psel, S_Setup, S_Idle)
    )
  )

  private val slverr = RegEnable(io.out.pslverr, 0.U, io.out.pready)
  private val rdata  = RegEnable(io.out.prdata, 0.U, io.out.pready)
  private val duser =
    RegEnable(io.out.pduser, 0.U.asTypeOf(chiselTypeOf(io.out.pduser)), io.out.pready)

  private val ctr = RegInit(0.U(32.W))
  ctr := MuxLookup(y, ctr)(
    Seq(
      S_Setup    -> 0.U,
      S_Access   -> (ctr + (factor * r).toInt.U),
      S_DelayAdj -> (ctr >> log2Floor(r)),
      S_Delay    -> (ctr - 1.U)
    )
  )
  delayDone := ctr === 0.U

  io.out.psel    := ~y.isOneOf(Seq(S_DelayAdj, S_Delay, S_Respond)) & io.in.psel
  io.out.penable := ~y.isOneOf(Seq(S_DelayAdj, S_Delay, S_Respond)) & io.in.penable
  io.out.pwrite  := io.in.pwrite
  io.out.paddr   := io.in.paddr
  io.out.pprot   := io.in.pprot
  io.out.pwdata  := io.in.pwdata
  io.out.pstrb   := io.in.pstrb
  io.out.pauser  := io.in.pauser
  io.in.pready   := y === S_Respond
  io.in.pslverr  := slverr
  io.in.prdata   := rdata
  io.in.pduser   := duser
}

class APBDelayerWrapper(implicit p: Parameters) extends LazyModule {
  val node = APBIdentityNode()

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    (node.in zip node.out) foreach {
      case ((in, edgeIn), (out, edgeOut)) =>
        val delayer = Module(new APBDelayerChisel(4.02535))
        delayer.io.clock := clock
        delayer.io.reset := reset
        delayer.io.in    <> in
        out              <> delayer.io.out
    }
  }
}

object APBDelayer {
  def apply()(implicit p: Parameters): APBNode = {
    val apbdelay = LazyModule(new APBDelayerWrapper)
    apbdelay.node
  }
}
