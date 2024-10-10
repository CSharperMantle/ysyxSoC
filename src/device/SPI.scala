package ysyx

import chisel3._
import chisel3.util._
import chisel3.util.experimental.decode._

import freechips.rocketchip.amba.apb._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

class SPIIO(val ssWidth: Int = 8) extends Bundle {
  val sck  = Output(Bool())
  val ss   = Output(UInt(ssWidth.W))
  val mosi = Output(Bool())
  val miso = Input(Bool())
}

class spi_top_apb extends BlackBox {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Reset())
    val in =
      Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
    val spi         = new SPIIO
    val spi_irq_out = Output(Bool())
  })
}

class flash extends BlackBox {
  val io = IO(Flipped(new SPIIO(1)))
}

case class XipStateProp(
  val state:    BitPat,
  val write:    Option[BitPat],
  val addr:     Option[BitPat],
  val writeReq: Boolean)
    extends DecodePattern {
  require(!(!write.isEmpty && addr.isEmpty))

  def bitPat = state
}

object XipEnableField extends BoolDecodeField[XipStateProp] {
  override def name = "penable"
  override def genTable(state: XipStateProp): BitPat = {
    if (state.addr.isEmpty) { n }
    else { y }
  }
}

object XipWriteField extends BoolDecodeField[XipStateProp] {
  override def name = "pwrite"
  override def genTable(state: XipStateProp): BitPat = {
    if (state.write.isEmpty) { n }
    else { y }
  }
}

object XipWdataField extends DecodeField[XipStateProp, UInt] {
  override def name       = "pwdata"
  override def chiselType = UInt(32.W)
  override def genTable(state: XipStateProp): BitPat = {
    if (state.write.isEmpty) { BitPat.dontCare(32) }
    else { state.write.get }
  }
}

object XipWdataSelField extends BoolDecodeField[XipStateProp] {
  override def name = "pwdata_sel"
  override def genTable(state: XipStateProp): BitPat = {
    if (state.writeReq) { y }
    else { n }
  }
}

object XipAddrField extends DecodeField[XipStateProp, UInt] {
  override def name       = "paddr"
  override def chiselType = UInt(32.W)
  override def genTable(state: XipStateProp): BitPat = {
    if (state.addr.isEmpty) { BitPat.dontCare(32) }
    else { state.addr.get }
  }
}

class APBSPI(address: Seq[AddressSet])(implicit p: Parameters) extends LazyModule {
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
    val (in, _)    = node.in(0)
    val spi_bundle = IO(new SPIIO)

    val mspi = Module(new spi_top_apb)
    mspi.io.clock := clock
    mspi.io.reset := reset
    spi_bundle    <> mspi.io.spi

    private val inFlash =
      BitPat("b0011????_????????_????????_????????") === in.paddr

    private object State extends ChiselEnum {
      implicit class CvtValueToType(v: Type) {
        def U:  UInt   = v.litValue.U(getWidth.W)
        def BP: BitPat = BitPat(v.U)
      }
      val S_Idle            = Value
      val S_PassThru        = Value
      val S_XipInitDiv      = Value
      val S_XipInitDivDone  = Value
      val S_XipInitSs       = Value
      val S_XipInitSsDone   = Value
      val S_XipInitCtrl     = Value
      val S_XipInitCtrlDone = Value
      val S_XipReq          = Value
      val S_XipReqDone      = Value
      val S_XipTrig         = Value
      val S_XipTrigDone     = Value
      val S_XipWaitRead     = Value
      val S_XipRead         = Value
      val S_XipReadDone     = Value
    }
    import State._
    private val y = RegInit(S_Idle)
    y := MuxLookup(y, S_Idle)(
      Seq(
        S_Idle -> MuxCase(
          S_Idle,
          Seq(
            (in.psel & ~inFlash)             -> S_PassThru,
            (in.psel & inFlash & ~in.pwrite) -> S_XipInitDiv,
            (in.psel & inFlash & in.pwrite)  -> S_XipReadDone
          )
        ),
        S_PassThru        -> Mux(mspi.io.in.pready, S_Idle, S_PassThru),
        S_XipInitDiv      -> Mux(mspi.io.in.pready, S_XipInitDivDone, S_XipInitDiv),
        S_XipInitDivDone  -> S_XipInitSs,
        S_XipInitSs       -> Mux(mspi.io.in.pready, S_XipInitSsDone, S_XipInitSs),
        S_XipInitSsDone   -> S_XipInitCtrl,
        S_XipInitCtrl     -> Mux(mspi.io.in.pready, S_XipInitCtrlDone, S_XipInitCtrl),
        S_XipInitCtrlDone -> S_XipReq,
        S_XipReq          -> Mux(mspi.io.in.pready, S_XipReqDone, S_XipReq),
        S_XipReqDone      -> S_XipTrig,
        S_XipTrig         -> Mux(mspi.io.in.pready, S_XipTrigDone, S_XipTrig),
        S_XipTrigDone     -> S_XipWaitRead,
        S_XipWaitRead     -> Mux(mspi.io.spi_irq_out, S_XipRead, S_XipWaitRead),
        S_XipRead         -> Mux(mspi.io.in.pready, S_XipReadDone, S_XipRead),
        S_XipReadDone     -> S_Idle
      )
    )

    private val xipStateProps = Seq(
      // scalafmt: { maxColumn = 512, align.tokens.add = [ { code = "," } ] }
      XipStateProp(S_Idle.BP,            None,                     None,                     false),
      XipStateProp(S_PassThru.BP,        None,                     None,                     false),
      XipStateProp(S_XipInitDiv.BP,      Some(0x00000000.U(32.W)), Some(0x10001014.U(32.W)), false), // TODO: Fill in actual divider
      XipStateProp(S_XipInitDivDone.BP,  None,                     None,                     false),
      XipStateProp(S_XipInitSs.BP,       Some(0x00000001.U(32.W)), Some(0x10001018.U(32.W)), false),
      XipStateProp(S_XipInitSsDone.BP,   None,                     None,                     false),
      XipStateProp(S_XipInitCtrl.BP,     Some(0x00003040.U(32.W)), Some(0x10001010.U(32.W)), false),
      XipStateProp(S_XipInitCtrlDone.BP, None,                     None,                     false),
      XipStateProp(S_XipReq.BP,          Some(0.U(32.W)),          Some(0x10001004.U(32.W)), true),
      XipStateProp(S_XipReqDone.BP,      None,                     None,                     false),
      XipStateProp(S_XipTrig.BP,         Some(0x00003140.U(32.W)), Some(0x10001010.U(32.W)), false),
      XipStateProp(S_XipTrigDone.BP,     None,                     None,                     false),
      XipStateProp(S_XipWaitRead.BP,     None,                     None,                     false),
      XipStateProp(S_XipRead.BP,         None,                     Some(0x10001000.U(32.W)), false),
      XipStateProp(S_XipReadDone.BP,     None,                     None,                     false)
      // scalafmt: { align.tokens.add = [] }
    )
    private val xipStateFields = Seq(
      XipEnableField,
      XipWriteField,
      XipWdataField,
      XipWdataSelField,
      XipAddrField
    )
    private val table = new DecodeTable(xipStateProps, xipStateFields)
    private val res   = table.decode(y.asUInt)

    private val rdata = RegEnable(mspi.io.in.prdata, 0.U, mspi.io.in.pready)

    private val xip = Wire(new APBBundle(APBBundleParameters(32, 32)))
    xip.psel    := in.psel
    xip.penable := res(XipEnableField)
    xip.pwrite  := res(XipWriteField)
    xip.paddr   := res(XipAddrField)
    xip.pprot   := in.pprot
    xip.pwdata  := Mux(res(XipWdataSelField), Cat(0x03.U(8.W), in.paddr(23, 0)), res(XipWdataField))
    xip.pstrb   := 0xf.U(4.W)
    xip.pauser  := DontCare
    xip.pready  := y === S_XipReadDone
    xip.pslverr := inFlash & in.pwrite
    xip.prdata  := Cat(rdata(7, 0), rdata(15, 8), rdata(23, 16), rdata(31, 24))
    xip.pduser  := DontCare

    mspi.io.in.psel    := Mux(inFlash, xip.psel, in.psel)
    mspi.io.in.penable := Mux(inFlash, xip.penable, in.penable)
    mspi.io.in.pwrite  := Mux(inFlash, xip.pwrite, in.pwrite)
    mspi.io.in.paddr   := Mux(inFlash, xip.paddr, in.paddr)
    mspi.io.in.pprot   := Mux(inFlash, xip.pprot, in.pprot)
    mspi.io.in.pwdata  := Mux(inFlash, xip.pwdata, in.pwdata)
    mspi.io.in.pstrb   := Mux(inFlash, xip.pstrb, in.pstrb)
    mspi.io.in.pauser  := Mux(inFlash, xip.pauser, in.pauser)
    in.pready          := Mux(inFlash, xip.pready, mspi.io.in.pready)
    in.pslverr         := Mux(inFlash, xip.pslverr, mspi.io.in.pslverr)
    in.prdata          := Mux(inFlash, xip.prdata, mspi.io.in.prdata)
    in.pduser          := Mux(inFlash, xip.pduser, mspi.io.in.pduser)
  }
}
