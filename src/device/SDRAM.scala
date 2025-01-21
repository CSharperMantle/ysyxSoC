package ysyx

import chisel3._
import chisel3.util._
import chisel3.util.experimental.decode._
import chisel3.experimental.Analog
import chisel3.experimental.BundleLiterals._

import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.amba.apb._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

class SdramBurstGen extends Module {
  class Port extends Bundle {
    val en   = Input(Bool())
    val a    = Input(UInt(3.W))
    val len  = Input(UInt(2.W))
    val ay   = Output(UInt(3.W))
    val done = Output(Bool())
  }

  val io = IO(new Port)

  private val ctr = RegInit(0.U(3.W))
  ctr := Mux(io.en, ctr + 1.U, 0.U)
  io.ay := MuxLookup(io.len, io.a)(
    (1 to 2).map(i => i.U(2.W) -> Cat(io.a(2, i), (io.a + ctr)(i - 1, 0))) ++ Seq(
      0.U(2.W) -> io.a,
      3.U(2.W) -> (io.a + ctr)
    )
  )
  io.done := MuxLookup(io.len, true.B)(
    (1 to 3).map(i => i.U(2.W) -> (ctr(i - 1, 0) === ((1 << i) - 1).U)) ++ Seq(
      0.U(2.W) -> true.B
    )
  )
}

class SDRAMIO extends Bundle {
  val clk = Output(Bool())
  val cke = Output(Bool())
  val cs  = Output(Bool())
  val ras = Output(Bool())
  val cas = Output(Bool())
  val we  = Output(Bool())
  val a   = Output(UInt(13.W))
  val ba  = Output(UInt(2.W))
  val dqm = Output(UInt(2.W))
  val dq  = Analog(16.W)
}

class SdramModelBlackBox extends BlackBox with HasBlackBoxInline {
  class Port extends Bundle {
    val wClk  = Input(Bool())
    val wEn   = Input(Bool())
    val wBank = Input(UInt(2.W))
    val wRow  = Input(UInt(13.W))
    val wCol  = Input(UInt(13.W))
    val wData = Input(UInt(16.W))
    val wMask = Input(UInt(2.W))
    val rEn   = Input(Bool())
    val rBank = Input(UInt(2.W))
    val rRow  = Input(UInt(13.W))
    val rCol  = Input(UInt(13.W))
    val rMask = Input(UInt(2.W))
    val rData = Output(UInt(16.W))
  }

  val io = IO(new Port)

  setInline(
    s"${name}.sv",
    s"""
       |module ${name}(
       |  input             wClk,
       |  input             wEn,
       |  input      [1:0]  wBank,
       |  input      [12:0] wRow,
       |  input      [12:0] wCol,
       |  input      [15:0] wData,
       |  input      [1:0]  wMask,
       |  input             rEn,
       |  input      [1:0]  rBank,
       |  input      [12:0] rRow,
       |  input      [12:0] rCol,
       |  input      [1:0]  rMask,
       |  output reg [15:0] rData
       |);
       |  import "DPI-C" function shortint soc_dpi_sdram_read(input  byte     rBank,
       |                                                      input  shortint rRow,
       |                                                      input  shortint rCol,
       |                                                      input  byte     rMask);
       |  import "DPI-C" function void soc_dpi_sdram_write(input byte     wBank,
       |                                                   input shortint wRow,
       |                                                   input shortint wCol,
       |                                                   input byte     wMask,
       |                                                   input shortint wData);
       |  always_ff @(posedge wClk) begin
       |    if (wEn) begin
       |      soc_dpi_sdram_write(wBank, wRow, wCol, wMask, wData);
       |    end
       |  end
       |  always @(rEn, rBank, rRow, rCol, rMask) begin
       |    if (rEn) begin
       |      rData = soc_dpi_sdram_read(rBank, rRow, rCol, rMask);
       |    end else begin
       |      rData = 0;
       |    end
       |  end
       |endmodule
       |""".stripMargin
  )
}

class SdramModel extends Module {
  class Port extends Bundle {
    val cke   = Input(Bool())
    val cs    = Input(Bool())
    val ras   = Input(Bool())
    val cas   = Input(Bool())
    val we    = Input(Bool())
    val a     = Input(UInt(13.W))
    val ba    = Input(UInt(2.W))
    val dqm   = Input(UInt(2.W))
    val dqi   = Input(UInt(16.W))
    val dqo   = Output(UInt(16.W))
    val dqoEn = Output(UInt(2.W))
  }

  val io = IO(new Port)

  private val backend = Module(new SdramModelBlackBox)

  private object CmdEncoding extends ChiselEnum {
    val Inhibit     = Value
    val Nop         = Value
    val Active      = Value
    val Read        = Value
    val Write       = Value
    val BurstTerm   = Value
    val Precharge   = Value
    val AutoRefresh = Value
    val LoadModeReg = Value
  }

  private val cmdTable = TruthTable(
    Seq(
      BitPat("b1???") -> BitPat(CmdEncoding.Inhibit.asUInt),
      BitPat("b0111") -> BitPat(CmdEncoding.Nop.asUInt),
      BitPat("b0011") -> BitPat(CmdEncoding.Active.asUInt),
      BitPat("b0101") -> BitPat(CmdEncoding.Read.asUInt),
      BitPat("b0100") -> BitPat(CmdEncoding.Write.asUInt),
      BitPat("b0110") -> BitPat(CmdEncoding.BurstTerm.asUInt),
      BitPat("b0010") -> BitPat(CmdEncoding.Precharge.asUInt),
      BitPat("b0001") -> BitPat(CmdEncoding.AutoRefresh.asUInt),
      BitPat("b0000") -> BitPat(CmdEncoding.LoadModeReg.asUInt)
    ),
    BitPat.dontCare(CmdEncoding.getWidth)
  )
  private val cmd = decoder(Cat(io.cs, io.ras, io.cas, io.we), cmdTable)

  private val modeReg        = RegInit(0.U(13.W))
  private val casLatency     = modeReg(5, 4)
  private val writeBurstMode = modeReg(9)

  private val activeRow = RegInit(0.U(13.W))

  private val readDone  = Wire(Bool())
  private val writeDone = Wire(Bool())

  private object State extends ChiselEnum {
    val S_Idle  = Value
    val S_Read  = Value
    val S_Write = Value
  }
  import State._
  private val y = RegInit(S_Idle)
  y := MuxLookup(y, S_Idle)(
    Seq(
      S_Idle -> MuxLookup(cmd, S_Idle)(
        Seq(
          CmdEncoding.Read.asUInt  -> S_Read,
          CmdEncoding.Write.asUInt -> S_Write
        )
      ),
      S_Read -> Mux(
        cmd.isOneOf(
          CmdEncoding.Inhibit.asUInt,
          CmdEncoding.Nop.asUInt,
          CmdEncoding.Read.asUInt
        ) & ~readDone,
        S_Read,
        S_Idle
      ),
      S_Write -> Mux(
        cmd.isOneOf(
          CmdEncoding.Inhibit.asUInt,
          CmdEncoding.Nop.asUInt,
          CmdEncoding.Write.asUInt
        ) & ~writeDone,
        S_Write,
        S_Idle
      )
    )
  )

  modeReg   := Mux(cmd === CmdEncoding.LoadModeReg.asUInt, io.a, modeReg)
  activeRow := Mux(cmd === CmdEncoding.Active.asUInt, io.a, activeRow)

  private class RwQueueItem extends Bundle {
    val valid    = Bool()
    val bankAddr = UInt(2.W)
    val colAddr  = UInt(13.W)
    val dqm      = UInt(2.W)
  }
  private object RwQueueItem {
    val Null = (new RwQueueItem).Lit(
      _.valid    -> false.B,
      _.bankAddr -> 0.U,
      _.colAddr  -> 0.U,
      _.dqm      -> "b11".U
    )
  }

  private val readQueue = RegInit(VecInit(Seq.fill(4)(RwQueueItem.Null)))
  for (i <- 0 until readQueue.length - 1) {
    readQueue(i) := readQueue(i + 1)
  }
  readQueue(readQueue.length - 1) := RwQueueItem.Null
  when(cmd === CmdEncoding.Read.asUInt) {
    readQueue(casLatency - 1.U).valid    := true.B
    readQueue(casLatency - 1.U).bankAddr := io.ba
    readQueue(casLatency - 1.U).colAddr  := io.a
    readQueue(casLatency - 1.U).dqm      := io.dqm
  }

  private val readCursor = RegInit(RwQueueItem.Null)
  readCursor := Mux(
    y =/= S_Read,
    RwQueueItem.Null,
    Mux(readQueue(1).valid, readQueue(1), readCursor)
  )

  private val readBurstGen = Module(new SdramBurstGen)
  readBurstGen.io.en := y === S_Read & readCursor.valid
  readBurstGen.io.a := Mux(
    y === S_Read & readQueue(1).valid,
    readQueue(1).colAddr,
    readCursor.colAddr
  )(2, 0)
  readBurstGen.io.len := modeReg(1, 0)

  readDone := readQueue.map(~_.valid).andR & readBurstGen.io.done

  backend.io.rEn   := y === S_Read & readCursor.valid
  backend.io.rMask := ~readCursor.dqm
  backend.io.rRow  := activeRow
  backend.io.rBank := readCursor.bankAddr
  backend.io.rCol  := Cat(readCursor.colAddr(12, 3), readBurstGen.io.ay)

  private val writeCursorNext = Wire(new RwQueueItem)
  writeCursorNext.valid    := true.B
  writeCursorNext.bankAddr := io.ba
  writeCursorNext.colAddr  := io.a
  writeCursorNext.dqm      := DontCare

  private val writeCursor = RegInit(RwQueueItem.Null)
  writeCursor := Mux(
    cmd === CmdEncoding.Write.asUInt,
    writeCursorNext,
    Mux(y === S_Write, writeCursor, RwQueueItem.Null)
  )

  private val writeBurstGen = Module(new SdramBurstGen)
  writeBurstGen.io.en  := y === S_Write | cmd === CmdEncoding.Write.asUInt
  writeBurstGen.io.a   := Mux(y === S_Write & writeCursor.valid, writeCursor.colAddr, io.a)(2, 0)
  writeBurstGen.io.len := modeReg(1, 0)

  writeDone := writeBurstGen.io.done

  backend.io.wClk  := clock.asBool
  backend.io.wEn   := cmd === CmdEncoding.Write.asUInt | (y === S_Write & writeCursor.valid)
  backend.io.wMask := ~io.dqm
  backend.io.wData := io.dqi
  backend.io.wRow  := activeRow
  backend.io.wBank := Mux(writeCursor.valid, writeCursor.bankAddr, io.ba)
  backend.io.wCol := Cat(
    Mux(writeCursor.valid, writeCursor.colAddr, io.a)(12, 3),
    writeBurstGen.io.ay
  )

  io.dqo   := backend.io.rData
  io.dqoEn := Mux(y === S_Read, ~readCursor.dqm, 0.U(2.W))
}

class sdram_top_axi extends BlackBox {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Bool())
    val in    = Flipped(new AXI4Bundle(AXI4BundleParameters(addrBits = 32, dataBits = 32, idBits = 4)))
    val sdram = new SDRAMIO
  })
}

class sdram_top_apb extends BlackBox {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Bool())
    val in    = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
    val sdram = new SDRAMIO
  })
}

class sdram extends BlackBox {
  val io = IO(Flipped(new SDRAMIO))
}

class sdramChisel extends RawModule {
  val io = IO(Flipped(new SDRAMIO))

  private val model =
    withClockAndReset(io.clk.asClock, false.asBool.asAsyncReset)(Module(new SdramModel))
  model.io.cke := io.cke
  model.io.cs  := io.cs
  model.io.ras := io.ras
  model.io.cas := io.cas
  model.io.we  := io.we
  model.io.a   := io.a
  model.io.ba  := io.ba
  model.io.dqm := io.dqm
  model.io.dqi := TriStateInBuf(io.dq, model.io.dqo, model.io.dqoEn.orR)
}

class AXI4SDRAM(address: Seq[AddressSet])(implicit p: Parameters) extends LazyModule {
  val beatBytes = 4
  val node = AXI4SlaveNode(
    Seq(
      AXI4SlavePortParameters(
        Seq(
          AXI4SlaveParameters(
            address       = address,
            executable    = true,
            supportsWrite = TransferSizes(1, beatBytes),
            supportsRead  = TransferSizes(1, beatBytes),
            interleavedId = Some(0)
          )
        ),
        beatBytes = beatBytes
      )
    )
  )

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val (in, _)      = node.in(0)
    val sdram_bundle = IO(new SDRAMIO)

    val msdram = Module(new sdram_top_axi)
    msdram.io.clock := clock
    msdram.io.reset := reset.asBool
    msdram.io.in    <> in
    sdram_bundle    <> msdram.io.sdram
  }
}

class APBSDRAM(address: Seq[AddressSet])(implicit p: Parameters) extends LazyModule {
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
    val (in, _)      = node.in(0)
    val sdram_bundle = IO(new SDRAMIO)

    val msdram = Module(new sdram_top_apb)
    msdram.io.clock := clock
    msdram.io.reset := reset.asBool
    msdram.io.in    <> in
    sdram_bundle    <> msdram.io.sdram
  }
}
