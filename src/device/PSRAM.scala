package ysyx

import chisel3._
import chisel3.util._
import chisel3.experimental.Analog

import freechips.rocketchip.amba.apb._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

class QSPIIO extends Bundle {
  val sck  = Output(Bool())
  val ce_n = Output(Bool())
  val dio  = Analog(4.W)
}

class psram_top_apb extends BlackBox {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Reset())
    val in    = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
    val qspi  = new QSPIIO
  })
}

class PsramModelBlackBox extends BlackBox with HasBlackBoxInline {
  class Port extends Bundle {
    val addr  = Input(UInt(32.W))
    val wData = Input(UInt(32.W))
    val wEn   = Input(Bool())
    val rEn   = Input(Bool())
    val rData = Output(UInt(32.W))
  }

  val io = IO(new Port)

  setInline(
    s"${name}.sv",
    s"""
       |module ${name} (
       |  input      [31:0] addr,
       |  input      [31:0] wData,
       |  input             wEn,
       |  input             rEn,
       |  output reg [31:0] rData
       |);
       |  import "DPI-C" function void soc_dpi_psram_read(input  int addr,
       |                                                  output int data);
       |  import "DPI-C" function void soc_dpi_psram_write(input int addr,
       |                                                   input int data);
       |
       |  always_comb begin
       |    if (rEn) begin
       |      soc_dpi_psram_read(addr, rData);
       |    end else begin
       |      rData = 32'h0;
       |    end
       |  end
       |
       |  always_comb begin
       |    if (wEn) begin
       |      soc_dpi_psram_write(addr, wData);
       |    end
       |  end
       |endmodule
       |""".stripMargin
  )
}

class PsramModel extends Module {
  class Port extends Bundle {
    val din    = Input(UInt(4.W))
    val dout   = Output(UInt(4.W))
    val doutEn = Output(Bool())
  }

  val io = IO(new Port)

  private object Cmd {
    val Read  = 0xeb.U(8.W)
    val Write = 0x38.U(8.W)
  }

  private val backend = Module(new PsramModelBlackBox)

  private val cmd  = RegInit(0.U(8.W))
  private val addr = RegInit(0.U(32.W))
  private val data = withReset(0.B) { RegInit(0.U(32.W)) }

  private val cmdCtr   = RegInit(0.U(4.W))
  private val addrCtr  = RegInit(0.U(4.W))
  private val dummyCtr = RegInit(0.U(4.W))
  private val dataCtr  = RegInit(0.U(4.W))

  private object State extends ChiselEnum {
    val S_ReadCmd   = Value
    val S_ReadAddr  = Value
    val S_ReadDummy = Value
    val S_RecvData  = Value
    val S_SendData  = Value
    val S_Idle      = Value
  }
  import State._

  private val cmdAction = MuxLookup(cmd, S_Idle)(
    Seq(
      Cmd.Read  -> S_ReadDummy,
      Cmd.Write -> S_RecvData
    )
  )

  private val y = RegInit(S_ReadCmd)
  y := MuxLookup(y, S_Idle)(
    Seq(
      S_ReadCmd   -> Mux(cmdCtr >= 7.U, S_ReadAddr, S_ReadCmd),
      S_ReadAddr  -> Mux(addrCtr >= 5.U, cmdAction, S_ReadAddr),
      S_ReadDummy -> Mux(dummyCtr >= 5.U, S_SendData, S_ReadDummy),
      S_SendData  -> Mux(dataCtr >= 7.U, S_Idle, S_SendData),
      S_RecvData  -> Mux(dataCtr >= 7.U, S_Idle, S_RecvData)
    )
  )

  cmdCtr   := Mux(y === S_ReadCmd, cmdCtr + 1.U, cmdCtr)
  addrCtr  := Mux(y === S_ReadAddr, addrCtr + 1.U, addrCtr)
  dummyCtr := Mux(y === S_ReadDummy, dummyCtr + 1.U, dummyCtr)
  dataCtr  := Mux((y === S_SendData) | (y === S_RecvData), dataCtr + 1.U, dataCtr)

  cmd  := Mux(y === S_ReadCmd, Cat(cmd(6, 0), io.din(0)), cmd)
  addr := Mux(y === S_ReadAddr, Cat(addr(27, 0), io.din), addr)
  data := MuxLookup(y, data)(
    Seq(
      S_ReadCmd   -> 0.U,
      S_ReadDummy -> backend.io.rData,
      S_RecvData  -> Cat(data(27, 0), io.din),
      S_SendData  -> Cat(0.U(4.W), data(31, 28))
    )
  )

  backend.io.addr  := addr
  backend.io.rEn   := y === S_ReadDummy
  backend.io.wEn   := cmd === Cmd.Write & reset.asBool
  backend.io.wData := Cat(data(7, 0), data(15, 8), data(23, 16), data(31, 24))

  io.dout   := Mux(dataCtr(0) === 1.B, data(7, 4), data(3, 0))
  io.doutEn := y === S_SendData
}

class psram extends BlackBox {
  val io = IO(Flipped(new QSPIIO))
}

class psramChisel extends RawModule {
  val io = IO(Flipped(new QSPIIO))

  private val model = withClockAndReset(io.sck.asClock, io.ce_n.asAsyncReset) {
    Module(new PsramModel)
  }
  model.io.din := TriStateInBuf(io.dio, model.io.dout, model.io.doutEn)
}

class APBPSRAM(address: Seq[AddressSet])(implicit p: Parameters) extends LazyModule {
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
    val qspi_bundle = IO(new QSPIIO)

    val mpsram = Module(new psram_top_apb)
    mpsram.io.clock := clock
    mpsram.io.reset := reset
    mpsram.io.in    <> in
    qspi_bundle     <> mpsram.io.qspi
  }
}
