package ysyx

import chisel3._
import chisel3.util._

import freechips.rocketchip.amba.apb._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

class PS2IO extends Bundle {
  val clk  = Input(Bool())
  val data = Input(Bool())
}

class VPs2KbdRecv extends BlackBox with HasBlackBoxInline {
  class Port extends Bundle {
    val clk          = Input(Bool())
    val clr_n        = Input(Bool())
    val ps2_clk      = Input(Bool())
    val ps2_data     = Input(Bool())
    val data_ready_n = Input(Bool())
    val data         = Output(UInt(8.W))
    val data_valid   = Output(Bool())
    val overflow     = Output(Bool())
  }

  val io = IO(new Port)

  setInline(
    "VPs2KbdRecv.v",
    s"""
       |module VPs2KbdRecv(
       |    input        clk,
       |    input        clr_n,
       |    input        ps2_clk,
       |    input        ps2_data,
       |    input        data_ready_n,
       |    output [7:0] data,
       |    output reg   data_valid,
       |    output reg   overflow
       |);
       |  reg [9:0] buffer;
       |  reg [7:0] fifo [7:0];
       |  reg [2:0] w_ptr, r_ptr;
       |  reg [3:0] count;
       |  reg [2:0] ps2_clk_sync;
       |
       |  always @(posedge clk) begin
       |    ps2_clk_sync <= {ps2_clk_sync[1:0], ps2_clk};
       |  end
       |
       |  wire sampling = ps2_clk_sync[2] & ~ps2_clk_sync[1];
       |
       |  always @(posedge clk) begin
       |    if (clr_n == 0) begin
       |      count <= 0;
       |      w_ptr <= 0;
       |      r_ptr <= 0;
       |      overflow <= 0;
       |      data_valid <= 0;
       |    end else begin
       |      if (data_valid) begin
       |        if (data_ready_n == 1'b0) begin
       |          r_ptr <= r_ptr + 3'b1;
       |          if (w_ptr == (r_ptr + 1'b1)) begin
       |            data_valid <= 1'b0;
       |          end
       |        end
       |      end
       |      if (sampling) begin
       |        if (count == 4'd10) begin
       |          if ((buffer[0] == 0) &&  // start bit
       |              (ps2_data) &&  // stop bit
       |              (^buffer[9:1])) begin  // odd parity
       |            fifo[w_ptr] <= buffer[8:1];  // kbd scan code
       |            w_ptr <= w_ptr + 3'b1;
       |            data_valid <= 1'b1;
       |            overflow <= overflow | (r_ptr == (w_ptr + 3'b1));
       |          end
       |          count <= 0;  // for next
       |        end else begin
       |          buffer[count] <= ps2_data;
       |          count <= count + 3'b1;
       |        end
       |      end
       |    end
       |  end
       |  assign data = fifo[r_ptr];
       |endmodule
       |""".stripMargin
  )
}

class Ps2Ctrl extends Module {
  class Port extends Bundle {
    val in  = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
    val ps2 = new PS2IO
  }

  val io = IO(new Port)

  private val addrBad = ~(BitPat("b00010000_00000001_00010000_00000???") === io.in.paddr)

  private val recv = Module(new VPs2KbdRecv)

  private val kbdData = RegInit(0.U(8.W))

  private object State extends ChiselEnum {
    val S_Idle  = Value
    val S_Setup = Value
    val S_Rd    = Value
    val S_Done  = Value
  }
  import State._
  private val y = RegInit(S_Idle)
  y := MuxLookup(y, S_Idle)(
    Seq(
      S_Idle -> Mux(io.in.psel, S_Setup, S_Idle),
      S_Setup -> MuxCase(
        S_Rd,
        Seq(
          (~io.in.penable)         -> S_Setup,
          (addrBad | io.in.pwrite) -> S_Done
        )
      ),
      S_Rd   -> S_Done,
      S_Done -> S_Idle
    )
  )

  recv.io.clk          := clock.asBool
  recv.io.clr_n        := ~reset.asBool
  recv.io.ps2_clk      := io.ps2.clk
  recv.io.ps2_data     := io.ps2.data
  recv.io.data_ready_n := ~(y === S_Done)

  kbdData := Mux(y === S_Rd, Mux(recv.io.data_valid, recv.io.data, 0.U), kbdData)

  io.in.pready  := y === S_Done
  io.in.pslverr := y === S_Done & (addrBad | io.in.pwrite)
  io.in.prdata  := kbdData
}

class PS2CtrlIO extends Bundle {
  val clock = Input(Clock())
  val reset = Input(Bool())
  val in    = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
  val ps2   = new PS2IO
}

class ps2_top_apb extends BlackBox {
  val io = IO(new PS2CtrlIO)
}

class ps2Chisel extends RawModule {
  val io = IO(new PS2CtrlIO)

  private val ctrl = withClockAndReset(io.clock, io.reset) { Module(new Ps2Ctrl) }
  ctrl.io.in  <> io.in
  ctrl.io.ps2 <> io.ps2
}

class APBKeyboard(address: Seq[AddressSet])(implicit p: Parameters) extends LazyModule {
  val node = APBSlaveNode(
    Seq(
      APBSlavePortParameters(
        Seq(
          APBSlaveParameters(
            address       = address,
            executable    = false,
            supportsRead  = true,
            supportsWrite = false
          )
        ),
        beatBytes = 4
      )
    )
  )

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val (in, _)    = node.in(0)
    val ps2_bundle = IO(new PS2IO)

    val mps2 = Module(new ps2Chisel)
    mps2.io.clock := clock
    mps2.io.reset := reset
    mps2.io.in    <> in
    ps2_bundle    <> mps2.io.ps2
  }
}
