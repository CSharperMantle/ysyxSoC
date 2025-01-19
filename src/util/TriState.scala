package ysyx

import chisel3._
import chisel3.util._
import chisel3.experimental.Analog

class TriStateInBuf(bits: Int) extends BlackBox(Map("width" -> bits)) with HasBlackBoxInline {
  val io = IO(new Bundle {
    val dio    = Analog(bits.W)
    val dout   = Input(UInt(bits.W))
    val out_en = Input(Bool())
    val din    = Output(UInt(bits.W))
  })

  setInline(
    "TriStateInBuf.v",
    """module TriStateInBuf #(
      |  parameter width = 1
      |)(
      |    inout  [width-1:0] dio,
      |    input  [width-1:0] dout,
      |    input              out_en,
      |    output [width-1:0] din
      |);
      |  assign din = dio;
      |  assign dio = out_en ? dout : {width{1'bz}};
      |endmodule
    """.stripMargin
  )
}

object TriStateInBuf {
  def apply(dio: Analog, dout: UInt, out_en: Bool): UInt = {
    val buf = Module(new TriStateInBuf(dio.getWidth))
    buf.io.dio    <> dio
    buf.io.dout   := dout
    buf.io.out_en := out_en
    buf.io.din
  }
}

// class TriStateSlice(inWidth: Int, end: Int, start: Int)
//     extends BlackBox(Map("width" -> inWidth, "end" -> end, "start" -> start))
//     with HasBlackBoxInline {
//   require(inWidth >= 1)
//   require(end - start + 1 >= 1)
//   val io = IO(new Bundle {
//     val x = Analog(inWidth.W)
//     val y = Analog((end - start + 1).W)
//   })
//
//   setInline(
//     "TriStateSlice.sv",
//     """
//       |module TriStateSlice #(
//       |  parameter width = 1,
//       |  parameter end = 0,
//       |  parameter start = 0
//       |)(
//       |    inout [width-1:0] x,
//       |    inout [end-start:0] y
//       |);
//       |  assign y = x[end:start];
//       |endmodule
//       |""".stripMargin
//   )
// }
//
// object TriStateSlice {
//   def apply(x: Analog, end: Int, start: Int) = {
//     val slicer = Module(new TriStateSlice(x.getWidth, end, start))
//     slicer.io.x <> x
//     slicer.io.y
//   }
//
//   implicit class TriStateSliceExtension(x: Analog) {
//     def apply(end: Int, start: Int) = {
//       TriStateSlice(x, end, start)
//     }
//   }
// }
