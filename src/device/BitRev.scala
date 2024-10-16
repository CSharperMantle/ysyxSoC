package ysyx

import chisel3._
import chisel3.util._

class bitrev extends BlackBox {
  val io = IO(Flipped(new SPIIO(1)))
}

class bitrevChisel extends RawModule { // we do not need clock and reset
  val io = IO(Flipped(new SPIIO(1)))

  private val clock = io.sck.asClock
  private val reset = io.ss(0).asAsyncReset

  private val rxCtr = withClockAndReset(clock, reset) {
    RegInit(0xf.U(4.W))
  }
  private val txCtr = withClockAndReset(clock, reset) {
    RegInit(0.U(4.W))
  }

  rxCtr := rxCtr + 1.U
  txCtr := txCtr - 1.U

  private val data = withClockAndReset(clock, reset) {
    RegInit(0.U(8.W))
  }

  data := MuxCase(
    data,
    Seq(
      ((rxCtr < 7.U) | (rxCtr === 0xf.U)) -> Cat(data(6, 0), io.mosi),
      (txCtr < 8.U) -> Cat(0.U(1.W), data(7, 1))
    )
  )

  io.miso := Mux(reset.asBool | (rxCtr < 8.U), 1.B, data(0))
}
