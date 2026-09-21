"""soc-switch 的端到端测试台：片外管理主机经外部 APB4 口配交换机，再从 RMII 引脚灌帧。

这颗没有核，外部总线是唯一的发起方，所以「管理通路真的生效」要单独验：
只打开 0、1、2 号口，3 号口留着关。广播从 0 号口进来，1、2 号口该发，
入口不该发，**关着的 3 号口也不该发**——这一条只有外部总线那两笔写真的落到
交换机上才成立。然后从 1 号口发往刚学到的地址，只该走 0 号口。

激励源借 emac 的 `mkRmiiTx`：前导码、SFD、CRC 都是它现成的。
"""
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)

BASE = 0x1000_0000
A = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55]
B = [0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE]
TAIL = [0x08, 0x00, 0xA5, 0x5A, 0x5A, 0xA5]
F0 = [0xFF] * 6 + A + TAIL      # 0 号口进，广播，源 A
F1 = A + B + TAIL               # 1 号口进，发往 A，源 B
NB = len(F0)

rows = "\n".join(f"      {i}: return (which == 0) ? 8'h{F0[i]:02X} : 8'h{F1[i]:02X};"
                 for i in range(NB))

(out / "SwitchTb.bsv").write_text(f'''package SwitchTb;

import Vector::*;
import GetPut::*;
// 外部总线口的类型在 Apb4 里，引脚上的方法在各自的包里：不 import 就认不出来
import Apb4::*;
import RmiiTx::*;
import RmiiRx::*;
import Eswitch::*;
import SocSwitchPkg::*;

// 由 tb/mkswitchtb.py 生成，勿手改。

typedef enum {{ Cfg, Send0, Gap0, Chk0, Send1, Gap1, Done }}
  Phase deriving (Bits, Eq);

function Bit#(8) frameByte(Bit#(8) i, Bit#(1) which);
  case (i)
{rows}
    default: return 0;
  endcase
endfunction

(* synthesize *)
module mkSwitchTb(Empty);
  SocSwitchIfc soc <- mkSocSwitch;
  RmiiTxIfc gen <- mkRmiiTx;

  Reg#(Phase)    ph    <- mkReg(Cfg);
  Reg#(Bit#(2))  st    <- mkReg(0);      // APB4 的两拍：0 建立，1 访问
  Reg#(Bit#(2))  ci    <- mkReg(0);      // 第几笔配置写
  Reg#(Bit#(8))  fi    <- mkReg(0);
  Reg#(Bit#(1))  which <- mkReg(0);
  Reg#(Bit#(3))  srcPort <- mkReg(0);
  Reg#(Bit#(16)) g     <- mkReg(0);
  Reg#(Bit#(32)) cyc   <- mkReg(0);
  Reg#(Bool)     bad   <- mkReg(False);
  Vector#(4, Reg#(Bit#(16))) txN  <- replicateM(mkReg(0));
  Vector#(4, Reg#(Bit#(16))) mark <- replicateM(mkReg(0));

  Bool cfgBus = ph == Cfg && ci < 2;
  // 先 porten 只开 0、1、2 号口，再 ctrl 置 en 与 learn
  Bit#(32) addr = 32'h{BASE:08X} + ((ci == 0) ? 32'h4 : 32'h0);
  Bit#(32) wdat = (ci == 0) ? 32'h00000007 : 32'h00000003;

  rule drive;
    soc.bus.req(addr, 3'b000, cfgBus, cfgBus && st == 1, True, wdat, 4'hF);
    for (Integer p = 0; p < 4; p = p + 1) begin
      Bool sel = fromInteger(p) == srcPort;
      soc.sw0_pins.rx[p].wire_in(sel ? gen.pins.txd : 0,
                                 sel ? gen.pins.tx_en : False, False);
    end
  endrule

  rule countTx;
    for (Integer p = 0; p < 4; p = p + 1)
      if (soc.sw0_pins.tx[p].tx_en) txN[p] <= txN[p] + 1;
  endrule

  rule tick_;
    cyc <= cyc + 1;
    if (cyc > 40000) begin
      $display("TIMEOUT in phase %0d", pack(ph));
      $finish(1);
    end
  endrule

  rule cfg (ph == Cfg);
    if (ci == 2) begin
      ph <= Send0;
    end else if (st == 0) st <= 1;
    else if (soc.bus.pready) begin
      st <= 0;
      ci <= ci + 1;
    end
  endrule

  rule feed (ph == Send0 || ph == Send1);
    gen.tx.put(tuple2(frameByte(fi, which), fi == fromInteger({NB - 1})));
    if (fi + 1 == fromInteger({NB})) begin
      fi <= 0;
      g  <= 0;
      ph <= (ph == Send0) ? Gap0 : Gap1;
    end else fi <= fi + 1;
  endrule

  rule gap (ph == Gap0 || ph == Gap1);
    if (g > 300) begin
      g <= 0;
      ph <= (ph == Gap0) ? Chk0 : Done;
    end else g <= g + 1;
  endrule

  rule chk0 (ph == Chk0);
    Bool wrong = False;
    if (txN[0] != 0) begin
      $display("FAIL the broadcast came back out of port 0, where it arrived");
      wrong = True;
    end
    if (txN[1] == 0 || txN[2] == 0) begin
      $display("FAIL the broadcast did not reach both enabled ports: %0d %0d", txN[1], txN[2]);
      wrong = True;
    end
    if (txN[3] != 0) begin
      $display("FAIL port 3 transmitted although the management host never enabled it");
      wrong = True;
    end
    if (wrong) bad <= True;
    for (Integer p = 0; p < 4; p = p + 1)
      mark[p] <= txN[p];
    which <= 1;
    srcPort <= 1;
    ph <= Send1;
  endrule

  rule fin (ph == Done);
    Bool wrong = bad;
    if (txN[0] == mark[0]) begin
      $display("FAIL the unicast to the address learned on port 0 never went out of port 0");
      wrong = True;
    end
    if (txN[2] != mark[2] || txN[3] != mark[3]) begin
      $display("FAIL the unicast to a learned address was flooded");
      wrong = True;
    end
    if (wrong) $display("FAILED");
    else $display("PASS soc-switch: the external bus configures the switch, a broadcast floods only the enabled ports, a unicast goes where its address was learned");
    $finish(wrong ? 1 : 0);
  endrule
endmodule

endpackage
''', encoding="utf-8")
print("  soc-switch 端到端测试台就位")
