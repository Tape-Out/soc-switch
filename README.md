# soc-switch

Coreless Ethernet switch SoC built on eswitch.

![maturity](https://img.shields.io/badge/maturity-planned-lightgrey) ![license](https://img.shields.io/badge/license-MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`xirang`](https://github.com/Tape-Out/xirang). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Assembled from one eswitch instance with no RTL of its own. With no core on the chip, a management host configures the switch over the external APB4 port. `ran test soc-switch` runs the schedule gate and an end-to-end testbench that enables three of the four ports over that bus and checks where a broadcast and a unicast go.

## License

Mulan PSL v2.
