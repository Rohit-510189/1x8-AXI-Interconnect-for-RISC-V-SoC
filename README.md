

## Overview

RISC-V SoC project focusing on an **AXI Interconnect** connecting one master to eight slaves.

### Architecture

```text
                 AXI Master
                     |
                     |
              +------+------+
              | AXI         |
              | Interconnect|
              +------+------+
                     |
       +------+------+------+------+------+------+------+------+
       |      |      |      |      |      |      |      |
      S0     S1     S2     S3     S4     S5     S6     S7
```

The interconnect performs:

* Address decoding
* Slave selection
* AXI read/write transaction routing
* Arbitration

## Project Structure

```text
HC_Proj_095/
├── rtl/
│   └── interconnect/
│       ├── priority_encoder.v
│       ├── arbiter.v
│       ├── axi_interconnect.v
│       ├── axi_interconnect_wrap_1x8.v
│       └── tb_axi_interconnect_wrap_1x8.v
│
├── scripts/
└── run/
    └── filelist.f
```

## Simulation

Simulation is performed using **Synopsys VCS**.

### Compile

From the `run` directory:

```bash
cd /home/student/Documents/HC_Proj_095/run
```

```bash
vcs -full64 -sverilog \
    -debug_access+all \
    -kdb \
    -top tb_axi_interconnect_wrap_1x8 \
    -f filelist.f \
    -l compile.log
```

If UVM is required:

```bash
vcs -full64 -sverilog -ntb_opts uvm \
    -debug_access+all \
    -kdb \
    -top tb_axi_interconnect_wrap_1x8 \
    -f filelist.f \
    -l compile.log
```

### Run Simulation

```bash
./simv
```

Save simulation output:

```bash
./simv -l sim_run.log
```

### Clean VCS Files

```bash
rm -rf simv simv.daidir csrc DVEfiles AN.DB ucli.key *.log
```
