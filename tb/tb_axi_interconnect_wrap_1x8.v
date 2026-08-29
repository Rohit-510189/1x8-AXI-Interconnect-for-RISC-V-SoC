/*
 * Testbench: tb_axi_interconnect_wrap_1x8
 *
 * Verifies basic AXI4 single-beat write and read operations through the
 * axi_interconnect_wrap_1x8 (1 slave port, 8 master ports).
 *
 * Address map used (matching wrapper defaults, ADDR_WIDTH=24-bit windows):
 *   M00 base = 32'h0000_0000
 *   M01 base = 32'h0100_0000   (offset chosen to fall in M01 window)
 *   M02 base = 32'h0200_0000
 *   M03 base = 32'h0300_0000
 *   M04 base = 32'h0400_0000
 *   M05 base = 32'h0500_0000
 *   M06 base = 32'h0600_0000
 *   M07 base = 32'h0700_0000
 *
 * NOTE: The wrapper defaults use small decimal base addresses (0,4,8…) with
 * 24-bit windows; they overlap.  This TB overrides the parameters to use
 * non-overlapping 16 MB windows so each master is uniquely reachable.
 *
 * Simulation tool: iverilog / ModelSim / Questa
 * Compile example (iverilog):
 *   iverilog -g2005 -o sim.vvp \
 *       tb_axi_interconnect_wrap_1x8.v \
 *       axi_interconnect_wrap_1x8.v   \
 *       axi_interconnect.v            \
 *       arbiter.v                     \
 *       priority_encoder.v            \
 *   && vvp sim.vvp
 */

`timescale 1ns/1ps
`default_nettype none

module tb_axi_interconnect_wrap_1x8;

// -------------------------------------------------------------------------
// Parameters
// -------------------------------------------------------------------------
localparam DATA_WIDTH = 32;
localparam ADDR_WIDTH = 32;
localparam STRB_WIDTH = DATA_WIDTH / 8;   // 4
localparam ID_WIDTH   = 8;

// Non-overlapping 16 MB windows (bit 27:24 selects the master)
localparam M00_BASE = 32'h0000_0000;
localparam M01_BASE = 32'h0100_0000;
localparam M02_BASE = 32'h0200_0000;
localparam M03_BASE = 32'h0300_0000;
localparam M04_BASE = 32'h0400_0000;
localparam M05_BASE = 32'h0500_0000;
localparam M06_BASE = 32'h0600_0000;
localparam M07_BASE = 32'h0700_0000;
localparam WIN_BITS  = 32'd24;  // 16 MB window per master

// Clock period
localparam CLK_PERIOD = 10;  // 10 ns -> 100 MHz

// -------------------------------------------------------------------------
// Clock & Reset
// -------------------------------------------------------------------------
reg clk = 0;
reg rst = 1;

always #(CLK_PERIOD/2) clk = ~clk;

// -------------------------------------------------------------------------
// AXI Slave-side signals (driven by this TB)
// -------------------------------------------------------------------------
reg  [ID_WIDTH-1:0]   s_awid;
reg  [ADDR_WIDTH-1:0] s_awaddr;
reg  [7:0]            s_awlen;
reg  [2:0]            s_awsize;
reg  [1:0]            s_awburst;
reg                   s_awlock;
reg  [3:0]            s_awcache;
reg  [2:0]            s_awprot;
reg  [3:0]            s_awqos;
reg                   s_awvalid;
wire                  s_awready;

reg  [DATA_WIDTH-1:0] s_wdata;
reg  [STRB_WIDTH-1:0] s_wstrb;
reg                   s_wlast;
reg                   s_wvalid;
wire                  s_wready;

wire [ID_WIDTH-1:0]   s_bid;
wire [1:0]            s_bresp;
wire                  s_bvalid;
reg                   s_bready;

reg  [ID_WIDTH-1:0]   s_arid;
reg  [ADDR_WIDTH-1:0] s_araddr;
reg  [7:0]            s_arlen;
reg  [2:0]            s_arsize;
reg  [1:0]            s_arburst;
reg                   s_arlock;
reg  [3:0]            s_arcache;
reg  [2:0]            s_arprot;
reg  [3:0]            s_arqos;
reg                   s_arvalid;
wire                  s_arready;

wire [ID_WIDTH-1:0]   s_rid;
wire [DATA_WIDTH-1:0] s_rdata;
wire [1:0]            s_rresp;
wire                  s_rlast;
wire                  s_rvalid;
reg                   s_rready;

// -------------------------------------------------------------------------
// AXI Master-side signals (slave memory models)
// -------------------------------------------------------------------------
// We instantiate 8 tiny register-based slave memories.
// Each memory has MEM_DEPTH 32-bit words.
localparam MEM_DEPTH = 16;

// Per-master AXI signals (master side of DUT = slave side of memory model)
wire [ID_WIDTH-1:0]   m_awid    [0:7];
wire [ADDR_WIDTH-1:0] m_awaddr  [0:7];
wire [7:0]            m_awlen   [0:7];
wire [2:0]            m_awsize  [0:7];
wire [1:0]            m_awburst [0:7];
wire                  m_awlock  [0:7];
wire [3:0]            m_awcache [0:7];
wire [2:0]            m_awprot  [0:7];
wire [3:0]            m_awqos   [0:7];
wire [3:0]            m_awregion[0:7];
wire                  m_awvalid [0:7];
reg                   m_awready [0:7];

wire [DATA_WIDTH-1:0] m_wdata   [0:7];
wire [STRB_WIDTH-1:0] m_wstrb   [0:7];
wire                  m_wlast   [0:7];
wire                  m_wvalid  [0:7];
reg                   m_wready  [0:7];

reg  [ID_WIDTH-1:0]   m_bid     [0:7];
reg  [1:0]            m_bresp   [0:7];
reg                   m_bvalid  [0:7];
wire                  m_bready  [0:7];

wire [ID_WIDTH-1:0]   m_arid    [0:7];
wire [ADDR_WIDTH-1:0] m_araddr  [0:7];
wire [7:0]            m_arlen   [0:7];
wire [2:0]            m_arsize  [0:7];
wire [1:0]            m_arburst [0:7];
wire                  m_arlock  [0:7];
wire [3:0]            m_arcache [0:7];
wire [2:0]            m_arprot  [0:7];
wire [3:0]            m_arqos   [0:7];
wire [3:0]            m_arregion[0:7];
wire                  m_arvalid [0:7];
reg                   m_arready [0:7];

reg  [ID_WIDTH-1:0]   m_rid     [0:7];
reg  [DATA_WIDTH-1:0] m_rdata   [0:7];
reg  [1:0]            m_rresp   [0:7];
reg                   m_rlast   [0:7];
reg                   m_rvalid  [0:7];
wire                  m_rready  [0:7];

// -------------------------------------------------------------------------
// Simple register-file slave memory model (one per master port)
// -------------------------------------------------------------------------
// Each model responds with 1-cycle ready latency for both AW/W and AR.
// Stores MEM_DEPTH words; address bits [5:2] index into the array.

reg [DATA_WIDTH-1:0] mem [0:7][0:MEM_DEPTH-1];  // 8 memories

integer m, i;

// State machine states
localparam SM_IDLE   = 2'd0;
localparam SM_WDATA  = 2'd1;
localparam SM_BRESP  = 2'd2;
localparam SM_RDATA  = 2'd3;

reg [1:0] wr_state [0:7];
reg [1:0] rd_state [0:7];

reg [ID_WIDTH-1:0]   wr_id    [0:7];
reg [ADDR_WIDTH-1:0] wr_addr  [0:7];
reg [ID_WIDTH-1:0]   rd_id    [0:7];
reg [ADDR_WIDTH-1:0] rd_addr  [0:7];

// Generate one slave memory model per master port
genvar g;
generate
for (g = 0; g < 8; g = g + 1) begin : slave_mem

    // Write channel state machine
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wr_state[g]  <= SM_IDLE;
            m_awready[g] <= 1'b0;
            m_wready[g]  <= 1'b0;
            m_bvalid[g]  <= 1'b0;
            m_bid[g]     <= {ID_WIDTH{1'b0}};
            m_bresp[g]   <= 2'b00;
        end else begin
            case (wr_state[g])
                SM_IDLE: begin
                    m_awready[g] <= 1'b1;  // always ready to accept address
                    m_wready[g]  <= 1'b0;
                    m_bvalid[g]  <= 1'b0;
                    if (m_awvalid[g] && m_awready[g]) begin
                        wr_id[g]     <= m_awid[g];
                        wr_addr[g]   <= m_awaddr[g];
                        m_awready[g] <= 1'b0;
                        m_wready[g]  <= 1'b1;
                        wr_state[g]  <= SM_WDATA;
                    end
                end
                SM_WDATA: begin
                    if (m_wvalid[g] && m_wready[g]) begin
                        // Write to memory; use bits [5:2] as word index
                        mem[g][wr_addr[g][5:2]] <= m_wdata[g];
                        if (m_wlast[g]) begin
                            m_wready[g]  <= 1'b0;
                            m_bvalid[g]  <= 1'b1;
                            m_bid[g]     <= wr_id[g];
                            m_bresp[g]   <= 2'b00;  // OKAY
                            wr_state[g]  <= SM_BRESP;
                        end
                    end
                end
                SM_BRESP: begin
                    if (m_bvalid[g] && m_bready[g]) begin
                        m_bvalid[g] <= 1'b0;
                        wr_state[g] <= SM_IDLE;
                    end
                end
                default: wr_state[g] <= SM_IDLE;
            endcase
        end
    end

    // Read channel state machine
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rd_state[g]  <= SM_IDLE;
            m_arready[g] <= 1'b0;
            m_rvalid[g]  <= 1'b0;
            m_rdata[g]   <= {DATA_WIDTH{1'b0}};
            m_rresp[g]   <= 2'b00;
            m_rlast[g]   <= 1'b0;
            m_rid[g]     <= {ID_WIDTH{1'b0}};
        end else begin
            case (rd_state[g])
                SM_IDLE: begin
                    m_arready[g] <= 1'b1;
                    m_rvalid[g]  <= 1'b0;
                    if (m_arvalid[g] && m_arready[g]) begin
                        rd_id[g]     <= m_arid[g];
                        rd_addr[g]   <= m_araddr[g];
                        m_arready[g] <= 1'b0;
                        rd_state[g]  <= SM_RDATA;
                    end
                end
                SM_RDATA: begin
                    m_rvalid[g] <= 1'b1;
                    m_rdata[g]  <= mem[g][rd_addr[g][5:2]];
                    m_rresp[g]  <= 2'b00;  // OKAY
                    m_rlast[g]  <= 1'b1;
                    m_rid[g]    <= rd_id[g];
                    if (m_rvalid[g] && m_rready[g]) begin
                        m_rvalid[g] <= 1'b0;
                        m_rlast[g]  <= 1'b0;
                        rd_state[g] <= SM_IDLE;
                    end
                end
                default: rd_state[g] <= SM_IDLE;
            endcase
        end
    end

end
endgenerate

// -------------------------------------------------------------------------
// DUT instantiation
// -------------------------------------------------------------------------
axi_interconnect_wrap_1x8 #(
    .DATA_WIDTH    (DATA_WIDTH),
    .ADDR_WIDTH    (ADDR_WIDTH),
    .STRB_WIDTH    (STRB_WIDTH),
    .ID_WIDTH      (ID_WIDTH),
    .AWUSER_ENABLE (0),
    .WUSER_ENABLE  (0),
    .BUSER_ENABLE  (0),
    .ARUSER_ENABLE (0),
    .RUSER_ENABLE  (0),
    .FORWARD_ID    (0),
    .M_REGIONS     (1),
    // Non-overlapping 16 MB base addresses
    .M00_BASE_ADDR (M00_BASE), .M00_ADDR_WIDTH ({1{WIN_BITS}}), .M00_CONNECT_READ(1'b1), .M00_CONNECT_WRITE(1'b1), .M00_SECURE(1'b0),
    .M01_BASE_ADDR (M01_BASE), .M01_ADDR_WIDTH ({1{WIN_BITS}}), .M01_CONNECT_READ(1'b1), .M01_CONNECT_WRITE(1'b1), .M01_SECURE(1'b0),
    .M02_BASE_ADDR (M02_BASE), .M02_ADDR_WIDTH ({1{WIN_BITS}}), .M02_CONNECT_READ(1'b1), .M02_CONNECT_WRITE(1'b1), .M02_SECURE(1'b0),
    .M03_BASE_ADDR (M03_BASE), .M03_ADDR_WIDTH ({1{WIN_BITS}}), .M03_CONNECT_READ(1'b1), .M03_CONNECT_WRITE(1'b1), .M03_SECURE(1'b0),
    .M04_BASE_ADDR (M04_BASE), .M04_ADDR_WIDTH ({1{WIN_BITS}}), .M04_CONNECT_READ(1'b1), .M04_CONNECT_WRITE(1'b1), .M04_SECURE(1'b0),
    .M05_BASE_ADDR (M05_BASE), .M05_ADDR_WIDTH ({1{WIN_BITS}}), .M05_CONNECT_READ(1'b1), .M05_CONNECT_WRITE(1'b1), .M05_SECURE(1'b0),
    .M06_BASE_ADDR (M06_BASE), .M06_ADDR_WIDTH ({1{WIN_BITS}}), .M06_CONNECT_READ(1'b1), .M06_CONNECT_WRITE(1'b1), .M06_SECURE(1'b0),
    .M07_BASE_ADDR (M07_BASE), .M07_ADDR_WIDTH ({1{WIN_BITS}}), .M07_CONNECT_READ(1'b1), .M07_CONNECT_WRITE(1'b1), .M07_SECURE(1'b0)
) dut (
    .clk (clk),
    .rst (rst),

    // --- Slave port (TB drives) ---
    .s00_axi_awid    (s_awid),
    .s00_axi_awaddr  (s_awaddr),
    .s00_axi_awlen   (s_awlen),
    .s00_axi_awsize  (s_awsize),
    .s00_axi_awburst (s_awburst),
    .s00_axi_awlock  (s_awlock),
    .s00_axi_awcache (s_awcache),
    .s00_axi_awprot  (s_awprot),
    .s00_axi_awqos   (s_awqos),
    .s00_axi_awuser  (1'b0),
    .s00_axi_awvalid (s_awvalid),
    .s00_axi_awready (s_awready),

    .s00_axi_wdata   (s_wdata),
    .s00_axi_wstrb   (s_wstrb),
    .s00_axi_wlast   (s_wlast),
    .s00_axi_wuser   (1'b0),
    .s00_axi_wvalid  (s_wvalid),
    .s00_axi_wready  (s_wready),

    .s00_axi_bid     (s_bid),
    .s00_axi_bresp   (s_bresp),
    .s00_axi_buser   (),
    .s00_axi_bvalid  (s_bvalid),
    .s00_axi_bready  (s_bready),

    .s00_axi_arid    (s_arid),
    .s00_axi_araddr  (s_araddr),
    .s00_axi_arlen   (s_arlen),
    .s00_axi_arsize  (s_arsize),
    .s00_axi_arburst (s_arburst),
    .s00_axi_arlock  (s_arlock),
    .s00_axi_arcache (s_arcache),
    .s00_axi_arprot  (s_arprot),
    .s00_axi_arqos   (s_arqos),
    .s00_axi_aruser  (1'b0),
    .s00_axi_arvalid (s_arvalid),
    .s00_axi_arready (s_arready),

    .s00_axi_rid     (s_rid),
    .s00_axi_rdata   (s_rdata),
    .s00_axi_rresp   (s_rresp),
    .s00_axi_rlast   (s_rlast),
    .s00_axi_ruser   (),
    .s00_axi_rvalid  (s_rvalid),
    .s00_axi_rready  (s_rready),

    // --- Master ports (slave memory models) ---
    .m00_axi_awid(m_awid[0]), .m00_axi_awaddr(m_awaddr[0]), .m00_axi_awlen(m_awlen[0]),
    .m00_axi_awsize(m_awsize[0]), .m00_axi_awburst(m_awburst[0]), .m00_axi_awlock(m_awlock[0]),
    .m00_axi_awcache(m_awcache[0]), .m00_axi_awprot(m_awprot[0]), .m00_axi_awqos(m_awqos[0]),
    .m00_axi_awregion(m_awregion[0]), .m00_axi_awuser(), .m00_axi_awvalid(m_awvalid[0]),
    .m00_axi_awready(m_awready[0]),
    .m00_axi_wdata(m_wdata[0]), .m00_axi_wstrb(m_wstrb[0]), .m00_axi_wlast(m_wlast[0]),
    .m00_axi_wuser(), .m00_axi_wvalid(m_wvalid[0]), .m00_axi_wready(m_wready[0]),
    .m00_axi_bid(m_bid[0]), .m00_axi_bresp(m_bresp[0]), .m00_axi_buser(1'b0),
    .m00_axi_bvalid(m_bvalid[0]), .m00_axi_bready(m_bready[0]),
    .m00_axi_arid(m_arid[0]), .m00_axi_araddr(m_araddr[0]), .m00_axi_arlen(m_arlen[0]),
    .m00_axi_arsize(m_arsize[0]), .m00_axi_arburst(m_arburst[0]), .m00_axi_arlock(m_arlock[0]),
    .m00_axi_arcache(m_arcache[0]), .m00_axi_arprot(m_arprot[0]), .m00_axi_arqos(m_arqos[0]),
    .m00_axi_arregion(m_arregion[0]), .m00_axi_aruser(), .m00_axi_arvalid(m_arvalid[0]),
    .m00_axi_arready(m_arready[0]),
    .m00_axi_rid(m_rid[0]), .m00_axi_rdata(m_rdata[0]), .m00_axi_rresp(m_rresp[0]),
    .m00_axi_rlast(m_rlast[0]), .m00_axi_ruser(1'b0), .m00_axi_rvalid(m_rvalid[0]),
    .m00_axi_rready(m_rready[0]),

    .m01_axi_awid(m_awid[1]), .m01_axi_awaddr(m_awaddr[1]), .m01_axi_awlen(m_awlen[1]),
    .m01_axi_awsize(m_awsize[1]), .m01_axi_awburst(m_awburst[1]), .m01_axi_awlock(m_awlock[1]),
    .m01_axi_awcache(m_awcache[1]), .m01_axi_awprot(m_awprot[1]), .m01_axi_awqos(m_awqos[1]),
    .m01_axi_awregion(m_awregion[1]), .m01_axi_awuser(), .m01_axi_awvalid(m_awvalid[1]),
    .m01_axi_awready(m_awready[1]),
    .m01_axi_wdata(m_wdata[1]), .m01_axi_wstrb(m_wstrb[1]), .m01_axi_wlast(m_wlast[1]),
    .m01_axi_wuser(), .m01_axi_wvalid(m_wvalid[1]), .m01_axi_wready(m_wready[1]),
    .m01_axi_bid(m_bid[1]), .m01_axi_bresp(m_bresp[1]), .m01_axi_buser(1'b0),
    .m01_axi_bvalid(m_bvalid[1]), .m01_axi_bready(m_bready[1]),
    .m01_axi_arid(m_arid[1]), .m01_axi_araddr(m_araddr[1]), .m01_axi_arlen(m_arlen[1]),
    .m01_axi_arsize(m_arsize[1]), .m01_axi_arburst(m_arburst[1]), .m01_axi_arlock(m_arlock[1]),
    .m01_axi_arcache(m_arcache[1]), .m01_axi_arprot(m_arprot[1]), .m01_axi_arqos(m_arqos[1]),
    .m01_axi_arregion(m_arregion[1]), .m01_axi_aruser(), .m01_axi_arvalid(m_arvalid[1]),
    .m01_axi_arready(m_arready[1]),
    .m01_axi_rid(m_rid[1]), .m01_axi_rdata(m_rdata[1]), .m01_axi_rresp(m_rresp[1]),
    .m01_axi_rlast(m_rlast[1]), .m01_axi_ruser(1'b0), .m01_axi_rvalid(m_rvalid[1]),
    .m01_axi_rready(m_rready[1]),

    .m02_axi_awid(m_awid[2]), .m02_axi_awaddr(m_awaddr[2]), .m02_axi_awlen(m_awlen[2]),
    .m02_axi_awsize(m_awsize[2]), .m02_axi_awburst(m_awburst[2]), .m02_axi_awlock(m_awlock[2]),
    .m02_axi_awcache(m_awcache[2]), .m02_axi_awprot(m_awprot[2]), .m02_axi_awqos(m_awqos[2]),
    .m02_axi_awregion(m_awregion[2]), .m02_axi_awuser(), .m02_axi_awvalid(m_awvalid[2]),
    .m02_axi_awready(m_awready[2]),
    .m02_axi_wdata(m_wdata[2]), .m02_axi_wstrb(m_wstrb[2]), .m02_axi_wlast(m_wlast[2]),
    .m02_axi_wuser(), .m02_axi_wvalid(m_wvalid[2]), .m02_axi_wready(m_wready[2]),
    .m02_axi_bid(m_bid[2]), .m02_axi_bresp(m_bresp[2]), .m02_axi_buser(1'b0),
    .m02_axi_bvalid(m_bvalid[2]), .m02_axi_bready(m_bready[2]),
    .m02_axi_arid(m_arid[2]), .m02_axi_araddr(m_araddr[2]), .m02_axi_arlen(m_arlen[2]),
    .m02_axi_arsize(m_arsize[2]), .m02_axi_arburst(m_arburst[2]), .m02_axi_arlock(m_arlock[2]),
    .m02_axi_arcache(m_arcache[2]), .m02_axi_arprot(m_arprot[2]), .m02_axi_arqos(m_arqos[2]),
    .m02_axi_arregion(m_arregion[2]), .m02_axi_aruser(), .m02_axi_arvalid(m_arvalid[2]),
    .m02_axi_arready(m_arready[2]),
    .m02_axi_rid(m_rid[2]), .m02_axi_rdata(m_rdata[2]), .m02_axi_rresp(m_rresp[2]),
    .m02_axi_rlast(m_rlast[2]), .m02_axi_ruser(1'b0), .m02_axi_rvalid(m_rvalid[2]),
    .m02_axi_rready(m_rready[2]),

    .m03_axi_awid(m_awid[3]), .m03_axi_awaddr(m_awaddr[3]), .m03_axi_awlen(m_awlen[3]),
    .m03_axi_awsize(m_awsize[3]), .m03_axi_awburst(m_awburst[3]), .m03_axi_awlock(m_awlock[3]),
    .m03_axi_awcache(m_awcache[3]), .m03_axi_awprot(m_awprot[3]), .m03_axi_awqos(m_awqos[3]),
    .m03_axi_awregion(m_awregion[3]), .m03_axi_awuser(), .m03_axi_awvalid(m_awvalid[3]),
    .m03_axi_awready(m_awready[3]),
    .m03_axi_wdata(m_wdata[3]), .m03_axi_wstrb(m_wstrb[3]), .m03_axi_wlast(m_wlast[3]),
    .m03_axi_wuser(), .m03_axi_wvalid(m_wvalid[3]), .m03_axi_wready(m_wready[3]),
    .m03_axi_bid(m_bid[3]), .m03_axi_bresp(m_bresp[3]), .m03_axi_buser(1'b0),
    .m03_axi_bvalid(m_bvalid[3]), .m03_axi_bready(m_bready[3]),
    .m03_axi_arid(m_arid[3]), .m03_axi_araddr(m_araddr[3]), .m03_axi_arlen(m_arlen[3]),
    .m03_axi_arsize(m_arsize[3]), .m03_axi_arburst(m_arburst[3]), .m03_axi_arlock(m_arlock[3]),
    .m03_axi_arcache(m_arcache[3]), .m03_axi_arprot(m_arprot[3]), .m03_axi_arqos(m_arqos[3]),
    .m03_axi_arregion(m_arregion[3]), .m03_axi_aruser(), .m03_axi_arvalid(m_arvalid[3]),
    .m03_axi_arready(m_arready[3]),
    .m03_axi_rid(m_rid[3]), .m03_axi_rdata(m_rdata[3]), .m03_axi_rresp(m_rresp[3]),
    .m03_axi_rlast(m_rlast[3]), .m03_axi_ruser(1'b0), .m03_axi_rvalid(m_rvalid[3]),
    .m03_axi_rready(m_rready[3]),

    .m04_axi_awid(m_awid[4]), .m04_axi_awaddr(m_awaddr[4]), .m04_axi_awlen(m_awlen[4]),
    .m04_axi_awsize(m_awsize[4]), .m04_axi_awburst(m_awburst[4]), .m04_axi_awlock(m_awlock[4]),
    .m04_axi_awcache(m_awcache[4]), .m04_axi_awprot(m_awprot[4]), .m04_axi_awqos(m_awqos[4]),
    .m04_axi_awregion(m_awregion[4]), .m04_axi_awuser(), .m04_axi_awvalid(m_awvalid[4]),
    .m04_axi_awready(m_awready[4]),
    .m04_axi_wdata(m_wdata[4]), .m04_axi_wstrb(m_wstrb[4]), .m04_axi_wlast(m_wlast[4]),
    .m04_axi_wuser(), .m04_axi_wvalid(m_wvalid[4]), .m04_axi_wready(m_wready[4]),
    .m04_axi_bid(m_bid[4]), .m04_axi_bresp(m_bresp[4]), .m04_axi_buser(1'b0),
    .m04_axi_bvalid(m_bvalid[4]), .m04_axi_bready(m_bready[4]),
    .m04_axi_arid(m_arid[4]), .m04_axi_araddr(m_araddr[4]), .m04_axi_arlen(m_arlen[4]),
    .m04_axi_arsize(m_arsize[4]), .m04_axi_arburst(m_arburst[4]), .m04_axi_arlock(m_arlock[4]),
    .m04_axi_arcache(m_arcache[4]), .m04_axi_arprot(m_arprot[4]), .m04_axi_arqos(m_arqos[4]),
    .m04_axi_arregion(m_arregion[4]), .m04_axi_aruser(), .m04_axi_arvalid(m_arvalid[4]),
    .m04_axi_arready(m_arready[4]),
    .m04_axi_rid(m_rid[4]), .m04_axi_rdata(m_rdata[4]), .m04_axi_rresp(m_rresp[4]),
    .m04_axi_rlast(m_rlast[4]), .m04_axi_ruser(1'b0), .m04_axi_rvalid(m_rvalid[4]),
    .m04_axi_rready(m_rready[4]),

    .m05_axi_awid(m_awid[5]), .m05_axi_awaddr(m_awaddr[5]), .m05_axi_awlen(m_awlen[5]),
    .m05_axi_awsize(m_awsize[5]), .m05_axi_awburst(m_awburst[5]), .m05_axi_awlock(m_awlock[5]),
    .m05_axi_awcache(m_awcache[5]), .m05_axi_awprot(m_awprot[5]), .m05_axi_awqos(m_awqos[5]),
    .m05_axi_awregion(m_awregion[5]), .m05_axi_awuser(), .m05_axi_awvalid(m_awvalid[5]),
    .m05_axi_awready(m_awready[5]),
    .m05_axi_wdata(m_wdata[5]), .m05_axi_wstrb(m_wstrb[5]), .m05_axi_wlast(m_wlast[5]),
    .m05_axi_wuser(), .m05_axi_wvalid(m_wvalid[5]), .m05_axi_wready(m_wready[5]),
    .m05_axi_bid(m_bid[5]), .m05_axi_bresp(m_bresp[5]), .m05_axi_buser(1'b0),
    .m05_axi_bvalid(m_bvalid[5]), .m05_axi_bready(m_bready[5]),
    .m05_axi_arid(m_arid[5]), .m05_axi_araddr(m_araddr[5]), .m05_axi_arlen(m_arlen[5]),
    .m05_axi_arsize(m_arsize[5]), .m05_axi_arburst(m_arburst[5]), .m05_axi_arlock(m_arlock[5]),
    .m05_axi_arcache(m_arcache[5]), .m05_axi_arprot(m_arprot[5]), .m05_axi_arqos(m_arqos[5]),
    .m05_axi_arregion(m_arregion[5]), .m05_axi_aruser(), .m05_axi_arvalid(m_arvalid[5]),
    .m05_axi_arready(m_arready[5]),
    .m05_axi_rid(m_rid[5]), .m05_axi_rdata(m_rdata[5]), .m05_axi_rresp(m_rresp[5]),
    .m05_axi_rlast(m_rlast[5]), .m05_axi_ruser(1'b0), .m05_axi_rvalid(m_rvalid[5]),
    .m05_axi_rready(m_rready[5]),

    .m06_axi_awid(m_awid[6]), .m06_axi_awaddr(m_awaddr[6]), .m06_axi_awlen(m_awlen[6]),
    .m06_axi_awsize(m_awsize[6]), .m06_axi_awburst(m_awburst[6]), .m06_axi_awlock(m_awlock[6]),
    .m06_axi_awcache(m_awcache[6]), .m06_axi_awprot(m_awprot[6]), .m06_axi_awqos(m_awqos[6]),
    .m06_axi_awregion(m_awregion[6]), .m06_axi_awuser(), .m06_axi_awvalid(m_awvalid[6]),
    .m06_axi_awready(m_awready[6]),
    .m06_axi_wdata(m_wdata[6]), .m06_axi_wstrb(m_wstrb[6]), .m06_axi_wlast(m_wlast[6]),
    .m06_axi_wuser(), .m06_axi_wvalid(m_wvalid[6]), .m06_axi_wready(m_wready[6]),
    .m06_axi_bid(m_bid[6]), .m06_axi_bresp(m_bresp[6]), .m06_axi_buser(1'b0),
    .m06_axi_bvalid(m_bvalid[6]), .m06_axi_bready(m_bready[6]),
    .m06_axi_arid(m_arid[6]), .m06_axi_araddr(m_araddr[6]), .m06_axi_arlen(m_arlen[6]),
    .m06_axi_arsize(m_arsize[6]), .m06_axi_arburst(m_arburst[6]), .m06_axi_arlock(m_arlock[6]),
    .m06_axi_arcache(m_arcache[6]), .m06_axi_arprot(m_arprot[6]), .m06_axi_arqos(m_arqos[6]),
    .m06_axi_arregion(m_arregion[6]), .m06_axi_aruser(), .m06_axi_arvalid(m_arvalid[6]),
    .m06_axi_arready(m_arready[6]),
    .m06_axi_rid(m_rid[6]), .m06_axi_rdata(m_rdata[6]), .m06_axi_rresp(m_rresp[6]),
    .m06_axi_rlast(m_rlast[6]), .m06_axi_ruser(1'b0), .m06_axi_rvalid(m_rvalid[6]),
    .m06_axi_rready(m_rready[6]),

    .m07_axi_awid(m_awid[7]), .m07_axi_awaddr(m_awaddr[7]), .m07_axi_awlen(m_awlen[7]),
    .m07_axi_awsize(m_awsize[7]), .m07_axi_awburst(m_awburst[7]), .m07_axi_awlock(m_awlock[7]),
    .m07_axi_awcache(m_awcache[7]), .m07_axi_awprot(m_awprot[7]), .m07_axi_awqos(m_awqos[7]),
    .m07_axi_awregion(m_awregion[7]), .m07_axi_awuser(), .m07_axi_awvalid(m_awvalid[7]),
    .m07_axi_awready(m_awready[7]),
    .m07_axi_wdata(m_wdata[7]), .m07_axi_wstrb(m_wstrb[7]), .m07_axi_wlast(m_wlast[7]),
    .m07_axi_wuser(), .m07_axi_wvalid(m_wvalid[7]), .m07_axi_wready(m_wready[7]),
    .m07_axi_bid(m_bid[7]), .m07_axi_bresp(m_bresp[7]), .m07_axi_buser(1'b0),
    .m07_axi_bvalid(m_bvalid[7]), .m07_axi_bready(m_bready[7]),
    .m07_axi_arid(m_arid[7]), .m07_axi_araddr(m_araddr[7]), .m07_axi_arlen(m_arlen[7]),
    .m07_axi_arsize(m_arsize[7]), .m07_axi_arburst(m_arburst[7]), .m07_axi_arlock(m_arlock[7]),
    .m07_axi_arcache(m_arcache[7]), .m07_axi_arprot(m_arprot[7]), .m07_axi_arqos(m_arqos[7]),
    .m07_axi_arregion(m_arregion[7]), .m07_axi_aruser(), .m07_axi_arvalid(m_arvalid[7]),
    .m07_axi_arready(m_arready[7]),
    .m07_axi_rid(m_rid[7]), .m07_axi_rdata(m_rdata[7]), .m07_axi_rresp(m_rresp[7]),
    .m07_axi_rlast(m_rlast[7]), .m07_axi_ruser(1'b0), .m07_axi_rvalid(m_rvalid[7]),
    .m07_axi_rready(m_rready[7])
);

// -------------------------------------------------------------------------
// Test control
// -------------------------------------------------------------------------
integer test_num;
integer pass_count;
integer fail_count;
integer timeout;

reg [ADDR_WIDTH-1:0] test_addr;
reg [DATA_WIDTH-1:0] test_wdata;
reg [DATA_WIDTH-1:0] test_rdata;

// Base addresses array for easy iteration
reg [ADDR_WIDTH-1:0] base_addr [0:7];

// -------------------------------------------------------------------------
// Task: AXI single-beat write
// -------------------------------------------------------------------------
task axi_write;
    input [ADDR_WIDTH-1:0] addr;
    input [DATA_WIDTH-1:0] data;
    input [ID_WIDTH-1:0]   id;
    integer to;
    begin
        // Drive AW channel
        @(posedge clk);
        s_awid    <= id;
        s_awaddr  <= addr;
        s_awlen   <= 8'h00;      // 1 beat
        s_awsize  <= 3'b010;     // 4 bytes
        s_awburst <= 2'b01;      // INCR
        s_awlock  <= 1'b0;
        s_awcache <= 4'b0000;
        s_awprot  <= 3'b000;
        s_awqos   <= 4'b0000;
        s_awvalid <= 1'b1;

        // Drive W channel simultaneously
        s_wdata  <= data;
        s_wstrb  <= {STRB_WIDTH{1'b1}};
        s_wlast  <= 1'b1;
        s_wvalid <= 1'b1;

        // Wait for AW handshake
        to = 0;
        while (!s_awready) begin
            @(posedge clk);
            to = to + 1;
            if (to > 200) begin
                $display("[TIMEOUT] AW channel stuck at addr=0x%08X", addr);
                disable axi_write;
            end
        end
        @(posedge clk);
        s_awvalid <= 1'b0;

        // Wait for W handshake
        to = 0;
        while (!s_wready) begin
            @(posedge clk);
            to = to + 1;
            if (to > 200) begin
                $display("[TIMEOUT] W channel stuck at addr=0x%08X", addr);
                disable axi_write;
            end
        end
        @(posedge clk);
        s_wvalid <= 1'b0;
        s_wlast  <= 1'b0;

        // Accept write response
        s_bready <= 1'b1;
        to = 0;
        while (!s_bvalid) begin
            @(posedge clk);
            to = to + 1;
            if (to > 200) begin
                $display("[TIMEOUT] B channel stuck at addr=0x%08X", addr);
                disable axi_write;
            end
        end
        @(posedge clk);
        s_bready <= 1'b0;
    end
endtask

// -------------------------------------------------------------------------
// Task: AXI single-beat read
// -------------------------------------------------------------------------
task axi_read;
    input  [ADDR_WIDTH-1:0] addr;
    input  [ID_WIDTH-1:0]   id;
    output [DATA_WIDTH-1:0] data;
    integer to;
    begin
        @(posedge clk);
        s_arid    <= id;
        s_araddr  <= addr;
        s_arlen   <= 8'h00;
        s_arsize  <= 3'b010;
        s_arburst <= 2'b01;
        s_arlock  <= 1'b0;
        s_arcache <= 4'b0000;
        s_arprot  <= 3'b000;
        s_arqos   <= 4'b0000;
        s_arvalid <= 1'b1;

        // Wait for AR handshake
        to = 0;
        while (!s_arready) begin
            @(posedge clk);
            to = to + 1;
            if (to > 200) begin
                $display("[TIMEOUT] AR channel stuck at addr=0x%08X", addr);
                data = 32'hDEAD_BEEF;
                disable axi_read;
            end
        end
        @(posedge clk);
        s_arvalid <= 1'b0;

        // Wait for read data
        s_rready <= 1'b1;
        to = 0;
        while (!s_rvalid) begin
            @(posedge clk);
            to = to + 1;
            if (to > 200) begin
                $display("[TIMEOUT] R channel stuck at addr=0x%08X", addr);
                data = 32'hDEAD_BEEF;
                disable axi_read;
            end
        end
        data = s_rdata;
        @(posedge clk);
        s_rready <= 1'b0;
    end
endtask

// -------------------------------------------------------------------------
// Task: single write-then-read-back check
// -------------------------------------------------------------------------
task check_rw;
    input [ADDR_WIDTH-1:0] addr;
    input [DATA_WIDTH-1:0] wdata;
    input [ID_WIDTH-1:0]   id;
    input [31:0]           slave_idx;
    reg   [DATA_WIDTH-1:0] rdata;
    begin
        $display("  [WR] Slave M%0d addr=0x%08X data=0x%08X", slave_idx, addr, wdata);
        axi_write(addr, wdata, id);

        $display("  [RD] Slave M%0d addr=0x%08X", slave_idx, addr);
        axi_read(addr, id, rdata);

        if (rdata === wdata) begin
            $display("  [PASS] M%0d: read 0x%08X == written 0x%08X", slave_idx, rdata, wdata);
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] M%0d: read 0x%08X != written 0x%08X", slave_idx, rdata, wdata);
            fail_count = fail_count + 1;
        end
    end
endtask

// -------------------------------------------------------------------------
// Stimulus
// -------------------------------------------------------------------------
initial begin
    // Initialise TB-driven signals
    s_awid    = 0; s_awaddr = 0; s_awlen = 0; s_awsize = 0;
    s_awburst = 0; s_awlock = 0; s_awcache = 0; s_awprot = 0;
    s_awqos   = 0; s_awvalid = 0;
    s_wdata   = 0; s_wstrb = 0; s_wlast = 0; s_wvalid = 0;
    s_bready  = 0;
    s_arid    = 0; s_araddr = 0; s_arlen = 0; s_arsize = 0;
    s_arburst = 0; s_arlock = 0; s_arcache = 0; s_arprot = 0;
    s_arqos   = 0; s_arvalid = 0;
    s_rready  = 0;

    pass_count = 0;
    fail_count = 0;

    // Build base address table
    base_addr[0] = M00_BASE;
    base_addr[1] = M01_BASE;
    base_addr[2] = M02_BASE;
    base_addr[3] = M03_BASE;
    base_addr[4] = M04_BASE;
    base_addr[5] = M05_BASE;
    base_addr[6] = M06_BASE;
    base_addr[7] = M07_BASE;

    // ------------------------------------------------------------------
    // Reset
    // ------------------------------------------------------------------
    $display("========================================");
    $display("  AXI Interconnect 1x8 Testbench Start  ");
    $display("========================================");
    rst = 1;
    repeat (8) @(posedge clk);
    rst = 0;
    repeat (4) @(posedge clk);
    $display("[INFO] Reset released");

    // ------------------------------------------------------------------
    // TEST 1: Basic single-beat write + read to each slave port
    //         Write unique data pattern per slave
    // ------------------------------------------------------------------
    $display("\n--- TEST 1: Single-beat W/R to all 8 slaves ---");
    for (i = 0; i < 8; i = i + 1) begin
        test_addr  = base_addr[i] + 32'h0000_0004;    // word offset 1
        test_wdata = 32'hA5A5_0000 | (i << 8) | i;
        check_rw(test_addr, test_wdata, i[ID_WIDTH-1:0], i);
        repeat (2) @(posedge clk);
    end

    // ------------------------------------------------------------------
    // TEST 2: Write multiple words to M00 and verify each
    // ------------------------------------------------------------------
    $display("\n--- TEST 2: Multi-word W/R to M00 ---");
    for (i = 0; i < 4; i = i + 1) begin
        test_addr  = base_addr[0] + (i * 4);
        test_wdata = 32'hC0DE_0000 | i;
        check_rw(test_addr, test_wdata, 8'h10 + i, 0);
        repeat (2) @(posedge clk);
    end

    // ------------------------------------------------------------------
    // TEST 3: Write multiple words to M07 and verify each
    // ------------------------------------------------------------------
    $display("\n--- TEST 3: Multi-word W/R to M07 ---");
    for (i = 0; i < 4; i = i + 1) begin
        test_addr  = base_addr[7] + (i * 4);
        test_wdata = 32'hBEEF_0000 | i;
        check_rw(test_addr, test_wdata, 8'h20 + i, 7);
        repeat (2) @(posedge clk);
    end

    // ------------------------------------------------------------------
    // TEST 4: Back-to-back writes to different slaves, then read back
    //         Tests routing: write to M02, M04, M06 then read all three
    // ------------------------------------------------------------------
    $display("\n--- TEST 4: Back-to-back writes to M02/M04/M06, read-back ---");
    axi_write(base_addr[2] + 32'h8, 32'hDEAD_CA02, 8'h30);
    axi_write(base_addr[4] + 32'h8, 32'hDEAD_CA04, 8'h31);
    axi_write(base_addr[6] + 32'h8, 32'hDEAD_CA06, 8'h32);
    repeat (2) @(posedge clk);

    axi_read(base_addr[2] + 32'h8, 8'h30, test_rdata);
    if (test_rdata === 32'hDEAD_CA02) begin
        $display("  [PASS] M02 read-back OK: 0x%08X", test_rdata);
        pass_count = pass_count + 1;
    end else begin
        $display("  [FAIL] M02 expected 0xDEADCA02, got 0x%08X", test_rdata);
        fail_count = fail_count + 1;
    end

    axi_read(base_addr[4] + 32'h8, 8'h31, test_rdata);
    if (test_rdata === 32'hDEAD_CA04) begin
        $display("  [PASS] M04 read-back OK: 0x%08X", test_rdata);
        pass_count = pass_count + 1;
    end else begin
        $display("  [FAIL] M04 expected 0xDEADCA04, got 0x%08X", test_rdata);
        fail_count = fail_count + 1;
    end

    axi_read(base_addr[6] + 32'h8, 8'h32, test_rdata);
    if (test_rdata === 32'hDEAD_CA06) begin
        $display("  [PASS] M06 read-back OK: 0x%08X", test_rdata);
        pass_count = pass_count + 1;
    end else begin
        $display("  [FAIL] M06 expected 0xDEADCA06, got 0x%08X", test_rdata);
        fail_count = fail_count + 1;
    end

    // ------------------------------------------------------------------
    // TEST 5: Overwrite and re-read (data integrity)
    // ------------------------------------------------------------------
    $display("\n--- TEST 5: Overwrite data integrity on M03 ---");
    axi_write(base_addr[3] + 32'hC, 32'hAAAA_AAAA, 8'h40);
    axi_write(base_addr[3] + 32'hC, 32'h5555_5555, 8'h41);
    axi_read (base_addr[3] + 32'hC, 8'h41, test_rdata);
    if (test_rdata === 32'h5555_5555) begin
        $display("  [PASS] M03 overwrite OK: 0x%08X", test_rdata);
        pass_count = pass_count + 1;
    end else begin
        $display("  [FAIL] M03 overwrite: expected 0x55555555, got 0x%08X", test_rdata);
        fail_count = fail_count + 1;
    end

    // ------------------------------------------------------------------
    // Summary
    // ------------------------------------------------------------------
    repeat (5) @(posedge clk);
    $display("\n========================================");
    $display("  TEST SUMMARY");
    $display("  PASSED : %0d", pass_count);
    $display("  FAILED : %0d", fail_count);
    if (fail_count == 0)
        $display("  RESULT : *** ALL TESTS PASSED ***");
    else
        $display("  RESULT : *** %0d TEST(S) FAILED ***", fail_count);
    $display("========================================\n");

    $finish;
end

// -------------------------------------------------------------------------
// Global watchdog (prevents infinite hang)
// -------------------------------------------------------------------------
initial begin
    #500000;
    $display("[WATCHDOG] Simulation exceeded 500 us — forcing stop.");
    $finish;
end

// -------------------------------------------------------------------------
// Optional VCD dump
// -------------------------------------------------------------------------
initial begin
    $dumpfile("tb_axi_interconnect_wrap_1x8.vcd");
    $dumpvars(0, tb_axi_interconnect_wrap_1x8);
end


endmodule

`default_nettype wire
