/* ================================================================
 * 4-Bank Polynomial Memory Wrapper (fixed)
 * ------------------------------------------------
 *
 * Module Name: poly_mem_wrapper_4bank
 * Author(s): Mavra Muzmmal, Jessica Buentipo
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Responsibilities:
 *   - Map logical coefficient indices -> {bank, local bank addr}
 *   - Detect illegal same-cycle multi-read / multi-write bank conflicts
 *   - Issue RAM accesses to 4 banked memories
 *   - Return read response ONE cycle later with aligned metadata
 *
 * Notes:
 *   - Port A of each bank is used for reads
 *   - Port B of each bank is used for writes
 *   - Read response latency = 1 cycle from accepted request
 *   - A request is accepted when: v_i && ready_o
 * ================================================================
*/

module poly_mem_wrapper_4bank #(
  parameter int N         = 256,
  parameter int W         = 16,
  parameter int NUM_POLYS = 4
)(
  input  logic clk,
  input  logic rst_n,

  // ---------------------------
  // Request side
  // ---------------------------
  input  logic [$clog2(NUM_POLYS)-1:0] poly_id_i,
  input  logic                         v_i,
  input  logic                         rd_en_i,
  output logic                         ready_o,

  // Read request
  input  logic [3:0][$clog2(N)-1:0]    rd_idx_i,
  input  logic [3:0]                   rd_lane_valid_i,

  // Read response (1 cycle later)
  output logic                         rd_valid_o,
  output logic [$clog2(NUM_POLYS)-1:0] rd_poly_id_o,
  output logic [3:0][$clog2(N)-1:0]    rd_idx_o,
  output logic [3:0]                   rd_lane_valid_o,
  output logic [3:0][W-1:0]            rd_data_o,

  // Write request
  input  logic [3:0]                   wr_en_i,
  input  logic [3:0][$clog2(N)-1:0]    wr_idx_i,
  input  logic [3:0][W-1:0]            wr_data_i
);

  // ================================================================
  // Parameter checks
  // ================================================================
  initial begin
    if (N != 256)
      $display("Warning: cmi_bank_idx helper currently assumes 8-bit indices / N=256.");
    if (N % 4 != 0)
      $fatal(1, "poly_mem_wrapper_4bank: N must be divisible by 4");
  end

  // ================================================================
  // Derived parameters
  // ================================================================
  localparam int NUM_BANKS  = 4;
  localparam int SLICE_N    = N / NUM_BANKS;
  localparam int BANK_DEPTH = SLICE_N * NUM_POLYS;
  localparam int BANK_AW    = $clog2(BANK_DEPTH);

  // ================================================================
  // Request decode
  // ================================================================
  logic [3:0][1:0]         rd_bank;
  logic [3:0][BANK_AW-1:0] rd_baddr;
  logic [3:0][1:0]         wr_bank;
  logic [3:0][BANK_AW-1:0] wr_baddr;

  // Accepted request pulse
  logic req_fire;

  // ================================================================
  // Bank mapping helpers
  // ================================================================
  function automatic [1:0] cmi_bank_idx(
    input logic [$clog2(N)-1:0] order
  );
    logic [3:0] sum;
    begin
      sum = order[1:0] + order[3:2] + order[5:4] + order[7:6];
      cmi_bank_idx = sum[1:0];
    end
  endfunction

  function automatic [BANK_AW-1:0] cmi_bank_addr(
    input logic [$clog2(NUM_POLYS)-1:0] pid,
    input logic [$clog2(N)-1:0]         order
  );
    logic [$clog2(SLICE_N)-1:0] row;
    begin
      row = order >> 2;
      cmi_bank_addr = pid * SLICE_N + row;
    end
  endfunction

  genvar i;
  generate
    for (i = 0; i < 4; i++) begin : G_DECODE
      assign rd_bank[i]  = cmi_bank_idx(rd_idx_i[i]);
      assign rd_baddr[i] = cmi_bank_addr(poly_id_i, rd_idx_i[i]);

      assign wr_bank[i]  = cmi_bank_idx(wr_idx_i[i]);
      assign wr_baddr[i] = cmi_bank_addr(poly_id_i, wr_idx_i[i]);
    end
  endgenerate

  // ================================================================
  // Conflict detection
  //
  // Read conflicts only matter among ACTIVE read lanes.
  // Write conflicts only matter among asserted write lanes.
  //
  // Read-vs-write same bank is allowed because:
  //   - reads use Port A
  //   - writes use Port B
  // ================================================================
  logic rd_conflict, wr_conflict;
  logic any_rd, any_wr;

  always_comb begin
    rd_conflict = 1'b0;
    wr_conflict = 1'b0;
    any_rd      = rd_en_i && (|rd_lane_valid_i);
    any_wr      = |wr_en_i;

    if (v_i && any_rd) begin
      if (rd_lane_valid_i[0] && rd_lane_valid_i[1] && (rd_bank[0] == rd_bank[1])) rd_conflict = 1'b1;
      if (rd_lane_valid_i[0] && rd_lane_valid_i[2] && (rd_bank[0] == rd_bank[2])) rd_conflict = 1'b1;
      if (rd_lane_valid_i[0] && rd_lane_valid_i[3] && (rd_bank[0] == rd_bank[3])) rd_conflict = 1'b1;
      if (rd_lane_valid_i[1] && rd_lane_valid_i[2] && (rd_bank[1] == rd_bank[2])) rd_conflict = 1'b1;
      if (rd_lane_valid_i[1] && rd_lane_valid_i[3] && (rd_bank[1] == rd_bank[3])) rd_conflict = 1'b1;
      if (rd_lane_valid_i[2] && rd_lane_valid_i[3] && (rd_bank[2] == rd_bank[3])) rd_conflict = 1'b1;
    end

    if (v_i && any_wr) begin
      if (wr_en_i[0] && wr_en_i[1] && (wr_bank[0] == wr_bank[1])) wr_conflict = 1'b1;
      if (wr_en_i[0] && wr_en_i[2] && (wr_bank[0] == wr_bank[2])) wr_conflict = 1'b1;
      if (wr_en_i[0] && wr_en_i[3] && (wr_bank[0] == wr_bank[3])) wr_conflict = 1'b1;
      if (wr_en_i[1] && wr_en_i[2] && (wr_bank[1] == wr_bank[2])) wr_conflict = 1'b1;
      if (wr_en_i[1] && wr_en_i[3] && (wr_bank[1] == wr_bank[3])) wr_conflict = 1'b1;
      if (wr_en_i[2] && wr_en_i[3] && (wr_bank[2] == wr_bank[3])) wr_conflict = 1'b1;
    end
  end

  assign ready_o  = ~(rd_conflict | wr_conflict);
  assign req_fire = v_i && ready_o;

  // ================================================================
  // Bank interface signals
  // ================================================================
  logic [NUM_BANKS-1:0]              a_we, b_we;
  logic [NUM_BANKS-1:0][BANK_AW-1:0] a_addr, b_addr;
  logic [NUM_BANKS-1:0][W-1:0]       a_wdata, b_wdata;
  logic [NUM_BANKS-1:0][W-1:0]       a_rdata, b_rdata;

  // ================================================================
  // Request routing into bank ports
  // ================================================================
  integer k;
  always_comb begin
    for (k = 0; k < NUM_BANKS; k++) begin
      a_we[k]    = 1'b0;
      a_addr[k]  = '0;
      a_wdata[k] = '0;

      b_we[k]    = 1'b0;
      b_addr[k]  = '0;
      b_wdata[k] = '0;
    end

    // ---------------------------
    // READS -> Port A
    // ---------------------------
    if (req_fire && rd_en_i) begin
      for (k = 0; k < 4; k++) begin
        if (rd_lane_valid_i[k]) begin
          a_addr[rd_bank[k]] = rd_baddr[k];
        end
      end
    end

    // ---------------------------
    // WRITES -> Port B
    // ---------------------------
    if (req_fire) begin
      for (k = 0; k < 4; k++) begin
        if (wr_en_i[k]) begin
          b_we[wr_bank[k]]    = 1'b1;
          b_addr[wr_bank[k]]  = wr_baddr[k];
          b_wdata[wr_bank[k]] = wr_data_i[k];
        end
      end
    end
  end

  // ================================================================
  // Instantiate bank memories
  // ================================================================
  genvar b;
  generate
    for (b = 0; b < NUM_BANKS; b++) begin : G_BANK
      poly_ram_bank #(
        .N(BANK_DEPTH),
        .W(W),
        .ADDR_W(BANK_AW),
        .PRELOAD_EN(1'b1),
        .BANK_ID(b),
        .GLOBAL_N(N),
        .NUM_BANKS(NUM_BANKS),
        .NUM_POLYS(NUM_POLYS),
        .PRELOAD_MODE(0)
      ) u_bank (
        .clk   (clk),
        .rst_n (rst_n),

        .a_we    (a_we[b]),
        .a_addr  (a_addr[b]),
        .a_wdata (a_wdata[b]),
        .a_rdata (a_rdata[b]),

        .b_we    (b_we[b]),
        .b_addr  (b_addr[b]),
        .b_wdata (b_wdata[b]),
        .b_rdata (b_rdata[b])
      );
    end
  endgenerate

  // ================================================================
  // Read-response bookkeeping pipeline
  //
  // These registers capture WHICH logical request was accepted.
  // One cycle later, a_rdata[] contains the corresponding bank data.
  // ================================================================
  logic                         rd_valid_r;
  logic [$clog2(NUM_POLYS)-1:0] rd_poly_id_r;
  logic [3:0][$clog2(N)-1:0]    rd_idx_r;
  logic [3:0]                   rd_lane_valid_r;
  logic [3:0][1:0]              rd_bank_r;

  always_ff @(posedge clk or posedge rst_n) begin
    if (rst_n) begin
      rd_valid_r      <= 1'b0;
      rd_poly_id_r    <= '0;
      rd_idx_r        <= '0;
      rd_lane_valid_r <= '0;
      rd_bank_r       <= '0;
    end else begin
      rd_valid_r <= req_fire && rd_en_i;

      if (req_fire && rd_en_i) begin
        rd_poly_id_r    <= poly_id_i;
        rd_idx_r        <= rd_idx_i;
        rd_lane_valid_r <= rd_lane_valid_i;
        rd_bank_r       <= rd_bank;
      end
    end
  end

  // ================================================================
  // Read response outputs
  //
  // Combinational reorder from bank outputs using delayed bank tags.
  // This yields an effective 1-cycle response latency.
  // ================================================================
  always_comb begin
    rd_valid_o      = rd_valid_r;
    rd_poly_id_o    = rd_poly_id_r;
    rd_idx_o        = rd_idx_r;
    rd_lane_valid_o = rd_lane_valid_r;

    rd_data_o       = '0;

    if (rd_valid_r) begin
      if (rd_lane_valid_r[0]) rd_data_o[0] = a_rdata[rd_bank_r[0]];
      if (rd_lane_valid_r[1]) rd_data_o[1] = a_rdata[rd_bank_r[1]];
      if (rd_lane_valid_r[2]) rd_data_o[2] = a_rdata[rd_bank_r[2]];
      if (rd_lane_valid_r[3]) rd_data_o[3] = a_rdata[rd_bank_r[3]];
    end
  end

endmodule