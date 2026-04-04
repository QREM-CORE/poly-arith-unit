// ============================================================================
// Purpose:
//   This module implements a single dual-port RAM bank used for storing
//   polynomial coefficients.
//
// Key Features:
//   - Depth: N entries
//   - Width: W bits per entry
//   - Two independent ports (Port A and Port B)
//   - Each port can read or write in the same clock cycle
//   - Reads are synchronous (data returned on next clock)
//   - Optional deterministic pre-load for NTT bring-up / debug
//
// Notes on pre-load support:
//   The 4-bank wrapper stores coefficients in BANKED order, not logical order.
//   To make readback through the wrapper look like a normal polynomial, this
//   bank can pre-fill its memory by reconstructing the wrapper's bank mapping.
// ============================================================================

module poly_ram_bank #(
  parameter int N            = 256,          // Number of memory locations in THIS bank
  parameter int W            = 16,           // Data width (bits per coefficient)
  parameter int ADDR_W       = $clog2(N),    // Address width needed for N entries

  // Optional deterministic pre-load support
  parameter bit PRELOAD_EN   = 1'b1,
  parameter int BANK_ID      = 0,
  parameter int GLOBAL_N     = 256,
  parameter int NUM_BANKS    = 4,
  parameter int NUM_POLYS    = 4,
  parameter int PRELOAD_MODE = 0             // 0=ramp, 1=impulse, 2=all-ones, 3=checkerboard
)(
  input  logic              clk,
  input  logic              rst_n,

  // --------------------------------------------------------------------------
  // PORT A (first memory port)
  // --------------------------------------------------------------------------
  input  logic              a_we,
  input  logic [ADDR_W-1:0] a_addr,
  input  logic [W-1:0]      a_wdata,
  output logic [W-1:0]      a_rdata,

  // --------------------------------------------------------------------------
  // PORT B (second independent memory port)
  // --------------------------------------------------------------------------
  input  logic              b_we,
  input  logic [ADDR_W-1:0] b_addr,
  input  logic [W-1:0]      b_wdata,
  output logic [W-1:0]      b_rdata
);

  localparam int SLICE_N = GLOBAL_N / NUM_BANKS;

  logic [W-1:0] mem [0:N-1];

  function automatic int bank_idx_from_order(input int order);
    int sum;
    begin
      sum = ((order >> 0) & 2'b11)
          + ((order >> 2) & 2'b11)
          + ((order >> 4) & 2'b11)
          + ((order >> 6) & 2'b11);
      bank_idx_from_order = sum & (NUM_BANKS - 1);
    end
  endfunction

  function automatic logic [W-1:0] preload_coeff(
    input int poly_id,
    input int order
  );
    int val;
    begin
      unique case (PRELOAD_MODE)
        1: val = (order == 0) ? 1 : 0;                                 // impulse
        2: val = 1;                                                    // all ones
        3: val = ((order & 1) == 0) ? 16'd1 : 16'd3328;                // +/-1 mod q
        default: val = (poly_id * GLOBAL_N + order) & ((1 << W) - 1);  // ramp
      endcase
      preload_coeff = val[W-1:0];
    end
  endfunction

  integer pid, row, lane;
  integer order;
  initial begin
    for (row = 0; row < N; row++) begin
      mem[row] = '0;
    end

    if (PRELOAD_EN) begin
      if ((NUM_BANKS != 4) || (GLOBAL_N % NUM_BANKS != 0)) begin
        $error("poly_ram_bank preload currently assumes 4 banks and GLOBAL_N divisible by NUM_BANKS");
      end else begin
        for (pid = 0; pid < NUM_POLYS; pid++) begin
          for (row = 0; row < SLICE_N; row++) begin
            for (lane = 0; lane < NUM_BANKS; lane++) begin
              order = row * NUM_BANKS + lane;
              if (bank_idx_from_order(order) == BANK_ID) begin
                mem[pid * SLICE_N + row] = preload_coeff(pid, order);
              end
            end
          end
        end
      end
    end
  end

  always_ff @(posedge clk) begin
    if (a_we)
      mem[a_addr] <= a_wdata;

    a_rdata <= mem[a_addr];
  end

  always_ff @(posedge clk) begin
    if (b_we)
      mem[b_addr] <= b_wdata;

    b_rdata <= mem[b_addr];
  end

endmodule
