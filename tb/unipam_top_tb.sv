`timescale 1ns/1ps

import poly_arith_pkg::*;

module unipam_top_tb;

    logic       clk;
    logic       rst;
    logic       start_i;
    pe_mode_e   op_type_i;

    // Correctly mapped all ports
    unipam_top DUT (
        .clk        (clk),
        .rst        (rst),
        .start_i    (start_i),
        .op_type_i  (op_type_i)
    );

    // ---------------------------------------------------------------------
    // Clock Generation (10ns period -> 100MHz)
    // ---------------------------------------------------------------------
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // ---------------------------------------------------------------------
    // Stimulus
    // ---------------------------------------------------------------------
    initial begin
        // Initialize signals
        rst       = 1;
        start_i   = 0;
        op_type_i = PE_MODE_NTT;

        // Hold reset for a couple of clocks
        #20;
        rst = 0;
        #10;

        // Pulse start_i for 1 clock cycle
        start_i = 1;
        #10;
        start_i = 0;

        // Pass 1 processes 1 block of 64 butterflies (64 clock cycles).
        // 64 cycles * 10ns = 640ns. We'll wait 700ns to be safe.

        #2800;
        $display("Simulation Complete.");
        $finish;
    end

 // OKAY SO FOR NTT IT WILL GIVE A START SIGNAL AND A PASS INDEX EACH TIME
endmodule