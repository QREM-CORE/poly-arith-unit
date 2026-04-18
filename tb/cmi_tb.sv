`timescale 1ns/1ps

module cmi_tb;

    localparam int N         = 256;
    localparam int W         = 16;
    localparam int NUM_POLYS = 4;
    localparam int POLY_W    = $clog2(NUM_POLYS);
    localparam int COEFF_W   = $clog2(N);

    logic clk;
    logic rst;

    logic [3:0][7:0]           coeff_idx_i;
    logic [3:0]                coeff_valid_i;
    logic [POLY_W-1:0]         poly_id_i;
    logic                      v_i;
    logic                      rd_en_i;
    logic [3:0]                wb_latency_i;
    logic [3:0]                wr_en_i;
    logic [3:0][W-1:0]         wr_data_i;
    logic [3:0][W-1:0]         coeff_o;
    logic                      ready_o;

    logic [POLY_W-1:0]         mem_poly_id_o;
    logic                      mem_v_o;
    logic                      mem_rd_en_o;
    logic [3:0][COEFF_W-1:0]   mem_rd_idx_o;
    logic [3:0]                mem_rd_lane_valid_o;
    logic [3:0]                mem_wr_en_o;
    logic [3:0][COEFF_W-1:0]   mem_wr_idx_o;
    logic [3:0][W-1:0]         mem_wr_data_o;

    logic                      mem_rd_valid_i;
    logic [POLY_W-1:0]         mem_rd_poly_id_i;
    logic [3:0][COEFF_W-1:0]   mem_rd_idx_i;
    logic [3:0]                mem_rd_lane_valid_i;
    logic [3:0][W-1:0]         mem_rd_data_i;
    logic                      mem_ready_i;

    cmi #(
        .N(N),
        .W(W),
        .NUM_POLYS(NUM_POLYS)
    ) dut (
        .clk(clk),
        .rst(rst),
        .coeff_idx_i(coeff_idx_i),
        .coeff_valid_i(coeff_valid_i),
        .poly_id_i(poly_id_i),
        .v_i(v_i),
        .rd_en_i(rd_en_i),
        .wb_latency_i(wb_latency_i),
        .wr_en_i(wr_en_i),
        .wr_data_i(wr_data_i),
        .coeff_o(coeff_o),
        .ready_o(ready_o),
        .mem_poly_id_o(mem_poly_id_o),
        .mem_v_o(mem_v_o),
        .mem_rd_en_o(mem_rd_en_o),
        .mem_rd_idx_o(mem_rd_idx_o),
        .mem_rd_lane_valid_o(mem_rd_lane_valid_o),
        .mem_wr_en_o(mem_wr_en_o),
        .mem_wr_idx_o(mem_wr_idx_o),
        .mem_wr_data_o(mem_wr_data_o),
        .mem_rd_valid_i(mem_rd_valid_i),
        .mem_rd_poly_id_i(mem_rd_poly_id_i),
        .mem_rd_idx_i(mem_rd_idx_i),
        .mem_rd_lane_valid_i(mem_rd_lane_valid_i),
        .mem_rd_data_i(mem_rd_data_i),
        .mem_ready_i(mem_ready_i)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    task automatic tick;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    task automatic clear_inputs;
        begin
            coeff_idx_i         = '0;
            coeff_valid_i       = '0;
            poly_id_i           = '0;
            v_i                 = 1'b0;
            rd_en_i             = 1'b0;
            wb_latency_i        = 4'd2;
            wr_en_i             = '0;
            wr_data_i           = '0;
            mem_rd_valid_i      = 1'b0;
            mem_rd_poly_id_i    = '0;
            mem_rd_idx_i        = '0;
            mem_rd_lane_valid_i = '0;
            mem_rd_data_i       = '0;
            mem_ready_i         = 1'b1;
        end
    endtask

    initial begin
        rst = 1'b1;
        clear_inputs();
        repeat (2) tick();
        rst = 1'b0;
        tick();

        // ------------------------------------------------------
        // 1) Read request forwarding
        // ------------------------------------------------------
        poly_id_i           = POLY_W'(2);
        v_i                 = 1'b1;
        rd_en_i             = 1'b1;
        coeff_idx_i[0]      = COEFF_W'(8);
        coeff_idx_i[1]      = COEFF_W'(9);
        coeff_idx_i[2]      = COEFF_W'(10);
        coeff_idx_i[3]      = COEFF_W'(11);
        coeff_valid_i       = 4'b1111;
        #1;

        if (mem_poly_id_o !== POLY_W'(2))
            $fatal(1, "CMI did not forward poly_id correctly");
        if (!mem_v_o || !mem_rd_en_o)
            $fatal(1, "CMI did not forward read request correctly");
        if (mem_rd_idx_o[0] !== COEFF_W'(8) || mem_rd_idx_o[3] !== COEFF_W'(11))
            $fatal(1, "CMI read indices mismatch");
        if (mem_rd_lane_valid_o !== 4'b1111)
            $fatal(1, "CMI read lane-valid mismatch");

        tick();
        clear_inputs();

        // ------------------------------------------------------
        // 2) Read response routing
        // ------------------------------------------------------
        mem_rd_valid_i      = 1'b1;
        mem_rd_lane_valid_i = 4'b1111;
        mem_rd_data_i[0]    = 16'h1111;
        mem_rd_data_i[1]    = 16'h2222;
        mem_rd_data_i[2]    = 16'h3333;
        mem_rd_data_i[3]    = 16'h4444;
        #1;
        if (coeff_o[0] !== 16'h1111 || coeff_o[1] !== 16'h2222 ||
            coeff_o[2] !== 16'h3333 || coeff_o[3] !== 16'h4444)
            $fatal(1, "CMI read response routing mismatch");
        tick();
        clear_inputs();

        // ------------------------------------------------------
        // 3) Ready mirrors downstream memory readiness
        // ------------------------------------------------------
        mem_ready_i = 1'b0;
        #1;
        if (ready_o !== 1'b0)
            $fatal(1, "CMI ready_o should mirror mem_ready_i");
        mem_ready_i = 1'b1;
        #1;
        if (ready_o !== 1'b1)
            $fatal(1, "CMI ready_o failed to return high");

        // ------------------------------------------------------
        // 4) Write-only drain cycle is allowed
        // ------------------------------------------------------
        clear_inputs();
        wr_en_i          = 4'b0011;
        wr_data_i[0]     = 16'hAAAA;
        wr_data_i[1]     = 16'hBBBB;
        #1;
        if (!mem_v_o)
            $fatal(1, "CMI must assert mem_v_o for write-only cycles");

        // ------------------------------------------------------
        // 5) Writeback alignment for latency=2
        // ------------------------------------------------------
        clear_inputs();
        coeff_idx_i[0]   = COEFF_W'(20);
        coeff_idx_i[1]   = COEFF_W'(21);
        coeff_idx_i[2]   = COEFF_W'(22);
        coeff_idx_i[3]   = COEFF_W'(23);
        coeff_valid_i    = 4'b1111;
        wb_latency_i     = 4'd2;
        tick();
        clear_inputs();
        tick();

        wr_en_i          = 4'b0011;
        wr_data_i[0]     = 16'hCAFE;
        wr_data_i[1]     = 16'hBEEF;
        wb_latency_i     = 4'd2;
        #1;
        if (mem_wr_en_o !== 4'b0011)
            $fatal(1, "CMI latency-2 write enable mismatch");
        if (mem_wr_idx_o[0] !== COEFF_W'(20) || mem_wr_idx_o[1] !== COEFF_W'(21))
            $fatal(1, "CMI latency-2 write index mismatch");
        if (mem_wr_data_o[0] !== 16'hCAFE || mem_wr_data_o[1] !== 16'hBEEF)
            $fatal(1, "CMI latency-2 write data mismatch");
        tick();
        clear_inputs();

        // ------------------------------------------------------
        // 6) Writeback alignment for latency=4
        // ------------------------------------------------------
        coeff_idx_i[0]   = COEFF_W'(40);
        coeff_idx_i[1]   = COEFF_W'(41);
        coeff_idx_i[2]   = COEFF_W'(42);
        coeff_idx_i[3]   = COEFF_W'(43);
        coeff_valid_i    = 4'b1111;
        wb_latency_i     = 4'd4;
        tick();
        clear_inputs();
        repeat (3) tick();

        wr_en_i          = 4'b1100;
        wr_data_i[2]     = 16'h1234;
        wr_data_i[3]     = 16'h5678;
        wb_latency_i     = 4'd4;
        #1;
        if (mem_wr_en_o !== 4'b1100)
            $fatal(1, "CMI latency-4 write enable mismatch");
        if (mem_wr_idx_o[2] !== COEFF_W'(42) || mem_wr_idx_o[3] !== COEFF_W'(43))
            $fatal(1, "CMI latency-4 write index mismatch");
        if (mem_wr_data_o[2] !== 16'h1234 || mem_wr_data_o[3] !== 16'h5678)
            $fatal(1, "CMI latency-4 write data mismatch");

        $display("TB PASS");
        $finish;
    end

endmodule
