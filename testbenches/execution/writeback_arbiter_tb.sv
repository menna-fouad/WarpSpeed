module writeback_arbiter_tb;
    reg clk, reset;
    reg [31:0] alu_data, fma_data, lsu_data, special_reg_data;
    reg [9:0]  alu_addr, fma_addr, lsu_addr, special_reg_addr;
    reg alu_valid, fma_valid, lsu_valid, special_reg_valid;

    wire [9:0]  rf_write_addr;
    wire [31:0] rf_write_data;
    wire rf_write_enable;
    wire buffer_full;

    integer errors = 0;
    integer tests = 0;

    writeback_arbiter DUT (
        .clk(clk),
        .reset(reset),

        .alu_data(alu_data),
        .fma_data(fma_data),
        .lsu_data(lsu_data),
        .special_reg_data(special_reg_data),

        .alu_addr(alu_addr),
        .fma_addr(fma_addr),
        .lsu_addr(lsu_addr),
        .special_reg_addr(special_reg_addr),
        
        .alu_valid(alu_valid),
        .fma_valid(fma_valid),
        .lsu_valid(lsu_valid),
        .special_reg_valid(special_reg_valid),

        .is_branch(1'b0),
        .is_load(1'b0),
        .is_store(1'b0),
        .stall(1'b0),

        .rf_write_addr(rf_write_addr),
        .rf_write_data(rf_write_data),
        .rf_write_enable(rf_write_enable),
        
        .buffer_full(buffer_full)
    );

    always #5 clk = ~clk;

    task automatic drive(
        input a_v, input [9:0] a_a, input [31:0] a_d,
        input f_v, input [9:0] f_a, input [31:0] f_d,
        input l_v, input [9:0] l_a, input [31:0] l_d,
        input s_v, input [9:0] s_a, input [31:0] s_d
    );
        begin
            alu_valid = a_v; alu_addr = a_a; alu_data = a_d;
            fma_valid = f_v; fma_addr = f_a; fma_data = f_d;
            lsu_valid = l_v; lsu_addr = l_a; lsu_data = l_d;
            special_reg_valid = s_v; special_reg_addr = s_a; special_reg_data = s_d;
            #1; // let result settle
        end
    endtask

    task automatic check(
        input exp_en, input [9:0] exp_addr, input [31:0] exp_data
    );
        begin
            tests = tests + 1;
            if (rf_write_enable !== exp_en ||
                (exp_en && (rf_write_addr !== exp_addr || rf_write_data !== exp_data))) begin
                errors = errors + 1;
                $display("FAIL | expected: en = %b addr = %h data = %h) | got: en = %b addr = %h data = %h\n",
                        exp_en, exp_addr, exp_data,
                        rf_write_enable, rf_write_addr, rf_write_data);
            end else begin
                $display("PASS | en = %b addr = %h data = %h\n",
                        rf_write_enable, rf_write_addr, rf_write_data);
            end
        end
    endtask

    // rf_write_* are registered, so results show up one edge after the inputs are driven
    task automatic tick;
        begin
            @(posedge clk);
            #1; // settle after clock edge
        end
    endtask

    initial begin
        clk = 0;
        reset = 1;

        drive(0,0,0, 0,0,0, 0,0,0, 0,0,0);

        tick();
        reset = 0;


        // 1. Nothing valid
        $display("\nTEST 1: Nothing valid");
        drive(0,0,0, 0,0,0, 0,0,0, 0,0,0);
        tick();
        check(0,0,0);

        // 2. Only ALU
        $display("\nTEST 2: Only ALU");
        drive(1,10'h001,32'hAAAA,
              0,0,0,
              0,0,0,
              0,0,0);
        tick();
        check(1,10'h001,32'hAAAA);

        // 3. Only FMA
        $display("\nTEST 3: Only FMA");
        drive(0,0,0,
              1,10'h002,32'hBBBB,
              0,0,0,
              0,0,0);
        tick();
        check(1,10'h002,32'hBBBB);

        // 4. Only LSU
        $display("\nTEST 4: Only LSU");
        drive(0,0,0,
              0,0,0,
              1,10'h003,32'hCCCC,
              0,0,0);
        tick();
        check(1,10'h003,32'hCCCC);

        // Arbitration is round-robin (source index: 0 = LSU, 1 = FMA, 2 = ALU, 3 = special reg),
        // rr_ptr points one past the last winner. After TESTS 2-4 (ALU, FMA, LSU) rr_ptr = 1.

        // 5. LSU + ALU
        // rr_ptr = 1 -> ALU wins, LSU goes into FIFO
        $display("\nTEST 5: LSU + ALU");
        drive(1,10'h010,32'hAAAA0010,
              0,0,0,
              1,10'h020,32'hBBBB0020,
              0,0,0);
        tick();
        check(1,10'h010,32'hAAAA0010);


        // LSU should now come from FIFO
        $display("TEST 5.2: Buffered LSU");
        drive(0,0,0, 0,0,0, 0,0,0, 0,0,0);
        tick();
        check(1,10'h020,32'hBBBB0020);


        // 6. All three valid
        // rr_ptr = 1 -> FMA wins, ALU then LSU are buffered
        $display("\nTEST 6: LSU + FMA + ALU");
        drive(1,10'h030,32'hAAAA0030,
              1,10'h040,32'hBBBB0040,
              1,10'h050,32'hCCCC0050,
              0,0,0);

        // FMA should be first
        tick();
        check(1,10'h040,32'hBBBB0040);

        drive(0,0,0, 0,0,0, 0,0,0, 0,0,0);

        // ALU should be next
        tick();
        check(1,10'h030,32'hAAAA0030);

        // LSU should be next
        tick();
        check(1,10'h050,32'hCCCC0050);

        // 7. Buffered data has priority over new LSU
        // rr_ptr = 1 -> FMA wins, ALU + LSU go into FIFO
        $display("\nTEST 7: Buffered data vs new LSU");

        drive(1,10'h060,32'hAAAA0060,
              1,10'h070,32'hBBBB0070,
              1,10'h080,32'hCCCC0080,
              0,0,0);

        tick();
        check(1,10'h070,32'hBBBB0070);

        // FIFO contains ALU + LSU.
        // New LSU arrives and queues behind the buffered LSU, ALU wins this cycle.
        drive(0,0,0,
              0,0,0,
              1,10'h090,32'hDDDD0090,
              0,0,0);

        tick();
        check(1,10'h060,32'hAAAA0060);

        // Buffered LSU drains before the new one
        drive(0,0,0, 0,0,0, 0,0,0, 0,0,0);
        tick();
        check(1,10'h080,32'hCCCC0080);

        // New LSU should be last
        tick();
        check(1,10'h090,32'hDDDD0090);

        // Nothing left
        tick();
        check(0,0,0);


        // Done
        if (errors == 0)
            $display("\nALL TESTS PASSED");
        else
            $fatal(1, "\nERRORS: %0d", errors);

        $finish;
    end
endmodule
