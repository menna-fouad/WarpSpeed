module memory_coalescing_unit_tb;
    logic clk, reset;
    always #5 clk = ~clk;

    logic [7:0][31:0] addr, src2;
    logic [7:0] valid;
    logic is_load, is_store, sub_warp_valid;
    logic [1:0] sub_warp_cycle;
    logic [9:0] reg_bank_addr;

    logic req_valid, req_ready, req_write;
    logic [31:0] req_addr, wr_word_en;
    logic [1023:0] wr_data;

    logic txn_valid, txn_is_load, txn_last;
    logic [2:0] txn_warp;
    logic [4:0] txn_dst;
    logic [31:0] txn_match_mask;
    logic [31:0][4:0] txn_word_sel;
    logic lsu_ready;

    int error_count = 0;

    // Cycles the issue FSM takes before it can present a request
    localparam int LOAD_SETUP = 3;      // last sub-warp -> COPY -> SELECT_LINE -> PREPARE_GROUP -> ISSUE
    localparam int STORE_SETUP = 7;     // +4 BUILD cycles before ISSUE
    localparam int NEXT_LOAD_SETUP = 2; // accepted line -> SELECT_LINE -> PREPARE_GROUP -> ISSUE
    localparam int NEXT_STORE_SETUP = 6;

    memory_coalescing_unit coalescing (.*);

    task automatic tick;
        @(posedge clk); #1;
    endtask

    task automatic send_subwarp(
        input logic [2:0] warp,
        input logic [4:0] dst,
        input logic [1:0] cycle,
        input logic load,
        input logic store,
        input logic [7:0] lane_valid,
        input logic [7:0][31:0] addresses,
        input logic [7:0][31:0] store_values
    );
        reg_bank_addr = {warp, 2'b00, dst};
        sub_warp_cycle = cycle;
        sub_warp_valid = 1'b1;
        is_load  = load;
        is_store = store;
        valid = lane_valid;
        addr = addresses;
        src2 = store_values;

        tick();

        sub_warp_valid = 1'b0;
        is_load  = 1'b0;
        is_store = 1'b0;
        valid = '0;
        addr  = '0;
        src2  = '0;
    endtask

    task automatic check(
        input logic condition,
        input string message
    );
        if (!condition) begin
            $error("FAIL | %s", message);
            error_count++;
        end
    endtask

    initial begin
        logic [31:0][31:0] test_addr;
        logic [31:0][31:0] test_data;
        logic [31:0] expected_mask;

        clk = 1'b0; reset = 1'b1;

        // Initial values
        addr = '0;
        src2 = '0;
        valid = '0;

        is_load = 1'b0;
        is_store = 1'b0;
        sub_warp_valid = 1'b0;
        sub_warp_cycle = 2'd0;
        reg_bank_addr = '0;

        // Cache always accepts requests for this first test
        req_ready = 1'b1;
        lsu_ready = 1'b1;

        // Reset
        repeat (2) tick();
        reset = 1'b0;
        tick();

        // TEST 1: All 32 threads are valid and load from one cache line
        $display("TEST 1: Single cache-line load");

        // Generate addresses for all 32 threads
        for (int t = 0; t < 32; ++t) begin
            test_addr[t] = 32'h0000_1000 + t*4;
            test_data[t] = '0;
        end

        // warp, destination register, subwarp, load, store, mask
        send_subwarp(3'd0, 5'd5, 2'd0, 1'b1, 1'b0, 8'hFF, test_addr[7:0], test_data[7:0]);
        send_subwarp(3'd0, 5'd5, 2'd1, 1'b1, 1'b0, 8'hFF, test_addr[15:8], test_data[15:8]);
        send_subwarp(3'd0, 5'd5, 2'd2, 1'b1, 1'b0, 8'hFF, test_addr[23:16], test_data[23:16]);
        send_subwarp(3'd0, 5'd5, 2'd3, 1'b1, 1'b0, 8'hFF, test_addr[31:24], test_data[31:24]);
        repeat (LOAD_SETUP) tick();

        check(req_valid === 1'b1, "req_valid should be asserted");
        check(req_addr === 32'h0000_1000, "req_addr should be cache-line base 0x1000");
        check(req_write === 1'b0, "req_write should be 0 for a load");
        check(txn_valid === 1'b1, "txn_valid should be asserted when request is accepted");
        check(txn_is_load === 1'b1, "txn_is_load should be 1");
        check(txn_warp === 3'd0, "txn_warp should be warp 0");
        check(txn_dst === 5'd5, "txn_dst should be register 5");
        check(txn_match_mask === 32'hFFFF_FFFF, "all 32 threads should belong to the transaction");
        check(txn_last === 1'b1, "single transaction should also be the last transaction");

        // Check word selection
        for (int t = 0; t < 32; ++t) begin
            if (txn_word_sel[t] !== t[4:0]) begin
                $error("FAIL: thread %0d word select = %0d, expected %0d", t, txn_word_sel[t], t);
                error_count++;
            end
        end

        tick(); // Accept transaction
        check(req_valid === 1'b0, "req_valid should deassert after final transaction");

        $display("TEST 2: Multiple cache-lines requested");

        // Generate addresses for all 32 threads
        for (int t = 0; t < 8; ++t) begin
            test_addr[t] = 32'h0000_1000 + t*4;
            test_data[t] = '0;
        end

        for (int t = 8; t < 24; ++t) begin
            test_addr[t] = 32'h0000_2000 + t*4;
            test_data[t] = '0;
        end

        for (int t = 24; t < 32; ++t) begin
            test_addr[t] = 32'h0000_3000 + t*4;
            test_data[t] = '0;
        end

        // warp, destination register, subwarp, load, store, mask
        send_subwarp(3'd2, 5'd12, 2'd0, 1'b1, 1'b0, 8'hFF, test_addr[7:0], test_data[7:0]);
        send_subwarp(3'd2, 5'd12, 2'd1, 1'b1, 1'b0, 8'hFF, test_addr[15:8], test_data[15:8]);
        send_subwarp(3'd2, 5'd12, 2'd2, 1'b1, 1'b0, 8'hFF, test_addr[23:16], test_data[23:16]);
        send_subwarp(3'd2, 5'd12, 2'd3, 1'b1, 1'b0, 8'hFF, test_addr[31:24], test_data[31:24]);
        repeat (LOAD_SETUP) tick();
        
        // The request is visible once the issue FSM reaches ISSUE.
        check(req_valid === 1'b1, "req_valid should be asserted");
        check(req_addr === 32'h0000_1000, "req_addr should be cache-line base 0x1000");
        check(req_write === 1'b0, "req_write should be 0 for a load");
        check(txn_valid === 1'b1, "txn_valid should be asserted when request is accepted");
        check(txn_is_load === 1'b1, "txn_is_load should be 1");
        check(txn_warp === 3'd2, "txn_warp should be warp 2");
        check(txn_dst === 5'd12, "txn_dst should be register 12");
        check(txn_match_mask === 32'h0000_00FF, "lower 8 threads should belong to the transaction");
        check(txn_last === 1'b0, "multiple transactions so first transaction should not be the last transaction");

        tick(); // accept 0x1000 transaction
        repeat (NEXT_LOAD_SETUP) tick();
        check(req_addr === 32'h0000_2000, "req_addr should be cache-line base 0x2000");
        check(txn_match_mask === 32'h00FF_FF00, "second set of 8 threads should belong to the transaction");
        check(txn_last === 1'b0, "second transaction should not be the last transaction");

        tick(); // accept 0x2000 transaction
        repeat (NEXT_LOAD_SETUP) tick();
        check(req_addr === 32'h0000_3000, "req_addr should be cache-line base 0x3000");
        check(txn_match_mask === 32'hFF00_0000, "second set of 8 threads should belong to the transaction");
        check(txn_last === 1'b1, "third transactions should be the last transaction");

        tick(); // Accept 0x2000 transaction
        check(req_valid === 1'b0, "req_valid should deassert after final transaction");

        // Check word selection
        for (int t = 0; t < 32; ++t) begin
            if (txn_word_sel[t] !== t[4:0]) begin
                $error("FAIL: thread %0d word select = %0d, expected %0d", t, txn_word_sel[t], t);
                error_count++;
            end
        end

        $display("TEST 3: Multiple cache-lines requested");
        
        for (int t = 0; t < 16; ++t) begin
            test_addr[t] = 32'h0000_1000 + t*4;
            test_data[t] = '0;
        end

        for (int t = 16; t < 32; ++t) begin
            test_addr[t] = 32'h0000_2000 + t*4;
            test_data[t] = '0;
        end

        // warp, destination register, subwarp, load, store, mask
        send_subwarp(3'd3, 5'd27, 2'd0, 1'b1, 1'b0, 8'h12, test_addr[7:0], test_data[7:0]);
        send_subwarp(3'd3, 5'd27, 2'd1, 1'b1, 1'b0, 8'h34, test_addr[15:8], test_data[15:8]);
        send_subwarp(3'd3, 5'd27, 2'd2, 1'b1, 1'b0, 8'h56, test_addr[23:16], test_data[23:16]);
        send_subwarp(3'd3, 5'd27, 2'd3, 1'b1, 1'b0, 8'h78, test_addr[31:24], test_data[31:24]);
        repeat (LOAD_SETUP) tick();

        // The request is visible once the issue FSM reaches ISSUE.
        check(req_valid === 1'b1, "req_valid should be asserted");
        check(req_addr === 32'h0000_1000, "req_addr should be cache-line base 0x1000");
        check(req_write === 1'b0, "req_write should be 0 for a load");
        check(txn_valid === 1'b1, "txn_valid should be asserted when request is accepted");
        check(txn_is_load === 1'b1, "txn_is_load should be 1");
        check(txn_warp === 3'd3, "txn_warp should be warp 3");
        check(txn_dst === 5'd27, "txn_dst should be register 27");
        check(txn_match_mask === 32'h0000_3412, "mask should apply to the transaction");
        check(txn_last === 1'b0, "multiple transactions so first transaction should not be the last transaction");

        tick(); // accept 0x1000 transaction
        repeat (NEXT_LOAD_SETUP) tick();
        check(req_addr === 32'h0000_2000, "req_addr should be cache-line base 0x2000");
        check(txn_match_mask === 32'h7856_0000, "second mask should apply to the transaction");
        check(txn_last === 1'b1, "second transaction should be the last transaction");

        tick(); // Accept 0x2000 transaction
        check(req_valid === 1'b0, "req_valid should deassert after final transaction");

        // Check word selection
        for (int t = 0; t < 32; ++t) begin
            if (txn_word_sel[t] !== t[4:0]) begin
                $error("FAIL: thread %0d word select = %0d, expected %0d", t, txn_word_sel[t], t);
                error_count++;
            end
        end

        $display("TEST 4: Single cache-line store");

        // Each thread stores its thread index + 100 to a different word
        for (int t = 0; t < 32; ++t) begin
            test_addr[t] = 32'h0000_4000 + t*4;
            test_data[t] = 32'd100 + t;
        end

        // Send all four subwarps
        send_subwarp(3'd4, 5'd15, 2'd0, 1'b0, 1'b1, 8'hFF, test_addr[7:0], test_data[7:0]);
        send_subwarp(3'd4, 5'd15, 2'd1, 1'b0, 1'b1, 8'hFF, test_addr[15:8], test_data[15:8]);
        send_subwarp(3'd4, 5'd15, 2'd2, 1'b0, 1'b1, 8'hFF, test_addr[23:16], test_data[23:16]);
        send_subwarp(3'd4, 5'd15, 2'd3, 1'b0, 1'b1, 8'hFF, test_addr[31:24], test_data[31:24]);
        repeat (STORE_SETUP) tick();

        // Check cache request
        check(req_valid === 1'b1, "Store request should be valid");
        check(req_addr === 32'h0000_4000, "Store address should be 0x4000");
        check(req_write === 1'b1, "req_write should be 1 for a store");
        check(wr_word_en === 32'hFFFF_FFFF, "All 32 words should be enabled");

        // Check store data packing
        for (int t = 0; t < 32; ++t) begin
            check(wr_data[t*32 +: 32] === test_data[t], $sformatf("Store data incorrect at word %0d", t));
        end

        // Check transaction metadata
        check(txn_valid === 1'b1, "Store transaction should be valid");
        check(txn_is_load === 1'b0, "txn_is_load should be 0 for store");
        check(txn_warp === 3'd4, "txn_warp should be warp 4");
        check(txn_match_mask === 32'hFFFF_FFFF, "All 32 threads should match");
        check(txn_last === 1'b1, "Store should require only one transaction");

        tick(); // Accept store transaction
        check(req_valid === 1'b0, "Store request should deassert after acceptance");

        $display("TEST 5: Partial-mask store with scattered addresses");

        for (int t = 0; t < 32; ++t) begin
            test_addr[t] = 32'h0000_5000 + (31-t)*4;
            test_data[t] = 32'd200 + t;
        end

        // Send all four subwarps
        send_subwarp(3'd5, 5'd7, 2'd0, 1'b0, 1'b1, 8'hA5, test_addr[7:0], test_data[7:0]);
        send_subwarp(3'd5, 5'd7, 2'd1, 1'b0, 1'b1, 8'hA5, test_addr[15:8], test_data[15:8]);
        send_subwarp(3'd5, 5'd7, 2'd2, 1'b0, 1'b1, 8'hA5, test_addr[23:16], test_data[23:16]);
        send_subwarp(3'd5, 5'd7, 2'd3, 1'b0, 1'b1, 8'hA5, test_addr[31:24], test_data[31:24]);
        repeat (STORE_SETUP) tick();

        check(req_valid === 1'b1, "Partial store request should be valid");
        check(req_addr === 32'h0000_5000, "Partial store address");
        check(req_write === 1'b1, "Partial store should be a write");
        check(wr_word_en === 32'hA5A5_A5A5, "Scattered store word enables");
        check(txn_match_mask === 32'hA5A5_A5A5, "Partial store match mask");
        check(txn_last === 1'b1, "Partial store should be one transaction");

        expected_mask = 32'hA5A5_A5A5;
        for (int t = 0; t < 32; ++t) begin
            if ((expected_mask[t]) & 1)
                check(wr_data[(31-t)*32 +: 32] === test_data[t], $sformatf("Store data incorrect for thread %0d", t));
        end

        tick();
        check(req_valid === 1'b0, "Partial store should finish");

        $display("TEST 6: Cache backpressure");

        req_ready = 1'b0;

        for (int t = 0; t < 32; ++t) begin
            test_addr[t] = 32'h0000_6000 + t*4;
            test_data[t] = '0;
        end

        send_subwarp(3'd6, 5'd9, 2'd0, 1'b1, 1'b0, 8'hFF, test_addr[7:0], test_data[7:0]);
        send_subwarp(3'd6, 5'd9, 2'd1, 1'b1, 1'b0, 8'hFF, test_addr[15:8], test_data[15:8]);
        send_subwarp(3'd6, 5'd9, 2'd2, 1'b1, 1'b0, 8'hFF, test_addr[23:16], test_data[23:16]);
        send_subwarp(3'd6, 5'd9, 2'd3, 1'b1, 1'b0, 8'hFF, test_addr[31:24], test_data[31:24]);
        repeat (LOAD_SETUP) tick();

        repeat (3) begin
            check(req_valid === 1'b1, "Request must remain valid during stall");
            check(req_addr === 32'h0000_6000, "Address must remain stable during stall");
            check(txn_valid === 1'b0, "No transaction accepted during stall");
            check(txn_match_mask === 32'hFFFF_FFFF, "Match mask must remain stable");
            tick();
        end

        req_ready = 1'b1;
        #1; check(txn_valid === 1'b1, "Transaction should become valid when ready");
        tick();
        check(req_valid === 1'b0, "Request should finish after acceptance");

        $display("TEST 7: FIFO waiting warp");

        req_ready = 1'b0;

        // Warp 0: two cache lines
        for (int t = 0; t < 16; ++t) begin
            test_addr[t] = 32'h0000_7000 + (t % 16)*4;
            test_data[t] = '0;
        end

        for (int t = 16; t < 32; ++t) begin
            test_addr[t] = 32'h0000_8000 + (t % 16)*4;
            test_data[t] = '0;
        end

        send_subwarp(3'd0, 5'd10, 2'd0, 1'b1, 1'b0, 8'hFF, test_addr[7:0], test_data[7:0]);
        send_subwarp(3'd0, 5'd10, 2'd1, 1'b1, 1'b0, 8'hFF, test_addr[15:8], test_data[15:8]);
        send_subwarp(3'd0, 5'd10, 2'd2, 1'b1, 1'b0, 8'hFF, test_addr[23:16], test_data[23:16]);
        send_subwarp(3'd0, 5'd10, 2'd3, 1'b1, 1'b0, 8'hFF, test_addr[31:24], test_data[31:24]);
        repeat (LOAD_SETUP) tick();

        check(req_addr === 32'h0000_7000, "Warp 0 first line");

        // Warp 1: one cache line
        for (int t = 0; t < 32; ++t)
            test_addr[t] = 32'h0000_9000 + t*4;

        send_subwarp(3'd1, 5'd11, 2'd0, 1'b1, 1'b0, 8'hFF, test_addr[7:0], test_data[7:0]);
        send_subwarp(3'd1, 5'd11, 2'd1, 1'b1, 1'b0, 8'hFF, test_addr[15:8], test_data[15:8]);
        send_subwarp(3'd1, 5'd11, 2'd2, 1'b1, 1'b0, 8'hFF, test_addr[23:16], test_data[23:16]);
        send_subwarp(3'd1, 5'd11, 2'd3, 1'b1, 1'b0, 8'hFF, test_addr[31:24], test_data[31:24]);

        check(txn_warp === 3'd0, "Warp 0 should still be issuing");
        check(coalescing.ready_count === 4'd1, "Warp 1 should be queued");

        req_ready = 1'b1;
        #1;

        tick(); // Warp 0 first line accepted
        repeat (NEXT_LOAD_SETUP) tick();
        check(req_addr === 32'h0000_8000, "Warp 0 second line");
        check(txn_last === 1'b1, "Warp 0 second line is last");

        tick(); // Warp 0 finishes, FIFO pops warp 1
        check(txn_warp === 3'd1, "Warp 1 should start issuing");
        repeat (LOAD_SETUP) tick();
        check(req_addr === 32'h0000_9000, "Warp 1 line");

        tick(); // Warp 1 finishes
        check(req_valid === 1'b0, "Both warps should finish");

        $display("TEST 8: Simultaneous FIFO push/pop");

        req_ready = 1'b0;

        // Warp 2 starts issuing
        for (int t = 0; t < 32; ++t)
            test_addr[t] = 32'h0000_A000 + t*4;

        send_subwarp(3'd2, 5'd2, 2'd0, 1'b1, 1'b0, 8'hFF, test_addr[7:0], test_data[7:0]);
        send_subwarp(3'd2, 5'd2, 2'd1, 1'b1, 1'b0, 8'hFF, test_addr[15:8], test_data[15:8]);
        send_subwarp(3'd2, 5'd2, 2'd2, 1'b1, 1'b0, 8'hFF, test_addr[23:16], test_data[23:16]);
        send_subwarp(3'd2, 5'd2, 2'd3, 1'b1, 1'b0, 8'hFF, test_addr[31:24], test_data[31:24]);

        // Warp 3 completes and enters FIFO
        send_subwarp(3'd3, 5'd3, 2'd0, 1'b1, 1'b0, 8'hFF, test_addr[7:0], test_data[7:0]);
        send_subwarp(3'd3, 5'd3, 2'd1, 1'b1, 1'b0, 8'hFF, test_addr[15:8], test_data[15:8]);
        send_subwarp(3'd3, 5'd3, 2'd2, 1'b1, 1'b0, 8'hFF, test_addr[23:16], test_data[23:16]);
        send_subwarp(3'd3, 5'd3, 2'd3, 1'b1, 1'b0, 8'hFF, test_addr[31:24], test_data[31:24]);

        check(coalescing.ready_count === 4'd1, "One warp should be queued");

        // Warp 4 sends subwarps 0-2
        send_subwarp(3'd4, 5'd4, 2'd0, 1'b1, 1'b0, 8'hFF, test_addr[7:0], test_data[7:0]);
        send_subwarp(3'd4, 5'd4, 2'd1, 1'b1, 1'b0, 8'hFF, test_addr[15:8], test_data[15:8]);
        send_subwarp(3'd4, 5'd4, 2'd2, 1'b1, 1'b0, 8'hFF, test_addr[23:16], test_data[23:16]);

        // Make warp 2 finish on same edge warp 4 completes
        req_ready = 1'b1;
        send_subwarp(3'd4, 5'd4, 2'd3, 1'b1, 1'b0, 8'hFF, test_addr[31:24], test_data[31:24]);

        check(txn_warp === 3'd3, "Warp 3 should now issue");
        check(coalescing.ready_count === 4'd1, "Warp 4 should replace warp 3 in FIFO");
        check(coalescing.ready_fifo[0] === 3'd4, "Warp 4 should be at FIFO head");

        repeat (LOAD_SETUP) tick();
        check(req_valid === 1'b1, "Warp 3 request should be valid");

        tick(); // Warp 3 finishes, warp 4 takes over
        check(txn_warp === 3'd4, "Warp 4 should issue next");

        repeat (LOAD_SETUP) tick();
        check(req_valid === 1'b1, "Warp 4 request should be valid");

        tick();
        check(req_valid === 1'b0, "All warps should finish");

        $display("TEST 9: Direct handoff");

        req_ready = 1'b0;
        
        for (int t = 0; t < 32; ++t)
            test_addr[t] = 32'h0000_B000 + t*4;

        // Warp 5 begins issuing
        send_subwarp(3'd5, 5'd5, 2'd0, 1'b1, 1'b0, 8'hFF, test_addr[7:0], test_data[7:0]);
        send_subwarp(3'd5, 5'd5, 2'd1, 1'b1, 1'b0, 8'hFF, test_addr[15:8], test_data[15:8]);
        send_subwarp(3'd5, 5'd5, 2'd2, 1'b1, 1'b0, 8'hFF, test_addr[23:16], test_data[23:16]);
        send_subwarp(3'd5, 5'd5, 2'd3, 1'b1, 1'b0, 8'hFF, test_addr[31:24], test_data[31:24]);

        // Warp 6 sends first three subwarps
        send_subwarp(3'd6, 5'd6, 2'd0, 1'b1, 1'b0, 8'hFF, test_addr[7:0], test_data[7:0]);
        send_subwarp(3'd6, 5'd6, 2'd1, 1'b1, 1'b0, 8'hFF, test_addr[15:8], test_data[15:8]);
        send_subwarp(3'd6, 5'd6, 2'd2, 1'b1, 1'b0, 8'hFF, test_addr[23:16], test_data[23:16]);

        check(coalescing.ready_count === 4'd0, "FIFO should be empty");

        // Warp 5 finishes exactly when warp 6 completes
        req_ready = 1'b1;
        send_subwarp(3'd6, 5'd6, 2'd3, 1'b1, 1'b0, 8'hFF, test_addr[31:24], test_data[31:24]);

        check(txn_warp === 3'd6, "Direct handoff should select warp 6");
        check(coalescing.ready_count === 4'd0, "FIFO should remain empty");

        repeat (LOAD_SETUP) tick();
        check(req_valid === 1'b1, "Warp 6 request should be valid");

        tick();
        check(req_valid === 1'b0, "Direct handoff warp should finish");

        $display("TEST 10: LSU backpressure");

        for (int t = 0; t < 32; ++t)
            test_addr[t] = 32'h0000_E000 + t*4;

        lsu_ready = 1'b0;
        send_subwarp(3'd7, 5'd8, 2'd0, 1'b1, 1'b0, 8'hFF, test_addr[7:0], test_data[7:0]);
        send_subwarp(3'd7, 5'd8, 2'd1, 1'b1, 1'b0, 8'hFF, test_addr[15:8], test_data[15:8]);
        send_subwarp(3'd7, 5'd8, 2'd2, 1'b1, 1'b0, 8'hFF, test_addr[23:16], test_data[23:16]);
        send_subwarp(3'd7, 5'd8, 2'd3, 1'b1, 1'b0, 8'hFF, test_addr[31:24], test_data[31:24]);
        repeat (LOAD_SETUP) tick();

        repeat (3) begin
            check(req_valid === 1'b0, "No request while the LSU is not ready");
            check(txn_valid === 1'b0, "No transaction while the LSU is not ready");
            tick();
        end

        lsu_ready = 1'b1;
        #1;
        check(req_valid === 1'b1, "Request should resume once the LSU is ready");
        check(req_addr === 32'h0000_E000, "Address should survive the LSU stall");
        check(txn_valid === 1'b1, "Transaction should be accepted once the LSU is ready");
        tick();
        check(req_valid === 1'b0, "Request should finish after acceptance");

        $display("TEST 11: Store spanning two cache lines");

        for (int t = 0; t < 32; ++t) begin
            test_addr[t] = ((t < 16) ? 32'h0000_C000 : 32'h0000_D000) + t*4;
            test_data[t] = 32'd300 + t;
        end

        send_subwarp(3'd1, 5'd3, 2'd0, 1'b0, 1'b1, 8'hFF, test_addr[7:0], test_data[7:0]);
        send_subwarp(3'd1, 5'd3, 2'd1, 1'b0, 1'b1, 8'hFF, test_addr[15:8], test_data[15:8]);
        send_subwarp(3'd1, 5'd3, 2'd2, 1'b0, 1'b1, 8'hFF, test_addr[23:16], test_data[23:16]);
        send_subwarp(3'd1, 5'd3, 2'd3, 1'b0, 1'b1, 8'hFF, test_addr[31:24], test_data[31:24]);
        repeat (STORE_SETUP) tick();

        check(req_addr === 32'h0000_C000, "First store line");
        check(req_write === 1'b1, "First line should be a write");
        check(wr_word_en === 32'h0000_FFFF, "First line word enables");
        check(txn_last === 1'b0, "First line should not be the last transaction");
        for (int t = 0; t < 16; ++t)
            check(wr_data[t*32 +: 32] === test_data[t], $sformatf("First line data at word %0d", t));

        tick(); // accept the first line
        repeat (NEXT_STORE_SETUP) tick();

        check(req_addr === 32'h0000_D000, "Second store line");
        check(wr_word_en === 32'hFFFF_0000, "Second line must not keep the first line's word enables");
        check(txn_match_mask === 32'hFFFF_0000, "Second line match mask");
        check(txn_last === 1'b1, "Second line should be the last transaction");
        for (int t = 16; t < 32; ++t)
            check(wr_data[t*32 +: 32] === test_data[t], $sformatf("Second line data at word %0d", t));

        tick();
        check(req_valid === 1'b0, "Store should finish after the second line");

        // Final result
        if (error_count == 0)
            $display("ALL COALESCER TESTS PASSED");
        else
            $fatal(1, "COALESCER TEST FAILED: %0d error(s)", error_count);

        $finish;
    end
endmodule
