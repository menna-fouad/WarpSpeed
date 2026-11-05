module load_store_unit_tb;
    logic clk, reset, wb_buffer_full;
    always #5 clk = ~clk;
    
    logic txn_valid, txn_is_load, txn_last;
    logic [2:0] txn_warp;
    logic [4:0] txn_dst;
    logic [31:0] txn_match_mask;
    logic [31:0][4:0] txn_word_sel;
    
    logic rsp_valid;
    logic [1023:0] rsp_line;

    logic [7:0] lsu_valid;
    logic [9:0] lsu_addr; // shared by all 8 lanes of the group being written back
    logic [7:0][31:0] lsu_data;
    
    logic [7:0] store_done; // one-hot per warp
    logic lsu_ready;

    int error_count = 0;

    // 4 to gather the line into the slot RAMs, 1 for the registered outputs
    localparam int WB_LATENCY = 5;

    load_store_unit lsu (.*);

    task automatic tick();
        @(posedge clk);
        #1;
    endtask

    task automatic send_txn(
        input logic is_load,
        input logic last,
        input logic [2:0] warp,
        input logic [4:0] dst,
        input logic [31:0] mask,
        input logic [31:0][4:0] word_sel
    );
        // The coalescer only issues while lsu_ready is high
        @(negedge clk);
        while (!lsu_ready) @(negedge clk);
        txn_valid = 1'b1;
        txn_is_load = is_load;
        txn_last = last;
        txn_warp = warp;
        txn_dst = dst;
        txn_match_mask = mask;
        txn_word_sel = word_sel;

        tick(); txn_valid = 1'b0;
    endtask

    task automatic send_rsp(
        input logic [1023:0] line
    );
        tick(); // the data cache spends at least one cycle in LOOKUP before it responds
        @(negedge clk);
        rsp_valid = 1'b1;
        rsp_line = line;

        tick(); rsp_valid = 1'b0;
    endtask

    task automatic check_wb_group(
        input logic [2:0] warp,
        input logic [1:0] group,
        input logic [4:0] dst,
        input logic [31:0] expected_mask,
        input logic [31:0][31:0] expected_data
    );
        int t;
        check(lsu_valid === expected_mask[group*8 +: 8],
            $sformatf("Warp %0d group %0d valid mask", warp, group));

        for (int l = 0; l < 8; ++l) begin
            t = group*8 + l;

            if (expected_mask[t]) begin
                check(lsu_addr === {warp, group, dst},
                    $sformatf("Thread %0d writeback address", t));

                check(lsu_data[l] === expected_data[t],
                    $sformatf("Thread %0d writeback data", t));
            end
        end
    endtask

    function automatic logic [1023:0] make_line(
        input logic [31:0] base
    );
        logic [1023:0] line;

        for (int w = 0; w < 32; ++w)
            line[w*32 +: 32] = base + w;

        return line;
    endfunction

    task automatic check(
        input logic condition,
        input string message
    );
        if (condition === 1'b1)
            $display("PASS | %s", message);
        else begin
            $display("FAIL | %s", message);
            error_count++;
        end
    endtask
    
    initial begin
        logic [31:0][4:0] word_sel;
        logic [31:0][31:0] expected_data;
        logic [31:0] expected_mask;
        logic [1023:0] line;

        clk = 1'b0;
        reset = 1'b1;
        wb_buffer_full = 1'b0;

        txn_valid = 1'b0;
        txn_is_load = 1'b0;
        txn_last = 1'b0;
        txn_warp = '0;
        txn_dst = '0;
        txn_match_mask = '0;
        txn_word_sel = '0;

        rsp_valid = 1'b0;
        rsp_line = '0;
        
        tick(); reset = 1'b0;

        $display("\nTEST 1: Single cache-line load");
        expected_mask = 32'h0000_00FF;
        word_sel = '0;
        expected_data = '0;

        // Threads 0-7 load words 0-7 from the cache line
        for (int t = 0; t < 8; ++t) begin
            word_sel[t] = 5'(t);
            expected_data[t] = 32'h1000 + t;
        end

        line = make_line(32'h1000);

        // Send transaction metadata from the coalescer
        // is_load, last, warp, dst, mask, word_sel
        send_txn(1'b1, 1'b1, 3'd2, 5'd10, expected_mask, word_sel);

        check(lsu.wb_active === 1'b0, "Writeback initially idle");
        check(lsu.ready_count === 4'd0, "Ready FIFO initially empty");
        
        send_rsp(line);
        check(lsu.rsp_processing === 1'b1, "Response is being gathered");
        check(lsu.wb_active === 1'b0, "Writeback waits for the gather");

        repeat (3) tick();
        check(lsu.load_complete === 1'b1, "Gather reaches its last group");
        check(lsu.direct_handoff === 1'b1, "Direct handoff asserted");

        tick();
        check(lsu.wb_active === 1'b1, "Writeback activated after the gather");
        check(lsu.wb_warp === 3'd2, "Warp 2 selected directly");
        check(lsu.ready_count === 4'd0, "Ready FIFO remains empty");

        tick(); // group 0 reaches the registered outputs
        for (int g = 0; g < 4; ++g) begin
            check_wb_group(3'd2, g[1:0], 5'd10, expected_mask, expected_data);
            tick();
        end

        // Advance past the final writeback group.
        tick();

        check(lsu_valid === 8'b0, "Writeback stops after group 3");
        check(store_done === 8'b0, "Load does not generate store_done");

        $display("\nTEST 2: Multiple cache-line load");
        expected_mask = 32'h0000_FFFF;
        word_sel = '0;
        expected_data = '0;

        // Threads 0-7 load words 0-7 from the cache line
        for (int t = 0; t < 8; ++t) begin
            word_sel[t] = 5'(t);
            expected_data[t] = 32'h1000 + t;
        end

        // Threads 8-15 load words 0-7 from line B
        for (int t = 8; t < 16; ++t) begin
            word_sel[t] = 5'(t - 8);
            expected_data[t] = 32'h2000 + (t - 8);
        end

        line = make_line(32'h1000);
        send_txn(1'b1, 1'b0, 3'd3, 5'd12, 32'h0000_00FF, word_sel);
        send_rsp(line);
        check(lsu_valid === 8'b0, "No writeback after first cache-line response");

        line = make_line(32'h2000);
        send_txn(1'b1, 1'b1, 3'd3, 5'd12, 32'h0000_FF00, word_sel);
        send_rsp(line);
        repeat (WB_LATENCY) tick();

        for (int g = 0; g < 4; ++g) begin
            check_wb_group(3'd3, g[1:0], 5'd12, expected_mask, expected_data);
            tick();
        end

        check(lsu_valid === 8'b0, "Writeback stops after group 3");
        check(store_done === 8'b0, "Load does not generate store_done");

        $display("\nTEST 3: Store instruction");
        expected_mask = 32'hABAB_ABAB;
        word_sel = '0;
        expected_data = '0;

        for (int t = 0; t < 32; ++t) begin
            word_sel[t] = 5'(t);
            expected_data[t] = 32'h3000 + t;
        end

        line = make_line(32'h3000);
        send_txn(1'b0, 1'b1, 3'd5, 5'd1, expected_mask, word_sel);
        @(negedge clk);
        rsp_valid = 1'b1;
        rsp_line = line;
        
        #1;
        check(store_done === 8'b0010_0000, "Store instruction from warp 5 fires store_done[5] only");
        check(lsu_valid === 8'b0, "Store should not generate register writeback");

        tick();
        rsp_valid = 1'b0;

        check(store_done === 8'b0, "store_done deasserts after response");

        $display("\nTEST 4: Simultaneous FIFO push/pop");        

        expected_mask = 32'h0000_00FF;
        word_sel = '0;

        for (int t = 0; t < 8; ++t)
            word_sel[t] = 5'(t);

        // Warp 0 begins writing back, but is stalled
        wb_buffer_full = 1'b1;
        send_txn(1'b1, 1'b1, 3'd0, 5'd5, expected_mask, word_sel);
        send_rsp(make_line(32'h5000));
        repeat (WB_LATENCY - 1) tick();

        check(lsu.wb_active === 1'b1, "Warp 0 writeback active");
        check(lsu.wb_warp === 3'd0, "Warp 0 selected");

        // Warp 1 completes its load while warp 0 is stalled
        send_txn(1'b1, 1'b1, 3'd1, 5'd6, expected_mask, word_sel);
        send_rsp(make_line(32'h6000));
        repeat (WB_LATENCY - 1) tick();

        check(lsu.ready_count === 4'd1, "One warp waiting in FIFO");
        check(lsu.ready_fifo[0] === 3'd1, "Warp 1 at FIFO head");

        // Warp 2 issues its transaction, and its response arrives on the last stalled edge.
        // Its gather and warp 0's writeback then each take 3 more edges to reach group 3
        send_txn(1'b1, 1'b1, 3'd2, 5'd7, expected_mask, word_sel);
        send_rsp(make_line(32'h7000));

        // Resume warp 0
        wb_buffer_full = 1'b0; #1;

        expected_data = '0;
        for (int t = 0; t < 8; ++t)
            expected_data[t] = 32'h5000 + t;

        for (int g = 0; g < 3; ++g) begin
            tick();
            check_wb_group(3'd0, g[1:0], 5'd5, expected_mask, expected_data);
        end

        // Warp 0's writeback and warp 2's gather both reach group 3 in this cycle
        check(lsu.wb_done === 1'b1, "Warp 0 finishing writeback");
        check(lsu.fifo_push === 1'b1, "Warp 2 pushing into FIFO");
        check(lsu.fifo_pop === 1'b1, "Warp 1 popping from FIFO");

        tick();
        check_wb_group(3'd0, 2'd3, 5'd5, expected_mask, expected_data);

        // Warp 1 should become active immediately
        check(lsu.wb_active === 1'b1, "Writeback remains active");
        check(lsu.wb_warp === 3'd1, "Warp 1 selected next");
        check(lsu.ready_count === 4'd1, "FIFO count unchanged");
        check(lsu.ready_fifo[0] === 3'd2, "Warp 2 now at FIFO head");

        // Verify warp 1 writeback
        expected_data = '0;
        for (int t = 0; t < 8; ++t)
            expected_data[t] = 32'h6000 + t;

        tick(); // warp 1's group 0 reaches the registered outputs
        for (int g = 0; g < 4; ++g) begin
            check_wb_group(3'd1, g[1:0], 5'd6, expected_mask, expected_data);
            tick();
        end

        // Warp 2 should now be active
        check(lsu.wb_active === 1'b1, "Warp 2 writeback active");
        check(lsu.wb_warp === 3'd2, "Warp 2 selected after warp 1");
        check(lsu.ready_count === 4'd0, "FIFO empty after popping warp 2");

        // Verify warp 2 writeback
        expected_data = '0;
        for (int t = 0; t < 8; ++t)
            expected_data[t] = 32'h7000 + t;

        for (int g = 0; g < 4; ++g) begin
            check_wb_group(3'd2, g[1:0], 5'd7, expected_mask, expected_data);
            tick();
        end

        check(lsu_valid === 8'b0, "All writebacks completed");
        check(lsu.ready_count === 4'd0, "Ready FIFO empty");

        $display("\nTEST 5: Writeback stall");

        expected_mask = 32'hFFFF_FFFF;
        word_sel = '0;
        expected_data = '0;

        for (int t = 0; t < 32; ++t) begin
            word_sel[t] = 5'(t);
            expected_data[t] = 32'h8000 + t;
        end

        line = make_line(32'h8000);
        send_txn(1'b1, 1'b1, 3'd4, 5'd10, expected_mask, word_sel);
        send_rsp(line);
        repeat (WB_LATENCY) tick();

        // Group 0
        check_wb_group(3'd4, 2'd0, 5'd10, expected_mask, expected_data);
        tick();

        // Group 1
        check_wb_group(3'd4, 2'd1, 5'd10, expected_mask, expected_data);

        wb_buffer_full = 1'b1;
        #1;

        for (int i = 0; i < 3; ++i) begin
            tick();
            check(lsu.wb_group === 2'd2, "Next writeback group held during stall");
            check_wb_group(3'd4, 2'd1, 5'd10, expected_mask, expected_data);
        end

        wb_buffer_full = 1'b0; // Release stall
        #1;
        check_wb_group(3'd4, 2'd1, 5'd10, expected_mask, expected_data);

        tick();
        check_wb_group(3'd4, 2'd2, 5'd10, expected_mask, expected_data);

        tick();
        check_wb_group(3'd4, 2'd3, 5'd10, expected_mask, expected_data);

        tick();
        check(lsu_valid === 8'b0, "Writeback stops after final group");

        // Release stall and check all three warps
        wb_buffer_full = 1'b0;

        $display("\nTEST 6: Warp reuse");

        // First load instruction: threads 0-7
        expected_mask = 32'h0000_00FF;
        word_sel = '0;
        expected_data = '0;

        for (int t = 0; t < 8; ++t) begin
            word_sel[t] = 5'(t);
            expected_data[t] = 32'hC000 + t;
        end

        send_txn(1'b1, 1'b1, 3'd6, 5'd8, expected_mask, word_sel);
        send_rsp(make_line(32'hC000));
        repeat (WB_LATENCY) tick();

        for (int g = 0; g < 4; ++g) begin
            check_wb_group(3'd6, 2'(g), 5'd8, expected_mask, expected_data);
            tick();
        end

        check(lsu_valid === 8'b0, "First writeback completed");
        check(lsu.slot_mask[6] === 32'b0, "Warp 6 mask cleared");

        // Second load instruction: threads 16-23
        expected_mask = 32'h00FF_0000;
        word_sel = '0;
        expected_data = '0;

        for (int t = 16; t < 24; ++t) begin
            word_sel[t] = 5'(t - 16);
            expected_data[t] = 32'hD000 + (t - 16);
        end

        send_txn(1'b1, 1'b1, 3'd6, 5'd9, expected_mask, word_sel);
        send_rsp(make_line(32'hD000));
        repeat (WB_LATENCY) tick();

        for (int g = 0; g < 4; ++g) begin
            check_wb_group(3'd6, 2'(g), 5'd9, expected_mask, expected_data);
            tick();
        end

        check(lsu_valid === 8'b0, "Second writeback completed");
        check(lsu.slot_mask[6] === 32'b0, "Warp 6 mask cleared again");

        $display("\nTEST 7: Back-to-back transactions");

        // Fastest legal sequence: the second transaction goes out on the first cycle lsu_ready
        // allows, and the cache responds at its minimum latency. That response lands on the
        // first transaction's last gather group, so the first line has to use group 3
        expected_mask = 32'hFFFF_FFFF;
        word_sel = '0;
        expected_data = '0;

        for (int t = 0; t < 8; ++t) begin
            word_sel[t] = 5'(t);
            expected_data[t] = 32'hE000 + t;
        end

        for (int t = 8; t < 24; ++t) begin
            word_sel[t] = 5'(t - 8);
            expected_data[t] = 32'hF000 + (t - 8);
        end

        for (int t = 24; t < 32; ++t) begin
            word_sel[t] = 5'(t - 16);
            expected_data[t] = 32'hE000 + (t - 16);
        end

        // First transaction: not the last cache line
        send_txn(1'b1, 1'b0, 3'd7, 5'd11, 32'hFF00_00FF, word_sel);

        tick();
        @(negedge clk);
        rsp_valid = 1'b1;
        rsp_line = make_line(32'hE000);
        #1;
        check(lsu_ready === 1'b0, "Not ready during a load response");

        tick();
        rsp_valid = 1'b0;
        check(lsu_ready === 1'b0, "Not ready on the first gather cycle");
        check(lsu.slot_mask[7] === 32'hFF00_00FF, "First response accumulated");

        tick();
        check(lsu_ready === 1'b1, "Ready again from the second gather cycle");

        // Second transaction: accepted on the first ready cycle
        txn_valid = 1'b1;
        txn_is_load = 1'b1;
        txn_last = 1'b1;
        txn_warp = 3'd7;
        txn_dst = 5'd11;
        txn_match_mask = 32'h00FF_FF00;
        txn_word_sel = word_sel;

        tick();
        txn_valid = 1'b0;
        check(lsu.outstanding_valid === 1'b1, "New transaction captured");
        check(lsu.outstanding_match_mask === 32'h00FF_FF00, "New transaction mask captured");
        check(lsu.outstanding_last === 1'b1, "Last flag captured for second transaction");
        check(lsu_valid === 8'b0, "No writeback before final response");

        // The cache's LOOKUP cycle
        tick();
        check(lsu.rsp_group === 2'd3, "First line is on its last gather group");

        // Return the second response at the cache's minimum latency
        rsp_valid = 1'b1;
        rsp_line = make_line(32'hF000);
        tick();
        rsp_valid = 1'b0;
        check(lsu.rsp_group === 2'd0, "Second line starts its gather");
        repeat (WB_LATENCY) tick();

        // Verify the complete load instruction
        for (int g = 0; g < 4; ++g) begin
            check_wb_group(3'd7, 2'(g), 5'd11, expected_mask, expected_data);
            tick();
        end

        check(lsu_valid === 8'b0, "Writeback completed");
        check(lsu.slot_mask[7] === 32'b0, "Warp 7 mask cleared");

        if (error_count === 0)
            $display("\nALL TESTS PASSED");
        else
            $fatal(1, "\nTESTS FAILED: %0d error(s)", error_count);

        $finish;
    end

    initial begin
        #2000000 $fatal("TIMEOUT");
    end
endmodule
