`timescale 1ns/1ps
module fetch_stage_tb;
    logic clk, reset;
    logic kernel_launch;
    logic issue_valid;
    logic [2:0] next_warp;
    logic [31:0] top_pc, recon_pc;
    logic fetch_allowed;
    logic consume;
    logic icache_ready, icache_rsp_valid;
    logic [127:0] icache_data;

    logic icache_req_valid;
    logic [31:0] icache_addr;
    logic fetch_valid;
    logic [31:0] instr, instr_pc;
    logic seq_pc_valid;
    logic [31:0] seq_next_pc;

    int errors = 0;
    int tests = 0;

    fetch_stage DUT (
        .clk(clk),
        .reset(reset),
        .kernel_launch(kernel_launch),
        .issue_valid(issue_valid),
        .next_warp(next_warp),
        .top_pc(top_pc),
        .recon_pc(recon_pc),
        .fetch_allowed(fetch_allowed),
        .consume(consume),
        .icache_ready(icache_ready),
        .icache_rsp_valid(icache_rsp_valid),
        .icache_data(icache_data),
        .icache_req_valid(icache_req_valid),
        .icache_addr(icache_addr),
        .fetch_valid(fetch_valid),
        .instr(instr),
        .instr_pc(instr_pc),
        .seq_pc_valid(seq_pc_valid),
        .seq_next_pc(seq_next_pc)
    );

    always #5 clk = ~clk;

    localparam logic [5:0] OP_ADD = 6'b000001, OP_BEQ = 6'b011000, OP_DONE = 6'b111111;

    // Instruction memory: every word is an ADD tagged with its PC, unless overridden
    logic [31:0] imem_override [logic [31:0]];
    function automatic logic [31:0] imem(input logic [31:0] pc);
        if (imem_override.exists(pc)) return imem_override[pc];
        return {OP_ADD, 10'b0, pc[15:0]};
    endfunction

    // Recon stack model: top_pc of each warp follows seq_next_pc
    logic [31:0] warp_pc [8];
    assign top_pc = warp_pc[next_warp];
    always @(posedge clk)
        if (seq_pc_valid) warp_pc[next_warp] <= seq_next_pc;

    // I-cache model: accepts a request when ready, responds rsp_latency cycles later
    int rsp_latency = 2;
    logic icache_auto;
    int pending_count = -1;
    logic [31:0] pending_addr;
    always @(posedge clk) begin
        if (reset) begin
            pending_count <= -1;
        end else if (icache_auto && icache_req_valid && icache_ready) begin
            pending_count <= rsp_latency;
            pending_addr <= icache_addr;
        end else if (pending_count > 0) begin
            pending_count <= pending_count - 1;
        end else begin
            pending_count <= -1;
        end
    end
    always_comb begin
        icache_rsp_valid = icache_auto && (pending_count == 0);
        for (int i = 0; i < 4; i++)
            icache_data[i*32 +: 32] = imem(pending_addr + 4*i);
    end

    int req_count = 0;
    always @(posedge clk)
        if (icache_req_valid && icache_ready) req_count++;

    task automatic tick();
        @(posedge clk);
        #1;
    endtask

    task automatic check_req(input exp_valid, input [31:0] exp_addr);
        tests++;
        if (icache_req_valid !== exp_valid || (exp_valid && icache_addr !== exp_addr)) begin
            errors++;
            $display("FAIL | expected req_valid = %0b addr = %0h | got req_valid = %0b addr = %0h",
                exp_valid, exp_addr, icache_req_valid, icache_addr);
        end else begin
            $display("PASS | req_valid = %0b addr = %0h", icache_req_valid, icache_addr);
        end
    endtask

    task automatic check(input [31:0] exp_pc, input exp_valid);
        tests++;
        if (fetch_valid !== exp_valid || (exp_valid && (instr !== imem(exp_pc) || instr_pc !== exp_pc))) begin
            errors++;
            $display("FAIL | expected instr = %h pc = %0h valid = %0b | got instr = %h pc = %0h valid = %0b",
                imem(exp_pc), exp_pc, exp_valid, instr, instr_pc, fetch_valid);
        end else begin
            $display("PASS | instr = %h pc = %0h valid = %0b", instr, instr_pc, fetch_valid);
        end
    endtask

    task automatic check_eq(input string msg, input logic [31:0] actual, input logic [31:0] expected);
        tests++;
        if (actual !== expected) begin
            errors++;
            $display("FAIL | %s = %0h, expected %0h", msg, actual, expected);
        end else begin
            $display("PASS | %s = %0h", msg, actual);
        end
    endtask

    // Wait until the prefetch buffer for next_warp has an instruction
    task automatic wait_fetch(input int max_cycles = 20);
        int n = 0;
        while (!fetch_valid && n < max_cycles) begin
            tick();
            n++;
        end
        if (!fetch_valid) begin
            errors++;
            $display("FAIL | no valid fetch within %0d cycles", max_cycles);
        end
    endtask

    // Consume the instruction at the head of the buffer
    task automatic do_consume();
        @(negedge clk);
        consume = 1'b1;
        #1;
        tests++;
        if (seq_pc_valid !== 1'b1 || seq_next_pc !== instr_pc + 4) begin
            errors++;
            $display("FAIL | consume: seq_pc_valid = %0b seq_next_pc = %0h (instr_pc = %0h)",
                     seq_pc_valid, seq_next_pc, instr_pc);
        end
        @(negedge clk);
        consume = 1'b0;
        #1;
    endtask

    task automatic set_warp(input [2:0] w, input [31:0] pc);
        @(negedge clk);
        while (icache_req_valid || DUT.req_outstanding) @(negedge clk);
        warp_pc[w] = pc;
        next_warp = w;
        #1;
    endtask

    initial begin
        clk = 1'b0;
        reset = 1'b1;
        kernel_launch = 1'b0;
        issue_valid = 1'b1;
        fetch_allowed = 1'b1;
        next_warp = 3'd0;
        recon_pc = 32'hFFFF_FFFF;
        consume = 1'b0;
        icache_ready = 1'b0;
        icache_auto = 1'b0;
        foreach (warp_pc[w]) warp_pc[w] = 32'h1000 + 32'h100 * w;

        tick();
        $display("No request while reset is high");
        check_req(1'b0, 32'h0);

        reset = 1'b0;
        #1;
        $display("\nNo request during kernel_launch");
        kernel_launch = 1'b1;
        #1;
        check_req(1'b0, 32'h0);
        tick();
        kernel_launch = 1'b0;
        #1;

        $display("\nNo request without issue_valid or fetch_allowed");
        issue_valid = 1'b0;
        #1;
        check_req(1'b0, 32'h0);
        issue_valid = 1'b1;
        fetch_allowed = 1'b0;
        #1;
        check_req(1'b0, 32'h0);
        fetch_allowed = 1'b1;
        #1;

        // 1. Empty buffer requests top_pc and keeps it up until the I$ accepts
        $display("\nRequest held until the I$ accepts it");
        check_req(1'b1, 32'h1000);
        tick();
        check_req(1'b1, 32'h1000);
        icache_auto = 1'b1;
        icache_ready = 1'b1;
        tick();
        icache_ready = 1'b0;
        #1;
        $display("No duplicate request while the response is outstanding");
        check_req(1'b0, 32'h1000);
        check(32'h0, 1'b0);
        wait_fetch();
        check(32'h1000, 1'b1);
        check_eq("requests issued", req_count, 1);

        // 2. Buffer serves four instructions, then refetches from the updated PC
        $display("\nFour instructions from one fill, consume reports the next sequential PC");
        icache_ready = 1'b1;
        for (int i = 0; i < 4; i++) begin
            check(32'h1000 + 4*i, 1'b1);
            check_req(1'b0, 32'h0);
            do_consume();
        end
        check_eq("recon stack PC after 4 consumes", warp_pc[0], 32'h1010);
        $display("Buffer drained => new request at the updated PC");
        #1;
        check_req(1'b1, 32'h1010);
        wait_fetch();
        check(32'h1010, 1'b1);
        check_eq("requests issued", req_count, 2);

        // 3. fetch_allowed / issue_valid gate fetch_valid without losing the buffer
        $display("\nfetch_allowed low hides the instruction, buffer is kept");
        fetch_allowed = 1'b0;
        #1;
        check(32'h0, 1'b0);
        tick(); tick();
        fetch_allowed = 1'b1;
        #1;
        check(32'h1010, 1'b1);
        check_eq("no refetch needed", req_count, 2);

        // 4. Cache-line boundary: entries past the end of the 128B line are invalid
        $display("\nFill stops at the cache-line boundary");
        set_warp(3'd1, 32'h1178); // line ends at 0x1180
        wait_fetch();
        check(32'h1178, 1'b1);
        do_consume();
        check(32'h117C, 1'b1);
        do_consume();
        check(32'h0, 1'b0);
        #1;
        check_req(1'b1, 32'h1180);
        wait_fetch();
        check(32'h1180, 1'b1);

        // 5. Reconvergence bound: entries at or past recon_pc are invalid
        $display("\nFill stops at the reconvergence PC");
        set_warp(3'd2, 32'h1200);
        recon_pc = 32'h1208;
        wait_fetch();
        check(32'h1200, 1'b1);
        do_consume();
        check(32'h1204, 1'b1);
        do_consume();
        check(32'h0, 1'b0);
        recon_pc = 32'hFFFF_FFFF;

        // 5b. A fill uses the PC and recon_pc from when its request was accepted
        $display("\nFill uses the PC and reconvergence PC latched at request time");
        set_warp(3'd7, 32'h1700);
        recon_pc = 32'h1708;
        tick();
        warp_pc[7] = 32'h1780;
        recon_pc = 32'hFFFF_FFFF;
        wait_fetch();
        check(32'h1700, 1'b1);
        do_consume();
        check(32'h1704, 1'b1);
        do_consume();
        check(32'h0, 1'b0);

        // 6. A branch or DONE invalidates the rest of the buffer
        $display("\nConsuming a branch invalidates the remaining entries");
        imem_override[32'h1304] = {OP_BEQ, 5'd1, 5'd2, 8'd8, 8'd16};
        set_warp(3'd3, 32'h1300);
        wait_fetch();
        do_consume();                      // 0x1300 ADD
        check(32'h1304, 1'b1);
        do_consume();                      // 0x1304 BEQ
        check(32'h0, 1'b0);

        $display("\nConsuming DONE invalidates the remaining entries");
        imem_override[32'h1400] = {OP_DONE, 26'b0};
        set_warp(3'd4, 32'h1400);
        wait_fetch();
        do_consume();
        check(32'h0, 1'b0);

        // 7. Each warp has its own buffer
        $display("\nWarp buffers are independent");
        set_warp(3'd5, 32'h1500);
        wait_fetch();
        check(32'h1500, 1'b1);
        set_warp(3'd6, 32'h1600);
        #1;
        check_req(1'b1, 32'h1600);
        wait_fetch();
        check(32'h1600, 1'b1);
        begin
            int req_before;
            req_before = req_count;
            set_warp(3'd5, 32'h1500);
            #1;
            $display("Switching back to warp 5 reuses its buffer");
            check(32'h1500, 1'b1);
            check_req(1'b0, 32'h0);
            check_eq("no new request", req_count, req_before);
        end

        // 8. kernel_launch clears every buffer
        $display("\nkernel_launch clears the buffers");
        @(negedge clk);
        kernel_launch = 1'b1;
        @(negedge clk);
        kernel_launch = 1'b0;
        #1;
        check(32'h0, 1'b0);
        check_req(1'b1, 32'h1500);

        $display("\ntests=%0d errors=%0d", tests, errors);
        if (errors == 0) $display("ALL TESTS PASSED");
        else $fatal(1, "%0d TEST(S) FAILED", errors);
        $finish;
    end

    initial begin
        #100000;
        $fatal(1, "FAIL | testbench timeout");
    end
endmodule
