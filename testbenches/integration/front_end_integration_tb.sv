`timescale 1ns/1ps
// I-cache stand-in with the same handshake as l1_icache:
// accepts a request when ready, then pulses rsp_valid with the 4 words starting at the request address
module l1_i_cache_stub #(
    parameter int MEM_WORDS = 16384
) (
    input logic clk,
    input logic reset,
    input int latency,
    input logic req_valid,
    input logic [31:0] addr,
    output logic ready,
    output logic rsp_valid,
    output logic [127:0] data
);
    logic [31:0] mem [0:MEM_WORDS-1];

    logic pending;
    logic [31:0] req_addr_q;
    int cnt;

    initial begin
        for (int i = 0; i < MEM_WORDS; i++) mem[i] = 32'hFC00_0000; // DONE everywhere by default
    end

    assign ready = !pending;

    always_ff @(posedge clk) begin
        if (reset) begin
            pending <= 1'b0;
            rsp_valid <= 1'b0;
            cnt <= 0;
        end else begin
            rsp_valid <= 1'b0;
            if (!pending && req_valid) begin
                pending <= 1'b1;
                req_addr_q <= addr;
                cnt <= latency;
            end else if (pending) begin
                if (cnt <= 1) begin
                    rsp_valid <= 1'b1;
                    pending <= 1'b0;
                end else begin
                    cnt <= cnt - 1;
                end
            end
        end
    end

    logic [13:0] base;
    assign base = req_addr_q[15:2];
    assign data = {mem[base + 14'd3], mem[base + 14'd2], mem[base + 14'd1], mem[base]};
endmodule

// Front end of sm_top (scheduler, sub-warp counter, reconvergence stacks, fetch, instruction latch,
// decode, register file) wired exactly as in sm_top. The execution units, commit tracker,
// mask_split_unit and data cache are replaced by small models.
module front_end_integration_tb;
    logic clk = 0;
    logic reset;
    always #5 clk = ~clk;

    integer errors = 0;
    integer checks = 0;
    task automatic check(input logic cond, input string msg);
        checks = checks + 1;
        if (cond) $display("[PASS] t=%0t: %s", $time, msg);
        else begin
            $display("[FAIL] t=%0t: %s", $time, msg);
            errors = errors + 1;
        end
    endtask

    // ------------------------------------------------------------------
    // Instruction encodings (same as final_integration_tb)
    // ------------------------------------------------------------------
    localparam logic [5:0] OP_ADD = 6'b000001, OP_ADDI = 6'b001001, OP_FMA = 6'b010000;
    localparam logic [5:0] OP_BEQ = 6'b011000, OP_BNE = 6'b011001;
    localparam logic [5:0] OP_MOV_TID = 6'b100001;
    localparam logic [31:0] DONE_INSTR = 32'hFC00_0000;

    function automatic logic [31:0] r_type(input logic [5:0] opc, input logic [4:0] s1, s2, d);
        return {opc, s1, s2, d, 11'b0};
    endfunction
    function automatic logic [31:0] i_type(input logic [5:0] opc, input logic [4:0] s1, d, input logic [15:0] imm);
        return {opc, s1, d, imm};
    endfunction
    function automatic logic [31:0] fma_type(input logic [4:0] s1, s2, s3, d);
        return {OP_FMA, s1, s2, s3, d, 6'b0};
    endfunction
    function automatic logic [31:0] special_type(input logic [5:0] opc, input logic [4:0] d);
        return {opc, d, 21'b0};
    endfunction
    // target and reconvergence offsets are signed byte offsets from the branch PC
    function automatic logic [31:0] branch_instr(input logic [5:0] opc, input logic [4:0] s1, s2,
                                                 input logic [7:0] toff, roff);
        return {opc, s1, s2, toff, roff};
    endfunction

    // ------------------------------------------------------------------
    // DUT front end
    // ------------------------------------------------------------------
    logic kernel_start;
    logic [3:0] num_warps;
    logic [31:0] start_pc;
    logic [31:0] init_mask_arr [0:7];
    logic tb_buffer_full;
    int icache_latency;

    logic fetch_valid, sub_warp_valid, have_ready;
    logic [1:0] sub_warp_cycle;

    sub_warp_counter u_swc (
        .clk(clk),
        .reset(reset),
        .fetch_valid(fetch_valid),
        .buffer_full(tb_buffer_full),
        .have_ready(have_ready),
        .sub_warp_cycle(sub_warp_cycle),
        .sub_warp_valid(sub_warp_valid)
    );

    logic consume, fetch_allowed;
    logic dec_done_detect;
    logic [7:0] commit_done;
    logic [7:0] recon_push2_done_arr;
    logic [2:0] ws_curr_warp, ws_next_warp;
    logic ws_issue_valid, warps_done, kernel_done, flush_done;

    assign consume = ws_issue_valid && ~|sub_warp_cycle && sub_warp_valid && !tb_buffer_full;

    warp_scheduler u_ws (
        .clk(clk),
        .reset(reset),
        .kernel_start(kernel_start),
        .num_warps(num_warps),
        .commit_done(commit_done),
        .store_done(8'b0),
        .push2_done(recon_push2_done_arr),
        .have_ready(have_ready),
        .consume(consume),
        .fetch_allowed(fetch_allowed),
        .sub_warp_cycle(sub_warp_cycle),
        .sub_warp_valid(sub_warp_valid),
        .buffer_full(tb_buffer_full),
        .done_detect(dec_done_detect),
        .flush_done(flush_done),
        .curr_warp(ws_curr_warp),
        .next_warp(ws_next_warp),
        .issue_valid(ws_issue_valid),
        .warps_done(warps_done),
        .kernel_done(kernel_done)
    );

    logic [7:0][31:0] top_mask_arr, top_pc_arr, top_recon_pc_arr;
    logic [7:0] push2_valid;
    logic [31:0] push2_mask_taken, push2_mask_not_taken;
    logic [31:0] push2_pc_taken, push2_pc_fallthrough, push2_recon_pc;
    logic push2_skip_taken, push2_skip_not_taken;
    logic seq_pc_valid;
    logic [31:0] seq_next_pc;

    // Registered PC update into the recon stacks
    logic [7:0] seq_pc_valid_q;
    logic [31:0] seq_next_pc_q;

    always_ff @(posedge clk) begin
        if (reset)
            seq_pc_valid_q <= '0;
        else
            seq_pc_valid_q <= {8{seq_pc_valid}} & (8'b1 << ws_next_warp);

        seq_next_pc_q <= seq_next_pc;
    end

    genvar gi;
    generate
        for (gi = 0; gi < 8; gi++) begin : g_recon
            recon_stack #(.DEPTH(8)) u_recon (
                .clk(clk),
                .reset(reset),
                .init_push(kernel_start && (gi < num_warps)),
                .init_mask(init_mask_arr[gi]),
                .init_pc(start_pc),
                .init_recon_pc(32'hFFFF_FFFF),
                .push2_valid(push2_valid[gi]),
                .push2_mask_taken(push2_mask_taken),
                .push2_mask_not_taken(push2_mask_not_taken),
                .push2_pc_taken(push2_pc_taken),
                .push2_pc_fallthrough(push2_pc_fallthrough),
                .push2_recon_pc(push2_recon_pc),
                .push2_skip_taken(push2_skip_taken),
                .push2_skip_not_taken(push2_skip_not_taken),
                .current_pc_valid(seq_pc_valid_q[gi]),
                .current_pc(seq_next_pc_q),
                .top_mask(top_mask_arr[gi]),
                .top_pc(top_pc_arr[gi]),
                .top_recon_pc(top_recon_pc_arr[gi]),
                .stack_empty(),
                .stack_full(),
                .push2_done(recon_push2_done_arr[gi])
            );
        end
    endgenerate

    logic ic_req_valid, ic_ready, ic_rsp_valid;
    logic [31:0] ic_addr;
    logic [127:0] ic_data;

    l1_i_cache_stub u_icache (
        .clk(clk),
        .reset(reset),
        .latency(icache_latency),
        .req_valid(ic_req_valid),
        .addr(ic_addr),
        .ready(ic_ready),
        .rsp_valid(ic_rsp_valid),
        .data(ic_data)
    );

    logic [31:0] fs_instr, fs_instr_pc;

    fetch_stage u_fetch (
        .clk(clk),
        .reset(reset),
        .kernel_launch(kernel_start),
        .issue_valid(ws_issue_valid),
        .next_warp(ws_next_warp),
        .top_pc(top_pc_arr[ws_next_warp]),
        .recon_pc(top_recon_pc_arr[ws_next_warp]),
        .fetch_allowed(fetch_allowed),
        .consume(consume),
        .icache_ready(ic_ready),
        .icache_rsp_valid(ic_rsp_valid),
        .icache_data(ic_data),
        .icache_req_valid(ic_req_valid),
        .icache_addr(ic_addr),
        .fetch_valid(fetch_valid),
        .instr(fs_instr),
        .instr_pc(fs_instr_pc),
        .seq_pc_valid(seq_pc_valid),
        .seq_next_pc(seq_next_pc)
    );

    logic [31:0] il_instr, il_instr_pc, il_active_mask;
    logic [9:0] il_src1_addr, il_src2_addr, il_src3_addr;

    instruction_latch u_ilatch (
        .clk(clk),
        .reset(reset),
        .buffer_full(tb_buffer_full),
        .consume(consume),
        .sub_warp_cycle(sub_warp_cycle),
        .next_warp(ws_next_warp),
        .curr_warp(ws_curr_warp),
        .buf_instr(fs_instr),
        .buf_instr_pc(fs_instr_pc),
        .active_mask(top_mask_arr[ws_next_warp]),
        .instr_q(il_instr),
        .instr_pc_q(il_instr_pc),
        .active_mask_q(il_active_mask),
        .src1_addr(il_src1_addr),
        .src2_addr(il_src2_addr),
        .src3_addr(il_src3_addr)
    );

    logic dec_sub_warp_valid;
    logic [1:0] dec_sub_warp_cycle;
    logic [5:0] dec_opcode;
    logic [7:0] dec_alu_valid, dec_fma_valid, dec_special_reg_valid;
    logic [9:0] dec_dest_addr;
    logic [15:0] dec_imm;
    logic [31:0] dec_instr_pc;

    decode_stage u_decode (
        .clk(clk),
        .reset(reset),
        .buffer_full(tb_buffer_full),
        .sub_warp_valid(sub_warp_valid),
        .sub_warp_cycle(sub_warp_cycle),
        .curr_warp(ws_curr_warp),
        .active_mask(il_active_mask),
        .instr(il_instr),
        .instr_pc(il_instr_pc),
        .sub_warp_valid_q(dec_sub_warp_valid),
        .sub_warp_cycle_q(dec_sub_warp_cycle),
        .opcode_q(dec_opcode),
        .done_detect(dec_done_detect),
        .alu_valid_q(dec_alu_valid),
        .fma_valid_q(dec_fma_valid),
        .special_reg_valid_q(dec_special_reg_valid),
        .dest_addr_q(dec_dest_addr),
        .imm_q(dec_imm),
        .instr_pc_q(dec_instr_pc)
    );

    // lane 0's register bank; every lane reads the same addresses
    logic [31:0] reg_src1_data, reg_src2_data, reg_src3_data;

    reg_file u_regfile (
        .clk(clk),
        .reset(reset),
        .src1_addr(il_src1_addr),
        .src2_addr(il_src2_addr),
        .src3_addr(il_src3_addr),
        .write_en(1'b0),
        .wr_addr(10'b0),
        .wr_data(32'b0),
        .src1_data(reg_src1_data),
        .src2_data(reg_src2_data),
        .src3_data(reg_src3_data)
    );

    // ------------------------------------------------------------------
    // Models of the back end
    // ------------------------------------------------------------------
    // Decoded instructions are accepted downstream on every edge where the writeback buffer
    // is not full (the ALU/FMA hold their inputs otherwise)
    wire dec_accept = dec_sub_warp_valid && !tb_buffer_full;
    wire dec_is_branch = (dec_opcode[5:3] == 3'b011);
    wire dec_is_done = (dec_opcode == 6'b111111);

    // Commit model: an ALU/FMA/special instruction commits COMMIT_LAT cycles after its last sub-warp
    localparam int COMMIT_LAT = 3;
    int commit_at [8];
    int cycle_count = 0;
    always @(posedge clk) cycle_count <= cycle_count + 1;

    // Branch model (stands in for int_alu + mask_split_unit): once the branch's last sub-warp is
    // decoded, push the taken/not-taken masks to the warp's stack. taken_plan[pc] picks which
    // threads take the branch at that PC (all threads by default).
    logic [31:0] taken_plan [logic [31:0]];
    int push_at;
    logic [2:0] push_warp;
    logic [31:0] push_mask_t, push_mask_nt, push_pc_t, push_pc_ft, push_pc_r;

    always @(posedge clk) begin
        if (reset) begin
            foreach (commit_at[w]) commit_at[w] <= -1;
            commit_done <= '0;
            push_at <= -1;
            push2_valid <= '0;
            push2_skip_taken <= 1'b1;
            push2_skip_not_taken <= 1'b1;
        end else begin
            commit_done <= '0;
            push2_valid <= '0;

            if (dec_accept && dec_sub_warp_cycle == 2'd3 && !dec_is_done) begin
                if (dec_is_branch) begin
                    logic [31:0] plan;
                    plan = taken_plan.exists(dec_instr_pc) ? taken_plan[dec_instr_pc] : 32'hFFFF_FFFF;
                    push_at <= cycle_count + 2;
                    push_warp <= dec_dest_addr[9:7];
                    push_mask_t <= il_active_mask & plan;
                    push_mask_nt <= il_active_mask & ~plan;
                    push_pc_t <= dec_instr_pc + {{24{dec_imm[15]}}, dec_imm[15:8]};
                    push_pc_ft <= dec_instr_pc + 32'd4;
                    push_pc_r <= dec_instr_pc + {{24{dec_imm[7]}}, dec_imm[7:0]};
                end else begin
                    commit_at[dec_dest_addr[9:7]] <= cycle_count + COMMIT_LAT;
                end
            end

            foreach (commit_at[w]) begin
                if (commit_at[w] == cycle_count) begin
                    commit_done[w] <= 1'b1;
                    commit_at[w] <= -1;
                end
            end

            if (push_at == cycle_count) begin
                push2_valid[push_warp] <= 1'b1;
                push2_mask_taken <= push_mask_t;
                push2_mask_not_taken <= push_mask_nt;
                push2_pc_taken <= push_pc_t;
                push2_pc_fallthrough <= push_pc_ft;
                push2_recon_pc <= push_pc_r;
                push2_skip_taken <= (push_pc_t == push_pc_r) || ~|push_mask_t;
                push2_skip_not_taken <= (push_pc_ft == push_pc_r) || ~|push_mask_nt;
                push_at <= -1;
            end
        end
    end

    // Data cache flush model: flush_done pulses a few cycles after warps_done
    int flush_count;
    always @(posedge clk) begin
        flush_done <= 1'b0;
        if (reset || !warps_done || kernel_done) flush_count <= 0;
        else if (flush_count == 3) flush_done <= 1'b1;
        else flush_count <= flush_count + 1;
    end

    // ------------------------------------------------------------------
    // Trace monitor: one entry per decoded instruction, with the 32-thread lane mask
    // assembled from the four sub-warps. The warp/PC/dst are taken at sub-warp 0: after DONE the
    // scheduler switches curr_warp right away, so DONE's (lane-less) sub-warps 1-3 carry the next
    // warp's ID
    // ------------------------------------------------------------------
    typedef struct {
        logic [2:0] warp;
        logic [31:0] pc;
        logic [31:0] mask;
        logic [4:0] dst;
        logic [5:0] opcode;
    } trace_t;

    trace_t trace [$];
    trace_t cur;
    logic [31:0] mask_acc;
    logic [1:0] exp_sub_warp;
    int sub_warp_order_errors = 0;

    always @(posedge clk) begin
        if (reset) begin
            exp_sub_warp = 2'd0;
            mask_acc = '0;
        end else if (dec_accept) begin
            if (dec_sub_warp_cycle != exp_sub_warp) sub_warp_order_errors++;
            exp_sub_warp = dec_sub_warp_cycle + 2'd1;
            if (dec_sub_warp_cycle == 2'd0)
                cur = '{dec_dest_addr[9:7], dec_instr_pc, 32'b0, dec_dest_addr[4:0], dec_opcode};
            mask_acc[dec_sub_warp_cycle*8 +: 8] = dec_alu_valid | dec_fma_valid | dec_special_reg_valid;
            if (dec_sub_warp_cycle == 2'd3) begin
                cur.mask = mask_acc;
                trace.push_back(cur);
                mask_acc = '0;
            end
        end
    end

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------
    task automatic tick(input int n = 1);
        repeat (n) begin
            @(posedge clk);
            #1;
        end
    endtask

    task automatic do_reset();
        reset = 1'b1;
        tb_buffer_full = 1'b0;
        kernel_start = 1'b0;
        num_warps = 4'd0;
        start_pc = 32'b0;
        icache_latency = 2;
        foreach (init_mask_arr[w]) init_mask_arr[w] = 32'hFFFF_FFFF;
        taken_plan.delete();
        tick(3);
        reset = 1'b0;
        tick(2);
        trace.delete();
        sub_warp_order_errors = 0;
    endtask

    task automatic load_word(input logic [31:0] addr, input logic [31:0] w);
        u_icache.mem[addr[15:2]] = w;
    endtask

    task automatic poke_reg(input logic [2:0] warp, input logic [1:0] sw, input logic [4:0] r, input logic [31:0] v);
        u_regfile.bram1[{warp, sw, r}] = v;
        u_regfile.bram2[{warp, sw, r}] = v;
        u_regfile.bram3[{warp, sw, r}] = v;
    endtask

    task automatic launch_kernel(input int nwarps, input logic [31:0] spc);
        @(negedge clk);
        num_warps = nwarps[3:0];
        start_pc = spc;
        kernel_start = 1'b1;
        @(negedge clk);
        kernel_start = 1'b0;
    endtask

    task automatic wait_kernel_done(input string name, input int max_cycles = 2000);
        int n = 0;
        while (!kernel_done && n < max_cycles) begin
            tick();
            n++;
        end
        check(kernel_done, $sformatf("%s: kernel_done within %0d cycles", name, max_cycles));
        check(sub_warp_order_errors == 0, $sformatf("%s: sub-warps always decoded in order 0,1,2,3", name));
    endtask

    // Compare the decoded instruction stream of one warp against the expected (pc, mask) list
    task automatic check_warp_trace(input string name, input logic [2:0] warp,
                                    input logic [31:0] exp_pc [], input logic [31:0] exp_mask []);
        trace_t got [$];
        foreach (trace[i]) if (trace[i].warp == warp) got.push_back(trace[i]);
        check(got.size() == exp_pc.size(),
              $sformatf("%s: warp %0d decoded %0d instructions, expected %0d", name, warp, got.size(), exp_pc.size()));
        for (int i = 0; i < exp_pc.size() && i < got.size(); i++)
            check(got[i].pc == exp_pc[i] && got[i].mask == exp_mask[i],
                  $sformatf("%s: warp %0d instr %0d pc = %h mask = %h (expected pc = %h mask = %h)",
                            name, warp, i, got[i].pc, got[i].mask, exp_pc[i], exp_mask[i]));
    endtask

    // ------------------------------------------------------------------
    // Scenarios
    // ------------------------------------------------------------------

    // 1. Register reads line up with decode: the data read for {warp, sub-warp} arrives
    //    on the same cycle decode emits that sub-warp
    task automatic scenario1_tag_alignment();
        logic [3:0] seen;
        $display("\nScenario 1: register read / decode tag alignment");
        do_reset();
        load_word(32'h1000, r_type(OP_ADD, 5'd2, 5'd3, 5'd1));
        load_word(32'h1004, DONE_INSTR);
        for (int sw = 0; sw < 4; sw++) begin
            poke_reg(3'd0, sw[1:0], 5'd2, 32'hCAFE_0000 + sw);
            poke_reg(3'd0, sw[1:0], 5'd3, 32'hBEEF_0000 + sw);
        end
        launch_kernel(1, 32'h1000);

        seen = '0;
        for (int c = 0; c < 60 && seen != 4'hF; c++) begin
            tick();
            if (dec_sub_warp_valid && dec_instr_pc == 32'h1000 && !seen[dec_sub_warp_cycle]) begin
                seen[dec_sub_warp_cycle] = 1'b1;
                check(reg_src1_data == 32'hCAFE_0000 + dec_sub_warp_cycle &&
                      reg_src2_data == 32'hBEEF_0000 + dec_sub_warp_cycle,
                      $sformatf("scenario1: sub-warp %0d reads src1 = %h src2 = %h", dec_sub_warp_cycle,
                                reg_src1_data, reg_src2_data));
                check(dec_dest_addr == {3'd0, dec_sub_warp_cycle, 5'd1},
                      $sformatf("scenario1: sub-warp %0d dest_addr = %b", dec_sub_warp_cycle, dec_dest_addr));
            end
        end
        check(seen == 4'hF, "scenario1: all four sub-warps were decoded");
        wait_kernel_done("scenario1");
    endtask

    // 2. buffer_full in the middle of an instruction freezes the front end; nothing is lost or repeated
    task automatic scenario2_buffer_full_hold();
        logic [9:0] held_dest, held_src1;
        logic [1:0] held_cycle;
        $display("\nScenario 2: buffer_full mid-instruction stall");
        do_reset();
        for (int i = 0; i < 4; i++) load_word(32'h2000 + 4*i, r_type(OP_ADD, 5'd2, 5'd3, 5'(i + 1)));
        load_word(32'h2010, DONE_INSTR);
        launch_kernel(1, 32'h2000);

        while (!(dec_sub_warp_valid && dec_instr_pc == 32'h2004 && dec_sub_warp_cycle == 2'd1)) tick();
        @(negedge clk);
        tb_buffer_full = 1'b1;
        held_dest = dec_dest_addr;
        held_src1 = il_src1_addr;
        held_cycle = sub_warp_cycle;
        tick(6);
        check(dec_dest_addr == held_dest, "scenario2: decode output held during stall");
        check(il_src1_addr == held_src1, "scenario2: register read address held during stall");
        check(sub_warp_cycle == held_cycle, "scenario2: sub_warp_cycle held during stall");
        check(!sub_warp_valid, "scenario2: sub_warp_valid low during stall");
        @(negedge clk);
        tb_buffer_full = 1'b0;

        wait_kernel_done("scenario2");
        check_warp_trace("scenario2", 3'd0,
            '{32'h2000, 32'h2004, 32'h2008, 32'h200C, 32'h2010},
            '{32'hFFFF_FFFF, 32'hFFFF_FFFF, 32'hFFFF_FFFF, 32'hFFFF_FFFF, 32'h0});
    endtask

    // 3. Several warps run the same program; each warp's stream is complete and in order
    task automatic scenario3_multi_warp();
        logic [31:0] pcs [] = '{32'h3000, 32'h3004, 32'h3008, 32'h300C};
        logic [31:0] masks [] = '{32'hFFFF_FFFF, 32'hFFFF_FFFF, 32'hFFFF_FFFF, 32'h0};
        $display("\nScenario 3: four warps, same program");
        do_reset();
        for (int i = 0; i < 3; i++) load_word(32'h3000 + 4*i, r_type(OP_ADD, 5'd2, 5'd3, 5'd1));
        load_word(32'h300C, DONE_INSTR);
        launch_kernel(4, 32'h3000);
        wait_kernel_done("scenario3");
        for (int w = 0; w < 4; w++)
            check_warp_trace("scenario3", w[2:0], pcs, masks);
        for (int w = 4; w < 8; w++)
            check_warp_trace("scenario3", w[2:0], '{}, '{});
    endtask

    // 4. Each instruction format puts its destination register in the right field
    task automatic scenario4_decode_formats();
        int idx;
        $display("\nScenario 4: decode format-dependent destination mux");
        do_reset();
        load_word(32'h4000, i_type(OP_ADDI, 5'd1, 5'd12, 16'h0005));
        load_word(32'h4004, fma_type(5'd1, 5'd2, 5'd3, 5'd9));
        load_word(32'h4008, special_type(OP_MOV_TID, 5'd20));
        load_word(32'h400C, branch_instr(OP_BEQ, 5'd0, 5'd0, 8'd4, 8'd4)); // to the next instruction
        load_word(32'h4010, r_type(OP_ADD, 5'd2, 5'd3, 5'd7));
        load_word(32'h4014, DONE_INSTR);
        launch_kernel(1, 32'h4000);
        wait_kernel_done("scenario4");
        check(trace.size() == 6, $sformatf("scenario4: 6 instructions decoded (got %0d)", trace.size()));
        if (trace.size() == 6) begin
            check(trace[0].dst == 5'd12, "scenario4: I-type dst from [20:16]");
            check(trace[1].dst == 5'd9,  "scenario4: FMA dst from [10:6]");
            check(trace[2].dst == 5'd20, "scenario4: special dst from [25:21]");
            check(trace[3].dst == 5'd0,  "scenario4: branch has no dst");
            check(trace[4].dst == 5'd7,  "scenario4: R-type dst from [15:11]");
            check(trace[0].mask == 32'hFFFF_FFFF && trace[1].mask == 32'hFFFF_FFFF &&
                  trace[2].mask == 32'hFFFF_FFFF && trace[3].mask == 32'hFFFF_FFFF,
                  "scenario4: every lane valid on its own execution unit");
        end
    endtask

    // 5. The warp's active mask gates decode's per-lane valid bits
    task automatic scenario5_masking();
        $display("\nScenario 5: active mask gates per-lane valid bits");
        do_reset();
        load_word(32'h5000, r_type(OP_ADD, 5'd2, 5'd3, 5'd1));
        load_word(32'h5004, fma_type(5'd1, 5'd2, 5'd3, 5'd4));
        load_word(32'h5008, DONE_INSTR);
        init_mask_arr[0] = 32'hF00F_0FF0;
        launch_kernel(1, 32'h5000);
        wait_kernel_done("scenario5");
        check_warp_trace("scenario5", 3'd0,
            '{32'h5000, 32'h5004, 32'h5008},
            '{32'hF00F_0FF0, 32'hF00F_0FF0, 32'h0});
    endtask

    // 6. Uniform branch over a block: the skipped instructions are never decoded
    task automatic scenario6_uniform_branch();
        $display("\nScenario 6: uniform taken branch redirects fetch");
        do_reset();
        load_word(32'h6000, r_type(OP_ADD, 5'd2, 5'd3, 5'd1));
        load_word(32'h6004, branch_instr(OP_BEQ, 5'd0, 5'd0, 8'h1C, 8'h1C)); // skip to 0x6020
        for (int i = 0; i < 6; i++) load_word(32'h6008 + 4*i, r_type(OP_ADD, 5'd2, 5'd3, 5'd31));
        load_word(32'h6020, r_type(OP_ADD, 5'd2, 5'd3, 5'd2));
        load_word(32'h6024, DONE_INSTR);
        launch_kernel(1, 32'h6000);
        wait_kernel_done("scenario6");
        check_warp_trace("scenario6", 3'd0,
            '{32'h6000, 32'h6004, 32'h6020, 32'h6024},
            '{32'hFFFF_FFFF, 32'hFFFF_FFFF, 32'hFFFF_FFFF, 32'h0});
    endtask

    // 7. Divergent if: the body runs only for the not-taken threads, then all threads reconverge
    task automatic scenario7_divergent_if();
        $display("\nScenario 7: divergent if (branch skips the body for some threads)");
        do_reset();
        load_word(32'h7000, r_type(OP_ADD, 5'd2, 5'd3, 5'd1));
        load_word(32'h7004, branch_instr(OP_BNE, 5'd1, 5'd0, 8'h0C, 8'h0C)); // skip to 0x7010
        load_word(32'h7008, r_type(OP_ADD, 5'd2, 5'd3, 5'd5));
        load_word(32'h700C, r_type(OP_ADD, 5'd2, 5'd3, 5'd6));
        load_word(32'h7010, r_type(OP_ADD, 5'd2, 5'd3, 5'd7));
        load_word(32'h7014, DONE_INSTR);
        taken_plan[32'h7004] = 32'hFFFF_0000;
        launch_kernel(1, 32'h7000);
        wait_kernel_done("scenario7");
        check_warp_trace("scenario7", 3'd0,
            '{32'h7000, 32'h7004, 32'h7008, 32'h700C, 32'h7010, 32'h7014},
            '{32'hFFFF_FFFF, 32'hFFFF_FFFF, 32'h0000_FFFF, 32'h0000_FFFF, 32'hFFFF_FFFF, 32'h0});
    endtask

    // 8. Divergent if/else: taken path first, then the else path (ending in a jump to the
    //    reconvergence point), then all threads reconverge
    task automatic scenario8_divergent_if_else();
        localparam logic [31:0] T = 32'h00FF_00FF;
        $display("\nScenario 8: divergent if/else with a jump to the reconvergence point");
        do_reset();
        load_word(32'h8000, r_type(OP_ADD, 5'd2, 5'd3, 5'd1));
        load_word(32'h8004, branch_instr(OP_BEQ, 5'd1, 5'd0, 8'h10, 8'h18)); // then: 0x8014, recon: 0x801C
        load_word(32'h8008, r_type(OP_ADD, 5'd2, 5'd3, 5'd5));               // else body
        load_word(32'h800C, r_type(OP_ADD, 5'd2, 5'd3, 5'd6));
        load_word(32'h8010, branch_instr(OP_BEQ, 5'd0, 5'd0, 8'h0C, 8'h0C)); // jump to 0x801C
        load_word(32'h8014, r_type(OP_ADD, 5'd2, 5'd3, 5'd7));               // then body
        load_word(32'h8018, r_type(OP_ADD, 5'd2, 5'd3, 5'd8));
        load_word(32'h801C, r_type(OP_ADD, 5'd2, 5'd3, 5'd9));               // reconverged
        load_word(32'h8020, DONE_INSTR);
        taken_plan[32'h8004] = T;
        launch_kernel(1, 32'h8000);
        wait_kernel_done("scenario8");
        check_warp_trace("scenario8", 3'd0,
            '{32'h8000, 32'h8004, 32'h8014, 32'h8018, 32'h8008, 32'h800C, 32'h8010, 32'h801C, 32'h8020},
            '{32'hFFFF_FFFF, 32'hFFFF_FFFF, T, T, ~T, ~T, ~T, 32'hFFFF_FFFF, 32'h0});
    endtask

    // 9. Slow I-cache: nothing is decoded until the first fill arrives
    task automatic scenario9_slow_icache();
        logic spurious;
        $display("\nScenario 9: no issue before the first instruction is available");
        do_reset();
        icache_latency = 20;
        load_word(32'h9000, r_type(OP_ADD, 5'd2, 5'd3, 5'd1));
        load_word(32'h9004, DONE_INSTR);
        launch_kernel(1, 32'h9000);
        spurious = 1'b0;
        for (int c = 0; c < 18; c++) begin
            tick();
            if (sub_warp_valid || dec_sub_warp_valid) spurious = 1'b1;
        end
        check(!spurious, "scenario9: no valid sub-warp while the I-cache is still filling");
        wait_kernel_done("scenario9");
        check_warp_trace("scenario9", 3'd0, '{32'h9000, 32'h9004}, '{32'hFFFF_FFFF, 32'h0});
    endtask

    // 10. A program that crosses a 128-byte cache line runs straight through
    task automatic scenario10_line_crossing();
        logic [31:0] pcs [];
        logic [31:0] masks [];
        $display("\nScenario 10: program crossing a cache-line boundary");
        do_reset();
        pcs = new[7];
        masks = new[7];
        for (int i = 0; i < 6; i++) begin
            load_word(32'hA070 + 4*i, r_type(OP_ADD, 5'd2, 5'd3, 5'(i + 1)));
            pcs[i] = 32'hA070 + 4*i;
            masks[i] = 32'hFFFF_FFFF;
        end
        load_word(32'hA088, DONE_INSTR);
        pcs[6] = 32'hA088;
        masks[6] = 32'h0;
        launch_kernel(1, 32'hA070);
        wait_kernel_done("scenario10");
        check_warp_trace("scenario10", 3'd0, pcs, masks);
    endtask

    // 11. A second kernel after the first one finished
    task automatic scenario11_relaunch();
        $display("\nScenario 11: relaunch after kernel_done");
        do_reset();
        load_word(32'hB000, r_type(OP_ADD, 5'd2, 5'd3, 5'd1));
        load_word(32'hB004, DONE_INSTR);
        load_word(32'hB100, r_type(OP_ADD, 5'd2, 5'd3, 5'd2));
        load_word(32'hB104, r_type(OP_ADD, 5'd2, 5'd3, 5'd3));
        load_word(32'hB108, DONE_INSTR);
        launch_kernel(2, 32'hB000);
        wait_kernel_done("scenario11 (first kernel)");
        trace.delete();
        launch_kernel(3, 32'hB100);
        tick();
        check(!kernel_done, "scenario11: kernel_start clears kernel_done");
        wait_kernel_done("scenario11 (second kernel)");
        for (int w = 0; w < 3; w++)
            check_warp_trace("scenario11", w[2:0],
                '{32'hB100, 32'hB104, 32'hB108}, '{32'hFFFF_FFFF, 32'hFFFF_FFFF, 32'h0});
    endtask

    initial begin
        scenario1_tag_alignment();
        scenario2_buffer_full_hold();
        scenario3_multi_warp();
        scenario4_decode_formats();
        scenario5_masking();
        scenario6_uniform_branch();
        scenario7_divergent_if();
        scenario8_divergent_if_else();
        scenario9_slow_icache();
        scenario10_line_crossing();
        scenario11_relaunch();

        $display("\n----------------------------------------------------");
        $display("checks=%0d errors=%0d", checks, errors);
        if (errors == 0) $display("ALL SCENARIOS PASSED");
        else $fatal(1, "SCENARIOS FAILED");
        $finish;
    end

    initial begin
        #2_000_000;
        $fatal(1, "[FAIL] testbench timeout");
    end
endmodule
