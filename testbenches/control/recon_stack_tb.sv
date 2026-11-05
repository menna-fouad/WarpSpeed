`timescale 1ns/1ps
module recon_stack_tb;
    logic clk, reset;
    logic init_push, push2_valid;
    logic [31:0] init_mask, init_pc, init_recon_pc;
    logic [31:0] push2_mask_taken, push2_mask_not_taken;
    logic [31:0] push2_pc_taken, push2_pc_fallthrough, push2_recon_pc;
    logic push2_skip_taken, push2_skip_not_taken;

    logic current_pc_valid;
    logic [31:0] current_pc;

    logic [31:0] top_mask, top_pc, top_recon_pc;
    logic stack_empty, stack_full, push2_done;

    int errors = 0;

    recon_stack #(.DEPTH(8)) DUT (
        .clk(clk),
        .reset(reset),
        .init_push(init_push),
        .init_mask(init_mask),
        .init_pc(init_pc),
        .init_recon_pc(init_recon_pc),
        .push2_valid(push2_valid),
        .push2_mask_taken(push2_mask_taken),
        .push2_mask_not_taken(push2_mask_not_taken),
        .push2_pc_taken(push2_pc_taken),
        .push2_pc_fallthrough(push2_pc_fallthrough),
        .push2_recon_pc(push2_recon_pc),
        .push2_skip_taken(push2_skip_taken),
        .push2_skip_not_taken(push2_skip_not_taken),
        .current_pc_valid(current_pc_valid),
        .current_pc(current_pc),
        .top_mask(top_mask),
        .top_pc(top_pc),
        .top_recon_pc(top_recon_pc),
        .stack_empty(stack_empty),
        .stack_full(stack_full),
        .push2_done(push2_done)
    );

    always #5 clk = ~clk;

    task automatic do_reset();
        reset = 1'b1;
        init_push = 1'b0; push2_valid = 1'b0; current_pc_valid = 1'b0;
        init_mask = '0; init_pc = '0; init_recon_pc = '0;
        push2_mask_taken = '0; push2_mask_not_taken = '0;
        push2_pc_taken = '0; push2_pc_fallthrough = '0; push2_recon_pc = '0;
        push2_skip_taken = 1'b1; push2_skip_not_taken = 1'b1;
        current_pc = '0;
        @(negedge clk);
        @(negedge clk);
        reset = 1'b0;
    endtask

    // Fetch stage reports the next sequential PC of the warp
    task automatic set_pc(input [31:0] pc);
        @(negedge clk);
        current_pc = pc; current_pc_valid = 1'b1;
        @(negedge clk);
        current_pc_valid = 1'b0;
    endtask

    task automatic check_top(input [31:0] exp_mask, exp_pc, exp_recon_pc);
        if (top_mask !== exp_mask || top_pc !== exp_pc || top_recon_pc !== exp_recon_pc) begin
            $error("FAIL | got mask = %h pc = %h recon = %h | expected mask = %h pc = %h recon = %h",
                    top_mask, top_pc, top_recon_pc, exp_mask, exp_pc, exp_recon_pc);
            errors++;
        end else begin
            $display("PASS | mask = %h pc = %h recon = %h", top_mask, top_pc, top_recon_pc);
        end
    endtask

    task automatic check_flags(input logic exp_empty, exp_full);
        if (stack_empty !== exp_empty || stack_full !== exp_full) begin
            $error("FAIL | got empty = %b full = %b, expected empty = %b full = %b",
                    stack_empty, stack_full, exp_empty, exp_full);
            errors++;
        end else begin
            $display("PASS | empty = %b full = %b", stack_empty, stack_full);
        end
    endtask

    task automatic check_depth(input int exp_depth);
        if (DUT.next_free !== exp_depth) begin
            $error("FAIL | got depth = %0d, expected depth = %0d", DUT.next_free, exp_depth);
            errors++;
        end else begin
            $display("PASS | depth = %0d", DUT.next_free);
        end
    endtask

    task automatic check_push2_done(input logic exp_done);
        if (push2_done !== exp_done) begin
            $error("FAIL | got push2_done = %b, expected push2_done = %b", push2_done, exp_done);
            errors++;
        end else begin
            $display("PASS | push2_done = %b", push2_done);
        end
    endtask

    task automatic kernel_launch(input [31:0] mask, pc, recon_pc);
        @(negedge clk);
        init_push = 1'b1; init_mask = mask; init_pc = pc; init_recon_pc = recon_pc;
        @(negedge clk);
        init_push = 1'b0;
    endtask

    task automatic push(input [31:0] mask_taken, mask_not_taken, pc_taken, pc_fallthrough, recon_pc,
                        input logic exp_push2_done);
        @(negedge clk);
        push2_valid = 1'b1;
        push2_mask_taken = mask_taken; push2_mask_not_taken = mask_not_taken;
        push2_pc_taken = pc_taken; push2_pc_fallthrough = pc_fallthrough;
        push2_recon_pc = recon_pc;
        // what mask_split_unit registers alongside the push2 data
        push2_skip_taken = (pc_taken == recon_pc) || ~|mask_taken;
        push2_skip_not_taken = (pc_fallthrough == recon_pc) || ~|mask_not_taken;

        #1; check_push2_done(exp_push2_done);

        @(negedge clk);
        push2_valid = 1'b0;
    endtask

    initial begin
        clk = 0;
        do_reset();

        $display("Empty after reset");
        check_flags(1'b1, 1'b0);

        // 1. Kernel launch
        $display("\nKernel Launch");
        kernel_launch(32'hFFFF_FFFF, 32'h0000_1000, 32'hFFFF_FFFF);
        check_top(32'hFFFF_FFFF, 32'h0000_1000, 32'hFFFF_FFFF);
        check_flags(1'b0, 1'b0);
        check_depth(1);

        // 2. Divergent branch at PC=0x1000, reconverge at 0x100C
        //    not-taken pushed first, taken on top. Parent PC becomes the recon PC
        $display("\nDivergent branch => taken entry on top");
        push(32'h0F0F_0F0F, 32'hF0F0_F0F0, 32'h0000_1008, 32'h0000_1004, 32'h0000_100C, 1'b1);
        check_top(32'h0F0F_0F0F, 32'h0000_1008, 32'h0000_100C);
        check_depth(3);

        // 3. Taken path reaches the recon PC => pop, revealing not-taken path
        $display("\nTaken path reaches reconvergence PC => not-taken path on top");
        set_pc(32'h0000_100C);
        check_top(32'hF0F0_F0F0, 32'h0000_1004, 32'h0000_100C);

        // 4. Non-reconvergent PC only advances the top PC
        $display("\nNon-reconvergent PC => no pop, top PC advances");
        set_pc(32'h0000_1008);
        check_top(32'hF0F0_F0F0, 32'h0000_1008, 32'h0000_100C);
        check_depth(2);

        // 5. Not-taken path reaches recon PC => back to parent, which resumes at recon PC
        $display("\nNot-taken path reaches reconvergence PC => parent resumes at recon PC");
        set_pc(32'h0000_100C);
        check_top(32'hFFFF_FFFF, 32'h0000_100C, 32'hFFFF_FFFF);
        check_depth(1);

        // 6. Nested divergence
        $display("\nNested divergence");
        set_pc(32'h0000_2000);
        push(32'h0000_FFFF, 32'hFFFF_0000, 32'h0000_2010, 32'h0000_2004, 32'h0000_2040, 1'b1); // outer
        check_top(32'h0000_FFFF, 32'h0000_2010, 32'h0000_2040);
        set_pc(32'h0000_2014);
        push(32'h0000_00FF, 32'h0000_FF00, 32'h0000_2020, 32'h0000_2018, 32'h0000_2030, 1'b1); // inner
        check_top(32'h0000_00FF, 32'h0000_2020, 32'h0000_2030);
        check_depth(5);

        $display("inner taken done => inner not-taken");
        set_pc(32'h0000_2030);
        check_top(32'h0000_FF00, 32'h0000_2018, 32'h0000_2030);
        $display("inner not-taken done => outer taken resumes at inner recon PC");
        set_pc(32'h0000_2030);
        check_top(32'h0000_FFFF, 32'h0000_2030, 32'h0000_2040);
        $display("outer taken done => outer not-taken");
        set_pc(32'h0000_2040);
        check_top(32'hFFFF_0000, 32'h0000_2004, 32'h0000_2040);
        $display("outer not-taken done => parent");
        set_pc(32'h0000_2040);
        check_top(32'hFFFF_FFFF, 32'h0000_2040, 32'hFFFF_FFFF);
        check_depth(1);

        // 7. Uniform branch (all threads taken) => only one entry pushed
        $display("\nUniform taken branch => only taken entry pushed");
        push(32'hFFFF_FFFF, 32'h0000_0000, 32'h0000_3010, 32'h0000_3004, 32'h0000_3020, 1'b1);
        check_top(32'hFFFF_FFFF, 32'h0000_3010, 32'h0000_3020);
        check_depth(2);
        set_pc(32'h0000_3020);
        check_top(32'hFFFF_FFFF, 32'h0000_3020, 32'hFFFF_FFFF);
        check_depth(1);

        // 8. Bodyless else: fallthrough == recon PC => only the taken entry pushed
        $display("\nFallthrough == recon PC => only taken entry pushed");
        push(32'hAAAA_AAAA, 32'h5555_5555, 32'h0000_4004, 32'h0000_4008, 32'h0000_4008, 1'b1);
        check_top(32'hAAAA_AAAA, 32'h0000_4004, 32'h0000_4008);
        check_depth(2);
        set_pc(32'h0000_4008);
        check_top(32'hFFFF_FFFF, 32'h0000_4008, 32'hFFFF_FFFF);

        // 9. Neither path has a body (both skipped) => nothing pushed, parent jumps to recon PC
        $display("\nBoth paths skipped => nothing pushed, parent PC = recon PC");
        push(32'hFFFF_FFFF, 32'h0000_0000, 32'h0000_5010, 32'h0000_5004, 32'h0000_5010, 1'b1);
        check_top(32'hFFFF_FFFF, 32'h0000_5010, 32'hFFFF_FFFF);
        check_depth(1);

        // 10. Both skipped and recon PC matches the top entry's recon PC => entry popped
        $display("\nBoth paths skipped inside a divergent entry with the same recon PC => pop");
        push(32'h0000_FFFF, 32'hFFFF_0000, 32'h0000_6010, 32'h0000_6004, 32'h0000_6040, 1'b1);
        check_depth(3);
        push(32'h0000_FFFF, 32'h0000_0000, 32'h0000_6040, 32'h0000_6014, 32'h0000_6040, 1'b1);
        check_top(32'hFFFF_0000, 32'h0000_6004, 32'h0000_6040);
        check_depth(2);
        set_pc(32'h0000_6040);
        check_depth(1);

        // 11. Overflow: 1 + 2 + 2 + 2 = 7 entries = full, next push2 rejected
        $display("\nstack_full after three nested divergent branches");
        for (int i = 0; i < 3; i++)
            push(32'h0000_0001, 32'h0000_0002, 32'h0000_700C + i*16, 32'h0000_7004 + i*16, 32'h0000_7F00 - i*16, 1'b1);
        check_flags(1'b0, 1'b1);
        check_depth(7);
        check_top(32'h0000_0001, 32'h0000_702C, 32'h0000_7EE0);

        $display("push2 while full is rejected and the stack is unchanged");
        push(32'hAAAA_AAAA, 32'h5555_5555, 32'hDEAD_0004, 32'hDEAD_0008, 32'hDEAD_0010, 1'b0);
        check_flags(1'b0, 1'b1);
        check_depth(7);
        check_top(32'h0000_0001, 32'h0000_702C, 32'h0000_7EE0);

        // 12. A new kernel launch resets the stack to a single entry
        $display("\nKernel launch resets the stack");
        kernel_launch(32'h0000_FFFF, 32'h0000_0000, 32'hFFFF_FFFF);
        check_top(32'h0000_FFFF, 32'h0000_0000, 32'hFFFF_FFFF);
        check_flags(1'b0, 1'b0);
        check_depth(1);

        if (errors == 0)
            $display("\nALL TESTS PASSED");
        else
            $fatal(1, "\n%0d TEST(S) FAILED", errors);

        $finish;
    end
endmodule
