`timescale 1ns/1ps
module mask_split_unit_tb;
    logic clk;
    logic reset;
    logic [1:0] sub_warp_cycle;
    logic sub_warp_valid;
    logic buffer_full;
    logic is_branch;
    logic [7:0] is_valid;
    logic [7:0] branch_taken_bits;
    logic [31:0] branch_pc;
    logic [7:0] push2_pc_recon_offset;
    logic [31:0] pc_taken;
    logic [9:0] reg_bank_addr;

    logic [31:0] push2_mask_taken;
    logic [31:0] push2_mask_not_taken;
    logic [31:0] push2_pc_taken;
    logic [31:0] push2_pc_fallthrough;
    logic [31:0] push2_recon_pc;
    logic push2_skip_taken, push2_skip_not_taken;
    logic [7:0] recon_push2_valid;

    int errors = 0;

    mask_split_unit DUT (
        .clk(clk),
        .reset(reset),
        .sub_warp_cycle(sub_warp_cycle),
        .sub_warp_valid(sub_warp_valid),
        .buffer_full(buffer_full),
        .is_branch(is_branch),
        .is_valid(is_valid),
        .branch_taken_bits(branch_taken_bits),
        .branch_pc(branch_pc),
        .push2_pc_recon_offset(push2_pc_recon_offset),
        .pc_taken(pc_taken),
        .reg_bank_addr(reg_bank_addr),
        .push2_mask_taken(push2_mask_taken),
        .push2_mask_not_taken(push2_mask_not_taken),
        .push2_pc_taken(push2_pc_taken),
        .push2_pc_fallthrough(push2_pc_fallthrough),
        .push2_recon_pc(push2_recon_pc),
        .push2_skip_taken(push2_skip_taken),
        .push2_skip_not_taken(push2_skip_not_taken),
        .recon_push2_valid(recon_push2_valid)
    );

    always #5 clk = ~clk;

    task automatic check(input logic [31:0] actual, expected);
        if (actual !== expected) begin
            $error("FAIL | expected = %0h actual = %0h", expected, actual);
            errors++;
        end else begin
            $display("PASS | %0h", actual);
        end
    endtask

    // Drive one sub-warp of a branch instruction. reg_bank_addr = {warp, sub_warp, dst}
    task automatic drive_sub_warp(
        input logic [2:0] warp,
        input logic [1:0] cycle,
        input logic [7:0] valid_lanes,
        input logic [7:0] taken_lanes
    );
        @(negedge clk);
        is_branch = 1'b1;
        sub_warp_valid = 1'b1;
        sub_warp_cycle = cycle;
        reg_bank_addr = {warp, cycle, 5'd0};
        is_valid = valid_lanes;
        branch_taken_bits = taken_lanes;
    endtask

    task automatic idle();
        @(negedge clk);
        is_branch = 1'b0;
        sub_warp_valid = 1'b0;
        is_valid = 8'b0;
        branch_taken_bits = 8'b0;
    endtask

    initial begin
        clk = 0;
        reset = 1;
        sub_warp_cycle = 2'd0;
        sub_warp_valid = 1'b0;
        buffer_full = 1'b0;
        is_branch = 1'b0;
        is_valid = 8'b0;
        branch_taken_bits = 8'b0;
        branch_pc = 32'b0;
        push2_pc_recon_offset = 8'b0;
        pc_taken = 32'b0;
        reg_bank_addr = 10'b0;

        repeat (2) @(negedge clk);
        reset = 0;

        // Test 1: all lanes active, warp 0
        $display("Test 1: full warp, masks accumulate over 4 sub-warps");
        branch_pc = 32'h0000_1000;
        pc_taken = 32'h0000_2000;
        push2_pc_recon_offset = 8'd16;

        drive_sub_warp(3'd0, 2'd0, 8'hFF, 8'h01);
        drive_sub_warp(3'd0, 2'd1, 8'hFF, 8'h02);
        drive_sub_warp(3'd0, 2'd2, 8'hFF, 8'h03);
        drive_sub_warp(3'd0, 2'd3, 8'hFF, 8'h04);
        @(posedge clk); #1;

        // byte 3 of the masks lands the same cycle as recon_push2_valid
        $display("push2 valid for warp 0 with all 4 mask bytes in place");
        check(recon_push2_valid, 8'b0000_0001);
        check(push2_mask_taken, 32'h0403_0201);
        check(push2_mask_not_taken, ~32'h0403_0201);
        check(push2_pc_taken, 32'h0000_2000);
        check(push2_pc_fallthrough, 32'h0000_1004);
        check(push2_recon_pc, 32'h0000_1010);
        $display("both paths have threads and distinct PCs => neither skipped");
        check(push2_skip_taken, 1'b0);
        check(push2_skip_not_taken, 1'b0);

        idle();
        @(posedge clk); #1;
        $display("push2 valid is a single-cycle pulse");
        check(recon_push2_valid, 8'b0);

        // Test 2: inactive lanes belong to neither mask, routed to warp 5
        $display("\nTest 2: inactive lanes excluded from both masks, routed to warp 5");
        branch_pc = 32'h0000_3000;
        pc_taken = 32'h0000_3020;
        push2_pc_recon_offset = 8'd40;

        drive_sub_warp(3'd5, 2'd0, 8'h0F, 8'hFF); // lanes 4-7 inactive
        drive_sub_warp(3'd5, 2'd1, 8'h00, 8'hFF); // whole sub-warp inactive
        drive_sub_warp(3'd5, 2'd2, 8'hFF, 8'hAA);
        drive_sub_warp(3'd5, 2'd3, 8'hF0, 8'h00); // lanes 0-3 inactive
        @(posedge clk); #1;

        check(recon_push2_valid, 8'b0010_0000);
        check(push2_mask_taken, 32'h00_AA_00_0F);
        check(push2_mask_not_taken, 32'hF0_55_00_00);
        check(push2_recon_pc, 32'h0000_3028);
        idle();

        // Test 3: buffer_full in the middle of a branch holds the masks
        $display("\nTest 3: buffer_full stall mid-branch");
        branch_pc = 32'h0000_4000;
        pc_taken = 32'h0000_4040;
        push2_pc_recon_offset = 8'd8;

        drive_sub_warp(3'd2, 2'd0, 8'hFF, 8'h11);
        drive_sub_warp(3'd2, 2'd1, 8'hFF, 8'h22);

        // ALU holds sub-warp 2's outputs while the buffer is full
        drive_sub_warp(3'd2, 2'd2, 8'hFF, 8'h33);
        buffer_full = 1'b1;
        repeat (3) @(posedge clk);
        #1;
        $display("no push2 while stalled");
        check(recon_push2_valid, 8'b0);
        // sub-warp 2 was never captured, so byte 2 still holds test 2's value
        check(push2_mask_taken[15:0], 16'h2211);
        check(push2_mask_taken[23:16], 8'hAA);

        @(negedge clk);
        buffer_full = 1'b0;
        drive_sub_warp(3'd2, 2'd3, 8'hFF, 8'h44);
        @(posedge clk); #1;
        $display("push2 fires after stall with all 4 bytes");
        check(recon_push2_valid, 8'b0000_0100);
        check(push2_mask_taken, 32'h4433_2211);
        check(push2_recon_pc, 32'h0000_4008);
        check(push2_skip_taken, 1'b0);
        check(push2_skip_not_taken, 1'b0);
        idle();

        // Test 4: a non-branch instruction never pushes
        $display("\nTest 4: non-branch instruction does not push");
        for (int c = 0; c < 4; c++) begin
            drive_sub_warp(3'd1, c[1:0], 8'hFF, 8'hFF);
            is_branch = 1'b0;
        end
        @(posedge clk); #1;
        check(recon_push2_valid, 8'b0);
        idle();

        // Test 5: sub_warp_valid low at cycle 3 means no push
        $display("\nTest 5: invalid sub-warp 3 does not push");
        for (int c = 0; c < 4; c++)
            drive_sub_warp(3'd3, c[1:0], 8'hFF, 8'h0F);
        sub_warp_valid = 1'b0;
        @(posedge clk); #1;
        check(recon_push2_valid, 8'b0);
        idle();

        // Test 6: skip flags from PCs equal to the recon PC
        $display("\nTest 6: offset 4 => fall-through is the recon PC, not-taken skipped");
        branch_pc = 32'h0000_5000;
        pc_taken = 32'h0000_5040;
        push2_pc_recon_offset = 8'd4;
        for (int c = 0; c < 4; c++)
            drive_sub_warp(3'd4, c[1:0], 8'hFF, 8'h0F);
        @(posedge clk); #1;
        check(recon_push2_valid, 8'b0001_0000);
        check(push2_pc_fallthrough, push2_recon_pc);
        check(push2_skip_taken, 1'b0);
        check(push2_skip_not_taken, 1'b1);
        idle();

        $display("\nTest 6b: branch target is the recon PC => taken skipped");
        branch_pc = 32'h0000_6000;
        pc_taken = 32'h0000_6010;
        push2_pc_recon_offset = 8'd16;
        for (int c = 0; c < 4; c++)
            drive_sub_warp(3'd4, c[1:0], 8'hFF, 8'h0F);
        @(posedge clk); #1;
        check(push2_pc_taken, push2_recon_pc);
        check(push2_skip_taken, 1'b1);
        check(push2_skip_not_taken, 1'b0);
        idle();

        // Test 7: skip flags from empty masks, set by the last sub-warp's byte
        $display("\nTest 7: every active lane taken => not-taken mask empty, skipped");
        branch_pc = 32'h0000_7000;
        pc_taken = 32'h0000_7080;
        push2_pc_recon_offset = 8'd32;
        drive_sub_warp(3'd6, 2'd0, 8'hFF, 8'hFF);
        drive_sub_warp(3'd6, 2'd1, 8'hFF, 8'hFF);
        drive_sub_warp(3'd6, 2'd2, 8'hFF, 8'hFF);
        drive_sub_warp(3'd6, 2'd3, 8'hFF, 8'hFF);
        @(posedge clk); #1;
        check(push2_mask_not_taken, 32'h0);
        check(push2_skip_taken, 1'b0);
        check(push2_skip_not_taken, 1'b1);
        idle();

        $display("\nTest 7b: no lane taken, only the last byte has active lanes => taken skipped");
        drive_sub_warp(3'd6, 2'd0, 8'h00, 8'h00);
        drive_sub_warp(3'd6, 2'd1, 8'h00, 8'h00);
        drive_sub_warp(3'd6, 2'd2, 8'h00, 8'h00);
        drive_sub_warp(3'd6, 2'd3, 8'h80, 8'h00);
        @(posedge clk); #1;
        check(push2_mask_taken, 32'h0);
        check(push2_mask_not_taken, 32'h8000_0000);
        check(push2_skip_taken, 1'b1);
        check(push2_skip_not_taken, 1'b0);
        idle();

        if (errors == 0)
            $display("\nALL TESTS PASSED");
        else
            $fatal(1, "\n%0d TEST(S) FAILED", errors);

        $finish;
    end
endmodule
