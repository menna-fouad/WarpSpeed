`timescale 1ns/1ps
module int_alu_tb;
    logic clk, reset;
    logic valid_in, buffer_full;
    logic sub_warp_valid_in;
    logic [1:0] sub_warp_cycle_in;
    logic [9:0] reg_bank_addr_in;
    logic [5:0] opcode;
    logic [31:0] src1, src2;
    logic [31:0] instr_pc;
    logic [15:0] offset;

    logic valid_out;
    logic branch_taken;
    logic [31:0] result;
    logic sub_warp_valid_out;
    logic [1:0] sub_warp_cycle_out;
    logic [9:0] reg_bank_addr_out;
    logic is_branch, is_load, is_store;
    logic [31:0] branch_pc_out;
    logic [7:0] push2_pc_recon_offset;
    logic [31:0] src2_out;
    logic mul_valid;
    logic [31:0] mul_result;
    logic [9:0] mul_addr;

    int errors = 0;

    int_alu DUT (
        .clk(clk),
        .reset(reset),
        .valid_in(valid_in),
        .buffer_full(buffer_full),
        .sub_warp_valid_in(sub_warp_valid_in),
        .sub_warp_cycle_in(sub_warp_cycle_in),
        .reg_bank_addr_in(reg_bank_addr_in),
        .opcode(opcode),
        .src1(src1),
        .src2(src2),
        .instr_pc(instr_pc),
        .offset(offset),
        .valid_out(valid_out),
        .branch_taken(branch_taken),
        .result(result),
        .mul_valid(mul_valid),
        .mul_result(mul_result),
        .mul_addr(mul_addr),
        .sub_warp_valid_out(sub_warp_valid_out),
        .sub_warp_cycle_out(sub_warp_cycle_out),
        .reg_bank_addr_out(reg_bank_addr_out),
        .is_branch(is_branch),
        .is_load(is_load),
        .is_store(is_store),
        .branch_pc_out(branch_pc_out),
        .push2_pc_recon_offset(push2_pc_recon_offset),
        .src2_out(src2_out)
    );

    always #5 clk = ~clk;

    // Opcode encodings: {format[2:0], suboptype[2:0]}
    localparam [5:0] NOP = 6'b000000, ADD = 6'b000001, SUB = 6'b000010;
    localparam [5:0] AND = 6'b000011, OR  = 6'b000100, SLT = 6'b000101;
    localparam [5:0] MUL = 6'b000110;

    // I-type instructions
    localparam [5:0] LUI = 6'b001000, ADDI = 6'b001001, SUBI = 6'b001010;
    localparam [5:0] ANDI = 6'b001011, ORI = 6'b001100, SLTI = 6'b001101;
    localparam [5:0] LOAD = 6'b001110, STORE = 6'b001111;

    // Branch instructions
    localparam [5:0] BEQ = 6'b011000, BNE = 6'b011001;
    localparam [5:0] BLT = 6'b011010, BGE = 6'b011011;

    task automatic check_result(input logic [31:0] actual, input logic [31:0] expected);
        if (actual !== expected) begin
            $error("FAIL | expected = %0h actual = %0h", expected, actual);
            errors++;
        end else begin
            $display("PASS | %0h", actual);
        end
    endtask

    task automatic check_bit(input logic actual, input logic expected);
        if (actual !== expected) begin
            $error("FAIL | expected = %0b actual = %0b", expected, actual);
            errors++;
        end else begin
            $display("PASS | %0b", actual);
        end
    endtask

    // The ALU is registered: drive inputs, then sample one cycle later
    task automatic drive(
        input logic [5:0] task_opcode,
        input logic [31:0] task_src1,
        input logic [31:0] task_src2,
        input logic [31:0] task_pc,
        input logic [15:0] task_offset
    );
        @(negedge clk);
        valid_in = 1'b1;
        opcode = task_opcode;
        src1 = task_src1;
        src2 = task_src2;
        instr_pc = task_pc;
        offset = task_offset;
        @(posedge clk);
        #1;
    endtask

    initial begin
        clk = 0;
        reset = 1;
        valid_in = 0;
        buffer_full = 0;
        sub_warp_valid_in = 0;
        sub_warp_cycle_in = 2'd0;
        opcode = 6'b0;
        src1 = 32'b0;
        src2 = 32'b0;
        instr_pc = 32'b0;
        offset = 16'b0;
        reg_bank_addr_in = 10'b0;

        repeat (2) @(posedge clk);
        #1;
        $display("reset: valid_out = 0, branch_taken = 0, result = 32'b0");
        check_bit(valid_out, 1'b0);
        check_bit(branch_taken, 1'b0);
        check_result(result, 32'b0);

        reset = 0;

        // R-type
        $display("\nNOP");
        drive(NOP, 32'd5, 32'd7, 32'b0, 16'b0);
        check_result(result, 32'b0);

        $display("ADD");
        drive(ADD, 32'd10, 32'd15, 32'b0, 16'b0);
        check_result(result, 32'd25);

        $display("SUB");
        drive(SUB, 32'd20, 32'd8, 32'b0, 16'b0);
        check_result(result, 32'd12);

        $display("AND");
        drive(AND, 32'hFF00FF00, 32'h0FF00FF0, 32'b0, 16'b0);
        check_result(result, 32'h0F000F00);

        $display("OR");
        drive(OR, 32'hF0F0F0F0, 32'h0F0F0F0F, 32'b0, 16'b0);
        check_result(result, 32'hFFFFFFFF);

        $display("SLT (true, false, signed negative < positive)");
        drive(SLT, 32'd3, 32'd5, 32'b0, 16'b0);
        check_result(result, 32'd1);
        drive(SLT, 32'd5, 32'd3, 32'b0, 16'b0);
        check_result(result, 32'd0);
        drive(SLT, -32'sd5, 32'd1, 32'b0, 16'b0);
        check_result(result, 32'd1);

        // MUL goes down its own 3-cycle pipeline (operand regs -> partial products -> sum)
        // and comes out on mul_valid/mul_result/mul_addr, not valid_out/result
        $display("MUL");
        reg_bank_addr_in = 10'h155;
        drive(MUL, 32'd6, 32'd7, 32'b0, 16'b0);
        valid_in = 1'b0;
        check_bit(valid_out, 1'b0);
        check_bit(mul_valid, 1'b0);
        @(posedge clk); #1;
        check_bit(mul_valid, 1'b0);
        @(posedge clk); #1;
        check_bit(mul_valid, 1'b1);
        check_result(mul_result, 32'd42);
        check_result(mul_addr, 10'h155);
        @(posedge clk); #1;
        check_bit(mul_valid, 1'b0);

        $display("MUL (upper partial products, low 32 bits kept)");
        drive(MUL, 32'h12345678, 32'h9ABCDEF0, 32'b0, 16'b0);
        valid_in = 1'b0;
        repeat (2) @(posedge clk);
        #1;
        check_bit(mul_valid, 1'b1);
        check_result(mul_result, 32'h242D2080);
        reg_bank_addr_in = 10'b0;

        // I-type: the second operand is the 16-bit immediate, not src2
        $display("\nLUI");
        drive(LUI, 32'h0000_ABCD, 32'b0, 32'b0, 16'b0);
        check_result(result, 32'hABCD_0000);

        $display("ADDI");
        drive(ADDI, 32'd100, 32'hDEAD, 32'b0, 16'd23);
        check_result(result, 32'd123);

        $display("SUBI");
        drive(SUBI, 32'd100, 32'hDEAD, 32'b0, 16'd23);
        check_result(result, 32'd77);

        $display("ANDI");
        drive(ANDI, 32'hFFFF_FFFF, 32'b0, 32'b0, 16'h00FF);
        check_result(result, 32'h0000_00FF);

        $display("ORI");
        drive(ORI, 32'h0000_0000, 32'b0, 32'b0, 16'h00FF);
        check_result(result, 32'h0000_00FF);

        $display("SLTI (true, false)");
        drive(SLTI, 32'd3, 32'b0, 32'b0, 16'd5);
        check_result(result, 32'd1);
        drive(SLTI, 32'd7, 32'b0, 32'b0, 16'd5);
        check_result(result, 32'd0);

        $display("LOAD: address = base + offset, is_load set, nothing else tagged");
        drive(LOAD, 32'h0000_1000, 32'b0, 32'b0, 16'd16);
        check_result(result, 32'h0000_1010);
        check_bit(is_load, 1'b1);
        check_bit(is_store, 1'b0);
        check_bit(is_branch, 1'b0);

        $display("STORE: address = base + offset, store data forwarded on src2_out");
        drive(STORE, 32'h0000_2000, 32'hCAFE_F00D, 32'b0, 16'd32);
        check_result(result, 32'h0000_2020);
        check_result(src2_out, 32'hCAFE_F00D);
        check_bit(is_store, 1'b1);
        check_bit(is_load, 1'b0);

        // Branch: offset[15:8] = target_offset (8b signed), offset[7:0] = reconv_offset
        $display("\nBRANCH target (positive target_offset = 8), recon offset passed through");
        drive(BEQ, 32'd5, 32'd5, 32'h0000_1000, {8'sd8, 8'd20});
        check_result(result, 32'h0000_1008);
        check_bit(branch_taken, 1'b1);
        check_bit(is_branch, 1'b1);
        check_result(branch_pc_out, 32'h0000_1000);
        check_result(push2_pc_recon_offset, 8'd20);

        $display("BRANCH target (negative target_offset = -4, sign-extended)");
        drive(BEQ, 32'd5, 32'd5, 32'h0000_1000, {8'hFC, 8'd0});
        check_result(result, 32'h0000_0FFC);

        $display("BEQ not taken");
        drive(BEQ, 32'd5, 32'd6, 32'h0000_1000, {8'sd8, 8'd0});
        check_bit(branch_taken, 1'b0);

        $display("BNE taken (not equal)");
        drive(BNE, 32'd5, 32'd6, 32'h0000_2000, {8'sd8, 8'd0});
        check_bit(branch_taken, 1'b1);
        $display("BNE not taken (equal)");
        drive(BNE, 32'd5, 32'd5, 32'h0000_2000, {8'sd8, 8'd0});
        check_bit(branch_taken, 1'b0);

        $display("BLT taken (signed)");
        drive(BLT, -32'sd3, 32'd1, 32'h0000_3000, {8'sd8, 8'd0});
        check_bit(branch_taken, 1'b1);
        $display("BLT not taken");
        drive(BLT, 32'd10, 32'd1, 32'h0000_3000, {8'sd8, 8'd0});
        check_bit(branch_taken, 1'b0);

        $display("BGE taken");
        drive(BGE, 32'd10, 32'd1, 32'h0000_4000, {8'sd8, 8'd0});
        check_bit(branch_taken, 1'b1);
        $display("BGE taken (equal)");
        drive(BGE, 32'd10, 32'd10, 32'h0000_4000, {8'sd8, 8'd0});
        check_bit(branch_taken, 1'b1);
        $display("BGE not taken");
        drive(BGE, 32'd1, 32'd10, 32'h0000_4000, {8'sd8, 8'd0});
        check_bit(branch_taken, 1'b0);

        $display("branch_taken clears on a non-branch instruction");
        drive(BEQ, 32'd1, 32'd1, 32'h0000_4000, {8'sd8, 8'd0});
        check_bit(branch_taken, 1'b1);
        drive(ADD, 32'd1, 32'd1, 32'b0, 16'b0);
        check_bit(branch_taken, 1'b0);
        check_bit(is_branch, 1'b0);

        // Pipeline metadata is registered alongside the result
        $display("\nvalid, sub-warp and reg_bank_addr registered through");
        @(negedge clk);
        reg_bank_addr_in = 10'b101_10_00110;
        sub_warp_valid_in = 1'b1;
        sub_warp_cycle_in = 2'd2;
        valid_in = 1'b1;
        @(posedge clk); #1;
        check_bit(valid_out, 1'b1);
        check_bit(sub_warp_valid_out, 1'b1);
        check_result(sub_warp_cycle_out, 2'd2);
        check_result(reg_bank_addr_out, 10'b101_10_00110);
        @(negedge clk);
        valid_in = 1'b0;
        @(posedge clk); #1;
        check_bit(valid_out, 1'b0);

        // buffer_full: ALU holds its current outputs
        $display("\nbuffer_full holds result and metadata");
        drive(ADD, 32'd1, 32'd2, 32'b0, 16'b0);
        check_result(result, 32'd3);
        @(negedge clk);
        buffer_full = 1'b1;
        opcode = SUB;
        src1 = 32'd50;
        src2 = 32'd8;
        reg_bank_addr_in = 10'd7;
        valid_in = 1'b0;
        repeat (3) @(posedge clk);
        #1;
        check_result(result, 32'd3);
        check_bit(valid_out, 1'b1);
        check_result(reg_bank_addr_out, 10'b101_10_00110);
        @(negedge clk);
        buffer_full = 1'b0;
        valid_in = 1'b1;
        @(posedge clk); #1;
        $display("released: next instruction goes through");
        check_result(result, 32'd42);
        check_result(reg_bank_addr_out, 10'd7);

        if (errors == 0)
            $display("\nALL TESTS PASSED");
        else
            $fatal(1, "\n%0d TEST(S) FAILED", errors);

        $finish;
    end
endmodule
