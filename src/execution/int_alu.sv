`default_nettype none
`timescale 1ns/1ps
module int_alu (
    input logic clk,
    input logic reset,
    input logic valid_in,
    input logic buffer_full,
    input logic sub_warp_valid_in,
    input logic [1:0] sub_warp_cycle_in,
    input logic [9:0] reg_bank_addr_in,

    input logic [5:0] opcode,
    input logic [31:0] src1,
    input logic [31:0] src2,
    
    // PC and offset for branch instructions
    input logic [31:0] instr_pc,
    input logic [15:0] offset,
    output logic valid_out,
    output logic branch_taken,
    output logic [31:0] result,

    output logic mul_valid,
    output logic [31:0] mul_result,
    output logic [9:0] mul_addr,

    // Output warp_id, thread_slot, and dest_register for writeback_arbiter
    // This is simply the input passed to the output
    output logic sub_warp_valid_out,
    output logic [1:0] sub_warp_cycle_out,
    output logic [9:0] reg_bank_addr_out,

    // Branch information
    output logic is_branch,
    output logic is_load,
    output logic is_store,
    output logic [31:0] branch_pc_out,
    output logic [7:0] push2_pc_recon_offset,
    output logic [31:0] src2_out // for store instruction
);
    // Define parameters for instruction types
    localparam R_TYPE = 3'b000, I_TYPE = 3'b001, BRANCH = 3'b011;

    // Define parameters for specific instructions
    // R-type instructions
    localparam [2:0] NOP = 3'b000, ADD  = 3'b001, SUB  = 3'b010;
    localparam [2:0] AND  = 3'b011, OR  = 3'b100, SLT = 3'b101;
    localparam [2:0] MUL = 3'b110;

    // I-type instructions
    localparam [2:0] LUI = 3'b000, ADDI = 3'b001, SUBI = 3'b010;
    localparam [2:0] ANDI = 3'b011, ORI = 3'b100, SLTI = 3'b101;
    localparam [2:0] LOAD = 3'b110, STORE = 3'b111;

    // Branch instructions
    localparam [2:0] BEQ = 3'b000, BNE = 3'b001;
    localparam [2:0] BLT = 3'b010, BGE = 3'b011;

    always_ff @(posedge clk) begin
        if (reset) begin
            branch_taken <= 1'b0;
            result <= 32'b0;
        end else if (!buffer_full) begin
            branch_taken <= 1'b0;
            case (opcode[5:3]) // case by opcode
                R_TYPE : case (opcode[2:0]) // case by sub opcode
                    NOP : result <= 32'b0;
                    ADD : result <= src1 + src2;
                    SUB : result <= src1 - src2;
                    AND : result <= src1 & src2;
                    OR  : result <= src1 | src2;
                    SLT : result <= $signed(src1) < $signed(src2);
                    default: result <= 32'b0;
                endcase

                I_TYPE : case (opcode[2:0])
                    LUI  : result <= {src1[15:0], 16'b0};
                    LOAD, STORE,
                    ADDI : result <= src1 + offset;
                    SUBI : result <= src1 - offset;
                    ANDI : result <= src1 & offset;
                    ORI  : result <= src1 | offset;
                    SLTI : result <= $signed(src1) < $signed(offset);
                    default : result <= 32'b0;
                endcase

                BRANCH : begin
                    result <= instr_pc + {{24{offset[15]}}, offset[15:8]};
                    case (opcode[2:0])
                        BEQ : branch_taken <= (src1 == src2);
                        BNE : branch_taken <= (src1 != src2);
                        BLT : branch_taken <= ($signed(src1) < $signed(src2));
                        BGE : branch_taken <= ($signed(src1) >= $signed(src2));
                        default : ; // do nothing
                    endcase
                end
                
                default : result <= 32'b0;
            endcase
        end
    end

    logic [31:0] a_low_b_low;
    logic [15:0] a_low_b_high;
    logic [15:0] a_high_b_low;
    logic is_mul;

    logic [9:0] mul_addr_temp;

    // Operand registers, no reset so they pack into the DSP AREG/BREG.
    // Keeps the slow BRAM read (reg_file) and the multiply in separate cycles.
    logic [31:0] mul_a, mul_b;
    logic is_mul_in;
    logic [9:0] mul_addr_in;

    always_ff @(posedge clk) begin
        if (!buffer_full) begin
            mul_a <= src1;
            mul_b <= src2;
            mul_addr_in <= reg_bank_addr_in;
        end
    end

    logic [15:0] upper_sum;
    assign upper_sum = a_low_b_low[31:16] + a_low_b_high + a_high_b_low;

    always_ff @(posedge clk) begin
        if (reset) begin
            a_low_b_low <= '0;
            a_low_b_high <= '0;
            a_high_b_low <= '0;
            mul_result <= '0;

            mul_addr_temp <= '0;
            mul_addr <= '0;

            is_mul_in <= 1'b0;
            is_mul <= 1'b0;
            mul_valid <= 1'b0;
        end else if (!buffer_full) begin
            is_mul_in <= valid_in && (opcode == {R_TYPE, MUL});

            // partial products
            a_low_b_low <= mul_a[15:0] * mul_b[15:0];
            a_low_b_high <= mul_a[15:0] * mul_b[31:16];
            a_high_b_low <= mul_a[31:16] * mul_b[15:0];
            is_mul <= is_mul_in;
            mul_addr_temp <= mul_addr_in;

            mul_result <= {upper_sum, a_low_b_low[15:0]};
            mul_valid <= is_mul;
            mul_addr <= mul_addr_temp;
        end
    end

    always_ff @(posedge clk) begin
        if (reset) begin
            sub_warp_cycle_out <= 2'b0;
            sub_warp_valid_out <= 1'b0;
            is_branch <= 1'b0;
            is_load <= 1'b0;
            is_store <= 1'b0;
            valid_out <= 1'b0;
            branch_pc_out <= 32'b0;
            push2_pc_recon_offset <= 8'b0;
            src2_out <= 32'b0;
            reg_bank_addr_out <= 10'b0;
        end else if (!buffer_full) begin
            sub_warp_cycle_out <= sub_warp_cycle_in;
            sub_warp_valid_out <= sub_warp_valid_in;
            is_branch <= (opcode[5:3] == BRANCH);
            is_load <= (opcode == {I_TYPE, LOAD});
            is_store <= (opcode == {I_TYPE, STORE});
            valid_out <= valid_in && (opcode != {R_TYPE, MUL});
            branch_pc_out <= instr_pc;
            push2_pc_recon_offset <= offset[7:0];
            src2_out <= src2;
            reg_bank_addr_out <= reg_bank_addr_in;
        end
    end
endmodule
