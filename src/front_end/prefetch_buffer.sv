`default_nettype none
`timescale 1ns/1ps
module prefetch_buffer #(
    parameter int NUM_WARPS = 8,
    parameter int DEPTH_PER_WARP = 4, // fixed at 4: one 128-bit fill holds 4 instructions
    parameter int WARP_W = $clog2(NUM_WARPS),
    parameter [2:0] BRANCH = 3'b011,
    parameter [5:0] DONE = 6'b111111
) (
    input logic clk,
    input logic reset,
    input logic kernel_launch,

    // fill data from fetch_stage when next_warp's buffer is empty
    input logic [WARP_W-1:0] next_warp,
    input logic fill_valid,
    input logic [127:0] fill_data, // 4 x 32b words
    input logic [127:0] fill_pcs, // 4 x 32b PCs
    input logic [31:0] fill_recon_pc, // used to not fetch past recon_pc
    input logic consume,

    output logic [31:0] instr,
    output logic [31:0] instr_pc,
    output logic fetch_valid
);
    logic [31:0] data [0:NUM_WARPS*DEPTH_PER_WARP-1];
    logic [31:0] pcs  [0:NUM_WARPS*DEPTH_PER_WARP-1];
    logic [NUM_WARPS*DEPTH_PER_WARP-1:0] valid;

    localparam int IDX_W = $clog2(NUM_WARPS*DEPTH_PER_WARP);
    logic [IDX_W-1:0] next_warp_base;
    assign next_warp_base = IDX_W'(next_warp) * IDX_W'(DEPTH_PER_WARP);

    always_ff @(posedge clk) begin
        if (reset || kernel_launch) begin
            valid <= 'b0;
            for (int i = 0; i < NUM_WARPS*DEPTH_PER_WARP; ++i) begin
                data[i] <= 'b0;
                pcs[i] <= 'b0;
            end
        end else if (fill_valid) begin
            for (int j = 0; j < 4; ++j) begin
                data[next_warp_base + j] <= fill_data[j*32 +: 32];
                pcs[next_warp_base + j] <= fill_pcs [j*32 +: 32];
                // Same 128B line as fill_pcs[0] iff its word offset + j doesn't pass word 31
                valid[next_warp_base + j] <= (fill_pcs[j*32 +: 32] < fill_recon_pc) &&
                                             (fill_pcs[6:2] <= 5'(31 - j));
            end
        end else if (consume) begin
            logic is_branch_or_done;
            is_branch_or_done = (data[next_warp_base][31:29] == BRANCH || data[next_warp_base][31:26] == DONE);

            for (int k = 0; k < DEPTH_PER_WARP - 1; ++k) begin
                data[next_warp_base + k] <= data[next_warp_base + k + 1];
                pcs[next_warp_base + k] <= pcs[next_warp_base + k + 1];
                valid[next_warp_base + k] <= is_branch_or_done ? 1'b0 : valid[next_warp_base + k + 1];
            end

            valid[next_warp_base + DEPTH_PER_WARP - 1] <= 1'b0;
        end
    end

    assign instr = data[next_warp_base]; // next_warp * DEPTH_PER_WARP
    assign instr_pc = pcs[next_warp_base];
    assign fetch_valid = valid[next_warp_base];
endmodule
