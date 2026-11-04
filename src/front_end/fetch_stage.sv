`default_nettype none
`timescale 1ns/1ps
module fetch_stage (
    input logic clk,
    input logic reset,
    input logic kernel_launch,
    input logic issue_valid, // valid (ready) warp issued
    input logic [2:0] next_warp,
    input logic [31:0] top_pc,
    input logic [31:0] recon_pc,

    input logic fetch_allowed,
    input logic consume, // calculated externally by
                         // (sub_warp_cycle == 2'd0) && sub_warp_valid
    
    input logic icache_ready,
    input logic icache_rsp_valid,
    input logic [127:0] icache_data,
    
    output logic icache_req_valid,
    output logic [31:0] icache_addr,

    output logic fetch_valid,
    output logic [31:0] instr,
    output logic [31:0] instr_pc,
    output logic seq_pc_valid,
    output logic [31:0] seq_next_pc
);
    logic [127:0] fill_pcs;
    logic [31:0] fill_recon_pc; // recon_pc latched with fill_pcs at request time
    logic fill_valid, req_outstanding;
    logic request_active;
    logic buf_valid;

    assign request_active = icache_req_valid || req_outstanding;

    always_ff @(posedge clk) begin
        if (reset) req_outstanding <= 1'b0;
        else if (icache_req_valid && icache_ready) req_outstanding <= 1'b1;
        else if (icache_rsp_valid) req_outstanding <= 1'b0;
    end

    always_comb begin
        if (reset || kernel_launch) begin
            icache_req_valid = 0;
            icache_addr = 32'h0000_0000;
        end else begin
            icache_req_valid = issue_valid && fetch_allowed && !buf_valid && !req_outstanding;
            icache_addr = top_pc;
        end
    end
    
    always_ff @(posedge clk) begin
        if (reset) begin
            fill_pcs <= 128'b0;
        end else if (icache_req_valid && icache_ready) begin
            for (int i = 0; i < 4; ++i)  begin
                // top_pc + 4 * i (top_pc, top_pc + 4, top_pc + 8, top_pc + 12)
                fill_pcs[i*32 +: 32] <= top_pc + (i << 2);
            end
            fill_recon_pc <= recon_pc;
        end
    end

    assign fill_valid = icache_rsp_valid && !reset;
    
    prefetch_buffer buffer (
        .clk(clk),
        .reset(reset),
        .kernel_launch(kernel_launch),
        .next_warp(next_warp),
        .fill_valid(fill_valid),
        .fill_data(icache_data),
        .fill_pcs(fill_pcs),
        .fill_recon_pc(fill_recon_pc),
        .consume(consume),

        .instr(instr),
        .instr_pc(instr_pc),
        .fetch_valid(buf_valid)
    );

    assign fetch_valid = buf_valid && issue_valid && fetch_allowed;
    assign seq_pc_valid = consume && fetch_valid;
    assign seq_next_pc = instr_pc + 32'd4;
endmodule
