`default_nettype none
`timescale 1ns/1ps
module writeback_arbiter (
    input logic clk,
    input logic reset,

    // Results
    input logic [31:0] alu_data,
    input logic [31:0] fma_data,
    input logic [31:0] lsu_data,
    input logic [31:0] special_reg_data,

    // Register file addresses
    input logic [9:0] alu_addr,
    input logic [9:0] fma_addr,
    input logic [9:0] lsu_addr,
    input logic [9:0] special_reg_addr,

    // Valid bits
    input logic alu_valid,
    input logic fma_valid,
    input logic lsu_valid,
    input logic special_reg_valid,

    // alu signal instructions
    input logic is_branch,
    input logic is_load,
    input logic is_store,

    // Global stall (OR of every lane's buffer_full)
    input logic stall,

    // Output address, data, and write enable to register bank
    output logic [9:0] rf_write_addr,
    output logic [31:0] rf_write_data,
    output logic rf_write_enable,

    // Backpressure — stall issue of instructions of this result type
    output logic buffer_full
);
    logic alu_writeback_valid;
    assign alu_writeback_valid = alu_valid && !is_branch && !is_load && !is_store;

    // Incoming results, indexed by source
    logic [3:0] in_valid;
    logic [3:0][9:0] in_addr;
    logic [3:0][31:0] in_data;

    assign in_valid = {special_reg_valid, alu_writeback_valid, fma_valid, lsu_valid} & {4{!stall}};
    assign in_addr = {special_reg_addr, alu_addr, fma_addr, lsu_addr};
    assign in_data = {special_reg_data, alu_data, fma_data, lsu_data};

    // One FIFO per source
    logic [3:0] fifo_push, fifo_pop, fifo_empty, fifo_full;
    logic [3:0][9:0] head_addr;
    logic [3:0][31:0] head_data;

    genvar s;
    generate
        for (s = 0; s < 4; ++s) begin : g_fifo
            writeback_fifo #(.DEPTH(16)) u_buffer (
                .clk(clk),
                .reset(reset),

                .push(fifo_push[s]),
                .addr(in_addr[s]),
                .data(in_data[s]),

                .pop(fifo_pop[s]),
                .head_addr(head_addr[s]),
                .head_data(head_data[s]),

                .empty(fifo_empty[s]),
                .full(fifo_full[s])
            );
        end
    endgenerate

    logic [3:0] cand_valid;
    assign cand_valid = ~fifo_empty | in_valid;

    // Round-robin between sources, starting from rr_ptr
    logic [1:0] rr_ptr, winner, idx;
    logic win_valid;

    always_comb begin
        winner = rr_ptr;
        win_valid = 1'b0;

        // Checked from lowest to highest priority, so rr_ptr itself wins if valid
        for (int k = 3; k >= 0; --k) begin
            idx = rr_ptr + k[1:0];
            if (cand_valid[idx]) begin
                winner = idx;
                win_valid = 1'b1;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (reset)
            rr_ptr <= '0;
        else if (win_valid)
            rr_ptr <= winner + 2'd1;
    end

    always_comb begin
        for (int i = 0; i < 4; ++i) begin
            fifo_pop[i] = win_valid && (winner == i[1:0]) && !fifo_empty[i];
            fifo_push[i] = in_valid[i] && !(win_valid && (winner == i[1:0]) && fifo_empty[i]);
        end
    end

    logic [3:0][9:0] cand_addr;
    logic [3:0][31:0] cand_data;

    always_comb begin
        for (int i = 0; i < 4; ++i) begin
            cand_addr[i] = fifo_empty[i] ? in_addr[i] : head_addr[i];
            cand_data[i] = fifo_empty[i] ? in_data[i] : head_data[i];
        end
    end

    assign rf_write_enable = win_valid;

    always_comb begin
        case (winner)
            2'd0: {rf_write_addr, rf_write_data} = {cand_addr[0], cand_data[0]};
            2'd1: {rf_write_addr, rf_write_data} = {cand_addr[1], cand_data[1]};
            2'd2: {rf_write_addr, rf_write_data} = {cand_addr[2], cand_data[2]};
            default: {rf_write_addr, rf_write_data} = {cand_addr[3], cand_data[3]};
        endcase
    end

    assign buffer_full = |fifo_full;
endmodule
