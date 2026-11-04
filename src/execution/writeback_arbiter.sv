`default_nettype none
`timescale 1ns/1ps
module writeback_arbiter (
    input logic clk,
    input logic reset,

    // Results
    input logic [31:0] alu_data,
    input logic [31:0] mul_data,
    input logic [31:0] fma_data,
    input logic [31:0] lsu_data,
    input logic [31:0] special_reg_data,

    // Register file addresses
    input logic [9:0] alu_addr,
    input logic [9:0] mul_addr,
    input logic [9:0] fma_addr,
    input logic [9:0] lsu_addr,
    input logic [9:0] special_reg_addr,

    // Valid bits
    input logic alu_valid,
    input logic mul_valid,
    input logic fma_valid,
    input logic lsu_valid,
    input logic special_reg_valid,

    // alu signal instructions
    input logic is_branch,
    input logic is_load,
    input logic is_store,

    // Global stall (registered OR of every lane's buffer_full)
    input logic stall,

    // Output address, data, and write enable to register bank
    output logic [9:0] rf_write_addr,
    output logic [31:0] rf_write_data,
    output logic rf_write_enable,

    // Backpressure — some FIFO has one slot left. sm_top registers the OR of these,
    // so the stall lands a cycle late, and the last slot absorbs that cycle's push.
    output logic buffer_full
);
    logic alu_writeback_valid;
    assign alu_writeback_valid = alu_valid && !is_branch && !is_load && !is_store;

    // Incoming results, indexed by source
    logic [4:0] in_valid;
    logic [4:0][9:0] in_addr;
    logic [4:0][31:0] in_data;

    assign in_valid = {special_reg_valid, alu_writeback_valid, mul_valid, fma_valid, lsu_valid} & {5{!stall}};
    assign in_addr = {special_reg_addr, alu_addr, mul_addr, fma_addr, lsu_addr};
    assign in_data = {special_reg_data, alu_data, mul_data, fma_data, lsu_data};

    // One FIFO per source
    logic [4:0] fifo_push, fifo_pop, fifo_empty, fifo_full, fifo_almost_full;
    logic [4:0][9:0] head_addr;
    logic [4:0][31:0] head_data;

    genvar s;
    generate
        for (s = 0; s < 5; ++s) begin : g_fifo
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
                .full(fifo_full[s]),
                .almost_full(fifo_almost_full[s])
            );
        end
    endgenerate

    logic [4:0] cand_valid;
    assign cand_valid = ~fifo_empty | in_valid;

    // Round-robin between sources, starting from rr_ptr
    logic [4:0] rr_ptr;
    logic [4:0] masked_requests, grant;

    // Lowest requester at/after the pointer; if none, wrap and take the lowest overall
    assign masked_requests = cand_valid & rr_ptr;
    assign grant = (|masked_requests) ? (masked_requests & ~(masked_requests - 1))
                                      : (cand_valid & ~(cand_valid - 1));

    always_ff @(posedge clk) begin
        if (reset)
            rr_ptr <= '1;
        else if (|cand_valid)
            rr_ptr <= ~(grant | (grant - 1));
    end

    always_comb begin
        for (int i = 0; i < 5; ++i) begin
            fifo_pop[i] = grant[i] && !fifo_empty[i];
            fifo_push[i] = in_valid[i] && !(grant[i] && fifo_empty[i]);
        end
    end

    logic [4:0][9:0] cand_addr;
    logic [4:0][31:0] cand_data;

    always_comb begin
        for (int i = 0; i < 5; ++i) begin
            cand_addr[i] = fifo_empty[i] ? in_addr[i] : head_addr[i];
            cand_data[i] = fifo_empty[i] ? in_data[i] : head_data[i];
        end
    end

    logic rf_write_enable_next;
    logic [9:0] rf_write_addr_next;
    logic [31:0] rf_write_data_next;

    assign rf_write_enable_next = |cand_valid;

    always_comb begin
        rf_write_addr_next = '0;
        rf_write_data_next = '0;
        for (int i = 0; i < 5; ++i) begin
            rf_write_addr_next |= cand_addr[i] & {10{grant[i]}};
            rf_write_data_next |= cand_data[i] & {32{grant[i]}};
        end
    end

    always_ff @(posedge clk) begin
        if (reset) begin
            rf_write_enable <= 1'b0;
            rf_write_addr <= '0;
            rf_write_data <= '0;
        end else begin
            rf_write_enable <= rf_write_enable_next;
            rf_write_addr <= rf_write_addr_next;
            rf_write_data <= rf_write_data_next;
        end
    end

    assign buffer_full = |fifo_almost_full;
endmodule
