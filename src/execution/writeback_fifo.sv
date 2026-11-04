`default_nettype none
`timescale 1ns/1ps
module writeback_fifo #(
    parameter int DEPTH = 16,
    parameter int PTR_DEPTH = $clog2(DEPTH),
    parameter int COUNT_DEPTH = $clog2(DEPTH+1)
) (
    input logic clk,
    input logic reset,

    input logic push,
    input logic [9:0] addr,
    input logic [31:0] data,

    input logic pop,

    output logic [9:0] head_addr,
    output logic [31:0] head_data,
    output logic empty,
    output logic full,
    output logic almost_full // one slot left
);
    (* ram_style = "distributed" *) logic [41:0] mem [0:DEPTH-1];
    logic [PTR_DEPTH-1:0] rd_ptr, wr_ptr;
    logic [COUNT_DEPTH-1:0] count_q;

    always_ff @(posedge clk) begin
        if (push)
            mem[wr_ptr] <= {addr, data};
    end

    assign {head_addr, head_data} = mem[rd_ptr];

    assign empty = ~|count_q; // (count_q == 0)
    assign full = (count_q == DEPTH);
    assign almost_full = (count_q >= (DEPTH - 1));

    // only the pointers can reset
    always_ff @(posedge clk) begin
        if (reset) begin
            rd_ptr <= '0;
            wr_ptr <= '0;
            count_q <= '0;
        end else begin
            if (push)
                wr_ptr <= wr_ptr + 1'b1;
            if (pop)
                rd_ptr <= rd_ptr + 1'b1;

            count_q <= count_q + push - pop;
        end
    end

    `ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if (!reset) begin
            assert (!(push && full && !pop))
                else $fatal(1, "Writeback FIFO overflow!");

            assert (!(pop && empty))
                else $fatal(1, "Writeback FIFO underflow!");
        end
    end
    `endif
endmodule
