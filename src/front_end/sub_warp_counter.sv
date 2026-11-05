`default_nettype none
`timescale 1ns/1ps
module sub_warp_counter(
    input logic clk,
    input logic reset,
    input logic fetch_valid, // from fetch_stage — hold at 3 instead of resetting to 0 for invalid fetch
    input logic buffer_full, // from writeback_arbiter - stall (hold current warp and make it not valid)
    input logic have_ready,
    output logic [1:0] sub_warp_cycle,
    output logic sub_warp_valid
);
    logic hold_d;
    logic waiting;

    assign hold_d = buffer_full || (sub_warp_cycle == 2'd3 && waiting);
    assign sub_warp_valid = !hold_d;

    always_ff @(posedge clk) begin
        if (reset) begin
            sub_warp_cycle <= 2'd3;
            waiting <= 1'b1;
        end else if (!buffer_full) begin
            if (sub_warp_cycle == 2'd3) begin
                if (!waiting) begin
                    waiting <= 1'b1; // Execute subwarp 3, then mark it completed.
                end else if (fetch_valid && have_ready) begin
                    sub_warp_cycle <= 2'd0;
                    waiting <= 1'b0;
                end
            end else begin
                sub_warp_cycle <= sub_warp_cycle + 2'd1;
                waiting <= 1'b0;
            end
        end
    end
endmodule
