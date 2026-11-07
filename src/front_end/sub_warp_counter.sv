`default_nettype none
`timescale 1ns/1ps
module sub_warp_counter(
    input logic clk,
    input logic reset,
    input logic fetch_valid, // from fetch_stage — hold at 3 instead of resetting to 0 for invalid fetch
    input logic buffer_full, // from writeback_arbiter - stall (hold current warp and make it not valid)
    input logic have_ready,
    input logic [31:0] next_mask, // mask of next_warp from recon_stack (recon_top_mask_arr[ws_next_warp])
    output logic [1:0] sub_warp_cycle,
    output logic sub_warp_valid,
    output logic sub_warp_last
);
    logic hold_d;
    logic waiting;
    logic [3:0] active_sw;

    assign hold_d = buffer_full || (sub_warp_last && waiting);
    assign sub_warp_valid = !hold_d;

    always_ff @(posedge clk) begin
        if (reset) begin
            sub_warp_cycle <= 2'd3;
            active_sw <= '1;
            waiting <= 1'b1;
        end else if (!buffer_full) begin
            if (sub_warp_last) begin
                if (fetch_valid && have_ready) begin
                    sub_warp_cycle <= 2'd0;
                    waiting <= 1'b0;

                    for (int sw = 0; sw < 4; ++sw) begin
                        active_sw[sw] <= |next_mask[sw * 8 +: 8];
                    end
                end else begin
                    waiting <= 1'b1;
                end
            end else begin
                sub_warp_cycle <= next_sw;
                waiting <= 1'b0;
            end
        end
    end

    logic [1:0] next_sw;
    logic has_next;

    always_comb begin
        next_sw = sub_warp_cycle;
        has_next = 1'b0;
        for (int sw = 3; sw >= 0; --sw) begin
            if (active_sw[sw] && sw > sub_warp_cycle) begin
                next_sw = sw[1:0];
                has_next = 1'b1;
            end
        end

        if (sub_warp_cycle == 2'd0 && !has_next) begin
            next_sw  = 2'd1;
            has_next = 1'b1;
        end
    end

    assign sub_warp_last = !has_next;
endmodule
