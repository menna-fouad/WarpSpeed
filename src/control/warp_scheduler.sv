`default_nettype none
`timescale 1ns/1ps
module warp_scheduler #(
    parameter int NUM_WARPS = 8,
    parameter int WARP_W = $clog2(NUM_WARPS)
) (
    input logic clk,
    input logic reset,

    input logic kernel_start,
    input logic [WARP_W:0] num_warps,

    input logic [NUM_WARPS-1:0] commit_done,
    input logic [NUM_WARPS-1:0] store_done,
    input logic [NUM_WARPS-1:0] push2_done,
    output logic have_ready,

    input logic consume,
    output logic fetch_allowed,

    input logic [1:0] sub_warp_cycle,
    input logic sub_warp_valid,
    input logic buffer_full,

    input logic done_detect,
    input logic flush_done, // used to set kernel_done

    output logic [WARP_W-1:0] curr_warp,
    output logic [WARP_W-1:0] next_warp,
    output logic issue_valid,
    output logic warps_done, // used to trigger flush_start in data cache
    output logic kernel_done
);
    // State for each warp
    logic [NUM_WARPS-1:0] ready;
    logic [NUM_WARPS-1:0] finished;
    logic [NUM_WARPS-1:0] instr_busy; // warp has an instruction in flight

    // ready_mask is computed combinationall to show which warps are ready
    logic [NUM_WARPS-1:0] ready_mask, pick_mask;
    always_comb begin
        for (int i = 0; i < NUM_WARPS; i++) begin
            ready_mask[i] = (ready[i] | push2_done[i] | commit_done[i] | store_done[i]) // ready
                            & ~finished[i]                                                    // and not finished previously
                            & ~(i == curr_warp & done_detect);                                // or this cycle
        end
        
        // A warp with an instruction in flight can't be picked, unless it completes this cycle.
        // The ready bit alone can't tell us this: a commit from an older instruction can set it
        // while a newer one is staged or running, and picking that warp would block the front end.
        for (int i = 0; i < NUM_WARPS; i++)
            pick_mask[i] = ready_mask[i] & (~instr_busy[i] | commit_done[i] | store_done[i] | push2_done[i]);

        // The warp being issued this cycle can't be picked to go after itself
        if (consume) pick_mask[next_warp] = 1'b0;
    end

    logic [WARP_W-1:0] picked_warp;
    always_comb begin
        // A staged warp (issue_valid) has had its ready bit cleared but still has to issue
        have_ready = |ready_mask || issue_valid;
        picked_warp = '0;
        for (int i = NUM_WARPS-1; i >= 0; i--)
            // If its ready and it not the warp just finished then pick it
            if (pick_mask[i]) picked_warp = i[WARP_W-1:0];
    end

    // GTO: stay on curr_warp if it's still ready and not being issued, else switch.
    logic [WARP_W-1:0] next_warp_d;
    logic next_valid_d;
    always_comb begin
        if (pick_mask[curr_warp]) begin
            next_warp_d = curr_warp;
            next_valid_d = 1'b1;
        end else if (|pick_mask) begin
            next_warp_d = picked_warp;
            next_valid_d = 1'b1;
        end else begin
            next_warp_d = curr_warp;
            next_valid_d = 1'b0;
        end
    end

    logic decision;
    assign decision = (sub_warp_cycle == 2'd0 && sub_warp_valid && !buffer_full);

    always_ff @(posedge clk) begin
        if (reset) begin
            curr_warp <= '0;
            next_warp <= '0;
            issue_valid <= 1'b0;
        end else if (kernel_start) begin
            curr_warp <= '0;
            next_warp <= '0;
            issue_valid <= (num_warps != 0);
        end else if (decision) begin
            curr_warp <= next_warp;      // adopt last span's staged pick
            next_warp <= next_warp_d;    // stage the following span's pick
            issue_valid <= next_valid_d;
        end else if (!issue_valid && next_valid_d) begin
            next_warp <= next_warp_d;
            issue_valid <= 1'b1;
        end
    end

    always_ff @(posedge clk) begin
        if (reset || kernel_start) begin
            instr_busy <= '0;
        end else begin
            for (int i = 0; i < NUM_WARPS; i++) begin
                if (consume && next_warp == i[WARP_W-1:0])
                    instr_busy[i] <= 1'b1;
                else if (commit_done[i] || store_done[i] || push2_done[i])
                    instr_busy[i] <= 1'b0;
            end
        end
    end

    assign fetch_allowed = !instr_busy[next_warp] && !finished[next_warp];

    always_ff @(posedge clk) begin
        if (reset) begin
            ready <= '0;
            finished <= '0;
        end else if (kernel_start) begin
            for (int i = 0; i < NUM_WARPS; i++) begin
                ready[i]    <= (i < num_warps);
                finished[i] <= (i >= num_warps);
            end
        end else begin
            for (int i = 0; i < NUM_WARPS; i++) begin
                logic set_ready, clear_ready;
                set_ready = commit_done[i] || store_done[i] || push2_done[i];
                clear_ready = decision && next_valid_d && (next_warp_d == i[WARP_W-1:0]);

                if (clear_ready) ready[i] <= 1'b0;
                else if (set_ready) ready[i] <= 1'b1;
            end
            if (done_detect)
                finished[curr_warp] <= 1'b1;
        end
    end

    assign warps_done = &finished;
    
    always_ff @(posedge clk) begin
        if (reset || kernel_start)
            kernel_done <= 1'b0;
        else if (warps_done && flush_done)
            kernel_done <= 1'b1;
    end
endmodule
