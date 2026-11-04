`default_nettype none
`timescale 1ns/1ps
module recon_stack # (
    parameter int DEPTH = 8,
    parameter int POINTER_DEPTH = $clog2(DEPTH + 1)
) (
    input logic clk,
    input logic reset,

    // Kernel launch
    input logic init_push,
    input logic [31:0] init_mask, // = 32'hFFFFFFFF
    input logic [31:0] init_pc, // = start_pc
    input logic [31:0] init_recon_pc,  // = 32'hFFFFFFFF (sentinel)

    // Divergent branch resolution
    input logic push2_valid,
    input logic [31:0] push2_mask_taken,
    input logic [31:0] push2_mask_not_taken,
    input logic [31:0] push2_pc_taken,
    input logic [31:0] push2_pc_fallthrough,
    input logic [31:0] push2_recon_pc,
    input logic push2_skip_taken, // registered in mask_split_unit
    input logic push2_skip_not_taken,

    input logic current_pc_valid, // asserted by whatever drives current_pc, only when this warp is selected
    input logic [31:0] current_pc, // fetch_stage's PC for this warp, compared against top's reconverge_pc

    // Top of stack data
    output logic [31:0] top_mask,
    output logic [31:0] top_pc,
    output logic [31:0] top_recon_pc,

    // Status Signals
    output logic stack_empty,
    output logic stack_full, // overflow condition on attempted push2/init_push
    output logic push2_done // feeds warp_scheduler to clear stall
);
    logic [31:0] masks [0:DEPTH-1];
    logic [31:0] pcs [0:DEPTH-1];
    logic [31:0] recon_pcs [0:DEPTH-1];
    logic [POINTER_DEPTH-1:0] next_free, top_valid, next_free_plus1;

    logic pop_valid;
    logic skip_taken, skip_not_taken;
    logic [1:0] push_count;

    assign pop_valid = !stack_empty && !init_push && !push2_valid &&
                       current_pc_valid && (current_pc == recon_pcs[top_valid]);

    assign top_valid = next_free - 1;
    assign next_free_plus1 = next_free + 1;

    // Skip pushing a branch whose PC already equals the reconvergence point
    // That branch has an empty body and would pop again immediately anyway
    // (compares done and registered in mask_split_unit)

    assign skip_taken = push2_skip_taken;
    assign skip_not_taken = push2_skip_not_taken;
    assign push_count = (!skip_taken && !skip_not_taken) ? 2'd2 :
                        (skip_taken != skip_not_taken)   ? 2'd1 : 2'd0;

    always_ff @(posedge clk) begin
        if (reset) next_free <= 'b0;
        else begin
            if (init_push) begin
                masks[0] <= init_mask;
                pcs[0] <= init_pc;
                recon_pcs[0] <= init_recon_pc;
                next_free <= 'b1;
            end else if (push2_valid && !stack_full) begin
                pcs[top_valid] <= push2_recon_pc;
                case (push_count)
                    2'd2: begin
                        masks[next_free] <= push2_mask_not_taken;
                        pcs[next_free] <= push2_pc_fallthrough;
                        recon_pcs[next_free] <= push2_recon_pc;

                        masks[next_free_plus1] <= push2_mask_taken;
                        pcs[next_free_plus1] <= push2_pc_taken;
                        recon_pcs[next_free_plus1] <= push2_recon_pc;

                        next_free <= next_free + 2;
                    end
                    2'd1: begin
                        if (skip_not_taken) begin
                            masks[next_free] <= push2_mask_taken;
                            pcs[next_free] <= push2_pc_taken;
                            recon_pcs[next_free] <= push2_recon_pc;
                        end else begin
                            masks[next_free] <= push2_mask_not_taken;
                            pcs[next_free] <= push2_pc_fallthrough;
                            recon_pcs[next_free] <= push2_recon_pc;
                        end
                        next_free <= next_free + 1;
                    end
                    2'd0: begin
                        if (push2_recon_pc == recon_pcs[top_valid])
                            next_free <= next_free - 1'b1;
                    end
                    default: ; // unreachable
                endcase
            end else begin
                if (pop_valid)
                    next_free <= next_free - 1;
                
                if (current_pc_valid)
                    pcs[top_valid] <= current_pc;
            end
        end
    end

    assign top_mask = masks[top_valid];
    assign top_pc = pcs[top_valid];
    assign top_recon_pc = recon_pcs[top_valid];

    assign stack_empty = ~|next_free; // (next_free == 0)
    assign stack_full = (next_free >= DEPTH - 1);
    assign push2_done = push2_valid && !stack_full;

    `ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if (!reset) begin
            assert($onehot0({init_push, push2_valid, pop_valid}))
                else $error("recon_stack: conflicting push/pop in same cycle");
        end
    end
    `endif
endmodule
