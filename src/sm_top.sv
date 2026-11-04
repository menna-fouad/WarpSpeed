`default_nettype none
`timescale 1ns/1ps
module sm_top (
    input logic sm_clk,
    input logic sm_reset,

    input logic mem_ui_clk,
    input logic mem_ui_reset,

    input logic kernel_start,
    input logic [3:0] num_warps,
    output logic kernel_done,

    // MIG interface
    input logic init_calib_complete,
    output logic app_en,
    output logic [2:0] app_cmd,
    output logic [26:0] app_addr,
    input  logic app_rdy,

    // write data channel
    output logic [127:0] app_wdf_data,
    output logic app_wdf_wren,
    output logic app_wdf_end,
    output logic [15:0] app_wdf_mask,
    input  logic app_wdf_rdy,

    // read data channel
    input  logic [127:0] app_rd_data,
    input  logic app_rd_data_valid,

    output logic [3:0] kernel_state_leds
);
    logic fetch_valid, sub_warp_valid, have_ready;
    (* max_fanout = 64 *) logic wb_buffer_full;
    logic [1:0] sub_warp_cycle;

    sub_warp_counter u_sub_warp_counter (
        .clk(sm_clk),
        .reset(sm_reset),
        .fetch_valid(fetch_valid), // from fetch_stage
        .buffer_full(wb_buffer_full), // from writeback_arbiter
        .have_ready(have_ready), // from warp_scheduler
        .sub_warp_cycle(sub_warp_cycle),
        .sub_warp_valid(sub_warp_valid)
    );

    logic consume, fetch_allowed;
    logic dec_done_detect;
    logic [7:0] commit_done;
    logic [7:0] recon_push2_done_arr;

    logic [2:0] ws_curr_warp, ws_next_warp;
    logic ws_issue_valid;
    logic warps_done;

    assign consume = ws_issue_valid && ~|sub_warp_cycle && sub_warp_valid && !wb_buffer_full;

    warp_scheduler u_warp_scheduler (
        .clk(sm_clk),
        .reset(sm_reset),
        .kernel_start(kernel_start),
        .num_warps(num_warps),
        .commit_done(commit_done), // from commit_tracker
        .store_done(store_done), // from load_store_unit
        .push2_done(recon_push2_done_arr), // from recon_pc
        .have_ready(have_ready), // output to sub_warp_counter
        .consume(consume),
        .fetch_allowed(fetch_allowed),
        .sub_warp_cycle(sub_warp_cycle), // from u_sub_warp_counter
        .sub_warp_valid(sub_warp_valid), // from u_sub_warp_counter
        .buffer_full(wb_buffer_full), // from writeback_arbiter
        .done_detect(dec_done_detect), // from decode
        .flush_done(flush_done), // from data cache
        .curr_warp(ws_curr_warp),
        .next_warp(ws_next_warp),
        .issue_valid(ws_issue_valid),
        .warps_done(warps_done),
        .kernel_done(kernel_done)
    );

    // recon_stack x8
    // recon_stack outputs
    logic [7:0][31:0] recon_top_mask_arr;
    logic [7:0][31:0] recon_top_pc_arr;
    logic [7:0][31:0] recon_top_recon_pc_arr;

    // Mask split unit outputs routed to reconvergence stacks
    logic [7:0] ms_push2_valid;
    logic [31:0] ms_push2_mask_taken;
    logic [31:0] ms_push2_mask_not_taken;
    logic [31:0] ms_push2_pc_taken;
    logic [31:0] ms_push2_pc_fallthrough;
    logic [31:0] ms_push2_recon_pc;
    logic ms_push2_skip_taken, ms_push2_skip_not_taken;

    // Current PC signals
    logic seq_pc_valid;
    logic [31:0] seq_next_pc;

    // Registered PC update into the recon stacks (one-hot per warp), taking the
    // fetch path off the recon stack CE. The warp is instr_busy until commit,
    // so nothing reads its stack top during the extra cycle.
    logic [7:0] seq_pc_valid_q;
    logic [31:0] seq_next_pc_q;

    always_ff @(posedge sm_clk) begin
        if (sm_reset)
            seq_pc_valid_q <= '0;
        else
            seq_pc_valid_q <= {8{seq_pc_valid}} & (8'b1 << ws_next_warp);

        seq_next_pc_q <= seq_next_pc;
    end

    genvar gi;
    generate
        for (gi = 0; gi < 8; ++gi) begin : g_recon
            recon_stack #(.DEPTH(8)) u_recon (
                .clk(sm_clk),
                .reset(sm_reset),
                .init_push(kernel_start && (gi < num_warps)),
                .init_mask(32'hFFFF_FFFF),
                .init_pc(32'b0),
                .init_recon_pc(32'hFFFF_FFFF),
                .push2_valid(ms_push2_valid[gi]),
                .push2_mask_taken(ms_push2_mask_taken),
                .push2_mask_not_taken(ms_push2_mask_not_taken),
                .push2_pc_taken(ms_push2_pc_taken),
                .push2_pc_fallthrough(ms_push2_pc_fallthrough),
                .push2_recon_pc(ms_push2_recon_pc),
                .push2_skip_taken(ms_push2_skip_taken),
                .push2_skip_not_taken(ms_push2_skip_not_taken),
                .current_pc_valid(seq_pc_valid_q[gi]),
                .current_pc(seq_next_pc_q),
                .top_mask(recon_top_mask_arr[gi]),
                .top_pc(recon_top_pc_arr[gi]),
                .top_recon_pc(recon_top_recon_pc_arr[gi]),
                .stack_empty(),
                .stack_full(),
                .push2_done(recon_push2_done_arr[gi])
            );
        end
    endgenerate

    logic [31:0] fs_instr, fs_instr_pc;

    logic icache_req_ready, icache_req_valid, icache_rsp_valid;
    logic [31:0] icache_req_addr;
    logic [127:0] icache_data;

    fetch_stage u_fetch (
        .clk(sm_clk),
        .reset(sm_reset),
        .kernel_launch(kernel_start),
        .issue_valid(ws_issue_valid),
        .next_warp(ws_next_warp),
        .top_pc(recon_top_pc_arr[ws_next_warp]),
        .recon_pc(recon_top_recon_pc_arr[ws_next_warp]),
        .fetch_allowed(fetch_allowed),
        .consume(consume),
        .icache_ready(icache_req_ready),
        .icache_rsp_valid(icache_rsp_valid),
        .icache_data(icache_data),
        .icache_req_valid(icache_req_valid),
        .icache_addr(icache_req_addr),
        .fetch_valid(fetch_valid),
        .instr(fs_instr),
        .instr_pc(fs_instr_pc),
        .seq_next_pc(seq_next_pc),
        .seq_pc_valid(seq_pc_valid)
    );

    logic [31:0] il_instr, il_instr_pc, il_active_mask;
    logic [9:0] il_src1_addr, il_src2_addr, il_src3_addr;

    instruction_latch u_instruction_latch (
        .clk(sm_clk),
        .reset(sm_reset),
        .consume(consume),
        .buffer_full(wb_buffer_full),
        .sub_warp_cycle(sub_warp_cycle),
        .next_warp(ws_next_warp),
        .curr_warp(ws_curr_warp),
        .buf_instr(fs_instr),
        .buf_instr_pc(fs_instr_pc),
        .active_mask(recon_top_mask_arr[ws_next_warp]),
        .active_mask_q(il_active_mask),
        .instr_q(il_instr),
        .instr_pc_q(il_instr_pc),
        .src1_addr(il_src1_addr),
        .src2_addr(il_src2_addr),
        .src3_addr(il_src3_addr)
    );

    logic dec_sub_warp_valid;
    logic [1:0] dec_sub_warp_cycle;
    logic [5:0] dec_opcode;
    logic [7:0] dec_alu_valid, dec_fma_valid, dec_special_reg_valid;
    logic [9:0] dec_dest_addr;
    logic [15:0] dec_imm;
    logic [31:0] dec_instr_pc;

    decode_stage u_decode (
        .clk(sm_clk),
        .reset(sm_reset),
        .buffer_full(wb_buffer_full),
        .sub_warp_valid(sub_warp_valid),
        .sub_warp_cycle(sub_warp_cycle),
        .curr_warp(ws_curr_warp),
        .active_mask(il_active_mask),
        .instr(il_instr),
        .instr_pc(il_instr_pc),
        .sub_warp_valid_q(dec_sub_warp_valid),
        .sub_warp_cycle_q(dec_sub_warp_cycle),
        .opcode_q(dec_opcode),
        .done_detect(dec_done_detect),
        .alu_valid_q(dec_alu_valid),
        .fma_valid_q(dec_fma_valid),
        .special_reg_valid_q(dec_special_reg_valid),
        .dest_addr_q(dec_dest_addr),
        .imm_q(dec_imm),
        .instr_pc_q(dec_instr_pc)
    );

    logic [7:0][31:0] reg_src1_data, reg_src2_data, reg_src3_data;
    logic [7:0] wb_rf_write_enable;
    logic [7:0][9:0] wb_rf_write_addr_arr;
    logic [7:0][31:0] wb_rf_write_data;

    generate
        for (gi = 0; gi < 8; ++gi) begin : gen_reg_bank
            reg_file u_reg_file (
                .clk(sm_clk),
                .reset(sm_reset),

                // source addresses from instruction latch
                .src1_addr(il_src1_addr),
                .src2_addr(il_src2_addr),
                .src3_addr(il_src3_addr),

                // write data from writeback_arbiter
                .write_en(wb_rf_write_enable[gi]),
                .wr_addr(wb_rf_write_addr_arr[gi]),
                .wr_data(wb_rf_write_data[gi]),

                .src1_data(reg_src1_data[gi]),
                .src2_data(reg_src2_data[gi]),
                .src3_data(reg_src3_data[gi])
            );
        end
    endgenerate

    logic [7:0] alu_valid_out, alu_branch_taken;
    logic [7:0][31:0] alu_result;

    logic [7:0] mul_valid_out;
    logic [7:0][31:0] mul_result_arr;
    logic [7:0][9:0] mul_addr_arr;

    logic [7:0] alu_is_branch_arr, alu_is_load_arr, alu_is_store_arr, alu_sub_warp_valid_arr;
    logic [7:0][1:0] alu_sub_warp_cycle_arr;
    logic [7:0][31:0] alu_branch_pc_arr;
    logic [7:0][7:0] alu_recon_offset_arr;
    
    logic [7:0][9:0] alu_reg_bank_addr_out;
    logic [7:0][31:0] alu_src2_out;

    generate
        for (gi = 0; gi < 8; ++gi) begin : gen_int_alu
            int_alu u_int_alu (
                .clk(sm_clk),
                .reset(sm_reset),
                .valid_in(dec_alu_valid[gi]),
                .buffer_full(wb_buffer_full),
                .sub_warp_valid_in(dec_sub_warp_valid),
                .sub_warp_cycle_in(dec_sub_warp_cycle),
                .reg_bank_addr_in(dec_dest_addr),
                .opcode(dec_opcode),
                .src1(reg_src1_data[gi]),
                .src2(reg_src2_data[gi]),
                .instr_pc(dec_instr_pc),
                .offset(dec_imm),
                .valid_out(alu_valid_out[gi]),
                .branch_taken(alu_branch_taken[gi]),
                .result(alu_result[gi]),
                .mul_valid(mul_valid_out[gi]),
                .mul_result(mul_result_arr[gi]),
                .mul_addr(mul_addr_arr[gi]),
                
                // outputs for writeback_arbiter
                .sub_warp_valid_out(alu_sub_warp_valid_arr[gi]),
                .sub_warp_cycle_out(alu_sub_warp_cycle_arr[gi]),
                .reg_bank_addr_out(alu_reg_bank_addr_out[gi]),

                // Instruction information
                .is_branch(alu_is_branch_arr[gi]),
                .is_load(alu_is_load_arr[gi]),
                .is_store(alu_is_store_arr[gi]),
                .branch_pc_out(alu_branch_pc_arr[gi]),
                .push2_pc_recon_offset(alu_recon_offset_arr[gi]),
                .src2_out(alu_src2_out[gi]) // needed for LSU store instruction
            );
        end
    endgenerate

    wire alu_is_branch = alu_is_branch_arr[0];
    wire alu_is_load = alu_is_load_arr[0];
    wire alu_is_store = alu_is_store_arr[0];
    wire alu_sub_warp_valid = alu_sub_warp_valid_arr[0];
    wire [1:0] alu_sub_warp_cycle = alu_sub_warp_cycle_arr[0];
    wire [9:0] alu_reg_bank_addr = alu_reg_bank_addr_out[0];
    wire [31:0] alu_branch_pc = alu_branch_pc_arr[0];
    wire [7:0] alu_push2_pc_recon_offset = alu_recon_offset_arr[0];

    mask_split_unit u_mask_split (
        .clk(sm_clk),
        .reset(sm_reset),
        .buffer_full(wb_buffer_full),
        .sub_warp_cycle(alu_sub_warp_cycle),
        .sub_warp_valid(alu_sub_warp_valid),
        .is_branch(alu_is_branch),
        .is_valid(alu_valid_out),
        .branch_taken_bits(alu_branch_taken),
        .branch_pc(alu_branch_pc),
        .push2_pc_recon_offset(alu_push2_pc_recon_offset),
        .pc_taken(alu_result[0]),
        .reg_bank_addr(alu_reg_bank_addr),
        .push2_mask_taken(ms_push2_mask_taken),
        .push2_mask_not_taken(ms_push2_mask_not_taken),
        .push2_pc_taken(ms_push2_pc_taken),
        .push2_pc_fallthrough(ms_push2_pc_fallthrough),
        .push2_recon_pc(ms_push2_recon_pc),
        .push2_skip_taken(ms_push2_skip_taken),
        .push2_skip_not_taken(ms_push2_skip_not_taken),
        .recon_push2_valid(ms_push2_valid)
    );

    logic [7:0] fma_valid_out;
    logic [7:0][31:0] fma_result;
    logic [7:0][9:0] fma_reg_bank_addr_out;

    generate
        for (gi = 0; gi < 8; ++gi) begin : gen_fma
            fp32_fma u_fma (
                .clk(sm_clk),
                .reset(sm_reset),
                .valid_in(dec_fma_valid[gi]),
                .buffer_full(wb_buffer_full),
                .src1(reg_src1_data[gi]),
                .src2(reg_src2_data[gi]),
                .src3(reg_src3_data[gi]),
                .reg_bank_addr_in(dec_dest_addr),
                .result(fma_result[gi]),
                .valid_out(fma_valid_out[gi]),
                .reg_bank_addr_out(fma_reg_bank_addr_out[gi])
            );
        end
    endgenerate

    wire [9:0] fma_reg_bank_addr = fma_reg_bank_addr_out[0];

    logic [7:0][31:0] sp_reg_result;
    logic [7:0][9:0] sp_reg_bank_addr_out;
    logic [7:0] sp_reg_valid_out;

    generate
        for (gi = 0; gi < 8; ++gi) begin : gen_special_reg
            special_reg_unit #(.LANE_NUM(gi)) u_special_reg (
                .warp_id(dec_dest_addr[9:7]),
                .sub_warp_cycle(dec_sub_warp_cycle),
                .special_instr_type(dec_opcode[2:0]),
                .result(sp_reg_result[gi]),
                .reg_bank_addr_in(dec_dest_addr),
                .reg_bank_addr_out(sp_reg_bank_addr_out[gi]),
                .valid_in(dec_special_reg_valid[gi]),
                .valid_out(sp_reg_valid_out[gi])
            );
        end
    endgenerate

    wire [9:0] sp_reg_bank_addr = sp_reg_bank_addr_out[0];

    logic [7:0] lsu_valid_out;
    logic [7:0][31:0] lsu_result;
    logic [9:0] lsu_reg_bank_addr;

    logic [7:0] wb_buffer_full_arr;

    generate
        for (gi = 0; gi < 8; ++gi) begin : gen_wb_arbiter
            writeback_arbiter u_writeback_arbiter (
                .clk(sm_clk),
                .reset(sm_reset),

                .alu_data(alu_result[gi]),
                .mul_data(mul_result_arr[gi]),
                .fma_data(fma_result[gi]),
                .lsu_data(lsu_result[gi]),
                .special_reg_data(sp_reg_result[gi]),

                .alu_addr(alu_reg_bank_addr),
                .mul_addr(mul_addr_arr[0]),
                .fma_addr(fma_reg_bank_addr),
                .lsu_addr(lsu_reg_bank_addr),
                .special_reg_addr(sp_reg_bank_addr),

                .alu_valid(alu_valid_out[gi]),
                .mul_valid(mul_valid_out[gi]),
                .fma_valid(fma_valid_out[gi]),
                .lsu_valid(lsu_valid_out[gi]),
                .special_reg_valid(sp_reg_valid_out[gi]),

                .is_branch(alu_is_branch),
                .is_load(alu_is_load),
                .is_store(alu_is_store),

                .stall(wb_buffer_full),

                .rf_write_addr(wb_rf_write_addr_arr[gi]),
                .rf_write_data(wb_rf_write_data[gi]),
                .rf_write_enable(wb_rf_write_enable[gi]),
                
                .buffer_full(wb_buffer_full_arr[gi])
            );
        end
    endgenerate

    always_ff @(posedge sm_clk) begin
        if (sm_reset)
            wb_buffer_full <= 1'b0;
        else
            wb_buffer_full <= |wb_buffer_full_arr;
    end

    commit_tracker u_commit_tracker (
        .clk(sm_clk),
        .reset(sm_reset),
        .consume(consume),
        .ws_next_warp(ws_next_warp),
        .recon_top_mask_arr(recon_top_mask_arr),
        .rf_write_enable(wb_rf_write_enable),
        .rf_write_addr(wb_rf_write_addr_arr),
        .commit_done(commit_done)
    );

    
    logic dcache_req_valid, dcache_req_ready;
    logic dcache_req_write; 
    logic [31:0] dcache_req_addr, dcache_wr_word_en;
    logic [1023:0] dcache_wr_data;

    logic lsu_ready;
    logic txn_valid, txn_is_load, txn_last;
    logic [2:0] txn_warp;
    logic [4:0] txn_dst;
    logic [31:0] txn_match_mask;
    logic [31:0][4:0] txn_word_sel;

    memory_coalescing_unit coalescing (
        .clk(sm_clk),
        .reset(sm_reset),
        // input from ALU
        .addr(alu_result),
        .src2(alu_src2_out), // store data
        .valid(alu_valid_out),
        .is_load(alu_is_load),
        .is_store(alu_is_store),
        .sub_warp_valid(alu_sub_warp_valid),
        .sub_warp_cycle(alu_sub_warp_cycle),
        .reg_bank_addr(alu_reg_bank_addr),
        // interface to data cache
        .req_valid(dcache_req_valid),
        .req_ready(dcache_req_ready),
        .req_addr(dcache_req_addr),
        .req_write(dcache_req_write),
        .wr_data(dcache_wr_data),
        .wr_word_en(dcache_wr_word_en),
        // load store unit interface
        .lsu_ready(lsu_ready),
        .txn_valid(txn_valid),
        .txn_is_load(txn_is_load),
        .txn_last(txn_last),
        .txn_warp(txn_warp),
        .txn_dst(txn_dst),
        .txn_match_mask(txn_match_mask),
        .txn_word_sel(txn_word_sel)
    );

    logic icache_mem_req_valid, icache_mem_req_ready;
    logic [31:0] icache_mem_req_addr;
    logic [127:0] icache_mem_rdata;
    logic icache_mem_rvalid, icache_mem_rlast;

    l1_icache icache (
        .clk(sm_clk),
        .reset(sm_reset),
        .req_valid(icache_req_valid),
        .req_ready(icache_req_ready),
        .req_addr(icache_req_addr),
        .req_id(1'b0),
        .rsp_valid(icache_rsp_valid),
        .instrs(icache_data),
        .rsp_id(),
        .mem_req_valid(icache_mem_req_valid),
        .mem_req_ready(icache_mem_req_ready),
        .mem_req_addr(icache_mem_req_addr),
        .mem_rdata(icache_mem_rdata),
        .mem_rvalid(icache_mem_rvalid),
        .mem_rlast(icache_mem_rlast)
    );

    logic dcache_rsp_valid;
    logic [1023:0] dcache_rsp_line;
    logic dcache_mem_req_valid, dcache_mem_req_ready, dcache_mem_req_write;
    logic [31:0] dcache_mem_req_addr;
    logic dcache_mem_wvalid, dcache_mem_wready, dcache_mem_wlast;
    logic [127:0] dcache_mem_wdata;
    logic [127:0] dcache_mem_rdata;
    logic dcache_mem_rvalid, dcache_mem_rlast;
    logic flush_start, flush_done; // sent to warp scheduler to mark kernel_done
    logic flush_done_req, flush_done_req_q;
    // Registered to keep the finished -> warps_done chain off the dcache req_ready path
    always_ff @(posedge sm_clk) begin
        if (sm_reset)
            flush_start <= 1'b0;
        else
            flush_start <= warps_done && !kernel_done;
    end

    l1_cache dcache (
        .clk(sm_clk),
        .reset(sm_reset),
        .req_valid(dcache_req_valid),
        .req_ready(dcache_req_ready),
        .req_addr(dcache_req_addr),
        .req_id(1'b0),
        .req_write(dcache_req_write),
        .wr_data(dcache_wr_data),
        .wr_word_en(dcache_wr_word_en),
        .rsp_valid(dcache_rsp_valid),
        .rsp_line(dcache_rsp_line),
        .rsp_id(),
        .flush_start(flush_start && !flush_done_req_q), // from warp_scheduler
        .flush_done(flush_done_req),
        .mem_req_valid(dcache_mem_req_valid),
        .mem_req_ready(dcache_mem_req_ready),
        .mem_req_write(dcache_mem_req_write),
        .mem_req_addr(dcache_mem_req_addr),
        .mem_wvalid(dcache_mem_wvalid),
        .mem_wready(dcache_mem_wready),
        .mem_wlast(dcache_mem_wlast),
        .mem_wdata(dcache_mem_wdata),
        .mem_rdata(dcache_mem_rdata),
        .mem_rvalid(dcache_mem_rvalid),
        .mem_rlast(dcache_mem_rlast)
    );

    always_ff @(posedge sm_clk) begin
        if (sm_reset || kernel_start) begin
            flush_done_req_q <= 1'b0;
        end else if (!flush_done_req_q) begin
            flush_done_req_q <= flush_done_req;
        end
    end

    logic wr_wempty; // empty signals from cmd and wr CDC fifos

    assign flush_done = flush_done_req_q && wr_wempty;

    logic [7:0] store_done;

    load_store_unit lsu (
        .clk(sm_clk),
        .reset(sm_reset),
        .wb_buffer_full(wb_buffer_full),
        // input transaction data from memory coalescing unit
        .txn_valid(txn_valid),
        .txn_is_load(txn_is_load),
        .txn_last(txn_last),
        .txn_warp(txn_warp),
        .txn_dst(txn_dst),
        .txn_match_mask(txn_match_mask),
        .txn_word_sel(txn_word_sel),
        // input data from data cache
        .rsp_valid(dcache_rsp_valid),
        .rsp_line(dcache_rsp_line),
        // outputs to writeback_arbiter
        .lsu_valid(lsu_valid_out),
        .lsu_addr(lsu_reg_bank_addr),
        .lsu_data(lsu_result),
        // output to warp scheduler to clear warp
        .store_done(store_done),
        .lsu_ready(lsu_ready)
    );

    logic arb_cmd_ready, arb_wready, arb_rsp_valid, arb_rsp_last;
    logic [127:0] arb_rsp_data;
    logic arb_cmd_valid, arb_cmd_write, arb_wvalid;
    logic [31:0] arb_cmd_addr;
    logic [127:0] arb_wdata;

    mem_arbiter memory_arbiter (
        .clk(sm_clk),
        .reset(sm_reset),
        .i_cache_req_valid(icache_mem_req_valid),
        .d_cache_req_valid(dcache_mem_req_valid),
        .d_cache_req_write(dcache_mem_req_write),
        .i_cache_req_addr(icache_mem_req_addr),
        .d_cache_req_addr(dcache_mem_req_addr),
        .i_cache_req_ready(icache_mem_req_ready),
        .d_cache_req_ready(dcache_mem_req_ready),
        .d_cache_wvalid(dcache_mem_wvalid),
        .d_cache_wlast(dcache_mem_wlast),
        .d_cache_wdata(dcache_mem_wdata),
        .d_cache_wready(dcache_mem_wready),
        .i_cache_rdata(icache_mem_rdata),
        .d_cache_rdata(dcache_mem_rdata),
        .i_cache_rvalid(icache_mem_rvalid),
        .d_cache_rvalid(dcache_mem_rvalid),
        .i_cache_rlast(icache_mem_rlast),
        .d_cache_rlast(dcache_mem_rlast),
        .arb_cmd_ready(arb_cmd_ready),
        .arb_wready(arb_wready),
        .rsp_valid(arb_rsp_valid),
        .rsp_last(arb_rsp_last),
        .rsp_data(arb_rsp_data),
        .arb_cmd_valid(arb_cmd_valid),
        .arb_cmd_write(arb_cmd_write),
        .arb_cmd_addr(arb_cmd_addr),
        .arb_wvalid(arb_wvalid),
        .arb_wdata(arb_wdata)
    );

    logic cmd_full, cmd_empty, cmd_pop;
    logic calib_sync_temp, calib_sync;
    assign arb_cmd_ready = !cmd_full && calib_sync;
    logic [25:0] cmd_rdata;

    always_ff @(posedge sm_clk) begin
        if (sm_reset) begin
            calib_sync_temp <= 1'b0;
            calib_sync <= 1'b0;
        end else begin
            calib_sync_temp <= init_calib_complete;
            calib_sync <= calib_sync_temp;
        end
    end

    // write side: mem_arbiter, read side: mem_expander
    cdc_fifo #(.DATA_WIDTH(26), .DEPTH(4)) cmd_fifo (
        .wclk(sm_clk),
        .wrst(sm_reset),
        .wen(arb_cmd_valid && arb_cmd_ready),
        .wdata({arb_cmd_write, arb_cmd_addr[31:7]}),
        .full(cmd_full),
        .rclk(mem_ui_clk),
        .rrst(mem_ui_reset),
        .ren(cmd_pop),
        .empty(cmd_empty),
        .rdata(cmd_rdata)
    );

    // write side: mem_arbiter, read side: mem_expander

    logic wr_full, wr_empty, wr_pop;
    logic [127:0] wr_rdata;
    assign arb_wready = !wr_full;

    cdc_fifo #(.DATA_WIDTH(128), .DEPTH(16)) wr_fifo (
        .wclk(sm_clk),
        .wrst(sm_reset),
        .wen(arb_wvalid & arb_wready),
        .wdata(arb_wdata),
        .full(wr_full),
        .wempty(wr_wempty),
        .rclk(mem_ui_clk),
        .rrst(mem_ui_reset),
        .ren(wr_pop),
        .empty(wr_empty),
        .rdata(wr_rdata)
    );

    // write side: mem_expander, read_side: mem_arbiter

    logic rd_wen, rd_ren, rd_full, rd_empty;
    logic [128:0] rd_wdata, rd_rdata;
    assign rd_ren = !rd_empty;
    assign arb_rsp_valid = rd_ren;
    assign {arb_rsp_last, arb_rsp_data} = rd_rdata;

    cdc_fifo #(.DATA_WIDTH(129), .DEPTH(16)) rd_fifo (
        .wclk(mem_ui_clk),
        .wrst(mem_ui_reset),
        .wen(rd_wen),
        .wdata(rd_wdata),
        .full(rd_full),
        .rclk(sm_clk),
        .rrst(sm_reset),
        .ren(rd_ren),
        .empty(rd_empty),
        .rdata(rd_rdata)
    );

    mem_expander expander (
        .ui_clk(mem_ui_clk),
        .ui_rst(mem_ui_reset),
        .init_calib_complete(init_calib_complete),
        .cmd_empty(cmd_empty),
        .cmd_rdata(cmd_rdata),
        .cmd_pop(cmd_pop),
        .wr_empty(wr_empty),
        .wr_data(wr_rdata),
        .wr_pop(wr_pop),
        .rd_wen(rd_wen),
        .rd_wdata(rd_wdata),
        .rd_full(rd_full),
        // MIG interface
        .app_en(app_en),
        .app_cmd(app_cmd),
        .app_addr(app_addr),
        .app_rdy(app_rdy), // driven in testbench
        .app_wdf_data(app_wdf_data),
        .app_wdf_wren(app_wdf_wren),
        .app_wdf_end(app_wdf_end),
        .app_wdf_mask(app_wdf_mask),
        .app_wdf_rdy(app_wdf_rdy), // driven in testbench
        .app_rd_data(app_rd_data), // rd data driven in testbench
        .app_rd_data_valid(app_rd_data_valid) // rd data valid driven in testbench
    );

    logic running;
    logic flushing;
    logic done;

    always_ff @(posedge sm_clk) begin
        if (sm_reset) begin
            running <= 1'b0;
            flushing <= 1'b0;
            done <= 1'b0;
        end else if (kernel_start && !running && !flushing) begin
            running <= 1'b1;
            flushing <= 1'b0;
            done <= 1'b0;
        end else if (flush_start) begin
            running <= 1'b0;
            flushing <= 1'b1;
            done <= 1'b0;
        end else if (kernel_done) begin
            running <= 1'b0;
            flushing <= 1'b0;
            done <= 1'b1;
        end
    end

    assign kernel_state_leds[0] = !running && !flushing && !done;
    assign kernel_state_leds[1] = running;
    assign kernel_state_leds[2] = flushing;
    assign kernel_state_leds[3] = done;
endmodule
