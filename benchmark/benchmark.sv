`timescale 1ns/1ps
module benchmark;
    logic sm_clk, sm_reset;
    logic mem_ui_clk, mem_ui_reset;

    always #5 sm_clk = ~sm_clk;
    always #6.154 mem_ui_clk = ~mem_ui_clk;
    
    logic kernel_start, kernel_done;
    logic [3:0] num_warps;

    // MIG interface
    logic init_calib_complete, app_en, app_rdy;
    logic [2:0] app_cmd;
    logic [26:0] app_addr;

    // write data channel
    logic app_wdf_rdy, app_wdf_wren, app_wdf_end;
    logic [127:0] app_wdf_data;
    logic [15:0] app_wdf_mask;

    // read data channel
    logic [127:0] app_rd_data;
    logic app_rd_data_valid;

    logic [3:0] kernel_state_leds;

    int counting = 0;
    int thread_ops = 0;
    int cycle_count = 0;

    int instr_count = 0;
    int stall = 0;
    int issue = 0;
    int flush = 0;
    int mem_wait = 0;   // every running warp is busy, and every busy warp is in memory
    int exec_wait = 0;  // every running warp is busy, and at least one is in execute
    int fetch_wait = 0;
    int empty_sub_warp = 0;
    int other = 0;

    sm_top u_sm_top (.*);

    // in_mem[w]: warp w's memory instruction has entered the coalescer and not yet completed
    logic [7:0] in_mem = '0;
    always @(posedge sm_clk) begin
        if (u_sm_top.coalescing.fifo_push) in_mem[u_sm_top.coalescing.incoming_warp] <= 1'b1;
        if (u_sm_top.lsu.wb_done) in_mem[u_sm_top.lsu.wb_warp] <= 1'b0;
        for (int w = 0; w < 8; ++w)
            if (u_sm_top.lsu.store_done[w]) in_mem[w] <= 1'b0;
    end

    always @(posedge sm_clk) begin
        if (counting) begin
            cycle_count <= cycle_count + 1;
            instr_count <= instr_count + u_sm_top.consume;

            if (u_sm_top.wb_buffer_full) stall <= stall + 1;
            else if ((|u_sm_top.dec_alu_valid) | (|u_sm_top.dec_fma_valid) | (|u_sm_top.dec_special_reg_valid))
                issue <= issue + 1;
            else if (u_sm_top.dec_sub_warp_valid) empty_sub_warp <= empty_sub_warp + 1;
            else if (u_sm_top.warps_done) flush <= flush + 1;
            else if (&(u_sm_top.u_warp_scheduler.instr_busy | u_sm_top.u_warp_scheduler.finished)) begin
                // every running warp has an instruction in flight
                if (&(in_mem | ~u_sm_top.u_warp_scheduler.instr_busy)) mem_wait <= mem_wait + 1;
                else exec_wait <= exec_wait + 1;
            end
            else if (!u_sm_top.u_fetch.buf_valid) fetch_wait <= fetch_wait + 1;
            else other <= other + 1;
        end

        if (counting && !u_sm_top.wb_buffer_full)
            thread_ops += $countones(u_sm_top.dec_alu_valid)
                        + $countones(u_sm_top.dec_fma_valid)
                        + $countones(u_sm_top.dec_special_reg_valid);
    end

    int mem_txns = 0;
    int mem_instrs = 0;

    always @(posedge sm_clk) begin
        if (counting) begin
            if (u_sm_top.dcache_req_valid && u_sm_top.dcache_req_ready)
                mem_txns <= mem_txns + 1;
            
            if (u_sm_top.coalescing.issue_done)
                mem_instrs <= mem_instrs + 1;
        end
    end

    int dc_access = 0;
    int dc_miss = 0;
    int dc_wb = 0;

    int ic_access = 0;
    int ic_miss = 0;

    always @(posedge sm_clk) begin
        if (counting) begin
            dc_access <= dc_access + (u_sm_top.dcache_req_valid && u_sm_top.dcache_req_ready);
            dc_miss <= dc_miss + (u_sm_top.dcache_mem_req_valid && u_sm_top.dcache_mem_req_ready
                                && !u_sm_top.dcache_mem_req_write);
            // evictions only: dirty victim written back on a miss, flush excluded
            dc_wb <= dc_wb + (u_sm_top.dcache_mem_req_valid && u_sm_top.dcache_mem_req_ready
                            && u_sm_top.dcache_mem_req_write && !u_sm_top.dcache.flushing);

            ic_access <= ic_access + (u_sm_top.icache_req_ready && u_sm_top.icache_req_valid);
            ic_miss <= ic_miss + (u_sm_top.icache_mem_req_valid && u_sm_top.icache_mem_req_ready);
        end
    end

    // Bandwidth: one beat = 16 bytes (MEM_W = 128). Counted on the cache side of the arbiter.
    int dc_rd_beats = 0; // DDR -> D$ fill beats
    int dc_wr_beats = 0; // D$ -> DDR writeback beats, including the end-of-kernel flush
    int ic_rd_beats = 0; // DDR -> I$ fill beats

    // Cycles the coalescer has a request but the blocking D$ can't take it
    int dc_blocked = 0;

    always @(posedge sm_clk) begin
        if (counting) begin
            dc_rd_beats <= dc_rd_beats + u_sm_top.dcache_mem_rvalid;
            dc_wr_beats <= dc_wr_beats + (u_sm_top.dcache_mem_wvalid && u_sm_top.dcache_mem_wready);
            ic_rd_beats <= ic_rd_beats + u_sm_top.icache_mem_rvalid;

            dc_blocked <= dc_blocked + (u_sm_top.dcache_req_valid && !u_sm_top.dcache_req_ready);
        end
    end

    int ld_count = 0;
    int ld_sum = 0;
    int ld_min = 32'h7FFF_FFFF;
    int ld_max = 0;

    int st_count = 0;
    int st_sum = 0;
    int st_min = 32'h7FFF_FFFF;
    int st_max = 0;

    int start [7:0];       // "customer walks in": instruction fully arrived at the coalescer
    int issue_start [7:0]; // "reaches the counter": coalescer starts issuing its D$ requests

    // queue = issue_start - start (waiting behind other warps), service = end - issue_start
    int ld_queue_sum = 0;
    int ld_service_sum = 0;
    int st_queue_sum = 0;
    int st_service_sum = 0;

    always @(posedge sm_clk) begin
        int latency;

        if (counting) begin
            if (u_sm_top.coalescing.fifo_push)
                start[u_sm_top.coalescing.incoming_warp] <= cycle_count;

            // Two ways to reach the counter: popped from the line, or the line was empty
            // and the instruction went straight in (then queue time is 0)
            if (u_sm_top.coalescing.fifo_pop)
                issue_start[u_sm_top.coalescing.ready_fifo[0]] <= cycle_count;
            else if (u_sm_top.coalescing.direct_handoff)
                issue_start[u_sm_top.coalescing.incoming_warp] <= cycle_count;

            if (u_sm_top.lsu.wb_done) begin
                latency = cycle_count - start[u_sm_top.lsu.wb_warp];
                ld_sum <= ld_sum + latency;
                ld_count <= ld_count + 1;

                ld_queue_sum <= ld_queue_sum + (issue_start[u_sm_top.lsu.wb_warp] - start[u_sm_top.lsu.wb_warp]);
                ld_service_sum <= ld_service_sum + (cycle_count - issue_start[u_sm_top.lsu.wb_warp]);

                if (latency < ld_min)
                    ld_min <= latency;
                
                if (latency > ld_max)
                    ld_max <= latency;
            end

            for (int w = 0; w < 8; ++w) begin
                if (u_sm_top.lsu.store_done[w]) begin
                    latency = cycle_count - start[w];
                    st_sum <= st_sum + latency;
                    st_count <= st_count + 1;

                    st_queue_sum <= st_queue_sum + (issue_start[w] - start[w]);
                    st_service_sum <= st_service_sum + (cycle_count - issue_start[w]);

                    if (latency < st_min) 
                        st_min <= latency;
                    
                    if (latency > st_max)
                        st_max <= latency;
                end
            end
        end
    end

    // One D$ fill ("back-room trip"):
    //   t0 = fill request accepted, t1 = first beat back, t2 = last beat back
    //   wait = t1 - t0 (trip to DDR), transfer = t2 - t1 (receiving the 8 beats)
    // The D$ is blocking, so only one fill is ever in flight: no per-warp array needed.
    int fill_count = 0;
    int fill_wait_sum = 0;
    int fill_transfer_sum = 0;
    int fill_wait_min = 32'h7FFF_FFFF;
    int fill_wait_max = 0;
    int fill_transfer_min = 32'h7FFF_FFFF;
    int fill_transfer_max = 0;

    int fill_t0, fill_t1;
    bit fill_waiting_first = 0; // between t0 and the first beat

    always @(posedge sm_clk) begin
        int wait_cycles, transfer_cycles;

        if (counting) begin
            // t0: same event dc_miss counts
            if (u_sm_top.dcache_mem_req_valid && u_sm_top.dcache_mem_req_ready
                && !u_sm_top.dcache_mem_req_write) begin
                fill_t0 <= cycle_count;
                fill_waiting_first <= 1'b1;
            end

            // t1: first beat after t0
            if (u_sm_top.dcache_mem_rvalid && fill_waiting_first) begin
                fill_t1 <= cycle_count;
                fill_waiting_first <= 1'b0;

                wait_cycles = cycle_count - fill_t0;
                fill_wait_sum <= fill_wait_sum + wait_cycles;
                if (wait_cycles < fill_wait_min) fill_wait_min <= wait_cycles;
                if (wait_cycles > fill_wait_max) fill_wait_max <= wait_cycles;
            end

            // t2: last beat. fill_t1 is read before this edge's update, which is fine:
            // a line is 8 beats, so the first and last beat are never in the same cycle
            if (u_sm_top.dcache_mem_rvalid && u_sm_top.dcache_mem_rlast) begin
                transfer_cycles = cycle_count - fill_t1;
                fill_transfer_sum <= fill_transfer_sum + transfer_cycles;
                fill_count <= fill_count + 1;
                if (transfer_cycles < fill_transfer_min) fill_transfer_min <= transfer_cycles;
                if (transfer_cycles > fill_transfer_max) fill_transfer_max <= transfer_cycles;
            end
        end
    end

    task reset_counts;
        thread_ops = 0;
        cycle_count = 0;
        instr_count = 0;

        stall = 0;
        issue = 0;
        flush = 0;
        mem_wait = 0;
        exec_wait = 0;
        fetch_wait = 0;
        empty_sub_warp = 0;
        other = 0;

        mem_txns = 0;
        mem_instrs = 0;

        dc_access = 0;
        dc_miss = 0;
        dc_wb = 0;

        ic_access = 0;
        ic_miss = 0;

        dc_rd_beats = 0;
        dc_wr_beats = 0;
        ic_rd_beats = 0;
        dc_blocked = 0;

        ld_count = 0;
        ld_sum = 0;
        ld_min = 32'h7FFF_FFFF;
        ld_max = 0;

        st_count = 0;
        st_sum = 0;
        st_min = 32'h7FFF_FFFF;
        st_max = 0;

        ld_queue_sum = 0;
        ld_service_sum = 0;
        st_queue_sum = 0;
        st_service_sum = 0;

        fill_count = 0;
        fill_wait_sum = 0;
        fill_transfer_sum = 0;
        fill_wait_min = 32'h7FFF_FFFF;
        fill_wait_max = 0;
        fill_transfer_min = 32'h7FFF_FFFF;
        fill_transfer_max = 0;
        fill_waiting_first = 0;
    endtask

    // fake MIG
    localparam logic [31:0] K = 32'hA5A50000;

    // beat_no = app_addr[25:3]; byte address of the beat = beat_no*16
    function automatic logic [127:0] init_beat(input logic [22:0] beat_no);
        logic [31:0] ba;
        ba = {5'b0, beat_no, 4'b0};
        return {(ba+32'd12)^K, (ba+32'd8)^K, (ba+32'd4)^K, ba^K}; // word 0 in [31:0]
    endfunction

    logic [127:0] mig_mem [logic [22:0]]; // associative array only holding values written to it
    logic [22:0] rq [$];
    logic [22:0] a;

    always_ff @(posedge mem_ui_clk) begin
        if (mem_ui_reset) begin
            rq.delete();
            app_rd_data_valid <= 1'b0;
            app_rdy <= 1'b0;
            app_wdf_rdy <= 1'b0;
        end else begin
            app_rdy <= init_calib_complete && ($urandom_range(0,3) != 0);
            app_wdf_rdy <= init_calib_complete && ($urandom_range(0,3) != 0);
            app_rd_data_valid <= 1'b0;

            if (app_en && app_rdy) begin
                if (app_cmd == 3'b000) begin
                    assert (app_wdf_wren && app_wdf_rdy) else $fatal(1, "MIG write command/data not paired");
                    mig_mem[app_addr[25:3]] = app_wdf_data;
                end else
                    rq.push_back(app_addr[25:3]);
            end

            if (rq.size() > 0 && $urandom_range(0,2) == 0) begin
                a = rq.pop_front();
                app_rd_data <= mig_mem.exists(a) ? mig_mem[a] : init_beat(a);
                app_rd_data_valid <= 1'b1;
            end
        end
    end

    task automatic reset;
        begin
            fork
                begin
                    @(negedge sm_clk);
                    sm_reset = 1'b1;
                    
                    repeat (3) @(negedge sm_clk);
                    sm_reset = 1'b0;
                end
                begin
                    @(negedge mem_ui_clk);
                    mem_ui_reset = 1'b1;

                    repeat (3) @(negedge mem_ui_clk);
                    mem_ui_reset = 1'b0;
                    init_calib_complete = 1'b1;
                end
            join
        end
    endtask

    task automatic launch_kernel(input logic [3:0] warps);
        @(negedge sm_clk);
        num_warps = warps;
        kernel_start = 1'b1;

        @(negedge sm_clk);
        kernel_start = 1'b0;
    endtask

    task automatic wait_for_kernel_done(input int max_cycles);
        for (int i = 0; i < max_cycles; i++) begin
            @(negedge sm_clk);

            if (kernel_done) begin
                $display("Kernel completed after %0d cycles", i + 1);
                return;
            end
        end

        $fatal(1, "Kernel timeout after %0d cycles", max_cycles);
    endtask

    // R-type
    localparam logic [5:0] NOP = 6'b000000, ADD = 6'b000001, SUB = 6'b000010,
                           AND = 6'b000011, OR  = 6'b000100, SLT = 6'b000101,
                           MUL = 6'b000110;

    // I-type
    localparam logic [5:0] LUI  = 6'b001000, ADDI  = 6'b001001, SUBI  = 6'b001010,
                           ANDI = 6'b001011, ORI   = 6'b001100, SLTI  = 6'b001101,
                           LOAD = 6'b001110, STORE = 6'b001111;

    // FMA
    localparam logic [5:0] FMA = 6'b010000;

    // Branch
    localparam logic [5:0] BEQ = 6'b011000, BNE = 6'b011001, BLT = 6'b011010, BGE = 6'b011011;

    // Special reg instructions
    localparam logic [5:0] MOV_TID = 6'b100001, MOV_WID = 6'b100010, MOV_BID = 6'b100100;

    // Shift instructions
    localparam logic [5:0] SLLI = 6'b101000, SRAI = 6'b101001;

    // DONE Instruction
    localparam logic [31:0] DONE_INSTR = 32'hFC00_0000;

    // R-type: opcode | src1 | src2 | dst | unused
    function automatic logic [31:0] encode_r(
        input logic [5:0] opcode,
        input logic [4:0] src1, src2, dst
    );
        return {opcode, src1, src2, dst, 11'b0};
    endfunction

    // I-type: opcode | src1 | dst/src2 | immediate
    function automatic logic [31:0] encode_i(
        input logic [5:0] opcode,
        input logic [4:0] src1, dst,
        input logic [15:0] imm
    );
        return {opcode, src1, dst, imm};
    endfunction

    // FMA: opcode | src1 | src2 | src3 | dst | unused
    function automatic logic [31:0] encode_fma(
        input logic [4:0] src1, src2, src3, dst
    );
        return {FMA, src1, src2, src3, dst, 6'b0};
    endfunction

    // Branch: opcode | src1 | src2 | target_offset | reconv_offset
    function automatic logic [31:0] encode_branch(
        input logic [5:0] opcode,
        input logic [4:0] src1, src2,
        input logic [7:0] target_offset,
        input logic [7:0] reconv_offset
    );
        return {opcode, src1, src2, target_offset, reconv_offset};
    endfunction

    // Special: opcode | dst | unused
    function automatic logic [31:0] encode_special(
        input logic [5:0] opcode,
        input logic [4:0] dst
    );
        return {opcode, dst, 21'b0};
    endfunction

    // shift: opcode | src1 | dst | unused | shift amount
    function automatic logic [31:0] encode_shift(
        input logic [5:0] opcode,
        input logic [4:0] src1,
        input logic [4:0] dst,
        input logic [4:0] shift_amount
    );
        return {opcode, src1, dst, 11'b0, shift_amount};
    endfunction

    task automatic load_instruction(
        input logic [31:0] byte_addr,
        input logic [31:0] instr
    );
        logic [22:0] beat;
        logic [127:0] data;

        beat = byte_addr[26:4];

        data = mig_mem.exists(beat) ? mig_mem[beat] : init_beat(beat);
        data[32 * byte_addr[3:2] +: 32] = instr;

        mig_mem[beat] = data;
    endtask

    // Reads one word of DDR as the fake MIG holds it (written beats, else the init pattern)
    function automatic logic [31:0] read_word(input logic [31:0] byte_addr);
        logic [22:0] beat;
        logic [127:0] data;

        beat = byte_addr[26:4];
        data = mig_mem.exists(beat) ? mig_mem[beat] : init_beat(beat);
        return data[32 * byte_addr[3:2] +: 32];
    endfunction

    generate
        for (genvar lane = 0; lane < 8; lane++) begin : init_rf
            task automatic initialize();
                for (int addr = 0; addr < 1024; addr++) begin
                    u_sm_top.gen_reg_bank[lane].u_reg_file.bram1[addr] = 32'b0;
                    u_sm_top.gen_reg_bank[lane].u_reg_file.bram2[addr] = 32'b0;
                    u_sm_top.gen_reg_bank[lane].u_reg_file.bram3[addr] = 32'b0;
                end
            endtask

            task automatic set_register(
                input logic [2:0] warp,
                input logic [1:0] sw,
                input logic [4:0] reg_num,
                input logic [31:0] value
            );
                logic [9:0] addr;

                addr = {warp, sw, reg_num};

                u_sm_top.gen_reg_bank[lane].u_reg_file.bram1[addr] = value;
                u_sm_top.gen_reg_bank[lane].u_reg_file.bram2[addr] = value;
                u_sm_top.gen_reg_bank[lane].u_reg_file.bram3[addr] = value;
            endtask

            task automatic check_register(
                input logic [2:0] warp,
                input logic [1:0] sw,
                input logic [4:0] reg_num,
                input logic [31:0] expected
            );
                logic [9:0] addr;
                addr = {warp, sw, reg_num};

                assert (
                    u_sm_top.gen_reg_bank[lane].u_reg_file.bram1[addr] === expected &&
                    u_sm_top.gen_reg_bank[lane].u_reg_file.bram2[addr] === expected &&
                    u_sm_top.gen_reg_bank[lane].u_reg_file.bram3[addr] === expected
                )
                    else $fatal(1, "Register mismatch: lane %0d, warp %0d, sw %0d, R%0d",
                                lane, warp, sw, reg_num);
            endtask
        end
    endgenerate

    task automatic initialize_all_lanes();
        init_rf[0].initialize();
        init_rf[1].initialize();
        init_rf[2].initialize();
        init_rf[3].initialize();
        init_rf[4].initialize();
        init_rf[5].initialize();
        init_rf[6].initialize();
        init_rf[7].initialize();
    endtask

    task automatic set_all_lanes(
        input logic [2:0] warp,
        input logic [1:0] sw,
        input logic [4:0] reg_num,
        input logic [31:0] value
    );
        init_rf[0].set_register(warp, sw, reg_num, value);
        init_rf[1].set_register(warp, sw, reg_num, value);
        init_rf[2].set_register(warp, sw, reg_num, value);
        init_rf[3].set_register(warp, sw, reg_num, value);
        init_rf[4].set_register(warp, sw, reg_num, value);
        init_rf[5].set_register(warp, sw, reg_num, value);
        init_rf[6].set_register(warp, sw, reg_num, value);
        init_rf[7].set_register(warp, sw, reg_num, value);
    endtask

    task automatic check_all_lanes(
        input logic [2:0] warp,
        input logic [1:0] sw,
        input logic [4:0] reg_num,
        input logic [31:0] expected
    );
        init_rf[0].check_register(warp, sw, reg_num, expected);
        init_rf[1].check_register(warp, sw, reg_num, expected);
        init_rf[2].check_register(warp, sw, reg_num, expected);
        init_rf[3].check_register(warp, sw, reg_num, expected);
        init_rf[4].check_register(warp, sw, reg_num, expected);
        init_rf[5].check_register(warp, sw, reg_num, expected);
        init_rf[6].check_register(warp, sw, reg_num, expected);
        init_rf[7].check_register(warp, sw, reg_num, expected);
    endtask

    task automatic set_lane_register(
        input logic [2:0] lane,
        input logic [2:0] warp,
        input logic [1:0] sw,
        input logic [4:0] reg_num,
        input logic [31:0] value
    );
        case (lane)
            0 : init_rf[0].set_register(warp, sw, reg_num, value);
            1 : init_rf[1].set_register(warp, sw, reg_num, value);
            2 : init_rf[2].set_register(warp, sw, reg_num, value);
            3 : init_rf[3].set_register(warp, sw, reg_num, value);
            4 : init_rf[4].set_register(warp, sw, reg_num, value);
            5 : init_rf[5].set_register(warp, sw, reg_num, value);
            6 : init_rf[6].set_register(warp, sw, reg_num, value);
            7 : init_rf[7].set_register(warp, sw, reg_num, value);
            default : $fatal(1, "Invalid lane %0d", lane);
        endcase
    endtask

    task automatic check_lane_register(
        input logic [2:0] lane,
        input logic [2:0] warp,
        input logic [1:0] sw,
        input logic [4:0] reg_num,
        input logic [31:0] expected
    );
        case (lane)
            0 : init_rf[0].check_register(warp, sw, reg_num, expected);
            1 : init_rf[1].check_register(warp, sw, reg_num, expected);
            2 : init_rf[2].check_register(warp, sw, reg_num, expected);
            3 : init_rf[3].check_register(warp, sw, reg_num, expected);
            4 : init_rf[4].check_register(warp, sw, reg_num, expected);
            5 : init_rf[5].check_register(warp, sw, reg_num, expected);
            6 : init_rf[6].check_register(warp, sw, reg_num, expected);
            7 : init_rf[7].check_register(warp, sw, reg_num, expected);
            default : $fatal(1, "Invalid lane %0d", lane);
        endcase
    endtask

    function automatic logic [31:0] fp32_value(input int value);
        case (value)
            1:  return 32'h3F800000;
            2:  return 32'h40000000;
            3:  return 32'h40400000;
            4:  return 32'h40800000;
            5:  return 32'h40A00000;
            6:  return 32'h40C00000;
            7:  return 32'h40E00000;
            8:  return 32'h41000000;
            9:  return 32'h41100000;
            10: return 32'h41200000;
            11: return 32'h41300000;
            12: return 32'h41400000;
            13: return 32'h41500000;
            14: return 32'h41600000;
            default: $fatal(1, "Unsupported FP32 value: %0d", value);
        endcase
    endfunction

    // Exact FP32 bits of a non-negative integer below 2^24
    function automatic logic [31:0] int_to_fp32(input int v);
        int p;
        if (v == 0) return 32'h0;
        if (v < 0 || v >= (1 << 24)) $fatal(1, "int_to_fp32: %0d is out of range", v);
        p = $clog2(v + 1) - 1;                                          // index of the top set bit
        return {1'b0, 8'(127 + p), 23'((v << (23 - p)) & 32'h007F_FFFF)}; // drop the hidden 1
    endfunction

    // Mandelbrot reference, Q12 fixed point
    // Returns the step count (max 32) and the final zx, zy the kernel leaves in R14, R15.
    function automatic int mandel_pixel(input int px, py, output int zx, zy);
        int cx, cy, zx2, zy2, zxzy, steps;
        cx = -8192 + px * 307;
        cy = -4096 + py * 1365;
        zx = 0; zy = 0; steps = 0;
        do begin
            zx2 = (zx * zx) >>> 12;
            zy2 = (zy * zy) >>> 12;
            if (zx2 + zy2 > 16384) break;
            zxzy = (zx * zy) >>> 12;
            zy = zxzy + zxzy + cy;
            zx = zx2 - zy2 + cx;
            steps++;
        end while (steps < 32);
        return steps;
    endfunction

    task automatic display_results(input logic [3:0] warps);
        $display("RESULTS");
        $display("Thread-ops: %0d | Thread-ops/cycle: %0.2f", thread_ops, real'(thread_ops)/cycle_count);
        $display("SIMD Efficiency: %0.2f%%", 100.0 * real'(thread_ops) / ((instr_count - warps) * 32));
        $display("\tIssue: %0d", issue);
        $display("\tEmpty sub warps: %0d", empty_sub_warp);
        $display("\tWriteback stalls: %0d", stall);
        $display("\tFetch Stalls: %0d", fetch_wait);
        $display("\tMemory Stalls: %0d", mem_wait);
        $display("\tExecute Stalls: %0d", exec_wait);
        $display("\tFlush: %0d", flush);
        $display("\tOther: %0d\n", other);

        assert (stall + issue + empty_sub_warp + flush + mem_wait + exec_wait + fetch_wait + other == cycle_count)
            else $error("Lost-cycle rows sum to %0d, cycle_count is %0d",
                   stall + issue + empty_sub_warp + flush + mem_wait + exec_wait + fetch_wait + other, cycle_count);
    endtask

    task automatic display_memory_results();
        int total_beats;
        real bytes, bytes_per_cycle, mb_per_s;

        if (dc_access > 0) begin
            $display("Coalescing ratio: %0.2f", real'(mem_txns)/mem_instrs);
            $display("D$ hit rate: %0.2f", real'(dc_access - dc_miss)/dc_access);
            $display("D$ MPKI: %0.2f", real'(dc_miss * 1000)/instr_count);
            $display("D$ writeback (evictions): %0d", dc_wb);
            $display("D$ writeback (flush): %0d", dc_wr_beats / 8 - dc_wb);
            $display("D$ blocked cycles: %0d (%0.1f%% of cycles)\n",
                     dc_blocked, 100.0 * real'(dc_blocked) / cycle_count);
        end

        if (ic_access > 0) begin
            $display("I$ hit rate: %0.2f", real'(ic_access - ic_miss)/ic_access);
            $display("I$ MPKI: %0.2f\n", real'(ic_miss * 1000)/instr_count);
        end

        if (ld_count > 0)
            $display("Load latency:  avg %0.1f | min %0d | max %0d cycles (%0d loads)",
                     real'(ld_sum) / ld_count, ld_min, ld_max, ld_count);
        if (ld_count > 0)
            $display("\tqueue %0.1f + service %0.1f", real'(ld_queue_sum) / ld_count, real'(ld_service_sum) / ld_count);
        if (st_count > 0) begin
            $display("Store latency: avg %0.1f | min %0d | max %0d cycles (%0d stores)",
                     real'(st_sum) / st_count, st_min, st_max, st_count);
            $display("\tqueue %0.1f + service %0.1f", real'(st_queue_sum) / st_count, real'(st_service_sum) / st_count);
        end
        if (fill_count > 0)
            $display("D$ fill: wait avg %0.1f (min %0d, max %0d) + transfer avg %0.1f (min %0d, max %0d) cycles (%0d fills)",
                     real'(fill_wait_sum) / fill_count, fill_wait_min, fill_wait_max,
                     real'(fill_transfer_sum) / fill_count, fill_transfer_min, fill_transfer_max, fill_count);
        $display("");

        total_beats = dc_rd_beats + dc_wr_beats + ic_rd_beats;
        bytes = 16.0 * total_beats;
        bytes_per_cycle = bytes / cycle_count;
        mb_per_s = bytes_per_cycle * 100.0; // 100 MHz sm_clk: bytes/cycle * 1e8 / 1e6

        $display("Bandwidth:");
        $display("\tD$ read:  %0d B (%0.1f MB/s)", 16 * dc_rd_beats, 1600.0 * dc_rd_beats / cycle_count);
        $display("\tD$ write: %0d B (%0.1f MB/s)", 16 * dc_wr_beats, 1600.0 * dc_wr_beats / cycle_count);
        $display("\tI$ read:  %0d B (%0.1f MB/s)", 16 * ic_rd_beats, 1600.0 * ic_rd_beats / cycle_count);
        $display("\tTotal:    %0.0f B | %0.2f B/cycle | %0.1f MB/s @ 100 MHz | %0.1f%% of 1.3 GB/s DDR2 peak",
                 bytes, bytes_per_cycle, mb_per_s, 100.0 * mb_per_s / 1300.0);
        $display("\tBus busy: %0.1f%% of cycles", 100.0 * real'(total_beats) / cycle_count);
        if (instr_count > 0)
            $display("\tBytes/instruction: %0.1f", bytes / instr_count);

        // Each counter pair sees the same events from two places, so they must agree
        assert (dc_access == mem_txns)
            else $error("dc_access (%0d) != mem_txns (%0d)", dc_access, mem_txns);
        assert (dc_rd_beats == 8 * dc_miss)
            else $error("dc_rd_beats (%0d) != 8 * dc_miss (%0d)", dc_rd_beats, 8 * dc_miss);
        assert (ic_rd_beats == 8 * ic_miss)
            else $error("ic_rd_beats (%0d) != 8 * ic_miss (%0d)", ic_rd_beats, 8 * ic_miss);
        assert (dc_wr_beats % 8 == 0 && dc_wr_beats >= 8 * dc_wb)
            else $error("dc_wr_beats (%0d) is not whole lines covering dc_wb (%0d)", dc_wr_beats, dc_wb);
        assert (ld_count + st_count == mem_instrs)
            else $error("ld_count (%0d) + st_count (%0d) != mem_instrs (%0d)", ld_count, st_count, mem_instrs);
        // queue + service is the same span as the total latency, split at issue start
        assert (ld_queue_sum + ld_service_sum == ld_sum && st_queue_sum + st_service_sum == st_sum)
            else $error("queue + service doesn't add up to total latency");
        // every miss is exactly one fill
        assert (fill_count == dc_miss)
            else $error("fill_count (%0d) != dc_miss (%0d)", fill_count, dc_miss);
    endtask

    // Per-kernel expected memory counts, derived by hand from the access traces
    task automatic check_memory_counts(
        input string name,
        input int loads, stores, misses, evictions, flushed,
        input int ic_misses = 1 // I$ lines the kernel spans (128 B each)
    );
        assert (ld_count == loads)
            else $error("%s: ld_count = %0d, expected %0d", name, ld_count, loads);
        assert (st_count == stores)
            else $error("%s: st_count = %0d, expected %0d", name, st_count, stores);
        assert (dc_miss == misses)
            else $error("%s: dc_miss = %0d, expected %0d", name, dc_miss, misses);
        assert (dc_wb == evictions)
            else $error("%s: dc_wb = %0d, expected %0d", name, dc_wb, evictions);
        assert (dc_wr_beats / 8 - dc_wb == flushed)
            else $error("%s: flush writebacks = %0d, expected %0d", name, dc_wr_beats / 8 - dc_wb, flushed);
        assert (ic_miss == ic_misses)
            else $error("%s: ic_miss = %0d, expected %0d (one per I$ line of the kernel)", name, ic_miss, ic_misses);
    endtask
    
    initial begin
        sm_clk = 1'b0; mem_ui_clk = 1'b0;
        sm_reset = 1'b1; mem_ui_reset = 1'b1;

        kernel_start = 1'b0; num_warps = 4'd0;
        
        init_calib_complete = 1'b0; app_rd_data = '0;

        mig_mem.delete();

        load_instruction(32'h00, encode_i(ADDI, 5'd0, 5'd1, 16'd5));
        load_instruction(32'h04, encode_i(ADDI, 5'd0, 5'd2, 16'd7));

        for (int i = 0; i < 200; ++i) begin
            load_instruction(32'(8 + 4 * i), encode_r(ADD, 5'd1, 5'd2, 5'd3));
        end

        load_instruction(32'd808, DONE_INSTR);

        for (int i = 0; i < 9; ++i) begin
            if (i == 1 || i == 2 || i == 4 || i == 8) begin
                reset();
                reset_counts();
                initialize_all_lanes();
                thread_ops = 0; counting = 1;

                launch_kernel(4'(i));
                wait_for_kernel_done(50000);
                counting = 0;
                
                for (int w = 0; w < 4; ++w) begin
                    for (int sw = 0; sw < 4; ++sw) begin
                        check_all_lanes(3'd0, sw, 5'd3, 32'd12);
                    end
                end

                $display("\n\nArithmetic Instructions (%0d warps)", i);
                display_results(i);
            end
        end

        /*
            0x00 MOV_WID R1
            0x04 MOV_TID R2
            0x08 FMA R3, R4, R5, R6
            0x0C MOV_BID R10
            0x10 ADD R7, R8, R9

            // Warp-dependent branch (R11 initialized to 3)
            0x14 BLT R1, R11, target_offset=0x10, recon_offset=0x18

            // Branch not-taken (warps 3 - 7)
            0x18 MOV_TID R12
            0x1C ADD R13, R8, R9
            0x20 BEQ R0, R0, target_offset=0x0C, recon_offset=0x0C

            // Branch taken (warps 0 - 2)
            0x24 FMA R14, R4, R5, R6
            0x28 MOV_WID R15

            // reconvergence point
            0x2C ADD R16, R8, R10

            // Thread-dependent branch (R17 initialized to 16)
            0x30 BGE R2, R17, target_offset=0x10, recon_offset=0x1C

            // Branch not-taken (threads 0-15)
            0x34 MOV_TID R18
            0x38 SUB R19, R8, R10
            0x3C BEQ R0, R0, target_offset=0x10, recon_offset=0x10

            // Branch taken (threads 16-31)
            0x40 FMA R20, R4, R5, R6
            0x44 ADD R21, R8, R9
            0x48 MOV_WID R22

            // reconvergence point
            0x4C FMA R23, R4, R5, R6
            0x50 ADD R24, R8, R10
            0x54 DONE
        */

        for (int i = 0; i < 9; ++i) begin
            if (i == 1 || i == 2 || i == 4 || i == 8) begin
                int expected_ops;

                reset();
                reset_counts();
                mig_mem.delete();
                initialize_all_lanes();

                for (int w = 0; w < i; ++w) begin
                    for (int sw = 0; sw < 4; sw++) begin
                        set_all_lanes(3'(w), 2'(sw), 5'd4, 32'h40000000);      // R4 = 2.0
                        set_all_lanes(3'(w), 2'(sw), 5'd5, 32'h40400000);      // R5 = 3.0
                        set_all_lanes(3'(w), 2'(sw), 5'd6, fp32_value(w + 1)); // R6 = warp_id + 1
                        set_all_lanes(3'(w), 2'(sw), 5'd11, 32'd3);
                        set_all_lanes(3'(w), 2'(sw), 5'd17, 32'd16);

                        for (int lane = 0; lane < 8; ++lane) begin
                            int t;
                            t = sw * 8 + lane;

                            set_lane_register(3'(lane), 3'(w), 2'(sw), 5'd8, 32'(100 + 10*w + t));
                            set_lane_register(3'(lane), 3'(w), 2'(sw), 5'd9, 32'(20 + t));
                        end
                    end
                end

                load_instruction('h00, encode_special(MOV_WID, 5'd1));
                load_instruction('h04, encode_special(MOV_TID, 5'd2));
                load_instruction('h08, encode_fma(5'd4, 5'd5, 5'd6, 5'd3));
                load_instruction('h0C, encode_special(MOV_BID, 5'd10));
                load_instruction('h10, encode_r(ADD, 5'd8, 5'd9, 5'd7));
                load_instruction('h14, encode_branch(BLT, 5'd1, 5'd11, 8'h10, 8'h18));
                load_instruction('h18, encode_special(MOV_TID, 5'd12));
                load_instruction('h1C, encode_r(ADD, 5'd8, 5'd9, 5'd13));
                load_instruction('h20, encode_branch(BEQ, 5'd0, 5'd0, 8'h0C, 8'h0C));
                load_instruction('h24, encode_fma(5'd4, 5'd5, 5'd6, 5'd14));
                load_instruction('h28, encode_special(MOV_WID, 5'd15));
                load_instruction('h2C, encode_r(ADD, 5'd8, 5'd10, 5'd16));
                load_instruction('h30, encode_branch(BGE, 5'd2, 5'd17, 8'h10, 8'h1C));
                load_instruction('h34, encode_special(MOV_TID, 5'd18));
                load_instruction('h38, encode_r(SUB, 5'd8, 5'd10, 5'd19));
                load_instruction('h3C, encode_branch(BEQ, 5'd0, 5'd0, 8'h10, 8'h10));
                load_instruction('h40, encode_fma(5'd4, 5'd5, 5'd6, 5'd20));
                load_instruction('h44, encode_r(ADD, 5'd8, 5'd9, 5'd21));
                load_instruction('h48, encode_special(MOV_WID, 5'd22));
                load_instruction('h4C, encode_fma(5'd4, 5'd5, 5'd6, 5'd23));
                load_instruction('h50, encode_r(ADD, 5'd8, 5'd10, 5'd24));
                load_instruction('h54, DONE_INSTR);

                // Header first, so "Kernel completed" prints under the right run
                $display("\n\nMulti-warp Arithmetic (%0d warps)", i);

                thread_ops = 0; counting = 1;
                launch_kernel(4'(i));
                wait_for_kernel_done(50000);
                counting = 0;

                // Per thread: 6 + (2 taken / 3 not taken) + 2 + 3 + 2 = 15 for warps 0-2, 16 for warps 3-7
                expected_ops = 0;
                for (int w = 0; w < i; ++w)
                    expected_ops += 32 * ((w < 3) ? 15 : 16);

                assert (thread_ops == expected_ops)
                    else $fatal(1, "Multi-warp arithmetic: thread_ops = %0d, expected %0d", thread_ops, expected_ops);

                for (int w = 0; w < i; ++w) begin
                    for (int sw = 0; sw < 4; sw++) begin
                        check_all_lanes(3'(w), 2'(sw), 5'd1, w);
                        check_all_lanes(3'(w), 2'(sw), 5'd3, fp32_value(w + 7));
                        check_all_lanes(3'(w), 2'(sw), 5'd10, w >> 2);
                        check_all_lanes(3'(w), 2'(sw), 5'd14, (w < 3 ? fp32_value(w + 7) : 0));
                        check_all_lanes(3'(w), 2'(sw), 5'd15, (w < 3 ? w : 0));
                        check_all_lanes(3'(w), 2'(sw), 5'd23, fp32_value(w + 7));

                        for (int lane = 0; lane < 8; ++lane) begin
                            int t;
                            t = sw * 8 + lane;
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd2, t);
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd7, 120 + 10 * w + 2 * t);
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd12, (w < 3 ? 0 : t));
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd13, (w < 3 ? 0 : 120 + 10 * w + 2 * t));
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd16, 100 + 10 * w + t + (w >> 2));
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd18, (t >= 16 ? 0 : t));
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd19, (t >= 16 ? 0 : 100 + 10 * w + t - (w >> 2)));
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd20, (t >= 16 ? fp32_value(w + 7) : 0));
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd21, (t >= 16 ? 120 + 10 * w + 2 * t : 0));
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd22, (t >= 16 ? w : 0));
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd24, 100 + 10 * w + t + (w >> 2));
                        end
                    end
                end

                display_results(i);
            end
        end

        /*
            0x00 MOV_WID R1             R1 = warp ID
            0x04 MOV_TID R2             R2 = thread ID
            0x08 ADD R3, R1, R2         R3 = w + t
            0x0C ADDI R4, R0, 2         R4 = 2
            0x10 ANDI R5, R2, 3         R5 = t % 4
            0x14 BLT R5, R4             Branch if t % 4 < 2
            target_offset=0x14, recon_offset=0x20

            // not taken (t % 4 = 2 or 3)
            0x18 ADDI R6, R3, 20        R6 = w + t + 20
            0x1C STORE R6, [R7]
            0x20 LOAD R8, [R7]
            0x24 BEQ R0, R0             Skip taken path
            target_offset=0x10, reconv_offset=0x10

            // taken path (t % 4 = 0 or 1)
            0x28 ADDI R6, R3, 10        R6 = w + t + 10
            0x2C STORE R6, [R7]
            0x30 LOAD R8, [R7]

            // reconvergence point
            0x34 LOAD R10, [R7]         All threads load after reconvergence
            0x38 DONE
        */

        for (int i = 0; i < 9; ++i) begin
            if (i == 1 || i == 2 || i == 4 || i == 8) begin
                reset();
                reset_counts();
                mig_mem.delete();
                initialize_all_lanes();

                load_instruction('h00, encode_special(MOV_WID, 5'd1));
                load_instruction('h04, encode_special(MOV_TID, 5'd2));
                load_instruction('h08, encode_r(ADD, 5'd1, 5'd2, 5'd3));
                load_instruction('h0C, encode_i(ADDI, 5'd0, 5'd4, 16'd2));
                load_instruction('h10, encode_i(ANDI, 5'd2, 5'd5, 16'd3));
                load_instruction('h14, encode_branch(BLT, 5'd5, 5'd4, 8'h14, 8'h20));
                load_instruction('h18, encode_i(ADDI, 5'd3, 5'd6, 16'd20));
                load_instruction('h1C, encode_i(STORE, 5'd7, 5'd6, 16'd0));
                load_instruction('h20, encode_i(LOAD, 5'd7, 5'd8, 16'd0));
                load_instruction('h24, encode_branch(BEQ, 5'd0, 5'd0, 8'h10, 8'h10));
                load_instruction('h28, encode_i(ADDI, 5'd3, 5'd6, 16'd10));
                load_instruction('h2C, encode_i(STORE, 5'd7, 5'd6, 16'd0));
                load_instruction('h30, encode_i(LOAD, 5'd7, 5'd8, 16'd0));
                load_instruction('h34, encode_i(LOAD, 5'd7, 5'd10, 16'd0));
                load_instruction('h38, DONE_INSTR);

                for (int w = 0; w < i; ++w) begin
                    for (int sw = 0; sw < 4; sw++) begin
                        for (int lane = 0; lane < 8; ++lane) begin
                            int t, addr;
                            t = sw * 8 + lane;
                            addr = 32'h1000 + 32'h400 * w + 128 * (t % 4) + 4 * (t / 4);

                            set_lane_register(3'(lane), 3'(w), 2'(sw), 5'd7, addr);
                        end
                    end
                end

                $display("\n\nMulti-warp Memory (%0d warps)", i);

                thread_ops = 0; counting = 1;
                launch_kernel(4'(i));
                wait_for_kernel_done(50000);
                counting = 0;

                // Per thread: 6 + (3 taken / 4 not taken) + 1 = 10 or 11, so 336 per warp
                assert (thread_ops == 336 * i)
                    else $fatal(1, "Multi-warp memory: thread_ops = %0d, expected %0d", thread_ops, 336 * i);

                for (int w = 0; w < i; ++w) begin
                    for (int sw = 0; sw < 4; sw++) begin
                        check_all_lanes(3'(w), 2'(sw), 5'd1, w);
                        check_all_lanes(3'(w), 2'(sw), 5'd4, 32'd2);

                        for (int lane = 0; lane < 8; ++lane) begin
                            int t;
                            t = sw * 8 + lane;
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd2, t);
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd3, w + t);
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd5, t % 4);
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd6, (t % 4 < 2 ? w + t + 10 : w + t + 20));
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd8, (t % 4 < 2 ? w + t + 10 : w + t + 20));
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd10, (t % 4 < 2 ? w + t + 10 : w + t + 20));
                        end
                    end
                end

                // Verify dirty lines were written back during the kernel flush
                for (int w = 0; w < i; ++w) begin
                    for (int t = 0; t < 32; ++t) begin
                        logic [31:0] addr, expected_data;
                        logic [22:0] beat;

                        addr = 32'h1000 + 'h400 * w + 128 * (t % 4) + 4 * (t / 4);
                        beat = addr[26:4];
                        expected_data = w + t + (t % 4 < 2 ? 10 : 20);

                        assert (mig_mem.exists(beat))
                            else $fatal(1, "Multi-warp memory: Missing dirty-line writeback");

                        assert (mig_mem[beat][32*addr[3:2] +: 32] === expected_data)
                            else $fatal(1, "Multi-warp memory: STORE writeback failed");
                    end
                end

                display_results(i);
                display_memory_results();
                // per warp: 3 loads, 2 stores, 4 compulsory misses, no evictions (each used set holds exactly 4 lines), 4 flushed
                check_memory_counts("Multi-warp memory", 3 * i, 2 * i, 4 * i, 0, 4 * i);
            end
        end

        /*
        Full-system kernel (final_integration_tb TEST 10)
        Three levels of nested divergence with INT, MUL, FMA, loads and stores,
        including a store that forces a cache line replacement.

            0x00 MOV_WID R1                 R1 = warp_id
            0x04 MOV_TID R2                 R2 = thread_id
            0x08 ADD R3, R1, R2             R3 = w + t
            0x0C LOAD R4, [R7]              // Input from A

            0x10 ANDI R5, R2, 1             R5 = t % 2
            0x14 BNE R5, R0                 // Odd threads skip outer IF
            target_offset = recon_offset = 0x54

            // outer not-taken branch (if t % 2 == 0)
            0x18 MUL R6, R3, R3             R6 = (w + t)^2
            0x1C STORE R6, [R7]             A becomes dirty

            0x20 ANDI R5, R2, 3             R5 = t % 4
            0x24 BNE R5, R0                 skip middle IF when t % 4 != 0
            target_offset = 0x30, recon_offset = 0x3C

            // middle not-taken branch (t % 4 == 0)
            0x28 STORE R6, [R8]             B
            0x2C STORE R6, [R9]             C
            0x30 STORE R6, [R10]            D

            0x34 ANDI R5, R2, 7             R5 = R2 % 8
            0x38 BNE R5, R0                 t % 8 != 0
            target_offset = recon_offset = 0x14

            // inner not-taken branch (if t % 8 == 0)
            0x3C  FMA R16, R13, R14, R15
            0x40  STORE R16, [R11]          E, force replacement
            0x44  LOAD R17, [R7]            Reload A
            0x48  STORE R17, [R12]          F

            // inner reconvergence point
            0x4C  ADDI R18, R6, 1           R18 = R6 + 1
            0x50  BEQ R0, R0                Skip ELSE
            target_offset = recon_offset = 0x10

            // middle taken branch (t % 4 != 0)
            0x54  MUL R6, R2, R3            R6 = t(w + t)
            0x58  STORE R6, [R8]
            0x5C  LOAD R17, [R8]

            // middle reconvergence point
            0x60  LOAD R19, [R7]
            0x64  ADD R20, R19, R3

            // outer reconvergence point
            0x68  LOAD R21, [R7]
            0x6C  ADD R22, R21, R3
            0x70  STORE R22, [R12]
            0x74  DONE
        */

        for (int i = 0; i < 9; ++i) begin
            if (i == 1 || i == 2 || i == 4 || i == 8) begin
                reset();
                reset_counts();
                mig_mem.delete();
                initialize_all_lanes();

                load_instruction('h00, encode_special(MOV_WID, 5'd1));
                load_instruction('h04, encode_special(MOV_TID, 5'd2));
                load_instruction('h08, encode_r(ADD, 5'd1, 5'd2, 5'd3));
                load_instruction('h0C, encode_i(LOAD, 5'd7, 5'd4, 16'd0));
                load_instruction('h10, encode_i(ANDI, 5'd2, 5'd5, 16'd1));
                load_instruction('h14, encode_branch(BNE, 5'd5, 5'd0, 8'h54, 8'h54));
                load_instruction('h18, encode_r(MUL, 5'd3, 5'd3, 5'd6));
                load_instruction('h1C, encode_i(STORE, 5'd7, 5'd6, 16'd0));
                load_instruction('h20, encode_i(ANDI, 5'd2, 5'd5, 16'd3));
                load_instruction('h24, encode_branch(BNE, 5'd5, 5'd0, 8'h30, 8'h3C));
                load_instruction('h28, encode_i(STORE, 5'd8, 5'd6, 16'd0));
                load_instruction('h2C, encode_i(STORE, 5'd9, 5'd6, 16'd0));
                load_instruction('h30, encode_i(STORE, 5'd10, 5'd6, 16'd0));
                load_instruction('h34, encode_i(ANDI, 5'd2, 5'd5, 16'd7));
                load_instruction('h38, encode_branch(BNE, 5'd5, 5'd0, 8'h14, 8'h14));
                load_instruction('h3C, encode_fma(5'd13, 5'd14, 5'd15, 5'd16));
                load_instruction('h40, encode_i(STORE, 5'd11, 5'd16, 16'd0));
                load_instruction('h44, encode_i(LOAD, 5'd7, 5'd17, 16'd0));
                load_instruction('h48, encode_i(STORE, 5'd12, 5'd17, 16'd0));
                load_instruction('h4C, encode_i(ADDI, 5'd6, 5'd18, 16'd1));
                load_instruction('h50, encode_branch(BEQ, 5'd0, 5'd0, 8'h10, 8'h10));
                load_instruction('h54, encode_r(MUL, 5'd2, 5'd3, 5'd6));
                load_instruction('h58, encode_i(STORE, 5'd8, 5'd6, 16'd0));
                load_instruction('h5C, encode_i(LOAD, 5'd8, 5'd17, 16'd0));
                load_instruction('h60, encode_i(LOAD, 5'd7, 5'd19, 16'd0));
                load_instruction('h64, encode_r(ADD, 5'd19, 5'd3, 5'd20));
                load_instruction('h68, encode_i(LOAD, 5'd7, 5'd21, 16'd0));
                load_instruction('h6C, encode_r(ADD, 5'd21, 5'd3, 5'd22));
                load_instruction('h70, encode_i(STORE, 5'd12, 5'd22, 16'd0));
                load_instruction('h74, DONE_INSTR);

                for (int w = 0; w < i; ++w) begin
                    for (int sw = 0; sw < 4; ++sw) begin
                        for (int lane = 0; lane < 8; ++lane) begin
                            int t;
                            logic [31:0] base_addr;

                            t = sw * 8 + lane;
                            base_addr = 32'h1000 + 128*w + 4*t;

                            // Memory addresses A-F
                            for (int k = 0; k < 6; ++k)
                                set_lane_register(3'(lane), 3'(w), 2'(sw), 5'(7+k), base_addr + 32'h800*k);

                            // FP32 operands
                            set_lane_register(3'(lane), 3'(w), 2'(sw), 5'd13, 32'h40000000);
                            set_lane_register(3'(lane), 3'(w), 2'(sw), 5'd14, 32'h40400000);
                            set_lane_register(3'(lane), 3'(w), 2'(sw), 5'd15, 32'h3F800000);
                        end
                    end
                end

                $display("\n\nFull System (%0d warps)", i);

                thread_ops = 0; counting = 1;
                launch_kernel(4'(i));
                wait_for_kernel_done(50000);
                counting = 0;

                // Per warp: 16 odd threads x 9 + 8 (t % 4 == 2) x 18 + 4 (t % 8 == 0) x 26 + 4 (t % 8 == 4) x 22 = 480
                assert (thread_ops == 480 * i)
                    else $fatal(1, "Full system: thread_ops = %0d, expected %0d", thread_ops, 480 * i);

                // Final register values. Each thread's path depends on t % 2, t % 4 and t % 8.
                for (int w = 0; w < i; ++w) begin
                    for (int sw = 0; sw < 4; ++sw) begin
                        for (int lane = 0; lane < 8; ++lane) begin
                            int t;
                            logic [31:0] sum, initial_val, square, product, final_val;

                            t = sw * 8 + lane;
                            sum = 32'(w + t);
                            initial_val = (32'h1000 + 32'(128 * w) + 32'(4 * t)) ^ K; // A's initial contents
                            square = sum * sum;
                            product = 32'(t) * sum;
                            final_val = (t % 2 == 0) ? square : initial_val;          // A after the even threads' store

                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd1, 32'(w));
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd2, 32'(t));
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd3, sum);
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd4, initial_val);

                            // R5 holds the last ANDI each thread ran
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd5,
                                (t % 2 != 0) ? 32'd1 : (t % 4 == 2) ? 32'd2 : 32'(t % 8));
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd6,
                                (t % 2 != 0) ? 32'd0 : (t % 4 == 2) ? product : square);

                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd16, (t % 8 == 0) ? 32'h40E00000 : 32'd0); // 2.0 * 3.0 + 1.0
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd17,
                                (t % 4 == 2) ? product : (t % 8 == 0) ? square : 32'd0);
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd18, (t % 4 == 0) ? square + 32'd1 : 32'd0);
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd19, (t % 2 == 0) ? square : 32'd0);
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd20, (t % 2 == 0) ? square + sum : 32'd0);
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd21, final_val);
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd22, final_val + sum);
                        end
                    end
                end

                display_results(i);
                display_memory_results();
                // per warp: 5 loads, 8 stores, 7 misses (6 compulsory + A reload), 3 dirty evictions (A, C, B), 3 flushed (D, E, F)
                check_memory_counts("Full system", 5 * i, 8 * i, 7 * i, 3 * i, 3 * i);
            end
        end

        /*
        Kernel A: branch cost.
        Same 202 instructions as the arithmetic kernel, but every other ADD is replaced by
        a uniform BEQ to the next PC. All threads take it, so there is no divergence.
        Compare its cycles against the arithmetic kernel at the same warp count.

        0x000: ADDI r1, r0, 5
        0x004: ADDI r2, r0, 7
        0x008: ADD  r3, r1, r2
        0x00C: BEQ  r0, r0, target_offset=+4, reconverge_offset=+4   // jump to next PC
        ...    (100 ADD/BEQ pairs)
        0x328: DONE
        */

        mig_mem.delete();
        load_instruction(32'h00, encode_i(ADDI, 5'd0, 5'd1, 16'd5));
        load_instruction(32'h04, encode_i(ADDI, 5'd0, 5'd2, 16'd7));

        for (int k = 0; k < 100; ++k) begin
            load_instruction(32'(8 + 8 * k),  encode_r(ADD, 5'd1, 5'd2, 5'd3));
            load_instruction(32'(12 + 8 * k), encode_branch(BEQ, 5'd0, 5'd0, 8'h04, 8'h04));
        end

        load_instruction(32'd808, DONE_INSTR);

        for (int i = 0; i < 9; ++i) begin
            if (i == 1 || i == 2 || i == 4 || i == 8) begin
                reset();
                reset_counts();
                initialize_all_lanes();

                // Header first, so "Kernel completed" prints under the right run
                $display("\n\nBranch Cost: 100 ADD + 100 uniform BEQ (%0d warps)", i);

                thread_ops = 0; counting = 1;
                launch_kernel(4'(i));
                wait_for_kernel_done(50000);
                counting = 0;

                // 202 instructions x 32 threads x i warps (branches count, DONE doesn't)
                assert (thread_ops == 202 * 32 * i)
                    else $fatal(1, "Branch cost: thread_ops = %0d, expected %0d", thread_ops, 202 * 32 * i);

                for (int w = 0; w < i; ++w)
                    for (int sw = 0; sw < 4; ++sw)
                        check_all_lanes(3'(w), 2'(sw), 5'd3, 32'd12);

                display_results(i);
            end
        end

        /*
        Kernel B: divergence cost.
        A loop of N iterations around an if/else. The testbench sets r1 per sub-warp:
        r1 = 0 runs the fall-through path (r3), r1 = 1 runs the taken path (r4).
        Sweep how many sub-warps take the branch: 0 and 4 don't diverge, 1-3 do.

        0x00: ADDI r11, r0, N                                        // loop count
        loop:
        0x04: BNE  r1, r0, target_offset=0x18, reconverge_offset=0x28 // taken -> 0x1C, join at 0x2C
        // fall-through path (5 instructions)
        0x08: ADDI r3, r3, 1
        0x0C: ADDI r3, r3, 1
        0x10: ADDI r3, r3, 1
        0x14: ADDI r3, r3, 1
        0x18: BEQ  r0, r0, target_offset=0x14, reconverge_offset=0x14 // jump to join (0x2C)
        // taken path (4 instructions)
        0x1C: ADDI r4, r4, 1
        0x20: ADDI r4, r4, 1
        0x24: ADDI r4, r4, 1
        0x28: ADDI r4, r4, 1
        join:
        0x2C: ADDI r10, r10, 1
        0x30: BNE  r10, r11, target_offset=-0x2C, reconverge_offset=+4 // back to loop (0x04)
        0x34: DONE
        */

        begin
        localparam int N = 50;
        int expected_ops;

        mig_mem.delete();
        load_instruction(32'h00, encode_i(ADDI, 5'd0, 5'd11, 16'(N)));
        load_instruction(32'h04, encode_branch(BNE, 5'd1, 5'd0, 8'h18, 8'h28));
        load_instruction(32'h08, encode_i(ADDI, 5'd3, 5'd3, 16'd1));
        load_instruction(32'h0C, encode_i(ADDI, 5'd3, 5'd3, 16'd1));
        load_instruction(32'h10, encode_i(ADDI, 5'd3, 5'd3, 16'd1));
        load_instruction(32'h14, encode_i(ADDI, 5'd3, 5'd3, 16'd1));
        load_instruction(32'h18, encode_branch(BEQ, 5'd0, 5'd0, 8'h14, 8'h14));
        load_instruction(32'h1C, encode_i(ADDI, 5'd4, 5'd4, 16'd1));
        load_instruction(32'h20, encode_i(ADDI, 5'd4, 5'd4, 16'd1));
        load_instruction(32'h24, encode_i(ADDI, 5'd4, 5'd4, 16'd1));
        load_instruction(32'h28, encode_i(ADDI, 5'd4, 5'd4, 16'd1));
        load_instruction(32'h2C, encode_i(ADDI, 5'd10, 5'd10, 16'd1));
        load_instruction(32'h30, encode_branch(BNE, 5'd10, 5'd11, 8'hD4, 8'h04));
        load_instruction(32'h34, DONE_INSTR);

        // taken = number of sub-warps with r1 = 1 (the highest sub-warps take the branch)
        for (int taken = 0; taken <= 4; ++taken) begin
            for (int i = 0; i < 9; ++i) begin
                if (i == 1 || i == 2 || i == 4 || i == 8) begin
                    reset();
                    reset_counts();
                    initialize_all_lanes();

                    for (int w = 0; w < i; ++w)
                        for (int sw = 4 - taken; sw < 4; ++sw)
                            set_all_lanes(3'(w), 2'(sw), 5'd1, 32'd1);

                    $display("\n\nDivergence Cost: %0d of 4 sub-warps taken, N = %0d (%0d warps)", taken, N, i);

                    thread_ops = 0; counting = 1;
                    launch_kernel(4'(i));
                    wait_for_kernel_done(200000);
                    counting = 0;

                    // Per thread: 1 setup + N x (BNE + path + ADDI + BNE)
                    // fall-through path = 5 instructions, taken path = 4
                    expected_ops = 0;
                    for (int sw = 0; sw < 4; ++sw)
                        expected_ops += 8 * (1 + N * ((sw >= 4 - taken) ? 7 : 8));
                    expected_ops *= i;

                    assert (thread_ops == expected_ops)
                        else $fatal(1, "Divergence cost: thread_ops = %0d, expected %0d", thread_ops, expected_ops);

                    for (int w = 0; w < i; ++w) begin
                        for (int sw = 0; sw < 4; ++sw) begin
                            check_all_lanes(3'(w), 2'(sw), 5'd10, 32'(N));
                            check_all_lanes(3'(w), 2'(sw), 5'd3, (sw >= 4 - taken) ? 32'd0 : 32'(4 * N));
                            check_all_lanes(3'(w), 2'(sw), 5'd4, (sw >= 4 - taken) ? 32'(4 * N) : 32'd0);
                        end
                    end

                    display_results(i);
                end
            end
        end
        end

        $display("SAXPY Kernel");

        /*
            // element = step * 256 + warp * 32 + thread
            // address = base + 4 * element
            // address = base + step * 1024 + warp * 128 + thread * 4

            // base for x in R1
            // base for y in R2

            0x00 MOV_WID R5                 // R5 = warp (0–7)
            0x04 SLLI R5, R5, 7             // R5 = warp × 128
            0x08 MOV_TID R6                 // R6 = thread (0–31)
            0x0C SLLI R6, R6, 2             // R6 = thread × 4
            0x10 ADD R5, R5, R6             // R5 = warp×128 + thread×4  (this thread's offset at step 0)
            0x14 ADD R1, R1, R5             // R1 = address of x at step 0
            0x18 ADD R2, R2, R5             // R2 = address of y at step 0
            0x1C LUI R3, 0x4000             // R3 = 0x40000000 = 2.0f
            0x20 ADDI R4, R0, 16            // R4 = 16 passes left

            0x24 LOAD R5, 0(R1)             // R5 = x[i]
            0x28 LOAD R6, 0(R2)             // R6 = y[i]
            0x2C FMA R6, R3, R5, R6         // R6 = a × x[i] + y[i]
            0x30 STORE R6, 0(R2)            // y[i] = R6

            0x34 ADDI R1, R1, 1024
            0x38 ADDI R2, R2, 1024
            0x3C SUBI R4, R4, 1

            0x40 BNE R4, R0, LOOP           // jump to next iteration when there are still steps left
                                            // target_offset = -0x1C (0xE4), recon_offset = 0x04

            0x44 DONE
        */

        begin
        localparam int N = 4096;
        localparam int PASSES = 16;
        localparam logic [31:0] X_BASE = 32'h0000_1000; // x: 0x1000 - 0x4FFF
        localparam logic [31:0] Y_BASE = 32'h0000_5000; // y: 0x5000 - 0x8FFF
        int errors;

        for (int i = 0; i < 9; ++i) begin
            if (i == 1 || i == 2 || i == 4 || i == 8) begin
                reset();
                reset_counts();
                mig_mem.delete();
                initialize_all_lanes();

                load_instruction('h00, encode_special(MOV_WID, 5'd5));
                load_instruction('h04, encode_shift(SLLI, 5'd5, 5'd5, 5'd7));
                load_instruction('h08, encode_special(MOV_TID, 5'd6));
                load_instruction('h0C, encode_shift(SLLI, 5'd6, 5'd6, 5'd2));
                load_instruction('h10, encode_r(ADD, 5'd5, 5'd6, 5'd5));
                load_instruction('h14, encode_r(ADD, 5'd5, 5'd1, 5'd1));
                load_instruction('h18, encode_r(ADD, 5'd5, 5'd2, 5'd2));
                load_instruction('h1C, encode_i(LUI, 5'd0, 5'd3, 16'h4000));
                load_instruction('h20, encode_i(ADDI, 5'd0, 5'd4, 16'(PASSES)));
                load_instruction('h24, encode_i(LOAD, 5'd1, 5'd5, 16'd0));            // R5 = x[i]
                load_instruction('h28, encode_i(LOAD, 5'd2, 5'd6, 16'd0));            // R6 = y[i]
                load_instruction('h2C, encode_fma(5'd3, 5'd5, 5'd6, 5'd6));           // R6 = a * x + y
                load_instruction('h30, encode_i(STORE, 5'd2, 5'd6, 16'd0));           // y[i] = R6
                load_instruction('h34, encode_i(ADDI, 5'd1, 5'd1, 16'd1024));
                load_instruction('h38, encode_i(ADDI, 5'd2, 5'd2, 16'd1024));
                load_instruction('h3C, encode_i(SUBI, 5'd4, 5'd4, 16'd1));
                load_instruction('h40, encode_branch(BNE, 5'd4, 5'd0, 8'hE4, 8'h04)); // -> 0x24, join at 0x44
                load_instruction('h44, DONE_INSTR);

                // x[e] = e % 4 + 1, y[e] = (e / 4) % 3 + 1, as FP32. Small integers keep 2x + y exact.
                for (int e = 0; e < N; ++e) begin
                    load_instruction(X_BASE + 32'(4 * e), fp32_value(e % 4 + 1));
                    load_instruction(Y_BASE + 32'(4 * e), fp32_value((e / 4) % 3 + 1));
                end

                // Every thread gets the same bases; the kernel adds its own offset
                for (int w = 0; w < i; ++w) begin
                    for (int sw = 0; sw < 4; ++sw) begin
                        set_all_lanes(3'(w), 2'(sw), 5'd1, X_BASE);
                        set_all_lanes(3'(w), 2'(sw), 5'd2, Y_BASE);
                    end
                end

                $display("\n\nSAXPY (%0d warps)", i);

                thread_ops = 0; counting = 1;
                launch_kernel(4'(i));
                wait_for_kernel_done(200000);
                counting = 0;

                // Per warp: (9 setup + 16 passes x 8) instructions x 32 threads, DONE not counted
                assert (thread_ops == (9 + PASSES * 8) * 32 * i)
                    else $fatal(1, "SAXPY: thread_ops = %0d, expected %0d", thread_ops, (9 + PASSES * 8) * 32 * i);

                // DDR after the flush: y = 2x + y where a warp of this run owns the element, x unchanged
                errors = 0;
                for (int e = 0; e < N; ++e) begin
                    logic [31:0] x_got, y_got, y_exp;
                    int xv, yv;

                    xv = e % 4 + 1;
                    yv = (e / 4) % 3 + 1;
                    y_exp = ((e / 32) % 8 < i) ? fp32_value(2 * xv + yv) : fp32_value(yv);
                    x_got = read_word(X_BASE + 32'(4 * e));
                    y_got = read_word(Y_BASE + 32'(4 * e));

                    if (x_got != fp32_value(xv) || y_got != y_exp) begin
                        if (errors < 5)
                            $error("SAXPY: element %0d: x = %h (expected %h), y = %h (expected %h)",
                                    e, x_got, fp32_value(xv), y_got, y_exp);
                        errors++;
                    end
                end
                if (errors > 0) $fatal(1, "SAXPY: %0d wrong elements in DDR", errors);

                // Registers after the last pass
                for (int w = 0; w < i; ++w) begin
                    for (int sw = 0; sw < 4; ++sw) begin
                        for (int lane = 0; lane < 8; ++lane) begin
                            int t, e_last;
                            logic [31:0] offset;

                            t = sw * 8 + lane;
                            offset = 32'(128 * w + 4 * t);
                            e_last = (PASSES - 1) * 256 + w * 32 + t;

                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd1, X_BASE + offset + 32'(PASSES * 1024));
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd2, Y_BASE + offset + 32'(PASSES * 1024));
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd3, 32'h4000_0000);
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd4, 32'd0);
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd5, fp32_value(e_last % 4 + 1));
                            check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd6,
                                fp32_value(2 * (e_last % 4 + 1) + (e_last / 4) % 3 + 1));
                        end
                    end
                end

                display_results(i);
                display_memory_results();
                // per warp: 32 loads, 16 stores, 32 misses (every x and y line once; STORE y hits).
                // Warp w only uses sets w and w + 8, 8 x + 8 y lines each, so most dirty y lines are
                // evicted during the run and the last ones are flushed.
                check_memory_counts("SAXPY", 32 * i, 16 * i, 32 * i, 12 * i, 4 * i);
            end
        end
        end

        $display("Matrix-Vector Kernel");

        /*
            matrix A (256 x 64) calculations
                // element = column * 256 + warp * 32 + thread
                // address = base + 4 * element
                // address = base + step * 1024 + warp * 128 + thread * 4

            vector x (64 elements) calculations
                // element = column
                // address = base + column * 4

            vector y (256 elements) calculations
                // element = warp * 32 + thread
                // address = base + warp * 128 + thread * 4

            // base for A in R1
            // base for x in R2
            // base for y in R3

            0x00 MOV_WID R6                 // R6 = warp (0–7)
            0x04 SLLI R6, R6, 7             // R6 = warp × 128
            0x08 MOV_TID R7                 // R7 = thread (0–31)
            0x0C SLLI R7, R7, 2             // R7 = thread × 4
            0x10 ADD R6, R6, R7             // R6 = warp×128 + thread×4  (this thread's row offset)
            0x14 ADD R1, R1, R6             // R1 = address of A[r][0]
            0x18 ADD R3, R3, R6             // R3 = address of y[r]  (x gets no offset: all threads read x[c])
            0x1C ADDI R4, R0, 64            // R4 = 64 columns left
            0x20 ADDI R5, R0, 0             // R5 = acc = 0.0f (0x00000000)

        loop:
            0x24 LOAD R9, 0(R1)             // R9 = A[r][c]
            0x28 LOAD R10, 0(R2)            // R10 = x[c]
            0x2C FMA R5, R9, R10, R5        // acc = A[r][c] × x[c] + acc

            0x30 ADDI R1, R1, 1024          // A address → next column (256 rows × 4 B)
            0x34 ADDI R2, R2, 4             // x address → next element
            0x38 SUBI R4, R4, 1             // one fewer column left

            0x3C BNE R4, R0, LOOP           // target_offset = -0x18 (0xE8) → 0x24, recon_offset = 0x04 → 0x40

            0x40 STORE R5, 0(R3)            // y[r] = acc
            0x44 DONE
        */

        // Column-major A: A[r][c] at A_BASE + 4 * (c * 256 + r). One thread per row (r = warp * 32 + thread),
        begin
            localparam int M = 256;                         // rows (one per thread at 8 warps)
            localparam int N = 64;                          // columns
            localparam logic [31:0] A_BASE = 32'h0001_0000; // A: 0x10000 - 0x1FFFF (64 KiB)
            localparam logic [31:0] X_BASE = 32'h0000_2000; // x: 0x2000 - 0x20FF (2 lines)
            localparam logic [31:0] Y_BASE = 32'h0000_3000; // y: 0x3000 - 0x33FF (8 lines)
            localparam logic [31:0] Y_UNTOUCHED = 32'hDEAD_BEEF;
            int errors;

            for (int i = 0; i < 9; ++i) begin
                if (i == 1 || i == 2 || i == 4 || i == 8) begin
                    reset();
                    reset_counts();
                    mig_mem.delete();
                    initialize_all_lanes();

                    load_instruction('h00, encode_special(MOV_WID, 5'd6));
                    load_instruction('h04, encode_shift(SLLI, 5'd6, 5'd6, 5'd7));
                    load_instruction('h08, encode_special(MOV_TID, 5'd7));
                    load_instruction('h0C, encode_shift(SLLI, 5'd7, 5'd7, 5'd2));
                    load_instruction('h10, encode_r(ADD, 5'd6, 5'd7, 5'd6));
                    load_instruction('h14, encode_r(ADD, 5'd1, 5'd6, 5'd1));
                    load_instruction('h18, encode_r(ADD, 5'd3, 5'd6, 5'd3));
                    load_instruction('h1C, encode_i(ADDI, 5'd0, 5'd4, 16'(N)));
                    load_instruction('h20, encode_i(ADDI, 5'd0, 5'd5, 16'd0));           // acc = 0.0f
                    load_instruction('h24, encode_i(LOAD, 5'd1, 5'd9, 16'd0));           // R9 = A[r][c]
                    load_instruction('h28, encode_i(LOAD, 5'd2, 5'd10, 16'd0));          // R10 = x[c]
                    load_instruction('h2C, encode_fma(5'd9, 5'd10, 5'd5, 5'd5));         // acc = A * x + acc
                    load_instruction('h30, encode_i(ADDI, 5'd1, 5'd1, 16'(4 * M)));      // next column of A
                    load_instruction('h34, encode_i(ADDI, 5'd2, 5'd2, 16'd4));           // next x
                    load_instruction('h38, encode_i(SUBI, 5'd4, 5'd4, 16'd1));
                    load_instruction('h3C, encode_branch(BNE, 5'd4, 5'd0, 8'hE8, 8'h04)); // -> 0x24, join at 0x40
                    load_instruction('h40, encode_i(STORE, 5'd3, 5'd5, 16'd0));          // y[r] = acc
                    load_instruction('h44, DONE_INSTR);

                    // A[r][c] = (r + c) % 4 + 1, x[c] = c % 3 + 1
                    for (int c = 0; c < N; ++c) begin
                        load_instruction(X_BASE + 32'(4 * c), int_to_fp32(c % 3 + 1));
                        for (int r = 0; r < M; ++r)
                            load_instruction(A_BASE + 32'(4 * (c * M + r)), int_to_fp32((r + c) % 4 + 1));
                    end
                    for (int r = 0; r < M; ++r)
                        load_instruction(Y_BASE + 32'(4 * r), Y_UNTOUCHED);

                    // Every thread gets the same bases, the kernel adds its own row offset to A and y
                    for (int w = 0; w < i; ++w) begin
                        for (int sw = 0; sw < 4; ++sw) begin
                            set_all_lanes(3'(w), 2'(sw), 5'd1, A_BASE);
                            set_all_lanes(3'(w), 2'(sw), 5'd2, X_BASE);
                            set_all_lanes(3'(w), 2'(sw), 5'd3, Y_BASE);
                        end
                    end

                    $display("\n\nMatrix-Vector (%0d warps)", i);

                    thread_ops = 0; counting = 1;
                    launch_kernel(4'(i));
                    wait_for_kernel_done(400000);
                    counting = 0;

                    // Per warp: (9 setup + 64 passes x 7 + STORE) instructions x 32 threads, DONE not counted
                    assert (thread_ops == (9 + N * 7 + 1) * 32 * i)
                        else $fatal(1, "Matrix-vector: thread_ops = %0d, expected %0d", thread_ops, (9 + N * 7 + 1) * 32 * i);

                    // DDR after the flush: y[r] = sum_c A[r][c] * x[c] for computed rows, untouched otherwise
                    errors = 0;
                    for (int r = 0; r < M; ++r) begin
                        int sum;
                        logic [31:0] y_got, y_exp;

                        sum = 0;
                        for (int c = 0; c < N; ++c)
                            sum += ((r + c) % 4 + 1) * (c % 3 + 1);
                        y_exp = (r / 32 < i) ? int_to_fp32(sum) : Y_UNTOUCHED;
                        y_got = read_word(Y_BASE + 32'(4 * r));

                        if (y_got != y_exp) begin
                            if (errors < 5)
                                $error("Matrix-vector: y[%0d] = %h, expected %h", r, y_got, y_exp);
                            errors++;
                        end
                    end
                    if (errors > 0) $fatal(1, "Matrix-vector: %0d wrong rows of y in DDR", errors);

                    // Registers after the loop
                    for (int w = 0; w < i; ++w) begin
                        for (int sw = 0; sw < 4; ++sw) begin
                            for (int lane = 0; lane < 8; ++lane) begin
                                int t, r, sum;
                                logic [31:0] offset;

                                t = sw * 8 + lane;
                                r = w * 32 + t;
                                offset = 32'(128 * w + 4 * t);
                                sum = 0;
                                for (int c = 0; c < N; ++c)
                                    sum += ((r + c) % 4 + 1) * (c % 3 + 1);

                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd1, A_BASE + offset + 32'(N * 4 * M));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd2, X_BASE + 32'(4 * N));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd3, Y_BASE + offset);
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd4, 32'd0);
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd5, int_to_fp32(sum));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd6, offset);
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd9, int_to_fp32((r + N - 1) % 4 + 1));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd10, int_to_fp32((N - 1) % 3 + 1));
                            end
                        end
                    end

                    display_results(i);
                    display_memory_results();
                    // per warp: 128 loads (64 A + 64 x), 1 store. Misses: all 64 A lines + the y line (write-allocate),
                    // plus x's 2 lines once for the whole run (x is reused by every warp and kept by PLRU).
                    // Warp w's y line sits in set w, which only warp w's own A lines use, so it is never evicted
                    // and goes out in the flush.
                    check_memory_counts("Matrix-vector", 128 * i, 1 * i, 65 * i + 2, 0, 1 * i);
                end
            end
        end

        $display("Mandelbrot Kernel");

        /*
            0x00 MOV_WID R1                     // R1 = warp
            0x04 MOV_TID R2                     // R2 = thread
            0x08 ADD R3, R1, R0                 // R3 = chunk
            
            // constants
            0x0C ADDI R4, R0, 12                // R4 = 12
            0x10 ADDI R5, R0, 40                // R5 = 40
            0x14 ADDI R6, R0, 307               // R6 = 307
            0x18 ADDI R7, R0, 1365              // R7 = 1365
            0x1C ADDI R8, R0, 16384             // R8 = 16384
            0x20 ADDI R9, R0, 32                // R9 = 32

        loop:
            0x24 ANDI R10, R3, 1
            0x28 SLLI R10, R10, 5
            0x2C ADD R10, R10, R2               // R10 = px
            0x30 SRAI R11, R3, 1                // R11 = py

            0x34 BGE R10, R5, skip              // target_offset = 0x6C, recon_offset = 0x6C
            0x38 MUL R12, R10, R6            
            0x3C SUBI R12, R12, 8192            // R12 = cx
            0x40 MUL R13, R11, R7
            0x44 SUBI R13, R13, 4096            // R13 = cy

            0x48 ADDI R14, R0, 0                // R14 = zx = 0
            0x4C ADDI R15, R0, 0                // R15 = zy = 0
            0x50 ADDI R16, R0, 0                // R16 = steps = 0

        continue_steps:
            0x54 MUL R17, R14, R14
            0x58 SRAI R17, R17, 12              // R17 = zx^2
            0x5C MUL R18, R15, R15
            0x60 SRAI R18, R18, 12              // R18 = zy^2
            0x64 ADD R19, R17, R18              // R19 = zx^2 + zy^2
            0x68 BLT R8, R19, break             // target_offset = 0x24, recon_offset = 0x24

            0x6C MUL R20, R14, R15
            0x70 SRAI R20, R20, 12              // R20 = zx * zy
            0x74 SUB R21, R17, R18              // R21 = zx^2 - zy^2
            0x78 ADD R14, R21, R12              // update zx
            0x7C ADD R22, R20, R20              // R22 = (zx * zy) + (zx * zy)
            0x80 ADD R15, R22, R13
            0x84 ADDI R16, R16, 1               // steps++
            0x88 BLT R16, R9, continue_steps    // target_offset = -0x34 (0xCC), recon_offset = 0x04

        break:
            // out[py * 64 + px] = steps;
            0x8C SLLI R23, R11, 6
            0x90 ADD R23, R23, R10          // R23 = py * 64 + px -> element
            0x94 SLLI R23, R23, 2           // R23 = 4 * (py * 64 + px) -> offset
            0x98 ADD R23, R31, R23          // R23 = address in output
            0x9C STORE R16, 0(R23)

        skip:
            // chunk += number of warps (preset in R30)
            0xA0 ADD R3, R3, R30
            0xA4 BLT R3, R4, loop          // target_offset = -0x80 (0x80), recon_offset = 0x04

            0xA8 DONE
        */

        // Image width 40, height 6, 12 chunks of 32 pixels (2 per row). Warp w does chunks w, w + i, w + 2i, ... < 12
        // (stride = number of warps i, preset in R30), so every warp count draws the whole image.
        // Output row-major with rows padded to 64 words: out[py][px] at OUT_BASE + 4 * (py * 64 + px),
        // so chunk k is exactly line k. OUT_BASE is 2048-aligned, so chunk k lands in D$ set k.
        begin
            localparam int WIDTH = 40;
            localparam int HEIGHT = 6;
            localparam int PITCH = 64;                          // words per output row
            localparam int CHUNKS = 12;
            localparam logic [31:0] OUT_BASE = 32'h0000_1000;   // out: 0x1000 - 0x15FF (12 lines)
            localparam logic [31:0] OUT_UNTOUCHED = 32'hFFFF_FFFF; // -1 = never written
            int errors, expected_ops;

            for (int i = 0; i < 9; ++i) begin
                if (i == 1 || i == 2 || i == 4 || i == 8) begin
                    reset();
                    reset_counts();
                    mig_mem.delete();
                    initialize_all_lanes();

                    load_instruction('h00, encode_special(MOV_WID, 5'd1));                  // R1 = warp
                    load_instruction('h04, encode_special(MOV_TID, 5'd2));                  // R2 = thread
                    load_instruction('h08, encode_r(ADD, 5'd1, 5'd0, 5'd3));                // R3 = chunk = warp
                    load_instruction('h0C, encode_i(ADDI, 5'd0, 5'd4, 16'(CHUNKS)));        // R4 = 12
                    load_instruction('h10, encode_i(ADDI, 5'd0, 5'd5, 16'(WIDTH)));         // R5 = 40
                    load_instruction('h14, encode_i(ADDI, 5'd0, 5'd6, 16'd307));            // R6 = x step
                    load_instruction('h18, encode_i(ADDI, 5'd0, 5'd7, 16'd1365));           // R7 = y step
                    load_instruction('h1C, encode_i(ADDI, 5'd0, 5'd8, 16'd16384));          // R8 = 4.0
                    load_instruction('h20, encode_i(ADDI, 5'd0, 5'd9, 16'd32));             // R9 = max steps
                    // loop:
                    load_instruction('h24, encode_i(ANDI, 5'd3, 5'd10, 16'd1));
                    load_instruction('h28, encode_shift(SLLI, 5'd10, 5'd10, 5'd5));
                    load_instruction('h2C, encode_r(ADD, 5'd10, 5'd2, 5'd10));              // R10 = px
                    load_instruction('h30, encode_shift(SRAI, 5'd3, 5'd11, 5'd1));          // R11 = py
                    load_instruction('h34, encode_branch(BGE, 5'd10, 5'd5, 8'h6C, 8'h6C));  // px >= 40 -> skip (0xA0)
                    load_instruction('h38, encode_r(MUL, 5'd10, 5'd6, 5'd12));
                    load_instruction('h3C, encode_i(SUBI, 5'd12, 5'd12, 16'd8192));         // R12 = cx
                    load_instruction('h40, encode_r(MUL, 5'd11, 5'd7, 5'd13));
                    load_instruction('h44, encode_i(SUBI, 5'd13, 5'd13, 16'd4096));         // R13 = cy
                    load_instruction('h48, encode_i(ADDI, 5'd0, 5'd14, 16'd0));             // zx = 0
                    load_instruction('h4C, encode_i(ADDI, 5'd0, 5'd15, 16'd0));             // zy = 0
                    load_instruction('h50, encode_i(ADDI, 5'd0, 5'd16, 16'd0));             // steps = 0
                    // continue_steps:
                    load_instruction('h54, encode_r(MUL, 5'd14, 5'd14, 5'd17));
                    load_instruction('h58, encode_shift(SRAI, 5'd17, 5'd17, 5'd12));        // R17 = zx^2
                    load_instruction('h5C, encode_r(MUL, 5'd15, 5'd15, 5'd18));
                    load_instruction('h60, encode_shift(SRAI, 5'd18, 5'd18, 5'd12));        // R18 = zy^2
                    load_instruction('h64, encode_r(ADD, 5'd17, 5'd18, 5'd19));             // R19 = |z|^2
                    load_instruction('h68, encode_branch(BLT, 5'd8, 5'd19, 8'h24, 8'h24));  // escaped -> break (0x8C)
                    load_instruction('h6C, encode_r(MUL, 5'd14, 5'd15, 5'd20));
                    load_instruction('h70, encode_shift(SRAI, 5'd20, 5'd20, 5'd12));        // R20 = zx * zy
                    load_instruction('h74, encode_r(SUB, 5'd17, 5'd18, 5'd21));             // R21 = zx^2 - zy^2
                    load_instruction('h78, encode_r(ADD, 5'd21, 5'd12, 5'd14));             // zx = R21 + cx
                    load_instruction('h7C, encode_r(ADD, 5'd20, 5'd20, 5'd22));             // R22 = 2 zx zy
                    load_instruction('h80, encode_r(ADD, 5'd22, 5'd13, 5'd15));             // zy = R22 + cy
                    load_instruction('h84, encode_i(ADDI, 5'd16, 5'd16, 16'd1));            // steps++
                    load_instruction('h88, encode_branch(BLT, 5'd16, 5'd9, 8'hCC, 8'h04));  // -> 0x54, join at 0x8C
                    // break:
                    load_instruction('h8C, encode_shift(SLLI, 5'd11, 5'd23, 5'd6));         // py * 64
                    load_instruction('h90, encode_r(ADD, 5'd23, 5'd10, 5'd23));             // + px
                    load_instruction('h94, encode_shift(SLLI, 5'd23, 5'd23, 5'd2));         // x 4 bytes
                    load_instruction('h98, encode_r(ADD, 5'd31, 5'd23, 5'd23));             // + OUT_BASE
                    load_instruction('h9C, encode_i(STORE, 5'd23, 5'd16, 16'd0));           // out[py][px] = steps
                    // skip:
                    load_instruction('hA0, encode_r(ADD, 5'd3, 5'd30, 5'd3));               // chunk += warps
                    load_instruction('hA4, encode_branch(BLT, 5'd3, 5'd4, 8'h80, 8'h04));   // -> 0x24, join at 0xA8
                    load_instruction('hA8, DONE_INSTR);

                    for (int k = 0; k < HEIGHT * PITCH; ++k)
                        load_instruction(OUT_BASE + 32'(4 * k), OUT_UNTOUCHED);

                    for (int w = 0; w < i; ++w)
                        for (int sw = 0; sw < 4; ++sw) begin
                            set_all_lanes(3'(w), 2'(sw), 5'd30, 32'(i));      // stride = number of warps
                            set_all_lanes(3'(w), 2'(sw), 5'd31, OUT_BASE);
                        end

                    $display("\n\nMandelbrot (%0d warps)", i);

                    thread_ops = 0; counting = 1;
                    launch_kernel(4'(i));
                    wait_for_kernel_done(400000);
                    counting = 0;

                    // Per thread: 9 setup, then per chunk 5 (header) + 2 (chunk++ and branch) if skipped,
                    // else 5 + 7 (cx, cy, zeros) + steps loop + 5 (store) + 2. Steps loop: 14 per full pass,
                    // plus 6 for the pass that escapes (none if it ran all 32).
                    expected_ops = 0;
                    for (int w = 0; w < i; ++w) begin
                        for (int t = 0; t < 32; ++t) begin
                            expected_ops += 9;
                            for (int chunk = w; chunk < CHUNKS; chunk += i) begin
                                int px, py, s, zx, zy;
                                px = (chunk & 1) * 32 + t;
                                py = chunk >> 1;
                                if (px >= WIDTH) expected_ops += 7;
                                else begin
                                    s = mandel_pixel(px, py, zx, zy);
                                    expected_ops += 19 + ((s == 32) ? 32 * 14 : 14 * s + 6);
                                end
                            end
                        end
                    end
                    assert (thread_ops == expected_ops)
                        else $fatal(1, "Mandelbrot: thread_ops = %0d, expected %0d", thread_ops, expected_ops);

                    // DDR after the flush: every pixel = its step count, padding (px 40-63) stays untouched
                    errors = 0;
                    for (int py = 0; py < HEIGHT; ++py) begin
                        for (int px = 0; px < PITCH; ++px) begin
                            int zx, zy;
                            logic [31:0] got, exp_val;

                            exp_val = (px < WIDTH) ? 32'(mandel_pixel(px, py, zx, zy)) : OUT_UNTOUCHED;
                            got = read_word(OUT_BASE + 32'(4 * (py * PITCH + px)));

                            if (got !== exp_val) begin
                                if (errors < 5)
                                    $error("Mandelbrot: out[%0d][%0d] = %h, expected %h", py, px, got, exp_val);
                                errors++;
                            end
                        end
                    end
                    if (errors > 0) $fatal(1, "Mandelbrot: %0d wrong pixels in DDR", errors);

                    // The picture as the GPU wrote it ('@' = never escaped, '.' = escaped, ' ' = not computed)
                    for (int py = 0; py < HEIGHT; ++py) begin
                        string line;
                        line = "";
                        for (int px = 0; px < WIDTH; ++px) begin
                            logic [31:0] v;
                            v = read_word(OUT_BASE + 32'(4 * (py * PITCH + px)));
                            line = {line, (v == OUT_UNTOUCHED) ? " " : (v == 32) ? "@" : "."};
                        end
                        $display("%s", line);
                    end

                    // Registers: replay each thread's chunk loop. R10/R11 come from the last chunk it visited,
                    // R12-R16 and R23 from the last pixel it actually computed (0 if it never computed one).
                    for (int w = 0; w < i; ++w) begin
                        for (int sw = 0; sw < 4; ++sw) begin
                            for (int lane = 0; lane < 8; ++lane) begin
                                int t, chunk, px, py, cx, cy, zx, zy, s, addr;

                                t = sw * 8 + lane;
                                cx = 0; cy = 0; zx = 0; zy = 0; s = 0; addr = 0;
                                for (chunk = w; chunk < CHUNKS; chunk += i) begin
                                    px = (chunk & 1) * 32 + t;
                                    py = chunk >> 1;
                                    if (px < WIDTH) begin
                                        cx = -8192 + px * 307;
                                        cy = -4096 + py * 1365;
                                        s = mandel_pixel(px, py, zx, zy);
                                        addr = OUT_BASE + 4 * (py * PITCH + px);
                                    end
                                end

                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd1, 32'(w));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd2, 32'(t));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd3, 32'(chunk));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd4, 32'(CHUNKS));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd5, 32'(WIDTH));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd8, 32'd16384);
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd9, 32'd32);
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd10, 32'(px));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd11, 32'(py));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd12, 32'(cx));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd13, 32'(cy));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd14, 32'(zx));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd15, 32'(zy));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd16, 32'(s));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd23, 32'(addr));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd30, 32'(i));
                                check_lane_register(3'(lane), 3'(w), 2'(sw), 5'd31, OUT_BASE);
                            end
                        end
                    end

                    display_results(i);
                    display_memory_results();
                    // One STORE per chunk (every chunk has at least 8 real pixels), 12 at any warp count, each a
                    // different line in its own set: 1 write-allocate miss each, no evictions, all flushed at the end.
                    // 0xA8 = 2 I$ lines.
                    check_memory_counts("Mandelbrot", 0, CHUNKS, CHUNKS, 0, CHUNKS, 2);
                end
            end
        end

        $finish;
    end
endmodule
