`timescale 1ns/1ps
module performance;
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

    int stall = 0;
    int issue = 0;
    int flush = 0;
    int commit_wait = 0;
    int fetch_wait = 0;
    int other = 0;

    sm_top u_sm_top (.*);

    always @(posedge sm_clk) begin
        if (counting) begin
            cycle_count <= cycle_count + 1;

            if (u_sm_top.wb_buffer_full) stall <= stall + 1;
            else if ((|u_sm_top.dec_alu_valid) | (|u_sm_top.dec_fma_valid) | (|u_sm_top.dec_special_reg_valid))
                issue <= issue + 1;
            else if (u_sm_top.warps_done) flush <= flush + 1;
            else if (|u_sm_top.u_warp_scheduler.instr_busy) commit_wait <= commit_wait + 1;
            else if (!u_sm_top.u_fetch.buf_valid) fetch_wait <= fetch_wait + 1;
            else other <= other + 1;
        end

        if (counting && !u_sm_top.wb_buffer_full)
            thread_ops += $countones(u_sm_top.dec_alu_valid)
                        + $countones(u_sm_top.dec_fma_valid)
                        + $countones(u_sm_top.dec_special_reg_valid);
    end

    task reset_counts;
        thread_ops = 0;
        cycle_count = 0;

        stall = 0;
        issue = 0;
        flush = 0;
        commit_wait = 0;
        fetch_wait = 0;
        other = 0;
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

    task display_results;
        $display("RESULTS");
        $display("Thread-ops: %0d | Thread-ops/cycle: %0.2f", thread_ops, real'(thread_ops)/cycle_count);
        $display("\tIssue: %0d", issue);
        $display("\tWriteback stalls: %0d", stall);
        $display("\tFetch Stalls: %0d", fetch_wait);
        $display("\tCommit Stalls: %0d", commit_wait);
        $display("\tFlush: %0d", flush);
        $display("\tOther: %0d\n", other);
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

                for (int sw = 0; sw < 4; ++sw) begin
                    check_all_lanes(3'd0, sw, 5'd3, 32'd12);
                end

                $display("\n\nArithmetic Instructions (%0d warps)", i);
                display_results();
            end
        end

        /*
            LOAD  R4, [R1]       // Cache miss
            STORE R5, [R1 + 4]   // Store hit
            LOAD  R6, [R1 + 4]   // Read after store
            STORE R5, [R2]       // Store miss (write-allocate)
            LOAD  R7, [R2]       // Read after write-allocate
            DONE
        */

        for (int i = 0; i < 9; ++i) begin
            if (i == 1 || i == 2 || i == 4 || i == 8) begin
                logic [31:0] addr1, addr2, store_val;
                logic [22:0] beat1, beat2;

                reset();
                reset_counts();
                mig_mem.delete();

                load_instruction('h00, encode_i(LOAD,  1, 4, 0));
                load_instruction('h04, encode_i(STORE, 1, 5, 4));
                load_instruction('h08, encode_i(LOAD,  1, 6, 4));
                load_instruction('h0C, encode_i(STORE, 2, 5, 0));
                load_instruction('h10, encode_i(LOAD,  2, 7, 0));
                load_instruction('h14, DONE_INSTR);

                // Each subwarp accesses a different cache line, and each warp its own 4 lines.
                // All 8 lanes within a subwarp access the same address.
                // At 8 warps this fills all 64 D$ lines (16 sets x 4 ways) without evicting.
                initialize_all_lanes();
                for (int w = 0; w < i; ++w) begin
                    for (int sw = 0; sw < 4; ++sw) begin
                        set_all_lanes(3'(w), 2'(sw), 1, 32'h1000 + w*512 + sw*128);
                        set_all_lanes(3'(w), 2'(sw), 2, 32'h2000 + w*512 + sw*128);
                        set_all_lanes(3'(w), 2'(sw), 5, 32'hABCD_0000 + w*256 + sw);
                    end
                end

                thread_ops = 0; counting = 1;

                launch_kernel(4'(i));
                wait_for_kernel_done(50000);
                counting = 0;

                // 5 instructions x 32 threads x i warps (DONE doesn't count)
                assert (thread_ops == 5 * 32 * i)
                    else $fatal(1, "Load/store: thread_ops = %0d, expected %0d", thread_ops, 5 * 32 * i);

                for (int w = 0; w < i; ++w) begin
                    for (int sw = 0; sw < 4; ++sw) begin
                        store_val = 32'hABCD_0000 + w*256 + sw;

                        // Verify register writebacks
                        check_all_lanes(3'(w), 2'(sw), 4, (32'h1000 + w*512 + sw*128) ^ K);
                        check_all_lanes(3'(w), 2'(sw), 6, store_val);
                        check_all_lanes(3'(w), 2'(sw), 7, store_val);

                        // Verify dirty lines were written back during kernel flush
                        addr1 = 32'h1004 + w*512 + sw*128;
                        addr2 = 32'h2000 + w*512 + sw*128;

                        beat1 = addr1[26:4];
                        beat2 = addr2[26:4];

                        assert (mig_mem.exists(beat1) && mig_mem.exists(beat2))
                            else $fatal(1, "Load/store: missing dirty-line writeback");

                        assert (mig_mem[beat1][32*addr1[3:2] +: 32] === store_val)
                            else $fatal(1, "Load/store: STORE hit writeback failed");

                        assert (mig_mem[beat2][32*addr2[3:2] +: 32] === store_val)
                            else $fatal(1, "Load/store: STORE miss writeback failed");
                    end
                end

                $display("\n\nLoad/Store Instructions (%0d warps)", i);
                display_results();
            end
        end

        $finish;
    end
endmodule