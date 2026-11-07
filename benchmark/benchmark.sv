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
    int empty_sub_warp = 0;
    int other = 0;

    sm_top u_sm_top (.*);

    always @(posedge sm_clk) begin
        if (counting) begin
            cycle_count <= cycle_count + 1;

            if (u_sm_top.wb_buffer_full) stall <= stall + 1;
            else if ((|u_sm_top.dec_alu_valid) | (|u_sm_top.dec_fma_valid) | (|u_sm_top.dec_special_reg_valid))
                issue <= issue + 1;
            else if (u_sm_top.dec_sub_warp_valid) empty_sub_warp <= empty_sub_warp + 1;
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
        empty_sub_warp = 0;
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

    task display_results;
        $display("RESULTS");
        $display("Thread-ops: %0d | Thread-ops/cycle: %0.2f", thread_ops, real'(thread_ops)/cycle_count);
        $display("\tIssue: %0d", issue);
        $display("\tEmpty sub warps: %0d", empty_sub_warp);
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
                
                for (int w = 0; w < 4; ++w) begin
                    for (int sw = 0; sw < 4; ++sw) begin
                        check_all_lanes(3'd0, sw, 5'd3, 32'd12);
                    end
                end

                $display("\n\nArithmetic Instructions (%0d warps)", i);
                display_results();
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

                display_results();
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

                display_results();
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

                display_results();
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

                display_results();
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

                        display_results();
                    end
                end
            end
        end

        $finish;
    end
endmodule
