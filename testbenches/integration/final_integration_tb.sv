`timescale 1ns/1ps
`default_nettype none

module final_integration_tb;
    logic sm_clk, sm_reset;
    logic mem_ui_clk, mem_ui_reset;

    always #5 sm_clk = ~sm_clk;
    always #6.154 mem_ui_clk = ~mem_ui_clk;
    
    logic kernel_start, kernel_done;
    logic [3:0] num_warps;

    logic init_calib_complete, app_rdy, app_en;
    logic [2:0] app_cmd;
    logic [26:0] app_addr;

    logic app_wdf_rdy, app_wdf_wren, app_wdf_end;
    logic [15:0] app_wdf_mask;
    logic [127:0] app_wdf_data;
    
    logic app_rd_data_valid;
    logic [127:0] app_rd_data;
    logic [3:0] kernel_state_leds;

    sm_top u_sm_top (.*);

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

    // TEST 6: Monitor transaction count sent to memory
    logic count_en = 1'b0;
    int txn_count = 0;
    always_ff @(posedge sm_clk) begin
        if (u_sm_top.txn_valid && count_en)
            txn_count <= txn_count + 1;
    end

    // TEST 9: Monitor
    logic test9_monitor_en = 1'b0;
    int test9_txn_count [0:3];

    always @(posedge sm_clk) begin
        if (test9_monitor_en && u_sm_top.txn_valid) begin
            int w, n, line;
            logic expected_load;
            logic [31:0] expected_addr;
            logic [31:0] expected_mask;
            logic [4:0] expected_dst;
            logic expected_last;

            w = int'(u_sm_top.txn_warp);

            assert (w < 4)
                else $fatal(1, "TEST 9: Unexpected warp %0d", w);

            n = test9_txn_count[w];

            assert (n < 12)
                else $fatal(1, "TEST 9: Too many transactions for warp %0d", w);

            // Expected transaction within each memory instruction
            case (n)
                // Taken path: STORE at 0x2C
                0, 1: begin
                    expected_addr = 32'h1000 + 32'h400 * w + 128 * (n % 2);
                    expected_mask = 32'h1111_1111 << (n % 2);
                    expected_load = 0;
                    expected_dst = 5'd6;
                    expected_last = (n == 1);
                end

                // Taken path: LOAD at 0x30
                2, 3: begin
                    expected_addr = 32'h1000 + 32'h400 * w + 128 * (n % 2);
                    expected_mask = 32'h1111_1111 << (n % 2);
                    expected_load = 1;
                    expected_dst = 5'd8;
                    expected_last = (n == 3);
                end

                // Not-taken path: STORE at 0x1C
                4, 5: begin
                    expected_addr = 32'h1100 + 32'h400 * w + 128 * (n % 2);
                    expected_mask = 32'h4444_4444 << (n % 2);
                    expected_load = 0;
                    expected_dst = 5'd6;
                    expected_last = (n == 5);
                end

                // Not-taken path: LOAD at 0x20
                6, 7: begin
                    expected_addr = 32'h1100 + 32'h400 * w + 128 * (n % 2);
                    expected_mask = 32'h4444_4444 << (n % 2);
                    expected_load = 1;
                    expected_dst = 5'd8;
                    expected_last = (n == 7);
                end

                // Reconverged path: LOAD at 0x34
                8, 9, 10, 11: begin
                    expected_addr = 32'h1000 + 32'h400 * w + 128 * (n - 8);
                    expected_mask = 32'h1111_1111 << (n - 8);
                    expected_load = 1;
                    expected_dst = 5'd10;
                    expected_last = (n == 11);
                end

                default: $fatal(1,
                    "TEST 9: Unexpected transaction %0d for warp %0d", n, w);
            endcase

            if (!expected_load) begin
                assert (u_sm_top.dcache_wr_word_en === 32'h0000_00FF)
                    else $fatal(1, "TEST 9: Incorrect word enables");

                for (int word_idx = 0; word_idx < 8; ++word_idx) begin
                    int t;
                    logic [31:0] expected_data;

                    t = 4*word_idx + (n % 2) + (n >= 4 ? 2 : 0);

                    expected_data = w + t + (n < 2 ? 10 : 20);

                    assert (
                        u_sm_top.dcache_wr_data[32*word_idx +: 32] === expected_data
                    )
                    else $fatal(1,
                        "TEST 9: Warp %0d txn %0d word %0d: got %h, expected %h",
                        w, n, word_idx,
                        u_sm_top.dcache_wr_data[32*word_idx +: 32],
                        expected_data);
                end
            end

            assert (
                u_sm_top.dcache_req_addr === expected_addr &&
                u_sm_top.txn_match_mask === expected_mask &&
                u_sm_top.txn_is_load === expected_load &&
                u_sm_top.txn_dst === expected_dst &&
                u_sm_top.txn_last === expected_last
            )
            else $fatal(1,
                "TEST 9: Warp %0d transaction %0d mismatch: addr=%h expected=%h, mask=%h expected=%h, load=%b expected=%b, dst=%0d expected=%0d, last=%b expected=%b",
                w, n,
                u_sm_top.dcache_req_addr, expected_addr,
                u_sm_top.txn_match_mask, expected_mask,
                u_sm_top.txn_is_load, expected_load,
                u_sm_top.txn_dst, expected_dst,
                u_sm_top.txn_last, expected_last
            );

            test9_txn_count[w] = n + 1;
        end
    end

    function automatic int test10_expected_count(
        input int t,
        input int r
    );
        case (r)
            1, 2, 3, 4, 21, 22: return 1;
            5: return (t % 2 != 0) ? 1 :
                      (t % 4 == 2) ? 2 : 3;

            6: return (t % 2 != 0) ? 0 :
                      (t % 4 == 2) ? 2 : 1;

            16: return (t % 8 == 0) ? 1 : 0;
            17: return (t % 4 == 2 || t % 8 == 0) ? 1 : 0;
            18: return (t % 4 == 0) ? 1 : 0;
            19, 20: return (t % 2 == 0) ? 1 : 0;

            default: return 0;
        endcase
    endfunction

    function automatic logic [31:0] test10_expected_value(
        input int w,
        input int t,
        input int r,
        input int n
    );
        logic [31:0] sum;
        logic [31:0] addr;
        logic [31:0] initial_val;
        logic [31:0] final_val;
        logic [31:0] square;
        logic [31:0] product;

        sum = 32'(w + t);

        addr = 32'h1000 + 32'(128 * w) + 32'(4 * t);
        initial_val = addr ^ K;

        square = sum * sum;
        product = 32'(t) * sum;

        final_val = (t % 2 == 0) ? square : initial_val;

        case (r)
            1: return 32'(w);
            2: return 32'(t);
            3: return sum;
            4: return initial_val;

            5: begin
                case (n)
                    0: return 32'(t % 2);
                    1: return 32'(t % 4);
                    2: return 32'(t % 8);
                    default: return 'x;
                endcase
            end

            6: begin
                case (n)
                    0: return square;
                    1: return product;
                    default: return 'x;
                endcase
            end

            16: return 32'h40E00000;
            17: return (t % 4 == 2) ? product : square;
            18: return square + 32'd1;
            19: return square;
            20: return square + sum;
            21: return final_val;
            22: return final_val + sum;

            default: return 'x;
        endcase
    endfunction

    task automatic test10_check_write_counts();
        int expected_count;
        int total_writes;

        total_writes = 0;

        for (int w = 0; w < 8; ++w) begin
            for (int t = 0; t < 32; ++t) begin
                for (int r = 0; r < 32; ++r) begin
                    expected_count = test10_expected_count(t, r);

                    assert (test10_write_count[w][t][r] == expected_count)
                        else $fatal(1,
                            "TEST 10: Missing writeback: warp=%0d thread=%0d R%0d actual_count=%0d expected_count=%0d",
                            w, t, r,
                            test10_write_count[w][t][r],
                            expected_count
                        );

                    total_writes += test10_write_count[w][t][r];
                end
            end
        end

        assert (total_writes == 2624)
            else $fatal(1,
                "TEST 10: Incorrect total writeback count: actual=%0d expected=2624",
                total_writes
            );
    endtask

    bit test10_reg_monitor_en = 0;

    //                    [warp][thread][register]
    int test10_write_count [0:7][0:31][0:31];

    always @(posedge sm_clk) begin
        if (test10_reg_monitor_en && !sm_reset) begin
            for (int lane = 0; lane < 8; ++lane) begin
                if (u_sm_top.wb_rf_write_enable[lane]) begin
                    int w, t, r, n;
                    int expected_count;
                    logic [31:0] expected_data;
                    logic [31:0] actual_data;

                    w = int'(u_sm_top.wb_rf_write_addr_arr[lane][9:7]);
                    t = 8 * int'(u_sm_top.wb_rf_write_addr_arr[lane][6:5]) + lane;
                    r = int'(u_sm_top.wb_rf_write_addr_arr[lane][4:0]);

                    n = test10_write_count[w][t][r];

                    expected_count = test10_expected_count(t, r);

                    // Reject unexpected or duplicate writes.
                    assert (n < expected_count)
                        else $fatal(1,
                            "TEST 10: Unexpected write: warp=%0d thread=%0d R%0d write_index=%0d expected_count=%0d",
                            w, t, r, n, expected_count
                        );

                    expected_data = test10_expected_value(w, t, r, n);
                    actual_data = u_sm_top.wb_rf_write_data[lane];

                    // Verify writeback data.
                    assert (actual_data === expected_data)
                        else $fatal(1,
                            "TEST 10: Incorrect writeback: warp=%0d thread=%0d R%0d write_index=%0d actual=%h expected=%h",
                            w, t, r, n, actual_data, expected_data
                        );

                    // Count only successfully verified writes.
                    test10_write_count[w][t][r] = n + 1;
                end
            end
        end
    end

    initial begin
        logic [31:0] a [0:3];
        logic [31:0] b [0:3];
        logic [31:0] c [0:3];
        logic [31:0] expected [0:3];

        logic [31:0] addr1, addr2;
        logic [22:0] beat1, beat2;

        $dumpfile("final_integration_tb");
        $dumpvars(0, final_integration_tb);

        sm_clk = 1'b0; mem_ui_clk = 1'b0;
        sm_reset = 1'b1; mem_ui_reset = 1'b1;

        kernel_start = 1'b0; num_warps = 4'd0;
        
        init_calib_complete = 1'b0; app_rd_data = '0;

        reset();

        $display("TEST 1: Single-warp DONE");
        mig_mem.delete();
        load_instruction(32'h0000_0000, DONE_INSTR);

        launch_kernel(4'd1);
        wait_for_kernel_done(500);

        $display("TEST 1 PASSED");

        $display("\nTEST 2: Single-warp arithmetic instructions and register writeback");

        /*
        ADDI R1, R0, 10     // R1 = 10
        ADDI R2, R0, 20     // R2 = 20
        ADD  R3, R1, R2     // R3 = 30
        SUB  R4, R3, R1     // R4 = 20
        MUL  R5, R4, R2     // R5 = 400
        DONE
        */

        reset();
        mig_mem.delete();
        load_instruction('h00, encode_i(ADDI, 0, 1, 10));
        load_instruction('h04, encode_i(ADDI, 0, 2, 20));
        load_instruction('h08, encode_r(ADD, 1, 2, 3));
        load_instruction('h0C, encode_r(SUB, 3, 1, 4));
        load_instruction('h10, encode_r(MUL, 4, 2, 5));
        load_instruction('h14, DONE_INSTR);

        initialize_all_lanes();

        launch_kernel(4'd1);
        wait_for_kernel_done(50000);

        for (int sw = 0; sw < 4; sw++) begin
            check_all_lanes(3'd0, 2'(sw), 5'd1, 32'd10);
            check_all_lanes(3'd0, 2'(sw), 5'd2, 32'd20);
            check_all_lanes(3'd0, 2'(sw), 5'd3, 32'd30);
            check_all_lanes(3'd0, 2'(sw), 5'd4, 32'd20);
            check_all_lanes(3'd0, 2'(sw), 5'd5, 32'd400);
        end

        $display("TEST 2 PASSED");

        // TEST: Single-warp branch divergence and reconvergence

        $display("\nTEST 3: Single-warp branch divergence");
        
        /*
        0x00: BNE  r1, r0, target_offset=0x0C, reconverge_offset=0x24

        // not-taken branch
        0x04: ADDI r2, r0, 22
        0x08: BEQ  r0, r0, target_offset=0x1C, reconverge_offset==0x1C // Unconditional branch to 0x24
        
        // taken branch
        0x0C: BNE R7, R0, target_offset=0x0C, reconverge_offset=0x14,  // reconverge at 0x20

        // inner not-taken branch
        0x10: ADDI r3, r0, 33
        0x14: BEQ R0, R0, target_offset=0x0C, reconverge_offset=0x0C

        // inner taken branch
        0x18: ADDI r4, r0, 44
        0x1C: ADDI r5, r0, 55

        // inner reconervgence
        0x20: ADDI r6, r0, 66

        // outer reconvergence
        0x24: DONE
        */

        reset();
        mig_mem.delete();

        initialize_all_lanes();

        for (int sw = 2; sw < 4; sw++) begin
            set_all_lanes(3'd0, 2'(sw), 5'd1, 32'd1);
        end

        set_all_lanes(3'd0, 2'd3, 5'd7, 32'd1);

        load_instruction(32'h00, encode_branch(BNE, 5'd1, 5'd0, 8'h0C, 8'h24));
        load_instruction(32'h04, encode_i(ADDI, 5'd0, 5'd2, 16'd22));
        load_instruction(32'h08, encode_branch(BEQ, 5'd0, 5'd0, 8'h1C, 8'h1C));
        load_instruction(32'h0C, encode_branch(BNE, 5'd7, 5'd0, 8'h0C, 8'h14));
        load_instruction(32'h10, encode_i(ADDI, 5'd0, 5'd3, 16'd33));
        load_instruction(32'h14, encode_branch(BEQ, 5'd0, 5'd0, 8'h0C, 8'h0C));
        load_instruction(32'h18, encode_i(ADDI, 5'd0, 5'd4, 16'd44));
        load_instruction(32'h1C, encode_i(ADDI, 5'd0, 5'd5, 16'd55));
        load_instruction(32'h20, encode_i(ADDI, 5'd0, 5'd6, 16'd66));
        load_instruction(32'h24, encode_i(ADDI, 5'd0, 5'd8, 16'd88));
        load_instruction(32'h28, DONE_INSTR);

        // Launch one warp and wait for completion
        launch_kernel(4'd1);
        wait_for_kernel_done(50000);

        // Verify branch execution results
        for (int sw = 0; sw < 4; sw++) begin
            check_all_lanes(0, sw, 2, (sw < 2) ? 22 : 0);
            check_all_lanes(0, sw, 3, (sw == 2) ? 33 : 0);
            check_all_lanes(0, sw, 4, (sw == 3) ? 44 : 0);
            check_all_lanes(0, sw, 5, (sw == 3) ? 55 : 0);
            check_all_lanes(0, sw, 6, (sw >= 2) ? 66 : 0);
            check_all_lanes(0, sw, 8, 88);
        end

        $display("TEST 3: Single-warp branch divergence PASSED");

        $display("\nTEST 4: Single-warp FP32 FMA integration");
        reset();
        mig_mem.delete();

        initialize_all_lanes();

        /*
            Kernel:

            0x00: FMA  R4, R1, R2, R3
            0x04: ADDI R5, R0, 42
            0x08: FMA  R6, R7, R8, R9
            0x0C: DONE

            First FMA: Four different calculations across subwarps
            Second FMA: Two additional calculations across subwarps
        */

        // First FMA operands and expected results

        // Subwarp 0: 3.0 * 2.0 + 1.5 = 7.5
        a[0] = 32'h40400000;
        b[0] = 32'h40000000;
        c[0] = 32'h3FC00000;
        expected[0] = 32'h40F00000;

        // Subwarp 1: 3.0 * 2.0 - 1.5 = 4.5
        a[1] = 32'h40400000;
        b[1] = 32'h40000000;
        c[1] = 32'hBFC00000;
        expected[1] = 32'h40900000;

        // Subwarp 2: -3.0 * 2.0 + 1.5 = -4.5
        a[2] = 32'hC0400000;
        b[2] = 32'h40000000;
        c[2] = 32'h3FC00000;
        expected[2] = 32'hC0900000;

        // Subwarp 3: -3.0 * 2.0 + 0.5 = -5.5
        a[3] = 32'hC0400000;
        b[3] = 32'h40000000;
        c[3] = 32'h3F000000;
        expected[3] = 32'hC0B00000;

        // Initialize operands for all 8 lanes across 4 subwarps
        for (int sw = 0; sw < 4; ++sw) begin
            set_all_lanes(3'd0, 2'(sw), 5'd1, a[sw]);
            set_all_lanes(3'd0, 2'(sw), 5'd2, b[sw]);
            set_all_lanes(3'd0, 2'(sw), 5'd3, c[sw]);

            // Second FMA: R6 = R7 * R8 + R9
            // -3.0 * -2.0 + 1.5 = 7.5 (subwarps 0 and 1)
            // -3.0 * -2.0 - 1.5 = 4.5 (subwarps 2 and 3)
            set_all_lanes(3'd0, 2'(sw), 5'd7, 32'hC0400000);
            set_all_lanes(3'd0, 2'(sw), 5'd8, 32'hC0000000);
            set_all_lanes(3'd0, 2'(sw), 5'd9, (sw < 2) ? 32'h3FC00000 : 32'hBFC00000);
        end

        // Load kernel instructions
        load_instruction(32'h00, encode_fma(5'd1, 5'd2, 5'd3, 5'd4));
        load_instruction(32'h04, encode_i(ADDI, 5'd0, 5'd5, 16'd42));
        load_instruction(32'h08, encode_fma(5'd7, 5'd8, 5'd9, 5'd6));
        load_instruction(32'h0C, DONE_INSTR);

        // Launch one warp
        launch_kernel(4'd1);
        wait_for_kernel_done(50000);

        // Verify all 32 threads (8 lanes x 4 subwarps)
        for (int sw = 0; sw < 4; ++sw) begin
            check_all_lanes(3'd0, 2'(sw), 5'd4, expected[sw]); // First FMA
            check_all_lanes(3'd0, 2'(sw), 5'd5, 32'd42); // Integer ALU
            check_all_lanes(3'd0, 2'(sw), 5'd6, (sw < 2) ? 32'h40F00000 : 32'h40900000); // Second FMA
        end

        $display("TEST 4: Single-warp FP32 FMA integration PASSED");

        $display("\nTEST 5: Multi-warp Special register integration");

        reset();
        initialize_all_lanes();

        load_instruction('h00, encode_special(MOV_TID, 5'd3));
        load_instruction('h04, encode_special(MOV_WID, 5'd4));
        load_instruction('h08, encode_special(MOV_BID, 5'd5));
        load_instruction('h0C, DONE_INSTR);

        launch_kernel(4'd5);
        wait_for_kernel_done(50000);

        for (int warp = 0; warp < 5; ++warp) begin
            for (int sw = 0; sw < 4; ++sw) begin
                for (int lane = 0; lane < 8; ++lane) begin
                    // Thread ID -> R3
                    check_lane_register(lane, 3'(warp), 2'(sw), 5'd3, 32'(sw*8 + lane));

                    // Warp ID -> R4
                    check_lane_register(lane, 3'(warp), 2'(sw), 5'd4, 32'(warp));

                    // Block ID -> R5
                    check_lane_register(lane, 3'(warp), 2'(sw), 5'd5, 32'(warp >> 2));
                end
            end
        end

        $display("TEST 5: Multi-warp Special register integration PASSED");

        $display("\nTEST 6: Single-warp LOAD/STORE integration");
        reset();
        mig_mem.delete();

        initialize_all_lanes();

        /*
            LOAD  R4, [R1]       // Cache miss
            STORE R5, [R1 + 4]   // Store hit
            LOAD  R6, [R1 + 4]   // Read after store
            STORE R5, [R2]       // Store miss (write-allocate)
            LOAD  R7, [R2]       // Read after write-allocate
            DONE
        */

        load_instruction('h00, encode_i(LOAD,  1, 4, 0));
        load_instruction('h04, encode_i(STORE, 1, 5, 4));
        load_instruction('h08, encode_i(LOAD,  1, 6, 4));
        load_instruction('h0C, encode_i(STORE, 2, 5, 0));
        load_instruction('h10, encode_i(LOAD,  2, 7, 0));
        load_instruction('h14, DONE_INSTR);

        // Each subwarp accesses a different cache line.
        // All 8 lanes within a subwarp access the same address.
        for (int sw = 0; sw < 4; ++sw) begin
            set_all_lanes(0, 2'(sw), 1, 32'h1000 + sw*128);
            set_all_lanes(0, 2'(sw), 2, 32'h2000 + sw*128);
            set_all_lanes(0, 2'(sw), 5, 32'hABCD_1234 + sw);
        end

        launch_kernel(4'd1);
        wait_for_kernel_done(50000);

        // Verify register writebacks
        for (int sw = 0; sw < 4; ++sw) begin
            check_all_lanes(0, 2'(sw), 4, (32'h1000 + sw*128) ^ K);
            check_all_lanes(0, 2'(sw), 6, 32'hABCD_1234 + sw);
            check_all_lanes(0, 2'(sw), 7, 32'hABCD_1234 + sw);
        end

        // Verify dirty lines were written back during kernel flush
        for (int sw = 0; sw < 4; ++sw) begin
            addr1 = 32'h1004 + sw*128;
            addr2 = 32'h2000 + sw*128;

            beat1 = addr1[26:4];
            beat2 = addr2[26:4];

            assert (mig_mem.exists(beat1) && mig_mem.exists(beat2))
                else $fatal(1, "TEST 6: Missing dirty-line writeback");

            assert (mig_mem[beat1][32*addr1[3:2] +: 32] === (32'hABCD_1234 + sw))
                else $fatal(1, "TEST 6: STORE hit writeback failed");

            assert (mig_mem[beat2][32*addr2[3:2] +: 32] === (32'hABCD_1234 + sw))
                else $fatal(1, "TEST 6: STORE miss writeback failed");
        end

        $display("TEST 6: Single-warp LOAD/STORE integration PASSED");

        $display("\nTEST 7: Single-warp coalesced memory access integration");
        reset();
        mig_mem.delete();
        count_en = 1'b1;

        initialize_all_lanes();

        /*
            LOAD  R2, [R1 + 0]
            DONE
        */

        load_instruction('h00, encode_i(LOAD, 1, 2, 0));
        load_instruction('h04, DONE_INSTR);

        for (int lane = 0; lane < 8; ++lane) begin
            for (int sw = 0; sw < 4; ++sw) begin
                if (sw < 2)
                    addr1 = 32'h1000 + 4 * (sw * 8 + lane);
                else if (sw == 2) 
                    addr1 = 32'h2000 + 4 * (sw * 8 + lane);
                else
                    addr1 = 32'h3000 + 4 * (sw * 8 + lane);

                set_lane_register(lane, 2'd0, 2'(sw), 5'd1, addr1);
            end
        end

        launch_kernel(4'd1);
        wait_for_kernel_done(50000);

        assert (txn_count === 'd3)
            else $fatal("Invalid number of coalesced transactions");

        for (int lane = 0; lane < 8; ++lane) begin
            for (int sw = 0; sw < 4; ++sw) begin
                if (sw < 2)
                    addr1 = 32'h1000 + 4 * (sw * 8 + lane);
                else if (sw == 2) 
                    addr1 = 32'h2000 + 4 * (sw * 8 + lane);
                else
                    addr1 = 32'h3000 + 4 * (sw * 8 + lane);
                
                check_lane_register(lane, 3'd0, 2'(sw), 5'd2, addr1 ^ K);
            end
        end

        $display("TEST 7: Single-warp coalesced memory access PASSED");

        $display("\nTEST 8: Multi-warp arithmetic integration test");
        reset();
        mig_mem.delete();

        initialize_all_lanes();

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

            // Branch taken (warps 0 -o 2)
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

        for (int w = 0; w < 8; ++w) begin
            for (int sw = 0; sw < 4; sw++) begin
                set_all_lanes(3'(w), 2'(sw), 5'd4, 32'h40000000); // R4 = 2.0 for all threads in all warps
                set_all_lanes(3'(w), 2'(sw), 5'd5, 32'h40400000); // R5 = 3.0 for all threads in all warps
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

        launch_kernel(4'd8);
        wait_for_kernel_done(50000);

        for (int w = 0; w < 8; ++w) begin
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
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd2, t);
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd7, 120 + 10 * w + 2 * t);
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd12, (w < 3 ? 0 : t));
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd13, (w < 3 ? 0 : 120 + 10 * w + 2 * t));
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd16, 100 + 10 * w + t + (w >> 2));
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd18, (t >= 16 ? 0 : t));
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd19, (t >= 16 ? 0 : 100 + 10 * w + t - (w >> 2)));
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd20, (t >= 16 ? fp32_value(w + 7) : 0));
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd21, (t >= 16 ? 120 + 10 * w + 2 * t : 0));
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd22, (t >= 16 ? w : 0));
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd24, 100 + 10 * w + t + (w >> 2));
                end
            end
        end

        $display("TEST 8: Multi-warp arithmetic integration test PASSED");

        $display("\nTEST 9: Multi-warp memory integration test");
        reset();
        mig_mem.delete();
        initialize_all_lanes();

        /*
            0x00 MOV_WID R1             R1 = warp ID
            0x04 MOV_TID R2             R2 = thread ID
            0x08 ADD R3, R1, R2         R3 = w + t
            0x0C ADDI R4, R0, 2         R4 = 2
            0x10 ANDI R5, R2, 3         R5 = t % 4
            0x14 BLT R5, R4     	    Branch if t % 4 < 2
            target_offset=0x14, recon_offset=0x20

            // not taken (t % 4 = 2 or 3)
            0x18 ADDI R6, R3, 20        R6 = w + t + 20
            0x1C STORE R6, [R7]
            0x20 LOAD R8, [R7]
            0x24 BEQ R0, R0     	    Skip taken path
            target_offset=0x10, reconv_offset=0x10

            // taken path (t % 4 = 0 or 1)
            0x28 ADDI R6, R3, 10        R6 = w + t + 10
            0x2C STORE R6, [R7]
            0x30 LOAD R8, [R7]

            // reconvergence point
            0x34 LOAD R10, [R7]	All threads load after reconvergence
            0x38 DONE	End kernel
        */

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

        for (int w = 0; w < 4; ++w) begin
            for (int sw = 0; sw < 4; sw++) begin
                for (int lane = 0; lane < 8; ++lane) begin
                    int t, addr;
                    t = sw * 8 + lane;
                    addr = 32'h1000 + 32'h400 * w + 128 * (t % 4) + 4 * (t / 4);

                    set_lane_register(3'(lane), 3'(w), 2'(sw), 5'd7, addr);
                end
            end
        end

        check_all_lanes(3'd0, 2'd0, 5'd0, 32'd0);

        test9_monitor_en = 1'b1;
        for (int w = 0; w < 4; ++w)
            test9_txn_count[w] = 0;

        launch_kernel(4'd4);
        wait_for_kernel_done(50000);

        test9_monitor_en = 1'b0;

        for (int w = 0; w < 4; ++w)
            assert (test9_txn_count[w] == 12)
                else $error("Warp %0d: Expected 12 transactions, got %0d", w, test9_txn_count[w]);

        for (int w = 0; w < 4; ++w) begin
            for (int sw = 0; sw < 4; sw++) begin
                check_all_lanes(3'(w), 2'(sw), 5'd1, w);
                check_all_lanes(3'(w), 2'(sw), 5'd4, 32'd2);

                for (int lane = 0; lane < 8; ++lane) begin
                    int t;
                    t = sw * 8 + lane;
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd2, t);
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd3, w + t);
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd5, t % 4);
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd6, (t % 4 < 2 ? w + t + 10 : w + t + 20));
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd8, (t % 4 < 2 ? w + t + 10 : w + t + 20));
                    check_lane_register(lane, 3'(w), 2'(sw), 5'd10, (t % 4 < 2 ? w + t + 10 : w + t + 20));
                end
            end
        end

        for (int w = 0; w < 4; ++w) begin
            for (int t = 0; t < 32; ++t) begin
                logic [31:0] addr, expected_data;
                logic [22:0] beat;

                addr = 32'h1000 + 'h400 * w + 128 * (t % 4) + 4 * (t / 4);
                beat = addr[26:4];

                expected_data = w + t + (t % 4 < 2 ? 10 : 20);

                assert (mig_mem.exists(beat))
                    else $fatal(1, "TEST 9: Missing dirty-line writeback");

                assert (mig_mem[beat][32*addr[3:2] +: 32] === (expected_data))
                    else $fatal(1, "TEST 9: STORE hit writeback failed");
            end
        end

        $display("TEST 9: Multi-warp memory integration test PASSED");

        $display("\nTEST 10: Multi-warp FULL SYSTEM integration test");
        reset();
        mig_mem.delete();
        initialize_all_lanes();

        /*
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

        for (int w = 0; w < 8; ++w) begin
            for (int sw = 0; sw < 4; ++sw) begin
                for (int lane = 0; lane < 8; ++lane) begin
                    int t;
                    logic [31:0] base_addr;

                    t = sw * 8 + lane;
                    base_addr = 32'h1000 + 128*w + 4*t;

                    // Memory addresses
                    for (int k = 0; k < 6; ++k)
                        set_lane_register(lane, 3'(w), 2'(sw), 5'(7+k), base_addr + 32'h800*k);

                    // FP32 operands
                    set_lane_register(lane, 3'(w), 2'(sw), 5'd13, 32'h40000000);
                    set_lane_register(lane, 3'(w), 2'(sw), 5'd14, 32'h40400000);
                    set_lane_register(lane, 3'(w), 2'(sw), 5'd15, 32'h3F800000);
                end
            end
        end

        for (int w = 0; w < 8; ++w)
            for (int t = 0; t < 32; ++t)
                for (int r = 0; r < 32; ++r)
                    test10_write_count[w][t][r] = 0;

        test10_reg_monitor_en = 1'b1;

        launch_kernel(4'd8);
        wait_for_kernel_done(50000);

        test10_reg_monitor_en = 1'b0;

        test10_check_write_counts();

        $display("TEST 10: Multi-warp FULL SYSTEM integration test PASSED");

        $finish;
    end

    initial begin
        #10_000_000;
        $fatal("TIMEOUT");
    end
endmodule
