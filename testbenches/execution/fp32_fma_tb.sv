`timescale 1ns/1ps
module fp32_fma_tb;
    // Declare DUT (Design Under Test) inputs and outputs
    logic clk, reset, valid_in;
    logic [31:0] src1, src2, src3;
    logic [31:0] result;
    logic valid_out, buffer_full;
    logic [9:0] reg_bank_addr_in;
    logic [9:0] reg_bank_addr_out;

    integer errors = 0;
    integer tests = 0;

    // Instantiate DUT
    fp32_fma dut (
        .clk(clk),
        .reset(reset),
        .valid_in(valid_in),
        .buffer_full(buffer_full),
        .src1(src1),
        .src2(src2),
        .src3(src3),
        .result(result),
        .valid_out(valid_out),
        .reg_bank_addr_in(reg_bank_addr_in),
        .reg_bank_addr_out(reg_bank_addr_out)
    );

    always #5 clk = ~clk;

    task automatic check_case(
        input [31:0] a, b, c, expected,
        input [9:0] tag
    );
        begin
            @(negedge clk);
            src1 = a; src2 = b; src3 = c;
            reg_bank_addr_in = tag;
            valid_in = 1'b1;

            @(negedge clk);
            valid_in = 1'b0;
            src1 = 0; src2 = 0; src3 = 0;

            wait (valid_out === 1'b1);
            #1;

            tests = tests + 1;
            if (result !== expected || reg_bank_addr_out !== tag) begin
                errors = errors + 1;
                $display("FAIL | a = %h b = %h c = %h  expected = %h got = %h  tag_exp = %h tag_got = %h",
                        a, b, c, expected, result, tag, reg_bank_addr_out);
            end else begin
                $display("PASS | result = %h tag = %h",
                        result, reg_bank_addr_out);
            end

            @(negedge clk);
        end
    endtask

    task automatic check_stall(input integer cycles);
        logic [31:0] held_result;
        logic held_valid;
        logic [9:0] held_addr;

        logic [4:0] held_stage_valids;
        logic [9:0] held_addr_partial, held_addr_mul, held_addr_align;
        logic [9:0] held_addr_add;

        begin
            buffer_full = 1'b1;

            held_result = result;
            held_valid = valid_out;
            held_addr = reg_bank_addr_out;

            held_stage_valids = {
                dut.valid_partial,
                dut.valid_mul_q,
                dut.valid_aligned_q,
                dut.valid_add_q,
                dut.norm_valid_q
            };

            held_addr_partial = dut.reg_bank_addr_partial;
            held_addr_mul = dut.reg_bank_addr_q;
            held_addr_align = dut.reg_bank_addr_aligned_q;
            held_addr_add = dut.reg_bank_addr_add_q;

            repeat (cycles) begin
                @(posedge clk);
                #1;

                tests++;

                if (result !== held_result ||
                    valid_out !== held_valid ||
                    reg_bank_addr_out !== held_addr ||
                    {dut.valid_partial,
                    dut.valid_mul_q,
                    dut.valid_aligned_q,
                    dut.valid_add_q,
                    dut.norm_valid_q} !== held_stage_valids ||
                    dut.reg_bank_addr_partial !== held_addr_partial ||
                    dut.reg_bank_addr_q !== held_addr_mul ||
                    dut.reg_bank_addr_aligned_q !== held_addr_align ||
                    dut.reg_bank_addr_add_q !== held_addr_add) begin

                    errors++;
                    $display("FAIL | FMA pipeline changed while stalled");

                end else begin
                    $display("PASS | FMA pipeline held during stall");
                end
            end

            @(negedge clk);
            buffer_full = 1'b0;
        end
    endtask

    task automatic test_full_pipeline_stall;
        logic [31:0] expected_results [0:4];
        logic [9:0] expected_tags [0:4];

        begin
            $display("\nTEST: Full pipeline stall");

            expected_results[0] = 32'h40F00000; // 7.5
            expected_results[1] = 32'h40900000; // 4.5
            expected_results[2] = 32'hC0900000; // -4.5
            expected_results[3] = 32'hC0B00000; // -5.5
            expected_results[4] = 32'h40D00000; // 6.5

            expected_tags[0] = 10'h201;
            expected_tags[1] = 10'h202;
            expected_tags[2] = 10'h203;
            expected_tags[3] = 10'h204;
            expected_tags[4] = 10'h205;

            // Issue five consecutive operations, one for each pipeline stage:
            // partial -> mul -> aligned -> add -> norm

            @(negedge clk);
            valid_in = 1'b1;
            src1 = 32'h40400000;
            src2 = 32'h40000000;
            src3 = 32'h3FC00000;
            reg_bank_addr_in = expected_tags[0];

            @(negedge clk);
            src3 = 32'hBFC00000;
            reg_bank_addr_in = expected_tags[1];

            @(negedge clk);
            src1 = 32'hC0400000;
            src3 = 32'h3FC00000;
            reg_bank_addr_in = expected_tags[2];

            @(negedge clk);
            src3 = 32'h3F000000;
            reg_bank_addr_in = expected_tags[3];

            @(negedge clk);
            src1 = 32'h40400000;
            reg_bank_addr_in = expected_tags[4];

            @(negedge clk);
            valid_in = 1'b0;
            buffer_full = 1'b1;

            // All five stages now hold an operation, with the first at the output.
            check_stall(4);

            // Verify that all five results emerge in order.
            for (int i = 0; i < 5; i++) begin
                if (i != 0) begin
                    @(posedge clk);
                    #1;
                end

                tests++;

                if (valid_out !== 1'b1 ||
                    result !== expected_results[i] ||
                    reg_bank_addr_out !== expected_tags[i]) begin

                    errors++;
                    $display("FAIL | Operation %0d expected %h tag %h, got %h tag %h",
                            i, expected_results[i], expected_tags[i],
                            result, reg_bank_addr_out);
                end else begin
                    $display("PASS | Operation %0d result %h tag %h",
                            i, result, reg_bank_addr_out);
                end
            end

            @(negedge clk);
        end
    endtask

    initial begin
        $dumpfile("fma.vcd");
        $dumpvars(0, fp32_fma_tb);

        clk = 0;
        reset = 1;
        valid_in = 0;
        buffer_full = 0;
        src1 = 0; src2 = 0; src3 = 0;
        #10 reset = 0;

        // Case 1: 3.0 * 2.0 + 1.5 = 7.5
        // 3.0  = 0_10000000_10000000000000000000000 = 32'h40400000
        // 2.0  = 0_10000000_00000000000000000000000 = 32'h40000000
        // 1.5  = 0_01111111_10000000000000000000000 = 32'h3FC00000
        // 7.5  = 0_10000001_11100000000000000000000 = 32'h40F00000
        check_case(32'h40400000, 32'h40000000, 32'h3FC00000, 32'h40F00000, 10'h001);

        // Case 2: 3.0 * 2.0 + (-1.5) = 4.5
        // 3.0  = 0_10000000_10000000000000000000000 = 32'h40400000
        // 2.0  = 0_10000000_00000000000000000000000 = 32'h40000000
        // -1.5 = 32'hBFC00000
        // 4.5  = 0_10000001_00100000000000000000000 = 32'h40900000
        check_case(32'h40400000, 32'h40000000, 32'hBFC00000, 32'h40900000, 10'h002);

        // Case 3: 1.0 * 1.0 + (-1.0) = 0.0
        // 1.0  = 0_01111111_00000000000000000000000 = 32'h3F800000
        // -1.0 = 1_01111111_00000000000000000000000 = 32'hBF800000
        // 0.0  = 
        check_case(32'h3F800000, 32'h3F800000, 32'hBF800000, 32'h00000000, 10'h003);

        // Case 4: 1.5 * 1.5 + 0.0 = 2.25 (product overflows past 2.0)
        // 2.25 = 0_10000000_00100000000000000000000 = 32'h40100000
        check_case(32'h3FC00000, 32'h3FC00000, 32'h00000000, 32'h40100000, 10'h004);

        // Case 5: src1 negative, src2 positive
        // -3.0 * 2.0 + 1.5 = -6.0 + 1.5 = -4.5
        // -3.0 = 0xC0400000, 2.0 = 0x40000000, 1.5 = 0x3FC00000
        // -4.5 = 1_10000001_00100000000000000000000 = 0xC0900000
        check_case(32'hC0400000, 32'h40000000, 32'h3FC00000, 32'hC0900000, 10'h005);

        // Case 6: src1 positive, src2 negative
        // 3.0 * -2.0 + 1.5 = -6.0 + 1.5 = -4.5 (same result, different sign combo path)
        // -2.0 = 0xC0000000
        check_case(32'h40400000, 32'hC0000000, 32'h3FC00000, 32'hC0900000, 10'h006);

        // Case 7: both multiplicands negative -> product sign flips back to positive
        // -3.0 * -2.0 + 1.5 = 6.0 + 1.5 = 7.5
        check_case(32'hC0400000, 32'hC0000000, 32'h3FC00000, 32'h40F00000, 10'h007);

        // Case 8: both multiplicands negative, addend also negative
        // -3.0 * -2.0 + (-1.5) = 6.0 - 1.5 = 4.5
        // -1.5 = 0xBFC00000
        // 4.5 = 0_10000001_00100000000000000000000 = 0x40900000
        check_case(32'hC0400000, 32'hC0000000, 32'hBFC00000, 32'h40900000, 10'h008);

        // Case 9: negative multiplicand, product bigger than addend, addend positive,
        // -3.0 * 2.0 + 0.5 = -6.0 + 0.5 = -5.5
        // 0.5 = 0x3F000000
        // -5.5 = -(1.375 x 2^2), sign=1, exp=129, mantissa=011000...0
        // -5.5 = 1_10000001_01100000000000000000000 = 0xC0B00000
        check_case(32'hC0400000, 32'h40000000, 32'h3F000000, 32'hC0B00000, 10'h009);

        test_full_pipeline_stall();

        $display("---------------------------------------------");
        $display("Tests: %0d  Errors: %0d", tests, errors);
        if (errors == 0)
            $display("ALL TESTS PASSED");
        else
            $fatal(1, "SOME TESTS FAILED");

        $finish;
    end
endmodule
