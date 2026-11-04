`default_nettype none
`timescale 1ns/1ps
module fp32_fma(
    input logic clk,
    input logic reset,
    input logic valid_in,
    input logic buffer_full,

    // Input source registers
    input logic [31:0] src1,
    input logic [31:0] src2,
    input logic [31:0] src3,

    // Input warp_id, thread_slot, and dest_register for writeback_arbiter
    input logic [9:0] reg_bank_addr_in,

    // Result
    output logic [31:0] result,
    output logic valid_out,

    // Output warp_id, thread_slot, and dest_register for writeback_arbiter
    // This is simply the input passed to the output
    output logic [9:0] reg_bank_addr_out
);
    // unpack sources into sign, exponent, and mantissa

    logic sign1, sign2, sign3;
    logic [23:0] mant1, mant2, mant3;
    logic [7:0] exp1, exp2, exp3;

    assign sign1 = src1[31];
    assign sign2 = src2[31];
    assign sign3 = src3[31];

    assign exp1 = src1[30:23];
    assign exp2 = src2[30:23];
    assign exp3 = src3[30:23];

    assign mant1 = {1'b1, src1[22:0]};
    assign mant2 = {1'b1, src2[22:0]};
    assign mant3 = {1'b1, src3[22:0]};

    // Multiply src1 and src2
    logic [23:0] a_low_b_low, a_low_b_high, a_high_b_low, a_high_b_high;
    logic [8:0] exp_partial;
    logic sign_partial;

    logic valid_partial;
    logic [9:0] reg_bank_addr_partial;

    logic [23:0] mant3_partial;
    logic [7:0] exp3_partial;
    logic sign3_partial;

    always_ff @(posedge clk) begin
        if (reset) begin
            a_low_b_low <= '0;
            a_low_b_high <= '0;
            a_high_b_low <= '0;
            a_high_b_high <= '0;
            exp_partial <= '0;
            sign_partial <= '0;
            valid_partial <= '0;
            reg_bank_addr_partial <= '0;
            mant3_partial <= '0;
            exp3_partial <= '0;
            sign3_partial <= 1'b0;
        end else if (!buffer_full) begin
            a_low_b_low <= mant1[11:0] * mant2[11:0];
            a_low_b_high <= mant1[11:0] * mant2[23:12];
            a_high_b_low <= mant1[23:12] * mant2[11:0];
            a_high_b_high <= mant1[23:12] * mant2[23:12];

            exp_partial <= exp1 + exp2 - 127;
            sign_partial <= sign1 ^ sign2;

            valid_partial <= valid_in;
            reg_bank_addr_partial <= reg_bank_addr_in;

            mant3_partial <= mant3;
            exp3_partial <= exp3;
            sign3_partial <= sign3;
        end
    end

    logic valid_mul_q;
    logic [47:0] mant_mul_q;
    logic [8:0] exp_mul_q;
    logic sign_mul_q;
    logic [9:0] reg_bank_addr_q;
    logic [23:0] mant3_q;
    logic [7:0] exp3_q;
    logic sign3_q;

    always_ff @(posedge clk) begin
        if (reset) begin
            valid_mul_q <= 1'b0;
            mant_mul_q <= 48'b0;
            exp_mul_q <= 9'b0;
            sign_mul_q <= 1'b0;
            reg_bank_addr_q <= 10'b0;
            mant3_q <= 24'b0;
            exp3_q <= 8'b0;
            sign3_q <= 1'b0;
        end else if (!buffer_full) begin
            valid_mul_q <= valid_partial;
            mant_mul_q <= {24'b0, a_low_b_low} +
                        {12'b0, a_low_b_high, 12'b0} +
                        {12'b0, a_high_b_low, 12'b0} +
                        {a_high_b_high, 24'b0};

            exp_mul_q <= exp_partial;
            sign_mul_q <= sign_partial;
            reg_bank_addr_q <= reg_bank_addr_partial;
            mant3_q <= mant3_partial;
            exp3_q <= exp3_partial;
            sign3_q <= sign3_partial;
        end
    end

    // Normalize the product before alignment
    logic mant_mul_overflow;
    logic [22:0] mant_mul_norm_frac;
    logic [23:0] mant_mul_norm;
    logic [8:0] exp_mul_norm;

    assign mant_mul_overflow = mant_mul_q[47];  // 1 if product >= 2.0
    assign mant_mul_norm_frac = mant_mul_overflow ? mant_mul_q[46:24] : mant_mul_q[45:23];
    assign mant_mul_norm = {1'b1, mant_mul_norm_frac};  // always proper 1.xxx, 24 bits
    assign exp_mul_norm = mant_mul_overflow ? (exp_mul_q + 9'd1) : exp_mul_q;

    // Alignment
    logic product_ge_addened;
    logic signed [8:0] exp_diff, exp_aligned;
    logic [23:0] mant3_aligned, mant_mul_aligned, shift_source;
    logic alignment_guard, alignment_round, alignment_sticky;
    
    assign product_ge_addened = (exp_mul_norm >= {1'b0, exp3_q});
    assign exp_diff = product_ge_addened ? (exp_mul_norm - exp3_q) : (exp3_q - exp_mul_norm);
    assign shift_source = product_ge_addened ? mant3_q : mant_mul_norm;

    assign alignment_guard = (exp_diff >= 1 && exp_diff < 24)
                             ? shift_source[exp_diff-1] : 1'b0;
    
    assign alignment_round = (exp_diff >= 2 && exp_diff < 24)
                             ? shift_source[exp_diff-2] : 1'b0;
    
    assign alignment_sticky = (exp_diff >= 24) ? |shift_source
                              : (exp_diff >= 3) ? |(shift_source & ~(24'hFFFFFF << (exp_diff - 3)))
                              : 1'b0;

    assign mant3_aligned = product_ge_addened
                           ? (exp_diff >= 24) ? 24'b0 : (mant3_q >> exp_diff)
                           : mant3_q;

    assign mant_mul_aligned = product_ge_addened
                           ? mant_mul_norm
                           : (exp_diff >= 24) ? 24'b0 : (mant_mul_norm >> exp_diff);

    assign exp_aligned = product_ge_addened ? exp_mul_norm : {1'b0, exp3_q};

    logic valid_aligned_q;
    logic sign_aligned_q;
    logic sign3_aligned_q;
    logic [23:0] mant3_aligned_q;
    logic [23:0] mant_mul_aligned_q;
    logic [8:0] exp_aligned_q;
    logic alignment_guard_q;
    logic alignment_round_q;
    logic alignment_sticky_q;
    logic [9:0] reg_bank_addr_aligned_q;

    always_ff @(posedge clk) begin
        if (reset) begin
            valid_aligned_q <= 1'b0;
            sign3_aligned_q <= 1'b0;
            sign_aligned_q <= 1'b0;
            mant3_aligned_q <= 24'b0;
            mant_mul_aligned_q <= 24'b0;
            exp_aligned_q <= 9'b0;
            alignment_guard_q <= 1'b0;
            alignment_round_q <= 1'b0;
            alignment_sticky_q <= 1'b0;
            reg_bank_addr_aligned_q <= 10'b0;
        end else if (!buffer_full) begin
            valid_aligned_q <= valid_mul_q;
            sign3_aligned_q <= sign3_q;
            sign_aligned_q <= sign_mul_q;
            mant3_aligned_q <= mant3_aligned;
            mant_mul_aligned_q <= mant_mul_aligned;
            exp_aligned_q <= exp_aligned;
            alignment_guard_q <= alignment_guard;
            alignment_round_q <= alignment_round;
            alignment_sticky_q <= alignment_sticky;
            reg_bank_addr_aligned_q <= reg_bank_addr_q;
        end
    end

    // Adding multiplicand and addend
    logic sign_add, same_sign, mul_ge_addend;
    logic [24:0] addition, result_add;
    logic [23:0] subtraction;

    assign same_sign = (sign3_aligned_q == sign_aligned_q);
    assign mul_ge_addend = (mant_mul_aligned_q >= mant3_aligned_q);

    assign addition = mant_mul_aligned_q + mant3_aligned_q;
    assign subtraction = mul_ge_addend
                         ? mant_mul_aligned_q - mant3_aligned_q
                         : mant3_aligned_q - mant_mul_aligned_q;
    
    assign result_add = same_sign ? addition : {1'b0, subtraction};
    assign sign_add = same_sign
                         ? sign_aligned_q
                         : (mul_ge_addend ? sign_aligned_q : sign3_aligned_q);
    
    logic [24:0] mant_add_q;
    logic [8:0] exp_add_q;
    logic sign_add_q, valid_add_q;
    logic guard_add_q, round_add_q, sticky_add_q;
    logic [9:0] reg_bank_addr_add_q;

    always_ff @(posedge clk) begin
        if (reset) begin
            valid_add_q <= 1'b0;
            mant_add_q <= 25'b0;
            exp_add_q <= 9'b0;
            sign_add_q <= 1'b0;
            guard_add_q <= 1'b0;
            round_add_q <= 1'b0;
            sticky_add_q <= 1'b0;
            reg_bank_addr_add_q <= 10'b0;
        end else if (!buffer_full) begin
            valid_add_q <= valid_aligned_q;
            mant_add_q <= result_add;
            exp_add_q <= exp_aligned_q;
            sign_add_q <= sign_add;
            guard_add_q <= alignment_guard_q;
            round_add_q <= alignment_round_q;
            sticky_add_q <= alignment_sticky_q;
            reg_bank_addr_add_q <= reg_bank_addr_aligned_q;
        end
    end

    function automatic [4:0] count_leading_zeros(input [23:0] val);
        begin
            casez (val)
                24'b1???????????????????????: count_leading_zeros = 5'd0;
                24'b01??????????????????????: count_leading_zeros = 5'd1;
                24'b001?????????????????????: count_leading_zeros = 5'd2;
                24'b0001????????????????????: count_leading_zeros = 5'd3;
                24'b00001???????????????????: count_leading_zeros = 5'd4;
                24'b000001??????????????????: count_leading_zeros = 5'd5;
                24'b0000001?????????????????: count_leading_zeros = 5'd6;
                24'b00000001????????????????: count_leading_zeros = 5'd7;
                24'b000000001???????????????: count_leading_zeros = 5'd8;
                24'b0000000001??????????????: count_leading_zeros = 5'd9;
                24'b00000000001?????????????: count_leading_zeros = 5'd10;
                24'b000000000001????????????: count_leading_zeros = 5'd11;
                24'b0000000000001???????????: count_leading_zeros = 5'd12;
                24'b00000000000001??????????: count_leading_zeros = 5'd13;
                24'b000000000000001?????????: count_leading_zeros = 5'd14;
                24'b0000000000000001????????: count_leading_zeros = 5'd15;
                24'b00000000000000001???????: count_leading_zeros = 5'd16;
                24'b000000000000000001??????: count_leading_zeros = 5'd17;
                24'b0000000000000000001?????: count_leading_zeros = 5'd18;
                24'b00000000000000000001????: count_leading_zeros = 5'd19;
                24'b000000000000000000001???: count_leading_zeros = 5'd20;
                24'b0000000000000000000001??: count_leading_zeros = 5'd21;
                24'b00000000000000000000001?: count_leading_zeros = 5'd22;
                24'b000000000000000000000001: count_leading_zeros = 5'd23;
                24'b000000000000000000000000: count_leading_zeros = 5'd24; // true zero
                default: count_leading_zeros = 5'd24;
            endcase
        end
    endfunction

    // Noramlize results
    logic overflow, exp_overflow, exp_underflow;
    logic [23:0] result_norm, result_norm_final;
    logic [8:0] result_exp_norm;
    logic [7:0] result_exp_final;
    logic [4:0] lz_count;
    logic is_true_zero;

    logic norm_guard, norm_round, norm_sticky;

    assign lz_count = count_leading_zeros(mant_add_q[23:0]);
    assign overflow = mant_add_q[24];
    assign is_true_zero = (!overflow && lz_count[4] && lz_count[3]); // lz_count[4] && lz_count[3] is only 1 if lz_count = 24

    assign result_norm = overflow
                         ? mant_add_q[24:1]
                         : is_true_zero ? 24'b0 : (mant_add_q[23:0] << lz_count);

    assign result_exp_norm = overflow
                             ? (exp_add_q + 1) :
                             is_true_zero ? 9'b0  : (exp_add_q - {4'b0, lz_count});
    
    assign exp_underflow = !overflow && ({4'b0, lz_count} >= exp_add_q); // negative (sign bit set on 9-bit) or exactly 0
    assign exp_overflow  = overflow && (result_exp_norm > 9'd255);

    assign result_norm_final = (is_true_zero || exp_underflow) ? 24'b0
                                      : exp_overflow ? 24'b0  // infinity: mantissa forced 0
                                      : result_norm;

    assign result_exp_final = (is_true_zero || exp_underflow) ? 8'b0 : result_exp_norm[7:0]; // overflow is handled after rounding
    
    assign norm_guard  = overflow ? mant_add_q[0] : guard_add_q;
    assign norm_round  = overflow ? 1'b0 : round_add_q;
    assign norm_sticky = overflow ? (guard_add_q | round_add_q | sticky_add_q) : sticky_add_q;

    logic [23:0] norm_mant_q;
    logic [7:0] norm_exp_q;
    logic [9:0] norm_reg_bank_addr_q;
    logic norm_sign_q, norm_valid_q;
    logic norm_guard_q, norm_round_q, norm_sticky_q;
    logic norm_exp_underflow_q, norm_exp_overflow_q;
    logic norm_is_true_zero_q;

    always_ff @(posedge clk) begin
        if (reset) begin
            norm_mant_q <= 24'b0;
            norm_exp_q <= 8'b0;
            norm_sign_q <= 1'b0;
            norm_valid_q <= 1'b0;
            norm_guard_q <= 1'b0;
            norm_round_q <= 1'b0;
            norm_sticky_q <= 1'b0;
            norm_exp_underflow_q <= 1'b0;
            norm_exp_overflow_q <= 1'b0;
            norm_reg_bank_addr_q <= 10'b0;
            norm_is_true_zero_q <= 1'b0;
        end else if (!buffer_full) begin
            norm_mant_q <= result_norm_final;
            norm_exp_q <= result_exp_final;
            norm_sign_q <= sign_add_q;
            norm_valid_q <= valid_add_q;
            norm_guard_q <= norm_guard;
            norm_round_q <= norm_round;
            norm_sticky_q <= norm_sticky;
            norm_exp_underflow_q <= exp_underflow;
            norm_exp_overflow_q <= exp_overflow;
            norm_reg_bank_addr_q <= reg_bank_addr_add_q;
            norm_is_true_zero_q <= is_true_zero;
        end
    end

    // Rounding
    logic round_up, rounding_overflow, final_exp_overflow, final_exp_underflow;
    logic [24:0] result_round;
    logic [23:0] result_round_final;
    logic [8:0] result_exp_rounded;

    logic [23:0] result_mant;
    logic [7:0] result_exp;
    logic result_sign;
    
    assign round_up = norm_guard_q && (norm_round_q || norm_sticky_q || norm_mant_q[0]);
    assign result_round = norm_mant_q + {23'b0, round_up};
    assign rounding_overflow = result_round[24];

    assign result_round_final = rounding_overflow ? {1'b1, 23'b0} : result_round[23:0];
    assign result_exp_rounded = rounding_overflow ? ({1'b0, norm_exp_q} + 9'd1) : {1'b0, norm_exp_q};

    assign final_exp_overflow  = norm_exp_overflow_q || (!norm_is_true_zero_q && !norm_exp_underflow_q && (result_exp_rounded > 9'd255));
    assign final_exp_underflow = norm_exp_underflow_q;

    assign result_mant = (norm_is_true_zero_q || final_exp_underflow) ? 24'b0
                                  : final_exp_overflow ? 24'b0
                                  : result_round_final;

    assign result_exp = (norm_is_true_zero_q || final_exp_underflow) ? 8'b0
                               : final_exp_overflow ? 8'hFF
                               : result_exp_rounded[7:0];
    
    assign result_sign = norm_is_true_zero_q ? 1'b0 : norm_sign_q;

    // Pack normalized results
    assign result = {result_sign, result_exp, result_mant[22:0]};

    assign valid_out = norm_valid_q;

    assign reg_bank_addr_out = norm_reg_bank_addr_q;
endmodule
