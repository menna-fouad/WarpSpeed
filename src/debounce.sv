module debounce #(
    parameter DEBOUNCE_BITS = 25
)(
    input logic clk,
    input logic raw_in,
    output logic clean_out
);
    // two-flop synchronizer
    reg sync0, sync1;
    always @(posedge clk) begin
        sync0 <= raw_in;
        sync1 <= sync0;
    end

    // Only accept a new level once it has been stable for 2^DEBOUNCE_BITS cycles
    reg [DEBOUNCE_BITS-1:0] count = 1'b0;
    initial clean_out = 1'b0;

    always @(posedge clk) begin
        if (sync1 != clean_out) begin
            count <= count + 1'b1;
            if (&count) begin
                clean_out <= sync1;
                count <= 0;
            end
        end else count <= 0;
    end
endmodule
