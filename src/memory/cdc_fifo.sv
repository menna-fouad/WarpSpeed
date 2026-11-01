`default_nettype none
`timescale 1ns/1ps
module cdc_fifo #(
    parameter int DATA_WIDTH = 128,
    parameter int DEPTH = 16,
    parameter int POINTER_WIDTH = $clog2(DEPTH)
) (
    // write interface
    input logic wclk,
    input logic wrst,
    input logic wen,
    input logic [DATA_WIDTH-1:0] wdata,
    output logic full,
    output logic wempty,

    // read interface
    input logic rclk,
    input logic rrst,
    input logic ren,
    output logic empty,
    output logic [DATA_WIDTH-1:0] rdata
);
    logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // Pointers
    logic [POINTER_WIDTH:0] wptr, wgray, rptr, rgray;
    logic [POINTER_WIDTH:0] rgray_sync1, rgray_sync2;
    logic [POINTER_WIDTH:0] wgray_sync1, wgray_sync2;

    // write domain
    wire [POINTER_WIDTH:0] wptr_next = wptr + {{POINTER_WIDTH{1'b0}}, (wen && !full)};
    wire [POINTER_WIDTH:0] wgray_next = (wptr_next >> 1) ^ wptr_next;

    always_ff @(posedge wclk) begin
        if (wrst) begin
            wptr <= '0;
            wgray <= '0;
            rgray_sync1 <= '0;
            rgray_sync2 <= '0;
            full <= 1'b0;
            wempty <= 1'b1;
        end else begin
            if (wen && !full) 
                mem[wptr[POINTER_WIDTH-1:0]] <= wdata;
            
            wptr <= wptr_next;
            wgray <= wgray_next;

            rgray_sync1 <= rgray;
            rgray_sync2 <= rgray_sync1;

            full <= (wgray_next == {~rgray_sync2[POINTER_WIDTH:POINTER_WIDTH-1], rgray_sync2[POINTER_WIDTH-2:0]});
            wempty <= (wgray_next == rgray_sync2);
        end
    end

    // read domain
    logic [POINTER_WIDTH:0] rptr_next, rgray_next;
    assign rptr_next = rptr + {{POINTER_WIDTH{1'b0}}, (ren && !empty)};
    assign rgray_next = (rptr_next >> 1) ^ rptr_next;

    always_ff @(posedge rclk) begin
        if (rrst) begin
            rptr <= '0;
            rgray <= '0;
            wgray_sync1 <= '0;
            wgray_sync2 <= '0;
            empty <= 1'b1;
        end else begin
            rptr <= rptr_next;
            rgray <= rgray_next;

            wgray_sync1 <= wgray;
            wgray_sync2 <= wgray_sync1;

            empty <= (rgray_next == wgray_sync2);
        end
    end

    assign rdata = mem[rptr[POINTER_WIDTH-1:0]];
endmodule

/*
Instantiation:
// cmd: SM -> mem
async_fifo #(.DATA_WIDTH(26), .DEPTH(4)) cmd_fifo (
    .wclk(clk),
    .wrst(reset),
    .wen(mem_req_valid && mem_req_ready),
    .wdata({mem_req_write, mem_req_addr[31:7]}),
    .full(cmd_full),
    .rclk(ui_clk),
    .rrst(ui_rst),
    .ren(cmd_pop),
    .rdata({cmd_write, cmd_line_addr}), .empty(cmd_empty)
);

// wdata: SM -> mem
async_fifo #(.DATA_WIDTH(128), .DEPTH(16)) wr_fifo (
    .wclk(clk),
    .wrst(reset),
    .wen(mem_wvalid && mem_wready),
    .wdata(mem_wdata),
    .full(wr_full),
    .rclk(ui_clk),
    .rrst(ui_rst),
    .ren(wr_pop),
    .rdata(app_wdf_data),
    .empty(wr_empty)
);

// rdata: mem -> SM
async_fifo #(.DATA_WIDTH(129), .DEPTH(16)) rd_fifo (
    .wclk(ui_clk),
    .wrst(ui_rst),
    .wen(app_rd_data_valid),
    .wdata({rd_last, app_rd_data}),
    .full(rd_full),
    .rclk(clk),
    .rrst(reset),
    .ren(!rd_empty),
    .rdata({mem_rlast, mem_rdata}),
    .empty(rd_empty)
);
*/
