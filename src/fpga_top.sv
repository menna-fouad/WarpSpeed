`timescale 1ns/1ps
module fpga_top(
    input logic clk_100mhz, // pin E3
    input logic reset_btn,  // pin 
    input logic kernel_launch_btn,
    input logic [3:0] warp_switch,
    output logic [5:0] led_out,
    // led_out[0] = IDLE
    // led_out[1] = PROCESSING
    // led_out[2] = FLUSH_START
    // led_out[3] = KERNEL_DONE
    // led_out[4] = ref_locked
    // led_out[5] = init_calib_complete

    inout [15:0] ddr2_dq,
    inout [1:0] ddr2_dqs_n,
    inout [1:0] ddr2_dqs_p,

    // Outputs
    output [12:0] ddr2_addr,
    output [2:0] ddr2_ba,
    output ddr2_ras_n,
    output ddr2_cas_n,
    output ddr2_we_n,
    output [0:0] ddr2_ck_p,
    output [0:0] ddr2_ck_n,
    output [0:0] ddr2_cke,
    output [0:0] ddr2_cs_n,
    output [1:0] ddr2_dm,
    output [0:0] ddr2_odt
);
    logic reset_clean;
    logic kernel_launch_clean;
    logic kernel_launch_prev;
    logic kernel_launch;
    logic mig_sys_rst_n;

    assign mig_sys_rst_n = ~reset_clean & ref_locked;

    logic clk_200mhz;
    logic ref_locked;
    logic mem_ui_clk;
    logic mem_ui_reset;

    logic init_calib_complete;
    logic app_en;
    logic [2:0] app_cmd;
    logic [26:0] app_addr;
    logic app_rdy;

    // write data channel
    logic [127:0] app_wdf_data;
    logic app_wdf_wren;
    logic app_wdf_end;
    logic [15:0] app_wdf_mask;
    logic app_wdf_rdy;

    // read data channel
    logic [127:0] app_rd_data;
    logic app_rd_data_valid;

    logic clk_100mhz_buf;

    IBUF u_sys_clk_ibuf (
        .I(clk_100mhz),
        .O(clk_100mhz_buf)
    );

    debounce u_debounce_reset (
        .clk(clk_100mhz_buf),
        .raw_in(reset_btn),
        .clean_out(reset_clean)
    );

    debounce u_debounce_kernel_launch (
        .clk(clk_100mhz_buf),
        .raw_in(kernel_launch_btn),
        .clean_out(kernel_launch_clean)
    );

    always_ff @(posedge clk_100mhz_buf) begin
        if (reset_clean) kernel_launch_prev <= 1'b0;
        else kernel_launch_prev <= kernel_launch_clean;
    end

    assign kernel_launch = kernel_launch_clean && !kernel_launch_prev && !reset_clean;

    logic [3:0] warp_sync1, warp_sync2;

    always_ff @(posedge clk_100mhz_buf) begin
        warp_sync1 <= warp_switch;
        warp_sync2 <= warp_sync1;
    end

    clk_ref_gen u_clk_ref_gen (
        .clk_in1(clk_100mhz_buf),
        .clk_out1(clk_200mhz),
        .reset(reset_clean),
        .locked(ref_locked)
    );

    sm_top u_sm_top (
        .sm_clk(clk_100mhz_buf),
        .sm_reset(reset_clean),
        .mem_ui_clk(mem_ui_clk),
        .mem_ui_reset(mem_ui_reset),
        .kernel_start(kernel_launch),
        .num_warps(warp_sync2),
        .kernel_done(),
        // MIG interface
        .init_calib_complete(init_calib_complete),
        .app_en(app_en),
        .app_cmd(app_cmd),
        .app_addr(app_addr),
        .app_rdy(app_rdy),
        .app_wdf_data(app_wdf_data),
        .app_wdf_wren(app_wdf_wren),
        .app_wdf_end(app_wdf_end),
        .app_wdf_mask(app_wdf_mask),
        .app_wdf_rdy(app_wdf_rdy),
        .app_rd_data(app_rd_data),
        .app_rd_data_valid(app_rd_data_valid),
        .kernel_state_leds(led_out[3:0])
    );

    assign led_out[4] = ref_locked;
    assign led_out[5] = init_calib_complete;
    
    mig_7series_0 u_mig (
        // Inouts
        .ddr2_dq(ddr2_dq),
        .ddr2_dqs_n(ddr2_dqs_n),
        .ddr2_dqs_p(ddr2_dqs_p),
        // Outputs
        .ddr2_addr(ddr2_addr),
        .ddr2_ba(ddr2_ba),
        .ddr2_ras_n(ddr2_ras_n),
        .ddr2_cas_n(ddr2_cas_n),
        .ddr2_we_n(ddr2_we_n),
        .ddr2_ck_p(ddr2_ck_p),
        .ddr2_ck_n(ddr2_ck_n),
        .ddr2_cke(ddr2_cke),
        .ddr2_cs_n(ddr2_cs_n),
        .ddr2_dm(ddr2_dm),
        .ddr2_odt(ddr2_odt),
        // Inputs
        // clocks
        .sys_clk_i(clk_100mhz_buf),
        .clk_ref_i(clk_200mhz),
        // user interface signals
        .app_addr(app_addr),
        .app_cmd(app_cmd),
        .app_en(app_en),
        .app_wdf_data(app_wdf_data),
        .app_wdf_end(app_wdf_end),
        .app_wdf_mask(app_wdf_mask),
        .app_wdf_wren(app_wdf_wren),
        .app_rd_data(app_rd_data),
        .app_rd_data_valid(app_rd_data_valid),
        .app_rdy(app_rdy),
        .app_wdf_rdy(app_wdf_rdy),
        .app_sr_req(1'b0),
        .app_ref_req(1'b0),
        .app_zq_req(1'b0),
        .ui_clk(mem_ui_clk),
        .ui_clk_sync_rst(mem_ui_reset),
        .init_calib_complete(init_calib_complete),
        .sys_rst(mig_sys_rst_n)
    );
endmodule
