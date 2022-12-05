`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/16/2022 03:12:43 PM
// Design Name: 
// Module Name: octanta_bram_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`define CLK_CYCLE 20
`define SIZE 64 // 

module octanta_bram_tb();

    reg clock;
    reg reset;
    reg en;
    reg signed [63:0] point;
    reg signed [`SIZE-1:0] points_x;
    reg signed [`SIZE-1:0] points_y;
    reg signed [`SIZE-1:0] points_z;
    reg signed [63:0] mid_point;
    reg signed [63:0] far_top_right;
    reg signed [63:0] near_bottom_left;
    reg [15:0] point_cloud_size;
    
    initial begin
        clock = 1;
        reset = 1;
        point = 0;
        en = 0;
        mid_point = 0;
        near_bottom_left = 0;
        far_top_right = 0;
        point_cloud_size = 0;
        #20;
        reset = 0;
    
        #40;
        en = 1;
        reset = 1;
        point = 0;
        mid_point = 0;
        
        // min. point
        near_bottom_left[63 -:16] = -10113;
        near_bottom_left[47 -:16] = -7972;
        near_bottom_left[31 -:16] = -441;
        near_bottom_left[15 -:16] = 0;
        // max. point
        far_top_right[63 -:16] = 5557;
        far_top_right[47 -:16] = 7985;
        far_top_right[31 -:16] = 315;
        far_top_right[15 -:16] = 0;

        //mid point
        mid_point[63 -:16] = -2278;
        mid_point[47 -:16] = 6;
        mid_point[31 -:16] = -63;
        mid_point[15 -:16] = 0;

        // -2.5713155 -0.42475653 -1.5455809 // 
        point[63 -:16] = 257;
        point[47 -:16] = -42;
        point[31 -:16] = -155;//-155;
        point[15 -:16] = 0;
        
        points_x[0 +:16] = 257;
        points_y[0 +:16] = -42;
        points_z[0 +:16] = -155;

        points_x[16 +:16] = -993;
        points_y[16 +:16] = -154;
        points_z[16 +:16] = -154;
        
        points_x[32 +:16] = -272;
        points_y[32 +:16] = -45;
        points_z[32 +:16] = -155;
        
        points_x[48 +:16] = -10112;
        points_y[48 +:16] = -7984;
        points_z[48 +:16] = -313;

        point_cloud_size = 4;
        
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        
        //en = 0;
        //-9.3366032 -1.5423169 -1.5547358 
       point[63 -:16] = -933;
       point[47 -:16] = -154;
       point[31 -:16] = -154;//-155;
       point[15 -:16] = 0;
        
        
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        
        //en = 0;
        //-9.3366032 -1.5423169 -1.5547358 
        
        
        point[63 -:16] = -10112;
        point[47 -:16] = 7984;
        point[31 -:16] = -313;//-155;
        point[15 -:16] = 0;
        
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        
        
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        
        
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;
        #`CLK_CYCLE;

        en = 0;
        #1000;
    
    end
    
    always #10 clock <= ~clock;
    
    design_1_wrapper uut(
        .i_clk_0(clock),
        .i_en_0(en),
        .i_far_top_right_0(far_top_right),
        .i_mid_point_0(mid_point),
        .i_near_bottom_left_0(near_bottom_left),
        .i_point_cloud_size_0(point_cloud_size),
        .i_points_x_0(points_x),
        .i_points_y_0(points_y),
        .i_points_z_0(points_z),
        .i_rst_n_0(reset)
    );
    
    design_v2_wrapper uut2(
        .i_clk_0(clock),
        .i_en_0(en),
        .i_far_top_right_0(far_top_right),
        .i_mid_point_0(mid_point),
        .i_near_bottom_left_0(near_bottom_left),
        .i_point_cloud_size_0(point_cloud_size),
        .i_points_x_0(points_x),
        .i_points_y_0(points_y),
        .i_points_z_0(points_z),
        .i_rst_n_0(reset)
    );

endmodule
