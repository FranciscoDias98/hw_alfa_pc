`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/24/2022 05:14:42 PM
// Design Name: 
// Module Name: octant_core_tb
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

module octant_core_tb();

    reg clock;
    reg reset;
    reg en;
    reg signed [63:0] point;
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
    
        #20;
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


        point_cloud_size = 40539;
        
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
        $finish;
    end





always #10 clock <= ~clock;


octant_core uut_octant_core(
    .i_clk(clock),
    .i_rst_n(reset),
    .i_en(en),
    .i_point(point),
    .i_mid_point(mid_point),
    .i_near_bottom_left(near_bottom_left),
    .i_far_top_right(far_top_right),
    .i_point_cloud_size(point_cloud_size)
);


endmodule
