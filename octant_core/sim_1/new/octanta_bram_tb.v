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
`define SIZE 112 //
 

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
   
    reg [3:0] depth;
    wire finish;
    integer i;
    
    parameter point_cloud_size = 7;
    
    reg signed [15:0] points_x_file [point_cloud_size-1:0];
    reg signed [15:0] points_y_file [point_cloud_size-1:0];
    reg signed [15:0] points_z_file [point_cloud_size-1:0];
    
    initial begin
        clock = 1;
        reset = 1;
        point = 0;
        en = 0;
        mid_point = 0;
        near_bottom_left = 0;
        far_top_right = 0;
        depth = 14;
        //finish =0;
        $readmemh("/home/francisco98/Points_to_Hw/x_points_frame1_hdl32.txt",points_x_file);
        $readmemh("/home/francisco98/Points_to_Hw/y_points_frame1_hdl32.txt",points_y_file);
        $readmemh("/home/francisco98/Points_to_Hw/z_points_frame1_hdl32.txt",points_z_file);
        
//        for(i = 0;i<point_cloud_size;i=i+1)begin
            
//        end
        
        
        #20;
        reset = 0;
    
        #40;
        en = 1;
        reset = 1;
        point = 0;
        mid_point = 0;
        
        // min. point
        near_bottom_left[63 -:16] = -25600;//-10113;
        near_bottom_left[47 -:16] = -25600;//-7972;
        near_bottom_left[31 -:16] = -25600;//-441;
        near_bottom_left[15 -:16] = 0;
        // max. point
        far_top_right[63 -:16] = 25600;//5557;
        far_top_right[47 -:16] = 25600;//7985;
        far_top_right[31 -:16] = 25600;//315;
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

        points_x[32 +:16] = -993;
        points_y[32 +:16] = -154;
        points_z[32 +:16] = -154;
        
        points_x[16 +:16] = 272;
        points_y[16 +:16] = -45;
        points_z[16 +:16] = -155;
        
        //point to test
//        points_x[48 +:16] = -10112; 
//        points_y[48 +:16] = -7984;
//        points_z[48 +:16] = -313;
        
        
        points_x[48 +:16] = -286;
        points_y[48 +:16] = -45;
        points_z[48 +:16] = -155;
        
        points_x[64 +:16] = -1325;
        points_y[64 +:16] = -218;
        points_z[64 +:16] = -157;
        
        
        points_x[80 +:16] = -302;
        points_y[80 +:16] = -49;
        points_z[80 +:16] = -154;
        
        points_x[96 +:16] = -1640;
        points_y[96 +:16] = -271;
        points_z[96 +:16] = -155;
        
        
        
        
        


        
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

        
        #50000;
        //en = 0;
        #500;
        $finish;
    
    end
    
    always @(posedge clock)begin
        if(finish) begin
            en = 0;
            reset = 0;
        end
    
    end

    always #10 clock <= ~clock;
    
//    design_1_wrapper uut(
//        .i_clk_0(clock),
//        .i_en_0(en),
//        .i_far_top_right_0(far_top_right),
//        .i_mid_point_0(mid_point),
//        .i_near_bottom_left_0(near_bottom_left),
//        .i_point_cloud_size_0(point_cloud_size),
//        .i_points_x_0(points_x),
//        .i_points_y_0(points_y),
//        .i_points_z_0(points_z),
//        .i_rst_n_0(reset)
//    );
    
    design_v2_wrapper #(.MAX_DEPTH(14)) uut2(
        .i_clk_0(clock),
        .i_en_0(en),
        .i_far_top_right_0(far_top_right),
        .i_mid_point_0(mid_point),
        .i_near_bottom_left_0(near_bottom_left),
        .i_point_cloud_size_0(point_cloud_size),
        .i_points_x_0(points_x),
        .i_points_y_0(points_y),
        .i_points_z_0(points_z),
        .i_rst_n_0(reset),
        .o_finish_0(finish)
    );

endmodule
