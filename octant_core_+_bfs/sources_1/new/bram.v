`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/10/2021 08:06:17 AM
// Design Name: 
// Module Name: bram
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


module bram#(
        parameter WIDTH = 152, 
        parameter DEPTH = 16,
        localparam ADDRW = $clog2(DEPTH)
    )(
    input wire i_clk,                       // clock (port a & b)
    input wire i_we,                        // write enable (port a)
    input wire [ADDRW-1:0] i_addr_write,    // write address (port a)
    input wire [ADDRW-1:0] i_addr_read,     // read address (port b)
    input wire [WIDTH-1:0] i_data_in,       // data in (port a)
    output reg [WIDTH-1:0] o_data_out       // data out (port b)
    );

    reg [15:0] oc_0 [DEPTH-1:0];
    reg [15:0] oc_1 [DEPTH-1:0];
    reg [15:0] oc_2 [DEPTH-1:0];
    reg [15:0] oc_3 [DEPTH-1:0];
    reg [15:0] oc_4 [DEPTH-1:0];
    reg [15:0] oc_5 [DEPTH-1:0];
    reg [15:0] oc_6 [DEPTH-1:0];
    reg [15:0] oc_7 [DEPTH-1:0];
    reg [15:0] prev_branch [DEPTH-1:0]; 
    reg [3:0] depth [DEPTH-1:0];
    
    integer i;
    initial begin
        for(i = 0; i < DEPTH; i = i+1) begin
            oc_0[i] <= 0;
            oc_1[i] <= 0;
            oc_2[i] <= 0;
            oc_3[i] <= 0;
            oc_4[i] <= 0;
            oc_5[i] <= 0;
            oc_6[i] <= 0;
            oc_7[i] <= 0;
            prev_branch[i] <= 0;
            depth[i] <= 0;
            
        end      
    end

    // Port A: Sync Write
    always @(posedge i_clk) begin
        if (i_we) begin
            oc_0[i_addr_write] <= i_data_in[151:136];
            oc_1[i_addr_write] <= i_data_in[135:120];
            oc_2[i_addr_write] <= i_data_in[119:104];
            oc_3[i_addr_write] <= i_data_in[103:88];
            oc_4[i_addr_write] <= i_data_in[87:72];
            oc_5[i_addr_write] <= i_data_in[71:56];
            oc_6[i_addr_write] <= i_data_in[55:40];
            oc_7[i_addr_write] <= i_data_in[39:24];
            prev_branch[i_addr_write] <= i_data_in[23:8];
            depth[i_addr_write] <= i_data_in[7:3];
        end
    end

    // Port B: Sync Read
    always @(posedge i_clk) begin
        o_data_out <= {oc_0[i_addr_read],oc_1[i_addr_read],oc_2[i_addr_read],oc_3[i_addr_read],oc_4[i_addr_read],oc_5[i_addr_read],oc_6[i_addr_read],oc_7[i_addr_read],prev_branch[i_addr_read],depth[i_addr_read],0};
    end
endmodule
