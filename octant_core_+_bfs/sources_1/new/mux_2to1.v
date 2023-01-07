`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/03/2023 10:29:56 PM
// Design Name: 
// Module Name: mux_2to1
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
`define ADDR_SIZE 4


module mux_2to1(
    input [`ADDR_SIZE-1:0] i_addr_octree,
    input [`ADDR_SIZE-1:0] i_addr_bfs,
    input wire i_select,
    output wire [`ADDR_SIZE-1:0] o_addr
    );


    assign o_addr = i_select ? i_addr_bfs : i_addr_octree;

endmodule
