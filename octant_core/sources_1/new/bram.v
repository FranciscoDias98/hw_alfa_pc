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
        parameter WIDTH = 32, 
        parameter DEPTH = 256,
        localparam ADDRW = $clog2(DEPTH)
    )(
    input wire i_clk,                       // clock (port a & b)
    input wire i_we,                        // write enable (port a)
    input wire [ADDRW-1:0] i_addr_write,    // write address (port a)
    input wire [ADDRW-1:0] i_addr_read,     // read address (port b)
    input wire [WIDTH-1:0] i_data_in,       // data in (port a)
    output reg [WIDTH-1:0] o_data_out       // data out (port b)
    );

    reg [15:0] memory_pointer [DEPTH-1:0];
    reg [7:0] memory_branch [DEPTH-1:0];
    reg [7:0] memory_leaf [DEPTH-1:0];
    
    integer i;
    initial begin
        for(i = 0; i < DEPTH; i = i+1) begin
            memory_pointer[i] <= 0;
            memory_branch[i] <= 0;
            memory_leaf[i] <= 0;
        end      
    end

    // Port A: Sync Write
    always @(posedge i_clk) begin
        if (i_we) begin
            memory_pointer[i_addr_write] <= i_data_in[31:16];
            memory_leaf[i_addr_write] <= i_data_in[15:8];
            memory_branch[i_addr_write] <= i_data_in[7:0];
        end
    end

    // Port B: Sync Read
    always @(posedge i_clk) begin
        o_data_out <= {memory_pointer[i_addr_read],memory_leaf[i_addr_write],memory_branch[i_addr_write]};
    end
endmodule
