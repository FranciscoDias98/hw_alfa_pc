`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/09/2023 01:30:14 PM
// Design Name: 
// Module Name: control_unit
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

`define IDLE 0
`define READING 1
`define UPDATING 2
`define WORK 3
`define WRITING 4


    module control_unit#(
    parameter DDR_BASE_ADDRESS =32'h0F000000
)(
    //input wire i_en,
    input wire i_clk,
    input wire i_rst,
    //module_interface connection
    input wire i_start,
    input wire i_initreadtxn,
    input wire i_read_TxnDone,
    input wire i_write_TxnDone,
    input wire [31:0] n_points,
    input wire [31:0] i_point_cloud_size,
    output reg o_finish,
    output reg [31:0] o_read_address,
    output reg [2:0] mod_int_state,
    //output reg [15:0] counter0,
    //octree_core conecction
    input wire i_finish_octree_core,
    input wire i_need_new_points,  // <------ WHERE TO PUT THIS IN OCTREE_CORE ????
    //input wire i_branch_octree,
    output reg o_en_octant_core,
    //bfs_core connection
    input wire i_finish_bfs_core,
    //input wire i_send_to_ddr,
    //input wire i_branch_bfs,
    output reg o_en_bfs_core,
    //mux connection
    output reg o_select_mux

    );

    reg flag_finish;



    // module_interface fsm
    always @(posedge i_clk) begin
        if(!i_rst) begin
            mod_int_state <= `IDLE;
            o_finish <= 0;
            flag_finish <= 0;
            o_read_address <= DDR_BASE_ADDRESS;
        end else begin
            case (mod_int_state)
                `IDLE: begin
                    o_finish <= 0;
                    o_en_octant_core <= 0;
                    o_en_bfs_core <= 0;
                    if (i_start) begin
                        mod_int_state <= `READING;
                    end
                end
                `READING: begin //1
                    //counter0 <= 0;
                    if(i_read_TxnDone && !i_initreadtxn) begin 
                        mod_int_state <= `UPDATING;
                        o_read_address <= DDR_BASE_ADDRESS + (n_points<<3);
                    end
                end
                `UPDATING: begin //2
                    //counter0 <= 0;
                    mod_int_state <= `WORK;
                    o_en_octant_core <= 1;
                end
                `WORK: begin //3
                    if (i_finish_octree_core) begin
                        mod_int_state <= `WRITING;
                        o_select_mux <= 1;
                        o_en_bfs_core <= 1;
                        o_en_octant_core <= 0;

                    end else if (i_need_new_points) begin
                        // if octree_core needs more points, alert module_interface 
                        mod_int_state <= `READING; //<--------------- MUDEI PARA: READING, ANTES: UPDATING
                    end
                end
                `WRITING: begin
                    if(i_write_TxnDone && i_finish_bfs_core) // change to number of the occupancy_code register in BFS
                    begin
                        mod_int_state <= `IDLE;
                        o_finish <= 1;
                        //flag_finish <= 1;
                        o_select_mux <= 0; // put select to 0, to then octree_core process next frame
                        o_en_bfs_core <= 0;                        
                    end
                end
                default: begin
                    mod_int_state <= `IDLE;
                end
            endcase

        end
    end




endmodule
