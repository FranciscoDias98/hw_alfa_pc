`timescale 1ns / 1ps

`define IDLE 0
`define READING 1
`define UPDATING 2
`define WORK 3
`define WRITING 4

//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/09/2022 03:44:18 PM
// Design Name: 
// Module Name: module_interface
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


module module_interface#(
        parameter AXI_MODULE_OUTPUTS = 32,
        DDR_BASE_ADDRESS =32'h0F000000,
        RANGE_WIDTH = 8                    
    )(
    input wire i_clk,
    input wire i_rst,
    input wire [2:0] state,
    //CONTROL UNIT CONNECTION
    //input wire [15:0] i_counter, 
    output reg [31:0] n_points,
    
    //DDR_MODULE CONNECTION
    input wire i_write_TxnDone,
    input wire i_read_TxnDone,
    input wire [(64*AXI_MODULE_OUTPUTS)-1:0] i_AMU_P,
    output reg [31:0] o_write_address,
    output reg [63:0] o_write_payload,
    output reg o_initwritetxn,
    output wire o_initreadtxn,
        

    // ************************************************************** //    
    //X AND Y  module
    // output reg [15:0] o_range,
    //output reg [15:0] o_azimuth,
    //output reg signed [15:0] o_elevation,
    //BRAM MODULE
    //input wire [255:0] i_range_data, //mudar tamanho quando for para fazer o write
    //output reg [17:0] o_ri_read_address,
    //output wire o_we,
    //ANGLE IMAGE
    //output reg [13:0] o_addrb_cnt,
    //output reg o_rstb,
    //output reg o_en_RI_read,
    //output reg [17:0] o_ri_write_address,
    // ************************************************************** //


    // **************************ALFA-Pc Modules ******************** //

    input wire [63:0] i_occupacy_code_64, //payload to write in ddr
    //input wire [15:0] i_branch_count, // nº of branchs for compression/decompression
    //input wire [15:0] i_leaf_count, // nº of leafs for compression/decompression
    input wire i_send_to_ddr,
    input wire i_bfs_finish,

    output reg [(32*16)-1:0] o_x_points,
    output reg [(32*16)-1:0] o_y_points,
    output reg [(32*16)-1:0] o_z_points,
    
    output reg [7:0] counter,

    



    // **************************ALFA-Pc Modules ******************** //


    //DEBUG
//    reg only1read,
//    reg first_read,
    //output reg [7:0] counter,
    //output reg [7:0] counter1,
    //output reg [(64*AXI_MODULE_OUTPUTS)-1:0] DEBUG_i_AMU_P
    
    output reg only1read,
    output reg first_read,                                                                                                     
    output reg first_write

);

//    reg only1read;
//    reg first_read;                                                                                                     
//    reg first_write;

              
    assign o_initreadtxn = ((state == `READING && !only1read && !first_read) || (state == `UPDATING && only1read)) ? 1:
                                                                                                                     0;                                            
    
    //assign o_we = (state == `WRITING && counter1<=15 && only1read) ? 1:
     //0; 
     
                                                                   


    // **************************ALFA-Pc Modules ******************** //
    //aux
    integer index;
    
    // **************************ALFA-Pc Modules ******************** //

    always @(posedge i_clk)
    begin
        if(!i_rst)
        begin
//            counter0 <= 0;
            counter <= 0;
            only1read <= 0;
            first_read <= 0;
            n_points <= 0;
            first_write <= 0;
            o_initwritetxn <= 0;
            o_write_address <= 0;
            o_write_payload <= 0;
            //********** ALFA-Pc **********
            index <= 0;
            //********** ALFA-Pc **********
        end else begin
            case(state)

            `IDLE:
            begin
                counter <= 0;
                only1read <= 0;
                first_read <= 0;
                n_points <= 0;
                first_write <= 0;
                o_initwritetxn <= 0;
                o_write_address <= 0;
                o_write_payload <= 0;
                //o_ri_read_address <= 0;

               // o_rstb <= 0;
               /// o_en_RI_read <= 0;
              //o_addrb_cnt  <= 0;
            end

            `READING: //1
            begin
//                counter0 <= 0;
                first_read <= 1;
                if(!only1read)
                begin
                    only1read <= 1;
                    n_points <= n_points + 32;
                end
            end

            `UPDATING:
            begin
                 //DEBUG_i_AMU_P <= i_AMU_P;
                 only1read <= 0;
                
                 //********** ALFA-Pc **********

                 // for initial test, only 7 points, the same as testbench

                for (index = 0;index < AXI_MODULE_OUTPUTS; index = index +1) begin
                    o_x_points[index*16 +:16] <= i_AMU_P[15+(64*index) -: 16];
                    o_y_points[index*16 +:16] <= i_AMU_P[31+(64*index) -: 16];
                    o_z_points[index*16 +:16] <= i_AMU_P[47+(64*index) -: 16];
                end


                //********** ALFA-Pc ********** 
            

            end

            `WORK:
            begin
               //...//
            end
            `WRITING:
            begin
            // Only write when 64 bit reg in bfs_core is full (flag = o_send_to_ddr_occ_code)
            // when bfs_core is finished, also send the remaining (last bytes)

            if(i_send_to_ddr | i_bfs_finish) begin
                if((!first_write )||(i_write_TxnDone && !o_initwritetxn)) begin

                    o_write_payload <= i_occupacy_code_64;
                    first_write <= 1;
                    counter <= counter + 1;
                    o_write_address <= DDR_BASE_ADDRESS + (counter<<3); 
                    o_initwritetxn <= 1;
                end
                else begin
                    o_initwritetxn <= 0;
                end
            end else begin
                o_initwritetxn <= 0;
            end

            end
            endcase
            
        end
    end
    
endmodule