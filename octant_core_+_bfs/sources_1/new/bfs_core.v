`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/21/2022 04:49:42 PM
// Design Name: 
// Module Name: bfs_core
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

`define BRANCH_WIDTH 152
`define IDLE 0
`define READ 1
`define WORK 2
`define STALL 3
`define BURST_SIZE 64

module bfs_core(
    input wire i_clk,
    input wire i_rst_n,
    input wire i_en,
    input wire [`BRANCH_WIDTH-1:0] i_doutb,
    input wire [15:0] i_leaf_count,
    input wire [15:0] i_branch_count,
    output reg o_enb,
    output wire [3:0] o_addrb_bfs,
    output reg o_finish_bfs,
    output reg [`BURST_SIZE-1:0] o_occ_code,
    output reg [15:0] o_branch_count

    );

    //reg inter.
    reg [3:0] state;
    reg [`BRANCH_WIDTH-1:0] prev_branch;
    reg [3:0] counter;
    reg [65535:0] aux_already_visited;  // bit nº0 = pos 2 bram 


    //reg BRAM
    reg [15:0] addrb_read;
    reg [15:0] prev_addrb_read;
    //wire BRAM
    assign o_addrb_bfs = addrb_read;


    //reg aux
    reg [7:0] aux_occupancy_table [7:0];
    reg [7:0] aux_table_id [1:0];
    integer i;
    integer j;
    reg [3:0] k;

    //Address stack to save next addrs to read from bram 
    reg [15:0] addrs_stack [0:7];
    reg [7:0] stack_addr;
   


    //wires
    wire [`BRANCH_WIDTH-1:0] branch;
    wire [7:0] aux_occupancy_code;
    wire [2:0] number_of_children;
    wire [15:0] aux_table_addrs [7:0];
    
    wire [15:0] bram_branch [7:0]; 

    //test wires

    /* 
    i_doutb[151:136] --> 0;
    i_doutb[135:120] --> 1;
    i_doutb[119:104] --> 2;
    i_doutb[103:88]  --> 3;
    i_doutb[87:72]   --> 4;
    i_doutb[71:56]   --> 5;
    i_doutb[55:40]   --> 6;
    i_doutb[39:24]   --> 7;

     */

    assign aux_table_addrs[7] = i_doutb[151:136];
    assign aux_table_addrs[6] = i_doutb[135:120];
    assign aux_table_addrs[5] = i_doutb[119:104];
    assign aux_table_addrs[4] = i_doutb[103:88];
    assign aux_table_addrs[3] = i_doutb[87:72];
    assign aux_table_addrs[2] = i_doutb[71:56];
    assign aux_table_addrs[1] = i_doutb[55:40];
    assign aux_table_addrs[0] = i_doutb[39:24];
    
    assign bram_branch[0] = i_doutb[151:136];
    assign bram_branch[1] = i_doutb[135:120];
    assign bram_branch[2] = i_doutb[119:104];
    assign bram_branch[3] = i_doutb[103:88];
    assign bram_branch[4] = i_doutb[87:72];
    assign bram_branch[5] = i_doutb[71:56];
    assign bram_branch[6] = i_doutb[55:40];
    assign bram_branch[7] = i_doutb[39:24];
    
    


    


    assign branch = i_doutb;

    assign aux_occupancy_code[7] = (i_doutb[151:136] && 1);
    assign aux_occupancy_code[6] = (i_doutb[135:120] && 1);
    assign aux_occupancy_code[5] = (i_doutb[119:104] && 1);
    assign aux_occupancy_code[4] = (i_doutb[103:88] && 1);
    assign aux_occupancy_code[3] = (i_doutb[87:72] && 1);
    assign aux_occupancy_code[2] = (i_doutb[71:56] && 1);
    assign aux_occupancy_code[1] = (i_doutb[55:40] && 1);
    assign aux_occupancy_code[0] = (i_doutb[39:24] && 1);

    assign number_of_children = (i_doutb[151:136] && 1) + (i_doutb[135:120] && 1) + (i_doutb[119:104] && 1) 
                                + (i_doutb[103:88] && 1) + (i_doutb[87:72] && 1) + (i_doutb[71:56] && 1) 
                                + (i_doutb[55:40] && 1) + (i_doutb[39:24] && 1);

    always @(posedge i_clk) begin
        if(!i_rst_n)begin
            o_finish_bfs <= 0;
            o_enb <= 1;
            addrb_read <= 2;
            for (i=0;i<8;i=i+1) begin
                aux_occupancy_table[i] <= 0;
            end
            for (i=0;i<8;i=i+1) begin
                addrs_stack[i] <= 0; 
            end
            prev_branch <= 0;
            o_occ_code <= 0;
            counter <=0;
            stack_addr <= 0;
            k <= 0;
            j <= 0;
            aux_already_visited <= 0;
            o_branch_count <= 0;
        end else begin
            case (state)
                `IDLE: begin
                    o_finish_bfs <= o_finish_bfs;
                end
 
                `READ: begin
                    for(k=8;k>0;k=k-1)begin // <-------------- ??????
                        if(aux_occupancy_code[k-1] == 1) begin
                            addrs_stack[stack_addr] = aux_table_addrs[k-1];
                            stack_addr = stack_addr + 1;
                        end                            
                    end
                end

                `WORK: begin
                    //keep previous branch
                    prev_branch <= i_doutb;
                    prev_addrb_read <= addrb_read;
                    //debug      
                    aux_occupancy_table[addrb_read] <= aux_occupancy_code;

                    //output
                    if(!(aux_already_visited[addrb_read])) begin
                        // set bit already visited
                        aux_already_visited[addrb_read] <= 1'b1;
                        
                        //re arrange stack(pop)
                        for (j=0;j<stack_addr;j=j+1) begin
                            addrs_stack[j] <= addrs_stack[j+1];
                        end

                        //assert stack size
                        stack_addr <= stack_addr - 1;

                        if(addrs_stack[0] == 1) begin // if addr = 1, LEAF
                            // put occ code in output reg
                            o_occ_code[(8*counter)-1 +: 8] <= 8'b0;
                            
                            
                            //assign next addr, next in stack
                            addrb_read <= addrs_stack[1];
                        end else begin 
                            // put occ code in output reg
                            o_occ_code[(8*counter)-1 +: 8] <= aux_occupancy_code;
                            
                            //assign next addr, bottom of stack
                            addrb_read <= addrs_stack[0];
                        end
                        
                        o_branch_count <= o_branch_count + 1;                  
                    end
                    
                    
                    if(o_branch_count == i_branch_count) begin
                        o_finish_bfs <= 1;
                    end

                    if (counter < 8) begin
                        counter <= counter + 1;
                    end else begin
                        counter <= 0;
                        //if counter = 64 bits, output is full, send to DDR ?
                        //send output

                    end
                    
                end 
                default: begin
                    
                end
            endcase
            
        end
    end


    always @(posedge i_clk) begin
        if (!i_rst_n) begin
            state <= 0;
        end
        else begin
            case (state)
                `IDLE: //0
                begin
                    if(i_en)
                        state <= `READ;
                    else
                        state <= `IDLE;                 
                end
                `READ: //1
                begin
                    state <= `WORK;
                end
                `WORK:
                begin
                    if (o_finish_bfs) begin
                        state <= `IDLE;
                    end else begin
                        state <= `STALL;
                    end
                    
                end
                `STALL: begin
                    if(addrb_read == 1) begin
                        state <= `WORK;
                    end else begin
                        state <= `READ;
                    end
                end
                                                                   
                default: 
                begin
                
                end
            endcase
        end
    end

endmodule
