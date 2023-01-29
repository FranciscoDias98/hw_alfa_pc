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
`define WORKING_BFS 2
`define STALL 3
`define BURST_SIZE 64
`define STACK_SIZE 512 // 65536
`define ADDR_SIZE 9

module bfs_core(
    input wire i_clk,
    input wire i_rst,
    input wire i_en,
    input wire [`BRANCH_WIDTH-1:0] i_doutb,
    //input wire [15:0] i_leaf_count,
    input wire [15:0] i_branch_count,
    //output reg o_enb,
    //output wire [3:0] o_addrb_bfs,
    output reg o_finish_bfs,
    output reg [`BURST_SIZE-1:0] o_occ_code,
    output reg [15:0] o_branch_count,
    output reg o_send_to_ddr_occ_code,
    output reg [`BRANCH_WIDTH-1:0] o_dinb,
    output reg o_we_b,
    output reg [`ADDR_SIZE-1:0] addrb_read,
    //********** debug **************
    output wire [7:0] debug_occ_code,
    output reg [3:0] state,
     //<------------------------ VER AQUI O ERRO!!!!
    output reg [15:0] stack_addr,// -> top of stack

    output reg [15:0] stack_pointer_bot,
    
    output reg [15:0] o_bfs_branch_count,
    
    output wire [15:0] current_addr_read,
    
    output wire [15:0] aux_table_addrs_7,
    output wire [15:0] aux_table_addrs_6,
    output wire [15:0] aux_table_addrs_5,
    output wire [15:0] aux_table_addrs_4,
    output wire [15:0] aux_table_addrs_3,
    output wire [15:0] aux_table_addrs_2,
    output wire [15:0] aux_table_addrs_1,
    output wire [15:0] aux_table_addrs_0,
    
//    output wire [15:0] stack_0,
//    output wire [15:0] stack_1,
//    output wire [15:0] stack_2,
//    output wire [15:0] stack_3,
//    output wire [15:0] stack_4,
//    output wire [15:0] stack_5,
//    output wire [15:0] stack_6,
//    output wire [15:0] stack_7,
    
    
    
    output wire [7:0] occ_code_0,
    output wire [7:0] occ_code_1,
    output wire [7:0] occ_code_2,
    output wire [7:0] occ_code_3,
    output wire [7:0] occ_code_4,
    output wire [7:0] occ_code_5,
    output wire [7:0] occ_code_6,
    output wire [7:0] occ_code_7  
    );

    //reg inter.
    //reg [3:0] state;
    reg [`BRANCH_WIDTH-1:0] prev_branch;
    reg [7:0] counter;
    reg [65535:0] aux_already_visited;  // bit nº0 = pos 2 bram 


    //reg BRAM
    //reg [15:0] addrb_read;
    reg [15:0] prev_addrb_read;
    //wire BRAM
    //assign o_addrb_bfs = addrb_read;


    //reg aux
    reg [7:0] aux_occupancy_table [7:0];
    reg [7:0] aux_table_id [1:0];
    integer i;
    integer j;
    reg [3:0] k;

    //Address stack to save next addrs to read from bram 
    reg [15:0] addrs_stack [0:(`STACK_SIZE-1)];  //<------------------------ VER AQUI O ERRO!!!!
//    reg [15:0] stack_addr; // -> top of stack

//    reg [15:0] stack_pointer_bot;

    
   


    //wires
    wire [`BRANCH_WIDTH-1:0] branch;
    wire [7:0] aux_occupancy_code;
    wire [2:0] number_of_children;
    wire [15:0] aux_table_addrs [7:0];
    
    wire [15:0] bram_branch [7:0]; 
    
    wire aux_addr_stack_visited;

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


    //debug
    
    assign debug_occ_code = aux_occupancy_code;
    
    assign aux_table_addrs_7 = aux_table_addrs[7];
    assign aux_table_addrs_6 = aux_table_addrs[6];
    assign aux_table_addrs_5 = aux_table_addrs[5];
    assign aux_table_addrs_4 = aux_table_addrs[4];
    assign aux_table_addrs_3 = aux_table_addrs[3];
    assign aux_table_addrs_2 = aux_table_addrs[2];
    assign aux_table_addrs_1 = aux_table_addrs[1];
    assign aux_table_addrs_0 = aux_table_addrs[0];
    
    
//    assign stack_0 = addrs_stack[0];
//    assign stack_1 = addrs_stack[1];
//    assign stack_2 = addrs_stack[2];
//    assign stack_3 = addrs_stack[3];
//    assign stack_4 = addrs_stack[4];
//    assign stack_5 = addrs_stack[5];
//    assign stack_6 = addrs_stack[6];
//    assign stack_7 = addrs_stack[7];
    
    
    assign occ_code_0 = o_occ_code[63-:8];
    assign occ_code_1 = o_occ_code[55-:8];
    assign occ_code_2 = o_occ_code[47-:8];
    assign occ_code_3 = o_occ_code[39-:8];
    assign occ_code_4 = o_occ_code[31-:8];
    assign occ_code_5 = o_occ_code[23-:8];
    assign occ_code_6 = o_occ_code[15-:8];
    assign occ_code_7 = o_occ_code[7-:8];
    
    assign aux_addr_stack_visited = aux_already_visited[addrb_read];
    
    assign current_addr_read = addrs_stack[stack_pointer_bot];
    
reg flag;

    always @(posedge i_clk) begin
        if(!i_rst)begin
            o_finish_bfs <= 0;
            //o_enb <= 1;
            addrb_read <= 2;
            for (i=0;i<8;i=i+1) begin
                aux_occupancy_table[i] <= 0;
            end
            for (i=0;i<`STACK_SIZE;i=i+1) begin
                addrs_stack[i] <= 0; 
            end
            prev_branch <= 0;
            o_occ_code <= 0;
            counter <=0;
            stack_addr <= 0;
            o_we_b <= 0;
            k <= 0;
            j <= 0;
            aux_already_visited <= 0;
            o_branch_count <= 0;
            o_dinb <= 0;
            stack_pointer_bot <= 0;
            o_bfs_branch_count <= 0;
        end else begin
            case (state)
                `IDLE: begin // 0
                    o_finish_bfs <= o_finish_bfs;
                    o_send_to_ddr_occ_code <= 0;
                end
 
                `READ: begin //1
                    for(k=8;k>0;k=k-1)begin // <-------------- ??????
                        //if(aux_occupancy_code[k-1] == 1) begin
                            //if((!(aux_already_visited[aux_table_addrs[k-1]])) &&  ((aux_table_addrs[k-1]) > 1)) begin
                            if(aux_table_addrs[k-1] > 1) begin
                                addrs_stack[stack_addr] = aux_table_addrs[k-1];
                                stack_addr = stack_addr + 1;
                                stack_addr = (stack_addr & (`STACK_SIZE-1));
                            //circular stack 
                            end
                        //end                            
                    end
                    
                    //escrever zeros na BRAM 
                    //o_we_b = 1;
                    o_dinb <= 0;
                    //o_bfs_branch_count <= o_bfs_branch_count + 1;
                end

                `WORKING_BFS: begin //2
                    //keep previous branch
                    o_we_b <= 0;
                    prev_branch <= o_dinb;
                    prev_addrb_read <= addrb_read;
                    

                    //debug      
                    aux_occupancy_table[addrb_read] <= aux_occupancy_code;

                    //output
                    if(!(aux_already_visited[addrb_read])) begin
                        // set bit already visited
                        aux_already_visited[addrb_read] <= 1'b1;
                        
//                        if(addrs_stack[stack_pointer_bot] == 1) begin // if addr = 1, LEAF
//                            // put occ code in output reg
//                            //o_occ_code[(8*counter)-1 +: 8] <= 8'b0;
//                            //o_occ_code[63-(8*counter)-:8] <= aux_occupancy_code;
                            
//                            //assign next addr, next in stack
//                            addrb_read <= addrs_stack[stack_pointer_bot + 1];
                            
//                        end else begin 
//                            // put occ code in output reg
//                            //o_occ_code[(8*counter)-1 +: 8] <= aux_occupancy_code;
//                            if(addrb_read > 1) 
                            o_occ_code[63-(8*counter)-:8] <= aux_occupancy_code;
                            //assign next addr, bottom of stack
                            addrb_read <= addrs_stack[stack_pointer_bot];
                            o_branch_count <= o_branch_count + 1;
                        //end
                        
                        
                         // when 8 bytes/occ_codes are processed, send to DDR 
                   
                        
                        
                    end else begin
                        //if(addrs_stack[stack_pointer_bot + 1] > 0) begin
                            addrb_read <= addrs_stack[stack_pointer_bot];
                            //o_occ_code[63-(8*counter)-:8] <= aux_occupancy_code;
                            //o_branch_count <= o_branch_count + 1;
                        //end
                        //flag <= 1;
                        
                        
                    end
                    //end else begin
//                        if(addrb_read == 0)
//                            addrb_read <= addrs_stack[stack_pointer_bot + 1];
//                        else begin 
                        if(addrb_read == 0) begin
                            o_finish_bfs <= 1;
                            o_send_to_ddr_occ_code <= 1;
                        end

                    //end
                     if (counter < 7) begin
                        counter <= counter + 1;
                    end else begin
                        counter <= 0;
                        o_send_to_ddr_occ_code <= 1; // where to put this to zero ????
                        //if counter = 64 bits, output is full, send to DDR ?
                        //send output

                    end

                    
                                     
                    
                   
                    
                end 
                `STALL: begin //3
                    o_send_to_ddr_occ_code <= 0;
                     
                   /*  //re arrange stack(pop)
                    for (j=0;j<`STACK_SIZE-2;j=j+1) begin    // <-------------------------- ERROOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOR
                        addrs_stack[j] = addrs_stack[j+1];
                    end
                        
                    addrs_stack[`STACK_SIZE-1] <= 0;
                        
                    //assert stack size
                    stack_addr <= stack_addr - 1; */

                    //stack pop
                    if(stack_pointer_bot < (`STACK_SIZE-1)) begin
                            stack_pointer_bot <= stack_pointer_bot + 1; //
                    end else begin
                        stack_pointer_bot <= 0;
                    end
                     
                    addrs_stack[stack_pointer_bot] <= 0;

                end
                default: begin
                    
                end
            endcase
            
        end
    end


    always @(posedge i_clk) begin
        if (!i_rst) begin
            state <= 0;
        end
        else begin
            case (state)
                `IDLE: //0
                begin
                    if(i_en && o_finish_bfs == 0)
                        state <= `READ;
                    else
                        state <= `IDLE;                 
                end
                `READ: //1
                begin
                    state <= `WORKING_BFS;
                end
                `WORKING_BFS:
                begin
                    if (o_finish_bfs) begin
                        state <= `IDLE;
                    end else begin
                        state <= `STALL;
                    end
                    
                end
                `STALL: begin
                    if(!o_finish_bfs) begin
                        //if(addrb_read == 1 || addrb_read == 0) begin
                            //state <= `WORKING_BFS;
                        //end else begin
                        if((stack_addr < i_branch_count) && (addrb_read > 0))
                            state <= `READ;
                        else 
                            state <= `WORKING_BFS;
                        //end
                    end else begin
                        state <= `IDLE;
                    end
                end
                                                                   
                default: 
                begin
                
                end
            endcase
        end
    end

endmodule
