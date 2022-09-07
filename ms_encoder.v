`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/27/2022 04:33:29 PM
// Design Name: 
// Module Name: ms_encoder
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

`define WIDTH 40
`define TOP 16777216 // 2^24
`define BOTTOM 65536 // 2^16
`define MAX_RANGE 65536

`define IDLE 0
`define RC 1 // Range controller
`define UPDATE_FREQ 2 // Frequency Update
`define REN 3 // Renormalization core
`define FLUSH 4
`define RESET 5




module ms_encoder(
                    input i_clk,
                    input i_rst_n,
                    input i_en,
                    input [`WIDTH-1:0] i_data,
                    input [31:0] i_size,
                    //input i_data,
                    output reg [63:0] o_range,
                    output reg [7:0] o_flush,
                    output reg [31:0] o_addr,
                    output reg o_we,
                    output reg o_finish_encoder

                  );
                  
    reg [2:0] state;
    reg [31:0] range;
    reg [31:0] low;
    reg [31:0] size;
    reg [31:0] freq_cum [0:256];    
    reg [7:0] symbol;     
    reg [39:0] data_in;
    reg finish;
    
     //----- registos BRAM -----//
    reg [15:0] addr;
    reg [3:0] count;
    //------------------//
    
    reg [31:0] flush_low;
    
    //----- registos testes -----//
    reg signed [63:0] low_test;
    reg signed [63:0] range_test;
    
    reg [7:0] symbol_test;
    
        
    
                                     
    

    integer i;
                      
    always @(posedge i_clk) begin
        if(!i_rst_n) begin
            o_range <= 0;
            range <= 32'hFFFF_FFFF;
            low <= 0;
            for(i=0;i<257;i=i+1)
                freq_cum[i]<= i;
            i<=0;
            addr <= 0;
            o_we <= 0;
            flush_low <= 0;
            o_flush <= 0;
            count <= 0;
            size <= 0;
            o_finish_encoder <= 0;
            //data_in = i_data;
            //symbol <= i_data;
        end
        else
        begin 
            case(state)
                `IDLE: begin // 0
                    o_range <= o_range;
                    symbol <= i_data[7 +(8*size) -:8];                    
                end
                 
                `RC: begin // 1 Calculate new range and range and low values for symbol; 
                    low <= low + freq_cum[symbol]*(range/freq_cum[256]);
                    range <= (range/freq_cum[256])*(freq_cum[symbol+1]-freq_cum[symbol]);
                    flush_low <= flush_low << 8;
                    size <= size + 1;
                    symbol_test <= symbol;
                    
                end
                
                `UPDATE_FREQ: begin // 2 Update cumulative frequency of encoded symbol
                    for(i=0;i<257;i=i+1)begin // ----- > sintese (ciclo for)
                        if(i>symbol)
                            freq_cum[i] <= freq_cum[i] + 1;
                    end
                 end
    
                `REN: begin //3 Check range limits
                    if((low ^(low + range)) < `TOP || (range < `BOTTOM)) begin
                        if((range < `BOTTOM) && (low ^(low + range)) >= `TOP) begin
                            range<= ((~low + 1)&(`BOTTOM - 1));        
                        end
                    
                        range <= range << 8;
                        low <= low << 8;
                       
                       
                        //test BRAM insertion
                        o_flush <= low >> 24;                 
                        o_we <= 1;
                        o_addr <= addr;
                        addr <= addr + 1;
                        
                        
                       
                        // falta codiçao de overflow                  
                     end
                      
                     if(freq_cum[256]>=`MAX_RANGE) begin
                        for(i=1;i<=256;i=i+1)begin
                            freq_cum[i] <= freq_cum[i]<<1;
                        end
                     end
                      
                    
                end
                
                `FLUSH: begin // 4 Flush the low value data when are no more symbols to encode
                    o_flush <= low[31-count*8 -:8];
                    o_we<=1;
                    o_addr <= addr + count;
                    count <= count + 1;
                                          
          
                end
                
                `RESET: begin // 5                   
                    o_we <= 0;
                   
                end
    
                default:
                    o_range <= 0;
            endcase     
         
        end
    
       
    
    end     
    
    always @(posedge i_clk) begin
        
        if(!i_rst_n)
            state <= `RESET;
        else
        begin 
            case(state)
                `IDLE: begin
                    if(i_en && (size < i_size)) 
                        state <= `RC;
                    else begin
                        state <= `FLUSH;
                    end
                    
                    if(finish)
                        state <= `IDLE;
               end
                `RC: begin
                    state <= `UPDATE_FREQ;
                end
                `UPDATE_FREQ: begin
                    state <= `REN;
                end
                
                `REN: begin
                    if(!i_en)
                        state<= `FLUSH;
                    else
                        state <= `RESET;
                end
                
                `FLUSH: begin
                    if(count<3) ////// Se der problema, ver aqui!!!!!!
                        state<= `FLUSH;
                    else begin                   
                        state <= `RESET;
                        o_finish_encoder <= 1;
                        finish <= 1;
                    end                 
                end
                
                `RESET: begin
                    state <=`IDLE;
                end
                default:
                    state <= `IDLE;
            endcase
        end
        
    end        
         
                
endmodule





////range_tmp = (prob*range)/`SCALE_FACTOR;
//                    //range = range - range_tmp; // 
//                    //range <= range - (prob*range)/`SCALE_FACTOR ;
//                    if(i_data)begin
//                        //high = low + range;
//                        low <= low + (range - ((prob*range)/`SCALE_FACTOR)) ;
//                        //range = range_tmp; // 
//                        range <= (prob*range)/`SCALE_FACTOR;
                     
//    //                    end else begin
//    //                        range = range - range_tmp;
//    //                        high = low + range;
//                    end else
//                        range <= range - ((prob*range)/`SCALE_FACTOR);

