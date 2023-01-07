`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/21/2022 08:52:41 PM
// Design Name: 
// Module Name: bfs_core_tb
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


module bfs_core_tb();

    reg clock;
    reg reset;
    reg en;

    reg [151:0] branch;
    
    reg [151:0] branch_table [0:7];
    

    initial begin 
        clock = 1;
        reset = 1;
        
        #20;
        reset = 0;
        branch_table[0][151:136] = 0; // 7
        branch_table[0][135:120] = 0; // 6
        branch_table[0][119:104] = 0; // 5
        branch_table[0][103:88]  = 3; // 4
        branch_table[0][87:72]   = 0; // 3
        branch_table[0][71:56]   = 0;  // 2
        branch_table[0][55:40]   = 0;  // 1
        branch_table[0][39:24]   = 7;  // 0      
        branch_table[0][23:0]    = 0;
        
        branch_table[1][151:136] = 0; // 7
        branch_table[1][135:120] = 0; // 6
        branch_table[1][119:104] = 0; // 5
        branch_table[1][103:88]  = 0; // 4
        branch_table[1][87:72]   = 4; // 3
        branch_table[1][71:56]   = 0;  // 2
        branch_table[1][55:40]   = 0;  // 1
        branch_table[1][39:24]   = 0;  // 0      
        branch_table[1][23:0]    = 0;
        
        branch_table[2][151:136] = 0; // 7
        branch_table[2][135:120] = 0; // 6
        branch_table[2][119:104] = 0; // 5
        branch_table[2][103:88]  = 0; // 4
        branch_table[2][87:72]   = 5; // 3
        branch_table[2][71:56]   = 0;  // 2
        branch_table[2][55:40]   = 0;  // 1
        branch_table[2][39:24]   = 0;  // 0      
        branch_table[2][23:0]    = 0;
        
        branch_table[3][151:136] = 0; // 7
        branch_table[3][135:120] = 0; // 6
        branch_table[3][119:104] = 0; // 5
        branch_table[3][103:88]  = 0; // 4
        branch_table[3][87:72]   = 6; // 3
        branch_table[3][71:56]   = 0;  // 2
        branch_table[3][55:40]   = 0;  // 1
        branch_table[3][39:24]   = 0;  // 0      
        branch_table[3][23:0]    = 0;            
        
        branch_table[7][151:136] = 8; // 7
        branch_table[7][135:120] = 0; // 6
        branch_table[7][119:104] = 0; // 5
        branch_table[7][103:88]  = 0; // 4
        branch_table[7][87:72]   = 0; // 3
        branch_table[7][71:56]   = 0;  // 2
        branch_table[7][55:40]   = 0;  // 1
        branch_table[7][39:24]   = 0;  // 0      
        branch_table[7][23:0]    = 0;       
        
        #40
        en = 1;
        reset = 1;
        
        branch = branch_table[0];
        
        
        #500;
        $finish;
            
       
        
    
    end
    

    
    
    always #10 clock <= ~clock;
    
    
    bfs_core uut_bfs(
        .i_clk(clock),
        .i_en(en),
        .i_doutb(branch),
        .i_rst_n(reset)
    );


endmodule
