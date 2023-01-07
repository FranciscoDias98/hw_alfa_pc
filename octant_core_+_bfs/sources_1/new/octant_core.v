`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/24/2022 09:37:27 AM
// Design Name: 
// Module Name: octant_core
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
`define WIDTH 64
`define WIDTH_NODE 32
`define IDLE 0
`define INIT 1
`define CALC_MID 2
`define READ_BRAM 4
`define OCTANT_CHECK 3
`define EVAL_NODE 5
`define BRAM_INS 6

`define NEAR_BOTTOM_LEFT 0
`define NEAR_TOP_LEFT 1
`define FAR_BOTTOM_LEFT 2
`define FAR_TOP_LEFT 3
`define NEAR_BOTTOM_RIGHT 4
`define NEAR_TOP_RIGHT 5
`define FAR_BOTTOM_RIGHT 6
`define FAR_TOP_RIGHT 7

`define RES 10
 
//`define CHILD_0 8'b0000_0001;
//`define CHILD_1 8'b0000_0010; 
//`define CHILD_2 8'b0000_0100; 
//`define CHILD_3 8'b0000_1000; 
//`define CHILD_4 8'b0001_0000;
//`define CHILD_5 8'b0010_0000; 
//`define CHILD_6 8'b0100_0000;   
//`define CHILD_7 8'b1000_0000;  

/*    3-------7
     /|      /|
    1-+-----5 |
    | |     | |   Z
    | 2-----+-6   | Y
    |/      |/    |/
    0-------4     +--X

    0   =>   0, 0, 0
    1   =>   0, 0, 1
    2   =>   0, 1, 0
    3   =>   0, 1, 1
    4   =>   1, 0, 0
    5   =>   1, 0, 1
    6   =>   1, 1, 0
    7   =>   1, 1, 1   */


// Branch Memory Organization //
//  [151 .... 24]     [23 : 7]         [6 : 2]            [1]        [0]
//  [O7.......O0]     [  PB  ]         [  D  ]            [L]        [X]
// 16 bits to Ox | 16 bits to PB  | 4 bits to depth | 1 bit to leaf | X

`define MAX_DEPTH 14
`define WIDTH_BRAM 6

module octant_core(
    input wire i_clk,
    input wire i_rst_n,
    input wire i_en,
    //input wire signed [`WIDTH-1:0] i_point,
    input wire signed [`WIDTH-1:0] i_points_x,
    input wire signed [`WIDTH-1:0] i_points_y,
    input wire signed [`WIDTH-1:0] i_points_z,
    input wire signed [`WIDTH-1:0] i_mid_point,
    input wire signed [`WIDTH-1:0] i_near_bottom_left,
    input wire signed [`WIDTH-1:0] i_far_top_right,
    input [15:0] i_point_cloud_size,
    output reg o_finish,
    output reg [3:0]  o_we_a,
    output reg [3:0]  o_we_b,
    output reg [31:0] o_dina,
    input wire [31:0] i_doutb,
    output wire [5:0] o_addra,
    output wire [5:0] o_addrb,
    output reg o_ena,
    output reg o_enb

    );

    localparam DEPTH_BITS= $clog2(`MAX_DEPTH);

    //reg inter.
    reg [3:0] state;                        //  | 16 bit | 16 bit | 16 bit | XXXX |

    //reg new coordinates for bounding box
    reg [7:0] coord_pointer;

    reg signed [15:0] near_bottom_left_x;
    reg signed [15:0] near_bottom_left_y;
    reg signed [15:0] near_bottom_left_z;

    reg signed [15:0] far_top_right_x;
    reg signed [15:0] far_top_right_y;
    reg signed [15:0] far_top_right_z;


    reg signed [15:0] r_mid_x;
    reg signed [15:0] r_mid_y;
    reg signed [15:0] r_mid_z;
    
    reg [63:0] point;
    reg [2:0] child_idx;
    reg [7:0] depth;
    reg [7:0] counter;
      
    //node reg // 16 bits for pointer to children, 8 bits for occupancy code
    reg [`WIDTH_NODE-1:0] current_node;  
    reg [`WIDTH_NODE-1:0] parent_node;  

    reg [`WIDTH_NODE-1:0] current_node_;  
    reg [`WIDTH_NODE-1:0] parent_node_; 

    //coordinates(debubg)
    wire signed [15:0] x_mid;
    wire signed [15:0] y_mid;
    wire signed [15:0] z_mid;

    wire signed [15:0] x_point_; 
    wire signed [15:0] y_point_;
    wire signed [15:0] z_point_;

    wire signed [15:0] x_point; 
    wire signed [15:0] y_point;
    wire signed [15:0] z_point;

    wire signed [15:0] w_near_bottom_left_x;
    wire signed [15:0] w_near_bottom_left_y;
    wire signed [15:0] w_near_bottom_left_z;

    wire signed [15:0] w_far_top_right_x;
    wire signed [15:0] w_far_top_right_y;
    wire signed [15:0] w_far_top_right_z;

    //IP BRAM 
    wire [`WIDTH_BRAM-1:0] addra_w;
    reg [`WIDTH_BRAM-1:0] addra_write;
    reg [`WIDTH_BRAM-1:0] addra_write_prev;
    reg [`WIDTH_BRAM-1:0] addra_read_prev;
    reg [`WIDTH_BRAM-1:0] addra_read;
    reg flag_read;

    reg [15:0] parent_ptr;
    reg [15:0] node_ptr;

    //bram regs.
    reg [15:0] addr_bram_r; // wr
    reg [15:0] addr_bram_w; // rd
    reg [`WIDTH_NODE-1:0] node_bram;
    reg we_bram;
    wire [`WIDTH_NODE-1:0] node_bram_w;
    
    wire [7:0] bram_node_wire_branch;
    wire [15:8] bram_node_wire_leaf;
    wire [31:16] bram_node_wire_pointer;

    //reg flags
    reg flag_out_of_bounds;
    reg flag_max_depth;
    reg [DEPTH_BITS-1:0] flag_leaf;
    reg flag_leaf_;

    

    //tests - debug
    reg op1;
    reg op2;
    reg result;
    
    wire [7:0] branch_test;
    wire [7:0] leaf_test;
    wire [15:0] parent_pointer_test;
    
    reg [`WIDTH_NODE-1:0] node_test;
    
    //////new method BB //////
    reg [31:0] octree_side_len;
    
    
    //////////////////////////

    //wires for points coordinates//

    //current point
    assign x_point_ = i_points_x[coord_pointer*16 +:16];
    assign y_point_ = i_points_y[coord_pointer*16 +:16];
    assign z_point_ = i_points_z[coord_pointer*16 +:16];
    
    // bottom left point of initial bounding box
    assign w_near_bottom_left_x = i_near_bottom_left[63 -:16];
    assign w_near_bottom_left_y = i_near_bottom_left[47 -:16];
    assign w_near_bottom_left_z = i_near_bottom_left[31 -:16];

    // top right point of of initial bounding box
    assign w_far_top_right_x = i_far_top_right[63 -:16];
    assign w_far_top_right_y = i_far_top_right[47 -:16];
    assign w_far_top_right_z = i_far_top_right[31 -:16];
    
    //mid point of bounding box
    assign x_mid = i_mid_point[63 -:16];
    assign y_mid = i_mid_point[47 -:16];
    assign z_mid = i_mid_point[31 -:16];


    //tests
    // address wire 
    //assign addra_w = flag_read ? addra_read : addra_write;
    assign o_addra = addra_write;
    assign o_addrb = addra_read;

    assign branch_test = current_node_[7:0];
    assign leaf_test = current_node_[15:8];
    assign parent_pointer_test = current_node_[31:16];
    
    assign bram_node_wire_branch = i_doutb[7:0];
    assign bram_node_wire_leaf = i_doutb[15:8];
    assign bram_node_wire_pointer = i_doutb[31:16];

    always @(posedge i_clk) begin
        if(!i_rst_n) begin
            o_finish <= 0;
            depth <= 0;
            counter <= 0;
            child_idx <= 0;
            addr_bram_w <= 0;
            addr_bram_r <= 0;
            we_bram <= 0;
            near_bottom_left_x <= 0;
            near_bottom_left_y <= 0;
            near_bottom_left_z <= 0;
            far_top_right_x <= 0;
            far_top_right_y <= 0;
            far_top_right_z <= 0;
            flag_out_of_bounds <= 0;
            flag_max_depth <= 0;
            parent_node <= 0;
            current_node <= 0;
            parent_node_ <= 0;
            current_node_ <= 31'hfffffff;
            o_we_a <= 0;
            o_we_b <=0;
            o_dina <= 0;
            //o_addra <= 0;
            o_ena <= 0;
            o_enb <= 0;
            r_mid_x <= 0;
            r_mid_y <= 0;
            r_mid_z <= 0;
            coord_pointer <= 0;
            addra_read <= 0;
            addra_write <= 0;
            addra_read_prev <= 1;
            addra_write_prev <= 1;
            flag_read <= 0;
            flag_leaf <= 0;
            parent_ptr <= 0;
            flag_leaf_ <= 0;
            octree_side_len <= 0; 
        end else begin
            case (state)
                `IDLE: //state 0
                begin
                    o_finish <= o_finish;
                end
                `INIT: // state 1
                begin
                    near_bottom_left_x <= w_near_bottom_left_x;
                    near_bottom_left_y <= w_near_bottom_left_y;
                    near_bottom_left_z <= w_near_bottom_left_z;
                    far_top_right_x <= w_far_top_right_x;
                    far_top_right_y <= w_far_top_right_y;
                    far_top_right_z <= w_far_top_right_z;
                    o_ena <= 1;
                end
                `CALC_MID: // state 2
                begin
                    r_mid_x <= ((near_bottom_left_x + far_top_right_x)>>>1);
                    r_mid_y <= ((near_bottom_left_y + far_top_right_y)>>>1);
                    r_mid_z <= ((near_bottom_left_z + far_top_right_z)>>>1);
                    
                    //Upper and Lower bounding box violation
                    if(x_point_ >= far_top_right_x || x_point_ < near_bottom_left_x) begin
                        flag_out_of_bounds <= 1;
                    end                 
                    if(y_point_ >= far_top_right_y || y_point_ < near_bottom_left_y)begin
                        flag_out_of_bounds <= 1;
                    end                 
                    if(z_point_ >= far_top_right_z || z_point_ < near_bottom_left_z)begin
                        flag_out_of_bounds <= 1;
                    end 
                    
                    
                    ///// test new method for BB //////
                    
                    octree_side_len <= (1 << depth);             
                    /////////////////////////////////////

                   

                    we_bram <= 0;
                    o_we_a <= 4'b0000;
                    
                end
                 `OCTANT_CHECK: // state 3
                begin
                    if(!flag_out_of_bounds && depth <= `MAX_DEPTH) begin
                        if(x_point_ <= r_mid_x) begin // left
                            if(y_point_ <= r_mid_y) begin // not far
                                if(z_point <= r_mid_z) begin // down
                                    // NEAR_BOTTOM_LEFT
                                    far_top_right_x <= r_mid_x; // subtrair o sidelen ????
                                    far_top_right_y <= r_mid_y;
                                    far_top_right_z <= r_mid_z;
                                    child_idx <= `NEAR_BOTTOM_LEFT;
                                end else begin // not down
                                    // NEAR_TOP_LEFT
                                    far_top_right_x <= r_mid_x;
                                    far_top_right_y <= r_mid_y;
                                    near_bottom_left_z <= r_mid_z;
                                    child_idx <= `NEAR_TOP_LEFT;
                                end
                            end else begin // far
                                if (z_point_ <= r_mid_z) begin // down
                                    // FAR_BOTTOM_LEFT
                                    far_top_right_x <= r_mid_x;
                                    near_bottom_left_y <= r_mid_y;
                                    far_top_right_z <= r_mid_z;
                                    child_idx <= `FAR_BOTTOM_LEFT;
                                end else begin //not down
                                    // FAR_TOP_LEFT
                                    far_top_right_x <= r_mid_x;
                                    near_bottom_left_y <= r_mid_y;
                                    near_bottom_left_z <= r_mid_z;
                                    child_idx <= `FAR_TOP_LEFT;
                                end
                            end
                        end else begin 
                            if (y_point_ <= r_mid_y) begin
                                if(z_point_ <= r_mid_z) begin
                                    // NEAR_BOTTOM_RIGHT
                                    near_bottom_left_x <= r_mid_x;
                                    far_top_right_y <= r_mid_y;
                                    far_top_right_z <= r_mid_z;
                                    child_idx <= `NEAR_BOTTOM_RIGHT;
                                end else begin
                                    // NEAR_TOP_RIGHT
                                    near_bottom_left_x <= r_mid_x;
                                    far_top_right_y <= r_mid_y;
                                    near_bottom_left_z <= r_mid_z;
                                    child_idx <= `NEAR_TOP_RIGHT;
                                end
                            end else begin
                                if(z_point_ <= r_mid_z) begin
                                    // FAR_BOTTOM_RIGHT
                                    near_bottom_left_x <= r_mid_x;
                                    far_top_right_y <= r_mid_y;
                                    far_top_right_z <= r_mid_z;
                                    child_idx <= `FAR_BOTTOM_RIGHT;
                                end
                                else begin
                                    // FAR_TOP_RIGHT
                                    near_bottom_left_x <= r_mid_x;
                                    far_top_right_y <= r_mid_y;
                                    near_bottom_left_z <= r_mid_z;
                                    child_idx <= `FAR_TOP_RIGHT;
                                    //curr_node[child_idx] <= 1;
                                end
                            end
                            
                        end
                        // Increase depth 
                        depth <= depth +1;

                        flag_leaf <= 0;
                        
                    end else begin
                        // Point is out of bounds or max depth is reached // <------ VER FLAG MAX DEPTH
                        // | 16 bits | 8 bits leaf mask| 8 bits branch mask | -> node  
                        flag_max_depth <= 1;

                        //Backtrack bounding box
                        case (child_idx)
                            `NEAR_BOTTOM_LEFT:
                            begin
                                near_bottom_left_x <= near_bottom_left_x;
                                near_bottom_left_y <= near_bottom_left_y;
                                near_bottom_left_z <= near_bottom_left_z;
                                far_top_right_x <= (r_mid_x <<< 1) - near_bottom_left_x;
                                far_top_right_y <= (r_mid_y <<< 1) - near_bottom_left_y;
                                far_top_right_z <= (r_mid_z <<< 1) - near_bottom_left_z;
                            end
                            `NEAR_TOP_LEFT:
                            begin
                                near_bottom_left_x <= near_bottom_left_x;
                                near_bottom_left_y <= near_bottom_left_y;
                                near_bottom_left_z <= (r_mid_z <<< 1) - far_top_right_z;
                                far_top_right_x <= (r_mid_x <<< 1) - near_bottom_left_x;
                                far_top_right_y <= (r_mid_y <<< 1) - near_bottom_left_y;
                                far_top_right_z <= far_top_right_z;
                            end  
                            `FAR_BOTTOM_LEFT:
                            begin
                                near_bottom_left_x <= near_bottom_left_x;
                                near_bottom_left_y <= (r_mid_y <<< 1) - far_top_right_y;
                                near_bottom_left_z <= near_bottom_left_z;
                                far_top_right_x <= (r_mid_x <<< 1) - near_bottom_left_x;
                                far_top_right_y <= far_top_right_y;
                                far_top_right_z <= (r_mid_z <<< 1) - near_bottom_left_z;
                            end
                            `FAR_TOP_LEFT:
                            begin
                                near_bottom_left_x <= near_bottom_left_x;
                                near_bottom_left_y <= (r_mid_y <<< 1) - far_top_right_y;
                                near_bottom_left_z <= (r_mid_z <<< 1) - far_top_right_z;
                                far_top_right_x <= (r_mid_x <<< 1) - near_bottom_left_x;
                                far_top_right_y <= far_top_right_y;
                                far_top_right_z <= far_top_right_z;
                            end
                            `NEAR_BOTTOM_RIGHT:
                            begin
                                near_bottom_left_x <= (r_mid_x <<< 1) - far_top_right_x;
                                near_bottom_left_y <= near_bottom_left_y;
                                near_bottom_left_z <= near_bottom_left_y;
                                far_top_right_x <= far_top_right_x;
                                far_top_right_y <= (r_mid_y <<< 1) - near_bottom_left_y;
                                far_top_right_z <= (r_mid_z <<< 1) - near_bottom_left_z;
                            end
                            `NEAR_TOP_RIGHT:
                            begin
                                near_bottom_left_x <= (r_mid_x <<< 1) - far_top_right_x;
                                near_bottom_left_y <= near_bottom_left_y;
                                near_bottom_left_z <= (r_mid_z <<< 1) - far_top_right_z;
                                far_top_right_x <= far_top_right_x;
                                far_top_right_y <= (r_mid_y <<< 1) - near_bottom_left_y;
                                far_top_right_z <= far_top_right_z;
                            end  
                            `FAR_BOTTOM_RIGHT:
                            begin
                                near_bottom_left_x <= (r_mid_x <<< 1) - far_top_right_x;
                                near_bottom_left_y <= near_bottom_left_y;
                                near_bottom_left_z <= near_bottom_left_z;
                                far_top_right_x <= far_top_right_x;
                                far_top_right_y <= (r_mid_y <<< 1) - near_bottom_left_y;
                                far_top_right_z <= (r_mid_z <<< 1) - near_bottom_left_z;
                            end
                            `FAR_TOP_RIGHT:
                            begin
                                near_bottom_left_x <= ((r_mid_x <<< 1) - far_top_right_x);
                                near_bottom_left_y <= near_bottom_left_y;
                                near_bottom_left_z <= (r_mid_z <<< 1) - far_top_right_z;
                                far_top_right_x <= far_top_right_x;
                                far_top_right_y <= (r_mid_y <<< 1) - near_bottom_left_y;
                                far_top_right_z <= far_top_right_z;
                            end    
                            default:
                            begin
//                                near_bottom_left_x <= near_bottom_left_x;
//                                near_bottom_left_y <= near_bottom_left_y;
//                                near_bottom_left_z <= near_bottom_left_z;
//                                far_top_right_x <= far_top_right_x;
//                                far_top_right_y <= far_top_right_y;
//                                far_top_right_z <= far_top_right_z;
                            end 
                        endcase
                        // Decrease depth 
                        depth <= depth -1;
                        flag_leaf <= flag_leaf +1;
                        flag_leaf_ <= 1;
                    end                    
                    //Read next position in BRAM 
                    //parent_node_ <= current_node_; 
                    
                    counter <= counter +1; 
                end
                `READ_BRAM: //4
                begin
                    flag_read <= 1;
                    if(flag_leaf == 1) begin
                        current_node_ <= current_node_;
                        flag_leaf_ <= 0;
                        //addra_read  -> parent ?
                        // addra_write -> old ?
                         
                        
                    end else begin
                         current_node_ <= i_doutb;
                    end       
                end
               
                `EVAL_NODE: //state 5
                begin
                    if(flag_out_of_bounds || flag_max_depth )begin 
                        //go back to parent node, and parent bounding bb
                        // Problema, só tem de alterar o nó folha uma vez, depois é fazer backtrack até pertencer à BB !!!! <-------
                        if (flag_leaf == 1) begin
                            current_node_[child_idx] <= current_node_[child_idx];
                            current_node_[child_idx + 8] <= 1'b1;
                            current_node_[31:16] <= parent_ptr; // parent_ptr
                            // addr write -> parent_ptr
                            //addra_write <= addra_write;
                            //addra_read <= addra_read;
                        end else begin
                             //update addr read to parent node
                            //addra_read <= current_node_[31:16];
                             current_node_[child_idx] <= current_node_[child_idx];
                             current_node_[child_idx + 8] <= current_node_[child_idx + 8];
                        
                            //Parent ptr update
                            //parent_ptr <= current_node_[31:16];
                        end 
                        //Keep current point to compare in parent BB
                        coord_pointer <= coord_pointer;   
                        
                        flag_out_of_bounds <= 0;
                        flag_max_depth <= 0;
                        

                    end
                    else begin
                        //Point belongs in current bounding box and max depth not reached
                        //Update branch in current node and pointer to parent
                        //current_node[child_idx] <= 1'b1; //branch mask
                        //current_node[child_idx + 8] <= 1'b0; //leaf mask
                        //current_node[31:16] <= addr_bram_r; // ver parent pointer ??????
                        //parent_node <= current_node;

                        current_node_[child_idx] <= 1'b1;
                        current_node_[child_idx + 8] <= current_node_[child_idx + 8];  
                        current_node_[31:16] <= parent_ptr;
                        //current_node_[31:16] <= parent_node_[31:16] + 1; //????
                        
                        //ptr update
                        parent_ptr <= current_node_[31:16]; 

                        //Next point
                        coord_pointer <= coord_pointer + 1;
                    end
                    
                    //Finish processing all points
                    if(coord_pointer == i_point_cloud_size) begin
                        o_finish <= 1'b1; 
                        coord_pointer <= 0;
                    end
                    flag_read <= 0;
                    
                                            
                end
                `BRAM_INS: //state 6
                begin
                    //Old BRAM
                    we_bram <= 1;
                    addr_bram_r <= addr_bram_r + 1;
                    addr_bram_w <= addr_bram_w + 1;
                    
                    //test 
                    
                    addra_write_prev <= addra_write;
                    addra_read_prev <= addra_read;
                    //addra_read  <= addra_read + 1;


                    if(flag_leaf)begin
                        addra_write <= current_node_[31:16]; // pointer to parent ??
                        addra_read <= addra_read;
                    end else begin
                        //update addr write
                        addra_write <= addra_write+1;
                        
                        //update addr read
                        addra_read <= addra_read+1;
                        parent_node_ <= current_node;
                    end
                   

                    //reg IP BRAM
                    //o_addra <= addra_w;
                    o_we_a <= 4'b1111;
                    o_dina <= current_node_;          
                    
                    //current_node_ <= i_doutb;   
                    //current_node_ <= i_douta;
                    //parent_node_ <= current_node_; 
                end
                default: 
                begin
                
                end
            endcase
        end
    end

    // module FSM
    always @(posedge i_clk) begin
        if (!i_rst_n) begin
            state <= 0;
        end
        else begin
            case (state)
                `IDLE: //0
                begin
                    if(i_en)
                        state <= `INIT;
                    else
                        state <= `IDLE;                 
                end
                `INIT: //1
                begin
                    state <= `CALC_MID;
                end
                `CALC_MID: //2
                begin
                    state <= `OCTANT_CHECK;
                end
                `OCTANT_CHECK: //3
                begin
                    state <= `READ_BRAM;
                end
                `READ_BRAM: //4
                begin
                    state <= `EVAL_NODE;
                end
                `EVAL_NODE: //5
                begin
                    state <= `BRAM_INS;                                                     
                end
                `BRAM_INS: //6
                begin
                    if(!i_en) begin
                        state <= `IDLE;
                    end
                    else
                        state <= `CALC_MID;
                end

                default: 
                begin
                
                end
            endcase
        end
    end



    bram #(.WIDTH(32),.DEPTH(4)) uut_bram(
        .i_clk(i_clk),
        .i_we(o_we_a),
        .i_data_in(current_node_),
        .i_addr_write(addra_write),
        .i_addr_read(addra_read),
        .o_data_out(node_bram_w)      
    );    
    
    
/*module bram#(
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
*/

endmodule
