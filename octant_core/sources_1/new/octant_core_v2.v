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
`define WIDTH 112
`define WIDTH_NODE 152
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

`define RES 100

`define BRANCH_WIDTH 152
 
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
//  [151 .... 24]     [23 : 8]         [7 : 3]            [2]        [1:0]
//  [O7.......O0]     [  PB  ]         [  D  ]            [L]        [xxx]
// 16 bits to Ox | 16 bits to PB  | 5 bits to depth | 1 bit to leaf | X

//`define MAX_DEPTH 14
`define WIDTH_BRAM 6

module octant_core_v2 #(parameter MAX_DEPTH = 5)(
    input wire i_clk,
    input wire i_rst_n,
    input wire i_en,
    //input wire signed [`WIDTH-1:0] i_point,
    input wire signed [`WIDTH-1:0] i_points_x,
    input wire signed [`WIDTH-1:0] i_points_y,
    input wire signed [`WIDTH-1:0] i_points_z,
    input wire signed [63:0] i_mid_point,
    input wire signed [63:0] i_near_bottom_left,
    input wire signed [63:0] i_far_top_right,
    input [15:0] i_point_cloud_size,
    output reg o_finish,
    output reg o_we_a,
    output reg o_we_b,
    output reg [`BRANCH_WIDTH-1:0] o_dina,
    input wire [`BRANCH_WIDTH-1:0] i_doutb,
    output wire [3:0] o_addra,
    output wire [3:0] o_addrb,
    output reg o_ena,
    output reg o_enb

    );

    localparam DEPTH_BITS= $clog2(MAX_DEPTH);

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
      
    //coordinates(debubg)

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

    //IP BRAM <----------------------------------------- ToDo: Mudar numero de bits para ser varialvel
    wire [`WIDTH_BRAM-1:0] addra_w;
    reg [7:0] addra_write;
    reg [7:0] addra_write_prev;
    reg [7:0] addra_read_prev;
    reg [7:0] addra_read;
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
    reg flag_child_exists;
    

    //tests - debug
    reg op1;
    reg op2;
    reg result;

    
    //***************new method BB *************** //
    reg [31:0] octree_side_len;
    reg [`BRANCH_WIDTH-1:0] branch_r;
    reg [`BRANCH_WIDTH-1:0] prev_branch_r;
    
    //wire aux debug
    wire [15:0] octant_0_write;
    wire [15:0] octant_1_write;
    wire [15:0] octant_2_write;
    wire [15:0] octant_3_write;
    wire [15:0] octant_4_write;
    wire [15:0] octant_5_write;
    wire [15:0] octant_6_write;
    wire [15:0] octant_7_write;
    wire [15:0] prev_branch_pos_write;
    wire [3:0] depth_write;
    wire leaf_write;
    //assigns wire aux debug
    assign octant_0_write = o_dina[151:136];
    assign octant_1_write = o_dina[135:120];
    assign octant_2_write = o_dina[119:104];
    assign octant_3_write = o_dina[103:88];
    assign octant_4_write = o_dina[87:72];
    assign octant_5_write = o_dina[71:56];
    assign octant_6_write = o_dina[55:40];
    assign octant_7_write = o_dina[39:24];
    assign prev_branch_pos_write = o_dina[23:8];
    assign depth_write = o_dina[7:3];
    assign leaf_write = o_dina[2];
    
    //wire aux debug bram
    wire [15:0] octant_0_bram_read;
    wire [15:0] octant_1_bram_read;
    wire [15:0] octant_2_bram_read;
    wire [15:0] octant_3_bram_read;
    wire [15:0] octant_4_bram_read;
    wire [15:0] octant_5_bram_read;
    wire [15:0] octant_6_bram_read;
    wire [15:0] octant_7_bram_read;
    wire [15:0] prev_branch_pos_bram_read;
    wire [3:0] depth_bram_read;
    wire leaf_bram_read;
    //assigns wire aux debug
    assign octant_0_bram_read = i_doutb[151:136];
    assign octant_1_bram_read = i_doutb[135:120];
    assign octant_2_bram_read = i_doutb[119:104];
    assign octant_3_bram_read = i_doutb[103:88];
    assign octant_4_bram_read = i_doutb[87:72];
    assign octant_5_bram_read = i_doutb[71:56];
    assign octant_6_bram_read = i_doutb[55:40];
    assign octant_7_bram_read = i_doutb[39:24];
    assign prev_branch_pos_bram_read = i_doutb[23:8];
    assign depth_bram_read = i_doutb[7:3];
    assign leaf_bram_read = i_doutb[2];
    
    
    //aux table with depth and last child id
    reg [7:0] table_aux [0:15];

    reg signed [95:0] table_bb_aux [0:15];
    
    wire signed [15:0] w_near_bottom_left_xtest;
    wire signed [15:0] w_near_bottom_left_ytest;
    wire signed [15:0] w_near_bottom_left_ztest;

    wire signed [15:0] w_far_top_right_xtest;
    wire signed [15:0] w_far_top_right_ytest;
    wire signed [15:0] w_far_top_right_ztest;
    
    assign w_near_bottom_left_xtest = table_bb_aux[depth][95:80];
    assign w_near_bottom_left_ytest = table_bb_aux[depth][79:64];
    assign w_near_bottom_left_ztest = table_bb_aux[depth][63:48];

    // top right point of of initial bounding box
    assign w_far_top_right_xtest = table_bb_aux[depth][47:32];
    assign w_far_top_right_ytest = table_bb_aux[depth][31:16];
    assign w_far_top_right_ztest = table_bb_aux[depth][15:0];
    
    
    reg [7:0] flag_out_of_bounds_r;
    reg flag_backtracking;

    //point and leaf counts
    reg [15:0] point_count;
    reg [15:0] leaf_count;
    reg [3:0] i;


    
    
    
    
    //**********************************************

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

    //tests
    // address wire 
    //assign addra_w = flag_read ? addra_read : addra_write;
    assign o_addra = addra_write;
    assign o_addrb = addra_read;

    /* assign branch_test = current_node_[7:0];
    assign leaf_test = current_node_[15:8];
    assign parent_pointer_test = current_node_[31:16];
    
    assign bram_node_wire_branch = i_doutb[7:0];
    assign bram_node_wire_leaf = i_doutb[15:8];
    assign bram_node_wire_pointer = i_doutb[31:16]; */

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

            //**** NEW ******//
            octree_side_len <= 51200; 
            branch_r <= 0;
            addra_read <= 1;
            addra_write <= 1;
            flag_out_of_bounds_r<=0;
            point_count <= 0;
            leaf_count <= 0;
            for ( i=0 ;i<=MAX_DEPTH;i=i+1) begin
                table_bb_aux[i] <=0;
            end
            flag_child_exists <= 0;
            //***************//
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
                    o_enb <= 1;
                end
                `CALC_MID: // state 2
                begin
                    r_mid_x <= ((near_bottom_left_x + far_top_right_x)>>>1);
                    r_mid_y <= ((near_bottom_left_y + far_top_right_y)>>>1);
                    r_mid_z <= ((near_bottom_left_z + far_top_right_z)>>>1);
                    
                    //Upper and Lower bounding box violation
                    if(x_point_ >= far_top_right_x || x_point_ < near_bottom_left_x 
                    || y_point_ >= far_top_right_y || y_point_ < near_bottom_left_y
                    || z_point_ >= far_top_right_z || z_point_ < near_bottom_left_z) 
                    begin
                        //if vilolation, keep side length to then backtrack
                        flag_out_of_bounds <= 1; 
                        //octree_side_len <= octree_side_len;
                        flag_out_of_bounds_r <= flag_out_of_bounds_r + 1;
                        //flag_backtracking <=1;
                        depth <= depth - 1;
                    end else begin
                
                        //octree_side_len <= octree_side_len >> 1;
                        table_bb_aux[depth] <= {near_bottom_left_x,near_bottom_left_y,near_bottom_left_z,far_top_right_x,far_top_right_y,far_top_right_z};
                        flag_backtracking <=0;
                        flag_out_of_bounds <= 0;

                        //test addrs update <----- CHECK PROBLEMA
                        if(depth < MAX_DEPTH) begin
                            addra_read <= addra_read + 1; // 
                            addra_write <= addra_write + 1;
                            addra_read_prev <= addra_read;
                            addra_write_prev <= addra_write;
                        end else begin
                            addra_read <= addra_read; // 
                            addra_write <= addra_write;
                            addra_read_prev <= addra_read_prev;
                            addra_write_prev <= addra_write_prev;
                        end
                        
                    end

                    we_bram <= 0;
                    o_we_a <= 0;
                    o_dina <= 0;
                    
                end
                 `OCTANT_CHECK: // state 3
                begin
                    if(!flag_out_of_bounds && depth <= MAX_DEPTH) begin
                        if(x_point_ <= r_mid_x) begin // left
                            if(y_point_ <= r_mid_y) begin // not far
                                if(z_point_ <= r_mid_z) begin // down
                                    // NEAR_BOTTOM_LEFT
                                    far_top_right_x <=  far_top_right_x - (octree_side_len >> 1);//r_mid_x; // far_top_right_x <= far_top_right_x - octree_side_len;
                                    far_top_right_y <=  far_top_right_y - (octree_side_len >> 1);//r_mid_y; // far_top_right_y <= far_top_right_y - octree_side_len;
                                    far_top_right_z <=  far_top_right_z - (octree_side_len >> 1);//r_mid_z; // far_top_right_z <= far_top_right_z - octree_side_len;
                                    near_bottom_left_x <= near_bottom_left_x;
                                    near_bottom_left_y <= near_bottom_left_y;
                                    near_bottom_left_z <= near_bottom_left_z;
                                    child_idx <= `NEAR_BOTTOM_LEFT;
                                end else begin // not down
                                    // NEAR_TOP_LEFT
                                    far_top_right_x <= far_top_right_x - (octree_side_len >> 1);
                                    far_top_right_y <= far_top_right_y - (octree_side_len >> 1);
                                    far_top_right_z <= far_top_right_z;
                                    near_bottom_left_z <= near_bottom_left_z + (octree_side_len >> 1);//near_bottom_left_z <= r_mid_z;
                                    child_idx <= `NEAR_TOP_LEFT;
                                end
                            end else begin // far
                                if (z_point_ <= r_mid_z) begin // down
                                    // FAR_BOTTOM_LEFT
                                    far_top_right_x <= far_top_right_x - (octree_side_len >> 1);//far_top_right_x <= r_mid_x;
                                    far_top_right_z <= far_top_right_z - (octree_side_len >> 1);//far_top_right_z <= r_mid_z;
                                    near_bottom_left_y <= near_bottom_left_y + (octree_side_len >> 1);//near_bottom_left_y <= r_mid_y;
                                    child_idx <= `FAR_BOTTOM_LEFT;
                                end else begin //not down
                                    // FAR_TOP_LEFT
                                    far_top_right_x <= far_top_right_x - (octree_side_len >> 1);//far_top_right_x <= r_mid_x;
                                    near_bottom_left_y <= near_bottom_left_y + (octree_side_len >> 1);//near_bottom_left_y <= r_mid_y;
                                    near_bottom_left_z <= near_bottom_left_z + (octree_side_len >> 1);//near_bottom_left_z <= r_mid_z;
                                    child_idx <= `FAR_TOP_LEFT;
                                end
                            end
                        end else begin 
                            if (y_point_ <= r_mid_y) begin
                                if(z_point_ <= r_mid_z) begin
                                    // NEAR_BOTTOM_RIGHT
                                    near_bottom_left_x <= near_bottom_left_x + (octree_side_len >> 1);//near_bottom_left_x <= r_mid_x;
                                    far_top_right_y <=  far_top_right_y - (octree_side_len >> 1);//far_top_right_y <= r_mid_y;
                                    far_top_right_z <=  far_top_right_z - (octree_side_len >> 1);//far_top_right_z <= r_mid_z;
                                    child_idx <= `NEAR_BOTTOM_RIGHT;
                                end else begin
                                    // NEAR_TOP_RIGHT
                                    near_bottom_left_x <= near_bottom_left_x + (octree_side_len >> 1);//near_bottom_left_x <= r_mid_x;
                                    far_top_right_y <=  far_top_right_y - (octree_side_len >> 1);//far_top_right_y <= r_mid_y;
                                    near_bottom_left_z <= near_bottom_left_z + (octree_side_len >> 1);//near_bottom_left_z <= r_mid_z;
                                    child_idx <= `NEAR_TOP_RIGHT; 
                                end
                            end else begin
                                if(z_point_ <= r_mid_z) begin 
                                    // FAR_BOTTOM_RIGHT
                                    near_bottom_left_x <= near_bottom_left_x + (octree_side_len >> 1);//near_bottom_left_x <= r_mid_x;
                                    near_bottom_left_y <= near_bottom_left_y + (octree_side_len >> 1);
                                    //far_top_right_y <=  far_top_right_y - octree_side_len;//far_top_right_y <= r_mid_y;
                                    far_top_right_z <=  far_top_right_z - (octree_side_len >> 1);//far_top_right_z <= r_mid_z;
                                    child_idx <= `FAR_BOTTOM_RIGHT;
                                end
                                else begin 
                                    // FAR_TOP_RIGHT
                                    near_bottom_left_x <= near_bottom_left_x + (octree_side_len >> 1);//near_bottom_left_x <= r_mid_x;
                                    near_bottom_left_y <= near_bottom_left_y + (octree_side_len >> 1);//near_bottom_left_x <= r_mid_x;
                                    near_bottom_left_z <= near_bottom_left_z + (octree_side_len >> 1);//near_bottom_left_x <= r_mid_x;
                                    //far_top_right_y <=  far_top_right_y - octree_side_len;//far_top_right_y <= r_mid_y;
                                    //near_bottom_left_z <= near_bottom_left_z + octree_side_len;//near_bottom_left_z <= r_mid_z;
                                    
                                    far_top_right_x <= far_top_right_x;
                                    far_top_right_y <= far_top_right_y;
                                    far_top_right_z <= far_top_right_z;
                                    child_idx <= `FAR_TOP_RIGHT;
                                    //curr_node[child_idx] <= 1;
                                end
                            end
                            
                        end
                        // Increase depth 
                        depth <= depth +1;
                        
                        flag_leaf <= 0;
                        flag_out_of_bounds_r =0;
                        
                    end else begin
                        //flag_max_depth <= 1;
                        //Backtrack bounding box
                        near_bottom_left_x <= table_bb_aux[depth][95:80];
                        near_bottom_left_y <= table_bb_aux[depth][79:64];
                        near_bottom_left_z <= table_bb_aux[depth][63:48];
                        far_top_right_x <= table_bb_aux[depth][47:32];
                        far_top_right_y <= table_bb_aux[depth][31:16];
                        far_top_right_z <= table_bb_aux[depth][15:0];
                
                        flag_leaf <= flag_leaf +1;
                        flag_leaf_ <= 1;
                    end                    
                    counter <= counter +1; 
                end
                `READ_BRAM: //4
                begin
                    flag_read <= 1;
                    branch_r <= i_doutb;
                    prev_branch_r <= o_dina;
                    o_dina <= i_doutb;
                    //change branch PB to previous branch <----------------------


                    //test
                    table_aux[depth] <= child_idx;

                    // assert octree side length size
                    case (depth)
                        0: begin
                            octree_side_len <= 51200;
                        end
                        1: begin 
                            octree_side_len <= 25600;
                        end
                        2: begin
                            octree_side_len <= 12800;
                        end
                        3: begin 
                            octree_side_len <= 6400;
                        end
                        4: begin 
                            octree_side_len <= 3200;
                        end
                        5: begin
                            octree_side_len <= 1600;
                        end
                        6: begin 
                            octree_side_len <= 800;
                        end
                        7: begin 
                            octree_side_len <= 400;
                        end
                        8: begin
                            octree_side_len <= 200;
                        end
                        9: begin 
                            octree_side_len <= 100;
                        end
                        10: begin 
                            octree_side_len <= 50;
                        end
                        11: begin
                            octree_side_len <= 25;
                        end
                        12: begin 
                            octree_side_len <= 12;
                        end
                        13: begin 
                            octree_side_len <= 6;
                        end
                        14: begin
                            octree_side_len <= 3;
                        end
                        15: begin 
                            octree_side_len <= 1;
                        end
                        default: begin
                            
                        end
                    endcase                 
                end
                `EVAL_NODE: //state 5
                begin
                    if(depth > MAX_DEPTH || !flag_backtracking && flag_leaf==1) begin
                        // child in branch is a LEAF
                        leaf_count <= leaf_count + 1;
                        flag_backtracking <= 1;
                        flag_out_of_bounds <= 0;

                        if(depth > MAX_DEPTH) begin
                            depth <= depth - 1;
                            near_bottom_left_x <= table_bb_aux[depth-1][95:80];
                            near_bottom_left_y <= table_bb_aux[depth-1][79:64];
                            near_bottom_left_z <= table_bb_aux[depth-1][63:48];
                            far_top_right_x <= table_bb_aux[depth-1][47:32];
                            far_top_right_y <= table_bb_aux[depth-1][31:16];
                            far_top_right_z <= table_bb_aux[depth-1][15:0];
                            //o_dina[7:3] <= depth-1;
                        end else begin
                            near_bottom_left_x <= table_bb_aux[depth][95:80];
                            near_bottom_left_y <= table_bb_aux[depth][79:64];
                            near_bottom_left_z <= table_bb_aux[depth][63:48];
                            far_top_right_x <= table_bb_aux[depth][47:32];
                            far_top_right_y <= table_bb_aux[depth][31:16];
                            far_top_right_z <= table_bb_aux[depth][15:0];
                            //o_dina[7:3] <= depth;
                        end
                        // assign LEAF in branch node
                        o_dina[151-(child_idx*16) -: 16] <= 16'd1;
                        o_we_a <= 1;
                    
                        // next point
                        if(point_count < i_point_cloud_size) begin
                            coord_pointer <= coord_pointer + 1;
                            // track n° of points
                            point_count <= point_count + 1;
                            flag_out_of_bounds <= 0;
                        end else begin 
                            o_finish <= 1;
                        end

                        //test debug
                        flag_child_exists <= 0;

                    end else begin
                        //child is not a LEAF keep going

                        // if already exist child do not change child node
                        if((i_doutb[151-(child_idx*16) -: 16]) > 0) begin

                            addra_read <= i_doutb[151-(child_idx*16) -: 16]; // addr to read next branch
                            addra_write <= i_doutb[151-(child_idx*16) -: 16];
                            //test debug
                            flag_child_exists <= 1;

                        end else begin
                            
                            branch_r[151-(child_idx*16) -: 16] <= addra_write + 1; // child branch, next position
                            branch_r[7:3] <= depth;
                            //branch_r[23:8] <= prev_branch_r[] // previous branch position in BRAM
                            o_dina[151-(child_idx*16) -: 16] <= addra_write + 1;
                            o_dina[7:3] <= depth;
                            o_dina[23:8] <= addra_read_prev;
                            o_we_a <= 1;
                            

                            //test debug
                            flag_child_exists <= 0;

                        end

                        // keep point
                        coord_pointer <= coord_pointer; 
                    end
                    flag_read <= 0;
                    
                                            
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
                    if (!i_en) begin
                        state <= `IDLE;
                    end else
                        state <= `CALC_MID;                                                     
                end
                default: 
                begin
                
                end
            endcase
        end
    end




    bram #(.WIDTH(152),.DEPTH(16)) uut_bram(
        .i_clk(i_clk),
        .i_we(o_we_a),
        .i_data_in(o_dina),
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


// if(flag_out_of_bounds || flag_max_depth )begin 
//                         //go back to parent node, and parent bounding bb
//                         // Problema, só tem de alterar o nó folha uma vez, depois é fazer backtrack até pertencer à BB !!!! <-------
//                         if (flag_leaf == 1) begin
                            
//                             //keep read addr because found a leaf(first time out of bounds)
//                             addra_read <= addra_read;
//                             addra_write <= addra_write;
//                             branch_r[151-(child_idx*16) -: 16] <= 16'd1;
//                             branch_r[7:3] <= depth; 
//                         end else begin

                             
//                             // go back to parent branch 
//                             addra_read <= branch_r[23:8];
//                             addra_write <= branch_r[23:8];

//                         end 
//                         //Keep current point to compare in parent BB
//                         flag_out_of_bounds <= 0;
//                         flag_max_depth <= 0;
//                         //****** New ******
//                         //assign octant position with 1 to signal LEAF
                       
                        
//                         //branch_r[23:8] <=  prev_branch_pos;
                        
                        
//                         //*****************//
//                     end
//                     else begin
//                         //**************** OLD *********************//

//                         //Point belongs in current bounding box and max depth not reached
//                         //Update branch in current node and pointer to parent
//                         //current_node[child_idx] <= 1'b1; //branch mask
//                         //current_node[child_idx + 8] <= 1'b0; //leaf mask
//                         //current_node[31:16] <= addr_bram_r; // ver parent pointer ??????
//                         //parent_node <= current_node;

//                         current_node_[child_idx] <= 1'b1;
//                         current_node_[child_idx + 8] <= current_node_[child_idx + 8];  
//                         current_node_[31:16] <= parent_ptr;
//                         //current_node_[31:16] <= parent_node_[31:16] + 1; //????
                        
//                         //ptr update
//                         parent_ptr <= current_node_[31:16]; 

//                         //Next point
//                         //coord_pointer <= coord_pointer + 1;
//                          //*****************************************//

//                         //****** New ******//
//                         //assign octant position with next bram position to signal next branch
//                         branch_r[151-(child_idx*16) -: 16] <= o_addrb + 1;
                        
//                         branch_r[23:8] <=  addra_read;
//                         branch_r[7:3] <= depth; 

//                         addra_write <= addra_write +1;
//                         addra_read <= addra_read +1;
//                         addra_write_prev <= addra_write;
//                         addra_read_prev <= addra_read;
                        
//                         //*****************//


//                     end 
                    

 /* end else begin
                            case (table_aux[depth])
                                `NEAR_BOTTOM_LEFT:
                                begin
                                    near_bottom_left_x <= near_bottom_left_x;
                                    near_bottom_left_y <= near_bottom_left_y;
                                    near_bottom_left_z <= near_bottom_left_z;
                                    far_top_right_x <=  far_top_right_x + octree_side_len;//far_top_right_x <= (r_mid_x <<< 1) - near_bottom_left_x;
                                    far_top_right_y <=  far_top_right_y + octree_side_len;//far_top_right_y <= (r_mid_y <<< 1) - near_bottom_left_y;
                                    far_top_right_z <=  far_top_right_z + octree_side_len;//far_top_right_z <= (r_mid_z <<< 1) - near_bottom_left_z;                        
                                end
                                `NEAR_TOP_LEFT:
                                begin
                                    near_bottom_left_x <= near_bottom_left_x;
                                    near_bottom_left_y <= near_bottom_left_y;
                                    near_bottom_left_z <= near_bottom_left_z - octree_side_len;//near_bottom_left_z <= (r_mid_z <<< 1) - far_top_right_z;
                                    far_top_right_x <= far_top_right_x + octree_side_len;//far_top_right_x <= (r_mid_x <<< 1) - near_bottom_left_x;
                                    far_top_right_y <= far_top_right_y + octree_side_len;//far_top_right_y <= (r_mid_y <<< 1) - near_bottom_left_y;
                                    far_top_right_z <= far_top_right_z;
                                end  
                                `FAR_BOTTOM_LEFT:
                                begin
                                    near_bottom_left_x <= near_bottom_left_x;
                                    near_bottom_left_y <= near_bottom_left_y - octree_side_len;//near_bottom_left_y <= (r_mid_y <<< 1) - far_top_right_y;
                                    near_bottom_left_z <= near_bottom_left_z;
                                    far_top_right_x <= far_top_right_x + octree_side_len;//far_top_right_x <= (r_mid_x <<< 1) - near_bottom_left_x;
                                    far_top_right_y <= far_top_right_y;
                                    far_top_right_z <= far_top_right_z + octree_side_len;//far_top_right_z <= (r_mid_z <<< 1) - near_bottom_left_z;
                                end
                                `FAR_TOP_LEFT:
                                begin
                                    near_bottom_left_x <= near_bottom_left_x;
                                    near_bottom_left_y <= near_bottom_left_y - octree_side_len;//near_bottom_left_y <= (r_mid_y <<< 1) - far_top_right_y;
                                    near_bottom_left_z <= near_bottom_left_z - octree_side_len;//near_bottom_left_z <= (r_mid_z <<< 1) - far_top_right_z;
                                    far_top_right_x <= far_top_right_x + octree_side_len;//far_top_right_x <= (r_mid_x <<< 1) - near_bottom_left_x;
                                    far_top_right_y <= far_top_right_y;
                                    far_top_right_z <= far_top_right_z;
                                end
                                `NEAR_BOTTOM_RIGHT:
                                begin
                                    near_bottom_left_x <= near_bottom_left_x - octree_side_len;//near_bottom_left_x <= (r_mid_x <<< 1) - far_top_right_x;
                                    near_bottom_left_y <= near_bottom_left_y;
                                    near_bottom_left_z <= near_bottom_left_y;
                                    far_top_right_x <= far_top_right_x;
                                    far_top_right_y <=  far_top_right_y + octree_side_len;//far_top_right_y <= (r_mid_y <<< 1) - near_bottom_left_y;
                                    far_top_right_z <=  far_top_right_z + octree_side_len;//far_top_right_z <= (r_mid_z <<< 1) - near_bottom_left_z;
                                end
                                `NEAR_TOP_RIGHT:
                                begin
                                    near_bottom_left_x <= near_bottom_left_x - octree_side_len;//near_bottom_left_x <= (r_mid_x <<< 1) - far_top_right_x;
                                    near_bottom_left_y <= near_bottom_left_y;
                                    near_bottom_left_z <= near_bottom_left_z - octree_side_len;//near_bottom_left_z <= (r_mid_z <<< 1) - far_top_right_z;
                                    far_top_right_x <= far_top_right_x;
                                    far_top_right_y <= far_top_right_y + octree_side_len;//far_top_right_y <= (r_mid_y <<< 1) - near_bottom_left_y;
                                    far_top_right_z <= far_top_right_z;
                                end  
                                `FAR_BOTTOM_RIGHT:
                                begin
                                    near_bottom_left_x <= near_bottom_left_x - octree_side_len;//near_bottom_left_x <= (r_mid_x <<< 1) - far_top_right_x;
                                    near_bottom_left_y <= near_bottom_left_y - octree_side_len; 
                                    near_bottom_left_z <= near_bottom_left_z;
                                    far_top_right_x <= far_top_right_x;
                                    far_top_right_y <= far_top_right_y;//far_top_right_y <= (r_mid_y <<< 1) - near_bottom_left_y;
                                    far_top_right_z <= far_top_right_z + octree_side_len;//far_top_right_z <= (r_mid_z <<< 1) - near_bottom_left_z;
                                end
                                `FAR_TOP_RIGHT:
                                begin
                                    near_bottom_left_x <= near_bottom_left_x - octree_side_len;//near_bottom_left_x <= ((r_mid_x <<< 1) - far_top_right_x);
                                    near_bottom_left_y <= near_bottom_left_y - octree_side_len;
                                    near_bottom_left_z <= near_bottom_left_z - octree_side_len;//near_bottom_left_z <= (r_mid_z <<< 1) - far_top_right_z;
                                    far_top_right_x <= far_top_right_x;
                                    far_top_right_y <= far_top_right_y; //far_top_right_y <= (r_mid_y <<< 1) - near_bottom_left_y;
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
                        end */
                        
                       
                        // Decrease depth 
                        /* if(depth==0) begin
                            depth <= depth;
                        end else begin
                            depth <= depth -1;    
                        end */