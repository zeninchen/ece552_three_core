`default_nettype none
// write back stage: select data to write back to register file
module writeback (
    // input  wire        i_Jalr,  // for jalr target addr selection
    // input  wire [31:0] i_branch_jal_target_addr,
    // input  wire [31:0] i_jalr_target_addr,    //jalr target addr or alu result
    input wire [31:0] i_next_pc,
    input  wire [31:0] i_alu_result,    //alu result
    input  wire [31:0] i_pc_4,
    input  wire [31:0] i_mem_read_data,
    input  wire [1:0]  i_WriteRegSrc,

    input wire [31:0] i_rs1_data,
    input wire [31:0] i_rs2_data,
    input wire[3:0]  i_Mask, // for store instruction byte/halfword/word selection
    input wire [2:0]  i_funct3, // for memory access byte/halfword/word selection
    input wire [31:0] i_inst, // for jalr target calculation and misaligned detection
    input wire [31:0] i_pc, 
    input wire        i_misaligned, // for misaligned load/store detection

    output wire        o_misaligned, // pass through for misaligned load/store detection
    output wire [31:0] o_pc,
    output wire [31:0] o_inst, // pass through for hazard detection
    output wire [31:0] o_rs1_data,
    output wire [31:0] o_rs2_data,

    output wire [31:0] o_next_pc,
    output wire [31:0] o_writeback_data
);
    assign o_next_pc = i_next_pc;
    // Extract byte/halfword selected by mask (NO variable shift)
    wire [7:0] load_byte =
        (i_Mask == 4'b0001) ? i_mem_read_data[7:0]   :
        (i_Mask == 4'b0010) ? i_mem_read_data[15:8]  :
        (i_Mask == 4'b0100) ? i_mem_read_data[23:16] :
                              i_mem_read_data[31:24]; // 4'b1000 default

    wire [15:0] load_half =
        (i_Mask == 4'b0011) ? i_mem_read_data[15:0]  :
                              i_mem_read_data[31:16]; // 4'b1100 default

    wire [31:0] load_result =
        (i_funct3 == 3'b000) ? {{24{load_byte[7]}},  load_byte} : // lb
        (i_funct3 == 3'b100) ? {24'b0,               load_byte} : // lbu
        (i_funct3 == 3'b001) ? {{16{load_half[15]}}, load_half} : // lh
        (i_funct3 == 3'b101) ? {16'b0,               load_half} : // lhu
                               i_mem_read_data;                  // lw
    assign o_misaligned = i_misaligned; // pass through for misaligned load/store detection
    assign o_inst = i_inst; // pass through for hazard detection
    assign o_pc = i_pc;
    assign o_rs1_data = i_rs1_data;
    assign o_rs2_data = i_rs2_data;
    
    // assign o_target_addr = i_Jalr ? i_jalr_target_addr : i_branch_jal_target_addr;
    assign o_writeback_data = (i_WriteRegSrc == 2'b10) ? i_pc_4 : 
                              (i_WriteRegSrc == 2'b01) ? load_result : i_alu_result;

endmodule
`default_nettype wire