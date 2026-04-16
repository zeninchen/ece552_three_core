`default_nettype none

module execute (
    input  wire [31:0] i_pc,
    input  wire [31:0] i_pc_4,
    input  wire [2:0]  i_funct3,      // decode: inst[14:12]
    input  wire        i_is_sub,      // decode: inst[30]

    input  wire [31:0] i_rs1_data,
    input  wire [31:0] i_rs2_data,
    input  wire [31:0] i_immediate,

    input  wire        i_Jalr,
    input  wire        i_Jal,
    input  wire        i_Branch,
    input  wire [2:0]  i_BranchType,

    input  wire [1:0]  i_ALUOp,
    input  wire [1:0]  i_ALUSrcA,
    input  wire        i_ALUSrcB,

    input wire        i_MemRead,
    input wire [1:0]  i_WriteRegSrc,
    input wire        i_MemWrite,

    input wire        [3:0] i_Mask, // for store instruction byte/halfword/word selection
    // input wire [31:0] i_wb_data,
    input wire [31:0] i_inst, // for jalr target calculation and misaligned detection

    output wire [31:0] o_inst, // pass through for hazard detection
    output wire        o_MemRead,
    output wire [1:0]  o_WriteRegSrc,
    output wire        o_MemWrite,

    output wire [3:0]  o_Mask,

    output wire [31:0] o_rs1_data,
    output wire [31:0] o_rs2_data,

    output wire [31:0] o_pc,
    output wire [31:0] o_pc_4,  
    output wire [2:0]  o_funct3, // for memory access byte/halfword/word selection
    output wire [31:0] o_alu_result,
    output wire [1:0]  addr_l2sb,          // for memory access byte/halfword selection (00: word, 01: halfword, 10: byte)
    
    // output wire [31:0] branch_jal_target_addr,   // PC + imm
    // output wire [31:0] jalr_target_addr,  // rs1 + imm (with bit0=0)
    // output wire        o_jalr,  // take jalr sel
    output wire [31:0] o_next_pc_resolve, // for next PC calculation in hart (branch/jal target or jalr target)
    output wire [31:0] o_store_data,       // rs2 forwarded to dmem write data path
    // output wire [31:0] o_wb_data,
    output wire o_misaligned,
    //not select for pc+4 in next PC calculation in hart (for non-control-transfer instructions)
    output wire not_pc_4,

    //forwading inputs
    //x to x forwarding
    input wire [31:0] i_ex_mem_alu_result,
    input wire i_ex_to_ex_forward_rs1_en,
    input wire i_ex_to_ex_forward_rs2_en,

    //mem to ex forwarding
    input wire [31:0] i_wb_wdata,
    input wire i_mem_to_ex_forward_rs1_en,
    input wire i_mem_to_ex_forward_rs2_en
);
    wire [31:0] jalr_target_addr;
    wire [31:0] branch_jal_target_addr;
    wire [31:0] rs1_data_forwarded, rs2_data_forwarded;
    assign rs1_data_forwarded = i_ex_to_ex_forward_rs1_en ? i_ex_mem_alu_result :
                              i_mem_to_ex_forward_rs1_en ? i_wb_wdata :
                              i_rs1_data;
    assign rs2_data_forwarded = i_ex_to_ex_forward_rs2_en ? i_ex_mem_alu_result :
                              i_mem_to_ex_forward_rs2_en ? i_wb_wdata :
                              i_rs2_data;
    assign o_rs1_data = rs1_data_forwarded;
    assign o_rs2_data = rs2_data_forwarded;

    assign o_inst = i_inst; // pass through for hazard detection
    // assign o_jalr = i_Jalr; // jalr (pipeline-friendly)
    assign o_pc = i_pc; // for jalr target calculation
    assign o_pc_4 = i_pc_4; // for jal/jalr
    assign o_WriteRegSrc = i_WriteRegSrc; // for writeback mux control
    assign o_MemRead = i_MemRead; // pass through for memory read control signal
    assign o_MemWrite = i_MemWrite; // pass through for memory write control signal
    // Store data pass-through (pipeline-friendly)
    // assign o_wb_data = i_wb_data; // pass through for writeback data
    assign o_funct3 = i_funct3; // pass through for memory access byte/halfword/word selection

    // ALU input muxes
    // ALUSrcA: 0 -> PC, 1 -> rs1, 3 -> 0
    // ALUSrcB: 0 -> rs2, 1 -> immediate
    wire [31:0] alu_op1 = (i_ALUSrcA == 2'b00) ? i_pc : (i_ALUSrcA == 2'b01) ? rs1_data_forwarded : 32'd0;
    wire [31:0] alu_op2 = (i_ALUSrcB == 1'b0) ? rs2_data_forwarded : i_immediate;


    // ALU control
    wire       alu_arith;
    wire       alu_sub;
    wire       alu_unsigned;
    wire [2:0] alu_opsel;

    alucontrol u_alucontrol (
        .i_funct3   (i_funct3),
        .ALUOp      (i_ALUOp),
        .i_is_sub   (i_is_sub),
        .o_arith    (alu_arith),
        .o_sub      (alu_sub),
        .o_unsigned (alu_unsigned),
        .o_opsel    (alu_opsel)
    );

    // ALU
    wire alu_eq;
    wire alu_slt;
    wire [31:0] alu_result_raw;
    alu u_alu (
        .i_opsel    (alu_opsel),
        .i_sub      (alu_sub),
        .i_unsigned (alu_unsigned),
        .i_arith    (alu_arith),
        .i_op1      (alu_op1),
        .i_op2      (alu_op2),
        .o_result   (alu_result_raw),
        .o_eq       (alu_eq),
        .o_slt      (alu_slt)
    );
    
    // jalr target calculation (rs1 + imm, with bit0=0)
    assign jalr_target_addr = {alu_result_raw[31:1], 1'b0};
    
    // Branch/JAL decision
    wire branchjalsel;
    branchjal_control u_bjc (
        .branch       (i_Branch),
        .jal          (i_Jal),
        .BranchType   (i_BranchType),
        .alu_eq       (alu_eq),
        .alu_slt      (alu_slt),
        .branchjalsel (branchjalsel)
    );
    ///////////////////////////////////////////
    //removed this calculation, because the pc +4 calulated from here is too late
    //assign branch_jal_target_addr = branchjalsel? i_pc + i_immediate : i_pc_4;
    //instead output branch_jal_target_addr for next PC calculation in hart, 
    //and let the hart select the target based on branchjalsel signal (for better timing)
    //out put the selector for pc+4
    ////////////////////////////////////////////
    // I want to do branch resolve in id, but the only thing that is doable for that is 
    //bne, beq, and jal, all the other require alu result for branch decision, so we will do branch resolve in ex stage for now, 
    //and we can optimize it later by doing branch resolve in id stage for bne/beq/jal, and keep the other branch instructions resolved in ex stage (because we need alu result for branch decision)
    assign not_pc_4 = branchjalsel | i_Jalr; // for non-control-transfer instructions, next PC is pc+4; for branch/jal, next PC is target addr; for jalr, next PC is jalr target addr (calculated from alu result)
    assign branch_jal_target_addr = i_pc + i_immediate; // for branch/jal target calculation (for next PC calculation in hart)   
    assign o_next_pc_resolve = (i_Jalr) ? jalr_target_addr : branch_jal_target_addr;
        
    // prepare for d-mem
    wire        is_dmem_access;
    wire [31:0] dmem_addr_aligned;

    assign addr_l2sb = alu_result_raw[1:0]; // for byte/halfword selection
    
    //addr for dmem
    assign dmem_addr_aligned = {alu_result_raw[31:2], 2'b00};
    assign is_dmem_access = i_MemRead | i_MemWrite;
    assign o_alu_result = is_dmem_access ? dmem_addr_aligned : alu_result_raw; // for dmem access, use aligned address for word addressing; for other instructions, pass through the original alu result (e.g., for jalr target addr calculation)
   
    // i_Mask << addr_l2sb
    // D-MEM mask lane selection
    // i_Mask is the "base" mask (sb=0001, sh=0011, sw=1111), shifted into lane by addr_l2sb
    wire [3:0] mask_shifted =
        (addr_l2sb == 2'd0) ? i_Mask :
        (addr_l2sb == 2'd1) ? {i_Mask[2:0], 1'b0} :
        (addr_l2sb == 2'd2) ? {i_Mask[1:0], 2'b00} :
                            {i_Mask[0],   3'b000}; // addr_l2sb==3

    assign o_Mask = is_dmem_access ? (mask_shifted) : 4'b0000; // shift mask according to address offset for store instruction

    // For store instructions, shift the data to the correct byte lane according to addr_l2sb (for byte/halfword store)
    wire [31:0] sb_wdata =
        (addr_l2sb == 2'd0) ? {24'b0, rs2_data_forwarded[7:0]} :
        (addr_l2sb == 2'd1) ? {16'b0, rs2_data_forwarded[7:0], 8'b0} :
        (addr_l2sb == 2'd2) ? {8'b0,  rs2_data_forwarded[7:0], 16'b0} :
                              {       rs2_data_forwarded[7:0], 24'b0};

    wire [31:0] sh_wdata =
        (addr_l2sb == 2'd0) ? rs2_data_forwarded :
        (addr_l2sb == 2'd2) ? {rs2_data_forwarded[15:0], 16'b0} :
                            32'b0;
    wire [31:0] sw_wdata = rs2_data_forwarded;

    assign o_store_data =
        (!i_MemWrite)       ? i_rs2_data :
        (i_funct3 == 3'b000) ? sb_wdata   :
        (i_funct3 == 3'b001) ? sh_wdata   :
                            sw_wdata;

    // ----------------------------
    // Misaligned detection
    // lw/sw require addr[1:0]==00; lh/lhu/sh require addr[0]==0; byte accesses always ok
    // ----------------------------
    wire is_word = (i_funct3 == 3'b010);                 // lw/sw
    wire is_half = (i_funct3 == 3'b001) | (i_funct3 == 3'b101); // lh/lhu/sh (001/101)
    
    wire dmem_misaligned;
    assign dmem_misaligned = is_dmem_access &
        ( (is_word & (addr_l2sb != 2'b00)) |
          (is_half &  addr_l2sb[0]) );

    // branch/jalr target misaligned (for trap handling in hart)
    wire ctrl_taken;
    assign ctrl_taken = i_Jalr | branchjalsel;

    wire [31:0] ctrl_target =
        i_Jalr ? jalr_target_addr : branch_jal_target_addr;

    wire ctrl_misaligned;
    assign ctrl_misaligned = ctrl_taken & (ctrl_target[1:0] != 2'b00);

    assign o_misaligned = dmem_misaligned | ctrl_misaligned; // for misaligned trap handling in hart
endmodule

`default_nettype wire