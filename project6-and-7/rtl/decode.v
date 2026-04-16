module decode (
    input  wire        i_clk,
    input  wire        i_rst,
	input  wire [31:0] i_pc,        //pc
	input  wire [31:0] i_pc_4,      //pc+4
    input  wire [31:0] i_inst,      // Instruction
    input  wire        i_wb_rd_wen,
    input  wire [4:0]  i_wb_rd_waddr,
    input  wire [31:0] i_wb_rd_wdata,
    
    // --- Output Signal ---
    output wire [31:0] o_inst,      // for forwarding and hazard detection
	output wire [31:0] o_pc,
	output wire [31:0] o_pc_4,
    output wire [31:0] o_rs1_data,
    output wire [31:0] o_rs2_data,
	output wire        o_inst_30,
	output wire [2:0]  o_inst_14_12,        //funct3
    output wire [31:0] o_immediate,
	output wire        o_Jalr,
	output wire        o_Jal,
    output wire        o_Branch,
	output wire [2:0]  o_BranchType,
    output wire        o_MemRead,
	output wire [1:0]  o_WriteRegSrc,
	output wire [1:0]  o_ALUOp,
    output wire        o_MemWrite,
	output wire [1:0]  o_ALUSrcA,
    output wire        o_ALUSrcB,
    output wire        o_RegWrite,
    output wire [3:0]  o_Mask
	
);
	//pass pc 
	assign o_pc = i_pc;
	assign o_pc_4 = i_pc_4;
    assign o_inst = i_inst; // for forwarding and hazard detection, also for branch target calculation in execute stage
	
	//pass ALU Control
	assign o_inst_30 = i_inst[30];
	assign o_inst_14_12 = i_inst[14:12];
	
    // --- Internal Signal ---
    wire        w_RegWrite;
	
    //Control Unit 
    Control Control_unit (
        .funct7      (i_inst[31:25]),
        .opcode      (i_inst[6:0]),
        .funct3      (i_inst[14:12]),
        .Jalr        (o_Jalr),
        .Jal         (o_Jal),
        .Branch      (o_Branch),
        .BranchType  (o_BranchType),
        .MemRead     (o_MemRead),
        .WriteRegSrc (o_WriteRegSrc),
        .ALUOp       (o_ALUOp),
        .MemWrite    (o_MemWrite),
        .ALUSrcA     (o_ALUSrcA),
        .ALUSrcB     (o_ALUSrcB),
        .RegWrite    (w_RegWrite),
        .Mask        (o_Mask)
    );
	assign o_RegWrite = w_RegWrite;
	// --- Internal Signal ---
	wire [5:0] w_imm_format;
    wire [6:0] w_opcode = i_inst[6:0];
    
    assign w_imm_format[0] = (w_opcode == 7'b0110011); // R-type
    assign w_imm_format[1] = (w_opcode == 7'b0010011) || // I-type (Imm)
                             (w_opcode == 7'b0000011) || // I-type (Load)
                             (w_opcode == 7'b1100111);   // I-type (Jalr)
    assign w_imm_format[2] = (w_opcode == 7'b0100011); // S-type
    assign w_imm_format[3] = (w_opcode == 7'b1100011); // B-type
    assign w_imm_format[4] = (w_opcode == 7'b0110111) || // U-type (LUI)
                             (w_opcode == 7'b0010111);   // U-type (AUIPC)
    assign w_imm_format[5] = (w_opcode == 7'b1101111); // J-type
	
    //Immediate Generator 
    imm Imm_gen (
        .i_inst      (i_inst),
        .i_format    (w_imm_format), 
        .o_immediate (o_immediate)
    );

    wire [4:0] rs1_addr = i_inst[19:15];
    wire [4:0] rs2_addr = i_inst[24:20];

    // Register File 
    rf #(
        .BYPASS_EN(1)
    ) Register_file (
        .i_clk       (i_clk),
        .i_rst       (i_rst),
        .i_rs1_raddr (rs1_addr), // rs1
        .o_rs1_rdata (o_rs1_data),
        .i_rs2_raddr (rs2_addr), // rs2
        .o_rs2_rdata (o_rs2_data),
        .i_rd_wen    (i_wb_rd_wen),    // from Control
        .i_rd_waddr  (i_wb_rd_waddr),  // rd
        .i_rd_wdata  (i_wb_rd_wdata)     // from MUX
    );

endmodule