module Control (
	input  [6:0] funct7,        // Instruction [31:25]
    input  [6:0] opcode,        // Instruction [6:0]
    input  [2:0] funct3,        // Instruction [14:12]
    output reg   Jalr,
    output reg   Jal,
    output reg Branch,
    output reg [2:0] BranchType, 
    output reg   MemRead,
    output reg [1:0] WriteRegSrc, 
    output reg [1:0] ALUOp,
    output reg   MemWrite,
    output reg [1:0] ALUSrcA,
    output reg   ALUSrcB,
    output reg   RegWrite,
	output reg   [3:0]Mask
);

    // RISC-V Opcode 
    localparam OP_R_TYPE   = 7'b0110011;
    localparam OP_IMM      = 7'b0010011;
	localparam OP_LUI      = 7'b0110111;
    localparam OP_AUIPC    = 7'b0010111;
    localparam OP_LOAD     = 7'b0000011;
    localparam OP_S_TYPE   = 7'b0100011;
    localparam OP_B_TYPE   = 7'b1100011;
    localparam OP_J_TYPE   = 7'b1101111;
    localparam OP_JALR     = 7'b1100111;
    

    always @(*) begin
		Jalr        = 1'd0;
		Jal         = 1'd0;
        Branch      = 1'd0;
		BranchType  = 3'd0;
		MemRead     = 1'd0;
		WriteRegSrc = 2'd0;
		ALUOp       = 2'd0;
		MemWrite    = 1'd0;
		ALUSrcA     = 2'b01;
		ALUSrcB     = 1'd0;
		RegWrite    = 1'd0;
		Mask        = 4'b1111;
		
        case (opcode)
            OP_R_TYPE: begin
				WriteRegSrc = 2'b00; 
				ALUOp       = 2'b10;
                ALUSrcB     = 1'b0; 
				RegWrite    = 1'b1;       
            end

            //
            OP_IMM: begin
                WriteRegSrc = 2'b00;
				ALUOp       = 2'b11;    //change to 11 I-type
                ALUSrcB     = 1'b1; 
				RegWrite    = 1'b1;
            end
			
			OP_LUI: begin //r[rd] = {imm, 12'b0}
                WriteRegSrc = 2'b00; //Add 0, imm????
				ALUSrcA     = 2'b10; // LUI should use 0 as ALU input
                ALUSrcB     = 1'b1; 
				ALUOp       = 2'b00; //should be 00 for load/store, U-type, J-type 
				RegWrite    = 1'b1;
            end
			
			OP_AUIPC: begin //r[rd] = pc + {imm, 12'b0}
                WriteRegSrc = 2'b00;
				ALUSrcA     = 2'b00;
                ALUSrcB     = 1'b1; 
				ALUOp       = 2'b00; //should be 00 for load/store, U-type, J-type
				RegWrite    = 1'b1;
            end
			
            OP_LOAD: begin
                MemRead     = 1'b1;
				WriteRegSrc = 2'b01;
				ALUOp       = 2'b00; 
                ALUSrcA     = 2'b01;
                ALUSrcB     = 1'b1; 
				RegWrite    = 1'b1;
				//Mask 
                case (funct3)
                    3'b000: Mask = 4'b0001; // LB (Byte)
                    3'b001: Mask = 4'b0011; // LH (Half-word)
                    3'b010: Mask = 4'b1111; // LW (Word)
                    3'b100: Mask = 4'b0001; // LBU
                    3'b101: Mask = 4'b0011; // LHU
                    default: Mask = 4'b1111;
                endcase
            end

            OP_S_TYPE: begin
				ALUOp       = 2'b00; 
                MemWrite    = 1'b1;
                ALUSrcA     = 2'b01;
                ALUSrcB     = 1'b1;
				//Mask 
				case (funct3)
                    3'b000: Mask = 4'b0001; // SB (Byte)
                    3'b001: Mask = 4'b0011; // SH (Half-word)
                    3'b010: Mask = 4'b1111; // SW (Word)
                    default: Mask = 4'b1111;
                endcase
            end

            OP_B_TYPE: begin
                Branch      = 1'b1;
                BranchType  = funct3; 
				ALUOp       = 2'b01; 
                ALUSrcA     = 2'b01;
                ALUSrcB     = 1'b0;
            end

            OP_J_TYPE: begin
                Jal         = 1'b1;
				WriteRegSrc = 2'b10; 
				ALUOp       = 2'b00;    //should be 00
                ALUSrcA     = 2'b00;     // jal should use PC as ALU input for target address calculation
                ALUSrcB     = 1'd1;     // jal should use immediate as ALU input for target address calculation
                RegWrite    = 1'b1;
            end

            OP_JALR: begin //target = r[rs1] + signext(offset)
                Jalr        = 1'b1;
				WriteRegSrc = 2'b10;
                ALUOp       = 2'b00;    //Addition
                ALUSrcA     = 2'b01;   
                ALUSrcB     = 1'b1;   
				RegWrite    = 1'b1;
            end

            default: begin
                Jalr        = 1'd0;
				Jal         = 1'd0;
                Branch      = 1'd0;
				BranchType  = 3'd0;
				MemRead     = 1'd0;
				WriteRegSrc = 2'd0;
				ALUOp       = 2'd0;
				MemWrite    = 1'd0;
				ALUSrcA     = 2'b01;
				ALUSrcB     = 1'd0;
				RegWrite    = 1'd0;
				Mask        = 4'b1111;
            end
        endcase
    end

endmodule