`default_nettype none

// branchjal_control.v

module branchjal_control (
    input  wire        branch,       // 1 when opcode == 110 0011
    input  wire        jal,          // 1 when opcode == 110 1111
    input  wire [2:0]  BranchType,   // funct3 for branch
    input  wire        alu_eq,       // o_eq from ALU: (rs1 == rs2)
    input  wire        alu_slt,      // o_slt from ALU: (rs1 < rs2), signed/unsigned decided by ALU i_unsigned
    output wire        branchjalsel  // 1 selects PC + (imm<<1)
);

    wire branch_taken;
    assign branch_taken =
        (BranchType == 3'b000) ? ( alu_eq)  : // BEQ
        (BranchType == 3'b001) ? (~alu_eq)  : // BNE
        (BranchType == 3'b100) ? ( alu_slt) : // BLT
        (BranchType == 3'b101) ? (~alu_slt) : // BGE
        (BranchType == 3'b110) ? ( alu_slt) : // BLTU (ALU must be unsigned compare)
        (BranchType == 3'b111) ? (~alu_slt) : // BGEU
                                1'b0;

    // Final select: choose PC+imm when JAL, or when a BRANCH is taken.
    assign branchjalsel = jal | (branch & branch_taken);

endmodule

`default_nettype wire