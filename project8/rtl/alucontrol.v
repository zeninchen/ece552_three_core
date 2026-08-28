`default_nettype none
module alucontrol (
    input  wire [2:0] i_funct3,     //instruction[14:12]
    input  wire [1:0] ALUOp,
    input  wire       i_is_sub,     //instruction[30]
    output wire       o_arith,      //for shift right logical vs arithmetic
    output wire       o_sub,        //for add vs sub
    output wire       o_unsigned, //for slt vs sltu
    output wire [2:0] o_opsel
);
    // ALUOp codes:
    // 00 => load/store(I/S), U, J (ALU does addition)
    // 01 => branch (ALU does subtraction for comparison)
    // 10 => R-type, I-type

    //o_arith: shift right logical/arithmetic for ALU if `i_arith` asserted
    assign o_arith = (((ALUOp == 2'b10)|(ALUOp == 2'b11)) & (i_funct3 == 3'b101) & i_is_sub) ? 1'b1 : 1'b0;
    //o_sub: addition or subtraction for ALU (branch always subtract; R-type SUB when funct3=000 and bit30=1)
    assign o_sub = (ALUOp == 2'b01) ? 1'b1 :
        ((ALUOp == 2'b10) & (i_funct3 == 3'b000) & i_is_sub) ? 1'b1 :
        1'b0;
    //o_unsigned: whether the operation is unsigned (for sltu, sltiu(funct3=011)/ bltu, bgeu(branch funct3=110/111))
    assign o_unsigned = ((ALUOp == 2'b01) & ((i_funct3 == 3'b110) | (i_funct3 == 3'b111))) ? 1'b1 :
        (((ALUOp == 2'b10)|(ALUOp == 2'b11)) & (i_funct3 == 3'b011)) ? 1'b1 :
        1'b0;

    // o_opsel
    assign o_opsel =
        ((ALUOp == 2'b00) | (ALUOp == 2'b01)) ? 3'b000 : // ADD/SUB class
        ((ALUOp == 2'b10)|(ALUOp == 2'b11)) ? (
            (i_funct3 == 3'b000) ? 3'b000 : // ADD/SUB/ADDI
            (i_funct3 == 3'b001) ? 3'b001 : // SLL/SLLI
            (i_funct3 == 3'b010) ? 3'b010 : // SLT/SLTI
            (i_funct3 == 3'b011) ? 3'b010 : // SLTU/SLTIU (unsigned flag to identify)
            (i_funct3 == 3'b100) ? 3'b100 : // XOR/XORI
            (i_funct3 == 3'b101) ? 3'b101 : // SRL/SRA/SRLI/SRAI
            (i_funct3 == 3'b110) ? 3'b110 : // OR/ORI
            (i_funct3 == 3'b111) ? 3'b111 : // AND/ANDI
                                   3'b000
        ) :
        3'b000;
    
endmodule