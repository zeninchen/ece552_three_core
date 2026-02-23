module control(
    input  wire [31:0] i_inst,
    output wire o_rd_wen,
    output wire [2:0] o_opsel,
    output wire o_sub,
    output wire o_unsigned,
    output wire o_arith,
    output wire o_mem_wen,
    output wire o_men_to_reg,
    output wire o_alu_src_2,
    output wire [5:0] o_format
    // [0] R-type
    // [1] I-type
    // [2] S-type
    // [3] B-type
    // [4] U-type
    // [5] J-type
    output wire u_format_load0,
    output wire alu_src1,
    output wire [1:0] sbhw_sel,
    //determine whether the store instruction is a byte, halfword, or word store
    output wire [1:0] lbhw_sel,
    //determine whether the load instruction is a byte, halfword, or word load
    output wire l_unsigned,
    //determine whether the load instruction is a signed or unsigned load(we sign extend or zero extend)
    output wire is_jump,
    output wire is_branch,
    output wire is_jal,
    output wire is_jalr
);
    //format logic 
    //R-format opcode 011 0011
    //I-format opcode 001 0011 or Load: 000 0011 jalr: 110 0111
    //S-format opcode 010 0011
    //B-format opcode 110 0011
    //U-format opcode 011 0111 or 001 0111
    //J-format opcode 110 1111
    always @(*) begin
        case (i_inst[6:0])
            7'b0110011: o_format = 6'b000001; // R-type
            7'b0010011, 7'b0000011, 7'b1100111: o_format = 6'b000010; // I-type
            7'b0100011: o_format = 6'b000100; // S-type
            7'b1100011: o_format = 6'b001000; // B-type
            7'b0110111, 7'b0010111: o_format = 6'b010000; // U-type
            7'b1101111: o_format = 6'b100000; // J-type
            default: o_format = 6'b000000; // default case
        endcase
    end
    //alu sr1 logci 

    //alu src2 logic
    //R-type and B-type instructions use register source 2, while I-type, S-type, U-type, and J-type instructions use the immediate value as source 2
    assign o_alu_src_2 = !(o_format[0] || o_format[3]);

    //reg write enable logic
    //only B and S type instructions do not write to a register, all other instruction types write to a register
    assign o_rd_wen = !(o_format[2] || o_format[3]);

    //mem write enable logic
    //only S-type instructions write to memory, all other instruction types do not write to memory
    assign o_mem_wen = o_format[2];

    //mem to reg logic
    //only load instructions (I-type with opcode 000 0011) write the memory data to a register, all other instruction types do not write memory data to a register
    assign o_men_to_reg = (i_inst[6:0] == 7'b0000011);

    //alu opsel logic
    //for R-type and I-type instructions, the opsel is determined by the funct3 field (bits 14:12)
    //for B-type instructions, the opsel is substraction, and the specific comparison will be based on the o_eq and o_slt from the ALU
    //for other instruction types, it's addition
    //o_sub and o_arithmetic are determined by funct7, which is bit[30] for both
    always @(*) begin
        if (o_format[0] || o_format[1]) begin // R-type or I-type
            o_opsel = i_inst[14:12];
            o_sub = i_inst[30];
            o_arith = i_inst[30];
        end
        else if (o_format[3]) begin // B-type
            o_opsel = 3'b000;
            o_sub = 1'b1; // subtraction for branch comparison
            o_arith = 1'b0; // not arithmetic shift for branch comparison
        end
        else begin // other instruction types
            o_opsel = 3'b000; // ADD for all other instruction types
            o_sub = 1'b0; // not subtraction for other instruction types
            o_arith = 1'b0; // not arithmetic shift for other instruction types
        end
    end

    //sbhw_sel logic
    //it's bascially funct3[1:0] for store instructions
    assign sbhw_sel = i_inst[13:12];

    //lbhw_sel logic
    //it's bascially funct3[1:0] for load instructions
    assign lbhw_sel = i_inst[13:12];

    //l_unsigned logic
    //it's bascially the sign bit of funct3 for load instructions
    assign l_unsigned = i_inst[14];

    //u_format_load0 logic & alu_src1 logic
    //for U-type instructions, we only want to load the immediate value into the register and not read from memory, so we can set u_format_load0 to 1 when it's a U-type instruction
    assign alu_src1 = o_format[4]; // for U-type instructions

    //we load 0 to sr1 when o_format is U and opcode[5] is 1
    assign u_format_load0 = o_format[4] && i_inst[5];

    //is_jal logic
    //when it's a j-tpye instruction (the only j-type instruction is jal), we set is_jal to 1
    assign is_jal = o_format[5];
    //is_jalr logic
    //when it's an I-type instruction with opcode 110 0111 (the only I-type instruction that is jalr), we set is_jalr to 1
    assign is_jalr = i_inst[6:0] == 7'b1100111;

    assign is_jump = is_jal || is_jalr;
endmodule