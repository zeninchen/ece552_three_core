module control(
    input  wire [31:0] i_inst,
    output wire o_rd_wen,
    output wire [2:0] o_opsel,
    output wire o_sub,
    output wire o_unsigned,
    output wire o_arith,
    output wire o_mem_wen,
    output wire o_men_to_reg,
    output wire o_alu_src_immediate,
    output wire [5:0] o_format
    // [0] R-type
    // [1] I-type
    // [2] S-type
    // [3] B-type
    // [4] U-type
    // [5] J-type
    output wire u_format_load0,
    output wire alu_src1,
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


endmodule