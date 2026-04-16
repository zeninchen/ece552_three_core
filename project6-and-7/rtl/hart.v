`default_nettype none

module hart #(
    // After reset, the program counter (PC) should be initialized to this
    // address and start executing instructions from there.
    parameter RESET_ADDR = 32'h00000000
) (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // Instruction fetch goes through a read only instruction memory (imem)
    // port. The port accepts a 32-bit address (e.g. from the program counter)
    // per cycle and sequentially returns a 32-bit instruction word. For
    // projects 6 and 7, this memory has been updated to be more realistic
    // - reads are no longer combinational, and both read and write accesses
    // take multiple cycles to complete.
    //
    // The testbench memory models a fixed, multi cycle memory with partial
    // pipelining. The memory will accept a new request every N cycles by
    // asserting `mem_ready`, and if a request is made, the memory perform
    // the request (read or write) after M cycles, asserting mem_valid to
    // indicate the read data is ready (or the write is complete). Requests
    // are completed in order. The values of N and M are deterministic, but
    // may change between test cases - you must design your CPU to work
    // correctly by looking at `mem_ready` and `mem_valid` rather than
    // hardcoding a latency assumption.
    //
    // Indicates that the memory is ready to accept a new read request.
    input  wire        i_imem_ready,
    // 32-bit read address for the instruction memory. This is expected to be
    // 4 byte aligned - that is, the two LSBs should be zero.
    output wire [31:0] o_imem_raddr,
    // Issue a read request to the memory on this cycle. This should not be
    // asserted if `i_imem_ready` is not asserted.
    output wire        o_imem_ren,
    // Indicates that a valid instruction word is being returned from memory.
    input  wire        i_imem_valid,
    // Instruction word fetched from memory, available sequentially some
    // M cycles after a request (imem_ren) is issued.
    input  wire [31:0] i_imem_rdata,

    // Data memory accesses go through a separate read/write data memory (dmem)
    // that is shared between read (load) and write (stored). The port accepts
    // a 32-bit address, read or write enable, and mask (explained below) each
    // cycle.
    //
    // The timing of the dmem interface is the same as the imem interface. See
    // the documentation above.
    //
    // Indicates that the memory is ready to accept a new read or write request.
    input  wire        i_dmem_ready,
    // Read/write address for the data memory. This should be 32-bit aligned
    // (i.e. the two LSB should be zero). See `o_dmem_mask` for how to perform
    // half-word and byte accesses at unaligned addresses.
    output wire [31:0] o_dmem_addr,
    // When asserted, the memory will perform a read at the aligned address
    // specified by `i_addr` and return the 32-bit word at that address
    // immediately (i.e. combinationally). It is illegal to assert this and
    // `o_dmem_wen` on the same cycle.
    output wire        o_dmem_ren,
    // When asserted, the memory will perform a write to the aligned address
    // `o_dmem_addr`. When asserted, the memory will write the bytes in
    // `o_dmem_wdata` (specified by the mask) to memory at the specified
    // address. It is illegal to assert this and `o_dmem_ren` on the same
    // cycle.
    output wire        o_dmem_wen,
    // The 32-bit word to write to memory when `o_dmem_wen` is asserted. When
    // write enable is asserted, the byte lanes specified by the mask will be
    // written to the memory word at the aligned address at the next rising
    // clock edge. The other byte lanes of the word will be unaffected.
    output wire [31:0] o_dmem_wdata,
    // The dmem interface expects word (32 bit) aligned addresses. However,
    // the processor supports byte and half-word loads and stores at unaligned
    // and 16-bit aligned addresses, respectively. To support this, the access
    // mask specifies which bytes within the 32-bit word are actually read
    // from or written to memory.
    //
    // To perform a half-word read at address 0x00001002, align `o_dmem_addr`
    // to 0x00001000, assert `o_dmem_ren`, and set the mask to 0b1100 to
    // indicate that only the upper two bytes should be read. Only the upper
    // two bytes of `i_dmem_rdata` can be assumed to have valid data; to
    // calculate the final value of the `lh[u]` instruction, shift the rdata
    // word right by 16 bits and sign/zero extend as appropriate.
    //
    // To perform a byte write at address 0x00002003, align `o_dmem_addr` to
    // `0x00002000`, assert `o_dmem_wen`, and set the mask to 0b1000 to
    // indicate that only the upper byte should be written. On the next clock
    // cycle, the upper byte of `o_dmem_wdata` will be written to memory, with
    // the other three bytes of the aligned word unaffected. Remember to shift
    // the value of the `sb` instruction left by 24 bits to place it in the
    // appropriate byte lane.
    output wire [ 3:0] o_dmem_mask,
    // Indicates that a valid data word is being returned from memory.
    input  wire        i_dmem_valid,
    // The 32-bit word read from data memory. When `o_dmem_ren` is asserted,
    // this will immediately reflect the contents of memory at the specified
    // address, for the bytes enabled by the mask. When read enable is not
    // asserted, or for bytes not set in the mask, the value is undefined.
    input  wire [31:0] i_dmem_rdata,
    // The output `retire` interface is used to signal to the testbench that
    // the CPU has completed and retired an instruction. A single cycle
    // implementation will assert this every cycle; however, a pipelined
    // implementation that needs to stall (due to internal hazards or waiting
    // on memory accesses) will not assert the signal on cycles where the
    // instruction in the writeback stage is not retiring.
    //
    // Asserted when an instruction is being retired this cycle. If this is
    // not asserted, the other retire signals are ignored and may be left invalid.
    output wire        o_retire_valid,
    // The 32 bit instruction word of the instrution being retired. This
    // should be the unmodified instruction word fetched from instruction
    // memory.
    output wire [31:0] o_retire_inst,
    // Asserted if the instruction produced a trap, due to an illegal
    // instruction, unaligned data memory access, or unaligned instruction
    // address on a taken branch or jump.
    output wire        o_retire_trap,
    // Asserted if the instruction is an `ebreak` instruction used to halt the
    // processor. This is used for debugging and testing purposes to end
    // a program.
    output wire        o_retire_halt,
    // The first register address read by the instruction being retired. If
    // the instruction does not read from a register (like `lui`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs1_raddr,
    // The second register address read by the instruction being retired. If
    // the instruction does not read from a second register (like `addi`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs2_raddr,
    // The first source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs1 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs1_rdata,
    // The second source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs2 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs2_rdata,
    // The destination register address written by the instruction being
    // retired. If the instruction does not write to a register (like `sw`),
    // this should be 5'd0.
    output wire [ 4:0] o_retire_rd_waddr,
    // The destination register data written to the register file in the
    // writeback stage by this instruction. If rd is 5'd0, this field is
    // ignored and can be treated as a don't care.
    output wire [31:0] o_retire_rd_wdata,
    output wire [31:0] o_retire_dmem_addr,
    output wire [ 3:0] o_retire_dmem_mask,
    output wire        o_retire_dmem_ren,
    output wire        o_retire_dmem_wen,
    output wire [31:0] o_retire_dmem_rdata,
    output wire [31:0] o_retire_dmem_wdata,
    // The current program counter of the instruction being retired - i.e.
    // the instruction memory address that the instruction was fetched from.
    output wire [31:0] o_retire_pc,
    // the next program counter after the instruction is retired. For most
    // instructions, this is `o_retire_pc + 4`, but must be the branch or jump
    // target for *taken* branches and jumps.
    output wire [31:0] o_retire_next_pc
);
    // YOUR CODE HERE
        // Fill in your implementation here.
	
	wire [31:0] if_pc, if_pc_4, if_instr;
    // wire [31:0] i_pc;        
	// wire [31:0] i_pc_4;     
    // wire [31:0] i_inst;

    wire [31:0] id_inst, ex_inst, mem_inst, wb_inst;
    wire [31:0] id_pc,   ex_pc,   mem_pc,   wb_pc;

	wire [31:0] id_pc_4;
    wire [31:0] id_rs1_data;
    wire [31:0] id_rs2_data;
	wire        id_inst_30;
	wire [2:0]  id_inst_14_12;
    wire [31:0] id_immediate;
    wire id_Jalr, id_Jal, id_Branch, id_MemRead, id_MemWrite, id_ALUSrcB;
    wire [1:0] id_ALUSrcA;
    wire [2:0] id_BranchType;
    wire [1:0] id_WriteRegSrc, id_ALUOp;
    wire id_RegWrite;
    wire [3:0] id_Mask;

    wire ex_RegWrite;
    wire        ex_MemRead;
    wire        ex_MemWrite;
    wire [1:0]  ex_WriteRegSrc;
	wire [3:0]  ex_Mask;

    wire [31:0] ex_pc_4;
    wire [2:0]  ex_funct3;
    wire [31:0] ex_alu_result;
    wire [1:0] ex_addr_l2sb;

    // wire [31:0] ex_branch_jal_target_addr;
    // wire [31:0] ex_jalr_target_addr;
    // wire        ex_Jalr;
    wire [31:0] ex_next_pc_target_addr;
    wire [31:0] ex_store_data;
    //wire [31:0] ex_wb_data;
    wire        ex_dmemmisaligned;
    wire        mem_misaligned, wb_misaligned;
    // wire        mem_Jalr;
    // wire [31:0] mem_branch_jal_target_addr;
    // wire [31:0] mem_jalr_target_addr;
    wire [31:0] mem_next_pc;
    wire        mem_MemRead;
    wire        mem_MemWrite;
    wire [1:0]  mem_WriteRegSrc;
	wire [3:0]  mem_Mask;
    wire [2:0]  mem_funct3;

    //wire mem_RegWrite;
	wire [31:0] mem_store_data;
    wire [31:0] mem_pc_4;
    wire [31:0] mem_mem_read_data;
    wire [31:0] mem_alu_result;
    wire [31:0] mem_wb_data;
    
    //wire wb_RegWrite;
    wire [2:0] wb_funct3;
    wire [31:0] wb_target_addr;     // next PC after this instruction (branch/jump decision)
    wire [31:0] wb_wdata;           // data written back to regfile
    wire [31:0] wb_next_pc;
    // add rf signal for retire interface check
    wire [31:0] ex_rs1_data;
    wire [31:0] ex_rs2_data;

    wire [31:0] mem_rs1_data;
    wire [31:0] mem_rs2_data;

    wire [31:0] wb_rs1_data;
    wire [31:0] wb_rs2_data;
    wire [6:0] op;
    wire retire_writes_rd;

    //data hazard detection signals
    wire nop_id, nop_ex, stall_decode;
    //new hazard detection signals 
    wire nop_mem, stall_execute;
    wire ex_not_pc_4; // for hazard detection to determine if the instruction in execute stage is a control transfer instruction or not (for better timing, we use the signal generated in execute stage, which is used for next PC calculation in hart, to determine if the instruction is control transfer or not)
	//p6 new hazard detection signals
    wire stall_fetch, stall_mem;
    wire nop_wb; // for the nop in mem stage when the mem stage is stalling, we need to insert nop in the writeback stage as well, because the instruction in mem stage is not finished yet, so we need to stall the writeback stage as well.
    wire memory_wait;
    wire fetch_wait;
    //forwarding signals
    wire ex_to_ex_forward_rs1_en, ex_to_ex_forward_rs2_en;
    wire mem_to_ex_forward_rs1_en, mem_to_ex_forward_rs2_en;
    wire mem_to_mem_forward_rs2_en;
    // phase 1: fetch
	fetch #(.RESET_ADDR(RESET_ADDR)) u_fetch (
        .i_clk         (i_clk),
        .i_rst         (i_rst),
        .i_target_addr (ex_next_pc_target_addr),
        .i_not_pc_4    (ex_not_pc_4), // for next PC calculation in hart (for non-control-transfer instructions)
        .i_stall        (stall_decode || stall_fetch || stall_mem), // for hazard detection to stall the fetch stage when there's a hazard

        .o_pc          (if_pc),
        .o_pc_4        (if_pc_4),

        .i_imem_ready   (i_imem_ready),
        .o_imem_ren     (o_imem_ren),
        .i_imem_valid   (i_imem_valid),
        .o_fetch_wait    (fetch_wait)
    );
	
    //connect fetch to imem
    assign o_imem_raddr = if_pc;
    assign if_instr = i_imem_rdata;

    // IF/ID pipeline register
    reg [31:0] if_id_pc;
    reg [31:0] if_id_pc_4;
    reg [31:0] if_id_inst;
    // Track the PC that was used when an imem request was accepted so returned
    // i_imem_rdata is tagged with the correct fetch PC on i_imem_valid.
    reg [31:0] if_req_pc;
    reg [31:0] if_req_pc_4;
    reg if_id_valid;
    always @(posedge i_clk) begin
        if (i_rst) begin
            if_req_pc   <= RESET_ADDR;
            if_req_pc_4 <= RESET_ADDR + 32'd4;
        end else if (o_imem_ren) begin
            if_req_pc   <= if_pc;
            if_req_pc_4 <= if_pc_4;
        end
    end
    wire decode_load = !stall_decode && !stall_mem && i_imem_valid;
    always @(posedge i_clk) begin
        if (i_rst||nop_id) begin
            if_id_pc   <= RESET_ADDR;
            if_id_pc_4 <= RESET_ADDR + 32'd4;
            if_id_inst <= 32'h00000013; // NOP = addi x0, x0, 0
            if_id_valid <= !nop_id; // if it's a nop, then it's not valid, if it's not a nop, then it's valid
            //if hazard detected stall the decode stage to hold the instruction, and insert the nop instruction into the execute stage
        end else if (decode_load) begin
            if_id_pc   <= if_req_pc;
            if_id_pc_4 <= if_req_pc_4;
            if_id_inst <= if_instr;
            if_id_valid <= 1'b1;
        end
        else begin
            if_id_valid <= 1'b0;
        end
    end

    reg mem_wb_RegWrite;
	// phase 2: decode
	decode u_decode (
        .i_clk(i_clk), 
		.i_rst(i_rst), 
		.i_pc(if_id_pc),
		.i_pc_4(if_id_pc_4),
        .i_inst(if_id_inst), 
        .i_wb_rd_wen  (mem_wb_RegWrite),
        .i_wb_rd_waddr(wb_inst[11:7]),
		.i_wb_rd_wdata(wb_wdata), //from write back stage, for RF bypassing

        .o_inst(id_inst),
        .o_pc(id_pc), 
		.o_pc_4(id_pc_4), 

		.o_rs1_data(id_rs1_data), 
		.o_rs2_data(id_rs2_data),
        .o_inst_30(id_inst_30), 
		.o_inst_14_12(id_inst_14_12), 
		.o_immediate(id_immediate),

        .o_Jalr(id_Jalr), 
		.o_Jal(id_Jal), 
		.o_Branch(id_Branch), 
		.o_BranchType(id_BranchType),

        .o_MemRead(id_MemRead), 
		.o_WriteRegSrc(id_WriteRegSrc), 
		.o_ALUOp(id_ALUOp),
        .o_MemWrite(id_MemWrite), 
		.o_ALUSrcA(id_ALUSrcA), 
		.o_ALUSrcB(id_ALUSrcB),
        .o_RegWrite(id_RegWrite),
        .o_Mask(id_Mask)
    );	
	
    // ID/EX pipeline registers
    reg [31:0] id_ex_pc;
    reg [31:0] id_ex_pc_4;
    reg [31:0] id_ex_inst;

    reg [31:0] id_ex_rs1_data;
    reg [31:0] id_ex_rs2_data;
    reg [31:0] id_ex_imm;

    reg        id_ex_inst_30;
    reg [2:0]  id_ex_inst_14_12;

    reg        id_ex_Jalr;
    reg        id_ex_Jal;
    reg        id_ex_Branch;
    reg [2:0]  id_ex_BranchType;

    reg        id_ex_MemRead;
    reg        id_ex_MemWrite;
    reg [1:0]  id_ex_WriteRegSrc;

    reg [1:0]  id_ex_ALUOp;
    reg [1:0]  id_ex_ALUSrcA;
    reg        id_ex_ALUSrcB;
    reg id_ex_RegWrite;
    reg [3:0]  id_ex_Mask;

    // EX/MEM pipeline registers
    reg [31:0] ex_mem_inst;

    reg        ex_mem_MemRead;
    reg [1:0]  ex_mem_WriteRegSrc;
    reg        ex_mem_MemWrite;
    reg [3:0]  ex_mem_Mask;
    reg ex_mem_RegWrite;
    reg [31:0] ex_mem_rs1_data;
    reg [31:0] ex_mem_rs2_data;
    reg [31:0] ex_mem_pc;
    reg [31:0] ex_mem_pc_4;
    reg [2:0]  ex_mem_funct3;
    reg [31:0] ex_mem_alu_result;
    reg [1:0]  ex_mem_addr_l2sb;
    reg [31:0] ex_mem_next_pc;
    reg [31:0] ex_mem_store_data;
    reg        ex_mem_misaligned;
    
    
    // MEM/WB pipeline registers
    reg        mem_wb_misaligned;
    reg [31:0] mem_wb_pc;
    reg [31:0] mem_wb_inst;
    reg [31:0] mem_wb_rs1_data;
    reg [31:0] mem_wb_rs2_data;

    reg [2:0]  mem_wb_funct3;
    reg [3:0]  mem_wb_Mask;
    reg [1:0]  mem_wb_WriteRegSrc;

    reg [31:0] mem_wb_pc_4;
    reg [31:0] mem_wb_next_pc;
    reg [31:0] mem_wb_alu_result;
    // dmem read data in MEM/WB, need pipeline reg to hold
    reg [31:0] mem_wb_mem_read_data;

    reg id_ex_valid;
    reg ex_mem_valid;
    reg mem_wb_valid;
    always @(posedge i_clk) begin
        //add nop_ex condition here for hazard, if nop_ex is true, we insert the nop instruction into the execute stage, and we stall the decode stage to hold the instruction
        if (i_rst||nop_ex) begin
            id_ex_valid <= !nop_ex; // if it's a nop, then it's not valid, if it's not a nop, then it's valid
            id_ex_pc         <= 32'd0;
            id_ex_pc_4       <= 32'd0;
            id_ex_inst       <= 32'h00000013; // NOP

            id_ex_rs1_data   <= 32'd0;
            id_ex_rs2_data   <= 32'd0;
            id_ex_imm        <= 32'd0;
            id_ex_inst_30    <= 1'b0;
            id_ex_inst_14_12 <= 3'd0;

            id_ex_Jalr       <= 1'b0;
            id_ex_Jal        <= 1'b0;
            id_ex_Branch     <= 1'b0;
            id_ex_BranchType <= 3'd0;

            id_ex_MemRead    <= 1'b0;
            id_ex_MemWrite   <= 1'b0;
            id_ex_WriteRegSrc<= 2'd0;

            id_ex_ALUOp      <= 2'd0;
            id_ex_ALUSrcA    <= 2'd0;
            id_ex_ALUSrcB    <= 1'b0;
            id_ex_RegWrite   <= 1'b0;
            id_ex_Mask       <= 4'd0;
            //if not stalled, 
        end else if(!stall_execute && !stall_mem)begin
            id_ex_valid <= if_id_valid; // if the instruction in the decode stage is valid, then the instruction in the execute stage is valid, otherwise it's not valid
            id_ex_pc         <= id_pc;
            id_ex_pc_4       <= id_pc_4;
            id_ex_inst       <= id_inst;

            id_ex_rs1_data   <= id_rs1_data;
            id_ex_rs2_data   <= id_rs2_data;
            id_ex_imm        <= id_immediate;

            id_ex_inst_30    <= id_inst_30;
            id_ex_inst_14_12 <= id_inst_14_12;

            id_ex_Jalr       <= id_Jalr;
            id_ex_Jal        <= id_Jal;
            id_ex_Branch     <= id_Branch;
            id_ex_BranchType <= id_BranchType;

            id_ex_MemRead    <= id_MemRead;
            id_ex_MemWrite   <= id_MemWrite;
            id_ex_WriteRegSrc<= id_WriteRegSrc;

            id_ex_ALUOp      <= id_ALUOp;
            id_ex_ALUSrcA    <= id_ALUSrcA;
            id_ex_ALUSrcB    <= id_ALUSrcB;
            id_ex_RegWrite   <= id_RegWrite;
            id_ex_Mask       <= id_Mask;
        end else id_ex_valid <= 1'b0; // if stalled, we need to invalidate the instruction in the execute stage, because it's not finished yet, and we will insert the nop instruction into the execute stage in the next cycle, so we need to invalidate the instruction in the execute stage to avoid any side effect from the instruction in the execute stage.
    end
    
    // phase 3: execute
    execute u_execute (
        .i_pc                    (id_ex_pc),
        .i_pc_4                  (id_ex_pc_4),
        .i_funct3                (id_ex_inst_14_12),
        .i_is_sub                (id_ex_inst_30),

        .i_rs1_data              (id_ex_rs1_data),
        .i_rs2_data              (id_ex_rs2_data),
        .i_immediate             (id_ex_imm),

        .i_Jalr                  (id_ex_Jalr),
        .i_Jal                   (id_ex_Jal),
        .i_Branch                (id_ex_Branch),
        .i_BranchType            (id_ex_BranchType),

        .i_ALUOp                 (id_ex_ALUOp),
        .i_ALUSrcA               (id_ex_ALUSrcA),
        .i_ALUSrcB               (id_ex_ALUSrcB),

        .i_MemRead               (id_ex_MemRead),
        .i_WriteRegSrc           (id_ex_WriteRegSrc),
        .i_MemWrite              (id_ex_MemWrite),
        .i_Mask                  (id_ex_Mask),
        // .i_wb_data               (mem_wb_data),
        .i_inst                 (id_ex_inst),
        .o_inst                 (ex_inst),
        .o_MemRead               (ex_MemRead),
        .o_WriteRegSrc           (ex_WriteRegSrc),
        .o_MemWrite              (ex_MemWrite),
        .o_Mask                  (ex_Mask),

        .o_rs1_data              (ex_rs1_data),
        .o_rs2_data              (ex_rs2_data),
        .o_pc                    (ex_pc),
        .o_pc_4                  (ex_pc_4),
        .o_funct3                (ex_funct3),
        .o_alu_result            (ex_alu_result),
        .addr_l2sb               (ex_addr_l2sb),    //use for address alignment in memory access

        .o_next_pc_resolve      (ex_next_pc_target_addr),   // for next PC calculation in hart (branch/jal target or jalr target)
        // .o_branch_jal_target_addr(ex_branch_jal_target_addr),
        // .o_jalr_target_addr      (ex_jalr_target_addr),
        // .o_jalr                  (ex_Jalr),
        .o_store_data            (ex_store_data),
        // .o_wb_data               (ex_wb_data),
        .o_misaligned            (ex_dmemmisaligned),
        .not_pc_4                (ex_not_pc_4),

        //forwarding signals
        .i_ex_to_ex_forward_rs1_en(ex_to_ex_forward_rs1_en),
        .i_ex_to_ex_forward_rs2_en(ex_to_ex_forward_rs2_en),
        .i_mem_to_ex_forward_rs1_en(mem_to_ex_forward_rs1_en),
        .i_mem_to_ex_forward_rs2_en(mem_to_ex_forward_rs2_en),
        //forwarding data
        .i_ex_mem_alu_result(ex_mem_alu_result),
        .i_wb_wdata(wb_wdata)
    );



    always @(posedge i_clk) begin
        if (i_rst||nop_mem) begin
            ex_mem_valid <= !nop_mem; // if it's a nop, then it's not valid, if it's not a nop, then it's valid
            ex_mem_inst        <= 32'h00000013; // NOP
            ex_mem_MemRead     <= 1'b0;
            ex_mem_WriteRegSrc <= 2'd0;
            ex_mem_MemWrite    <= 1'b0;
            ex_mem_Mask        <= 4'd0;
            ex_mem_RegWrite    <= 1'b0;

            ex_mem_rs1_data    <= 32'd0;
            ex_mem_rs2_data    <= 32'd0;
            ex_mem_pc          <= 32'd0;
            ex_mem_pc_4        <= 32'd0;
            ex_mem_funct3      <= 3'd0;
            ex_mem_alu_result  <= 32'd0;
            ex_mem_addr_l2sb   <= 2'd0;
            ex_mem_next_pc     <= 32'd0;
            ex_mem_store_data  <= 32'd0;
            ex_mem_misaligned  <= 1'b0;
        end else if(!stall_mem) begin
            ex_mem_valid <= id_ex_valid; // if the instruction in the execute stage is valid, then the instruction in the memory stage is valid, otherwise it's not valid
            ex_mem_inst        <= ex_inst;
            ex_mem_MemRead     <= ex_MemRead;
            ex_mem_WriteRegSrc <= ex_WriteRegSrc;
            ex_mem_MemWrite    <= ex_MemWrite;
            ex_mem_Mask        <= ex_Mask;
            ex_mem_RegWrite    <= id_ex_RegWrite;

            ex_mem_rs1_data    <= ex_rs1_data;
            ex_mem_rs2_data    <= ex_rs2_data;
            ex_mem_pc          <= ex_pc;
            ex_mem_pc_4        <= ex_pc_4;
            ex_mem_funct3      <= ex_funct3;
            ex_mem_alu_result  <= ex_alu_result;
            ex_mem_addr_l2sb   <= ex_addr_l2sb;
            ex_mem_next_pc     <= ex_next_pc_target_addr;
            ex_mem_store_data  <= ex_store_data;
            ex_mem_misaligned  <= ex_dmemmisaligned;
        end else ex_mem_valid <= 1'b0; // if stalled, we need to invalidate the instruction in the memory stage, because it's not finished yet, and we will insert the nop instruction into the memory stage in the next cycle, so we need to invalidate the instruction in the memory stage to avoid any side effect from the instruction in the memory stage.
    end

    assign o_dmem_addr = ex_mem_alu_result;
    assign o_dmem_ren = ex_mem_MemRead;
    assign o_dmem_wen = ex_mem_MemWrite;
    assign o_dmem_mask = ex_mem_Mask;
    assign o_dmem_wdata = ex_mem_store_data;

    // phase 4: memory
    memory_stage u_memory (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_next_pc               (ex_mem_next_pc),
        .i_alu_result            (ex_mem_alu_result),
        .i_pc_4                  (ex_mem_pc_4),
        .i_funct3                (ex_mem_funct3),

        .i_WriteRegSrc           (ex_mem_WriteRegSrc),
        .i_Mask                  (ex_mem_Mask),

        .i_rs1_data(ex_mem_rs1_data),
        .i_rs2_data(ex_mem_rs2_data),
        .i_inst(ex_mem_inst),
        .i_pc(ex_mem_pc),
        .i_misaligned(ex_mem_misaligned),
        .o_misaligned(mem_misaligned),
        .o_pc(mem_pc),
        .o_inst(mem_inst),
        .o_rs1_data(mem_rs1_data),
        .o_rs2_data(mem_rs2_data),

        .o_funct3                (mem_funct3),
        .o_Mask                  (mem_Mask),
        .o_WriteRegSrc           (mem_WriteRegSrc),
        
        .o_pc_4                  (mem_pc_4),
        .o_next_pc               (mem_next_pc),
        .o_alu_result            (mem_alu_result),

            //extra signals for p6
        .i_ex_mem_MemRead(ex_mem_MemRead),
        .i_ex_mem_MemWrite(ex_mem_MemWrite),
        //ren signals
        .i_dmem_ready(i_dmem_ready),
        .i_dmem_valid(i_dmem_valid),
        .o_dmem_ren(o_dmem_ren),
        .o_dmem_wen(o_dmem_wen),
        .o_memory_wait(memory_wait)
    );

    



    always @(posedge i_clk) begin
        if (i_rst || nop_wb) begin
            mem_wb_valid <= !nop_wb; // if it's a nop, then it's not valid, if it's not a nop, then it's valid
            mem_wb_misaligned    <= 1'b0;
            mem_wb_pc            <= 32'd0;
            mem_wb_inst          <= 32'h00000013; // NOP
            mem_wb_rs1_data      <= 32'd0;
            mem_wb_rs2_data      <= 32'd0;

            mem_wb_funct3        <= 3'd0;
            mem_wb_Mask          <= 4'd0;
            mem_wb_WriteRegSrc   <= 2'd0;

            mem_wb_pc_4          <= 32'd0;
            mem_wb_next_pc       <= 32'd0;
            mem_wb_alu_result    <= 32'd0;

            mem_wb_mem_read_data <= 32'd0;
            mem_wb_RegWrite <= 1'b0;
        end else begin
            mem_wb_valid <= ex_mem_valid; // if the instruction in the memory stage is valid, then the instruction in the writeback stage is valid, otherwise it's not valid
            mem_wb_misaligned    <= mem_misaligned;
            mem_wb_pc            <= mem_pc;
            mem_wb_inst          <= mem_inst;
            mem_wb_rs1_data      <= mem_rs1_data;
            mem_wb_rs2_data      <= mem_rs2_data;

            mem_wb_funct3        <= mem_funct3;
            mem_wb_Mask          <= mem_Mask;
            mem_wb_WriteRegSrc   <= mem_WriteRegSrc;

            mem_wb_pc_4          <= mem_pc_4;
            mem_wb_next_pc       <= mem_next_pc;
            mem_wb_alu_result    <= mem_alu_result;

            // pipeline the dmem combinational read data in MEM stage
            mem_wb_mem_read_data <= i_dmem_rdata;
            mem_wb_RegWrite <= ex_mem_RegWrite;
        end
    end
    
    // phase 5: writeback
    writeback u_writeback (
        .i_next_pc               (mem_wb_next_pc),
        .i_alu_result            (mem_wb_alu_result),
        .i_pc_4                  (mem_wb_pc_4),
        .i_mem_read_data         (mem_wb_mem_read_data),
        .i_WriteRegSrc           (mem_wb_WriteRegSrc),

        .i_rs1_data(mem_wb_rs1_data),
        .i_rs2_data(mem_wb_rs2_data),
        .i_Mask(mem_wb_Mask),
        .i_funct3(mem_wb_funct3),
        .i_inst(mem_wb_inst),
        .i_pc(mem_wb_pc),
        .i_misaligned(mem_wb_misaligned),

        .o_misaligned(wb_misaligned),
        .o_pc(wb_pc),
        .o_inst(wb_inst),
        .o_rs1_data(wb_rs1_data),
        .o_rs2_data(wb_rs2_data),

        .o_next_pc               (wb_next_pc),
        .o_writeback_data        (wb_wdata)
    );

	wire next_pc_unaligned = (wb_next_pc[1:0] != 2'b00);

    assign o_retire_next_pc = wb_next_pc; //next PC after this instruction (branch/jump target or pc+4)


	assign o_retire_valid     = ~(wb_inst == 32'h00000013) && mem_wb_valid ; //when the instruction is not nop
	assign o_retire_inst      = wb_inst;
	assign o_retire_halt      = (wb_inst == 32'h00100073);
	assign o_retire_trap      = wb_misaligned; 
	assign o_retire_pc        = wb_pc;
	
    assign op = wb_inst[6:0];
    assign o_retire_rd_waddr  = mem_wb_RegWrite ? wb_inst[11:7] : 5'd0;

    assign o_retire_rd_wdata  = wb_wdata;
    	
    assign o_retire_rs1_raddr = wb_inst[19:15]; // rs1

    assign o_retire_rs2_raddr = wb_inst[24:20]; // rs2
	assign o_retire_rs1_rdata = wb_rs1_data;
	assign o_retire_rs2_rdata = wb_rs2_data; 

    //this reg are for the sole purpose of passing the retire dmem signals from the execute stage to the retire interface,
    // since the retire interface is connected after the writeback stage, we need to pipeline the dmem signals in the memory stage and pass them to the retire interface in the writeback stage
    //some are defined already
    //reg[31:0] mem_wb_alu_result;    
    reg mem_wb_MemRead, mem_wb_MemWrite;
    // reg[3:0] mem_wb_Mask;
    reg[31:0] mem_wb_store_data;
    always@(posedge i_clk) begin
        //mem_wb_alu_result <= ex_mem_alu_result;
        mem_wb_MemRead <= ex_mem_MemRead;
        mem_wb_MemWrite <= ex_mem_MemWrite;
        //mem_wb_Mask <= ex_mem_Mask;
        mem_wb_store_data <= ex_mem_store_data;        
    end
    assign o_retire_dmem_addr = mem_wb_alu_result; // the dmem address accessed by this instruction, which is calculated in the execute stage, and passed through the memory stage to the writeback stage for the retire interface
    assign o_retire_dmem_ren  = mem_wb_MemRead;
    assign o_retire_dmem_wen  = mem_wb_MemWrite;
    assign o_retire_dmem_mask = mem_wb_Mask;
    assign o_retire_dmem_wdata= mem_wb_store_data;
    assign o_retire_dmem_rdata = mem_wb_mem_read_data;

    //instantiate hazard_detect.v
    hazard_detect u_hazard_detect (
        .i_if_idrs1_addr (if_id_inst[19:15]),
        .i_if_idrs2_addr (if_id_inst[24:20]),
        .i_id_ex_rd_addr (id_ex_inst[11:7]),
        .i_id_ex_reg_write_en(id_ex_RegWrite),
        .i_id_ex_MemRead(id_ex_MemRead),
        .i_ex_mem_rd_addr(ex_mem_inst[11:7]),
        .i_ex_mem_reg_write_en(ex_mem_RegWrite),
        .o_nop_mem(nop_mem),
        .o_nop_ex(nop_ex),
        .o_nop_id(nop_id),
        .o_nop_wb(nop_wb),
        .o_stall_mem(stall_mem),
        .o_stall_fetch(stall_fetch),
        .o_stall_execute(stall_execute),
        .o_stall_decode(stall_decode),
        .i_ex_not_pc_4(ex_not_pc_4), // for hazard detection to determine if the instruction in execute stage is a control transfer instruction or not (for better timing, we use the signal generated in execute stage, which is used for next PC calculation in hart, to determine if the instruction is control transfer or not)
        .i_id_not_pc_4(1'b0),//TODO: not yet implemented
        .i_id_MemWrite(id_MemWrite), // for store-load hazard detection, if the instruction in memory stage is a store instruction, and the instruction in execute stage is a load instruction that reads from the same address, we need to stall the execute stage to wait for the store instruction to write the data to memory
        .i_ex_mem_MemRead(ex_mem_MemRead), // for load-use hazard detection, if the instruction in memory stage is a load instruction, and the instruction in execute stage reads from the same address, we need to stall the execute stage to wait for the load instruction to read the data from memory
        .i_ex_mem_MemWrite(ex_mem_MemWrite),
        .i_imem_valid(i_imem_valid), // for stalling the fetch stage when there's a hazard, we only want to stall the fetch stage when the instruction in the fetch stage is valid, otherwise we might stall the fetch stage unnecessarily when there's a hazard but the instruction in the fetch stage is not valid yet
        .i_dmem_valid(i_dmem_valid), // for stalling the fetch stage when there's a hazard, we only want to stall the fetch stage when the instruction in the memory stage is valid, otherwise we might stall the fetch stage unnecessarily when there's a hazard but the instruction in the memory stage is not valid yet
        .i_dmem_ready(i_dmem_ready),
        .i_imem_ready(i_imem_ready)
    );

    //instantiate forward.v
    forward u_forward (
        .i_id_ex_rs1_addr(id_ex_inst[19:15]),
        .i_id_ex_rs2_addr(id_ex_inst[24:20]),
        .i_ex_mem_rd_addr(ex_mem_inst[11:7]),
        .i_ex_mem_reg_write_en(ex_mem_RegWrite),
        .i_mem_wb_rd_addr(mem_wb_inst[11:7]),
        .i_mem_wb_reg_write_en(mem_wb_RegWrite),
        .i_ex_mem_rs2_addr(ex_mem_inst[24:20]), // for store data forwarding
        .i_ex_mem_MemWrite(ex_mem_MemWrite), // for store data forwarding
        .o_ex_to_ex_forward_rs1_en(ex_to_ex_forward_rs1_en),
        .o_ex_to_ex_forward_rs2_en(ex_to_ex_forward_rs2_en),
        .o_mem_to_ex_forward_rs1_en(mem_to_ex_forward_rs1_en),
        .o_mem_to_ex_forward_rs2_en(mem_to_ex_forward_rs2_en),
        .o_mem_to_mem_forward_rs2_en(mem_to_mem_forward_rs2_en)
    );
endmodule

`default_nettype wire
