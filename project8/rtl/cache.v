`default_nettype none


module cache #(
    parameter ENABLE_PREFETCH = 1'b0
) (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // External memory interface. See hart interface for details. This
    // interface is nearly identical to the phase 5 memory interface, with the
    // exception that the byte mask (`o_mem_mask`) has been removed. This is
    // no longer needed as the cache will only access the memory at word
    // granularity, and implement masking internally.
    input  wire        i_mem_ready,
    output wire [31:0]  o_mem_addr,  //reg to wire
    output wire         o_mem_ren,   //reg to wire
    output wire         o_mem_wen,   //reg to wire
    output wire [31:0] o_mem_wdata,  //reg to wire
    input  wire [31:0] i_mem_rdata,
    input  wire        i_mem_valid,
    // Interface to CPU hart. This is nearly identical to the phase 5 hart memory
    // interface, but includes a stall signal (`o_busy`), and the input/output
    // polarities are swapped for obvious reasons.
    //
    // The CPU should use this as a stall signal for both instruction fetch
    // (IF) and memory (MEM) stages, from the instruction or data cache
    // respectively. If a memory request is made (`i_req_ren` for instruction
    // cache, or either `i_req_ren` or `i_req_wen` for data cache), this
    // should be asserted *combinationally* if the request results in a cache
    // miss.
    //
    // In case of a cache miss, the CPU must stall the respective pipeline
    // stage and deassert ren/wen on subsequent cycles, until the cache
    // deasserts `o_busy` to indicate it has serviced the cache miss. However,
    // the CPU must keep the other request lines constant. For example, the
    // CPU should not change the request address while stalling.
    output wire        o_busy,
    // 32-bit read/write address to access from the cache. This should be
    // 32-bit aligned (i.e. the two LSBs should be zero). See `i_req_mask` for
    // how to perform half-word and byte accesses to unaligned addresses.
    input  wire [31:0] i_req_addr,
    // When asserted, the cache should perform a read at the aligned address
    // specified by `i_req_addr` and return the 32-bit word at that address,
    // either immediately (i.e. combinationally) on a cache hit, or
    // synchronously on a cache miss. It is illegal to assert this and
    // `i_dmem_wen` on the same cycle.
    input  wire        i_req_ren,
    // When asserted, the cache should perform a write at the aligned address
    // specified by `i_req_addr` with the 32-bit word provided in
    // `o_req_wdata` (specified by the mask). This is necessarily synchronous,
    // but may either happen on the next clock edge (on a cache hit) or after
    // multiple cycles of latency (cache miss). As the cache is write-through
    // and write-allocate, writes must be applied to both the cache and
    // underlying memory.
    // It is illegal to assert this and `i_dmem_ren` on the same cycle.
    input  wire        i_req_wen,
    // The memory interface expects word (32 bit) aligned addresses. However,
    // WISC-25 supports byte and half-word loads and stores at unaligned and
    // 16-bit aligned addresses, respectively. To support this, the access
    // mask specifies which bytes within the 32-bit word are actually read
    // from or written to memory.
    input  wire [ 3:0] i_req_mask,
    // The 32-bit word to write to memory, if the request is a write
    // (i_req_wen is asserted). Only the bytes corresponding to set bits in
    // the mask should be written into the cache (and to backing memory).
    input  wire [31:0] i_req_wdata,
    // THe 32-bit data word read from memory on a read request.
    output wire [31:0] o_res_rdata   //reg to wire
);
    // These parameters are equivalent to those provided in the project
    // 6 specification. Feel free to use them, but hardcoding these numbers
    // rather than using the localparams is also permitted, as long as the
    // same values are used (and consistent with the project specification).
    //
    // 16 sets * 4 ways per set * 16 bytes per way = 1K cache
    localparam O = 4;            // 4 bit offset => 16 byte cache line
    localparam S = 4;            // 4 bit set index => 16 sets
    localparam DEPTH = 16;   // 16 sets
    localparam W = 4;            // 4 way set associative, NMRU
    localparam T = 24;   // 24 bit tag 32 - 4 - 4=24
    localparam D = 4;   // 16 bytes per line / 4 bytes per word = 4 words per line


    // The following memory arrays model the cache structure. As this is
    // an internal implementation detail, you are *free* to modify these
    // arrays as you please.


    // Backing memory, modeled as two separate ways.
    reg [   31:0] datas0 [DEPTH - 1:0][D - 1:0];
    reg [   31:0] datas1 [DEPTH - 1:0][D - 1:0];
    reg [   31:0] datas2 [DEPTH - 1:0][D - 1:0];
    reg [   31:0] datas3 [DEPTH - 1:0][D - 1:0];
    reg [T - 1:0] tags0  [DEPTH - 1:0];
    reg [T - 1:0] tags1  [DEPTH - 1:0];
    reg [T - 1:0] tags2  [DEPTH - 1:0];
    reg [T - 1:0] tags3  [DEPTH - 1:0];
    reg [3:0] valid [DEPTH - 1:0];


    // reg  [1:0] lru   [DEPTH - 1:0];
    reg lru01 [DEPTH - 1:0];    //1 means way 0 is more recently used than way 1, 0 means way 1 is more recently used than way 0
    reg lru02 [DEPTH - 1:0];    //1 means way 0 is more recently used than way 2, 0 means way 2 is more recently used than way 0
    reg lru03 [DEPTH - 1:0];    //1 means way 0 is more recently used than way 3, 0 means way 3 is more recently used than way 0
    reg lru12 [DEPTH - 1:0];    //1 means way 1 is more recently used than way 2, 0 means way 2 is more recently used than way 1
    reg lru13 [DEPTH - 1:0];    //1 means way 1 is more recently used than way 3, 0 means way 3 is more recently used than way 1
    reg lru23 [DEPTH - 1:0];    //1 means way 2 is more recently used than way 3, 0 means way 3 is more recently used than way 2

    // address
    wire [T-1:0] req_tag   = i_req_addr[31:8];
    wire [S-1:0] req_index = i_req_addr[7:4];
    wire [1:0]   req_offset= i_req_addr[3:2];


    reg  [31:0] miss_addr_q;
    reg  [3:0]  miss_mask_q;
    reg  [31:0] miss_wdata_q;


    wire [T-1:0] miss_tag    = miss_addr_q[31:8];
    wire [S-1:0] miss_index  = miss_addr_q[7:4];
    wire [1:0]   miss_offset = miss_addr_q[3:2];


    // Hit
    wire hit0 = valid[req_index][0] & (tags0[req_index] == req_tag);
    wire hit1 = valid[req_index][1] & (tags1[req_index] == req_tag);
    wire hit2 = valid[req_index][2] & (tags2[req_index] == req_tag);
    wire hit3 = valid[req_index][3] & (tags3[req_index] == req_tag);
    wire hit  = hit0 | hit1 | hit2 | hit3;

    // lru victim selection. We can determine the LRU victim by comparing the lru bits.
    wire [1:0] lru_victim =
        (!lru01[req_index] && !lru02[req_index] && !lru03[req_index]) ? 2'd0 :
        ( lru01[req_index] && !lru12[req_index] && !lru13[req_index]) ? 2'd1 :
        ( lru02[req_index] &&  lru12[req_index] && !lru23[req_index]) ? 2'd2 :
                                                                        2'd3;
    // Prefer filling an invalid way before evicting by LRU
    wire [1:0] victim_select =
        !valid[req_index][0] ? 2'b00 :
        !valid[req_index][1] ? 2'b01 :
        !valid[req_index][2] ? 2'b10 :
        !valid[req_index][3] ? 2'b11 :
        lru_victim;
   
    // Single-entry write buffer.
    reg        wb_valid;
    reg [31:0] wb_addr_q;
    reg [31:0] wb_wdata_q;


    // Full word for store hit after applying byte mask.
    // Important for SB/SH because backing memory has no byte mask.
    wire [31:0] hit_word =
        hit0 ? datas0[req_index][req_offset] :
        hit1 ? datas1[req_index][req_offset] :
        hit2 ? datas2[req_index][req_offset] :
        hit3 ? datas3[req_index][req_offset] :
               32'd0;


    wire [31:0] store_hit_word = {
        i_req_mask[3] ? i_req_wdata[31:24] : hit_word[31:24],
        i_req_mask[2] ? i_req_wdata[23:16] : hit_word[23:16],
        i_req_mask[1] ? i_req_wdata[15: 8] : hit_word[15: 8],
        i_req_mask[0] ? i_req_wdata[ 7: 0] : hit_word[ 7: 0]
    };


    //state
    // parameter IDLE = 2'd0, REFILL = 2'd1, WRITE_THROUGH = 2'd2;
    parameter IDLE = 2'd0, REFILL = 2'd1;
    reg [1:0] state, next_state;
   
    //memory register
    reg [31:0] mem_addr_reg;
    reg [31:0] mem_res_rdata;
    reg        mem_ren_reg;
    reg        mem_wen_reg;
    reg [31:0] mem_wdata_reg;


    assign o_mem_addr  = mem_addr_reg;
    // assign o_res_rdata = mem_res_rdata;
    // assign o_res_rdata = hit0 ? datas0[req_index][req_offset] :
    //                     hit1 ? datas1[req_index][req_offset] :
    //                     mem_res_rdata;
    // o_res_rdata is assigned below after refill-return helper wires.
    reg mem_busy;
    reg refill_complete; //flag for refill complete
    always @(posedge i_clk) begin
        if (i_rst)
            mem_busy <= 1'b0;
        else if (i_mem_valid)
            mem_busy <= 1'b0;
        else if (o_mem_ren)
            mem_busy <= 1'b1;
    end                    
    //assign o_mem_ren   = state == REFILL && !mem_busy && i_mem_ready ? 1'b1 : 1'b0;
    //we don't busy weight for better cpi
    //flag for refill complete, so we only request 4 times
    assign o_mem_ren   = state == REFILL && i_mem_ready && !refill_complete ? 1'b1 : 1'b0;
    assign o_mem_wen   = mem_wen_reg;
    assign o_mem_wdata = mem_wdata_reg;
   
    reg [1:0] refill_counter;
    reg [1:0] recieve_counter;
    reg       miss_is_write;
    reg [1:0] victim_way;
    reg busy;


    assign o_busy = busy ? 1'b1 : 1'b0; //when miss or not idle, busy, otherwise not busy
   
    reg write_miss_propogation;
    always @(posedge i_clk) begin
        if(i_rst) begin
            write_miss_propogation <= 1'b0;
        //if miss is write, then propogate the signal to next cycle, otherwise reset to 0
        end else if(miss_is_write) begin
            write_miss_propogation <= miss_is_write ? 1'b1 : 1'b0;
        end else if(!o_busy) begin //when not busy, stop propogation, reset to 0
            write_miss_propogation <= 1'b0;
        end
        //while busy, stays the same, no update
    end


   
    //Optimization1:
    // Last memory response of a refill line.
    // For a read miss, we can return the missed word in this same cycle
    // instead of waiting one more cycle for state to become IDLE.
    wire refill_last_resp =
        (state == REFILL) && i_mem_valid && (recieve_counter == 2'd3);


    wire read_miss_done_now =
        refill_last_resp && !write_miss_propogation;


    wire [31:0] refill_return_word =
        (victim_way == 2'b00) ? ((miss_offset == recieve_counter) ? i_mem_rdata : datas0[miss_index][miss_offset]) :
        (victim_way == 2'b01) ? ((miss_offset == recieve_counter) ? i_mem_rdata : datas1[miss_index][miss_offset]) :
        (victim_way == 2'b10) ? ((miss_offset == recieve_counter) ? i_mem_rdata : datas2[miss_index][miss_offset]) :
        (victim_way == 2'b11) ? ((miss_offset == recieve_counter) ? i_mem_rdata : datas3[miss_index][miss_offset]) : 32'd0;


    // Full word for write miss after applying byte mask.
    // refill_return_word is the old memory word at the missed offset.
    wire [31:0] store_miss_word = {
        miss_mask_q[3] ? miss_wdata_q[31:24] : refill_return_word[31:24],
        miss_mask_q[2] ? miss_wdata_q[23:16] : refill_return_word[23:16],
        miss_mask_q[1] ? miss_wdata_q[15: 8] : refill_return_word[15: 8],
        miss_mask_q[0] ? miss_wdata_q[ 7: 0] : refill_return_word[ 7: 0]
    };


    assign o_res_rdata = hit0 ? datas0[req_index][req_offset] :
                        hit1 ? datas1[req_index][req_offset] :
                        hit2 ? datas2[req_index][req_offset] :
                        hit3 ? datas3[req_index][req_offset] :
                        read_miss_done_now ? refill_return_word :
                        mem_res_rdata;


   
    always @(posedge i_clk) begin
        if(i_rst) state <= IDLE;
        else state <= next_state;
    end


    reg [31:0] i_mem_next_rdata;
    wire [31:0] next_mem_addr = (miss_addr_q & 32'hFFFF_FFF0) + {28'd0, refill_counter, 2'b00};
    reg transit_into_refill;
    always @(*) begin
        mem_addr_reg = '0;
        mem_wen_reg = 1'b0;
        mem_wdata_reg = '0;
        transit_into_refill = 1'b0;
        busy = 1'b1;
        case (state)
            IDLE: begin
                // Drain pending write buffer entry in the background.
                mem_wen_reg   = wb_valid & i_mem_ready;
                mem_addr_reg  = (wb_valid & i_mem_ready) ? wb_addr_q : 32'd0;
                mem_wdata_reg = (wb_valid & i_mem_ready) ? wb_wdata_q : 32'd0;


                // Miss still stalls. Store hit only stalls if buffer is full and memory
                // cannot drain it this cycle.
                busy = (((i_req_ren | i_req_wen) & ~hit) |
                        (i_req_wen & hit & wb_valid & !i_mem_ready)) ? 1'b1 : 1'b0;


                // Start a refill only when there is no pending write, or the pending
                // write is draining this cycle.
                next_state =
                    (((i_req_ren | i_req_wen) & ~hit) & (!wb_valid | i_mem_ready)) ?
                        REFILL : IDLE;


                transit_into_refill =
                    (((i_req_ren | i_req_wen) & ~hit) & (!wb_valid | i_mem_ready)) ?
                        1'b1 : 1'b0;
            end


            REFILL: begin
                // Read miss and write miss can both finish on the final refill response.
                // For write miss, the write-through memory update is buffered.
                busy = (read_miss_done_now |
                    (refill_last_resp & write_miss_propogation)) ? 1'b0 : 1'b1;


                next_state =
                    ((i_mem_valid & (recieve_counter == 2'd3)) ? IDLE : REFILL);


                mem_addr_reg = i_mem_ready ? next_mem_addr : 32'd0;
            end
            // WRITE_THROUGH: begin
            //     next_state   = (i_mem_ready ? IDLE : WRITE_THROUGH);
            //     mem_addr_reg = miss_addr_q;
            // end
            default:
                next_state = IDLE;
        endcase
    end

    always @(posedge i_clk) begin
        if (i_rst) begin
            refill_complete <= 1'b0;
        end else if (state != REFILL) begin
            refill_complete <= 1'b0;
        end else if (state == REFILL && i_mem_ready && !refill_complete && refill_counter == 2'd3) begin
            refill_complete <= 1'b1;
        end else begin
            refill_complete <= refill_complete;
        end
    end
    // integer i;


    always @(posedge i_clk) begin
        if (i_rst) begin
            refill_counter <= 2'd0;
        end else if (state != REFILL) begin
            refill_counter <= 2'd0;
        end else if (state == REFILL && i_mem_ready && !refill_complete) begin
            refill_counter <= refill_counter + 2'd1;
        end
    end


    //recieve counter for receiving data from memory, used for writing into cache
    always @(posedge i_clk) begin
        if (i_rst) begin
            recieve_counter <= 0;
        end else if (state == REFILL && i_mem_valid) begin
            recieve_counter <= recieve_counter + 1;
        end else if (state == REFILL && !i_mem_valid) begin
            recieve_counter <= recieve_counter;
        end else begin
            recieve_counter <= 0;
        end
    end


    // Single-entry write buffer update.
    always @(posedge i_clk) begin
        if (i_rst) begin
            wb_valid   <= 1'b0;
            wb_addr_q  <= 32'd0;
            wb_wdata_q <= 32'd0;
        end else if (state == IDLE && i_req_wen && hit && (!wb_valid || i_mem_ready)) begin
            // Store hit: update cache now, write memory later.
            // If old buffer entry is draining this cycle, this replaces it.
            wb_valid   <= 1'b1;
            wb_addr_q  <= i_req_addr & 32'hFFFF_FFFC;
            wb_wdata_q <= store_hit_word;
        end else if (state == REFILL && i_mem_valid && (recieve_counter == 2'd3) && write_miss_propogation) begin
            // Write miss: after refill completes, buffer the write-through update.
            wb_valid   <= 1'b1;
            wb_addr_q  <= miss_addr_q & 32'hFFFF_FFFC;
            wb_wdata_q <= store_miss_word;
        end else if (state == IDLE && wb_valid && i_mem_ready) begin
            // Background write finished.
            wb_valid <= 1'b0;
        end
    end


    always @(posedge i_clk) begin
        if (i_rst) begin
            mem_wen_reg <= 0;
            mem_res_rdata <= 0;
            mem_wdata_reg <= 0;


            miss_is_write <= 0;
            victim_way <= 0;
            miss_addr_q  <= 32'd0;
            miss_mask_q  <= 4'd0;
            miss_wdata_q <= 32'd0;
            // for (i = 0; i < DEPTH; i = i + 1) begin///synthesis???
            //  valid[i] <= 0;
            //  lru[i]   <= 0;
            // end
            // for verilog rule DEPTH = 16
            valid[0] <= 4'b0000;
            valid[1] <= 4'b0000;
            valid[2] <= 4'b0000;
            valid[3] <= 4'b0000;
            valid[4] <= 4'b0000;
            valid[5] <= 4'b0000;
            valid[6] <= 4'b0000;
            valid[7] <= 4'b0000;
            valid[8] <= 4'b0000;
            valid[9] <= 4'b0000;
            valid[10] <= 4'b0000;
            valid[11] <= 4'b0000;
            valid[12] <= 4'b0000;
            valid[13] <= 4'b0000;
            valid[14] <= 4'b0000;
            valid[15] <= 4'b0000;
            
            lru01[0] <= 1'b0; lru02[0] <= 1'b0; lru03[0] <= 1'b0; lru12[0] <= 1'b0; lru13[0] <= 1'b0; lru23[0] <= 1'b0;
            lru01[1] <= 1'b0; lru02[1] <= 1'b0; lru03[1] <= 1'b0; lru12[1] <= 1'b0; lru13[1] <= 1'b0; lru23[1] <= 1'b0;
            lru01[2] <= 1'b0; lru02[2] <= 1'b0; lru03[2] <= 1'b0; lru12[2] <= 1'b0; lru13[2] <= 1'b0; lru23[2] <= 1'b0;
            lru01[3] <= 1'b0; lru02[3] <= 1'b0; lru03[3] <= 1'b0; lru12[3] <= 1'b0; lru13[3] <= 1'b0; lru23[3] <= 1'b0;
            lru01[4] <= 1'b0; lru02[4] <= 1'b0; lru03[4] <= 1'b0; lru12[4] <= 1'b0; lru13[4] <= 1'b0; lru23[4] <= 1'b0;
            lru01[5] <= 1'b0; lru02[5] <= 1'b0; lru03[5] <= 1'b0; lru12[5] <= 1'b0; lru13[5] <= 1'b0; lru23[5] <= 1'b0;
            lru01[6] <= 1'b0; lru02[6] <= 1'b0; lru03[6] <= 1'b0; lru12[6] <= 1'b0; lru13[6] <= 1'b0; lru23[6] <= 1'b0;
            lru01[7] <= 1'b0; lru02[7] <= 1'b0; lru03[7] <= 1'b0; lru12[7] <= 1'b0; lru13[7] <= 1'b0; lru23[7] <= 1'b0;
            lru01[8] <= 1'b0; lru02[8] <= 1'b0; lru03[8] <= 1'b0; lru12[8] <= 1'b0; lru13[8] <= 1'b0; lru23[8] <= 1'b0;
            lru01[9] <= 1'b0; lru02[9] <= 1'b0; lru03[9] <= 1'b0; lru12[9] <= 1'b0; lru13[9] <= 1'b0; lru23[9] <= 1'b0;
            lru01[10] <= 1'b0; lru02[10] <= 1'b0; lru03[10] <= 1'b0; lru12[10] <= 1'b0; lru13[10] <= 1'b0; lru23[10] <= 1'b0;
            lru01[11] <= 1'b0; lru02[11] <= 1'b0; lru03[11] <= 1'b0; lru12[11] <= 1'b0; lru13[11] <= 1'b0; lru23[11] <= 1'b0;
            lru01[12] <= 1'b0; lru02[12] <= 1'b0; lru03[12] <= 1'b0; lru12[12] <= 1'b0; lru13[12] <= 1'b0; lru23[12] <= 1'b0;
            lru01[13] <= 1'b0; lru02[13] <= 1'b0; lru03[13] <= 1'b0; lru12[13] <= 1'b0; lru13[13] <= 1'b0; lru23[13] <= 1'b0;
            lru01[14] <= 1'b0; lru02[14] <= 1'b0; lru03[14] <= 1'b0; lru12[14] <= 1'b0; lru13[14] <= 1'b0; lru23[14] <= 1'b0;
            lru01[15] <= 1'b0; lru02[15] <= 1'b0; lru03[15] <= 1'b0; lru12[15] <= 1'b0; lru13[15] <= 1'b0; lru23[15] <= 1'b0;
            // valid[16] <= 2'b00; lru[16] <= 1'b0;
            // valid[17] <= 2'b00; lru[17] <= 1'b0;
            // valid[18] <= 2'b00; lru[18] <= 1'b0;
            // valid[19] <= 2'b00; lru[19] <= 1'b0;
            // valid[20] <= 2'b00; lru[20] <= 1'b0;
            // valid[21] <= 2'b00; lru[21] <= 1'b0;
            // valid[22] <= 2'b00; lru[22] <= 1'b0;
            // valid[23] <= 2'b00; lru[23] <= 1'b0;
            // valid[24] <= 2'b00; lru[24] <= 1'b0;
            // valid[25] <= 2'b00; lru[25] <= 1'b0;
            // valid[26] <= 2'b00; lru[26] <= 1'b0;
            // valid[27] <= 2'b00; lru[27] <= 1'b0;
            // valid[28] <= 2'b00; lru[28] <= 1'b0;
            // valid[29] <= 2'b00; lru[29] <= 1'b0;
            // valid[30] <= 2'b00; lru[30] <= 1'b0;
            // valid[31] <= 2'b00; lru[31] <= 1'b0;
        end else begin
            mem_wen_reg <= 0;
            miss_is_write <= 0;
            case (state)
                IDLE: begin
                    //Miss
                    // if ((i_req_ren | i_req_wen) & ~hit) begin
                    if ((i_req_ren | i_req_wen) & ~hit & (!wb_valid | i_mem_ready)) begin
                        miss_is_write <= i_req_wen;
                        victim_way    <= victim_select;


                        miss_addr_q   <= i_req_addr;
                        miss_mask_q   <= i_req_mask;
                        miss_wdata_q  <= i_req_wdata;
                    end
                    // if(i_req_wen & hit) begin
                    if(i_req_wen & hit & (!wb_valid | i_mem_ready)) begin
                        if (hit0) begin
                            datas0[req_index][req_offset][ 7: 0] <= i_req_mask[0] ? i_req_wdata[ 7: 0] : datas0[req_index][req_offset][ 7: 0];
                            datas0[req_index][req_offset][15: 8] <= i_req_mask[1] ? i_req_wdata[15: 8] : datas0[req_index][req_offset][15: 8];
                            datas0[req_index][req_offset][23:16] <= i_req_mask[2] ? i_req_wdata[23:16] : datas0[req_index][req_offset][23:16];
                            datas0[req_index][req_offset][31:24] <= i_req_mask[3] ? i_req_wdata[31:24] : datas0[req_index][req_offset][31:24];
                            lru01[req_index] <= 1'b1;
                            lru02[req_index] <= 1'b1;
                            lru03[req_index] <= 1'b1;
                        end else if (hit1) begin
                            datas1[req_index][req_offset][ 7: 0] <= i_req_mask[0] ? i_req_wdata[ 7: 0] : datas1[req_index][req_offset][ 7: 0];
                            datas1[req_index][req_offset][15: 8] <= i_req_mask[1] ? i_req_wdata[15: 8] : datas1[req_index][req_offset][15: 8];
                            datas1[req_index][req_offset][23:16] <= i_req_mask[2] ? i_req_wdata[23:16] : datas1[req_index][req_offset][23:16];
                            datas1[req_index][req_offset][31:24] <= i_req_mask[3] ? i_req_wdata[31:24] : datas1[req_index][req_offset][31:24];
                            lru01[req_index] <= 1'b0;
                            lru12[req_index] <= 1'b1;
                            lru13[req_index] <= 1'b1;
                        end else if (hit2) begin
                            datas2[req_index][req_offset][ 7: 0] <= i_req_mask[0] ? i_req_wdata[ 7: 0] : datas2[req_index][req_offset][ 7: 0];
                            datas2[req_index][req_offset][15: 8] <= i_req_mask[1] ? i_req_wdata[15: 8] : datas2[req_index][req_offset][15: 8];
                            datas2[req_index][req_offset][23:16] <= i_req_mask[2] ? i_req_wdata[23:16] : datas2[req_index][req_offset][23:16];
                            datas2[req_index][req_offset][31:24] <= i_req_mask[3] ? i_req_wdata[31:24] : datas2[req_index][req_offset][31:24];
                            lru02[req_index] <= 1'b0;
                            lru12[req_index] <= 1'b0;
                            lru23[req_index] <= 1'b1;
                        end else if (hit3) begin
                            datas3[req_index][req_offset][ 7: 0] <= i_req_mask[0] ? i_req_wdata[ 7: 0] : datas3[req_index][req_offset][ 7: 0];
                            datas3[req_index][req_offset][15: 8] <= i_req_mask[1] ? i_req_wdata[15: 8] : datas3[req_index][req_offset][15: 8];
                            datas3[req_index][req_offset][23:16] <= i_req_mask[2] ? i_req_wdata[23:16] : datas3[req_index][req_offset][23:16];
                            datas3[req_index][req_offset][31:24] <= i_req_mask[3] ? i_req_wdata[31:24] : datas3[req_index][req_offset][31:24];
                            lru03[req_index] <= 1'b0;
                            lru13[req_index] <= 1'b0;
                            lru23[req_index] <= 1'b0;
                        end
                    end
                    // Update LRU on read hit too.
                    if (i_req_ren & hit) begin
                        if (hit0) begin
                            lru01[req_index] <= 1'b1;
                            lru02[req_index] <= 1'b1;
                            lru03[req_index] <= 1'b1;
                        end else if (hit1) begin
                            lru01[req_index] <= 1'b0;
                            lru12[req_index] <= 1'b1;
                            lru13[req_index] <= 1'b1;
                        end else if (hit2) begin
                            lru02[req_index] <= 1'b0;
                            lru12[req_index] <= 1'b0;
                            lru23[req_index] <= 1'b1;
                        end else begin
                            lru03[req_index] <= 1'b0;
                            lru13[req_index] <= 1'b0;
                            lru23[req_index] <= 1'b0;
                        end
                    end
                end


                REFILL: begin                  
                    if (i_mem_valid) begin
                        if (victim_way == 0) datas0[miss_index][recieve_counter] <= i_mem_rdata;
                        else if (victim_way == 1)               datas1[miss_index][recieve_counter] <= i_mem_rdata;
                        else if (victim_way == 2)               datas2[miss_index][recieve_counter] <= i_mem_rdata;
                        else if (victim_way == 3)               datas3[miss_index][recieve_counter] <= i_mem_rdata;

                        if (recieve_counter == 2'd3) begin
                            if (victim_way == 0) begin
                                tags0[miss_index] <= miss_tag;
                                valid[miss_index][0] <= 1'b1;
                            end else if (victim_way == 1) begin
                                tags1[miss_index] <= miss_tag;
                                valid[miss_index][1] <= 1'b1;
                            end else if (victim_way == 2) begin
                                tags2[miss_index] <= miss_tag;
                                valid[miss_index][2] <= 1'b1;
                            end else if (victim_way == 3) begin
                                tags3[miss_index] <= miss_tag;
                                valid[miss_index][3] <= 1'b1;
                            end

                            if (victim_way == 2'd0) begin
                                lru01[miss_index] <= 1'b1;
                                lru02[miss_index] <= 1'b1;
                                lru03[miss_index] <= 1'b1;
                            end else if (victim_way == 2'd1) begin
                                lru01[miss_index] <= 1'b0;
                                lru12[miss_index] <= 1'b1;
                                lru13[miss_index] <= 1'b1;
                            end else if (victim_way == 2'd2) begin
                                lru02[miss_index] <= 1'b0;
                                lru12[miss_index] <= 1'b0;
                                lru23[miss_index] <= 1'b1;
                            end else begin
                                lru03[miss_index] <= 1'b0;
                                lru13[miss_index] <= 1'b0;
                                lru23[miss_index] <= 1'b0;
                            end

                            if (write_miss_propogation) begin
                                if (victim_way == 0)
                                    datas0[miss_index][miss_offset] <= store_miss_word;
                                else if (victim_way == 1)
                                    datas1[miss_index][miss_offset] <= store_miss_word;
                                else if (victim_way == 2)
                                    datas2[miss_index][miss_offset] <= store_miss_word;
                                else if (victim_way == 3)
                                    datas3[miss_index][miss_offset] <= store_miss_word;
                            end
                        end
                    end
                end

                // WRITE_THROUGH: begin
                //     mem_wen_reg   <= 1'b1;
                //     mem_wdata_reg <= miss_wdata_q;
                // end
               
                default:;
            endcase
        end
    end
endmodule


`default_nettype wire