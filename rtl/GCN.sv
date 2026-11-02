`timescale 1ns/1ps

module GCN
#(
    parameter FEATURE_COLS = 96,
    parameter WEIGHT_ROWS = 96,
    parameter FEATURE_ROWS = 6,
    parameter WEIGHT_COLS = 3,
    parameter FEATURE_WIDTH = 5,
    parameter WEIGHT_WIDTH = 5,
    parameter DOT_PROD_WIDTH = 16,
    parameter ADDRESS_WIDTH = 13,
    parameter COUNTER_WEIGHT_WIDTH = $clog2(WEIGHT_COLS),
    parameter COUNTER_FEATURE_WIDTH = $clog2(FEATURE_ROWS),
    parameter MAX_ADDRESS_WIDTH = 2,
    parameter NUM_OF_NODES = 6,             
    parameter COO_NUM_OF_COLS = 6,          
    parameter COO_NUM_OF_ROWS = 2,          
    parameter COO_BW = $clog2(COO_NUM_OF_COLS)  
)
(
    input logic clk,    
    input logic reset,  
    input logic start,
    input logic [WEIGHT_WIDTH-1:0] data_in [0:WEIGHT_ROWS-1], 
    input logic [COO_BW - 1:0] coo_in [0:1], 

    output logic [COO_BW - 1:0] coo_address, 
    output logic [ADDRESS_WIDTH-1:0] read_address, 
    output logic enable_read, 
    output logic done, 
    output logic [MAX_ADDRESS_WIDTH - 1:0] max_addi_answer [0:FEATURE_ROWS - 1] 
);

    // ==========================================
    //       INTERNAL SIGNALS
    // ==========================================

    // --- STAGE 1: TRANSFORMATION SIGNALS ---
    logic trans_done;
    logic en_read_trans, en_write_fm_wm, en_scratch, en_w_cnt, en_f_cnt, read_mode;
    // NOTE: 'enable_write' is an output from your FSM but unused in the logic, so we leave it open.
    
    logic [COUNTER_WEIGHT_WIDTH-1:0] w_count;
    logic [COUNTER_FEATURE_WIDTH-1:0] f_count;
    logic [WEIGHT_WIDTH-1:0] scratch_out [0:WEIGHT_ROWS-1];
    logic [DOT_PROD_WIDTH-1:0] mult_result;
    
    // Memory 1 (Z Matrix) Signals
    logic [DOT_PROD_WIDTH-1:0] fm_wm_mem_out [0:WEIGHT_COLS-1]; 
    logic [COUNTER_FEATURE_WIDTH-1:0] fm_wm_read_addr;

    // --- STAGE 2: AGGREGATION SIGNALS ---
    logic aggr_done;
    logic write_adj_en;
    logic [COUNTER_FEATURE_WIDTH-1:0] adj_write_row, adj_read_row;
    logic [DOT_PROD_WIDTH-1:0] adj_data_in [0:WEIGHT_COLS-1];
    logic [DOT_PROD_WIDTH-1:0] adj_data_out [0:WEIGHT_COLS-1];
    
    // FSM States
    typedef enum logic [3:0] {
        AGGR_IDLE,
        AGGR_START,
        AGGR_FETCH_EDGE,
        AGGR_READ_U,      
        AGGR_ACC_V,       
        AGGR_READ_V,      
        AGGR_ACC_U,       
        AGGR_DONE
    } aggr_state_t;
    
    aggr_state_t aggr_state, aggr_next_state;
    logic [COO_BW:0] edge_cnt; 
    logic [DOT_PROD_WIDTH-1:0] vec_u [0:WEIGHT_COLS-1];
    logic [DOT_PROD_WIDTH-1:0] vec_v [0:WEIGHT_COLS-1];

    // --- STAGE 3: ARGMAX SIGNALS ---
    logic [MAX_ADDRESS_WIDTH - 1:0] answers_temp [0:FEATURE_ROWS - 1];
    logic argmax_done;
    logic [COUNTER_FEATURE_WIDTH:0] output_cnt; 

    // ==========================================
    //       STAGE 1: TRANSFORMATION
    // ==========================================

    // INSTANTIATION MATCHING YOUR PROVIDED FILE EXACTLY
    Transformation_FSM #(
        .FEATURE_ROWS(FEATURE_ROWS),
        .WEIGHT_COLS(WEIGHT_COLS),
        .COUNTER_WEIGHT_WIDTH(COUNTER_WEIGHT_WIDTH),
        .COUNTER_FEATURE_WIDTH(COUNTER_FEATURE_WIDTH)
    ) fsm_inst (
        .clk(clk),
        .reset(reset),
        .weight_count(w_count),
        .feature_count(f_count),
        .start(start),
        
        .enable_write_fm_wm_prod(en_write_fm_wm),
        .enable_read(en_read_trans),
        .enable_write(), // This port is present in your file but assigned 0 everywhere, so we leave it unconnected.
        .enable_scratch_pad(en_scratch),
        .enable_weight_counter(en_w_cnt),
        .enable_feature_counter(en_f_cnt),
        .read_feature_or_weight(read_mode),
        .done(trans_done)
    );

    // Counters for Stage 1
    always_ff @(posedge clk or posedge reset) begin
        if(reset) begin
            w_count <= 0;
            f_count <= 0;
        end else begin
            if(en_w_cnt) w_count <= w_count + 1;
            if(en_f_cnt) f_count <= (f_count == FEATURE_ROWS-1) ? 0 : f_count + 1;
        end
    end

    // Address Logic
    assign read_address = (read_mode) ? (13'(f_count) + 13'd512) : 13'(w_count);
    assign enable_read = en_read_trans; 

    Scratch_Pad #( .WEIGHT_ROWS(WEIGHT_ROWS), .WEIGHT_WIDTH(WEIGHT_WIDTH) ) sp (
        .clk(clk), .reset(reset),
        .write_enable(en_scratch),
        .weight_col_in(data_in),   
        .weight_col_out(scratch_out) 
    );

    Vector_Multiplier #( .LEN(FEATURE_COLS), .IN_WIDTH(FEATURE_WIDTH), .OUT_WIDTH(DOT_PROD_WIDTH) ) mult (
        .vec_a(data_in),       
        .vec_b(scratch_out),   
        .dot_product(mult_result)      
    );

    Matrix_FM_WM_Memory #(
        .FEATURE_ROWS(FEATURE_ROWS), .WEIGHT_COLS(WEIGHT_COLS), .DOT_PROD_WIDTH(DOT_PROD_WIDTH)
    ) z_mem (
        .clk(clk), .rst(reset),
        .write_row(f_count), .write_col(w_count),
        .wr_en(en_write_fm_wm),
        .fm_wm_in(mult_result),
        .read_row(fm_wm_read_addr), 
        .fm_wm_row_out(fm_wm_mem_out)
    );

    // ==========================================
    //       STAGE 2: AGGREGATION FSM
    // ==========================================

    Matrix_FM_WM_ADJ_Memory #(
        .FEATURE_ROWS(FEATURE_ROWS), .WEIGHT_COLS(WEIGHT_COLS), .DOT_PROD_WIDTH(DOT_PROD_WIDTH)
    ) final_mem (
        .clk(clk), .rst(reset),
        .write_row(adj_write_row), 
        .read_row(adj_read_row), 
        .wr_en(write_adj_en),
        .fm_wm_adj_row_in(adj_data_in),
        .fm_wm_adj_out(adj_data_out) 
    );

    always_ff @(posedge clk or posedge reset) begin
        if(reset) aggr_state <= AGGR_IDLE;
        else aggr_state <= aggr_next_state;
    end

    always_ff @(posedge clk or posedge reset) begin
        if(reset) begin
            edge_cnt <= 0;
            for(int i=0; i<WEIGHT_COLS; i++) begin vec_u[i] <= 0; vec_v[i] <= 0; end
        end else begin
            if (aggr_state == AGGR_ACC_U) edge_cnt <= edge_cnt + 1;
            if (aggr_state == AGGR_READ_U) vec_u <= fm_wm_mem_out;
            if (aggr_state == AGGR_READ_V) vec_v <= fm_wm_mem_out;
        end
    end

    // Combinational FSM Logic - LATCH PREVENTION STRATEGY
    always_comb begin
        // 1. DEFAULT ASSIGNMENTS
        // Assigning every output a value here ensures NO LATCHES are created
        // if a case branch is missing an assignment.
        aggr_next_state = aggr_state;
        write_adj_en = 0;
        coo_address = 0;
        fm_wm_read_addr = 0; 
        adj_read_row = 0;
        adj_write_row = 0;
        aggr_done = 0;
        for(int k=0; k<WEIGHT_COLS; k++) adj_data_in[k] = 0;

        // 2. STATE LOGIC
        unique case(aggr_state)
            AGGR_IDLE: begin
                if(trans_done) aggr_next_state = AGGR_START;
            end
            
            AGGR_START: aggr_next_state = AGGR_FETCH_EDGE;

            AGGR_FETCH_EDGE: begin
                if(edge_cnt >= COO_NUM_OF_COLS) aggr_next_state = AGGR_DONE;
                else begin
                    coo_address = edge_cnt[COO_BW-1:0];
                    aggr_next_state = AGGR_READ_U;
                end
            end

            AGGR_READ_U: begin
                coo_address = edge_cnt[COO_BW-1:0];
                fm_wm_read_addr = coo_in[0][COUNTER_FEATURE_WIDTH-1:0] - 1; 
                aggr_next_state = AGGR_ACC_V;
            end

            AGGR_ACC_V: begin
                coo_address = edge_cnt[COO_BW-1:0];
                adj_read_row = coo_in[1][COUNTER_FEATURE_WIDTH-1:0] - 1; 
                write_adj_en = 1;
                adj_write_row = coo_in[1][COUNTER_FEATURE_WIDTH-1:0] - 1;
                for(int k=0; k<WEIGHT_COLS; k++) adj_data_in[k] = adj_data_out[k] + vec_u[k];
                aggr_next_state = AGGR_READ_V;
            end

            AGGR_READ_V: begin
                coo_address = edge_cnt[COO_BW-1:0];
                fm_wm_read_addr = coo_in[1][COUNTER_FEATURE_WIDTH-1:0] - 1; 
                aggr_next_state = AGGR_ACC_U;
            end

            AGGR_ACC_U: begin
                coo_address = edge_cnt[COO_BW-1:0];
                adj_read_row = coo_in[0][COUNTER_FEATURE_WIDTH-1:0] - 1; 
                write_adj_en = 1;
                adj_write_row = coo_in[0][COUNTER_FEATURE_WIDTH-1:0] - 1;
                for(int k=0; k<WEIGHT_COLS; k++) adj_data_in[k] = adj_data_out[k] + vec_v[k];
                aggr_next_state = AGGR_FETCH_EDGE;
            end

            AGGR_DONE: begin
                aggr_done = 1;
                // STAGE 3 INTERFACE:
                // Allows Stage 3 to take control of memory reading
                adj_read_row = output_cnt[COUNTER_FEATURE_WIDTH-1:0];
            end
        endcase
    end

    // ==========================================
    //       STAGE 3: ARGMAX & OUTPUT
    // ==========================================

    always_ff @(posedge clk or posedge reset) begin
        if(reset) begin
            output_cnt <= 0;
            argmax_done <= 0;
            for(int i=0; i<FEATURE_ROWS; i++) answers_temp[i] <= 0;
        end else if (aggr_done && !argmax_done) begin
            // Compare and Store Max Index
            if (adj_data_out[0] >= adj_data_out[1] && adj_data_out[0] >= adj_data_out[2])
                answers_temp[COUNTER_FEATURE_WIDTH'(output_cnt)] <= 2'd0;
            else if (adj_data_out[1] >= adj_data_out[0] && adj_data_out[1] >= adj_data_out[2])
                answers_temp[COUNTER_FEATURE_WIDTH'(output_cnt)] <= 2'd1;
            else
                answers_temp[COUNTER_FEATURE_WIDTH'(output_cnt)] <= 2'd2;

            // Increment or Finish
            if (output_cnt == FEATURE_ROWS - 1) 
                argmax_done <= 1;
            else 
                output_cnt <= output_cnt + 1;
        end
    end

    assign max_addi_answer = answers_temp;
    assign done = argmax_done;

endmodule

