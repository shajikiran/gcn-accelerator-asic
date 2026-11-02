module Vector_Multiplier
#(
    parameter LEN = 96,
    parameter IN_WIDTH = 5,
    parameter OUT_WIDTH = 16
)
(
    input logic [IN_WIDTH-1:0] vec_a [0:LEN-1], // Feature Row
    input logic [IN_WIDTH-1:0] vec_b [0:LEN-1], // Weight Col (from Scratchpad)
    output logic [OUT_WIDTH-1:0] dot_product
);

    logic [OUT_WIDTH-1:0] sum;
    
    // Combinational logic to sum (A[0]*B[0] + A[1]*B[1] + ...)
    always_comb begin
        sum = 0;
        for(int i=0; i<LEN; i++) begin
            sum = sum + (vec_a[i] * vec_b[i]);
        end
        dot_product = sum;
    end

endmodule
