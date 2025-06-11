// FILE: aib_avmm_glue_logic.v (Corrected Version)

module aib_avmm_glue_logic #(
    parameter NBR_CHNLS = 24
) (
    // Inputs
    input [NBR_CHNLS-1:0] i_waitreq_ch,
    input [NBR_CHNLS-1:0] i_rdatavld_ch,
    input [32*NBR_CHNLS-1:0] i_rdata_ch,
    
    // Outputs
    output     o_waitreq,
    output     o_rdatavld,
    output reg [31:0] o_rdata // <<< FIX: Changed from 'output' to 'output reg'
);

    // The reduction logic is now scalable and works for any NBR_CHNLS.
    assign o_waitreq  = &i_waitreq_ch;
    assign o_rdatavld = |i_rdatavld_ch;

    // The read data bus is now a scalable priority mux instead of a giant OR-gate.
    // It selects the data from the first channel that has valid data.
    wire [31:0] rdata_lanes [0:NBR_CHNLS-1];
    genvar i;
    generate
        for (i = 0; i < NBR_CHNLS; i = i + 1) begin : gen_rdata_unpack
            assign rdata_lanes[i] = i_rdata_ch[(i+1)*32-1 -: 32];
        end

        integer j;
        // This always block now correctly assigns to a 'reg' type.
        always @* begin
            o_rdata = 32'b0; // Default value
            for (j = 0; j < NBR_CHNLS; j = j + 1) begin
                if (i_rdatavld_ch[j]) begin
                    o_rdata = rdata_lanes[j];
                end
            end
        end
    endgenerate

endmodule