module cn_update_block (
    input  wire        phase_select,  // 0 = CN phase, 1 = VN phase
    input  wire        sign_in,       // sc(t-1) arriving from previous column
    input  wire [3:0]  min1_in,       // min1c(t-1) arriving from previous column
    input  wire [3:0]  min2_in,       // min2c(t-1) arriving from previous column
    input  wire        lvc_sign,      // sign     of locally stored Lvc
    input  wire [3:0]  lvc_mag,       // magnitude of locally stored Lvc
    output wire        sign_out,      // sc(t)    to next column
    output wire [3:0]  min1_out,      // min1c(t) to next column
    output wire [3:0]  min2_out       // min2c(t) to next column
);

    // -------------------------------------------------------------------------
    // CN-phase computations (combinational)
    // -------------------------------------------------------------------------

    // Eq. (3): sc(t) = sc(t-1) XOR sgn(Lvc)
    wire sign_updated = sign_in ^ lvc_sign;

    // Eq. (4): min1c(t) = min( |Lvc|, min1c(t-1) )
    wire [3:0] min1_updated = (lvc_mag < min1_in) ? lvc_mag : min1_in;

    // Eq. (5): min2c(t) = second minimum
    //   Case A: lvc_mag is the new minimum  → old min1 becomes min2
    //   Case B: lvc_mag is between min1 and min2 → lvc_mag becomes min2
    //   Case C: lvc_mag >= min2             → min2 unchanged
    wire [3:0] min2_updated =
        (lvc_mag < min1_in) ? min1_in :          
        (lvc_mag < min2_in) ? lvc_mag : min2_in; 

    // -------------------------------------------------------------------------
    // Output mux: CN-phase result on phase_select=0, passthrough on phase_select=1
    // -------------------------------------------------------------------------
    assign sign_out = phase_select ? sign_in    : sign_updated;
    assign min1_out = phase_select ? min1_in    : min1_updated;
    assign min2_out = phase_select ? min2_in    : min2_updated;

endmodule
