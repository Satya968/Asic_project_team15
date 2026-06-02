module vn_update_block (
    input  wire        phase_select,  // 0 = CN phase, 1 = VN phase
    input  wire        sign_c,        // final sc(T-1) from CN phase
    input  wire [3:0]  min1_c,        // final min1c(T-1)
    input  wire [3:0]  min2_c,        // final min2c(T-1)
    input  wire        lvc_sign,      // sign of this node's Lvc (sign-magnitude)
    input  wire [3:0]  lvc_mag,       // magnitude of this node's Lvc
    output wire signed [4:0] mcv_tc   // mcv in 2's-complement, 0 during CN phase
);

    wire is_min1_live = (lvc_mag <= min1_c);

    // mcv magnitude: use min2 if this node IS the min1 contributor, else min1
    wire [3:0] mcv_mag = is_min1_live ? min2_c : min1_c;

    wire mcv_sign = sign_c ^ lvc_sign;

    wire signed [4:0] mcv_val =
        mcv_sign ? -$signed({1'b0, mcv_mag})
                 :  $signed({1'b0, mcv_mag});

    assign mcv_tc = phase_select ? mcv_val : 5'sd0;

endmodule
