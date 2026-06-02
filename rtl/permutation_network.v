module permutation_network #(
    parameter       Q                = 42,
    parameter       NUM_LAYERS       = 3,
    parameter       MSG_W            = 10,
    parameter [5:0] BYPASS_SENTINEL  = 6'd63,
    parameter [5:0] SHIFT_0          = 6'd63,
    parameter [5:0] SHIFT_1          = 6'd63,
    parameter [5:0] SHIFT_2          = 6'd63
)(
    input  wire [(Q * NUM_LAYERS * MSG_W)-1:0] msg_in,
    output wire [(Q * NUM_LAYERS * MSG_W)-1:0] msg_out
);

    genvar l, n;
    generate
        for (l = 0; l < NUM_LAYERS; l = l + 1) begin : GEN_L

            // Pick the shift for this layer (must use individual params, not array)
            wire [5:0] shift_l;
            assign shift_l = (l == 0) ? SHIFT_0 :
                             (l == 1) ? SHIFT_1 :
                                        SHIFT_2;

            // Active when shift != BYPASS_SENTINEL
            wire active_l;
            assign active_l = (shift_l != BYPASS_SENTINEL);

            for (n = 0; n < Q; n = n + 1) begin : GEN_N
                // Source node: (n - shift_l + Q) mod Q
                // n is a genvar (integer 0..Q-1).
                // Represent it as a 6-bit localparam for clean wire arithmetic.
                // The modulo is handled by a single conditional:
                //   n >= shift_l  →  src = n - shift_l          (no wrap)
                //   n <  shift_l  →  src = n + Q - shift_l      (wrap)
                localparam [5:0] N_VAL = n;   // elaboration-time constant

                wire [5:0] src;
                assign src = active_l
                           ? ( (N_VAL >= shift_l)
                               ? (N_VAL - shift_l)
                               : (N_VAL + 6'd42 - shift_l) )
                           : N_VAL;

                // Wire: msg_out[node=n, layer=l]  ←  msg_in[node=src, layer=l]
                assign msg_out[(n   * NUM_LAYERS + l) * MSG_W +: MSG_W]
                     = msg_in [(src * NUM_LAYERS + l) * MSG_W +: MSG_W];
            end
        end
    endgenerate

endmodule
