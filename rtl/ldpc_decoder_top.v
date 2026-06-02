// ldpc_decoder_top.v — 16-Stage Frame-Interleaved LDPC Pipeline
//
// All 16 column stages active every clock cycle, each processing a different
// frame.  Control signals (frame_id / phase_select / first_iter) are pipelined
// through shift registers so stage k sees the values that were valid at the
// controller k cycles earlier.
//
// New top-level ports vs. single-frame version:
//   qv_frame [3:0]  — frame select for LLR loading
//   rd_frame [3:0]  — select which frame's C_hat to read

module ldpc_decoder_top #(
    parameter Q           = 42,
    parameter NUM_COLS    = 16,
    parameter NUM_LAYERS  = 3,
    parameter NUM_FRAMES  = 16,
    parameter LV_WIDTH    = 7,
    parameter MSG_W       = 10,
    parameter MAX_ITER    = 10,
    parameter [5:0] BP    = 6'd63
)(
    input  wire                  clk,
    input  wire                  rst,
    input  wire                  start,

    // LLR write
    input  wire                  qv_we,
    input  wire [3:0]            qv_col,
    input  wire [3:0]            qv_frame,
    input  wire [5:0]            qv_node,
    input  wire signed [4:0]     qv_data,

    // Hard-decision read
    input  wire [3:0]            rd_frame,
    output wire [NUM_COLS*Q-1:0] C_hat,

    output wire                  decoding,
    output wire                  done
);

    localparam BUS_W = Q * NUM_LAYERS * MSG_W;
    localparam [5:0] BPAS = BP;

    // SEED
    localparam [MSG_W-1:0] SEED_ONE = {1'b0, 1'b0, 4'hF, 4'hF};
    localparam [BUS_W-1:0] SEED_BUS = {(Q*NUM_LAYERS){SEED_ONE}};

    // Differential shifts (CN pass)
    localparam [5:0] DSH_c0_l0=6'd29;  localparam [5:0] DSH_c0_l1=6'd37;  localparam [5:0] DSH_c0_l2=6'd25;
    localparam [5:0] DSH_c1_l0=6'd1;   localparam [5:0] DSH_c1_l1=6'd36;  localparam [5:0] DSH_c1_l2=6'd39;
    localparam [5:0] DSH_c2_l0=6'd12;  localparam [5:0] DSH_c2_l1=6'd29;  localparam [5:0] DSH_c2_l2=6'd24;
    localparam [5:0] DSH_c3_l0=6'd8;   localparam [5:0] DSH_c3_l1=6'd5;   localparam [5:0] DSH_c3_l2=6'd30;
    localparam [5:0] DSH_c4_l0=6'd25;  localparam [5:0] DSH_c4_l1=6'd30;  localparam [5:0] DSH_c4_l2=6'd39;
    localparam [5:0] DSH_c5_l0=6'd31;  localparam [5:0] DSH_c5_l1=6'd10;  localparam [5:0] DSH_c5_l2=6'd14;
    localparam [5:0] DSH_c6_l0=6'd37;  localparam [5:0] DSH_c6_l1=6'd27;  localparam [5:0] DSH_c6_l2=6'd11;
    localparam [5:0] DSH_c7_l0=6'd29;  localparam [5:0] DSH_c7_l1=6'd14;  localparam [5:0] DSH_c7_l2=6'd1;
    localparam [5:0] DSH_c8_l0=6'd23;  localparam [5:0] DSH_c8_l1=6'd12;  localparam [5:0] DSH_c8_l2=6'd31;
    localparam [5:0] DSH_c9_l0=6'd1;   localparam [5:0] DSH_c9_l1=6'd19;  localparam [5:0] DSH_c9_l2=6'd40;
    localparam [5:0] DSH_c10_l0=6'd34; localparam [5:0] DSH_c10_l1=6'd3;  localparam [5:0] DSH_c10_l2=6'd12;
    localparam [5:0] DSH_c11_l0=6'd7;  localparam [5:0] DSH_c11_l1=6'd17; localparam [5:0] DSH_c11_l2=6'd4;
    localparam [5:0] DSH_c12_l0=6'd39; localparam [5:0] DSH_c12_l1=6'd23; localparam [5:0] DSH_c12_l2=6'd37;
    localparam [5:0] DSH_c13_l0=6'd41; localparam [5:0] DSH_c13_l1=6'd32; localparam [5:0] DSH_c13_l2=6'd9;
    localparam [5:0] DSH_c14_l0=BP;    localparam [5:0] DSH_c14_l1=6'd13; localparam [5:0] DSH_c14_l2=6'd0;
    localparam [5:0] DSH_c15_l0=BP;    localparam [5:0] DSH_c15_l1=BP;    localparam [5:0] DSH_c15_l2=6'd2;

    // VN wraparound shifts
    localparam [5:0] VN_WRAP_l0=6'd6, VN_WRAP_l1=6'd24, VN_WRAP_l2=6'd1;

    // LAYER_ACTIVE
    localparam [2:0] LA_ALL=3'b111, LA_C14=3'b110, LA_C15=3'b100;

    // =========================================================================
    // Controller
    // =========================================================================
    wire [3:0] ctrl_frame;
    wire       ctrl_phase, ctrl_first;
    wire       decoding_w, done_w;

    decoder_controller #(
        .NUM_COLS(NUM_COLS), .NUM_FRAMES(NUM_FRAMES), .MAX_ITER(MAX_ITER)
    ) CTRL (
        .clk(clk), .rst(rst), .start(start),
        .frame_id(ctrl_frame), .phase_select(ctrl_phase), .first_iter(ctrl_first),
        .decoding(decoding_w), .done(done_w)
    );
    assign decoding = decoding_w;
    assign done     = done_w;

    // =========================================================================
    // 16-STAGE PIPELINE CONTROL SHIFT REGISTERS
    //
    // stage 0 : direct from controller
    // stage k : controller value delayed k cycles (stored in shift reg slot k-1)
    //
    // Invariant: at any time T, stage k sees controller values from time T-k.
    //   stg_frame[k] = ctrl_frame delayed k cycles  = (T_cnt - k)[3:0]
    //   stg_phase[k] = ctrl_phase delayed k cycles  = (T_cnt - k)[4]
    //   stg_first[k] = ctrl_first delayed k cycles  = ((T_cnt-k) < 32)
    // =========================================================================
    reg [3:0] frame_sr [1:NUM_COLS-1];
    reg       phase_sr [1:NUM_COLS-1];
    reg       first_sr [1:NUM_COLS-1];
    reg       dec_sr   [1:NUM_COLS-1];  // pipeline the decoding enable too

    integer pi;
    always @(posedge clk or posedge rst) begin : PIPE_SR
        if (rst) begin
            for (pi = 1; pi < NUM_COLS; pi = pi+1) begin
                frame_sr[pi] <= 4'd0;
                phase_sr[pi] <= 1'b0;
                first_sr[pi] <= 1'b0;
                dec_sr  [pi] <= 1'b0;
            end
        end else begin
            frame_sr[1] <= ctrl_frame;
            phase_sr[1] <= ctrl_phase;
            first_sr[1] <= ctrl_first;
            dec_sr  [1] <= decoding_w;
            for (pi = 2; pi < NUM_COLS; pi = pi+1) begin
                frame_sr[pi] <= frame_sr[pi-1];
                phase_sr[pi] <= phase_sr[pi-1];
                first_sr[pi] <= first_sr[pi-1];
                dec_sr  [pi] <= dec_sr  [pi-1];
            end
        end
    end

    wire [3:0] stg_frame [0:NUM_COLS-1];
    wire       stg_phase [0:NUM_COLS-1];
    wire       stg_first [0:NUM_COLS-1];
    wire       stg_en    [0:NUM_COLS-1];

    assign stg_frame[0] = ctrl_frame;
    assign stg_phase[0] = ctrl_phase;
    assign stg_first[0] = ctrl_first;
    assign stg_en   [0] = decoding_w;

    genvar sk;
    generate
        for (sk = 1; sk < NUM_COLS; sk = sk+1) begin : GEN_STG
            assign stg_frame[sk] = frame_sr[sk];
            assign stg_phase[sk] = phase_sr[sk];
            assign stg_first[sk] = first_sr[sk];
            assign stg_en   [sk] = dec_sr  [sk];
        end
    endgenerate

    // =========================================================================
    // Permutation networks and column slices
    // =========================================================================
    wire [BUS_W-1:0] perm_out    [0:NUM_COLS-1];
    wire [BUS_W-1:0] perm_out_vn;
    wire [BUS_W-1:0] col_out     [0:NUM_COLS-1];

    // col0 input mux
    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(DSH_c0_l0),.SHIFT_1(DSH_c0_l1),.SHIFT_2(DSH_c0_l2))
        PERM_0    (.msg_in(SEED_BUS),    .msg_out(perm_out[0]));

    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(VN_WRAP_l0),.SHIFT_1(VN_WRAP_l1),.SHIFT_2(VN_WRAP_l2))
        PERM_0_VN (.msg_in(col_out[15]),.msg_out(perm_out_vn));

    wire [BUS_W-1:0] col0_msg_in = stg_phase[0] ? perm_out_vn : perm_out[0];

    // Macro-like task for column slice connections
    // Column 0
    column_slice #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.NUM_FRAMES(NUM_FRAMES),
                   .LV_WIDTH(LV_WIDTH),.MSG_W(MSG_W),.LAYER_ACTIVE(LA_ALL)) CS_0 (
        .clk(clk),.rst(rst),
        .node_en(stg_en[0]),.phase_select(stg_phase[0]),
        .first_iter(stg_first[0]),.frame_id(stg_frame[0]),.rd_frame_id(rd_frame),
        .qv_we(qv_we&&(qv_col==4'd0)),.qv_wframe(qv_frame),
        .qv_waddr(qv_node),.qv_wdata(qv_data),
        .layer_msg_in(col0_msg_in),.layer_msg_out(col_out[0]),.C_hat(C_hat[0*Q+:Q]));

    // Column 1
    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(DSH_c1_l0),.SHIFT_1(DSH_c1_l1),.SHIFT_2(DSH_c1_l2))
        PERM_1(.msg_in(col_out[0]),.msg_out(perm_out[1]));
    column_slice #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.NUM_FRAMES(NUM_FRAMES),
                   .LV_WIDTH(LV_WIDTH),.MSG_W(MSG_W),.LAYER_ACTIVE(LA_ALL)) CS_1 (
        .clk(clk),.rst(rst),
        .node_en(stg_en[1]),.phase_select(stg_phase[1]),
        .first_iter(stg_first[1]),.frame_id(stg_frame[1]),.rd_frame_id(rd_frame),
        .qv_we(qv_we&&(qv_col==4'd1)),.qv_wframe(qv_frame),
        .qv_waddr(qv_node),.qv_wdata(qv_data),
        .layer_msg_in(perm_out[1]),.layer_msg_out(col_out[1]),.C_hat(C_hat[1*Q+:Q]));

    // Column 2
    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(DSH_c2_l0),.SHIFT_1(DSH_c2_l1),.SHIFT_2(DSH_c2_l2))
        PERM_2(.msg_in(col_out[1]),.msg_out(perm_out[2]));
    column_slice #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.NUM_FRAMES(NUM_FRAMES),
                   .LV_WIDTH(LV_WIDTH),.MSG_W(MSG_W),.LAYER_ACTIVE(LA_ALL)) CS_2 (
        .clk(clk),.rst(rst),
        .node_en(stg_en[2]),.phase_select(stg_phase[2]),
        .first_iter(stg_first[2]),.frame_id(stg_frame[2]),.rd_frame_id(rd_frame),
        .qv_we(qv_we&&(qv_col==4'd2)),.qv_wframe(qv_frame),
        .qv_waddr(qv_node),.qv_wdata(qv_data),
        .layer_msg_in(perm_out[2]),.layer_msg_out(col_out[2]),.C_hat(C_hat[2*Q+:Q]));

    // Column 3
    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(DSH_c3_l0),.SHIFT_1(DSH_c3_l1),.SHIFT_2(DSH_c3_l2))
        PERM_3(.msg_in(col_out[2]),.msg_out(perm_out[3]));
    column_slice #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.NUM_FRAMES(NUM_FRAMES),
                   .LV_WIDTH(LV_WIDTH),.MSG_W(MSG_W),.LAYER_ACTIVE(LA_ALL)) CS_3 (
        .clk(clk),.rst(rst),
        .node_en(stg_en[3]),.phase_select(stg_phase[3]),
        .first_iter(stg_first[3]),.frame_id(stg_frame[3]),.rd_frame_id(rd_frame),
        .qv_we(qv_we&&(qv_col==4'd3)),.qv_wframe(qv_frame),
        .qv_waddr(qv_node),.qv_wdata(qv_data),
        .layer_msg_in(perm_out[3]),.layer_msg_out(col_out[3]),.C_hat(C_hat[3*Q+:Q]));

    // Column 4
    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(DSH_c4_l0),.SHIFT_1(DSH_c4_l1),.SHIFT_2(DSH_c4_l2))
        PERM_4(.msg_in(col_out[3]),.msg_out(perm_out[4]));
    column_slice #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.NUM_FRAMES(NUM_FRAMES),
                   .LV_WIDTH(LV_WIDTH),.MSG_W(MSG_W),.LAYER_ACTIVE(LA_ALL)) CS_4 (
        .clk(clk),.rst(rst),
        .node_en(stg_en[4]),.phase_select(stg_phase[4]),
        .first_iter(stg_first[4]),.frame_id(stg_frame[4]),.rd_frame_id(rd_frame),
        .qv_we(qv_we&&(qv_col==4'd4)),.qv_wframe(qv_frame),
        .qv_waddr(qv_node),.qv_wdata(qv_data),
        .layer_msg_in(perm_out[4]),.layer_msg_out(col_out[4]),.C_hat(C_hat[4*Q+:Q]));

    // Column 5
    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(DSH_c5_l0),.SHIFT_1(DSH_c5_l1),.SHIFT_2(DSH_c5_l2))
        PERM_5(.msg_in(col_out[4]),.msg_out(perm_out[5]));
    column_slice #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.NUM_FRAMES(NUM_FRAMES),
                   .LV_WIDTH(LV_WIDTH),.MSG_W(MSG_W),.LAYER_ACTIVE(LA_ALL)) CS_5 (
        .clk(clk),.rst(rst),
        .node_en(stg_en[5]),.phase_select(stg_phase[5]),
        .first_iter(stg_first[5]),.frame_id(stg_frame[5]),.rd_frame_id(rd_frame),
        .qv_we(qv_we&&(qv_col==4'd5)),.qv_wframe(qv_frame),
        .qv_waddr(qv_node),.qv_wdata(qv_data),
        .layer_msg_in(perm_out[5]),.layer_msg_out(col_out[5]),.C_hat(C_hat[5*Q+:Q]));

    // Column 6
    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(DSH_c6_l0),.SHIFT_1(DSH_c6_l1),.SHIFT_2(DSH_c6_l2))
        PERM_6(.msg_in(col_out[5]),.msg_out(perm_out[6]));
    column_slice #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.NUM_FRAMES(NUM_FRAMES),
                   .LV_WIDTH(LV_WIDTH),.MSG_W(MSG_W),.LAYER_ACTIVE(LA_ALL)) CS_6 (
        .clk(clk),.rst(rst),
        .node_en(stg_en[6]),.phase_select(stg_phase[6]),
        .first_iter(stg_first[6]),.frame_id(stg_frame[6]),.rd_frame_id(rd_frame),
        .qv_we(qv_we&&(qv_col==4'd6)),.qv_wframe(qv_frame),
        .qv_waddr(qv_node),.qv_wdata(qv_data),
        .layer_msg_in(perm_out[6]),.layer_msg_out(col_out[6]),.C_hat(C_hat[6*Q+:Q]));

    // Column 7
    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(DSH_c7_l0),.SHIFT_1(DSH_c7_l1),.SHIFT_2(DSH_c7_l2))
        PERM_7(.msg_in(col_out[6]),.msg_out(perm_out[7]));
    column_slice #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.NUM_FRAMES(NUM_FRAMES),
                   .LV_WIDTH(LV_WIDTH),.MSG_W(MSG_W),.LAYER_ACTIVE(LA_ALL)) CS_7 (
        .clk(clk),.rst(rst),
        .node_en(stg_en[7]),.phase_select(stg_phase[7]),
        .first_iter(stg_first[7]),.frame_id(stg_frame[7]),.rd_frame_id(rd_frame),
        .qv_we(qv_we&&(qv_col==4'd7)),.qv_wframe(qv_frame),
        .qv_waddr(qv_node),.qv_wdata(qv_data),
        .layer_msg_in(perm_out[7]),.layer_msg_out(col_out[7]),.C_hat(C_hat[7*Q+:Q]));

    // Column 8
    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(DSH_c8_l0),.SHIFT_1(DSH_c8_l1),.SHIFT_2(DSH_c8_l2))
        PERM_8(.msg_in(col_out[7]),.msg_out(perm_out[8]));
    column_slice #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.NUM_FRAMES(NUM_FRAMES),
                   .LV_WIDTH(LV_WIDTH),.MSG_W(MSG_W),.LAYER_ACTIVE(LA_ALL)) CS_8 (
        .clk(clk),.rst(rst),
        .node_en(stg_en[8]),.phase_select(stg_phase[8]),
        .first_iter(stg_first[8]),.frame_id(stg_frame[8]),.rd_frame_id(rd_frame),
        .qv_we(qv_we&&(qv_col==4'd8)),.qv_wframe(qv_frame),
        .qv_waddr(qv_node),.qv_wdata(qv_data),
        .layer_msg_in(perm_out[8]),.layer_msg_out(col_out[8]),.C_hat(C_hat[8*Q+:Q]));

    // Column 9
    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(DSH_c9_l0),.SHIFT_1(DSH_c9_l1),.SHIFT_2(DSH_c9_l2))
        PERM_9(.msg_in(col_out[8]),.msg_out(perm_out[9]));
    column_slice #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.NUM_FRAMES(NUM_FRAMES),
                   .LV_WIDTH(LV_WIDTH),.MSG_W(MSG_W),.LAYER_ACTIVE(LA_ALL)) CS_9 (
        .clk(clk),.rst(rst),
        .node_en(stg_en[9]),.phase_select(stg_phase[9]),
        .first_iter(stg_first[9]),.frame_id(stg_frame[9]),.rd_frame_id(rd_frame),
        .qv_we(qv_we&&(qv_col==4'd9)),.qv_wframe(qv_frame),
        .qv_waddr(qv_node),.qv_wdata(qv_data),
        .layer_msg_in(perm_out[9]),.layer_msg_out(col_out[9]),.C_hat(C_hat[9*Q+:Q]));

    // Column 10
    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(DSH_c10_l0),.SHIFT_1(DSH_c10_l1),.SHIFT_2(DSH_c10_l2))
        PERM_10(.msg_in(col_out[9]),.msg_out(perm_out[10]));
    column_slice #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.NUM_FRAMES(NUM_FRAMES),
                   .LV_WIDTH(LV_WIDTH),.MSG_W(MSG_W),.LAYER_ACTIVE(LA_ALL)) CS_10 (
        .clk(clk),.rst(rst),
        .node_en(stg_en[10]),.phase_select(stg_phase[10]),
        .first_iter(stg_first[10]),.frame_id(stg_frame[10]),.rd_frame_id(rd_frame),
        .qv_we(qv_we&&(qv_col==4'd10)),.qv_wframe(qv_frame),
        .qv_waddr(qv_node),.qv_wdata(qv_data),
        .layer_msg_in(perm_out[10]),.layer_msg_out(col_out[10]),.C_hat(C_hat[10*Q+:Q]));

    // Column 11
    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(DSH_c11_l0),.SHIFT_1(DSH_c11_l1),.SHIFT_2(DSH_c11_l2))
        PERM_11(.msg_in(col_out[10]),.msg_out(perm_out[11]));
    column_slice #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.NUM_FRAMES(NUM_FRAMES),
                   .LV_WIDTH(LV_WIDTH),.MSG_W(MSG_W),.LAYER_ACTIVE(LA_ALL)) CS_11 (
        .clk(clk),.rst(rst),
        .node_en(stg_en[11]),.phase_select(stg_phase[11]),
        .first_iter(stg_first[11]),.frame_id(stg_frame[11]),.rd_frame_id(rd_frame),
        .qv_we(qv_we&&(qv_col==4'd11)),.qv_wframe(qv_frame),
        .qv_waddr(qv_node),.qv_wdata(qv_data),
        .layer_msg_in(perm_out[11]),.layer_msg_out(col_out[11]),.C_hat(C_hat[11*Q+:Q]));

    // Column 12
    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(DSH_c12_l0),.SHIFT_1(DSH_c12_l1),.SHIFT_2(DSH_c12_l2))
        PERM_12(.msg_in(col_out[11]),.msg_out(perm_out[12]));
    column_slice #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.NUM_FRAMES(NUM_FRAMES),
                   .LV_WIDTH(LV_WIDTH),.MSG_W(MSG_W),.LAYER_ACTIVE(LA_ALL)) CS_12 (
        .clk(clk),.rst(rst),
        .node_en(stg_en[12]),.phase_select(stg_phase[12]),
        .first_iter(stg_first[12]),.frame_id(stg_frame[12]),.rd_frame_id(rd_frame),
        .qv_we(qv_we&&(qv_col==4'd12)),.qv_wframe(qv_frame),
        .qv_waddr(qv_node),.qv_wdata(qv_data),
        .layer_msg_in(perm_out[12]),.layer_msg_out(col_out[12]),.C_hat(C_hat[12*Q+:Q]));

    // Column 13
    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(DSH_c13_l0),.SHIFT_1(DSH_c13_l1),.SHIFT_2(DSH_c13_l2))
        PERM_13(.msg_in(col_out[12]),.msg_out(perm_out[13]));
    column_slice #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.NUM_FRAMES(NUM_FRAMES),
                   .LV_WIDTH(LV_WIDTH),.MSG_W(MSG_W),.LAYER_ACTIVE(LA_ALL)) CS_13 (
        .clk(clk),.rst(rst),
        .node_en(stg_en[13]),.phase_select(stg_phase[13]),
        .first_iter(stg_first[13]),.frame_id(stg_frame[13]),.rd_frame_id(rd_frame),
        .qv_we(qv_we&&(qv_col==4'd13)),.qv_wframe(qv_frame),
        .qv_waddr(qv_node),.qv_wdata(qv_data),
        .layer_msg_in(perm_out[13]),.layer_msg_out(col_out[13]),.C_hat(C_hat[13*Q+:Q]));

    // Column 14
    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(DSH_c14_l0),.SHIFT_1(DSH_c14_l1),.SHIFT_2(DSH_c14_l2))
        PERM_14(.msg_in(col_out[13]),.msg_out(perm_out[14]));
    column_slice #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.NUM_FRAMES(NUM_FRAMES),
                   .LV_WIDTH(LV_WIDTH),.MSG_W(MSG_W),.LAYER_ACTIVE(LA_C14)) CS_14 (
        .clk(clk),.rst(rst),
        .node_en(stg_en[14]),.phase_select(stg_phase[14]),
        .first_iter(stg_first[14]),.frame_id(stg_frame[14]),.rd_frame_id(rd_frame),
        .qv_we(qv_we&&(qv_col==4'd14)),.qv_wframe(qv_frame),
        .qv_waddr(qv_node),.qv_wdata(qv_data),
        .layer_msg_in(perm_out[14]),.layer_msg_out(col_out[14]),.C_hat(C_hat[14*Q+:Q]));

    // Column 15
    permutation_network #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.MSG_W(MSG_W),.BYPASS_SENTINEL(BPAS),
        .SHIFT_0(DSH_c15_l0),.SHIFT_1(DSH_c15_l1),.SHIFT_2(DSH_c15_l2))
        PERM_15(.msg_in(col_out[14]),.msg_out(perm_out[15]));
    column_slice #(.Q(Q),.NUM_LAYERS(NUM_LAYERS),.NUM_FRAMES(NUM_FRAMES),
                   .LV_WIDTH(LV_WIDTH),.MSG_W(MSG_W),.LAYER_ACTIVE(LA_C15)) CS_15 (
        .clk(clk),.rst(rst),
        .node_en(stg_en[15]),.phase_select(stg_phase[15]),
        .first_iter(stg_first[15]),.frame_id(stg_frame[15]),.rd_frame_id(rd_frame),
        .qv_we(qv_we&&(qv_col==4'd15)),.qv_wframe(qv_frame),
        .qv_waddr(qv_node),.qv_wdata(qv_data),
        .layer_msg_in(perm_out[15]),.layer_msg_out(col_out[15]),.C_hat(C_hat[15*Q+:Q]));

endmodule
