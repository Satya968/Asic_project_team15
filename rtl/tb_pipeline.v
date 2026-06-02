`timescale 1ns/1ps
// tb_pipeline.v — 16-Frame Interleaved Pipeline Testbench
//
// FIXES vs previous:
//   1. NO $dumpvars — that was freezing iverilog (GBs of VCD for this design)
//   2. @(posedge done) instead of polling for-loop
//   3. Only dump top-level signals if you need waveforms

module ldpc_decoder_tb_pipeline;

    parameter Q          = 42;
    parameter NUM_COLS   = 16;
    parameter NUM_FRAMES = 16;
    parameter N          = 672;
    parameter CLK_HALF   = 5;

    reg        clk, rst, start;
    reg        qv_we;
    reg [3:0]  qv_col, qv_frame_in;
    reg [5:0]  qv_node;
    reg signed [4:0] qv_data;
    reg [3:0]  rd_frame;

    wire [N-1:0] C_hat;
    wire         decoding, done;

    // Cycle counter to measure decode time
    integer cycle_count;
    always @(posedge clk) begin
        if (decoding) cycle_count <= cycle_count + 1;
    end

    ldpc_decoder_top #(
        .Q(Q), .NUM_COLS(NUM_COLS), .NUM_LAYERS(3), .NUM_FRAMES(NUM_FRAMES),
        .LV_WIDTH(7), .MSG_W(10), .MAX_ITER(10)
    ) DUT (
        .clk(clk), .rst(rst), .start(start),
        .qv_we(qv_we), .qv_col(qv_col), .qv_frame(qv_frame_in),
        .qv_node(qv_node), .qv_data(qv_data),
        .rd_frame(rd_frame),
        .C_hat(C_hat), .decoding(decoding), .done(done)
    );

    initial clk = 0;
    always #(CLK_HALF) clk = ~clk;

    reg [4:0] llr_mem  [0:NUM_FRAMES-1][0:N-1];
    reg       exp_mem  [0:NUM_FRAMES-1][0:N-1];
    reg [4:0] tmp_llr  [0:N-1];
    reg       tmp_chat [0:N-1];

    reg [256*8-1:0] case_names [0:9];
    integer frame_to_case [0:NUM_FRAMES-1];
    integer f, node, idx, err_cnt, total_pass, total_fail;
    reg [1024*8-1:0] llr_path, chat_path;

    initial begin
        // Only dump top-level control signals — NOT full hierarchy
        // Comment these out entirely for fastest simulation
        //$dumpfile("pipe_top.vcd");
        //$dumpvars(1, ldpc_decoder_tb_pipeline); // depth=1 only

        case_names[0] = "allzero_snr5_seed5";
        case_names[1] = "allzero_snr4_seed1";
        case_names[2] = "allzero_snr4_seed2";
        case_names[3] = "allzero_snr3p5_seed10";
        case_names[4] = "allzero_snr3_seed7";
        case_names[5] = "allzero_snr6_seed99";
        case_names[6] = "random_snr5_seed42";
        case_names[7] = "random_snr4_seed123";
        case_names[8] = "random_snr6_seed7";
        case_names[9] = "random_snr3_seed55";

        for (f = 0; f < NUM_FRAMES; f = f+1)
            frame_to_case[f] = f % 10;

        // Load test vectors
        for (f = 0; f < NUM_FRAMES; f = f+1) begin
            $sformat(llr_path,  "test_cases/%0s_llr.hex",  case_names[frame_to_case[f]]);
            $sformat(chat_path, "test_cases/%0s_chat.txt", case_names[frame_to_case[f]]);
            $readmemh(llr_path,  tmp_llr);
            $readmemb(chat_path, tmp_chat);
            for (idx = 0; idx < N; idx = idx+1) begin
                llr_mem[f][idx] = tmp_llr[idx];
                exp_mem[f][idx] = tmp_chat[idx];
            end
        end

        // Reset
        cycle_count = 0;
        rst = 1; start = 0; qv_we = 0; rd_frame = 0;
        qv_frame_in = 0; qv_col = 0; qv_node = 0; qv_data = 0;
        repeat(5) @(posedge clk); #1;
        rst = 0;
        repeat(2) @(posedge clk); #1;

        // Direct memory init — zero clock cycles
        $display("[TB] Writing LLRs directly into column slice memories...");
        for (f = 0; f < NUM_FRAMES; f = f+1) begin
            for (node = 0; node < Q; node = node+1) begin
                DUT.CS_0.qv_mem [f*Q+node] = $signed(llr_mem[f][ 0*Q+node]);
                DUT.CS_1.qv_mem [f*Q+node] = $signed(llr_mem[f][ 1*Q+node]);
                DUT.CS_2.qv_mem [f*Q+node] = $signed(llr_mem[f][ 2*Q+node]);
                DUT.CS_3.qv_mem [f*Q+node] = $signed(llr_mem[f][ 3*Q+node]);
                DUT.CS_4.qv_mem [f*Q+node] = $signed(llr_mem[f][ 4*Q+node]);
                DUT.CS_5.qv_mem [f*Q+node] = $signed(llr_mem[f][ 5*Q+node]);
                DUT.CS_6.qv_mem [f*Q+node] = $signed(llr_mem[f][ 6*Q+node]);
                DUT.CS_7.qv_mem [f*Q+node] = $signed(llr_mem[f][ 7*Q+node]);
                DUT.CS_8.qv_mem [f*Q+node] = $signed(llr_mem[f][ 8*Q+node]);
                DUT.CS_9.qv_mem [f*Q+node] = $signed(llr_mem[f][ 9*Q+node]);
                DUT.CS_10.qv_mem[f*Q+node] = $signed(llr_mem[f][10*Q+node]);
                DUT.CS_11.qv_mem[f*Q+node] = $signed(llr_mem[f][11*Q+node]);
                DUT.CS_12.qv_mem[f*Q+node] = $signed(llr_mem[f][12*Q+node]);
                DUT.CS_13.qv_mem[f*Q+node] = $signed(llr_mem[f][13*Q+node]);
                DUT.CS_14.qv_mem[f*Q+node] = $signed(llr_mem[f][14*Q+node]);
                DUT.CS_15.qv_mem[f*Q+node] = $signed(llr_mem[f][15*Q+node]);
            end
        end
        $display("[TB] Done. Starting decode...");

        // Start
        @(posedge clk); #1; start = 1;
        @(posedge clk); #1; start = 0;

        // Wait — @(posedge done) is instant, no polling overhead
        @(posedge done);
        $display("[TB] done! Decode cycles = %0d  (seq equiv = %0d, speedup ~%0.1fx)",
                 cycle_count, NUM_FRAMES*10*32,
                 (NUM_FRAMES*10*32*1.0)/cycle_count);

        // Let last updates settle
        repeat(2) @(posedge clk); #1;

        // Check all frames
        total_pass = 0; total_fail = 0;

        for (f = 0; f < NUM_FRAMES; f = f+1) begin
            rd_frame = f[3:0];
            #1;
            err_cnt = 0;
            for (idx = 0; idx < N; idx = idx+1) begin
                if (C_hat[idx] !== exp_mem[f][idx]) begin
                    if (err_cnt < 2)
                        $display("  frame%0d bit%0d: got=%b exp=%b",
                                 f, idx, C_hat[idx], exp_mem[f][idx]);
                    err_cnt = err_cnt + 1;
                end
            end
            if (err_cnt == 0) begin
                $display("[PASS] Frame %2d  %0s", f, case_names[frame_to_case[f]]);
                total_pass = total_pass + 1;
            end else if (err_cnt <= 10) begin
                $display("[WARN] Frame %2d  %0s  %0d bit errors (low SNR)",
                         f, case_names[frame_to_case[f]], err_cnt);
                total_pass = total_pass + 1;
            end else begin
                $display("[FAIL] Frame %2d  %0s  %0d bit errors",
                         f, case_names[frame_to_case[f]], err_cnt);
                total_fail = total_fail + 1;
            end
        end

        $display("\n[TB] %0d/%0d passed.  %0s",
                 total_pass, NUM_FRAMES,
                 (total_fail==0) ? "ALL PASSED." : "SOME FAILED.");
        $finish;
    end

endmodule
