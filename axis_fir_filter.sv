module axis_fir_filter #(
    parameter NUM_TAPS = 2,  
    parameter TAP_WIDTH = 3, 
    parameter DATA_IN_WIDTH = 8, 
    parameter DATA_OUT_WIDTH = TAP_WIDTH + DATA_IN_WIDTH + 8, 
    parameter UP_COUNTER_MAX = 3 + (NUM_TAPS - 1),
    parameter DOWN_COUNTER_MAX = 3 + 2 * (NUM_TAPS - 1),
    parameter UP_COUNTER_WIDTH = $clog (UP_COUNTER_MAX),
    parameter DOWN_COUNTER_WIDTH = $clog (DOWN_COUNTER_MAX)
) (
    input clk,
    input rst_n,
    input signed [DATA_IN_WIDTH-1:0] s_axis_tdata,
    input s_axis_tvalid,
    input s_axis_tlast,
    input m_axis_tready,

    output signed [DATA_OUT_WIDTH-1:0] m_axis_tdata,
    output m_axis_tvalid,
    output m_axis_tlast,
    output s_axis_tready
);
    logic signed [DATA_OUT_WIDTH-1:0] fir_data_out;
    logic signed [DATA_OUT_WIDTH-1:0] data_skid_buffer; 
    logic m_axis_tready_reg;  // Registered ready signal to break long combinational path and improve timing
    logic skid_valid;  // Indicates when there's valid data in the skid buffer
    logic fir_valid;  // Indicates when there's valid data at the FIR filter's output
    logic [UP_COUNTER_WIDTH-1:0] up_counter;  // Used to count the # cycles between a valid FIR input and the first valid FIR output, controlling the assertion of fir_valid
    logic [DOWN_COUNTER_WIDTH-1:0] down_counter;  // Used to count the # cycles between the last FIR input and the first invalid FIR output, controlling the de-assertion of fir_valid
    logic fir_clock_en;

    fir_filter #(NUM_TAPS, TAP_WIDTH, DATA_IN_WIDTH, DATA_OUT_WIDTH) FIR (
        .clk(clk),
        .rst_n(rst_n),
        .clk_en(fir_clock_en),
        .data_in(s_axis_tdata),
        .data_out(fir_data_out)
    )

    always@(posedge clk) begin
        if (!rst_n)
            skid_valid <= 0;
        else if (fir_valid && m_axis_tready_reg && !m_axis_tready)  // We have received data from the FIR, but the downstream is not ready, need to put the FIR data into the skid buffer
            skid_valid <= 1;
        else if (skid_valid && m_axis_tready)  // The data in the skid buffer has been taken away
            skid_valid <= 0;
    end

    always@(posedge clk) begin
        if (!rst_n)
            data_skid_buffer <= 0;
        else if (fir_valid && m_axis_tready_reg && !m_axis_tready)
            data_skid_buffer <= fir_data_out;
        else if (skid_valid && m_axis_tready)
            data_skid_buffer <= 0;
    end

    always@(posedge clk) begin
        if (!rst_n) // reset when the last FIR output has been taken away
            up_counter <= 0;
        else if (m_axis_tready_reg && s_axis_tvalid && up_counter != UP_COUNTER_MAX)
            up_counter <= up_counter + 1;
    end

    assign fir_valid = up_counter == UP_COUNTER_MAX;

    // Whenever the upstream de-asserts s_axis_tvalid, the FIR pipeline halts by de-asserted fir_clock_en, at the mean time, fir_valid will be deasserted
    // When s_axis_tlast is observed, the down counter will be activated to correctly deasserts fir_valid and generate last signal to the downstream, at the same time, s_axis_tready should be deasserted to reject input from a new signal

    assign m_axis_tready_reg = !skid_valid; 
    assign m_axis_tdata = skid_valid ? data_skid_buffer : fir_data_out;
    assign s_axis_tready = m_axis_tready_reg;
    assign fir_clock_en = m_axis_tready_reg && s_axis_tvalid;  // FIR must be enabled when there's handshake at the input
    assign m_axis_tvalid = skid_valid || fir_valid;

endmodule

module fir_filter #(
    parameter NUM_TAPS = 2,  // Length of the finite impulse response of the filter
    parameter TAP_WIDTH = 3,  // Number of bits used to represent a tap
    parameter DATA_IN_WIDTH = 8,  // Number of bits used to represent numbers in the input signal
    parameter DATA_OUT_WIDTH = TAP_WIDTH + DATA_IN_WIDTH + 8  // Tap will be multiplied with the input, the results will be TAP_WIDTH + DATA_IN_WIDTH bit wide. +8 to make room for the accumulation, can be adjusted
) (
    input clk,
    input rst_n,
    input clk_en,
    input signed [DATA_IN_WIDTH-1:0] data_in,
    output logic signed [DATA_OUT_WIDTH-1:0] data_out
);

    logic signed [TAP_WIDTH-1:0] tap [NUM_TAPS-1:0];
    logic signed [DATA_IN_WIDTH-1:0] data_in_pipe [NUM_TAPS:0];
    logic signed [DATA_OUT_WIDTH-1:0] data_out_pipe [NUM_TAPS:0];

    assign data_in_pipe[0] = data_in;
    assign data_out_pipe[0] = 0;
    assign data_out = data_out_pipe[NUM_TAPS];
    
    // Hardcoded for now
    assign tap[0] = 3'b001;
    assign tap[1] = 3'b010;

    genvar i;

    generate
        for (i = 0; i < NUM_TAPS; i = i + 1) begin: FIRFilter
            fir_tap # (TAP_WIDTH, DATA_IN_WIDTH) FIRTap (
                .clk(clk),
                .clk_en(clk_en),
                .rst_n(rst_n),
                .data_in(data_in_pipe[i]),
                .tap(tap[i]),
                .partial_sum(data_out_pipe[i]),
                .data_out(data_out_pipe[i+1]),
                .data_in_delayed(data_in_pipe[i+1])
            );
        end
    endgenerate    
endmodule

module fir_tap #(
    parameter TAP_WIDTH = 2,
    parameter DATA_IN_WIDTH = 8,
    parameter DATA_OUT_WIDTH = TAP_WIDTH + DATA_IN_WIDTH + 8
) (
    input clk,
    input clk_en, // Used to halt the system pipeline if needed
    input rst_n,
    input signed [DATA_IN_WIDTH-1:0] data_in,
    input signed [TAP_WIDTH-1:0] tap,
    input signed [DATA_OUT_WIDTH-1:0] partial_sum,
    output signed [DATA_OUT_WIDTH-1:0] data_out, // +8 to account for the space needed for accumulation
    output signed [DATA_IN_WIDTH-1:0] data_in_delayed
);
    reg signed [TAP_WIDTH + DATA_IN_WIDTH-1:0] product_reg;
    reg signed [DATA_IN_WIDTH-1:0] data_reg, data_delayed_reg;
    reg signed [DATA_OUT_WIDTH-1:0] accumulator_reg;
    
    always@(posedge clk) begin
        if (!rst_n) begin
            data_reg <= 0;
            data_delayed_reg <= 0; // This reg ensures the correct delaying of signals to correctly compute the convolution
        end
        else if (clk_en) begin
            data_reg <= data_in;
            data_delayed_reg <= data_reg;
        end
    end
    
    // This reg cuts the combinational path between the multiplier and the accumulator 
    always@(posedge clk) begin
        if (!rst_n)
            product_reg <= 0;
        else if (clk_en)
            product_reg <= data_reg * tap;
    end

    // This reg cuts the long chain of accumulators
    always@(posedge clk) begin
        if (!rst_n)
            accumulator_reg <= 0;
        else if (clk_en)
            accumulator_reg <= partial_sum + { {(DATA_OUT_WIDTH-(TAP_WIDTH + DATA_IN_WIDTH)){product_reg[(TAP_WIDTH + DATA_IN_WIDTH-1)]}}, product_reg}; // Sign extension and addition
    end

    assign data_out = accumulator_reg;
    assign data_in_delayed = data_delayed_reg;
endmodule