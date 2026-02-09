//========================================================================
// Lab 1 - Iterative Div Unit
//========================================================================

`ifndef RISCV_INT_DIV_ITERATIVE_V
`define RISCV_INT_DIV_ITERATIVE_V

`include "imuldiv-DivReqMsg.v"

module imuldiv_IntDivIterative
(

  input         clk,
  input         reset,

  input         divreq_msg_fn,
  input  [31:0] divreq_msg_a,
  input  [31:0] divreq_msg_b,
  input         divreq_val,
  output        divreq_rdy,

  output [63:0] divresp_msg_result,
  output        divresp_val,
  input         divresp_rdy
);

  wire a_mux_sel;
  wire a_en;
  wire b_en;
  wire sub_out_msb;
  wire sub_mux_sel;
  wire cntr_mux_sel;
  wire cntr_en;
  wire [4:0] counter;
  wire sign_en;
  wire div_sign;
  wire rem_sign;
  wire div_sign_mux_sel;
  wire rem_sign_mux_sel;
  wire is_op_signed;

  imuldiv_IntDivIterativeDpath dpath
  (
    .clk                (clk),
    .reset              (reset),
    .divreq_msg_fn      (divreq_msg_fn),
    .divreq_msg_a       (divreq_msg_a),
    .divreq_msg_b       (divreq_msg_b),
    .divresp_msg_result (divresp_msg_result),

    .a_mux_sel          (a_mux_sel),
    .a_en               (a_en),
    .b_en               (b_en),
    .sub_out_msb        (sub_out_msb),
    .sub_mux_sel        (sub_mux_sel),
    .cntr_mux_sel       (cntr_mux_sel),
    .counter            (counter),
    .sign_en            (sign_en),
    .div_sign           (div_sign),
    .rem_sign           (rem_sign),
    .div_sign_mux_sel   (div_sign_mux_sel),
    .rem_sign_mux_sel   (rem_sign_mux_sel)
  );

  imuldiv_IntDivIterativeCtrl ctrl
  (
    .clk                (clk),
    .reset              (reset),
    .divreq_msg_fn      (divreq_msg_fn),
    .divreq_val         (divreq_val),
    .divreq_rdy         (divreq_rdy),
    .divresp_val        (divresp_val),
    .divresp_rdy        (divresp_rdy),

    .a_mux_sel          (a_mux_sel),
    .a_en               (a_en),
    .b_en               (b_en),
    .sub_out_msb        (sub_out_msb),
    .sub_mux_sel        (sub_mux_sel),
    .cntr_mux_sel       (cntr_mux_sel),
    .counter            (counter),
    .sign_en            (sign_en),
    .div_sign           (div_sign),
    .rem_sign           (rem_sign),
    .div_sign_mux_sel   (div_sign_mux_sel),
    .rem_sign_mux_sel   (rem_sign_mux_sel)
  );

endmodule

//------------------------------------------------------------------------
// Datapath
//------------------------------------------------------------------------

module imuldiv_IntDivIterativeDpath
(
  input         clk,
  input         reset,

  input         divreq_msg_fn,      // Function of MulDiv Unit
  input  [31:0] divreq_msg_a,       // Operand A
  input  [31:0] divreq_msg_b,       // Operand B

  output [63:0] divresp_msg_result, // Result of operation

  input         a_mux_sel,
  input         a_en,
  input         b_en,
  output        sub_out_msb,
  input         sub_mux_sel,
  input         cntr_mux_sel,
  output [4:0]  counter,
  input         sign_en,

  output        div_sign,
  output        rem_sign,
  input         div_sign_mux_sel,
  input         rem_sign_mux_sel
);

  reg [64:0] a_reg;
  reg [64:0] b_reg;
  reg div_sign_reg;
  reg rem_sign_reg;
  reg[4:0] counter_reg;
  assign div_sign = div_sign_reg;
  assign rem_sign = rem_sign_reg;
  assign counter  = counter_reg;

  //sign bits
  wire sign_bit_a = divreq_msg_a[31];
  wire sign_bit_b = divreq_msg_b[31];
  wire div_sign_out = sign_bit_a ^ sign_bit_b;
  wire rem_sign_out = sign_bit_a;
  
  //counter
  wire[4:0] counter_minus1 = counter_reg - 5'd1;
  wire[4:0] cntr_mux_out = (cntr_mux_sel == 1'b0) ? 5'd31 : counter_minus1;

  //initial unsign + initialize
  wire is_op_signed_initial = divreq_msg_fn;
  wire[31:0] unsigned_a = is_op_signed_initial ? ( ( sign_bit_a ) ? (~divreq_msg_a + 1'b1) : divreq_msg_a ) : divreq_msg_a;
  wire[31:0] unsigned_b = is_op_signed_initial ? ( ( sign_bit_b ) ? (~divreq_msg_b + 1'b1) : divreq_msg_b ) : divreq_msg_b;
  wire[64:0] a_init = {33'b0, unsigned_a};
  wire[64:0] b_init = {1'b0, unsigned_b, 32'b0};

  //stuff around subtraction
  wire[64:0] a_shift_out = a_reg << 1;
  wire[64:0] sub_out = a_shift_out - b_reg;
  assign sub_out_msb = sub_out[64];

  //later subtraction stuff
  wire[64:0] sub_out_modi = {sub_out[64:1], 1'b1};
  wire[64:0] sub_mux_out = (sub_mux_sel == 1'b0) ? a_shift_out : sub_out_modi;

  //a input
  wire[64:0] a_inp = (a_mux_sel == 1'b0) ? a_init : sub_mux_out;

  //answer
  wire[31:0] res_rem = a_reg[63:32];
  wire[31:0] res_div = a_reg[31:0];

  wire[31:0] minus_res_rem = ~res_rem + 32'b1;
  wire[31:0] minus_res_div = ~res_div + 32'b1;

  wire[31:0] rem_out = rem_sign_mux_sel ? minus_res_rem : res_rem;
  wire[31:0] div_out = div_sign_mux_sel ? minus_res_div : res_div;
  //note we choose for our selectors to be powerful here!
  assign divresp_msg_result = {rem_out, div_out};

  always @(posedge clk) begin
    if (reset) begin
      a_reg <= 65'b0;
      b_reg <= 65'b0;
      counter_reg <= 5'b0;
      div_sign_reg <= 1'b0;
      rem_sign_reg <= 1'b0;
    end
    else begin
      if(a_en) a_reg <= a_inp;
      if(b_en) b_reg <= b_init;
      counter_reg <= cntr_mux_out;
      if(sign_en) begin
        div_sign_reg <= div_sign_out;
        rem_sign_reg <= rem_sign_out;
      end
    end
  end

endmodule

//------------------------------------------------------------------------
// Control Logic
//------------------------------------------------------------------------

module imuldiv_IntDivIterativeCtrl
(
  input         clk,
  input         reset,

  input         divreq_msg_fn,
  input         divreq_val,
  output        divreq_rdy,
  output        divresp_val,
  input         divresp_rdy,
  output        a_mux_sel,
  output        a_en,
  output        b_en,
  input         sub_out_msb,
  output        sub_mux_sel,
  output        cntr_mux_sel,
  input [4:0]   counter,
  output        sign_en,
  input         div_sign,
  input         rem_sign,
  output        div_sign_mux_sel,
  output        rem_sign_mux_sel
);
  //boilerplate
  reg [1:0] state, next_state; 
  reg divreq_rdy_reg, divresp_val_reg;
  reg a_mux_sel_reg, a_en_reg, b_en_reg;
  reg sub_mux_sel_reg, cntr_mux_sel_reg, sign_en_reg;
  reg div_sign_mux_sel_reg, rem_sign_mux_sel_reg;

  assign divreq_rdy = divreq_rdy_reg;
  assign divresp_val = divresp_val_reg;
  assign a_mux_sel = a_mux_sel_reg;
  assign a_en = a_en_reg;
  assign b_en = b_en_reg;
  assign sub_mux_sel = sub_mux_sel_reg;
  assign cntr_mux_sel = cntr_mux_sel_reg;
  assign sign_en = sign_en_reg;
  assign div_sign_mux_sel = div_sign_mux_sel_reg;
  assign rem_sign_mux_sel = rem_sign_mux_sel_reg;

  wire is_counter_zero = (counter == 5'd0);

  //sadly is_op_signed needs to be persistent reg
  reg is_op_signed;
  always @(posedge clk) begin
    if(reset) is_op_signed <= 1'b0;
    else if(divreq_rdy_reg && divreq_val) is_op_signed <= divreq_msg_fn;
  end

  always @(*) begin
    next_state = state;
    //defaults
    divresp_val_reg = 1'b0;
    divreq_rdy_reg = 1'b0;
    a_mux_sel_reg = 1'b0;
    a_en_reg = 1'b0;
    b_en_reg = 1'b0;
    sub_mux_sel_reg = 1'b0;
    cntr_mux_sel_reg = 1'b0;
    sign_en_reg = 1'b0;
    div_sign_mux_sel_reg = 1'b0;
    rem_sign_mux_sel_reg = 1'b0;

    case(state)
      2'd0: begin //not started case
        divreq_rdy_reg = 1'b1;
        if(divreq_val) begin
          a_mux_sel_reg = 1'b0;
          a_en_reg = 1'b1;
          b_en_reg = 1'b1;
          cntr_mux_sel_reg = 1'b0;
          sign_en_reg = 1'b1;
          next_state = 2'd1;
        end
      end
      2'd1: begin
        a_mux_sel_reg = 1'b1;
        a_en_reg = 1'b1;
        b_en_reg = 1'b0;
        sub_mux_sel_reg = ~sub_out_msb;
        cntr_mux_sel_reg = 1'b1;
        if (is_counter_zero) next_state = 2'd2;
      end
      2'd2: begin
        divresp_val_reg = 1'b1;
        div_sign_mux_sel_reg = is_op_signed & div_sign;
        rem_sign_mux_sel_reg = is_op_signed & rem_sign;
        if(divresp_rdy) next_state = 2'd0;
      end
    endcase
  end

  always @(posedge clk) begin
    if(reset) state <= 2'd0;
    else state <= next_state;
  end

endmodule

`endif
