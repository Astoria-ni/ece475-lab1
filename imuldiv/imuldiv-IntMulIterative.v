//========================================================================
// Lab 1 - Iterative Mul Unit
//========================================================================

`ifndef RISCV_INT_MUL_ITERATIVE_V
`define RISCV_INT_MUL_ITERATIVE_V

module imuldiv_IntMulIterative
(
  input                clk,
  input                reset,

  input  [31:0] mulreq_msg_a,
  input  [31:0] mulreq_msg_b,
  input         mulreq_val,
  output        mulreq_rdy,

  output [63:0] mulresp_msg_result,
  output        mulresp_val,
  input         mulresp_rdy
);

  wire          a_mux_sel;
  wire          b_mux_sel;
  wire          result_mux_sel;
  wire          result_en;
  wire          add_mux_sel;
  wire          cntr_mux_sel;
  wire          sign_en;
  wire          sign_mux_sel;
  wire          b_reg_lsb; //b_reg[0]
  wire    [4:0] counter;
  wire          sign;

  imuldiv_IntMulIterativeDpath dpath
  (
    .clk                (clk),
    .reset              (reset),
    .mulreq_msg_a       (mulreq_msg_a),
    .mulreq_msg_b       (mulreq_msg_b),
    .mulresp_msg_result (mulresp_msg_result),
    .a_mux_sel          (a_mux_sel),
    .b_mux_sel          (b_mux_sel),
    .result_mux_sel     (result_mux_sel),
    .result_en          (result_en),
    .add_mux_sel        (add_mux_sel),
    .cntr_mux_sel       (cntr_mux_sel),
    .sign_en            (sign_en),
    .sign_mux_sel       (sign_mux_sel),
    .b_reg_lsb          (b_reg_lsb),
    .counter            (counter),
    .sign               (sign)
  );

  imuldiv_IntMulIterativeCtrl ctrl
  (
    .clk                (clk),
    .reset              (reset),
    .mulreq_val         (mulreq_val),
    .mulreq_rdy         (mulreq_rdy),
    .mulresp_val        (mulresp_val),
    .mulresp_rdy        (mulresp_rdy),
    .a_mux_sel          (a_mux_sel),
    .b_mux_sel          (b_mux_sel),
    .result_mux_sel     (result_mux_sel),
    .result_en          (result_en),
    .add_mux_sel        (add_mux_sel),
    .cntr_mux_sel       (cntr_mux_sel),
    .sign_en            (sign_en),
    .sign_mux_sel       (sign_mux_sel),
    .b_reg_lsb          (b_reg_lsb),
    .counter            (counter),
    .sign               (sign)
  );

endmodule

//------------------------------------------------------------------------
// Datapath
//------------------------------------------------------------------------

module imuldiv_IntMulIterativeDpath
(
  input         clk,
  input         reset,

  input  [31:0] mulreq_msg_a,       // Operand A
  input  [31:0] mulreq_msg_b,       // Operand B
  output [63:0] mulresp_msg_result, // Result of operation

  input         a_mux_sel,
  input         b_mux_sel,
  input         result_mux_sel,
  input         result_en,
  input         add_mux_sel,
  input         cntr_mux_sel,
  input         sign_en,
  input         sign_mux_sel,

  output        b_reg_lsb,
  output  [4:0] counter,
  output        sign
);

  reg  [63:0] a_reg;       // Register for storing operand A
  reg  [31:0] b_reg;       // Register for storing operand B
  reg  [63:0] result_reg;
  reg  [4:0]  counter_reg;
  reg         sign_reg;
  //just do stuff in diagram
  wire sign_bit_a = mulreq_msg_a[31];
  wire sign_bit_b = mulreq_msg_b[31];

  wire[31:0] unsigned_a = ( sign_bit_a ) ? (~mulreq_msg_a + 1'b1) : mulreq_msg_a;
  wire[31:0] unsigned_b = ( sign_bit_b ) ? (~mulreq_msg_b + 1'b1) : mulreq_msg_b;

  wire[63:0] unsigned_a_64 = {32'b0, unsigned_a};
  
  wire[63:0] a_shift_out = a_reg << 1;
  wire[31:0] b_shift_out = b_reg >> 1;

  wire[63:0] a_mux_out = (a_mux_sel == 1'b0) ? unsigned_a_64 : a_shift_out;
  wire[31:0] b_mux_out = (b_mux_sel == 1'b0) ? unsigned_b : b_shift_out;

  wire[63:0] adder_out = result_reg + a_reg;
  wire[63:0] add_mux_out = (add_mux_sel == 1'b0) ? result_reg : adder_out;

  wire[63:0] result_mux_out = (result_mux_sel == 1'b0) ? 64'b0 : add_mux_out;

  wire[4:0] counter_minus1 = counter_reg - 5'd1;
  wire[4:0] cntr_mux_out = (cntr_mux_sel == 1'b0) ? 5'd31 : counter_minus1;

  wire sign_out = sign_bit_a ^ sign_bit_b;

  wire[63:0] signed_result = (~result_reg + 64'b1);
  wire[63:0] sign_mux_out = (sign_mux_sel == 1'b0) ? result_reg : signed_result;

  assign b_reg_lsb = b_reg[0];
  assign counter = counter_reg;
  assign mulresp_msg_result = sign_mux_out;
  assign sign = sign_reg;

  //Sequential
  always @(posedge clk) begin
    if(reset) begin
      a_reg <= 64'b0;
      b_reg <= 32'b0;
      result_reg <= 64'b0;
      counter_reg <= 5'b0;
      sign_reg <= 1'b0;
    end else begin
      a_reg <= a_mux_out;
      b_reg <= b_mux_out;
      counter_reg <= cntr_mux_out;
      if(result_en) result_reg <= result_mux_out;
      if(sign_en) sign_reg <= sign_out;
    end
  end

endmodule

//------------------------------------------------------------------------
// Control Logic
//------------------------------------------------------------------------

module imuldiv_IntMulIterativeCtrl
(
  input         clk,
  input         reset,

  input         mulreq_val,
  output        mulreq_rdy,
  output        mulresp_val,
  input         mulresp_rdy,

  output        a_mux_sel,
  output        b_mux_sel,
  output        result_mux_sel,
  output        result_en,
  output        add_mux_sel,
  output        cntr_mux_sel,
  output        sign_en,
  output        sign_mux_sel,

  input         b_reg_lsb,
  input   [4:0] counter,
  input         sign
);
  //state = 2'd0: not started
  //state = 2'd1: calculating
  //state = 2'd2: done

  reg [1:0] state, next_state;
  
  reg mulreq_rdy_reg, mulresp_val_reg;
  reg a_mux_sel_reg, b_mux_sel_reg, result_mux_sel_reg, result_en_reg;
  reg add_mux_sel_reg, cntr_mux_sel_reg, sign_en_reg, sign_mux_sel_reg;

  assign mulreq_rdy = mulreq_rdy_reg;
  assign mulresp_val = mulresp_val_reg;

  assign a_mux_sel = a_mux_sel_reg;
  assign b_mux_sel = b_mux_sel_reg;
  assign result_mux_sel = result_mux_sel_reg;
  assign result_en = result_en_reg;

  assign add_mux_sel = add_mux_sel_reg;
  assign cntr_mux_sel = cntr_mux_sel_reg;
  assign sign_en = sign_en_reg;
  assign sign_mux_sel = sign_mux_sel_reg;

  wire is_counter_zero = (counter == 5'd0);

  always @(*) begin
    next_state = state;
    //defaults
    mulreq_rdy_reg = 1'b0;
    mulresp_val_reg = 1'b0;
    a_mux_sel_reg = 1'b0;
    b_mux_sel_reg = 1'b0;
    result_mux_sel_reg = 1'b0;
    result_en_reg = 1'b0;
    add_mux_sel_reg = 1'b0;
    cntr_mux_sel_reg = 1'b0;
    sign_en_reg = 1'b0;
    sign_mux_sel_reg = 1'b0;

    case(state)
      2'd0: begin //not started case
        mulreq_rdy_reg = 1'b1; //ready for input!
        if(mulreq_val) begin //if we received smth
          a_mux_sel_reg = 1'b0; //initially we take in |a|
          b_mux_sel_reg = 1'b0;
          cntr_mux_sel_reg = 1'b0; //initialize counter = 31
          result_mux_sel_reg = 1'b0; //initialize result=0
          result_en_reg = 1'b1;
          sign_en_reg = 1'b1; //allow for updating of result & sign

          next_state = 2'd1;
        end
      end

      2'd1: begin //currently calculating case
        a_mux_sel_reg = 1'b1;
        b_mux_sel_reg = 1'b1;
        cntr_mux_sel_reg = 1'b1; //^ initialized already so use new values
        result_mux_sel_reg = 1'b1; //again initialized so use new values
        add_mux_sel_reg = b_reg_lsb; //add only if b[0]
        result_en_reg = 1'b1; //allow updating result. not sign, note.

        if(is_counter_zero) next_state = 2'd2;
      end
      2'd2: begin //done case
        mulresp_val_reg = 1'b1;
        result_en_reg = 1'b0; //freeze result reg
        sign_mux_sel_reg = sign; //use this sign!!
        if(mulresp_rdy) next_state = 2'd0; //we're done...
      end
    endcase
  end

  always @(posedge clk) begin
    if(reset) state <= 2'd0;
    else state <= next_state;
  end

endmodule

`endif
