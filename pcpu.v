// Project #2: 16-bit pipeline processor
// Verilog source file for "System LSI design"
//                                                  2013.10  T. Ikenaga

// Operation Code
`define NOP   5'b00000
`define HALT  5'b00001
`define LOAD  5'b00010
`define STORE 5'b00011
`define LDIH  5'b10000
`define ADD   5'b01000
`define ADDI  5'b01001
`define ADDC  5'b10001
`define SUB   5'b01010
`define SUBI  5'b01011
`define SUBC  5'b10010
`define CMP   5'b01100

`define AND   5'b01101
`define OR    5'b01110
`define XOR   5'b01111
`define SLL   5'b00100
`define SRL   5'b00101
`define SLA   5'b00110
`define SRA   5'b00111

`define JUMP  5'b11000
`define JMPR  5'b11001
`define BZ    5'b11010
`define BNZ   5'b11011
`define BN    5'b11100
`define BNN   5'b11101
`define BC    5'b11110
`define BNC   5'b11111
`define MUL   5'b10100
`define COPY  5'b10101

// add 3 more operations
// `define XX   5'b10011
// `define XX   5'b10110
// `define XX   5'b10111

// FSM for CPU Control
`define idle 1'b0
`define exec 1'b1

// main module
module pcpu (reset, clock, enable, start, i_addr, i_datain, d_addr,
             d_datain, d_dataout, d_we, select_y, y);

input  reset, clock, enable, start;
input  [15:0] i_datain;
output [7:0]  i_addr;
output [7:0]  d_addr;
input  [15:0] d_datain;
output [15:0] d_dataout;
output d_we;

// Debugging
input  [3:0]  select_y;
output [15:0] y;

// Definition of Flip flop
reg [7:0]  pc;
reg [15:0] id_ir, ex_ir, mem_ir, wb_ir;
reg [15:0] gr [0:7];                        // register file 16-bit * 8
reg [15:0] reg_A, reg_B, reg_C, reg_C1;
reg [15:0] smdr, smdr1;
reg [15:0]  store_add_in_Mem;
reg zf, nf, cf, dw;
reg state;

// Definition of temporary variables
reg [14:0] temp_reg_ALUo_SLA_SRA;
reg [16:0] ALUo;
reg [15:0] y;
reg next_state;

// Temp variables for `MUL
reg [15:0] stored0;
reg [15:0] stored1;
reg [15:0] stored2;
reg [15:0] stored3;
reg [15:0] stored4;
reg [15:0] stored5;
reg [15:0] stored6;
reg [15:0] stored7;

reg [15:0] add01;
reg [15:0] add23;
reg [15:0] add45;
reg [15:0] add67;
reg [15:0] add0123;
reg [15:0] add4567;
reg [15:0] add01234567;

assign i_addr = pc;
assign d_we  = dw;
assign d_addr = reg_C[7:0];
assign d_dataout = smdr1;
assign ZF = zf;
assign NF = nf;
assign CF = cf;

wire Jump_Flush_Flag;
wire Ex_ir_Arith_Flag;
wire Mem_ir_Arith_Flag;
wire Wb_ir_Arith_Flag;
wire Mem_ir_LOAD_Flag;
wire Wb_ir_LOAD_Flag;

assign Jump_Flush_Flag =(mem_ir[15:11] == `JUMP)|| (mem_ir[15:11] == `JMPR)||
                    ((mem_ir[15:11] == `BZ ) && (zf == 1'b1)) ||
                    ((mem_ir[15:11] == `BNZ) && (zf == 1'b0)) ||
                    ((mem_ir[15:11] == `BN ) && (nf == 1'b1)) ||
                    ((mem_ir[15:11] == `BNN) && (nf == 1'b0)) ||
                    ((mem_ir[15:11] == `BC ) && (cf == 1'b1)) ||
                    ((mem_ir[15:11] == `BNC) && (cf == 1'b0));

assign Ex_ir_Arith_Flag = (ex_ir[15:11]==`LDIH)||(ex_ir[15:11]==`ADD)||(ex_ir[15:11]==`ADDI)||
                    (ex_ir[15:11]==`ADDC)||(ex_ir[15:11]==`SUB)||(ex_ir[15:11]==`SUBI)||
                    (ex_ir[15:11]==`SUBC)||(ex_ir[15:11]==`AND)||(ex_ir[15:11]==`OR)||
                    (ex_ir[15:11]==`XOR)||(ex_ir[15:11]==`SLL)||(ex_ir[15:11]==`SRL)||
                    (ex_ir[15:11]==`SLA)||(ex_ir[15:11]==`SRA)||(ex_ir[15:11]==`COPY)||(ex_ir[15:11]==`MUL);

assign Mem_ir_Arith_Flag= (mem_ir[15:11]==`LDIH)||(mem_ir[15:11]==`ADD)||(mem_ir[15:11]==`ADDI)||
                    (mem_ir[15:11]==`ADDC)||(mem_ir[15:11]==`SUB)||(mem_ir[15:11]==`SUBI)||
                    (mem_ir[15:11]==`SUBC)||(mem_ir[15:11]==`AND)||(mem_ir[15:11]==`OR)||
                    (mem_ir[15:11]==`XOR)||(mem_ir[15:11]==`SLL)||(mem_ir[15:11]==`SRL)||
                    (mem_ir[15:11]==`SLA)||(mem_ir[15:11]==`SRA)||(mem_ir[15:11]==`COPY)||(mem_ir[15:11]==`MUL);

assign Wb_ir_Arith_Flag = (wb_ir[15:11]==`LDIH)||(wb_ir[15:11]==`ADD)||(wb_ir[15:11]==`ADDI)||
                    (wb_ir[15:11]==`ADDC)||(wb_ir[15:11]==`SUB)||(wb_ir[15:11]==`SUBI)||
                    (wb_ir[15:11]==`SUBC)||(wb_ir[15:11]==`AND)||(wb_ir[15:11]==`OR)||
                    (wb_ir[15:11]==`XOR)||(wb_ir[15:11]==`SLL)||(wb_ir[15:11]==`SRL)||
                    (wb_ir[15:11]==`SLA)||(wb_ir[15:11]==`SRA)||(wb_ir[15:11]==`COPY)||(wb_ir[15:11]==`MUL);

assign Mem_ir_LOAD_Flag = (mem_ir[15:11]==`LOAD);
assign Wb_ir_LOAD_Flag  = (wb_ir[15:11] ==`LOAD);

// CPU control (FSM)
always @(posedge clock or negedge reset)
    begin
    if (!reset)
        state <= `idle;
    else
        state <= next_state;
    end

always @(state or enable or start or wb_ir[15:11])
    begin
    case (state)
        `idle : if ((enable == 1'b1) && (start == 1'b1))
                    next_state <= `exec;
                else
                    next_state <= `idle;
        `exec : if ((enable == 1'b0) || (wb_ir[15:11] == `HALT))
                    next_state <= `idle;
                else
                    next_state <= `exec;
    endcase
    end

// IF_block (1st Stage)
always @(posedge clock or negedge reset)
    begin
    if (!reset)
        begin
        id_ir <= 16'b0000000000000000;
        pc    <= 8'b00000000;
        end
    else if(state==`exec)
        begin
        // Check branch control
        if ( Jump_Flush_Flag )	// flush id_ir=0, when jump-controll happen
            begin
            pc <= reg_C[7:0];
            id_ir <= 16'b0000000000000000;
            end
        // iff the last one instruction, which already stored in ID_IR:op_code = `LOAD
        // and id_ir:op1 =  the newest instruction:op2/op3
        else if (id_ir[15:11] == `LOAD)
            begin
            pc <= pc;
            id_ir <= 16'b0000000000000000;
            end
        else
            begin
            pc <= pc+1;
            id_ir <= i_datain;
            end
        end
    end

// ID_block (2nd Stage)
always @(posedge clock or negedge reset)
    begin
    if (!reset)
        begin
        ex_ir <= 16'b0000000000000000;
        reg_A <= 16'b0000000000000000;
        reg_B <= 16'b0000000000000000;
        smdr  <= 16'b0000000000000000;
        end
    else if(state==`exec)
        begin
        if ( Jump_Flush_Flag )		// flush ex_ir=0
            ex_ir <= 16'b0000000000000000;
        else
            ex_ir <= id_ir;

        if (id_ir[15:11] == `STORE)
            begin
            // Assign for reg_A
            if( (id_ir[6:4]==ex_ir[10:8]) && Ex_ir_Arith_Flag)
                // Only between arithmetic operations
                reg_A <= ALUo[15:0];
            else if( (id_ir[6:4]==mem_ir[10:8]) && Mem_ir_Arith_Flag)    // if(id_ir:op2==mem_ir:op1)
                // Only between arithmetic operations
                reg_A <= reg_C;
            else if ( (id_ir[6:4]==mem_ir[10:8]) && Mem_ir_LOAD_Flag )
                reg_A <= d_datain;
            else if( (id_ir[6:4]==wb_ir[10:8]) && Wb_ir_Arith_Flag )     // if(id_ir:op2==wb_ir:op1)
                // Only between arithmetic operations
                reg_A <= reg_C1;
            else if ( (id_ir[6:4]==wb_ir[10:8])  && Wb_ir_LOAD_Flag )
                reg_A <= reg_C1;
            else
                reg_A <= gr[(id_ir[6:4])];        // [r2] to reg_A

            // Assign for reg_B
            reg_B <= {12'b000000000000, id_ir[3:0]};    // val3 to reg_B

            // Assign for smdr
            if( (id_ir[10:8]==ex_ir[10:8]) && Ex_ir_Arith_Flag )
                // if(id_ir:op2==ex_ir:op1) then reg_A <= 16'hFFFF;
                // Only between arithmetic operations
                smdr  <= ALUo[15:0];
            else if( (id_ir[10:8]==mem_ir[10:8]) && Mem_ir_Arith_Flag )    // if(id_ir:op2==mem_ir:op1)
                // Only between arithmetic operations
                smdr  <= reg_C;
            else if ( (id_ir[10:8]==mem_ir[10:8]) && Mem_ir_LOAD_Flag )
                smdr <= d_datain;
            else if( (id_ir[10:8]==wb_ir[10:8]) && Wb_ir_Arith_Flag )      // if(id_ir:op2==wb_ir:op1)
                // Only between arithmetic operations
                smdr  <= reg_C1;
            else if ( (id_ir[10:8]==wb_ir[10:8])  && Wb_ir_LOAD_Flag )
                smdr <= reg_C1;
            else
                begin
                smdr  <= gr[id_ir[10:8]];        // when 'STORE, [r1] to smdr
                end
            end

        else if ( id_ir[15:11] == `LOAD )
            begin
            // Assign for reg_A
            if( (id_ir[6:4]==ex_ir[10:8]) && Ex_ir_Arith_Flag )
                // if(id_ir:op2==ex_ir:op1) then reg_A <= 16'hFFFF;
                // Only between arithmetic operations
                reg_A <= ALUo[15:0];
            else if( (id_ir[6:4]==mem_ir[10:8]) && Mem_ir_Arith_Flag )    // if(id_ir:op2==mem_ir:op1)
                // Only between arithmetic operations
                reg_A <= reg_C;
            else if ( (id_ir[6:4]==mem_ir[10:8]) && Mem_ir_LOAD_Flag )
                reg_A <= d_datain;
            else if( (id_ir[6:4]==wb_ir[10:8]) && Wb_ir_Arith_Flag )        // if(id_ir:op2==wb_ir:op1)
                // Only between arithmetic operations
                reg_A <= reg_C1;
            else if ( (id_ir[6:4]==wb_ir[10:8])  && Wb_ir_LOAD_Flag )
                reg_A <= reg_C1;
            else
                reg_A <= gr[(id_ir[6:4])];        // [r2] to reg_A

            // Assign for reg_B
            reg_B <= {12'b000000000000, id_ir[3:0]};    // val3 to reg_B
            end

        else if ( id_ir[15:11] == `LDIH )
            begin                                // Cuz LDIH is a immediate operation,
            reg_A <= 16'b0000000000000000;        // The LDIH:op2/op3 are immediate number but not gr[X]
            reg_B <= {8'b00000000, id_ir[7:0]};    // { 0 0 val1 val2 }
            end

        else if ((id_ir[15:11] == `ADD) || (id_ir[15:11] == `ADDC) || (id_ir[15:11] == `SUB) || (id_ir[15:11] == `SUBC) ||
                 (id_ir[15:11] == `AND) || (id_ir[15:11] == `OR)   || (id_ir[15:11] == `XOR) || (id_ir[15:11] == `CMP ) || (id_ir[15:11] == `MUL))
            begin
            // Assign for reg_A
            if( (id_ir[6:4]==ex_ir[10:8]) && Ex_ir_Arith_Flag )
                // if(id_ir:op2==ex_ir:op1) then reg_A <= 16'hFFFF;
                // Only between arithmetic operations
                reg_A <= ALUo[15:0];
            else if( (id_ir[6:4]==mem_ir[10:8]) && Mem_ir_Arith_Flag )
                    reg_A <= reg_C;
            else if( (id_ir[6:4]==mem_ir[10:8]) && Mem_ir_LOAD_Flag )
                    reg_A <= d_datain;
            else if( (id_ir[6:4]==wb_ir[10:8]) && Wb_ir_Arith_Flag )
                    reg_A <= reg_C1;
            else if( (id_ir[6:4]==wb_ir[10:8]) && Wb_ir_LOAD_Flag )
                    reg_A <= reg_C1;
            else
                reg_A <= gr[(id_ir[6:4])];        // [r2] to reg_A

            // Assign for reg_B
            if ( (id_ir[2:0]==ex_ir[10:8]) && Ex_ir_Arith_Flag )        // if(id_ir:op3==ex_ir:op1)
                // Only between arithmetic operations
                reg_B <= ALUo[15:0];
            else if ( (id_ir[2:0]==mem_ir[10:8]) && Mem_ir_Arith_Flag )    // if(id_ir:op3==mem_ir:op1)
                // Only between arithmetic operations
                reg_B <= reg_C;
            else if ( (id_ir[2:0]==mem_ir[10:8]) && Mem_ir_LOAD_Flag )
                reg_B <= d_datain;
            else if ( (id_ir[2:0]==wb_ir[10:8]) && Wb_ir_Arith_Flag )    // if(id_ir:op3==wb_ir:op1)
                // Only between arithmetic operations
                reg_B <= reg_C1;
            else if ( (id_ir[2:0]==wb_ir[10:8])  && Wb_ir_LOAD_Flag )
                reg_B <= reg_C1;
            else
                reg_B <= gr[(id_ir[2:0])];        // [r3] to reg_B
            end

        else if ( (id_ir[15:11] == `ADDI) || (id_ir[15:11] == `SUBI) )
            begin            // ADDI/SUBI are 'RI':Register-Immediate operation,
            // Assign for reg_A
            if( (id_ir[10:8]==ex_ir[10:8]) && Ex_ir_Arith_Flag )
                // if(id_ir:op2==ex_ir:op1) then reg_A <= 16'hFFFF;
                // Only between arithmetic operations
                reg_A <= ALUo[15:0];
            else if( (id_ir[10:8]==mem_ir[10:8]) && Mem_ir_Arith_Flag )    // if(id_ir:op2==mem_ir:op1)
                // Only between arithmetic operations
                reg_A <= reg_C;
            else if ( (id_ir[10:8]==mem_ir[10:8]) && Mem_ir_LOAD_Flag )
                reg_A <= d_datain;
            else if( (id_ir[10:8]==wb_ir[10:8]) && Wb_ir_Arith_Flag )        // if(id_ir:op2==wb_ir:op1)
                // Only between arithmetic operations
                reg_A <= reg_C1;
            else if ( (id_ir[10:8]==wb_ir[10:8])  && Wb_ir_LOAD_Flag )
                reg_A <= reg_C1;
            else
                reg_A <= gr[(id_ir[10:8])];        // [r1] to reg_A

            // Assign for reg_B.    The ADDI/SUBI:op2/op3 are immediate number but not gr[X]
            reg_B <= {8'b00000000, id_ir[7:0]};
            end

        else if ( (id_ir[15:11] == `SLL) || (id_ir[15:11] == `SRL) ||
                  (id_ir[15:11] == `SLA) || (id_ir[15:11] == `SRA)
                )
            begin
            // Assign for reg_A
            if( (id_ir[6:4]==ex_ir[10:8]) && Ex_ir_Arith_Flag )
                // if(id_ir:op2==ex_ir:op1) then reg_A <= 16'hFFFF;
                // Only between arithmetic operations
                reg_A <= ALUo[15:0];
            else if( (id_ir[6:4]==mem_ir[10:8]) && Mem_ir_Arith_Flag )    // if(id_ir:op2==mem_ir:op1)
                // Only between arithmetic operations
                reg_A <= reg_C;
            else if ( (id_ir[6:4]==mem_ir[10:8]) && Mem_ir_LOAD_Flag )
                reg_A <= d_datain;
            else if( (id_ir[6:4]==wb_ir[10:8]) && Wb_ir_Arith_Flag )    // if(id_ir:op2==wb_ir:op1)
                // Only between arithmetic operations
                reg_A <= reg_C1;
            else if ( (id_ir[6:4]==wb_ir[10:8])  && Wb_ir_LOAD_Flag )
                reg_A <= reg_C1;
            else
                reg_A <= gr[(id_ir[6:4])];        // [r2] to reg_A

            reg_B <= {12'b000000000000, id_ir[3:0]};    // val3 to reg_B
            end

        else if ( id_ir[15:11] == `COPY )        // all-immediate operation
            begin
            reg_A <= 16'b0000000000000000;        // all-0 to reg_A

            // Assign for reg_B
            if ( (id_ir[2:0]==ex_ir[10:8]) && Ex_ir_Arith_Flag )        // if(id_ir:op3==ex_ir:op1)
                // Only between arithmetic operations
                reg_B <= ALUo[15:0];
            else if ( (id_ir[2:0]==mem_ir[10:8]) && Mem_ir_Arith_Flag )    // if(id_ir:op3==mem_ir:op1)
                // Only between arithmetic operations
                reg_B <= reg_C;
            else if ( (id_ir[2:0]==mem_ir[10:8]) && Mem_ir_LOAD_Flag )
                reg_B <= d_datain;
            else if ( (id_ir[2:0]==wb_ir[10:8]) && Wb_ir_Arith_Flag )    // if(id_ir:op3==wb_ir:op1)
                // Only between arithmetic operations
                reg_B <= reg_C1;
            else if ( (id_ir[2:0]==wb_ir[10:8])  && Wb_ir_LOAD_Flag )
                reg_B <= reg_C1;
            else
                reg_B <= gr[(id_ir[2:0])];        // [r3] to reg_B
            end

        else if ( id_ir[15:11] == `JUMP )        // all-immediate operation
            begin
            reg_A <= 16'b0000000000000000;        // all-0 to reg_A
            reg_B <= {8'b00000000, id_ir[7:0]};    // 16'b{r2,r3} to reg_B
            end

        else if ( (id_ir[15:11] == `BZ ) || (id_ir[15:11] == `BN ) || (id_ir[15:11] == `BC ) ||
                  (id_ir[15:11] == `BNZ) || (id_ir[15:11] == `BNN) || (id_ir[15:11] == `BNC) ||
                  (id_ir[15:11] == `JMPR)
                )
            begin
            // Assign for reg_A
            if( (id_ir[10:8]==ex_ir[10:8]) && Ex_ir_Arith_Flag )
                // if(id_ir:op2==ex_ir:op1) then reg_A <= 16'hFFFF;
                // Only between arithmetic operations
                reg_A <= ALUo[15:0];
            else if( (id_ir[10:8]==mem_ir[10:8]) && Mem_ir_Arith_Flag )    // if(id_ir:op2==mem_ir:op1)
                // Only between arithmetic operations
                reg_A <= reg_C;
            else if ( (id_ir[10:8]==mem_ir[10:8]) && Mem_ir_LOAD_Flag )
                reg_A <= d_datain;
            else if( (id_ir[10:8]==wb_ir[10:8]) && Wb_ir_Arith_Flag )        // if(id_ir:op2==wb_ir:op1)
                // Only between arithmetic operations
                reg_A <= reg_C1;
            else if ( (id_ir[10:8]==wb_ir[10:8])  && Wb_ir_LOAD_Flag )
                reg_A <= reg_C1;
            else
                reg_A <= gr[(id_ir[10:8])];        // [r1] to reg_A

            // Assign for reg_B
            reg_B <= {8'b00000000, id_ir[7:0]};    // 16'b{r2,r3} to reg_B
            end

        else ;        // when `NOP or `HALT, do nothing
        end
    end

// EX_block (3rd Stage)
always @(posedge clock or negedge reset)
    begin
    if (!reset)
        begin
        mem_ir<= 16'b0000000000000000;
        reg_C <= 16'b0000000000000000;
        smdr1 <= 16'b0000000000000000;
        zf <= 1'b0;
        nf <= 1'b0;
        cf <= 1'b0;
        dw <= 1'b0;
        end
    else if(state==`exec)
        begin
        if ( Jump_Flush_Flag )		// flush mem_ir=0
            mem_ir <= 16'b0000000000000000;
        else
            mem_ir <= ex_ir;

        reg_C  <= ALUo[15:0];    // reg_C is 16-bit, so cut off the ALUo
                                // the MSD-ALUo[16] is carry-flag---CF
                                // the rest bits of ALUo, ALUo[15:0] store the data
        //Change the flag : ZF & NF
        if ((ex_ir[15:11] == `ADD)  || (ex_ir[15:11] == `ADDI)  || (ex_ir[15:11] == `ADDC)  ||
            (ex_ir[15:11] == `SUB)  || (ex_ir[15:11] == `SUBI)  || (ex_ir[15:11] == `SUBC)  || (ex_ir[15:11] == `CMP) ||
            (ex_ir[15:11] == `AND)  || (ex_ir[15:11] == `OR )   || (ex_ir[15:11] == `XOR)   ||
            (ex_ir[15:11] == `SLL)  || (ex_ir[15:11] == `SRL)   || (ex_ir[15:11] == `SLA)   || (ex_ir[15:11] == `SRA) ||
            (ex_ir[15:11] == `COPY) || (ex_ir[15:11] == `MUL)
           )
            begin
            if (ALUo[15:0] == 16'b0000000000000000)
                zf <= 1'b1;
            else
                zf <= 1'b0;
            if (ALUo [15] == 1'b1)
                nf <= 1'b1;
            else
                nf <= 1'b0;
            end

        //Change the flag : CF
        if ((ex_ir[15:11] == `ADD) || (ex_ir[15:11] == `ADDI) || (ex_ir[15:11] == `ADDC) ||
            (ex_ir[15:11] == `SUB) || (ex_ir[15:11] == `SUBI) || (ex_ir[15:11] == `SUBC) ||
            (ex_ir[15:11] == `CMP) || (ex_ir[15:11] == `MUL)
           )
            cf <= ALUo[16];
        else if ((ex_ir[15:11] == `AND) || (ex_ir[15:11] == `OR)  || (ex_ir[15:11] == `XOR))
            cf <= 1'b0;
        else if ((ex_ir[15:11] == `SLL) || (ex_ir[15:11] == `SRL) ||    // CF doesn't change for shift operations
                 (ex_ir[15:11] == `SLA) || (ex_ir[15:11] == `SRA) ||
                 (ex_ir[15:11] == `COPY)
                )
             ;    // don't do anything, not change the value of CF
        else ;    // don't do anything

        // Change dw & smdr1 only when 'STORE happens
        // Adding ' !Jump_Flush_Flag &&' is for preventing for the case when
        // JUMP xxx;
        // STORE xxx; happens, avoid STORE executing
        if ( !Jump_Flush_Flag && ex_ir[15:11] == `STORE)
            begin
            dw <= 1'b1;
            smdr1 <= smdr;
            end
        else
            dw <= 1'b0;
        end
    end

// MEM_block (4th Stage)
always @(posedge clock or negedge reset)
    begin
    if (!reset)
        begin
        wb_ir  <= 16'b0000000000000000 ;
        reg_C1 <= 16'b0000000000000000 ;
        end
    else if(state==`exec)
        begin
        wb_ir  <= mem_ir;
        if (mem_ir[15:11] == `LOAD)
            reg_C1 <= d_datain;
        else
            reg_C1 <= reg_C;
        end
    end

// WB_block (5th Stage)
always @(posedge clock or negedge reset)
    begin
    if (!reset)
        begin
        gr[0] <= 16'b0000000000000000;
        gr[1] <= 16'b0000000000000000;
        gr[2] <= 16'b0000000000000000;
        gr[3] <= 16'b0000000000000000;
        gr[4] <= 16'b0000000000000000;
        gr[5] <= 16'b0000000000000000;
        gr[6] <= 16'b0000000000000000;
        gr[7] <= 16'b0000000000000000;
        end
    else if(state==`exec)
        begin
        if (wb_ir[10:8] != 3'b000)        // gr[0] is all-0 constant register, it's read-only
            if ((wb_ir[15:11] == `LOAD) || (wb_ir[15:11] == `LDIH) || (wb_ir[15:11] == `ADD) ||
                (wb_ir[15:11] == `ADDI) || (wb_ir[15:11] == `ADDC) || (wb_ir[15:11] == `SUB) ||
                (wb_ir[15:11] == `SUBI) || (wb_ir[15:11] == `SUBC) ||
                (wb_ir[15:11] == `AND)  || (wb_ir[15:11] == `OR)   || (wb_ir[15:11] == `XOR) ||
                (wb_ir[15:11] == `SLL)  || (wb_ir[15:11] == `SRL)  ||
                (wb_ir[15:11] == `SLA)  || (wb_ir[15:11] == `SRA)  || (wb_ir[15:11] == `COPY)|| (wb_ir[15:11] == `MUL)
               )                        // 'CMP do not change the gr[r1]
                gr[wb_ir[10:8]] <= reg_C1;
        end
    end

// ALU_block
always @(reg_A or reg_B or ex_ir[15:11])
    begin
    temp_reg_ALUo_SLA_SRA <= 15'b000000000000000;
    store_add_in_Mem <= 16'b0000000000000000;
	// initial state for `MUL
	stored0 <= 16'b0000000000000000;
	stored1 <= 16'b0000000000000000;
	stored2 <= 16'b0000000000000000;
	stored3 <= 16'b0000000000000000;
	stored4 <= 16'b0000000000000000;
	stored5 <= 16'b0000000000000000;
	stored6 <= 16'b0000000000000000;
	stored7 <= 16'b0000000000000000;
	add01 <= 16'b0000000000000000;
	add23 <= 16'b0000000000000000;
	add45 <= 16'b0000000000000000;
	add67 <= 16'b0000000000000000;
	add0123 <= 16'b0000000000000000;
	add4567 <= 16'b0000000000000000;
	add01234567 <= 16'b0000000000000000;

    case (ex_ir[15:11])
        `NOP   : ;
        `HALT  : ;
        `LOAD  : ALUo = {1'b0,reg_A} + {1'b0,reg_B};
        `STORE : begin
                 ALUo = {1'b0,reg_A} + {1'b0,reg_B};
                 store_add_in_Mem <= ALUo[15:0];
                 end
        `LDIH  : ALUo = {1'b0,reg_B[7:0],8'b00000000};

        `ADD   : ALUo = {1'b0,reg_A} + {1'b0,reg_B};
        `ADDI  : ALUo = {1'b0,reg_A} + {1'b0,reg_B};
        `ADDC  : ALUo = {1'b0,reg_A} + {1'b0,reg_B}+{16'b0000000000000000,cf};
        `SUB   : ALUo = {1'b0,reg_A} - {1'b0,reg_B};
        `SUBI  : ALUo = {1'b0,reg_A} - {1'b0,reg_B};
        `SUBC  : ALUo = {1'b0,reg_A} - {1'b0,reg_B}-{16'b0000000000000000,cf};
        `CMP   : ALUo = {1'b0,reg_A} - {1'b0,reg_B};

        `AND   : ALUo = {1'b0,reg_A} & {1'b0,reg_B};
        `OR    : ALUo = {1'b0,reg_A} | {1'b0,reg_B};
        `XOR   : ALUo = {1'b0,reg_A} ^ {1'b0,reg_B};

        `SLL   : ALUo = {1'b0,reg_A << reg_B};
        `SRL   : ALUo = {1'b0,reg_A >> reg_B};

        `SLA   : begin
                 temp_reg_ALUo_SLA_SRA = {reg_A[14:0] << reg_B};
                 ALUo = {1'b0,reg_A[15],temp_reg_ALUo_SLA_SRA};
                 end
        `SRA   : begin
                 if (!reg_A[15])    // reg_A[15] is 0
                                    // then reg_A is positive, it's same as SRL
                    ALUo = {1'b0,reg_A >> reg_B};
                 else                // reg_A[15] is 1
                    case(reg_B[3:0])
                        4'b0000 : ALUo = {1'b0, {1{1'b1}},  reg_A[14:0]};
                        4'b0001 : ALUo = {1'b0, {2{1'b1}},  reg_A[14:1]};
                        4'b0010 : ALUo = {1'b0, {3{1'b1}},  reg_A[14:2]};
                        4'b0011 : ALUo = {1'b0, {4{1'b1}},  reg_A[14:3]};
                        4'b0100 : ALUo = {1'b0, {5{1'b1}},  reg_A[14:4]};
                        4'b0101 : ALUo = {1'b0, {6{1'b1}},  reg_A[14:5]};
                        4'b0110 : ALUo = {1'b0, {7{1'b1}},  reg_A[14:6]};
                        4'b0111 : ALUo = {1'b0, {8{1'b1}},  reg_A[14:7]};
                        4'b1000 : ALUo = {1'b0, {9{1'b1}},  reg_A[14:8]};
                        4'b1001 : ALUo = {1'b0, {10{1'b1}}, reg_A[14:9]};
                        4'b1010 : ALUo = {1'b0, {11{1'b1}}, reg_A[14:10]};
                        4'b1011 : ALUo = {1'b0, {12{1'b1}}, reg_A[14:11]};
                        4'b1100 : ALUo = {1'b0, {13{1'b1}}, reg_A[14:12]};
                        4'b1101 : ALUo = {1'b0, {14{1'b1}}, reg_A[14:13]};
                        4'b1110 : ALUo = {1'b0, {15{1'b1}}, reg_A[14]    };
                        4'b1111 : ALUo = {1'b0, {16{1'b1}}               };
                        default : ALUo = 17'bXXXXXXXXXXXXXXXXX;
                    endcase
                 end
            // Warning! The expression below is error!
            // ALUo = {1'b0, reg_A[15], reg_B{reg_A[15]}}, reg_A[14:reg_B]}
            // Cuz reg_B must be constant value, in bit-operations, must not use variables

        `JUMP  : ALUo = {1'b0,8'b00000000,reg_B[7:0]};
        `JMPR  : ALUo = {1'b0,reg_A} + {1'b0,reg_B};
        `BZ    : ALUo = {1'b0,reg_A} + {1'b0,reg_B};
        `BNZ   : ALUo = {1'b0,reg_A} + {1'b0,reg_B};
        `BN    : ALUo = {1'b0,reg_A} + {1'b0,reg_B};
        `BNN   : ALUo = {1'b0,reg_A} + {1'b0,reg_B};
        `BC    : ALUo = {1'b0,reg_A} + {1'b0,reg_B};
        `BNC   : ALUo = {1'b0,reg_A} + {1'b0,reg_B};
        `COPY  : ALUo = {1'b0,reg_A} + {1'b0,reg_B};
        `MUL   : begin
				 stored0 = reg_B[0]? {8'b00000000, reg_A[7:0]     } : 16'b0000000000000000;
				 stored1 = reg_B[1]? {7'b0000000, reg_A[7:0], 1'b0} : 16'b0000000000000000;
				 stored2 = reg_B[2]? {6'b000000, reg_A[7:0], 2'b00} : 16'b0000000000000000;
				 stored3 = reg_B[3]? {5'b00000, reg_A[7:0], 3'b000} : 16'b0000000000000000;
				 stored4 = reg_B[4]? {4'b0000, reg_A[7:0], 4'b0000} : 16'b0000000000000000;
				 stored5 = reg_B[5]? {3'b000, reg_A[7:0], 5'b00000} : 16'b0000000000000000;
				 stored6 = reg_B[6]? {2'b00, reg_A[7:0], 6'b000000} : 16'b0000000000000000;
				 stored7 = reg_B[7]? {1'b0, reg_A[7:0], 7'b0000000} : 16'b0000000000000000;
				 add01 = stored0 + stored1;
				 add23 = stored2 + stored3;
				 add45 = stored4 + stored5;
				 add67 = stored6 + stored7;
				 add0123 = add01 + add23;
				 add4567 = add45 + add67;
				 add01234567 = add0123 +add4567;
				 ALUo = {1'b0,add01234567};
				 end

        default : ALUo = 17'bXXXXXXXXXXXXXXXXX;
    endcase
    end

// Debugging
always @(select_y or gr[1] or gr[2] or gr[3] or gr[4] or gr[5] or gr[6]
         or gr[7] or reg_A or reg_B or reg_C or reg_C1 or smdr or id_ir
         or dw or zf or nf or cf or pc)        // Used for test
    begin
        case (select_y)
            4'b0000 : y = {3'b000, dw, 1'b0, zf, nf, cf, pc};
            4'b0001 : y = gr[1];
            4'b0010 : y = gr[2];
            4'b0011 : y = gr[3];
            4'b0100 : y = gr[4];
            4'b0101 : y = gr[5];
            4'b0110 : y = gr[6];
            4'b0111 : y = gr[7];
            4'b1000 : y = reg_A;
            4'b1001 : y = reg_B;
            4'b1011 : y = reg_C;
            4'b1100 : y = reg_C1;
            4'b1101 : y = smdr;
            4'b1110 : y = id_ir;
            default : y = 16'bXXXXXXXXXXXXXXXX ;
        endcase
    end

endmodule
