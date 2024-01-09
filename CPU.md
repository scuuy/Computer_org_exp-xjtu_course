# <center>实验六 CPU综合设计

### 一、 **实验目的**

1 掌握复杂系统设计方法。

2 深刻理解计算机系统硬件原理。



### **二、实验内容**

1）设计一个基于MIPS指令集的CPU，支持以下指令：{add, sub, addi,  lw, sw, beq, j, nop}；

2）CPU需要包含寄存器组、RAM模块、ALU模块、指令译码模块；

3）该CPU能运行基本的汇编指令；（D~C+）

以下为可选内容：

4）实现多周期CPU（B-~B+）；

5）实现以下高级功能之一（A-~A+）：

(1)实现5级流水线CPU；

(2)实现超标量；

(3)实现4路组相联缓存；

可基于RISC V 、ARM指令集实现。

**如发现代码为抄袭代码，成绩一律按不及格处理。**



### **三、实验要求**

编写相应测试程序，完成所有指令测试。

本次实验需要完成基于MIPS指令集的CPU,其中需要完成以下指令

```verilog
{add, sub, addi,  lw, sw, beq, j, nop}
```

各指令格式如下

![img](https://img-blog.csdnimg.cn/img_convert/ee13be8639960bb50f578d679f922e4d.png)

- R-Type:

| Instruction | Opcode/Function | Syntax       | Operation    |
| ----------- | --------------- | ------------ | ------------ |
| add         | 000000/100000   | f $d, $s, $t | $d = $s + $t |
| sub         | 000000/100010   | f $d, $s, $t | $d = $s - $t |

- I-Type:

| I-type | op     | rs   | rt   | immediate | **示例**       | **示例含义**              |
| ------ | ------ | ---- | ---- | --------- | -------------- | ------------------------- |
| addi   | 001000 | rs   | rt   | immediate | addi $1,$2,100 | $1=$2+100                 |
| lw     | 100011 | rs   | rt   | immediate | lw $1,10($2)   | $1=memory[$2  +10]        |
| sw     | 101011 | rs   | rt   | immediate | sw $1,10($2)   | memory[$2+10]  =$1        |
| beq    | 000100 | rs   | rt   | immediate | beq $1,$2,10   | if($1==$2)   goto PC+4+40 |

- J-Type:

| J-type | op     | address | **示例** | **示例含义**         |
| ------ | ------ | ------- | -------- | -------------------- |
| j      | 000010 | address | j 10000  | goto 10000(有PC扩展) |

- NOP：

nop：空指令，不执行任何操作。
操作码：000000

> ALU 操作码：
> 00000：加法
> 00001：减法
> 00010：逻辑与
> 00011：逻辑或
> 00100：逻辑非



### **四、实验代码及结果**

*本次实验完成32位单周期CPU和五级流水CPU的设计，分别进行分析。*

#### 1. 单周期CPU

单周期CPU的原理图如图

<img src="C:\Users\ROG\AppData\Roaming\Typora\typora-user-images\image-20231224120156895.png" alt="image-20231224120156895" style="zoom:67%;" />

下面先对design代码中主要部分进行分析

代码共实例化以下5个模块。

<img src="C:\Users\ROG\AppData\Roaming\Typora\typora-user-images\image-20231224120726694.png" alt="image-20231224120726694" style="zoom:50%;" />

先对宏观控制模块`SimpleMIPSCPU`的design代码进行分析

##### 1.0 单周期主模块

```verilog
module SimpleMIPSCPU(
    input wire clk,    // 时钟
    input wire rst,    // 复位
    output reg [31:0] result,  // 输出结果
    output wire [31:0] instruction //当前指令
    //默认，初始PC=0
);
```

- 端口定义，包含输入的时钟信号和复位信号，result保存alu运算结果或`load`读取的值等，由于是单周期CPU，因此result将与时钟信号同频变化。 输出当前指令instruction用于调试代码，检查是否成功读入指令。

```verilog
// 寄存器写入
always @(*) begin
    if (rst) begin end
      // 复位时将寄存器清零,在子模块通过RST信号自动清零
     else if (RegWrite) begin
        // 写入寄存器   add sub addi lw 
        // 其中add sub写入rd      lw,addi写入rt
        //只有lw从外存写入Reg，其余均为ALU写入Reg
        WriteData1 <= MemtoReg ? result1 : alu_result;
        //MemtoReg=1 : lw
        //MemtoReg=0 : add sub addi
    end
end
```

- 此处是寄存器写入的循环代码。这里不是经历一个时钟沿直接写入`Regfile`，而是修改`WriteData`值，将写入值总线值进行修改，因此此处使用了`always@(*)`而并非`clk`，是保证在一个周期内，当运算结果得到后，能第一时间修改总线值。
- 当`WriteData`修改为该周期的运算结果后，在下一个时钟沿（实例化的`Regfile`中），寄存器写入该值。

- 复位信号是优先判定的，当然，此处可以做空操作，因为已经向子模块传递了`rst`信号，因此不必在此处理。

- `result1`中存储从Memory中读取的值，而`alu_result`存储ALU运算结果。

```verilog
always @* begin
    // 从指令中提取字段
    opcode = instruction[31:26];
    rs = instruction[25:21];
    rt = instruction[20:16];
    rd = instruction[15:11];
    Funct = instruction[5:0];
    // 生成控制信号
end
```

- 此处是一个周期中的指令译码阶段，根据MIPS指令格式，先将指令划分为`OP,Rs,Rt,Rd,Func`，再根据控制模块收到`OP`后发出的控制信号，决定真正的操作数。

```verilog
// 同步复位
always @(posedge clk or posedge rst) begin
    if (rst) begin
        // 复位时初始化
        result <= 32'b0;
    end else begin
        // 输出结果
        result <= MemtoReg ? result1 : alu_result;
        //$display("%d,%d,%d",MemtoReg,result1,alu_result);
    end
end
```

- 此处是对输出结果`result`的循环修改，根据复位信号的有无刷新，或根据该时钟周期内的运算结果对`result`进行更新，result并没有在后续子模块中使用，仅仅为了在主模块中显示并调试。

```verilog
//PC计数器
//assign PC = PC1;//中间变量，实现同时实例化和循环判断
always @(posedge clk or posedge rst)begin
    if(rst)PC1=32'b0;
    else begin
        if(Branch&&alu_result==0)PC1=PC1+32'b100+immediate_sigext;
        else if(Jump)begin
            PC1 = PC1 + 4;
            PC1 = {PC[31:28],immediate_sigext[27:0]};
        end
        else PC1 = PC1 + 4;
    end
    //$display("PC is %d, PC1 is %d",PC,PC1);
end
```

*ps：该CPU采用字节寻址方式，因此下一条指令的位置是PC+4而非PC+1*

- 此处是程序计数器PC的更新循环，在设计时，并没有将其设置成独立的子模块，而是用verilog的行为级描述方式直接操作PC，这样更简洁。

- 优先判定复位，将计数器清零（重新读取第一条指令）。接着在每个时钟沿，根据控制信号判定对PC执行的操作，分别有：

  - Branch：是对beq指令的判定，如果控制器发出了Branch信号，并且rs和rt相等（alu_result=0)，则对PC进行条件转移，根据定义，首先对`PC+4`,接着再加上16位立即数扩展为32位后的值，扩展代码如下:

  ```verilog
  // 立即数生成
  assign immediate = instruction[15:0];
  assign immediate_sigext = {14'b0,immediate[15:0],2'b0};
  ```

  ​		注意此时扩展至32位是对立即数x4（即左移两位），然后对其余位补零。

  - Jump：根据控制器发出的Jump信号，判定此时为直接跳转指令，因此先对PC+4，再根据定义，对PC值进行拼接操作，即PC高4位与立即数左移后的低28位拼接，得到新的PC值，这里同样是立即数左移2位，因此直接借用`immediate_sigext`的低28位即可。
  - 无跳转：对PC+4更新即可。

单周期主模块完整代码如下

```verilog
module SimpleMIPSCPU(
    input wire clk,    // 时钟
    input wire rst,    // 复位
    output reg [31:0] result,  // 输出结果
    output wire [31:0] instruction //当前指令
    //默认，初始PC=0
);
initial result = 32'b0;

wire [31:0] PC;
reg [31:0] PC1;
initial PC1=0;  //PC初始化0
assign PC = PC1;

IMem IM(.A(PC), .RD(instruction));
wire [31:0] result1,WriteData,RD1,RD2;
reg [31:0] WriteData1;
assign WriteData = WriteData1;  //中间变量，用来分别实例化和always赋值

wire [4:0] WriteReg;
//RD1=rs  RD2=rt
// 控制信号
wire RegWrite, MemtoReg, MemWrite, ALUSrc, Branch, Jump, Zero,RegDst,CF;
reg [5:0] opcode,Funct;
reg [4:0] rs, rt, rd;
wire [4:0] ALUControl;
integer i;

wire [31:0] immediate,immediate_sigext,alu_result;//立即数，扩展2位后立即数，ALU结果

 // 寄存器文件
RegFile regfile(
     .RST(rst),
     .CLK(clk),
     .WE3(RegWrite),
     .RA1(rs),
     .RA2(rt), 
     .WA3(WriteReg),
     .WD3(WriteData),
     .RD1(RD1),
     .RD2(RD2)
);

// 运算单元
ALU alu (
    .A(RD1),
    .B(ALUSrc ? immediate : RD2),
    .OP(ALUControl),
    .F(alu_result),
    .CF(CF)
);
// 主控制单元
Controller ctrl (
    .Op(opcode),
    .Funct(Funct),
    .Zero(Zero),
    .RegDst(RegDst),
    .RegWrite(RegWrite),
    .MemToReg(MemtoReg),
    .MemWrite(MemWrite),
    .ALUSrc(ALUSrc),
    .Jump(Jump),
    .ALUControl(ALUControl)
);
// 数据存储单元,只有sw指令用到
DataMemory dmem (  // 1Kx32bit
    .clk(clk),
    .address(alu_result),
    .write_data(RD2),
    .mem_read(MemWrite),  //R_W使用一个信号控制
    .mem_write(MemWrite),
    .read_data(result1)
);
assign WriteReg = RegDst ? rd:rt;
// 寄存器写入
always @(*) begin
    if (rst) begin end
      // 复位时将寄存器清零,在子模块通过RST信号自动清零
     else if (RegWrite) begin
        // 写入寄存器   add sub addi lw 
        // 其中add sub写入rd      lw,addi写入rt
        //只有lw从外存写入Reg，其余均为ALU写入Reg
        WriteData1 <= MemtoReg ? result1 : alu_result;
        //MemtoReg=1 : lw
        //MemtoReg=0 : add sub addi
    end
end
// 立即数生成
assign immediate = instruction[15:0];
assign immediate_sigext = {14'b0,immediate[15:0],2'b0};
// 控制信号生成
assign Branch = ctrl.Branch;
always @* begin
    // 从指令中提取字段
    opcode = instruction[31:26];
    rs = instruction[25:21];
    rt = instruction[20:16];
    rd = instruction[15:11];
    Funct = instruction[5:0];
    // 生成控制信号
end
// 同步复位
always @(posedge clk or posedge rst) begin
    if (rst) begin
        // 复位时初始化
        result <= 32'b0;
    end else begin
        // 输出结果
        result <= MemtoReg ? result1 : alu_result;
        //$display("%d,%d,%d",MemtoReg,result1,alu_result);
    end
end
//PC计数器
//assign PC = PC1;//中间变量，实现同时实例化和循环判断
always @(posedge clk or posedge rst)begin
    if(rst)PC1=32'b0;
    else begin
        if(Branch&&alu_result==0)PC1=PC1+32'b100+immediate_sigext;
        else if(Jump)begin
            PC1 = PC1 + 4;
            PC1 = {PC[31:28],immediate_sigext[27:0]};
        end
        else PC1 = PC1 + 4;
    end
    //$display("PC is %d, PC1 is %d",PC,PC1);
end
endmodule
```

除了已经分析过的主要部分，现对其余部分进行简单分析如下

- PC通过PC1间接赋值，因为有实例化必须是wire类型的问题存在，WriteData和WriteData1同理。

- WriteReg是写入寄存器的编号，这里要根据控制信号判定是写入rd还是rt寄存器，如下

  - ```verilog
    assign WriteReg = RegDst ? rd:rt;
    ```

下面分别对各子模块分析，之前实验分析过的部分不再赘述。

##### 1.1 IM指令寄存器

design代码如下

```verilog
`define D_WIDTH 32
module IMem(
	input [31:0] A, 
	output [`D_WIDTH-1:0] RD);
	parameter IMEM_SIZE = 32;
	reg[`D_WIDTH-1:0] RAM[IMEM_SIZE-1:0];
	initial
	$readmemb("D:/草稿箱/计组实验/experiment_6/instructions.txt",RAM);
	//读入指令码
	assign RD = RAM[A/4];
endmodule
```

- 修改宏观变量值，此处设计的是32个32位的寄存器，通过`readmemb`函数以二进制方式读取存储在本地的指令。
- `A/4`是字节寻址模式下，某条指令的首地址，例如PC=0+4=4,则A/4=1,是IM中第二条指令。

##### 1.2 RegFile寄存器组

```verilog
`define DATA_WIDTH 32
module RegFile
	#(parameter ADDR_SIZE = 5)
	(input RST, CLK, WE3,
	input [ADDR_SIZE-1:0] RA1, RA2, WA3,
	input [`DATA_WIDTH-1:0] WD3,
	output [`DATA_WIDTH-1:0] RD1, RD2);
	reg [`DATA_WIDTH-1:0] rf[2 ** ADDR_SIZE-1:0];
	integer i;
	initial 
	for(i=0;i<2**ADDR_SIZE-1;i=i+1)rf[i]=0;
	always@(posedge CLK,posedge RST)begin
	   $display("i = %d, data is %d",WA3,WD3);
	    if(RST)for(i=0;i<2**ADDR_SIZE-1;i=i+1)rf[i]=0;
		else if(WE3) rf[WA3] = WD3;
    end
	assign RD1 = (RA1 != 0) ? rf[RA1] : 0;
	assign RD2 = (RA2 != 0) ? rf[RA2] : 0;
endmodule
```

- rf二元数组充当寄存器文件，大小为32x32位，WA3控制写入地址，WD3控制写入数据，WE3写入信号。
- RD1和RD2是两个输出端口，当RA地址不为0时，将RD1和RD2接在rf[RA1]和rf[RA2]上，输出这两个位置的值,否则默认为0。
- 添加initial初始化，对寄存器的内容初始化为0。

##### 1.3 ALU

```verilog
module ALU(F,CF,A,B,OP);
	parameter SIZE=32;
	output reg [SIZE-1:0]F;
	output CF;
	input [SIZE-1:0]A,B;
	input [4:0]OP;
    //功能定义
	parameter ALU_AND =5'B00010;
	parameter ALU_OR =5'B00011;
	//parameter ALU_XOR =5'B010;
	parameter ALU_NOR =5'B00100;
	parameter ALU_ADD =5'B00000;
	parameter ALU_SUB =5'B00001;
	//parameter ALU_SLT =5'B110;
	//parameter ALU_SLL =5'B111;
	reg [7:0]EN;
	wire [7:0]EN1;
    assign EN1=EN;
	wire [SIZE-1:0]Fw,Fa;
	assign Fa=A&B;
	always@(*)begin
	EN=0;
	case(OP)
		ALU_AND :begin F=Fa; end
		ALU_OR :begin F=A|B; end
		ALU_NOR :begin F=~(A|B); end
		ALU_ADD : begin EN[4]=1;F=Fw;end
		ALU_SUB : begin EN[5]=1;F=Fw;end
		default: F=Fw;
	endcase
	//$display("F is %d,A %d,B %d,OP is %d",F,A,B,OP);
end
add32 add32(A,B,'b0,Fw,CF,EN1[4]);
SUB sub_1(Fw,CF,A,B,EN1[5]);
endmodule
```

- 此处要在过往实验的ALU模块上修改，首先是编码不同，该实验用到的编码是5位ALU信号，根据表格修改，并且删去移位等功能，原使能方式对加法器和减法器使能方式不变，因此直接将`case`中对应执行的语句变成使能有效，在初始化时使能无效即可。
- 其中加法器通过半加器->1位全加器->4位全加器->32位全加器实现，代码见附录。

##### 1.4 Controller控制器

```verilog
module Controller(
	input [5:0] Op,Funct,
	input Zero,
	output MemToReg, MemWrite,
	output PCSrc,ALUSrc,
	output RegDst,RegWrite,
	output Jump,
	output [4:0] ALUControl);
	wire[1:0] ALUOp;
	wire Branch;
	MainDec mainDec(Op,MemToReg,MemWrite,Branch,ALUSrc,RegDst,RegWrite,Jump,ALUOp);
	ALUDec aludec(Funct,ALUOp,ALUControl);
	assign PCSrc=Branch & Zero;
endmodule
```

- 此处是宏观控制器调用两个子控制器，分别发出控制信号和ALU译码信号。子模块定义见附录。
- 注意到，在该代码中，Branch并不是模块的端口，而是中间变量，因此在CPU主模块中通过以下方式调用

```verilog
assign Branch = ctrl.Branch;
```

##### 1.5 Memory

```verilog
module DataMemory (
    input wire clk,
    input wire [31:0] address,  // 地址 //bug
    input wire [31:0] write_data,  // 写入数据
    input wire mem_read,  // 读取使能
    input wire mem_write,  // 写入使能
    output reg [31:0] read_data  // 读取数据
);
integer i;
// 内存
reg [31:0] memory [0:1023];     //1Kx32bit
initial begin 
    for(i=0;i<1024;i=i+1)memory[i]=0;
end
// 内存读写操作
always @(*) begin
    if (!mem_read) begin
        // 读取内存
        read_data <= memory[address];
        //$display("result1 is : %d",read_data);
    end
end
always @(negedge clk)begin
    if (mem_write) begin
        // 写入内存
        memory[address] <= write_data;
    end
end
endmodule
```

- 1Kx32bitRAM的实现，有读写使能信号（事实上CPU主模块只用了一个变量，1写入，0读出）。
- 注意到这里写入内存不是在时钟上升沿，而是下降沿`negedge`,这样设计是因为调试中发现了冲突，因此错开时序，不影响单周期的特性。

##### 1.6 simulation

sim代码如下

```verilog
module sim_SimpleMIPSCPU;
reg clk;    // 时钟
reg rst;    // 复位
wire [31:0] result;  // 输出结果,默认初始PC=0
wire [31:0] op,PC,Mem_0;
reg [31:0] r1,r2,r3,r4,r5;
wire [4:0] WriteReg;
wire MemWrite;
integer i;
SimpleMIPSCPU Simple_CPU(clk,rst,result,op);
assign PC = SimpleMIPSCPU.PC;
assign Mem_0 = SimpleMIPSCPU.dmem.memory[0];
assign WriteReg = SimpleMIPSCPU.WriteReg;
assign MemWrite = SimpleMIPSCPU.MemWrite;
always @ * begin
    r1 <= SimpleMIPSCPU.regfile.rf[1];
    r2 <= SimpleMIPSCPU.regfile.rf[2];
    r3 <= SimpleMIPSCPU.regfile.rf[3];
    r4 <= SimpleMIPSCPU.regfile.rf[4];
    r5 <= SimpleMIPSCPU.regfile.rf[5];
    end
initial begin
    clk=0;rst=0;i=0;
    fork
        repeat(500)#5 i=i+1;
        repeat(500)#5 clk=~clk;
        repeat(1)begin
            #2000 rst=1;
            #5 rst=0;
        end
    join
end
endmodule
```

- 只需循环改变时钟信号和复位信号即可，并在sim中添加了一些想要观察到的中间值，用于调试代码。

指令寄存器读取的指令如下

<img src="C:\Users\ROG\AppData\Roaming\Typora\typora-user-images\image-20231224143438516.png" alt="image-20231224143438516" style="zoom:67%;" />

运行结果如图

![image-20231224143423280](C:\Users\ROG\AppData\Roaming\Typora\typora-user-images\image-20231224143423280.png)

- 观察op和程序计数器，PC=0,读入第一条空指令

```verilog
// nop					(0000_0000H)
000000_00000_00000_0000_0000_0000_0000	
```

- PC=4,读入第二条指令

```verilog
// R-type add 				(0022_1820H)
//$3 = $1 + $2                $1=0, $2=0 ,因此$3=0
000000_00001_00010_00011_00000_100000  
```

此时寄存器被选中，但是值仍然为0.

- PC=8,读入第三条指令

```verilog
//I-type  addi  $2 = $1 +5        $2=5, $1=0 	(2022_0005H)
001000_00001_00010_0000_0000_0000_0101
```

该立即数指令，将2号寄存器值修改为5，可以观察到，在下个时钟上升沿，2号寄存器值变为5。

- PC=cH,读入第四条指令

```verilog
//I-type $3 = $1 +2        $3=2, $1=0		(2023_0002H)
001000_00001_00011_0000_0000_0000_0010
```

立即数指令，将3号寄存器值修改为2，可以观察到下个时钟沿成功赋值。

- PC=10H,读入第五条指令

```verilog
//2号寄存器存入内存,memory[$1=0]中,即memory[0]=5
// 					(AC22_0000H)
101011_00001_00010_0000_0000_0000_0000	
```

该指令为save指令，内存地址为一号寄存器的值0，因此将2号寄存器的值5存入存储器0号地址，可以观察到,`Mem_0`在时钟下降沿被成功赋值，符合预期。

- PC=14H,读入第六条指令

```verilog
//lw $1 = memory[$1+0=0]=5			(8C21_0000H)
100011_00001_00001_0000_0000_0000_0000	
```

load指令，将1号寄存器的值修改为刚刚存入的存储值5，可以观察到下个时钟沿成功修改。

- PC=18H,读入第七条指令

```verilog
//beq  $3!=$4					(1083_0010H)
000100_00100_00011_0000_0000_0001_0000	
```

beq指令，由于3号寄存器和4号寄存器值不同，因此不发生跳转。

- PC=1cH,读入第八条指令

```verilog
//Jump goto 4'b0000 + 3<<2 = 'b100  PC->12=cH	(0800_0003H)
000010_0000_0000_0000_0000_0000_0000_11	
```

Jump指令，立即跳转，根据定义计算得到PC将跳转至cH位置，结果如图

![image-20231224144205794](C:\Users\ROG\AppData\Roaming\Typora\typora-user-images\image-20231224144205794.png)

光标处，PC值从1cH跳转至0cH，符合预期。

##### 1.7 RTL分析

运行RTL分析如下

![image-20231224144445460](C:\Users\ROG\AppData\Roaming\Typora\typora-user-images\image-20231224144445460.png)



#### 2.五级流水CPU

五级流水CPU结构图如下

![image-20231206221054649](C:\Users\ROG\AppData\Roaming\Typora\typora-user-images\image-20231206221054649.png)

一条指令分为五个时钟周期，分别处理取指、译码、执行、访存、写回寄存器。因此，模块设计也分为一个主模块，以及5个子模块，外加一个冒险处理模块。

控制信号由每个阶段单独执行译码OP得到控制信号。

##### 2.0 主模块flow_main

代码如下

```verilog
module flow_main(
    input wire clk,    // 时钟
    input wire rst    // 复位
);
    wire [31:0] PC;
    wire [31:0] next_pc;   //FI级缓存PC
    wire branch,branch_taken;       //分支，分支成功判定
    wire [31:0] target_address;     //跳转地址或分支地址
    
    // 定义流水线寄存器 一级
    wire [31:0] instruction1;       //IR1 
    wire [31:0] pc1;                //NPC1 （一级寄存器中）   

    // 实例化五个阶段+4组级间寄存器+若干器件
    wire Jump;
    //assign PC = FIstage.next_pc;
    wire RegWrite;
    wire [4:0]rs1,rt1,WriteReg;
    wire [31:0] WriteData,RD1,RD2;
InstructionFetchStage FIstage(
    .clk(clk),
    .rst(rst),
    .pc(PC),
    .branch(branch),
    .Jump(Jump),
    .branch_taken(branch_taken),
    .target_address(target_address),
    .next_pc(next_pc),
    .instruction1(instruction1)
    );

// 寄存器文件
RegFile_delay regfile(
     .RST(rst),
     .CLK(clk),
     .WE3(RegWrite),
     .RA1(rs1),
     .RA2(rt1), 
     .WA3(WriteReg),
     .WD3(WriteData),
     .RD1(RD1),
     .RD2(RD2)
);

wire [31:0] immediate2,instruction2,npc1,rs2,rt2;

IDstage idstage (
    .clk(clk),
    .pc_next(PC),
    .RD1(RD1),
    .RD2(RD2),
    .instruction1(instruction1),
    .rs2(rs2),
    .rt2(rt2),
    .immediate2(immediate2),
    .instruction2(instruction2),
    .npc1(npc1)
);

assign rs1 = instruction2[25:21];
assign rt1 = instruction2[20:16]; //FI/ID -> RegFile读入端

wire ALUSrc,ALU_Z;
wire [4:0] ALUOp;
wire [31:0] npc2,npc3,ALUOut,rt3,instruction3;
wire [4:0]ex_rd,mem_rd; //EX,MEM阶段旁路寄存器
wire [1:0] forward_ex,forward_mem; //EX前推信号，MEM前推信号
wire [31:0] alu_result,mem_data,alu_result_tonext,pass_data,pass_data2;
ExecuteStage EXstage(
    .rs2_wire(regfile.rf[instruction3[25:21]]),
    .rt2_wire(regfile.rf[instruction3[25:21]]),
    .clk(clk),
    .rst(rst),
    .npc1(npc1),
    .alu_result(alu_result),
    .alu_result_tonext(alu_result_tonext),
    .mem_data(mem_data),
    //.rs2(rs2),
    //.rt2(rt2),
    .immediate2(immediate2),
    .instruction2(instruction2),
    .forward_ex(forward_ex),
    .forward_mem(forward_mem),
    .ALUSrc(ALUSrc),
    .npc3(npc3),
    .npc2(npc2),
    .ALU_Z(ALU_Z),
    .ALUOut(ALUOut),
    .rt3(rt3),
    .instruction3(instruction3),
    .EX_rd(ex_rd),
    .pass_data(pass_data)
    //output reg [31:0] alu_in_A
);

// branch taken
//
//
//
wire MemWrite,PCSrc;
wire [31:0] MemOut,ALUOut2,instruction4;

MemoryStage Memstage(
    .pass_data(pass_data),
    .alu_result_tonext(alu_result_tonext),
    .clk(clk),
    .rst(rst),
    .npc3(npc3),
    .npc2(npc2),
    .ALUOut(ALUOut),
    .alu_result(alu_result),
    .rt3(rt3),
    .instruction3(instruction3),
    .MemWrite(MemWrite),
    .ALU_Z(ALU_Z),
    .forward_mem(forward_mem),
    .mem_data(mem_data),
    .MemOut(MemOut),
    .ALUOut2(ALUOut2),
    .instruction4(instruction4),
    .Jump(Jump),
    .branch(branch),
    .target_address(target_address),
    .MEM_rd_delay(mem_rd),
    .pass_data2(pass_data2)
);

wire MemtoReg,RegDst;

WriteBackStage WBstage(
    .pass_data2(pass_data2),
    .forward_mem(forward_mem),
    .mem_data(mem_data),
    .clk(clk),
    .rst(rst),
    .MemOut(MemOut),
    .ALUOut2(ALUOut2),
    .instruction4(instruction4),
    .MemtoReg(MemtoReg),
    .RegDst(RegDst),
    .Write_data(WriteData),
    .Write_Reg(WriteReg),
    .RegWrite1(RegWrite)
);

//实例化数据前推
ForwardingUnit forward_unit(
    .clk(clk),
    .rst(rst),
    .rs1(rs1),
    .rt1(rt1),
    .rs2(instruction2[25:21]), 
    .rt2(instruction2[20:16]), 
    .ex_rd(ex_rd), // EX/MEM 阶段的目标寄存器
    .mem_rd(mem_rd), // MEM/WB 阶段的目标寄存器
    .forward_ex(forward_ex), // EX 阶段的前推信号
    .forward_mem(forward_mem) // MEM 阶段的前推信号
);

endmodule
```

- 端口： 只需提供时钟信号和复位信号。
- 一些信号量声明如下

> PC： 程序计数器，结构图中的PC寄存器。
>
> next_pc : FI级缓存PC，用于保存FI阶段获取的PC值。
>
> branch : 分支信号，由控制器发出。
>
> branch_taken: 分支成功判定，如果符合beq条件（即rs值=rt值），则branch_taken=1。
>
> Jump：跳转信号，由控制器发出。
>
> target_address：跳转目标地址，可能是beq的目标地址，或Jump的目标地址。
>
> instruction1 : 一级指令缓存， instruction_i代表中间第几级缓存指令寄存器。
>
> ex_rd,mem_rd: EX,MEM阶段旁路寄存器。
>
> forward_ex,forward_mem: EX前推信号，MEM前推信号。
>
> alu_result_tonext ：EX阶段ALU运算结果寄存器。
>
> 其余信号见代码注释。

下面分别对五个阶段子模块分析

##### 2.1 取值阶段FI

代码如下

```verilog
// Control Hazard Handling in IF Stage (Instruction Fetch)
module InstructionFetchStage (
    input wire clk,
    input wire rst,
    input wire [31:0] pc,
    input wire branch,
    input wire Jump,
    input wire branch_taken,
    input wire [31:0] target_address,
    output reg [31:0] next_pc,
    output reg [31:0] instruction1
);
    reg [31:0] pc1;
    initial pc1=0;
    wire [31:0] instruction;
    IMem IM(.A(next_pc), .RD(instruction)); //取指令，接下来做更新PC,一级寄存器操作
    initial next_pc = 0;
    // 分支预测的简单实现
    assign pc = pc1;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            next_pc = 0;
            instruction1=32'b0;
        end else begin
            instruction1 <= instruction;
            // 使用分支预测单元的信号进行分支预测
            if(branch)
                pc1 = branch ? (branch_taken ? target_address : pc1 + 4) : pc1 + 4;
            else if(Jump)
                //target_address = instruction
                pc1 = target_address;
            else
                pc1 = pc1 + 4;
            next_pc <= pc1;
            $display("pc1 is %d", pc1);
        end
    end
endmodule
```

- 优先判定复位信号rst。
- 取值阶段实例化IMem指令存储器，在该阶段取值，并执行对PC的更新操作。更新的逻辑和单周期类似，此时branch的预测和jump的预测目标地址均为target_address，由后续阶段计算传入此处。
- 为了保证时序性，always在时钟上升沿执行。

tips:这里用了一个显示中间结果的指令，用于debug:

```verilog
$display("pc1 is %d",pc1)
```

写法和c语言类似，用于展示此处pc1变量的值。

实例化的IM，RegFile具体代码见单周期CPU，此处不再写出。



##### 2.2 译码阶段ID

代码如下

```verilog
module IDstage (
    input wire clk,
    input wire [31:0] pc_next,
    input wire [31:0] RD1,
    input wire [31:0] RD2,
    input wire [31:0] instruction1,
    output reg [31:0] rs2,
    output reg [31:0] rt2,
    output reg [31:0] immediate2,
    output reg [31:0] instruction2,
    output reg [31:0] npc1
    //output wire RegWrite
);
wire Zero;
    // 主控制单元
Controller ctrl (           //根据不同指令提供不同的信号 //待完成
    .Op(instruction1[31:26]),
    .Funct(instruction1[5:0]),
    .Zero(Zero)
    //.RegDst(RegDst),
    //.RegWrite(RegWrite)
    //.MemToReg(MemtoReg),
    //.MemWrite(MemWrite),
    //.ALUSrc(ALUSrc),
    //.Jump(Jump),
    //.ALUControl(ALUControl)
);
    reg [31:0]instruction11;  //在同一个时钟信号,解决instruction1同时读写冲突
    always@(negedge clk)begin
        instruction11 = instruction1;
        //rs1_reg = instruction11[25:21];
        //rt1_reg = instruction11[20:16];
    end
    // ID 阶段的寄存器解析逻辑
    always @(posedge clk) begin
        // 从指令中提取字段,更新NPC1
        rs2 <= RD1;
        rt2 <= RD2;
        immediate2 <= {{16{1'b0}},instruction11[15:0]};
        //$display("im2 %d",immediate2);
        instruction2 <= instruction1;
        npc1 <= pc_next; 
    end
endmodule
```

- 在该阶段中实例化控制器，并对指令进行译码，在`always`中，提取立即数等，并将pc,instruction等存入下一级的级间缓存。
- 注意到，这里实例化控制器并没有传递所有端口变量，这种写法是合理的，只要保证所有的`input`变量被实例化即可。这样方便我们后续模块复用同一个控制器模块。

控制器代码同单周期CPU。



##### 2.3 执行阶段

代码如下

```verilog
// Data Hazard Handling in Execute Stage (EX)
module ExecuteStage (
    input wire [31:0] rs2_wire,
    input wire [31:0] rt2_wire,
    input wire clk,
    input wire rst,
    input wire [31:0] npc1,
    output reg [31:0] alu_result,
    output reg [31:0] alu_result_tonext,
    input wire [31:0] mem_data,
    input wire [31:0] immediate2,
    input wire [31:0] instruction2,
    input wire [1:0] forward_ex,
    input wire [1:0] forward_mem,
    input wire ALUSrc,
    //input wire [4:0] ALUOp,
    output reg [31:0] npc3,
    output reg [31:0] npc2,
    output wire ALU_Z,
    output reg [31:0] ALUOut,
    output reg [31:0] rt3,
    output reg [31:0] instruction3,
    output reg [4:0] EX_rd,
    output reg [31:0] pass_data
    //output reg [31:0] alu_in_A
);
wire [4:0] ALUControl;
wire RegDst,RegWrite;
wire [31:0] alu_result_wire;
wire [31:0] ALUOut1,rs2,rt2;
reg [31:0] rs2_reg,rt2_reg;
initial begin
    rs2_reg = 0;
    rt2_reg = 0;
end
    assign rs2 = (forward_mem==2'b10) ? mem_data : rs2_wire;
    assign rt2 = (forward_mem==2'b01) ? mem_data : rt2_wire;
//    if(forward_mem==2'b10)
//                            rs2_reg = mem_data;
//                        else if(forward_mem==2'b01)
//                            rt2_reg = mem_data;
                            
// 主控制单元
Controller ctrl (
    .Op(instruction2[31:26]),
    .Funct(instruction2[5:0]),
    .Zero(ALU_Z),
    .RegDst(RegDst),
    .RegWrite(RegWrite),
    //.MemToReg(MemtoReg),
    //.MemWrite(MemWrite),
    .ALUSrc(ALUSrc),
    //.Jump(Jump),
    .ALUControl(ALUControl)
);

ALU alu(
     .F(ALUOut1),
     .CF(ALU_Z),
     .A(rs2),
     .B(ALUSrc ? immediate2:rt2),
     .OP(ALUControl)
);   //实例化ALU

    wire [27:0] IR2_extended;
    assign IR2_extended = {instruction2[25:0],2'b00};// addr26位扩展为28位
    always @ ( ALUOut1) begin
        ALUOut = ALUOut1;
        if(forward_ex || forward_mem)pass_data = ALUOut1;
        else pass_data = 0;
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin          //复位，第三级寄存器全部清空
            //alu_in_A <= 0;
            npc3 <= 32'b0;
            npc2 <= 32'b0;
            ALUOut <= 32'b0;
            rt3 <= 5'b0;
            instruction3 <= 32'b0;
        end else begin
            alu_result_tonext = alu_result;
            alu_result = ALUOut1;
            rs2_reg = rs2_wire;
            rt2_reg = rt2_wire;
            // 使用前推单元的信号进行前推
            if(forward_ex)begin
                if(forward_ex==2'b10)
                    rs2_reg = alu_result;  //alu_result寄存器
                else if(forward_ex==2'b01)
                    rt2_reg = alu_result;      //不可能出现同时1，一次修改了两个寄存器
                //处理后续寄存器
                npc3 <= {npc1[31:28],IR2_extended[27:0]}; //Jump对应PC值
                                npc2 <= npc1 + immediate2;
                                //$display("rs2 %d rt2 %d imme2 %d ALUOut1 %d",rs2,rt2,immediate2,ALUOut1);
                                rt3 <= rt2;
                                instruction3 = instruction2;
                                if(RegWrite)
                                    EX_rd = RegDst ? instruction2[15:11] : instruction2[20:16];
                                    
            end
//            else if(forward_mem)begin
//                    if(forward_mem==2'b10)
//                        rs2_reg = mem_data;
//                    else if(forward_mem==2'b01)
//                        rt2_reg = mem_data;
//                        //处理后续寄存器
//                    npc3 <= {npc1[31:28],IR2_extended[27:0]}; //Jump对应PC值
//                                    npc2 = npc1 + immediate2;
//                                    $display("rs2 %d rt2 %d imme2 %d ALUOut1 %d",rs2,rt2,immediate2,ALUOut1);
//                                    ALUOut = ALUOut1;
//                                    rt3 = rt2;
//                                    instruction3 = instruction2;
//                                    if(RegWrite)
//                                        EX_rd = RegDst ? instruction2[15:11] : instruction2[20:16];                   
//                end
            else begin
            npc3 <= {npc1[31:28],IR2_extended[27:0]}; //Jump对应PC值
            npc2 <= npc1 + immediate2;
            rt3 <= rt2;
            instruction3 = instruction2;
            if(RegWrite)
                EX_rd = RegDst ? instruction2[15:11] : instruction2[20:16];
            end
           $display("rs2 %d rt2 %d imme2 %d ALUOut1 %d,ALUOp %d",rs2,rt2,immediate2,ALUOut1,ALUControl);
           //ALUOut = ALUOut1;
        end
    end
endmodule
```

- 该阶段有两个主要任务，一是完成执行阶段的运算，二是要判定是否有前推信号，要根据前推信号，对ALU的操作数更新，从而得到最新的正确的ALU运算结果。
- 在always中，完成对级间缓存的更新，ALU运算结果的传递，注意这里如果有前推信号，不能放在always中判定，因为处在一个时钟信号中，没办法先后正确的执行两步操作。因此采用如下写法

```verilog
  assign rs2 = (forward_mem==2'b10) ? mem_data : rs2_wire;
  assign rt2 = (forward_mem==2'b01) ? mem_data : rt2_wire;
```

这是对MEM阶段前推的判定，其中forward_mem的1在高位表示rs将被替换，低位表示rt将被替换，否则将保持原输入值。

因此，EX阶段处理的数据前推包括EX前推和MEM前推：

- **EX前推**：指令i在EX阶段得到ALU运算结果，并且是R型指令，在WB阶段会将值存入寄存器，但此时指令j处于ID阶段，并且该指令要使用指令i的计算结果，然而当指令j走到EX阶段，指令i还没有写回，因此需要特判数据前推，将指令i的值通过旁路寄存器先存下来，当指令j执行EX阶段时，判定是否有前推信号，并根据前推信号，使用上一指令i放入旁路寄存器的值。
- **MEM前推**：这里容易有一个误区，就是有没有可能存储器的值会造成需要前推的现象，例如一个指令需要写入MEM，另一个需要读入？事实上，根据笔者分析，不会出现这种情况，因为对MEM的读取操作也是需要走到MEM阶段才会执行，而在EX阶段，该指令本身就不会用到该值。事实上，这里的MEM前推，本质上仍然是EX的ALU操作数需要提前获取，具体是指，当一条指令i处于EX阶段，其运算结果将要存入寄存器，但有另一条后续指令j此时在FI取值阶段，还没有经过译码，但是后续要使用该寄存器值，不同于EX前推，两条指令相差一个时钟信号，MEM前推指令i和j相差了两个时钟信号，因此，即使指令i在此时将值存入旁路寄存器，下一个执行EX阶段的指令也不是j，等到指令j执行EX时，该信号已经消失，如何再次恢复该信号？就要用到MEM前推，即在指令i处于MEM阶段时，再次译码，判定是否可能出现MEM这种前推状况，再使用一个额外的旁路寄存器，将值单独传递给EX阶段的指令j，这样，相差1个或者2个时钟信号的数据前推就都得到妥善处理。

EX阶段实例化了控制器和ALU，同单周期模块。



##### 2.4 访存阶段MEM

代码如下

```verilog
// Data Hazard Handling in Memory Stage (MEM)
module MemoryStage (
    input wire [31:0] pass_data,
    input wire [31:0] alu_result_tonext,
    input wire clk,
    input wire rst,
    input wire [31:0] npc3,
    input wire [31:0] npc2,
    input wire [31:0] ALUOut,
    input wire [31:0] alu_result,
    input wire [31:0] rt3,
    input wire [31:0] instruction3,
    //input wire [1:0] forward_mem,
    input wire MemWrite,
    input wire ALU_Z,
    input wire [1:0] forward_mem,
    output reg [31:0] mem_data,   //mem_data寄存器   
    output reg [31:0] MemOut,
    output reg [31:0] ALUOut2,
    output reg [31:0] instruction4,
    output wire Jump,
    output wire branch,
    output wire [31:0] target_address,
    output reg [4:0] MEM_rd_delay,
    output reg [31:0] pass_data2
);
reg [4:0] MEM_rd;
initial MEM_rd = 0;
wire RegDst,RegWrite;
// 主控制单元
Controller ctrl_mem(
    .Op(instruction3[31:26]),
    .Funct(instruction3[5:0]),
    .Zero(ALU_Z),
    .RegDst(RegDst),
    .RegWrite(RegWrite),
    //.MemToReg(MemtoReg),
    .MemWrite(MemWrite),
    //.ALUSrc(ALUSrc),
    .Jump(Jump)
    //.ALUControl(ALUControl)
);
assign branch = ctrl_mem.Branch; //取branch信号
wire [31:0] MemOut_;
// 数据存储单元,只有sw指令用到
DataMemory dmem (  // 1Kx32bit
    .clk(clk),
    .address(ALUOut),
    .write_data(rt3),
    .mem_read(MemWrite),  //R_W使用一个信号控制
    .mem_write(MemWrite),   //1写0读
    .read_data(MemOut_)
);
    assign target_address = (Jump) ? npc3 :
                            (branch&&ALU_Z) ? npc2 :
                            0; 
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            MemOut <= 0;
            ALUOut2 <= 0;
            instruction4 <= 0;
        end else begin      //控制冒险，保存目标PC值
            pass_data2<=pass_data;
            mem_data <= alu_result_tonext;
            MemOut <= MemOut_;
            ALUOut2 <= ALUOut;
            //$display("ALUOut2 is %d",ALUOut2);
            instruction4 = instruction3;
            if(instruction3[31:26]==0) //R-Type
                MEM_rd <= RegDst ? instruction3[15:11]:instruction3[20:16];
            MEM_rd_delay <= MEM_rd;
            end
            // 使用前推单元的信号进行前推,数据冒险
        end
endmodule
```

- 该模块实例化了控制器，1Kx32bit存储器。
- 控制器发出该阶段的所有控制信号，存储器负责读出和写入存储器，在always中，执行对下一级间缓存的写入，此外，该阶段还要做两件事，一是前文提到的MEM前推判定，要根据指令译码结果，对是否需要更新旁路寄存器值发生MEM前推进行判定。二是，要判定分支和跳转指令，并根据定义计算目的地址，写入target_address中。具体计算逻辑如下

```verilog
    assign target_address = (Jump) ? npc3 :
                            (branch&&ALU_Z) ? npc2 :
                            0; 
```

此外，这里有一个特殊处理，因为相差了两个时钟信号，因此MEM前推有一个延期一个时钟信号的操作，保证指令j在运行到EX阶段再出现对应的控制信号，延迟一个时钟信号的细节如下

```verilog
                MEM_rd <= RegDst ? instruction3[15:11]:instruction3[20:16];
            MEM_rd_delay <= MEM_rd;
```

该模块实例化的子模块代码同单周期CPU。



##### 2.5 写回寄存器阶段WB

```verilog
module WriteBackStage (
    input wire [31:0] pass_data2,
    input wire [1:0] forward_mem,
    input wire [31:0] mem_data,
    input wire clk,
    input wire rst,
    input wire [31:0] MemOut,
    input wire [31:0] ALUOut2,
    input wire [31:0] instruction4,
    output wire MemtoReg,
    output wire RegDst,
    output reg [31:0] Write_data,
    output reg [4:0] Write_Reg,
    output reg RegWrite1
);
wire RegWrite;
    // 主控制单元
Controller_pass ctrl (
    .instruction(instruction4),
    .Op(instruction4[31:26]),
    .Funct(instruction4[5:0]),
    //.Zero(Zero),
    .RegDst(RegDst),
    .RegWrite(RegWrite),
    .MemToReg(MemtoReg)
    //.MemWrite(MemWrite),
    //.ALUSrc(ALUSrc),
    //.Jump(Jump),
    //.ALUControl(ALUControl)
);
    reg [31:0] Write_data1,Write_Reg1;
    initial begin
        Write_data = 0;
        Write_data1 = 0;
    end
    // 数据冒险解决逻辑
    always @(posedge clk,posedge rst) begin
        if (rst) begin
            Write_data1 <= 0;
            Write_Reg1 <= 0;
        end else begin
            if(RegWrite)begin
                    RegWrite1 = RegWrite;
                    Write_data = Write_data1;           //做延期一个时钟信号处理
                    Write_data1 = MemtoReg ? MemOut : ALUOut2; 
                    if(pass_data2) Write_data = pass_data2;
                    //Write_Reg = Write_Reg1;
                    Write_Reg = RegDst ? instruction4[15:11] : instruction4[20:16];  
                   // $display("Write_Reg is %d, Write_data is %d",Write_Reg,Write_data); 
//                    if(mem_data)
//                        if(!forward_mem)
//                            Write_data = mem_data;
            // 使用前推单元的信号进行前推
//            write_data <= (forward_wb == 2'b10) ? mem_data :
//                            (forward_wb == 2'b01) ? mem_data :
//                               (forward_wb == 2'b00) ? 0 : 0; // 此处需要根据实际情况确定是否需要前推
            end    
        end
    end
endmodule
```

- 该阶段只需要实例化控制器，发出写入信号，写入地址等，在Regfile子模块中将根据这些信号完成对寄存器组的更新。

- 同样，这里用到了延期一个时钟信号的处理，如下

```verilog
 if(RegWrite)begin
                    RegWrite1 = RegWrite;
                    Write_data = Write_data1;           //做延期一个时钟信号处理
                    Write_data1 = MemtoReg ? MemOut : ALUOut2; 
```

对RegWrite信号做延期处理。

- 这里实例化的控制器增加了对空指令的特判，因为空指令被判定为R型，在单周期中不存在一个周期处理两个指令的行为，因此当时是忽略对空指令的特判的，然而此处必须修改控制器代码，避免对0号寄存器错误的写入。

  宏观控制器基本不变，mainDec修改代码如下

```verilog
module MainDec_pass(
    input [31:0] instruction,
	input [5:0] Op,
	output MemToReg, MemWrite,
	output Branch, ALUSrc,
	output RegDst, RegWrite,
	output Jump,
	output [1:0] ALUOp);
	reg[8:0] Controls;
	assign{RegWrite,RegDst,ALUSrc,Branch,MemWrite,MemToReg,Jump,ALUOp}=Controls;
	always@(*)begin
	    if(instruction==0)//空指令
	       Controls<=9'b000000011;//illegal Op
	    else
		case(Op)
			6'b000000: Controls <=9'b110000010;// RTYPE
			6'b100011: Controls<=9'b101001000;//LW
			6'b101011: Controls <=9'b001010000;//SW
			6'b000100: Controls <=9'b000100001;//BEQ
			6'b001000: Controls<=9'b101000000;//ADDI
			6'b000010: Controls <=9'b000000100;//J
			default: Controls<=9'bxxxxxxxxx;//illegal Op
		endcase  
		//$display("Op is %d,Controls is %d",Op,Controls);
    end
endmodule
```

即增加了对空指令的特判，让其真正执行空操作。



##### 2.6 前推单元forward_unit

​	在设计时，本身需要考虑数据冒险和控制冒险（不需要考虑结构冒险，因为这种代码的写法本身类似于结构冗余的），因此需要对跳转和前推分别处理，但通过行为级描述方式，我们可以将控制冒险在两个阶段中解决，即MEM阶段和FI阶段，MEM阶段计算得到`target_address`，在FI阶段，则可能会将PC值修改为该值。通过这种方式便解决了控制冒险。 对于数据冒险，本实验采用数据前推的方式解决，因此设计了该前推单元子模块。

具体代码如下

```verilog
module ForwardingUnit (
    input wire clk,
    input wire rst,
    input wire [4:0] rs1,
    input wire [4:0] rt1,
    input wire [4:0] rs2, 
    input wire [4:0] rt2, 
    input wire [4:0] ex_rd, // EX/MEM 阶段的目标寄存器
    input wire [4:0] mem_rd, // MEM/WB 阶段的目标寄存器
    output reg [1:0] forward_ex, // EX 阶段的前推信号
    output reg [1:0] forward_mem // MEM 阶段的前推信号
    //output reg [31:0] forward_data
);

//assign forward_ex = rst ? 0 :
//    (rs2==ex_rd) ? 2'b10 : 
//    (rt2==ex_rd) ? 2'b01 :
//    0;
//assign forward_mem = rst ? 0 :
//        (rs1==mem_rd) ? 2'b10 : 
//        (rt1==mem_rd) ? 2'b01 :
//        0;
        // 初始化前推信号
        // 检查 EX 阶段是否前推
        always@(posedge clk)begin
        forward_ex = 0;
        forward_mem = 0;
        if (rs2 == ex_rd) forward_ex[1] = 1;
        if (rt2 == ex_rd) forward_ex[0] = 1;
        if(forward_ex == 2'b11) forward_ex = 0;//空指令
        forward_mem = 0;
         //检查 MEM 阶段是否前推
        if (rs1 == mem_rd && forward_ex[1] == 0) forward_mem[1] = 1;
        if (rt1 == mem_rd && forward_ex[0] == 0) forward_mem[0] = 1;
        if (forward_mem == 2'b11) forward_mem = 0;//空指令
        end
        //如果寄存器即需要EX前推，又需要MEM前推，事实上，
        //只需要EX前推，因为EX阶段才是最新的值，而MEM在EX阶段已经做过前推了
        //因此，这里特判一下没有EX前推
endmodule
```

- 各信号的定义见2.0，此处对EX前推和MEM前推的旁路寄存器值进行更新。
- 有两处细节，一是，前推信号理论上不会出现rs和rt均更新，因为一条指令不可能同时对寄存器组中的rs和rt做修改。二是，优先判定EX前推，其次判定MEM前推，并且MEM前推的前提是没有此时没有出现EX前推。这样做是因为,EX前推执行的指令才是最新的，因此即使可能本身需要MEM前推，但EX阶段再次对寄存器更新，故这种情况下应该选择二次更新的EX前推值进行计算。

现罗列对于设计的指令集中，EX阶段可能要进行数据前推的指令如下

- R-Type:

| Instruction | Opcode/Function | Syntax       | Operation    |
| ----------- | --------------- | ------------ | ------------ |
| add         | 000000/100000   | f $d, $s, $t | $d = $s + $t |
| sub         | 000000/100010   | f $d, $s, $t | $d = $s - $t |

- I-Type:

| I-type | op     | rs   | rt   | immediate | **示例**       | **示例含义**       |
| ------ | ------ | ---- | ---- | --------- | -------------- | ------------------ |
| addi   | 001000 | rs   | rt   | immediate | addi $1,$2,100 | $1=$2+100          |
| lw     | 100011 | rs   | rt   | immediate | lw $1,10($2)   | $1=memory[$2  +10] |

因此要判定，后续使用的寄存器是否是已经运算结束的值要存的目的寄存器。

在Mem阶段需要前推的只有sw指令

| I-type | op     | rs   | rt   | immediate | **示例**     | **示例含义**       |
| ------ | ------ | ---- | ---- | --------- | ------------ | ------------------ |
| sw     | 101011 | rs   | rt   | immediate | sw $1,10($2) | memory[$2+10]  =$1 |



##### 2.7 simulation

sim代码如下

```verilog
module sim_5_stage_Flow;
    reg clk,rst;
    integer cnt;
    wire [31:0] ALU_out,Mem_0;
    wire [31:0] PC,npc1,npc2,npc3;
    wire [31:0] instruction,WriteData,alu_result,mem_data,ALUOut2;
    reg [31:0] r1,r2,r3,r4,r5;
    wire [4:0] WriteReg,ex_rd,mem_rd,rs1,rt1;
    wire [2:0] forward_ex,forward_mem;
    wire RegWrite,MemWrite,RegDst,branch,Jump;
    flow_main flow_main(clk,rst);
assign npc1 = flow_main.npc1;
assign npc2 = flow_main.npc2;
assign npc3 = flow_main.npc3;
assign ALUOut2 = flow_main.ALUOut2;
assign rs1 = flow_main.rs1;
assign rt1 = flow_main.rt1;
assign alu_result = flow_main.alu_result;
assign mem_data = flow_main.mem_data;
assign forward_ex = flow_main.forward_ex;
assign forward_mem = flow_main.forward_mem;
assign ex_rd = flow_main.ex_rd;
assign mem_rd = flow_main.mem_rd;
assign PC = flow_main.PC;
assign instruction = flow_main.instruction1;
assign ALU_out = flow_main.ALUOut;
assign Mem_0 = flow_main.Memstage.dmem.memory[0];
assign WriteReg = flow_main.WriteReg;
assign MemWrite = flow_main.MemWrite;
assign RegWrite = flow_main.RegWrite;
assign WriteData = flow_main.WriteData;
assign RegDst = flow_main.RegDst;
assign branch = flow_main.branch;
assign Jump = flow_main.Jump;
always @ * begin
    r1 <= flow_main.regfile.rf[1];
    r2 <= flow_main.regfile.rf[2];
    r3 <= flow_main.regfile.rf[3];
    r4 <= flow_main.regfile.rf[4];
    r5 <= flow_main.regfile.rf[5];
    end
    initial begin
        clk=0;rst=0;cnt=0;
        fork
            repeat(500)#5 clk=~clk;
            repeat(1)begin
               #2000 rst=1;
               #5 rst=0;
            end
            repeat(500)#5 cnt = cnt+1; //当前第cnt条程序进入FI阶段
        join
    end
endmodule
```

- 此处笔者写了大量的查询中间信号的值，因为多周期的debug比较繁琐，因此把这些信号全部罗列出来方便检查错误。
- 同样地，在sim中只需要修改时钟信号和rst信号，观察输出波形。

以下是本次测试使用的IM中的指令

<img src="C:\Users\ROG\AppData\Roaming\Typora\typora-user-images\image-20231224162418580.png" alt="image-20231224162418580" style="zoom:67%;" />

运行结果如下

![image-20231224162755688](C:\Users\ROG\AppData\Roaming\Typora\typora-user-images\image-20231224162755688.png)

- PC=4,读取第一条空指令。

- PC=8,读取第二条R-type指令

  ```verilog
  // R-type add 				(0022_1820H)
  //$3 = $1 + $2                $1=0, $2=0 ,因此$3=0
  000000_00001_00010_00011_00000_100000  
  ```

  寄存器值均为0，因此结果也为0，结果不变，但可以在时序中观察到，WriteReg信号在之后的第四个时钟信号发出了对3号寄存器写入的信号，如图

![image-20231224163022622](C:\Users\ROG\AppData\Roaming\Typora\typora-user-images\image-20231224163022622.png)

- PC=cH,第三条指令进入FI阶段，第二条指令进入ID阶段，第一条指令EX阶段

  ```verilog
  //I-type  addi  $2 = $1 +5        $2=5, $1=0 	(2022_0005H)
  001000_00001_00010_0000_0000_0000_0101
  ```

  立即数指令，对2号寄存器写入5，可以在波形中看到，75ns时，对寄存器组中的r2成功写入值5。

- PC=10H，读入该指令

  ```verilog
  //I-type $3 = $1 +2        $3=2, $1=0		(2023_0002H)
  001000_00001_00011_0000_0000_0000_0010
  ```

  3号寄存器值将在WB阶段之后更新为2，在时序中，85ns处，寄存器值被更新，而该指令是在35ns处被读入，中间间隔50ns，一个时钟信号为10ns，即在WB阶段后下一个时钟上升沿更新了寄存器值。

- PC=14H,读入空指令

  ```verilog
  // nop					(0000_0000H)
  000000_00000_00000_0000_0000_0000_0000	
  ```

- PC=18H,读入一个R-type指令

  ```verilog
  //R-tpye sub    $4 = $2 - $3, 要使用旁路寄存器中的$3,且处于MEM前推
  //					 (0043_2022H)
  000000_00010_00011_00100_00000_100010	
  //此时 $2 = 5, $3 = 2, $4 = 3
  ```

  注意，该指令会发生数据前推，在途中可以观察到，黄色光标处forward_mem信号置1:

  ![image-20231224164024873](C:\Users\ROG\AppData\Roaming\Typora\typora-user-images\image-20231224164024873.png)

  并且，mem_data值为2，即3号寄存器中的值之前被存入了旁路寄存器中，在该指令EX阶段使用了该值进行运算，最终该指令存入寄存器的值是最新的值`$4 = $2 - $3=3`。

- PC=1cH,读入如下Jump指令

  ```verilog
  //Jump goto 4'b0000 + 3<<2 = 'b100  PC->12=cH	(0800_0003H)
  000010_0000_0000_0000_0000_0000_0000_11	
  ```

  因为Jump信号是在MEM阶段计算target_address，因此PC将在第四个时钟信号更新为Jump的跳转地址，如图

  ![image-20231224165125196](C:\Users\ROG\AppData\Roaming\Typora\typora-user-images\image-20231224165125196.png)

  跳转到根据定义计算的PC=cH，符合预期。

后续执行了一系列空指令。

##### 2.8 RTL分析

![image-20231224165351972](C:\Users\ROG\AppData\Roaming\Typora\typora-user-images\image-20231224165351972.png)



### **五、调试和心得体会**

​	**调试**

​	本实验中，debug用了大量时间，特别是五级流水的调试，对于时序的调试非常的繁琐，因为每一条指令是相邻的次序执行，因此对每个时钟信号要做的事情划分要十分明确。并且，我个人认为5级流水是指5个机器周期，这里我们特例一个机器周期=1个时钟周期，这种写法让时序关系在1个时钟信号中很难处理。笔者认为，如果采用分频处理，对每一个机器周期再多划分几个时钟周期，可能调试会更容易些。

​	本次调试中用到了一些技巧，例如使用`display()`显示程序运行的结果，类似于`print()`语句，还有对时序的把握，有些地方做了一些延迟1个时钟信号的处理，这些都是为了让5条指令同时运行时时序的正确性。具体的调试的方式可以参考代码中的一些注释。

​	**心得体会**

​	本次实验代码量很大，对五级流水的调试时间远远大于书写代码，也有一定的心得，例如在sim代码中多写一些中间变量，方便观察和定位错误。在检查仿真的正确性时，先观察PC和读入的指令时序是否正确，再依次观察对应指令的各个阶段，按照5个阶段的次序依次排查各信号量是否正常变化。

​	通过这次实验，极大的锻炼了书写verilog代码和debug的能力，特别是对于时序的把握，中间变量以及一系列的细节，甚至阻塞和非阻塞的赋值方式以及前后的排序组合不同，会导致截然不同的结果。因此笔者认为该实验非常锻炼对流水CPU框架的把握以及每一处的细节实现。特别是数据前推的处理等。这次实验为将来的硬件编程打下了足够的基础，也对verilog的编程以及仿真有了更深刻的认识。



### 附录

##### 1. 32位加法器实现代码

```verilog
module add32(A,B,Ci,S,C,EN);  //32
input [31:0]A,B;
input Ci,EN;
output reg [31:0]S;
output reg C;
wire C1,C2,C3,C4,C5,C6,C7,Ct;
wire [31:0] St;
add4 add0_3(A[3:0],B[3:0],Ci,St[3:0],C1);
add4 add4_7(A[7:4],B[7:4],C1,St[7:4],C2);
add4 add8_11(A[11:8],B[11:8],C2,St[11:8],C3);
add4 add12_15(A[15:12],B[15:12],C3,St[15:12],C4);
add4 add16_19(A[19:16],B[19:16],C4,St[19:16],C5);
add4 add20_23(A[23:20],B[23:20],C5,St[23:20],C6);
add4 add24_27(A[27:24],B[27:24],C6,St[27:24],C7);
add4 add28_31(A[31:28],B[31:28],C7,St[31:28],Ct);
always@(*)begin //debug
    if(~EN)begin
     S=32'bz;
     C=32'bz;
     end
     else begin
     S=St;
     C=Ct;
     end
     //$display("S is %d, C is %d",S,C);
end
endmodule

module add4(A,B,Ci,S,C);  //4
input [3:0] A,B;
input Ci;
output reg [3:0]S;
output reg C;
wire C1,C2,C3,CC;
wire [3:0]SS;
add1 add4_1(A[0],B[0],Ci,SS[0],C1);
add1 add4_2(A[1],B[1],C1,SS[1],C2);
add1 add4_3(A[2],B[2],C2,SS[2],C3);
add1 add4_4(A[3],B[3],C3,SS[3],CC);
always@(*)begin
    S<=SS;
    C<=CC;
end
endmodule

module add1(				//1
input A,input B,input Ci,
output S,output C);
    wire S1,C1,C2;
    //assign {C,S}=A+B+Ci;
   half_add half_add1(A,B,S1,C1);
    half_add half_add2(S1,Ci,S,C2);
    assign C=C1|C2;
endmodule

module half_add(input A,input B,output S,output C);		//half
    assign S=A^B;
    assign C=A&B;
endmodule
```

##### 2. 控制器子模块

```verilog
module MainDec(
	input [5:0] Op,
	output MemToReg, MemWrite,
	output Branch, ALUSrc,
	output RegDst, RegWrite,
	output Jump,
	output [1:0] ALUOp);
	reg[8:0] Controls;
	assign{RegWrite,RegDst,ALUSrc,Branch,MemWrite,MemToReg,Jump,ALUOp}=Controls;
	always@(*)begin
		case(Op)
			6'b000000: Controls <=9'b110000010;// RTYPE
			6'b100011: Controls<=9'b101001000;//LW
			6'b101011: Controls <=9'b001010000;//SW
			6'b000100: Controls <=9'b000100001;//BEQ
			6'b001000: Controls<=9'b101000000;//ADDI
			6'b000010: Controls <=9'b000000100;//J
			default: Controls<=9'bxxxxxxxxx;//illegal Op
		endcase  
		//$display("Op is %d,Controls is %d",Op,Controls);
    end
endmodule


module ALUDec(
	input [5:0] Funct,
	input [1:0] ALUOp,
	output reg[4:0] ALUControl);
	always@(*)
		case(ALUOp)
			2'b00: ALUControl<=5'b00000;// add(for lw/sw/addi/J)
			2'b01: ALUControl <=5'b00001;//sub(for beq)
			default: case(Funct)     //R-type Instructions
				6'b100000: ALUControl<=5'b00000; //add
				6'b100010: ALUControl <=5'b00001;//sub
				//6'b100100: ALUControl <=5'b00010;//and
				//6'b100101: ALUControl<=5'b00011;//or
				//6'b101010: ALUControl<=5'b111;//slt
				default:  ALUControl<=5'bxxxxx;//???
				endcase
		endcase
endmodule
```

##### 3. 单周期测试指令

```verilog
// nop					(0000_0000H)
000000_00000_00000_0000_0000_0000_0000	

// R-type add 				(0022_1820H)
//$3 = $1 + $2                $1=0, $2=0 ,因此$3=0
000000_00001_00010_00011_00000_100000  

//I-type  addi  $2 = $1 +5        $2=5, $1=0 	(2022_0005H)
001000_00001_00010_0000_0000_0000_0101

//I-type $3 = $1 +2        $3=2, $1=0		(2023_0002H)
001000_00001_00011_0000_0000_0000_0010

//2号寄存器存入内存,memory[$1=0]中,即memory[0]=5
// 					(AC22_0000H)
101011_00001_00010_0000_0000_0000_0000	

//lw $1 = memory[$1+0=0]=5			(8C21_0000H)
100011_00001_00001_0000_0000_0000_0000	

//beq  $3!=$4					(1083_0010H)
000100_00100_00011_0000_0000_0001_0000	

//Jump goto 4'b0000 + 3<<2 = 'b100  PC->12=cH	(0800_0003H)
000010_0000_0000_0000_0000_0000_0000_11	
```

##### 4. 额外测试指令

```verilog
// nop					(0000_0000H)
000000_00000_00000_0000_0000_0000_0000	

// R-type add 				(0022_1820H)
//$3 = $1 + $2                $1=0, $2=0 ,因此$3=0
000000_00001_00010_00011_00000_100000  

//I-type  addi  $2 = $1 +5        $2=5, $1=0 	(2022_0005H)
001000_00001_00010_0000_0000_0000_0101

//I-type $3 = $1 +2        $3=2, $1=0		(2023_0002H)
001000_00001_00011_0000_0000_0000_0010

// nop					(0000_0000H)
000000_00000_00000_0000_0000_0000_0000	

//R-tpye sub    $4 = $2 - $3, 要使用旁路寄存器中的$3,且处于MEM前推
//					 (0043_2022H)
000000_00010_00011_00100_00000_100010	
//此时 $2 = 5, $3 = 2, $4 = 3

//做一个EX阶段的数据前推，将$4 = 3 存入 memory[$1=0]中,即memory[0]=3
//此时$4=3 是上一条指令刚计算得到的，因此在EX阶段会出现前推，在MEM中存入前推数据
// 					(AC24_0000H)
101011_00001_00100_0000_0000_0000_0000	

//lw $5 = memory[$1+0=0]=3			(8C25_0000H)
100011_00001_00101_0000_0000_0000_0000	

//beq  $6=$7 goto PC+4+16x4=PC+68=PC+44H	(10C7_0010H)
000100_00110_00111_0000_0000_0001_0000	

//Jump goto 4'b0000 + 3<<2 = 'b100  PC->12=cH	(0800_0003H)
000010_0000_0000_0000_0000_0000_0000_11	
```

##### 5.五级流水测试指令

```verilog
// nop					(0000_0000H)
000000_00000_00000_0000_0000_0000_0000	

// R-type add 				(0022_1820H)
//$3 = $1 + $2                $1=0, $2=0 ,因此$3=0
000000_00001_00010_00011_00000_100000  

//I-type  addi  $2 = $1 +5        $2=5, $1=0 	(2022_0005H)
001000_00001_00010_0000_0000_0000_0101

//I-type $3 = $1 +2        $3=2, $1=0		(2023_0002H)
001000_00001_00011_0000_0000_0000_0010

// nop					(0000_0000H)
000000_00000_00000_0000_0000_0000_0000	

//R-tpye sub    $4 = $2 - $3, 要使用旁路寄存器中的$3,且处于MEM前推
//					 (0043_2022H)
000000_00010_00011_00100_00000_100010	
//此时 $2 = 5, $3 = 2, $4 = 3

//Jump goto 4'b0000 + 3<<2 = 'b100  PC->12=cH	(0800_0003H)
000010_0000_0000_0000_0000_0000_0000_11	

// nop					(0000_0000H)
000000_00000_00000_0000_0000_0000_0000	
// nop					(0000_0000H)
000000_00000_00000_0000_0000_0000_0000	
// nop					(0000_0000H)
000000_00000_00000_0000_0000_0000_0000	
```

##### 6.MIPS指令集

| **MIPS 指令集(共31条）** |              |          |              |                        |                                                              |                              |                                                              |                              |                                                              |
| ------------------------ | ------------ | -------- | ------------ | ---------------------- | ------------------------------------------------------------ | ---------------------------- | ------------------------------------------------------------ | ---------------------------- | ------------------------------------------------------------ |
| **助记符**               | **指令格式** | **示例** | **示例含义** | **操作及其解释**       |                                                              |                              |                                                              |                              |                                                              |
| Bit #                    | 31..26       | 25..21   | 20..16       | 15..11                 | 10..6                                                        | 5..0                         |                                                              |                              |                                                              |
| R-type                   | op           | rs       | rt           | rd                     | shamt                                                        | func                         |                                                              |                              |                                                              |
| add                      | 000000       | rs       | rt           | rd                     | 00000                                                        | 100000                       | add $1,$2,$3                                                 | $1=$2+$3                     | rd <- rs + rt  ；其中rs＝$2，rt=$3, rd=$1                    |
| addu                     | 000000       | rs       | rt           | rd                     | 00000                                                        | 100001                       | addu $1,$2,$3                                                | $1=$2+$3                     | rd <- rs + rt  ；其中rs＝$2，rt=$3, rd=$1,无符号数           |
| sub                      | 000000       | rs       | rt           | rd                     | 00000                                                        | 100010                       | sub $1,$2,$3                                                 | $1=$2-$3                     | rd <- rs - rt  ；其中rs＝$2，rt=$3, rd=$1                    |
| subu                     | 000000       | rs       | rt           | rd                     | 00000                                                        | 100011                       | subu $1,$2,$3                                                | $1=$2-$3                     | rd <- rs - rt  ；其中rs＝$2，rt=$3, rd=$1,无符号数           |
| and                      | 000000       | rs       | rt           | rd                     | 00000                                                        | 100100                       | and $1,$2,$3                                                 | $1=$2 & $3                   | rd <- rs & rt  ；其中rs＝$2，rt=$3, rd=$1                    |
| or                       | 000000       | rs       | rt           | rd                     | 00000                                                        | 100101                       | or $1,$2,$3                                                  | $1=$2 \| $3                  | rd <- rs \| rt  ；其中rs＝$2，rt=$3, rd=$1                   |
| xor                      | 000000       | rs       | rt           | rd                     | 00000                                                        | 100110                       | xor $1,$2,$3                                                 | $1=$2 ^ $3                   | rd <- rs xor rt  ；其中rs＝$2，rt=$3, rd=$1(异或）           |
| nor                      | 000000       | rs       | rt           | rd                     | 00000                                                        | 100111                       | nor $1,$2,$3                                                 | $1=~($2 \| $3)               | rd <- not(rs \| rt)  ；其中rs＝$2，rt=$3, rd=$1(或非）       |
| slt                      | 000000       | rs       | rt           | rd                     | 00000                                                        | 101010                       | slt $1,$2,$3                                                 | if($2<$3)   $1=1 else   $1=0 | if (rs < rt) rd=1 else rd=0 ；其中rs＝$2，rt=$3, rd=$1       |
| sltu                     | 000000       | rs       | rt           | rd                     | 00000                                                        | 101011                       | sltu $1,$2,$3                                                | if($2<$3)   $1=1 else   $1=0 | if (rs < rt) rd=1 else rd=0 ；其中rs＝$2，rt=$3, rd=$1  (无符号数） |
| sll                      | 000000       | 00000    | rt           | rd                     | shamt                                                        | 000000                       | sll $1,$2,10                                                 | $1=$2<<10                    | rd <- rt << shamt ；shamt存放移位的位数，  也就是指令中的立即数，其中rt=$2, rd=$1 |
| srl                      | 000000       | 00000    | rt           | rd                     | shamt                                                        | 000010                       | srl $1,$2,10                                                 | $1=$2>>10                    | rd <- rt >> shamt ；(logical) ，其中rt=$2, rd=$1             |
| sra                      | 000000       | 00000    | rt           | rd                     | shamt                                                        | 000011                       | sra $1,$2,10                                                 | $1=$2>>10                    | rd <- rt >> shamt  ；(arithmetic) 注意符号位保留  其中rt=$2, rd=$1 |
| sllv                     | 000000       | rs       | rt           | rd                     | 00000                                                        | 000100                       | sllv $1,$2,$3                                                | $1=$2<<$3                    | rd <- rt << rs ；其中rs＝$3，rt=$2, rd=$1                    |
| srlv                     | 000000       | rs       | rt           | rd                     | 00000                                                        | 000110                       | srlv $1,$2,$3                                                | $1=$2>>$3                    | rd <- rt >> rs ；(logical)其中rs＝$3，rt=$2, rd=$1           |
| srav                     | 000000       | rs       | rt           | rd                     | 00000                                                        | 000111                       | srav $1,$2,$3                                                | $1=$2>>$3                    | rd <- rt >> rs ；(arithmetic) 注意符号位保留  其中rs＝$3，rt=$2, rd=$1 |
| jr                       | 000000       | rs       | 00000        | 00000                  | 00000                                                        | 001000                       | jr $31                                                       | goto $31                     | PC <- rs                                                     |
| I-type                   | op           | rs       | rt           | immediate              |                                                              |                              |                                                              |                              |                                                              |
| addi                     | 001000       | rs       | rt           | immediate              | addi $1,$2,100                                               | $1=$2+100                    | rt <- rs + (sign-extend)immediate ；其中rt=$1,rs=$2          |                              |                                                              |
| addiu                    | 001001       | rs       | rt           | immediate              | addiu $1,$2,100                                              | $1=$2+100                    | rt <- rs + (zero-extend)immediate ；其中rt=$1,rs=$2          |                              |                                                              |
| andi                     | 001100       | rs       | rt           | immediate              | andi $1,$2,10                                                | $1=$2 & 10                   | rt <- rs & (zero-extend)immediate ；其中rt=$1,rs=$2          |                              |                                                              |
| ori                      | 001101       | rs       | rt           | immediate              | andi $1,$2,10                                                | $1=$2 \| 10                  | rt <- rs \| (zero-extend)immediate ；其中rt=$1,rs=$2         |                              |                                                              |
| xori                     | 001110       | rs       | rt           | immediate              | andi $1,$2,10                                                | $1=$2 ^ 10                   | rt <- rs xor (zero-extend)immediate ；其中rt=$1,rs=$2        |                              |                                                              |
| lui                      | 001111       | 00000    | rt           | immediate              | lui $1,100                                                   | $1=100*65536                 | rt <- immediate*65536 ；将16位立即数放到目标寄存器高16      位，目标寄存器的低16位填0 |                              |                                                              |
| lw                       | 100011       | rs       | rt           | immediate              | lw $1,10($2)                                                 | $1=memory[$2  +10]           | rt <- memory[rs + (sign-extend)immediate] ；rt=$1,rs=$2      |                              |                                                              |
| sw                       | 101011       | rs       | rt           | immediate              | sw $1,10($2)                                                 | memory[$2+10]  =$1           | memory[rs + (sign-extend)immediate] <- rt ；rt=$1,rs=$2      |                              |                                                              |
| beq                      | 000100       | rs       | rt           | immediate              | beq $1,$2,10                                                 | if($1==$2)   goto PC+4+40    | if (rs == rt) PC <- PC+4 + (sign-extend)immediate<<2         |                              |                                                              |
| bne                      | 000101       | rs       | rt           | immediate              | bne $1,$2,10                                                 | if($1!=$2)  goto PC+4+40     | if (rs != rt) PC <- PC+4 + (sign-extend)immediate<<2         |                              |                                                              |
| slti                     | 001010       | rs       | rt           | immediate              | slti $1,$2,10                                                | if($2<10)   $1=1 else   $1=0 | if (rs <(sign-extend)immediate) rt=1 else rt=0 ；   其中rs＝$2，rt=$1 |                              |                                                              |
| sltiu                    | 001011       | rs       | rt           | immediate              | sltiu $1,$2,10                                               | if($2<10)   $1=1 else   $1=0 | if (rs <(zero-extend)immediate) rt=1 else rt=0 ；  其中rs＝$2，rt=$1 |                              |                                                              |
| J-type                   | op           | address  |              |                        |                                                              |                              |                                                              |                              |                                                              |
| j                        | 000010       | address  | j 10000      | goto 10000             | PC <- (PC+4)[31..28],address,0,0  ；address=10000/4          |                              |                                                              |                              |                                                              |
| jal                      | 000011       | address  | jal 10000    | $31<-PC+4;  goto 10000 | $31<-PC+4；PC <- (PC+4)[31..28],address,0,0   ；address=10000/4 |                              |                                                              |                              |                                                              |
