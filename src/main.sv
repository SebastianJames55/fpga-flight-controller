module main(
	input logic CLK, // 50 MHz
	input logic BTN,
	//input logic INTERRUPT_PIN,
	inout logic SCL_PIN,
	inout logic SDA_PIN,
	input logic PPM_PIN,
	output logic[3:0] ESC_PINS
	
	//output logic signed[15:0] gyro_xout,
	//output logic signed[15:0] gyro_yout,
	//output logic signed[15:0] gyro_zout
	
);
	localparam signed MOTOR_MIN = 150;
	localparam signed MOTOR_MAX = 600;
	
	wire rst = ~BTN;
	
	logic divclk;
	logic[5:0] divclk_counter;
	
   logic scl_i;
	logic sda_i;
	logic scl_o;
	logic sda_o;
	
	assign scl_i = SCL_PIN;
	assign SCL_PIN = scl_o ? 1'bz : 1'b0; 
	assign sda_i = SDA_PIN;
	assign SDA_PIN = sda_o ? 1'bz : 1'b0;
	
	
	
	logic signed[15:0] gyro_xout;
	logic signed[15:0] gyro_yout;
	logic signed[15:0] gyro_zout;
	logic booting;
	logic data_ready;
	mpu6050_driver(.clk(CLK), .rst,/* .mpu6050_interrupt(INTERRUPT_PIN),*/ .scl_i, .sda_i, .scl_o, .sda_o, .gyro_xout, .gyro_yout, .gyro_zout, .data_ready, .booting);
	
	logic[11:0] ch[0:5];
	ppm_decoder decoder(divclk, rst, PPM_PIN, ch);
	
//	logic[11:0] lx;
//	logic[11:0] ly;
//	logic[11:0] rx;
//	logic[11:0] ry;
//	logic[11:0] arm;
//	assign lx = ch[0];//yaw
//	assign ly = ch[1];//throttle
//	assign rx = ch[2];//roll
//	assign ry = ch[3];//pitch
//	assign arm = ch[4];
	
//	wire[11:0] lx = ch[0];//yaw	
   wire[11:0] ly = ch[1];//throttle
   wire[11:0] rx = ch[2];//roll
   wire[11:0] ry = ch[3];//pitch
	wire[11:0] arm = ch[4];
	
	logic armed = 1'b0;

	wire  signed[12:0] throttle = ((ly * (MOTOR_MAX - MOTOR_MIN))>>>10) + MOTOR_MIN;
	logic signed[12:0] rollNet;
	logic signed[12:0] pitchNet;// = 13'b0;
	logic signed[12:0] yawNet = 13'b0;
	pid #(.P(31'd4 <<< 18), .D(31'd160 <<< 18), .I(13'b1100010010011), .MAX(10'd300)) roll(.clk(CLK),.rst,.data_ready,.gyro(gyro_xout),.stick(rx), .power(rollNet));
	pid #(.P(31'd4 <<< 18), .D(31'd160 <<< 18), .I(13'b1100010010011), .MAX(10'd300)) pitch(.clk(CLK),.rst,.data_ready,.gyro(gyro_yout),.stick(ry), .power(pitchNet));
	//pid #(.P(31'd4 <<< 18), .D(31'd160 <<< 18), .I(13'b1100010010011), .MAX(10'd300)) yaw(.clk(CLK),.rst,.data_ready,.gyro(gyro_zout),.stick(lx), .power(yawNet));
	
	//assign yawNet = 0;
	
	//logic signed[15:0] x_filtered;
	//logic signed[15:0] y_filtered;
	//logic signed[15:0] z_filtered;
	//lpf x_lpf(.clk(CLK), .rst, .data_ready, .value(gyro_xout), .filtered(x_filtered));
	//lpf y_lpf(.clk(CLK), .rst, .data_ready, .value(gyro_yout), .filtered(y_filtered));
	//lpf z_lpf(.clk(CLK), .rst, .data_ready, .value(gyro_zout), .filtered(z_filtered));
	
	
	wire signed[15:0] fl = throttle +rollNet -pitchNet -yawNet;
   wire signed[15:0] fr = throttle -rollNet -pitchNet +yawNet;
   wire signed[15:0] bl = throttle +rollNet +pitchNet +yawNet;
   wire signed[15:0] br = throttle -rollNet +pitchNet -yawNet;
	
	logic[11:0] motorOutput [0:3];
	
//	logic signed[15:0] mpu_out[0:2];
//	assign mpu_out[0] = gyro_xout;
	
   assign motorOutput[0] = !armed ? 1'b0 : (br < MOTOR_MIN ? MOTOR_MIN : (br > MOTOR_MAX ? MOTOR_MAX : br));
	assign motorOutput[1] = !armed ? 1'b0 : (fr < MOTOR_MIN ? MOTOR_MIN : (fr > MOTOR_MAX ? MOTOR_MAX : fr));
	assign motorOutput[2] = !armed ? 1'b0 : (fl < MOTOR_MIN ? MOTOR_MIN : (fl > MOTOR_MAX ? MOTOR_MAX : fl));
   assign motorOutput[3] = !armed ? 1'b0 : (bl < MOTOR_MIN ? MOTOR_MIN : (bl > MOTOR_MAX ? MOTOR_MAX : bl));

	logic[11:0] prev_arm;
	
	pwm_encoder encoder(divclk, rst, motorOutput, ESC_PINS);
	
	always @(posedge CLK) begin
		if (rst) begin
			divclk_counter <= 0;
			armed <= 0;
			prev_arm <= 0;
		end else begin
			if (divclk_counter < 25) begin
				divclk = 0;
				divclk_counter <= divclk_counter + 1;
			end else if (divclk_counter < 49) begin
				divclk = 1;
				divclk_counter <= divclk_counter + 1;
			end else begin
				divclk_counter <= 0;
			end
			
			prev_arm <= arm;
			if (arm > 500 && prev_arm < 500) begin
				if (ly < 50) armed <= 1;
			end
			if (arm < 500) armed <= 0;
		end
	end
endmodule

