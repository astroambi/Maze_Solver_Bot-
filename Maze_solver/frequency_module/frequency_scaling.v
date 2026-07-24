
module pwm_generator(
    input clk_3125KHz,
    input [3:0] duty_cycle,
    output reg clk_195KHz, pwm_signal
);

initial begin   
    clk_195KHz = 0; pwm_signal = 1;
	 end
//end
////////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE //////////////////
//
///*
//Add your logic here
//*/

reg[2:0] counter =0;


reg[4:0] pwm_counter =0;

always @(posedge clk_3125KHz) begin 

if(counter==0) begin 
clk_195KHz = ~clk_195KHz;
 counter = counter +1;


end

else begin

counter = counter +1;

end

pwm_signal <= (pwm_counter<duty_cycle)?1:0;

pwm_counter <=(pwm_counter==15)?0:(pwm_counter+1);

 
end
 

endmodule
