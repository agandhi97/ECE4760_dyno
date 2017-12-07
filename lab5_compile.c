/** ECE 4760 MOTOR DYNAMOMETER
 Aasta Gandhi, Kowin Shi, Erika Yu**/

// clock AND protoThreads configure!
// You MUST check this file!
#include "config_1_2_2.h"
// threading library
#include "pt_cornell_1_2_2.h"

////////////////////////////////////
// graphics libraries
#include "tft_master.h"
#include "tft_gfx.h"
#include <stdlib.h>
#include <math.h>
////////////////////////////////////

/////////UART THREADS //////////////

// The following threads are necessary for UART control
static struct pt pt_input, pt_output, pt_DMA_output ;

///////set up SPI//////
// for 60 MHz PB clock use divide-by-3
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000
volatile int spiClkDiv = 2 ; // 20 MHz DAC clock
volatile SpiChannel spiChn = SPI_CHANNEL2;

////////////////////////////////////

// pullup/down macros for keypad
// PORT B
#define EnablePullDownB(bits) CNPUBCLR=bits; CNPDBSET=bits;
#define DisablePullDownB(bits) CNPDBCLR=bits;
#define EnablePullUpB(bits) CNPDBCLR=bits; CNPUBSET=bits;
#define DisablePullUpB(bits) CNPUBCLR=bits;
//PORT A
#define EnablePullDownA(bits) CNPUACLR=bits; CNPDASET=bits;
#define DisablePullDownA(bits) CNPDACLR=bits;
#define EnablePullUpA(bits) CNPDACLR=bits; CNPUASET=bits;
#define DisablePullUpA(bits) CNPUACLR=bits;

// string buffer
char buffer[60];
void printLine(int line_number, char* print_buffer, short text_color, short back_color){
    int v_pos;
    v_pos = line_number * 10 ;
    // erase the pixels
    tft_fillRoundRect(0, v_pos, 320, 8, 1, back_color);// x,y,w,h,radius,color
    tft_setTextColor(text_color); 
    tft_setCursor(0, v_pos);
    tft_setTextSize(1);
    tft_writeString(print_buffer);
}

// string buffer
char buffer[60];

////////// current sensor init //////////

float adc_raw = 0; //current sensor reading
float voltage = 0; //initialize final voltage 
float current = 0; //initialize final current

/////////////////////////////

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_currSensor, pt_cmd;



// define setup parameters for OpenADC10
// Turn module on | ouput in integer | trigger mode auto | enable autosample
// ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
// ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
// ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
// ADC_FORMAT_INTG16 -- Sets minimum integer size to 16
#define PARAM1 ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON 
/* The clock is always running. Code manually starts acquire process in code, otherwise conversion will start as soon as previous one finishes*/
// define setup parameters for OpenADC10
// ADC ref external | disable offset test | disable scan mode | do 1 sample | use single buf | alternate mode off
#define PARAM2 ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_3 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
/* ADC_VREF_AVDD_AVSS -- sets voltage references to 0 and Vdd; ADC_OFFSET_CAL_DISABLE turns off offset calibration, ADC_SAMPLES_PER_INT_1 -- don?t care since there is no interrupt; ADC_SCAN_OFF ADC_ALT_BUGG, ADC_ALT_INPUT -- disable unneeded special features */
// Define setup parameters for OpenADC10
// use peripherial bus clock | set sample time | set ADC clock divider
// ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
// ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
#define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_15 | ADC_CONV_CLK_Tcy //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2 (timing setup)
// define setup parameters for OpenADC10
// set AN11 and as analog inputs
#define PARAM4 ENABLE_AN11_ANA | ENABLE_AN1_ANA | ENABLE_AN0_ANA// pin 24
// define setup parameters for OpenADC10
// do not assign channels to scan
#define PARAM5 SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15

///// UI counters /////

#define init_angle 0
static int imax = 300000;
int period = 0xFFFFF; //(for timer setup, so it does not overflow)
volatile int button = 0; //0: beam angle, 1: prop. gain, 2: diff. gain, 3: Integral gain
volatile int raw_p; //adc channel 2, adc5

static int des_angle = 1000;
static int des_angle2 = 1000;
//The actual period of the wave
int generate_period = 5050;
int pwm_on_time = 4200;
int pwm_on_time_servo2 = 2500;
int pwm_on_time_servo1 = 2500;


volatile int timeCapture; //input capture
volatile float newtime; //holds rpm value
volatile float rpm_filt = 0; //filtered rpm value
volatile int ccc = 0;
volatile int tm = 0; //counter for time in seconds
int lim = 500; //torque limit
volatile float strain_filt = 0; //calculates filtered strain
int swtt = 0; //switch
volatile float curr_filt = 0; //filter for current
volatile int volt_raw; //read adc voltage from current sensor
volatile float voltcalc; //used to convert adc voltage to actual voltage value
volatile float tork; //torque
float initstrain; //initial strain
volatile float radsec; //radsec converseion
volatile float power; //power calculation
volatile float eff; //efficiency
volatile float eff_filt = 0; //filter for efficiency

/////// RPM Data Collection Variables ///////
#define NOLOAD 'A' 
#define TEN 'B'
#define TWENTY 'C'
#define THIRTY 'D'
#define FORTY 'E'
#define FIFTY 'F'
#define SIXTY 'G'
#define SEVENTY 'H'
#define EIGHTY 'I'
#define NINETY 'J'
#define WAIT 'K' 
#define PREP 'L'
volatile char rpm_state = WAIT;   //state variable


//*** Interrupt Service Routine to read input capture***//
void __ISR(_INPUT_CAPTURE_4_VECTOR, ipl3)C4Handler(void) 
{
    timeCapture = mIC4ReadCapture();    //read value from input capture
    WriteTimer3(0);
    mIC4ClearIntFlag();                 //clear interrupt flag
}


// == Timer 2 ISR =====================================================
// just toggles a pin for timeing strobe
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    // clear the timer interrupt flag
    mT2ClearIntFlag();
}

// == Timer 3 ISR =====================================================
void __ISR(_TIMER_4_VECTOR, ipl4) Timer4Handler(void)
{
    //if timer is less than 15 sec, wait for raw_p to reach adc limit
    if(tm<15){
    raw_p = ReadADC10(2);
    if(raw_p < lim){

    mPORTBClearBits(BIT_4); // start transaction
  
    WriteSPI2(0x600100);
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
    // CS high
    mPORTBSetBits(BIT_4); // end transaction
    // clear the timer interrupt flag
    ccc++;
    swtt = 1;
    }
    else if(raw_p > lim){ //send transcation from SPI
    // === Channel A =============
    // CS low to start transaction
    mPORTBClearBits(BIT_4); // start transaction
    WriteSPI2(0xE00100);
    //1011 0000 0000 0000 0101 0000
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
    // CS high
    mPORTBSetBits(BIT_4); // end transaction
    // clear the timer interrupt flag
    ccc++;
    swtt = 0;
    }
    }
    else if (tm < 120){
        initstrain = strain_filt+3; //initialize first strain value with small offset
    }
    mT4ClearIntFlag();//maybe move this to bottom if it doesn't work?
}

//set up variables for control loop
volatile float run_sum = 0;  //running sum to average at the end of each load reading
volatile float rpm_sum = 0;  //rpm at no load that sets baseline for all other rpm
volatile float num_points = 0; //number of values read to average
volatile float rpm_des = 0; //desired rpm value
volatile int time = 0; //keep track of time
volatile char dest = NOLOAD; //next state to go transition to
char cont = 0;  //check for timing
static PT_THREAD (protothread_cmd(struct pt *pt))
{
    PT_BEGIN(pt);

    while(1) {
        if(PT_GET_TIME() > 500) {
        
         //state machine to transition from no load to 55% load 
        switch(rpm_state) {
            //transition state that prepares to calculate running rpm sum and sets the motors
            case PREP: 
                if(dest == NOLOAD) {
                    rpm_state = dest; //determine next state
                    run_sum = 0; //reset sum
                    rpm_sum = newtime; //current rpm value to reset 
                    num_points = 1;  //reset # of points
                    time = PT_GET_TIME(); //get time
                    
                }
                else if(newtime > rpm_des ) { //wait to transition
                    rpm_state = PREP; 
                    des_angle += 10; 
                    des_angle2 += 10; 
                    SetDCOC1PWM(des_angle); //2000 to 5000, 2000 is fully opened, starts engaging ~ 3500, engages hard ~ 4500, 4700 is limit for this motor
                    SetDCOC4PWM(des_angle2);
                    PT_YIELD_TIME_msec(5); 
                }
                else { //reset everything 
                    run_sum = 0; 
                    num_points = 0; 
                    rpm_state = dest; 
                    time = PT_GET_TIME(); 
                
                }
                
                break; 
           
                //no load -- first state, calculate rpm average and set baseline
            case NOLOAD:
                if((PT_GET_TIME() - time) >= 4000){
                    rpm_sum = rpm_sum/num_points; 
                    run_sum = run_sum/num_points; 
                    //send to UART
                    sprintf(PT_send_buffer,"%.2f %.2f %.2f\n\r", rpm_sum, run_sum, eff);
                    PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
                    rpm_state = PREP; //go to PREP state
                    dest = NINETY;   //set next state
                    rpm_des = rpm_sum*0.95; //set next desired load
                    
                }
                else{ //wait for motors to set 
                    rpm_state = NOLOAD; 
                    rpm_sum += newtime; 
                    run_sum += tork; 
                    num_points++; 
                }
                        
                break; 
                //95% Load 
            case TEN:
                if((PT_GET_TIME() - time) >= 4000){
                    run_sum = run_sum/num_points; 

                    sprintf(PT_send_buffer,"%.2f %.2f %.2f\n\r", rpm_des, run_sum, eff);
                    PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
                    
                    rpm_state = PREP;
                    dest = WAIT; 
                    
                }
                else{
                    
                    rpm_state = TEN;
                    run_sum += tork; 
                    num_points++; 
                }
                        
                break; 
                //90% Load
            case TWENTY:
                if((PT_GET_TIME() - time) >= 4000){
                    run_sum = run_sum/num_points; 
                    
                    sprintf(PT_send_buffer,"%.2f %.2f %.2f\n\r", rpm_des, run_sum, eff);
                    PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
                    rpm_state = PREP;
                    dest = TEN; 
                    rpm_des = rpm_sum*0.55;
                   
                }
                else{
                    
                    rpm_state = TWENTY;
                    run_sum += tork; 
                    num_points++; 
                }
                        
                break; 
            //85% load    
            case THIRTY:
                if((PT_GET_TIME() - time) >= 4000){
                    run_sum = run_sum/num_points; 
                    
                    sprintf(PT_send_buffer,"%.2f %.2f %.2f\n\r", rpm_des, run_sum, eff);
                    PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
                    rpm_state = PREP; 
                    dest = TWENTY; 
                    rpm_des = rpm_sum*0.60;
                }
                else{
                    
                    rpm_state = THIRTY;
                    run_sum += tork; 
                    num_points++; 
                  
                }
                        
                break; 
            //80% load    
            case FORTY:
                if((PT_GET_TIME() - time) >= 4000){
                    run_sum = run_sum/num_points; 
                  
                    sprintf(PT_send_buffer,"%.2f %.2f %.2f\n\r", rpm_des, run_sum, eff);
                    PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
                    rpm_state = PREP;
                    dest = THIRTY; 
                    rpm_des = rpm_sum*0.65;
                }
                else{
                    
                    rpm_state = FORTY;
                    run_sum += tork; 
                    num_points++; 
                }
                        
                break; 
                //75% load
            case FIFTY:
                if((PT_GET_TIME() - time) >= 4000){
                    run_sum = run_sum/num_points; 
                  
                    sprintf(PT_send_buffer,"%.2f %.2f %.2f\n\r", rpm_des, run_sum, eff);
                    PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
                    rpm_state = PREP; 
                    dest = FORTY; 
                    rpm_des = rpm_sum*0.70;
                }
                else{
                    
                    rpm_state = FIFTY;
                    run_sum += tork; 
                    num_points++; 
                }
                        
                break;  
                //70% load
            case SIXTY:
                if((PT_GET_TIME() - time) >= 4000){
                    run_sum = run_sum/num_points; 
                   
                   sprintf(PT_send_buffer,"%.2f %.2f %.2f\n\r", rpm_des, run_sum, eff);
                    PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
                    rpm_state = PREP; 
                    dest = FIFTY; 
                    rpm_des = rpm_sum*0.75;
                }
                else{
                   
                    rpm_state = SIXTY;
                    run_sum += tork; 
                    num_points++; 
                }
                        
                break; 
                //65% load
            case SEVENTY:
                if((PT_GET_TIME() - time) >= 4000){
                    run_sum = run_sum/num_points; 
                 
                   sprintf(PT_send_buffer,"%.2f %.2f %.2f\n\r", rpm_des, run_sum, eff);
                    PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
                    rpm_state = PREP; 
                    dest = SIXTY; 
                    rpm_des = rpm_sum*0.80;
                }
                else{
                    
                    rpm_state = SEVENTY;
                    run_sum += tork; 
                    num_points++; 
                }
                        
                break; 
                //60% load
             case EIGHTY:
                if((PT_GET_TIME() - time) >= 4000){
                    run_sum = run_sum/num_points; 
                  
                    sprintf(PT_send_buffer,"%.2f %.2f %.2f\n\r", rpm_des, run_sum, eff);
                  
                    PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
                    rpm_state = PREP; 
                    dest = SEVENTY; 
                    rpm_des = rpm_sum*0.85;
                }
                else{
                    
                    rpm_state = EIGHTY;
                    run_sum += tork; 
                    num_points++; 
                }
                        
                break; 
                //55% Load
             case NINETY:
                if((PT_GET_TIME() - time) >= 4000){
                    run_sum = run_sum/num_points; 
                  
                   sprintf(PT_send_buffer,"%.2f %.2f %.2f\n\r", rpm_des, run_sum, eff);
                    PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
                    rpm_state = PREP; 
                    dest = EIGHTY; 
                    rpm_des = rpm_sum*0.90;
                }
                else{
                    
                    rpm_state = NINETY;
                    run_sum += tork; 
                    num_points++; 
                }
                        
                break;                
             case WAIT:
                 
                 if(tm > 100 && tm < 120) { //give start signal to increase up voltage
                    sprintf(buffer,"starting \n\r" );
                    printLine(18, buffer, ILI9340_YELLOW, ILI9340_BLACK);
                 }
                 if( (tm > 120) && (cont == 0) ) { //wait until timer reaches 120 seconds to start control loop
                     
                    rpm_state = PREP;
                    rpm_des = rpm_sum; 
                    dest = NOLOAD; 
                    SetDCOC1PWM(2500); //2500 to 5000, 2500 is fully opened, starts engaging ~ 3500, engages hard ~ 4500, 4700 is limit for this motor
                    SetDCOC4PWM(2500);
                    cont = 1; 
                    
                    time = PT_GET_TIME();
                 } 
                 else if( (PT_GET_TIME() - time >= 5000) && (cont == 1) ) { //reset motors to run new sequence
                     
                    rpm_state = PREP;
                    rpm_des = rpm_sum; 
                    dest = NOLOAD; 
                    SetDCOC1PWM(2500); //2500 to 5000, 2500 is fully opened, starts engaging ~ 3500, engages hard ~ 4500, 4700 is limit for this motor
                    SetDCOC4PWM(2500);
                    cont = 1; 
                    time = PT_GET_TIME();
                
                    
                 }
                 else { //wait 
                     rpm_state = WAIT; 
                 }
                 
                        
                break;        
        
        
        }
        
        
     PT_YIELD_TIME_msec(1);     
    } // END WHILE(1)
        
    }     
    PT_END(pt);
} // timer thread

////////////current sensor reading and rpm, torque, efficiency calculations
static PT_THREAD (protothread_currSensor(struct pt *pt))
{
    //print sequence values and error
    PT_BEGIN(pt);
      while(1) {
        tm = PT_GET_TIME()/1000; //get time in seconds
        //read all values from adc
        volt_raw = ReadADC10(0);
        adc_raw = ReadADC10(1);
        raw_p = ReadADC10(2);
        
        //check for divide by zero for time capture
        if(timeCapture == 0){
            timeCapture = 1;
        }
        newtime = 9375000/(timeCapture); //calculate rpm 
        rpm_filt = rpm_filt + ((newtime - rpm_filt)/16); //filter rpm
        strain_filt = strain_filt + ((((float)raw_p) - strain_filt)/16); //filter strain
        tork = (strain_filt-initstrain)/156; //395 180 //calculate torque
        voltcalc = ((float)volt_raw)*0.012890625; //calculate voltage
        radsec = rpm_filt*0.10471976; //convert rpm to radsec
        power = radsec*tork; //calculate power
        eff = 100*power/(voltcalc*current); //calculate efficiency
        if(eff<0){
            eff = 0;
        }
        
        //print all values to TFT

        PT_YIELD_TIME_msec(1);
        sprintf(buffer,"adc is %d, going up? %d, rpm is %.2f", raw_p, swtt, rpm_filt);
        printLine(4, buffer, ILI9340_YELLOW, ILI9340_BLACK);
        
        sprintf(buffer,"strain is %.2f, init is %.2f, torque is %f Nm", strain_filt, initstrain, tork);
        printLine(6, buffer, ILI9340_YELLOW, ILI9340_BLACK);
        
        sprintf(buffer,"radsec is %.2f, power is %.2f W", radsec, power);
        printLine(8, buffer, ILI9340_YELLOW, ILI9340_BLACK);
        
        sprintf(buffer,"efficiency %.2f, filt %.2f", eff, eff_filt);
        printLine(10, buffer, ILI9340_YELLOW, ILI9340_BLACK);
        
        voltage = ((adc_raw - 128)/1024)*3.3; //millivolts //calculate voltage 
        current = voltage/(0.040); //calculate current  
        curr_filt = curr_filt + ((current - curr_filt)/16); //filter current 
        //current = voltage*4.0; 
        
            //print values
            sprintf(buffer,"Voltage: %f V", voltcalc);
            printLine(16, buffer, ILI9340_YELLOW, ILI9340_BLACK);
            sprintf(buffer, "voltage: %f", voltage);
            printLine(20, buffer, ILI9340_YELLOW, ILI9340_BLACK); 
            sprintf(buffer, "current: %f", curr_filt);
            printLine(22, buffer, ILI9340_YELLOW, ILI9340_BLACK); 
       
      } // END WHILE(1)
  PT_END(pt);
} 
     
// === Main ?======================================================
void main(void) {
//SYSTEMConfigPerformance(PBCLK);

 ANSELA = 0; ANSELB = 0; 

 // === config threads ==========
 // turns OFF UART support and debugger pin, unless defines are set
 PT_setup();

 // === setup system wide interrupts ?========
 INTEnableSystemMultiVectoredInt();
 
 // SCK2 is pin 26
    // SDO2 (MOSI) is in PPS output group 2, could be connected to RB5 which is pin 14
    PPSOutput(2, RPB5, SDO2);
    // control CS for DAC
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);
    // divide Fpb by 2, configure the I/O ports. Not using SS in this example
    // 16 bit transfer CKP=1 CKE=1
    // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
    // For any given peripherial, you will need to match these
    // clk divider set to 2 for 20 MHz
    SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE32 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , spiClkDiv);
    // end DAC setup
 
  // set up timer3 to run PID in ISR
  OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_64, period);
  //set up capture for photointerrupter 
  OpenCapture4(  IC_EVERY_RISE_EDGE | IC_INT_2CAPTURE | IC_TIMER3_SRC | IC_ON );
  // turn on the interrupt so that every capture can be recorded
  ConfigIntCapture4(IC_INT_ON | IC_INT_PRIOR_3 | IC_INT_SUB_PRIOR_3 );
  INTClearFlag(INT_IC4);
  // connect PIN 9 to IC1 capture unit
  PPSInput(1, IC4, RPB7);
    
// configure and enable the ADC
CloseADC10(); // ensure the ADC is off before setting the configuration
// use ground as neg ref ffor A | use AN11 for input A 
// configure to sample AN11
// ADC_CH0_NEG_SAMPLEA_NVREF, ADC_CH0_POS_SAMPLEA_AN11, select mux channel 0 , set negative input to look at NVREF, and positive input to sample AN11)
SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF); // configure to sample AN11,AN1,AN0
OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above
	EnableADC10(); // Enable the ADC
  
// set up compare3 for PWM mode
  OpenOC3(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE , pwm_on_time, pwm_on_time); //
  // OC3 is PPS group 4, map =to RPB9 (pin 18)
  PPSOutput(4, RPB9, OC3);
  
  // set up compare1 for PWM mode
  OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE , pwm_on_time_servo1, pwm_on_time_servo1); //
  // OC1 is PPS group 1, map =to RPB3
  PPSOutput(1, RPB3, OC1);
  
    // set up compare4 for PWM mode
  OpenOC4(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE , pwm_on_time_servo2, pwm_on_time_servo2); //
  // OC4 is PPS group 3, map =to RPA2
  PPSOutput(3, RPA2, OC4);
  
  // === Config timer and output compares to make pulses ========
  // set up timer2 to generate the wave period
  OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_16, generate_period);
  ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
  mT2ClearIntFlag(); // and clear the interrupt flag
 
  // set up timer4 to run strain gage DAQ in ISR
  OpenTimer4(T4_ON | T4_SOURCE_INT | T4_PS_1_256, 1000);
  ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_2);
  mT4ClearIntFlag(); // and clear the interrupt flag
  
 // init the threads
 PT_INIT(&pt_currSensor);
 PT_INIT(&pt_cmd);  

//initialize TFT display
tft_init_hw();
tft_begin();
tft_fillScreen(ILI9340_BLACK);
tft_setRotation(1); // 320x240 landscape display

 
 // round-robin scheduler for threads
 while (1){ 
   PT_SCHEDULE(protothread_currSensor(&pt_currSensor));
    PT_SCHEDULE(protothread_cmd(&pt_cmd));
    }
} // main

// === end ?======================================================
