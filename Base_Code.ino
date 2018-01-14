///////////////////////////////////////////////////////////
//  Base Code v4
//  September, 2017
//  By Chris McClellan
//
//  Notes:  Added code for new motor controller.
//
///////////////////////////////////////////////////////////

#include <MsTimer2.h>
#include <math.h>

// Setup Motor Controller 1 Pins
static byte Reset = 4;         // Motor Controller Reset assigned to Digital Pin 4
static byte PWM_Out = 5;       // Motor Controller PWM assigned to Digital Pin 5 
static byte DIR = 6;           // Motor Controller DIR assigned to Digital Pin 6  

// Setup Encoder Counter 1 Pins
static byte SEL1 = 38;         // Select 1 output signal assigned to Pin 38
static byte OE = 39;           // Output Enable signal assinged to Pin 39
static byte RST = 40;          // Reset output signal assigned to Pin 40
static byte SEL2 = 41;         // Select 2 output signal assinged to Pin 41

// Declare Contstants
static float pi = 3.141593;

// Setup ISR Global Variables
volatile byte Go = 0;                         // Interrupt flag for START
volatile byte Timer_Go = 0;                   // Interrupt flag for timer
volatile byte Stop = 0;                       // Interrupt flag for E-STOP  
volatile byte Stop_bit = 0;                   // E-Stop input flag
volatile signed long int Reference_Input_Encoder = 0;   // Reference Input Signal in encoder counts (R(s)*I(s))
volatile signed long int Encoder_Count = 0;   // Current Encoder position in encoder counts
volatile float Controller_Output = 0;         // Capture/Control Loop Controller_Output in encoder counts
volatile float CO[3] = {0,0,0};               // Controller Output array in volts 
volatile float E[3] = {0,0,0};                // Error array in encoder counts
volatile float C1 = 0;                        // Controller Output coefficient @ t-1 
volatile float C2 = 0;                        // Controller Output coefficient @ t-2 
volatile float E0 = 0;                        // Error coefficient @ t  
volatile float E1 = 0;                        // Error coefficient @ t-1 
volatile float E2 = 0;                        // Error coefficient @ t-2 
volatile float Ts = 0;                        // Capture/Control Loop Period in seconds
volatile float Freq = 0;                      // Capture/Control Loop Frequency in hertz
volatile float t = 0;                         // Time counter in seconds
volatile long unsigned int Cnt = 0;           // Counter variable for Capture/Control Loop 
volatile float Ramp_Slope = 0;                // Slope of Open Loop Ramp function
volatile float Chirp_Rate = 0;                // Rate at which the chirp will change frequency 
volatile float temp = 0;                      // temp variable used as intermediary for assigning Ramp Slope to Reference Input
volatile float VtoPWM = 0;                    // Conversion factor from Volts to PWM duty cycle for analogWrite function  


//////////////////////////////////////////////////////////////////////////////////////////
//                   User Adjustable Variables
//////////////////////////////////////////////////////////////////////////////////////////
//
// Declare Mode of Operation where:
//
//      0 = Open Loop Step    -- applies a constant voltage to motor  
//                            Inputs  ---------------------------------------------------
//                            Step_Input -- sets the duty cycle of PWM output; an integer  
//                                          value between 0 and 1023
//                            I_Gain -- gain of the Input Filter function I(s) in encoder counts/radians
//                            Time -- sets the duration of the test in seconds  
//                            Period -- sets the Capture Loop period in milli seconds; an
//                                      integer value between 2 and 1000
//                            Outputs ---------------------------------------------------
//                            Freq -- frequency at which the Capture Loop is run in Hertz
//                            Time -- duration of test in seconds
//                            Reference_Input -- duty cycle of PWM output; an integer value
//                                               between 0 and 1023
//                            Count -- encoder count; a signed integer value 
//
//      1 = Open Loop Ramp    -- applies linearly varying voltage to motor                 
//                            Inputs  ---------------------------------------------------
//                            Ramp_Start -- sets the initial PWM output when t = 0; an
//                                          integer value between 0 and 1023; 
//                            Ramp_Final -- sets the final PWM output when t = Time; an
//                                          integer value between 0 and 1023; 
//                            I_Gain -- gain of the Input Filter function I(s) in encoder counts/radians
//                            Time -- sets the duration of the test in seconds  
//                            Period -- sets the Capture Loop period in milli seconds; an
//                                      integer value between 2 and 1000
//                            Outputs ---------------------------------------------------
//                            Freq -- frequency at which the Capture Loop is run in Hertz
//                            Time -- duration of test in seconds
//                            Reference_Input -- duty cycle of PWM output; an integer value
//                                               between 0 and 1023
//                            Count -- encoder count; a signed integer value
//                               
//      2 = Open Loop Chirp   -- applies a sinusoidal voltage in which the frequency
//                               linearly increases with time
//                            Inputs  ---------------------------------------------------
//                            Freq_Final -- sets the final frequency at t = Time; 
//                                          the start frequency is assumed to be 0 rad/s
//                                          at t = 0
//                            PWM_Amp -- sets the PWM output amplitude of the sinusoidal 
//                                       chirp signal; an integer value between 0 and 1023
//                            I_Gain -- gain of the Input Filter function I(s) in encoder counts/radians
//                            Time -- sets the duration of the test in seconds  
//                            Period -- sets the Capture Loop period in milli seconds; an
//                                      integer value between 2 and 1000
//                            Outputs ---------------------------------------------------
//                            Freq -- frequency at which the Capture Loop is run in Hertz
//                            Time -- duration of test in seconds
//                            Reference_Input -- duty cycle of PWM output; an integer value
//                                               between 0 and 1023
//                            Count -- encoder count; a signed integer value 
//                               
//      3 = Closed Loop Step with PID Controller -- applies a discrete PID Controller based 
//                                                    on Kp, Ki,and Kd values for a step input
//                            Inputs  ---------------------------------------------------
//                            Kp -- Proportional gain value
//                            Ki -- Integral gain value
//                            Kd -- Derivative gain value
//                            N -- Filter Coefficient value for Derivative output calculation in rad/s
//                            Reference_Input -- gain of step input in radians
//                            I_Gain -- gain of the Input Filter function I(s) in encoder counts/radians
//                            V_In -- Power supplied to Motor Controller in volts
//                            Time -- sets the duration of the test in seconds  
//                            Period -- sets the Control Loop period in milli seconds; an
//                                      integer value between 2 and 1000
//                            Outputs ---------------------------------------------------
//                            Freq -- frequency at which the Control Loop is run in Hertz
//                            Time -- duration of test in seconds
//                            Reference_Input -- rotational position in radians
//                            Count -- encoder count; a signed integer value 
//
//      4 = User Defined Function
//
//
//
////////////////////////////////////////////////////////////////////////////////////////// 

// Declare Mode of Operation
static byte Mode = 0;

// Declare Step Input PWM output (0-1023)
static float Step_Input = 0;

// Declare Start Ramp PWM output (0-1023) 
static float Ramp_Start = 0;

// Declare Final Ramp PWM output (0-1023) 
static float Ramp_Final = 0;

// Declare Final Frequency of Chirp
static float Freq_Final = 0;

// Declare PWM output amplitude of Chirp (0-1023)
static float PWM_Amp = 0;

// Declare PID Gains 
static float Kp = 0;                     
static float Ki = 0;                
static float Kd = 0;                
static float N = 0;                 

// Declare Desired Input Value (radians)
float Reference_Input = 0;          

// Declare Input Filter Function I(s) (assumed to be a gain)
float I_Gain = 0;

// Declare Voltage Input (volts)
float V_in = 0;

// Declare Capture/Control Loop Period in milli seconds (2-1000)
static unsigned int Period = 0;

// Declare Test Duration in seconds
static float Time = 0;

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// Read Encoder Position Function
signed long int Read_Encoder()
{
  signed long int Count = 0;     // encoder position total
  signed long int temp0;         // temp variable used for reading in encoder value
  signed long int temp1;         // temp variable used for reading in encoder value 
  signed long int temp2;         // temp variable used for reading in encoder value
  signed long int temp3;         // temp variable used for reading in encoder value
    
  // initiate sequence to retrieve encoder counter values    
  digitalWrite(OE, HIGH);
  digitalWrite(OE, LOW);

  // retrieve byte 0 from encoder counter 
  digitalWrite(SEL1, HIGH);              
  digitalWrite(SEL2, LOW);
  temp0 = PINA;               // Read in values from Port A

  // retrieve byte 1 from encoder counter  
  digitalWrite(SEL1, LOW);
  digitalWrite(SEL2, LOW);
  temp1 = PINA;               // Read in values from Port A
    
  // retreive byte 2 from encoder counter  
  digitalWrite(SEL1, HIGH);
  digitalWrite(SEL2, HIGH);
  temp2 = PINA;               // Read in values from Port A    
    
  // retrieve byte 3 from encoder counter  
  digitalWrite(SEL1, LOW);
  digitalWrite(SEL2, HIGH);
  temp3 = PINA;               // Read in values from Port A  
  
  Count = temp0 + (temp1 << 8)+ (temp2 << 16)+ (temp3 << 24);     // calculate encoder count 
  
  // end encoder counter read sequence
  digitalWrite(OE, HIGH);  
 
  return Count;
} 

// Reset Encoder Counter Function
void Reset_Encoder_Counter()
{
  // Reset Encoder Counter by toggling RST pin
  digitalWrite(RST, HIGH); 
  delay(100);
  digitalWrite(RST, LOW);          
  delay(100);
  digitalWrite(RST, HIGH); 
}

// Ready Motor Function
void Ready_Motor()
{
  // The following sequence will toggle the Reset pin to clear all fault flags
  digitalWrite(Reset, HIGH);         // Set Motor Controller Reset pin HIGH
  delay(100);
  digitalWrite(Reset, LOW);          // Set Motor Controller Reset pin LOW
  delay(100);
  digitalWrite(Reset, HIGH);         // Set Motor Controller Reset pin HIGH  
}

// Stop Motor Function
void Stop_Motor()
{
  digitalWrite(PWM_Out, LOW);         // Set Motor Controller PWM low to turn off output
}

// Move Motor Function
void Move_Motor(long int PWM_Value)
{  
  if(PWM_Value >= 0)               
  {  
    if(PWM_Value > 1023)                // Limit Controller_Output to 0 - 1023
      PWM_Value = 1023;    
    analogWrite(PWM_Out, PWM_Value);    // Set PWM_Out to PWM signal     
    digitalWrite(DIR, HIGH);            // Set DIR pin HIGH
  }
  else
  {    
    PWM_Value = -PWM_Value;             // Invert signal to keep positive
    if(PWM_Value > 1023)                // Limit Controller_Output to 0 - 1023
      PWM_Value = 1023; 
    analogWrite(PWM_Out, PWM_Value);    // Set PWM2_Out to PWM signal
    digitalWrite(DIR, LOW);             // Set DIR pin LOW
  }  
}

// Open Loop Step Function
void OLStep()
{
  Controller_Output = Step_Input;
  Reference_Input = Controller_Output;
}

// Open Loop Ramp Function
void OLRamp()
{ 
  temp += Ramp_Slope;                          // When setting Reference_Input += Ramp_Slope directly, Arduino pins  
  Reference_Input = temp + Ramp_Start;         // would change Modes; had to add temp variable to act as intermediary; now works
  Controller_Output = Reference_Input;
}

// Open Loop Chirp Function
void OLChirp()
{
  t = Cnt/Freq;                                 // Calculate current time in seconds from start
  temp = pi*Chirp_Rate*t*t;        
  Reference_Input = PWM_Amp*sin(pi + temp);     // Calculate Chirp Sigal
  Controller_Output = Reference_Input;
} 

// Closed Loop Step Function with PID Controller
void CLStep(signed long int Target_Position)
{
  E[0] = Target_Position - Encoder_Count;            // Calculate current error value 
  
  CO[0] = -C1*CO[1] - C2*CO[2] + E0*E[0] + E1*E[1] + E2*E[2];      // Calculate new Controller Output in volts           
  Controller_Output = CO[0]*VtoPWM;                                // Convert from volts to PWM duty cycle for analogWrite function   

  CO[2] = CO[1];           // save Controller Output @ t-1 as Controller Output @ t-2
  CO[1] = CO[0];           // save Controller Output @ t as Controller Output @ t-1
  E[2] = E[1];             // save Error @ t-1 as Error @ t-2
  E[1] = E[0];             // save Error @ t as Error @ t-1
}  

// Main Setup Routine
void setup() 
{
  Serial.begin(115200);          // Setup serial output to 115,200 bps (bits per second)
     
  // Setup Data input pins from Encoder as inputs
  DDRA = 0;                      // Set pins 22-29 as inputs

  // Setup Motor Controller 1 pins as Outputs
  pinMode(Reset, OUTPUT);        // Setup digital pin 4 as an output
  pinMode(PWM_Out, OUTPUT);      // Setup digital pin 5 as an output
  pinMode(DIR, OUTPUT);          // Setup digital pin 6 as an output
  
  Stop_Motor();
  
  // Setup Encoder 1 Counter pins as Outputs
  pinMode(SEL1, OUTPUT);         // Setup digital pin 38 as an output 
  pinMode(OE, OUTPUT);           // Setup digital pin 39 as an output  
  pinMode(RST, OUTPUT);          // Setup digital pin 40 as an output
  pinMode(SEL2, OUTPUT);         // Setup digital pin 41 as an output   
  
  // Setup Interrupt Service Routine driven by a rising edge on int 0(Pin2) for E-Stop
  pinMode(2, INPUT);                     // Setup digital pin 2 as an input
  attachInterrupt(0,STOP,RISING);        // Setup 'STOP' interrupt to begin on RISING edge
  
  // Setup Interrupt Service Routine driven by a rising edge on int 1(Pin3) for Start Button
  pinMode(3, INPUT);                     // Setup digital pin 3 as an input
  attachInterrupt(1,START,RISING);       // Setup 'START' interrupt to begin on RISING edge
  
  // Adjust PWM Timer 3 for Pin 5 to work in Fast PWM mode with 10-bit resolution at 15.6kHz; 
  // set clock prescaler to 1  
  TCCR3A |= 3;           // Set WGM1 & WGM0 to 1 
  TCCR3B &= ~15;         // Set bits 3,2,1 and 0 to 0 
  TCCR3B |= 9;           // Set WGM2 & CS0 to 1 
  
  if(Period < 2)         // Check to see if the Period set by the user is less than 2 ms
    Period = 2;          // If it is, reset back to 2 ms
  
  // Setup Timer for Capture/Control Loop Frequency
  MsTimer2::set(Period, Timer_ISR);   // Set Capture/Control Loop Frequency (Hz) to 1000/Period  
}

void loop()
{  
  // Define Local Variables  
  long unsigned int Cnt_Max = 0;        // Number of iterations to be preformed

  // Reset Global Variables  
  Go = 0;                               // Reset START flag      
  Stop = 0;                             // Reset E-STOP flag    
  CO[0] = 0;                            // Reset Controller Output at t
  CO[1] = 0;                            // Reset Controller Output at t-1
  CO[2] = 0;                            // Reset Controller Output at t-2
  E[0] = 0;                             // Reset Error at t
  E[1] = 0;                             // Reset Error at t-1
  E[2] = 0;                             // Reset Error at t-2
  
  // Precalculate Catpture/Control Loop Variables to speed up execuation time
  Ts = (float)Period/1000;              // Catpture/Control Loop Period converted to seconds from milliseconds
  Freq = 1/Ts;                          // Calculate Catpture/Control Loop Frequency in Hz
  Reference_Input_Encoder = int(Reference_Input * I_Gain);     // Convert Reference Input from radians to encoder counts
  Cnt_Max = Freq * Time;                // Calculate number of interations to be performed
  VtoPWM = 1023/V_in;                   // Conversion factor of Volts to PWM duty cycle for analogWrite function 
  Ramp_Slope = (Ramp_Final-Ramp_Start)/(Time*Freq);  // Calculate Ramp Slope [(start value - final value) / number of iterations]  
  Chirp_Rate = Freq_Final/Time;         // Calculate Chirp Rate (final frequency / time) 
  C1 = -(2 + N*Ts)/(1 + N*Ts);          // Calculate Controller Output coefficient @ t-1
  C2 = 1/(1 + N*Ts);                    // Calculate Controller Output coefficient @ t-2  
  E0 = (Kp*(1 + N*Ts) + Ki*Ts*(1 + N*Ts) + Kd*N)/(1 + N*Ts);      // Calculate Error coefficient @ t
  E1 = -(Kp*(2 + N*Ts) + Ki*Ts + 2*Kd*N)/(1 + N*Ts);              // Calculate Error coefficient @ t-1
  E2 = (Kp + Kd*N)/(1 + N*Ts);                                    // Calculate Error coefficient @ t-2

  // Send out initial settings
  Serial.println(Freq);                 // Send Freq value out serially
  Serial.println(Time);                 // Send Time value out serially
  Serial.println(I_Gain);               // Send I_Gain value out serially
    
  Stop_Motor();
    
  while(!Go);                           // Wait for START push button to be pressed 
  delay(100);                           // Set delay of 100ms to debounce switch
  Go = 0;                               // Reset START flag 
     
  Reset_Encoder_Counter();              // Reset Encoder Counter to position 0
  Ready_Motor();                        // Get motor ready
  MsTimer2::start();                    // Start timer  
  
  for(Cnt=0;Cnt <= Cnt_Max;Cnt++)     
  { 
    
    while(Timer_Go == 0);               // Wait for next Timer iteration`          
    Timer_Go = 0;                       // Reset Timer flag
    
    Serial.println(Reference_Input);    // Send Reference_Input value out serially
    Serial.println(Encoder_Count);      // Send Encoder_Count value out serially    

    if((Stop) || (Stop_bit))            // If E-Stop has been pressed, break from loop
      break;
  }
  
  Stop_Motor();                         // Stop motor  
  MsTimer2::stop();                     // Stop timer
  delay(100);                           // Set delay of 100ms to debounce switch
} 

// Timer Interrupt Service Routine trigger by MsTimer2 function
void Timer_ISR()
{ 
  Encoder_Count = Read_Encoder();       // Read in current encoder position   
  
  // Open Loop Step Mode
  if(Mode == 0)                     
    OLStep();   
  
  // Open Loop Ramp Mode
  else if(Mode == 1)                
    OLRamp();  

  // Open Loop Chirp Mode
  else if(Mode == 2)
    OLChirp();
  
  // Closed Loop Step Mode with PID Controller
  else if(Mode == 3)  
    CLStep(Reference_Input_Encoder);
  
  //////////////////////////////////////////////////////////////////////////////////////
  //  User Defined Mode
  //////////////////////////////////////////////////////////////////////////////////////
  else if(Mode == 4)
  {

    
    
  } 
  //////////////////////////////////////////////////////////////////////////////////////   
  //////////////////////////////////////////////////////////////////////////////////////              
    
  Stop_bit = digitalRead(2);            // Read in status of E-Stop   
  if((!Stop) && (!Stop_bit))            // If the E-Stop has not been pressed then...
    Move_Motor(Controller_Output);      // Call Move Motor Function 
  else                                  // Else...
    Stop_Motor();                       // Call Stop Motor Function 
 
  Timer_Go = 1;                         // Set Timer flag
}  

// Interrupt Service Rountine triggered by RISING edge from user activated
// momentary push button (Start Button)
void START()
{
  Go = 1;                              // Set Push Button flag  
}   
       
// Interrupt Service Rountine triggered by RISING edge from E-Stop
void STOP()
{
  Stop = 1;                           // Set STOP flag
  Stop_Motor();                       // Call Stop Motor Function 
}
