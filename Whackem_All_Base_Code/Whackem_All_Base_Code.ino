///////////////////////////////////////////////////////////
//  Whack'em All Base Code v4
//  November 16, 2017
//  By Chris McClellan
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

// Setup Emitter Pins for Optical Sensors 1-4 
static byte EM1 = 46;           // Emmitter on Optical Sensor 1 assigned to Digital Pin 46
static byte EM2 = 47;           // Emmitter on Optical Sensor 2 assigned to Digital Pin 47
static byte EM3 = 48;           // Emmitter on Optical Sensor 3 assigned to Digital Pin 48
static byte EM4 = 49;           // Emmitter on Optical Sensor 4 assigned to Digital Pin 49

// Setup Target Done Output
static byte Target_Done = 11;  // Target Done, which tells this slave Mega that it can move onto the next target, is assigned to Pin 11 

// Declare Contstants
static float pi = 3.141593;

// Setup ISR Global Variables
volatile byte Go = 0;                           // Interrupt flag for START
volatile byte Timer_Go = 0;                     // Interrupt flag for Timer
volatile byte Stop = 0;                         // Interrupt flag for E-STOP  
volatile byte Received = 0;                     // Interrupt flag for signaling when the optical sensor is within the slot
volatile byte DONE = 0;                         // Interrupt flag for signaling when the 2.0s interval has been satisfied 
volatile long signed int Received_Position = 0; // Encoder position when ISR for Receiver is triggered
volatile long signed int Time_Count = 0;        // Time Count variable
volatile long signed int Received_Count = 0;    // Received Time count for when Recieved funtion is triggered 
volatile signed long int Reference_Input_Encoder = 0;   // Reference Input Signal in encoder counts (R(s)*I(s))
volatile signed long int Encoder_Count = 0;   // Current Encoder position in encoder counts
volatile float Step_Input = 0;                // Step Input PWM output (0-255)
volatile float Reference_Input = 0;           // Input reference signal
volatile float VtoPWM = 0;                    // Conversion factor for Volt to PWM  
volatile float KdxFreq = 0;                   // Combined gain of Kd*Freq  
volatile float PeriodinSeconds = 0;           // Control Loop Period in seconds
volatile float Controller_Output = 0;         // Control loop Controller_Output
volatile float CO[3] = {0,0,0};               // Controller Output array in volts 
volatile float E[3] = {0,0,0};                // Error array in encoder counts
volatile float C1 = 0;                        // Controller Output coefficient @ t-1 
volatile float C2 = 0;                        // Controller Output coefficient @ t-2 
volatile float E0 = 0;                        // Error coefficient @ t  
volatile float E1 = 0;                        // Error coefficient @ t-1 
volatile float E2 = 0;                        // Error coefficient @ t-2 
volatile float Ts = 0;                        // Capture/Control Loop Period in seconds
volatile float Freq = 0;       // Calculated Control Loop Frequency

// Voltage Input set to 12.0 Volts
float V_in = 12.0;

//////////////////////////////////////////////////////////////////////////////////////////
// User Defined Global Variables
//////////////////////////////////////////////////////////////////////////////////////////

// Declare Target Sequence
volatile byte Target_Sequence[] = {1,2,3,4,3,2,1,4,3,2};
// {1,2,3,4}

// Declare number of targets in Target Sequence
volatile byte TS_Length = 10;

// Declare PID Gains
// old Kp (lab 2): 0.0635
static float Kp = 0.0635;                // Set Proportioanl Gain     
static float Ki = 0;                // Set Integral Gain

// old Kd (lab 2): 0.00365
static float Kd = 0.00365;                // Set Derivative Gain

// old N (lab 2): 958.1013
static float N = 958.1013; // TODO

// Set Input Filter Function Gain (I(s)) 
float I_Gain = 651.9; //

// Control Loop Period in milli seconds 
// old Period (lab 2): 0.0131
static unsigned int Period = 0.0131; // TODO

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
    
  // retrieve byte 2 from encoder counter  
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

// Closed Loop Step Function with PID Controller
// Note: it is assumed that every Controller has a Proportional term
void CLStep(signed long int Target_Position)
{
  E[0] = Target_Position - Encoder_Count;            // Calculate current error value 
  
  CO[0] = -C1*CO[1] - C2*CO[2] + E0*E[0] + E1*E[1] + E2*E[2];      // Calculate new Controller Output in volts           
  Controller_Output = CO[0]*VtoPWM;                                // Convert from volts to PWM duty cycle for analogWrite function   

  CO[2] = CO[1];           // save Controller Output @ t-1 as Controller Output @ t-2
  CO[1] = CO[0];           // save Controller Output @ t as Controller Output @ t-1
  E[2] = E[1];             // save Error @ t-1 as Error @ t-2
  E[1] = E[0];             // save Error @ t as Error @ t-1                              // Save old error value
} 

// Turn Off Emitters Function
void Turn_Off_Emitters()
{
  digitalWrite(EM1, LOW);                              // Turn off Emitters of all Optical Sensors
  digitalWrite(EM2, LOW);
  digitalWrite(EM3, LOW);
  digitalWrite(EM4, LOW);
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
  
  // Setup Emitter Pins for Optical Sensors 1-4
  pinMode(EM1, OUTPUT);           // Setup digital pin 46 as an output
  pinMode(EM2, OUTPUT);           // Setup digital pin 47 as an output
  pinMode(EM3, OUTPUT);           // Setup digital pin 48 as an output
  pinMode(EM4, OUTPUT);           // Setup digital pin 49 as an output
  
  // Setup Target Done signal
  pinMode(Target_Done, INPUT);  // Setup digital pin 11 as an input
  
  // Setup Interrupt Service Routine driven by a rising edge on int 0(Pin2) for E-Stop
  pinMode(2, INPUT);                     // Setup digital pin 2 as an input
  attachInterrupt(0,STOP,RISING);        // Setup 'STOP' interrupt to begin on RISING edge
  
  // Setup Interrupt Service Routine driven by a rising edge on int 1(Pin3) for Start Button
  pinMode(3, INPUT);                     // Setup digital pin 3 as an input
  attachInterrupt(1,START,RISING);       // Setup 'START' interrupt to begin on RISING edge
  
    // Setup Interrupt Service Routine driven by a CHANGING edge on int 2(Pin21) for RECEIVER 4
  pinMode(21, INPUT);                     // Setup digital pin 21 as an input
  attachInterrupt(2,RECEIVER4,CHANGE);    // Setup 'RECEIVER1' interrupt to begin on a CHANGING edge 
 
  // Setup Interrupt Service Routine driven by a CHANGING edge on int 3(Pin20) for RECEIVER 3
  pinMode(20, INPUT);                     // Setup digital pin 20 as an input
  attachInterrupt(3,RECEIVER3,CHANGE);    // Setup 'RECEIVER2' interrupt to begin on a CHANGING edge  

  // Setup Interrupt Service Routine driven by a CHANGING edge on int 4(Pin19) for RECEIVER 2
  pinMode(19, INPUT);                     // Setup digital pin 19 as an input
  attachInterrupt(4,RECEIVER2,CHANGE);    // Setup 'RECEIVER3' interrupt to begin on a CHANGING edge
  
  // Setup Interrupt Service Routine driven by a CHANGING edge on int 5(Pin18) for RECEIVER 1
  pinMode(18, INPUT);                     // Setup digital pin 18 as an input
  attachInterrupt(5,RECEIVER1,CHANGE);    // Setup 'RECEIVER4' interrupt to begin on a CHANGING edge
  
  // Adjust PWM Timer 3 for Pin 5 to work in Fast PWM mode with 10-bit resolution at 15.6kHz; 
  // set clock prescaler to 1  
  TCCR3A |= 3;           // Set WGM1 & WGM0 to 1 
  TCCR3B &= ~15;         // Set bits 3,2,1 and 0 to 0 
  TCCR3B |= 9;           // Set WGM2 & CS0 to 1 
  
  if(Period < 2)         // Check to see if the Period set by the user is less than 2 ms
    Period = 2;          // If it is, reset back to 2 ms
  
  // Setup Timer for Control Loop Frequency
  MsTimer2::set(Period, Timer_ISR);   // Set Control Loop Frequency (Hz) to 1000/Period  
}

void loop()
{  
  // Define Local Variables  
  int i = 0;                              // Sequence counter  
  byte Stop_bit = 0;                      // E-Stop input flag 
  unsigned long int Start_Count = 0;      // Start of sequence time count
  float Elapsed_Time = 0;                 // Total time for sequence 

  // Reset Global Variables  
  Go = 0;                               // Reset START flag      
  Stop = 0;                             // Reset E-STOP flag     
  Time_Count = 0;                       // Reset global Time Counter (in milliseconds)
  DONE = 0;                             // Reset DONE flag  
  CO[0] = 0;                            // Reset Controller Output at t
  CO[1] = 0;                            // Reset Controller Output at t-1
  CO[2] = 0;                            // Reset Controller Output at t-2
  E[0] = 0;                             // Reset Error at t
  E[1] = 0;                             // Reset Error at t-1
  E[2] = 0;                             // Reset Error at t-2
  
  /////////////////////////////////////////////////////////////////////////////////////
  // User Defined Local Variables and Global Variable Reset
  /////////////////////////////////////////////////////////////////////////////////////
  
  
  /////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////  
    
  // Precalculate Loop Variables to speed up execuation time
  Ts = (float)Period/1000;              // Control Loop Period converted to seconds from milliseconds
  Freq = 1/Ts;                          // Calculate Control Loop Frequency in Hz
  KdxFreq = Kd*Freq;                    // Combined gain of Kd*Freq 
  VtoPWM = 1023/V_in;                   // Conversion factor of Volts to PWM 
  C1 = -(2 + N*Ts)/(1 + N*Ts);          // Calculate Controller Output coefficient @ t-1
  C2 = 1/(1 + N*Ts);                    // Calculate Controller Output coefficient @ t-2  
  E0 = (Kp*(1 + N*Ts) + Ki*Ts*(1 + N*Ts) + Kd*N)/(1 + N*Ts);      // Calculate Error coefficient @ t
  E1 = -(Kp*(2 + N*Ts) + Ki*Ts + 2*Kd*N)/(1 + N*Ts);              // Calculate Error coefficient @ t-1
  E2 = (Kp + Kd*N)/(1 + N*Ts);                                    // Calculate Error coefficient @ t-2
  
  //////////////////////////////////////////////////////////////////////////////////////
  // User Defined Setup Code
  //////////////////////////////////////////////////////////////////////////////////////
  
  // Send out initial settings
  Serial.println(Freq);                 // Send Freq value out serially
  Serial.println(Time);                 // Send Time value out serially
  Serial.println(I_Gain);               // Send I_Gain value out serially
  
  //////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////
    
  Turn_Off_Emitters(); 
  Stop_Motor();  
    
  while(!Go);                     // Wait for START push button to be pressed 
  delay(100);                     // Set delay of 100ms to debounce switch
  Go = 0;                         // Reset START flag 
  
  Reset_Encoder_Counter();        // Reset Encoder Counter to position 0
  Ready_Motor();                  // Get motor ready
  MsTimer2::start();              // Start timer
    
  Serial.println("Wack em All Started.");
  Serial.println("Good Luck...");   
  
  Start_Count = Time_Count;       // Start timing
  
  for(i=0;i < TS_Length;i++)     
  {  
    if(Target_Sequence[i] == 1)
      digitalWrite(EM1, HIGH);
    else if(Target_Sequence[i] == 2)
      digitalWrite(EM2, HIGH);    
    else if(Target_Sequence[i] == 3)
      digitalWrite(EM3, HIGH);
    else if(Target_Sequence[i] == 4)
      digitalWrite(EM4, HIGH);
   
    //////////////////////////////////////////////////////////////////////////////////////
    // User Defined Loop Code
    //////////////////////////////////////////////////////////////////////////////////////
    // actually have to move here... 
    // SEARCH FUNCTION
    while (Received == 0){
       // If E-Stop has been pressed, break from loop
      if((Stop) || (Stop_bit)) {
          break;
      }
      OLStep();
      Move_Motor(Controller_Output);
    }
    
    Stop_Motor(); 
    
    // CONTROL
    if ( Received ==1) {
      volatile long signed int Value = Received_Position+8;
      
      while (DONE != 1){
        //Timer_Go == 1
        
        while (Timer_Go == 1 ) {
          // If E-Stop has been pressed, break from loop
          if((Stop) || (Stop_bit)) {
            break;
          }
          Encoder_Count = Read_Encoder();
          //add the 8 if it reads position but does not set done flag
          CLStep(Value);  
          Move_Motor(Controller_Output);
          Timer_Go = 0;
        }
      }
    }
    //////////////////////////////////////////////////////////////////////////////////////   
    //////////////////////////////////////////////////////////////////////////////////////     
        
    Received = 0;               // Reset Received flag
    DONE = 0;                   // Reset DONE flag
        
    Serial.print("Target ");
    Serial.print(i);
    Serial.println(" completed!"); 
    
    Turn_Off_Emitters();        // Turn off Emitters of all Optical Sensors
  }
  
  Elapsed_Time = (float)(Time_Count - Start_Count)/Freq;
  Serial.print("Elapsed Time = ");
  Serial.print(Elapsed_Time);
  Serial.println("seconds");
  Serial.println();  
  
  Stop_Motor();                  // Stop motor
  MsTimer2::stop();              // Stop timer
  delay(100);                    // Set delay of 100ms to debounce switch
} 

// Timer Interrupt Service Routine trigger by MsTimer2 function
void Timer_ISR()
{
  Timer_Go = 1;                                                    // Set Timer flag
  Time_Count++;                                                    // Increment Time_Count variable
  
  if(((Time_Count - Received_Count) >= ((int)2*Freq)) && (Received == 1))   // If held there for 2 seconds (1000x2ms) and the slot is still lined-up with Optical Sensor...
    DONE = 1;                                                      // Set DONE flag

//  byte Move_On; 
//  Move_On = digitalRead(Target_Done);
//  if(((Time_Count - Received_Count) >= ((int)(2*Freq*0.9))) && (Move_On == 1))
//  {
//    DONE = 1;
//    Received_Count = Time_Count;
//  }
}  

// Interrupt Service Rountine triggered by RISING edge from user activated
// momentary push button (Start Button)
void START()
{
  Go = 1;                   // Set Push Button flag  
}   
       
// Interrupt Service Rountine triggered by RISING edge from E-Stop
void STOP()
{
  Stop = 1;                 // Set STOP flag
  Stop_Motor();             // Call Stop Motor Function 
}

// Interrupt Service Rountine triggered by CHANGING edge from Receiver 1
void RECEIVER1()
{
  byte Receiver_Bit;
  
  Received_Position = Read_Encoder();  // Read in current Encoder position
 
  Receiver_Bit = digitalRead(18);      // Read in status of Receiver pin of Optical Sensor 1
  if(Receiver_Bit)                     // If Receiver Bit = 1 because the slot has just lined up with the Optical Sensor then...
  {  
    Received = 1;                      // Set Received flag to 1 to signal that the slot is now lined up with the Optical Sensor,
    Received_Count = Time_Count;       // Set the Received Count variable to the current time.
  }    
  else                                 // Else Receiver Bit = 0 because the slot is no longer lined up with the Optica Sensor then...
    Received = 0;                      // Set Received flag to 0 to signal that the slot is no longer lined up with the Optical Sensor.
}

// Interrupt Service Rountine triggered by CHANGING edge from Receiver 2
void RECEIVER2()
{
  byte Receiver_Bit;
 
  Received_Position = Read_Encoder();  // Read in current Encoder position 
 
  Receiver_Bit = digitalRead(19);      // Read in status of Receiver pin of Optical Sensor 2
  if(Receiver_Bit)                     // If Receiver Bit = 1 because the slot has just lined up with the Optical Sensor then...
  {  
    Received = 1;                      // Set Received flag to 1 to signal that the slot is now lined up with the Optical Sensor,
    Received_Count = Time_Count;       // Set the Received Count variable to the current time.
  }    
  else                                 // Else Receiver Bit = 0 because the slot is no longer lined up with the Optica Sensor then...
    Received = 0;                      // Set Received flag to 0 to signal that the slot is no longer lined up with the Optical Sensor.
}

// Interrupt Service Rountine triggered by CHANGING edge from Receiver 3
void RECEIVER3()
{
  byte Receiver_Bit;

  Received_Position = Read_Encoder();  // Read in current Encoder position
 
  Receiver_Bit = digitalRead(20);      // Read in status of Receiver pin of Optical Sensor 3
  if(Receiver_Bit)                     // If Receiver Bit = 1 because the slot has just lined up with the Optical Sensor then...
  {  
    Received = 1;                      // Set Received flag to 1 to signal that the slot is now lined up with the Optical Sensor,
    Received_Count = Time_Count;       // Set the Received Count variable to the current time.
  }    
  else                                 // Else Receiver Bit = 0 because the slot is no longer lined up with the Optica Sensor then...
    Received = 0;                      // Set Received flag to 0 to signal that the slot is no longer lined up with the Optical Sensor.
}  

// Interrupt Service Rountine triggered by CHANGING edge from Receiver 4
void RECEIVER4()
{
  byte Receiver_Bit;
  
  Received_Position = Read_Encoder();  // Read in current Encoder position
 
  Receiver_Bit = digitalRead(21);      // Read in status of Receiver pin of Optical Sensor 4
  if(Receiver_Bit)                     // If Receiver Bit = 1 because the slot has just lined up with the Optical Sensor then...
  {  
    Received = 1;                      // Set Received flag to 1 to signal that the slot is now lined up with the Optical Sensor,
    Received_Count = Time_Count;       // Set the Received Count variable to the current time.
  }    
  else                                 // Else Receiver Bit = 0 because the slot is no longer lined up with the Optica Sensor then...
    Received = 0;                      // Set Received flag to 0 to signal that the slot is no longer lined up with the Optical Sensor.
}

