

// Please read the ready-to-localize tuturial together with this example.
// https://www.pozyx.io/Documentation/Tutorials/ready_to_localize
/**
  The Pozyx ready to localize tutorial (c) Pozyx Labs

  Please read the tutorial that accompanies this sketch: https://www.pozyx.io/Documentation/Tutorials/ready_to_localize/Arduino

  This tutorial requires at least the contents of the Pozyx Ready to Localize kit. It demonstrates the positioning capabilities
  of the Pozyx device both locally and remotely. Follow the steps to correctly set up your environment in the link, change the
  parameters and upload this sketch. Watch the coordinates change as you move your device around!
*/

#include <Pozyx_definitions.h>
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include<TimeLib.h>
#include <TimerOne.h>




//The below headers are used for continous printing the Serial.print("")
#ifndef ARDPRINTF
#define ARDPRINTF
#define ARDBUFFER 16
#include <stdarg.h>
#include <Arduino.h>

///////////User defined data types//////////////
/// Fixed-point Format: 11.5 (16-bit)///////
#define FIXED_POINT_FRACTIONAL_BITS 5
#define GRAVITY_FACTOR 0.1019
#define MICRO_TESLA_CONVERSION 0.0625
typedef int16_t fixed_point_t;


////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

boolean debug = false;                            //change here to debug using serial monitor  

int32_t startTrans = 2147483647;                      // start transmission with this number XYZ coordinates
int32_t stopTrans = 2130706431;                       // end of transmission of 1 set of "XYZ" coordinates

byte b[4];                                            //used for converting int32_t to 4*int8_t for uart transmission
byte b1[2];                                           //used for converting int16_t to 2*int8_t for uart transmission
uint32_t TimerCount=0;

bool interruptFlag = true;                             //Flag that is placed inside an interrupt

uint16_t remote_id = 0x6F50;                            // set this to the ID of the remote device
bool remote = false;                                    // set this to true to use the remote ID
boolean use_processing = false;                         // set this to true to output data for the processing sketch

const uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[num_anchors] = {0x666E, 0x6646, 0x666C, 0x6643};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[num_anchors] = {0,0,5120,6722};               // anchor x-coorindates in mm
int32_t anchors_y[num_anchors] = {0,8647,8647,0};                  // anchor y-coordinates in mm
int32_t heights[num_anchors] = {2918,2657,3192,2679};              // anchor z-coordinates in mm
//uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;             // positioning algorithm to use. try for fast moving objects.
uint8_t algorithm = POZYX_POS_ALG_TRACKING;
uint8_t dimension = POZYX_3D;                           // positioning dimension
int32_t height = 0;                                  // height of device, required in 2.5D positioning

////////////////////////////////////////////////

void setup(){
  Serial.begin(115200);
  
  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    
    abort();
  }

  if(!remote){
    remote_id = NULL;
  }

  // clear all previous devices in the device list
  Pozyx.clearDevices(remote_id);
  // sets the anchor manually
  setAnchorsManual();
  // sets the positioning algorithm
  Pozyx.setPositionAlgorithm(algorithm, dimension, remote_id);

  //printCalibrationResult();
  //delay(2000);

  
  Serial.flush();             //Flushing the buffer 
   
  // Initializing the interrupt
  Timer1.initialize(60000);           //microsec
  Timer1.attachInterrupt( arduinoTime ); //Attach interrupt to the Timer1 interrupt
 }

void loop(){
  
  coordinates_t position;//pozyx gets the position data and stores it in here
  linear_acceleration_t LinAcc;
  magnetic_t Mag;
  angular_vel_t AngVel;
  pos_error_t ErrCov;
  
  
  int status;

  TimerCount=TimerCount+1;
  
  printstartTrans(startTrans);              //send start transmission signal to uart
  printTimerCount(TimerCount);              //send timer signal to uart
  
    
  //getting postion from pozyx[int32_t]
  if(remote){
    status = Pozyx.doRemotePositioning(remote_id, &position, dimension, height, algorithm);
  }else{
    status = Pozyx.doPositioning(&position, dimension, height, algorithm);
  }

  if (status == POZYX_SUCCESS){
    // prints out the result
    printCoordinates(position);
    }else{
    // prints out the error code
    printErrorCode("positioning");
  }
  

 //getting linear acceleration[int32_t]
  if(remote){
  status = Pozyx.getLinearAcceleration_mg(remote_id,&LinAcc); //getting acceleration data
  }else{
    status = Pozyx.getLinearAcceleration_mg(&LinAcc);
  }
  if (status == POZYX_SUCCESS){
    printLinAcc(LinAcc);
  }else{
    Serial.println("error linear acceleration");  
  }
  
 //getting magnetic_ut [float32_t]
  if(remote){
  status = Pozyx.getMagnetic_uT(remote_id, &Mag); 
  }else{
  status = Pozyx.getMagnetic_uT(&Mag);
  }
  if (status == POZYX_SUCCESS){
    printMag(Mag);
  }else{
    Serial.println("error magnetic");  
  }
  
  //getting gyro(angular velocity) [float32_t]
  if(remote){
  status = Pozyx.getAngularVelocity_dps(remote_id, &AngVel); 
  }else{
  status = Pozyx.getAngularVelocity_dps(&AngVel);
  }
  if (status == POZYX_SUCCESS){
    printAngVel(AngVel);
  }else{
    Serial.println("error Angular Velocity");  
  }
  
  //getting position covariance [int_16t]
  if(remote){
  status = Pozyx.getPositionError(remote_id,&ErrCov); //getting acceleration data
  }else{
  status = Pozyx.getPositionError(&ErrCov);
  }
  if (status == POZYX_SUCCESS){
    printErrCov(ErrCov);                    
  }else{
    Serial.println("error Error Covariance");  
  }


 //print stop transmission signal
 printstopTrans(stopTrans);           

  //making the timer '0' to prevent overflow
  if(TimerCount==4294967295){
      TimerCount=1;
    }
  interruptFlag=false;
}





//Interrupt Service Routine [ISR]
void arduinoTime(){
  interruptFlag = true;
}

// prints the coordinates for either humans or for processing
void printCoordinates(coordinates_t coor){
  uint16_t network_id = remote_id;
  if (network_id == NULL){
    network_id = 0;
  }
  // below are the position coordinates sent to serial monitor(arduino)
  if(!use_processing){
      if (debug == true){
        if (interruptFlag = true){  
          //Serial.print("Position Coordinates = ");
          Serial.print(coor.x);
          Serial.print(",");
          Serial.print(coor.y);
          Serial.print(",");
          Serial.println(coor.z);
          
        }
      }else{
        // The coordinates are sent to UART ,which is then caught by the octave.
          IntegerToBytes(coor.x);
          serialWrite(4);
          IntegerToBytes(coor.y);
          serialWrite(4);
          IntegerToBytes(coor.z);
          serialWrite(4);                  
      }   
  }else{
    // dont use this not needed
    Serial.print("POS,0x");
    Serial.print(network_id,HEX);
    Serial.print(",");
    Serial.print(coor.x);
    Serial.print(",");
    Serial.print(coor.y);
    Serial.print(",");
    Serial.println(coor.z);
  }
}

void printTimerCount(uint16_t TimerCount1)
{
  if(debug == true){
    Serial.print("TimerCount = ");
    Serial.print(TimerCount1);
    Serial.print("\n");
  }else{
    IntegerToBytes(TimerCount1);
    serialWrite(4);
  }
}

void printstartTrans(int32_t startTrans1){
  if(debug == true){
    Serial.print("Start Transmission = ");
    Serial.print(startTrans1);
    Serial.print("\n");
  }else{
    IntegerToBytes(startTrans1);
    serialWrite(4);
    //serialPrint(4);
  
  }
}

void printstopTrans(int32_t stopTrans1){
  if(debug == true){
    Serial.print("Stop Transmission = ");
    Serial.print(stopTrans1);
    Serial.print("\n");
  }else{
    IntegerToBytes(stopTrans1);
    serialWrite(4);
    //serialPrint(4);
  }
}

void printErrCov(pos_error_t ErrCov1){
  uint16_t ErrCovX;
  if(debug == true){
    Serial.print("Error Variance = ");
    Serial.print(ErrCov1.x);
    Serial.print(",");
    Serial.print(ErrCov1.y);
    Serial.print(",");
    Serial.print(ErrCov1.z);
    Serial.print("\n");
    Serial.print("ErrCov xy,xz,yz = ");
    Serial.print(ErrCov1.xy);
    Serial.print(",");
    Serial.print(ErrCov1.xz);
    Serial.print(",");
    Serial.print(ErrCov1.yz);
    Serial.print("\n");
  }else{
    IntegerToBytes16(ErrCov1.x);
    serialWrite(2);
    IntegerToBytes16(ErrCov1.y);
    serialWrite(2);
    IntegerToBytes16(ErrCov1.z);
    serialWrite(2);
    IntegerToBytes16(ErrCov1.xy);
    serialWrite(2);
    IntegerToBytes16(ErrCov1.xz);
    serialWrite(2);
    IntegerToBytes16(ErrCov1.yz);
    serialWrite(2);     
  }
}

void printLinAcc (linear_acceleration_t LinAcc1){
  if (debug == true){
    Serial.print("Linear Acceleration = ");
    Serial.print(LinAcc1.x);
    Serial.print(",");
    Serial.print(LinAcc1.y);
    Serial.print(",");
    Serial.print(LinAcc1.z);
    Serial.print("\n");
    //float to fixed checker
    /*int a=float_to_fixed(LinAcc1.x);
    Serial.print(a);
    Serial.print(",");
    Serial.print(fixed_to_float(a,5));*/
  }else{
    IntegerToBytes16(float_to_fixed(LinAcc1.x*GRAVITY_FACTOR));
    serialWrite(2);
    IntegerToBytes16(float_to_fixed(LinAcc1.y*GRAVITY_FACTOR));
    serialWrite(2);
    IntegerToBytes16(float_to_fixed(LinAcc1.z*GRAVITY_FACTOR));
    serialWrite(2);
    }
}

void printMag(magnetic_t Mag1){
  if (debug == true){
    Serial.print("Mag =");
    Serial.print(Mag1.x);
    Serial.print(",");
    Serial.print(Mag1.y);
    Serial.print(",");
    Serial.print(Mag1.z);
    Serial.print("\n");  
  }else{
    IntegerToBytes16(float_to_fixed(Mag1.x*MICRO_TESLA_CONVERSION));
    serialWrite(2);
    IntegerToBytes16(float_to_fixed(Mag1.y*MICRO_TESLA_CONVERSION));
    serialWrite(2);
    IntegerToBytes16(float_to_fixed(Mag1.z*MICRO_TESLA_CONVERSION));
    serialWrite(2);
  }
}

void printAngVel(angular_vel_t AngVel1){
  if (debug == true){
    Serial.print("Angular Vel = ");
    Serial.print(AngVel1.x);
    Serial.print(",");
    Serial.print(AngVel1.y);
    Serial.print(",");
    Serial.print(AngVel1.z);
    Serial.print("\n");
  } else{
  IntegerToBytes16(float_to_fixed(AngVel1.x));  //Converting to fixed point for uart transfer
  serialWrite(2);
  IntegerToBytes16(float_to_fixed(AngVel1.y));
  serialWrite(2);
  IntegerToBytes16(float_to_fixed(AngVel1.z));
  serialWrite(2);
}
}


//Print bytes in arduino serial monitor[used for debugging]
void serialPrint(int n){
for (int i=0; i<n; ++i) {
    Serial.print((int )b[i]);
    Serial.print("\n");
}
}

//Write bytes to uart 
void serialWrite(int n){
for (int i=0; i<n; ++i) {
    Serial.write((int )b[i]);
}
}

// Integer to byte conversion function for uart transmission. 32 bits
void IntegerToBytes(int32_t val) {
  b[3] = (byte )((val >> 24) & 0xff);
  b[2] = (byte )((val >> 16) & 0xff);
  b[1] = (byte )((val >> 8) & 0xff);
  b[0] = (byte )(val & 0xff);
}

// Integer to byte conversion function for uart transmission. 16 bits 
void IntegerToBytes16(int16_t val1){
  b[3] = 0x00;
  b[2] = 0x00;
  b[1] = (byte )((val1 >> 8) & 0xff);
  b[0] = (byte )(val1 & 0xff);
}

//Float to fixed convertion function
fixed_point_t float_to_fixed(float input){
    return (fixed_point_t)(round(input * (1 << FIXED_POINT_FRACTIONAL_BITS)));    //fixed_point_t = uint16_t[typedef]
}


// Convert  fixed-point to float
float fixed_to_float(int16_t input, uint8_t fractional_bits){
    return ((float)input / (float)(1 << fractional_bits));
}


// error printing function for debugging
void printErrorCode(String operation){
  if(debug == true){
    Serial.print("Error X, Y, Z = 0, 0, 0 ");
    Serial.print("\n");
  }else{
    IntegerToBytes16(0);
    serialWrite(4);
    IntegerToBytes16(0);
    serialWrite(4);
    IntegerToBytes16(0);
    serialWrite(4);     
}
}
// print out the anchor coordinates (also required for the processing sketch)
void printCalibrationResult(){
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size, remote_id);


  if(list_size == 0){
    printErrorCode("configuration");
    return;
  }

  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids, list_size, remote_id);


}

// function to manually set the anchor coordinates
void setAnchorsManual(){
  for(int i = 0; i < num_anchors; i++){
    device_coordinates_t anchor;
    anchor.network_id = anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = heights[i];
    Pozyx.addDevice(anchor, remote_id);
  }
  if (num_anchors > 4){
    Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, num_anchors, remote_id);
  }
}





// ardprintf command for printing
int ardprintf(char *str, ...)
{
  int i, count=0, j=0, flag=0;
  char temp[ARDBUFFER+1];
  for(i=0; str[i]!='\0';i++)  if(str[i]=='%')  count++;

  va_list argv;
  va_start(argv, count);
  for(i=0,j=0; str[i]!='\0';i++)
  {
    if(str[i]=='%')
    {
      temp[j] = '\0';
      Serial.print(temp);
      j=0;
      temp[0] = '\0';

      switch(str[++i])
      {
        case 'd': Serial.print(va_arg(argv, int));
                  break;
        case 'l': Serial.print(va_arg(argv, long));
                  break;
        case 'f': Serial.print(va_arg(argv, double));
                  break;
        case 'c': Serial.print((char)va_arg(argv, int));
                  break;
        case 's': Serial.print(va_arg(argv, char *));
                  break;
        default:  ;
      };
    }
    else 
    {
      temp[j] = str[i];
      j = (j+1)%ARDBUFFER;
      if(j==0) 
      {
        temp[ARDBUFFER] = '\0';
        Serial.print(temp);
        temp[0]='\0';
      }
    }
  };
  Serial.println();
  return count + 1;
}
#undef ARDBUFFER
#endif
