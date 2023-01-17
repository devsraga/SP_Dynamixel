/******************************************************************************

                            Online C Compiler.
                Code, Compile, Run and Debug C program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/

#include <bits/stdc++.h>

using namespace std;
// 1.  comunication with openCM 9.04.................


#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

// 2.  Variables.....
   // 2.1  id assignments.......

int DXL_ID[]={1,2,3,4,5,6};
int delayTime = (6000/180)* 180;
int V_max=8;
int warning_pin_1 = 18; // red led
int warning_pin_2 = 19; // green led
int warning_pin_3 = 20; // blue led
int button_pin_1 = 16;
float goal_pos_0[6]={120,240,120,240,120,240};
float goal_pos_1[6]={60,300,60,300,60,300};
float goal_pos_2[6]={180,180,180,180,180,180};
float goal_pos_3[6]={10,310,10,300,60,300};
float goal_pos_4[6]={90,270,90,270,90,270};
int n = 6;
   // 2.2 protocal..............

const float DXL_PROTOCOL_VERSION = 2.0;

   // 2.3 actuator variables....



Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

float *inv(float px,float py,float pz,float alpha,float bita,float gama){

float a=12;
float b=17;
float d=4;
float t=2.5;
float c1=cos(alpha*(2 * acos(0.0))/180);
float c2=cos(bita*(2 * acos(0.0))/180);
float c3=cos(gama*(2 * acos(0.0))/180);
float s1=sin(alpha*(2 * acos(0.0))/180);
float s2=sin(bita*(2 * acos(0.0))/180);
float s3=sin(gama*(2 * acos(0.0))/180);
float X1=((sqrt(3))/6)*(2*b+d);
float X2=(-((sqrt(3))/6)*(b-d));
float X3=-(((sqrt(3))/6)*(b+2*d));
float X4=(-((sqrt(3))/6)*(b+2*d));
float X5=-(((sqrt(3))/6)*(b-d));
float X6=((sqrt(3))/6)*(2*b+d);
float Y1=.5*d;
float Y2=.5*(b+d);
float Y3=.5*b;
float Y4=((-.5)*b);
float Y5=-(.5*(b+d));
float Y6=-(.5*d);
float Z1=0;
float Z2=0;
float Z3=0; 
float Z4=0;
float Z5=0;
float Z6=0;
float a1[]={X1,Y1,Z1,1};
float a2[]={X2,Y2,Z2,1};
float a3[]={X3,Y3,Z3,1};
float a4[]={X4,Y4,Z4,1};
float a5[]={X5,Y5,Z5,1};
float a6[]={X6,Y6,Z6,1};

//float A={a1';a2';a3';a4';a5';a6'};

float U1=((sqrt(3))/6)*(2*a+t);
float U2=(-((sqrt(3))/6)*(a-t));
float U3=-(((sqrt(3))/6)*(a+2*t));
float U4=(-((sqrt(3))/6)*(a+2*t));
float U5=-(((sqrt(3))/6)*(a-t));
float U6=((sqrt(3))/6)*(2*a+t);
float V1=.5*t;
float V2=.5*(a+t);
float V3=.5*a;
float V4=((-.5)*a);
float V5=-(.5*(a+t));
float V6=-(.5*t);
float W1=0;
float W2=0;
float W3=0;
float W4=0;
float W5=0;
float W6=0;
float T[][4]={
          { c2*c3+s1*s2*s3,    -(c2*s3)+s1*s2*s3,    c1*s2,   px},  
          { c1*s3,               c1*c3,              -s1,     py},
          { (-s2)*c3+s1*c2*s3,   s2*s3+s1*c2*c3,     c1*c2,   pz},
          {0,                    0,                  0,       1}
         };
float b1[]={
          c2*c3+s1*s2*s3*(U1) +  -(c2*s3)+s1*s2*s3*(V1)  +  c1*s2*(W1) + px, 
          c1*s3*(U1)          +   c1*c3*(V1)             +  -s1*(W1)   + py,
          (-s2)*c3+s1*c2*s3*(U1)+   s2*s3+s1*c2*c3*(V1) +     c1*c2*(W1)    + pz,
          0  +                 0      +            0  +     1
         };
float b2[]={
          c2*c3+s1*s2*s3*(U2) +  -(c2*s3)+s1*s2*s3*(V2)  +  c1*s2*(W2) + px, 
          c1*s3*(U2)          +   c1*c3*(V2)             +  -s1*(W2)   + py,
          (-s2)*c3+s1*c2*s3*(U2)+   s2*s3+s1*c2*c3*(V2) +     c1*c2*(W2)    + pz,
          0  +                 0      +            0  +     1
         };
float b3[]={
          c2*c3+s1*s2*s3*(U3) +  -(c2*s3)+s1*s2*s3*(V3)  +  c1*s2*(W3) + px, 
          c1*s3*(U3)          +   c1*c3*(V3)             +  -s1*(W3)   + py,
          (-s2)*c3+s1*c2*s3*(U3)+   s2*s3+s1*c2*c3*(V3) +     c1*c2*(W3)    + pz,
          0  +                 0      +            0  +     1
         };
float b4[]={
          c2*c3+s1*s2*s3*(U4) +  -(c2*s3)+s1*s2*s3*(V2)  +  c1*s2*(W3) + px, 
          c1*s3*(U4)          +   c1*c3*(V2)             +  -s1*(W3)   + py,
          (-s2)*c3+s1*c2*s3*(U4)+   s2*s3+s1*c2*c3*(V2) +     c1*c2*(W3)    + pz,
          0  +                 0      +            0  +     1
         };
float b5[]={
          c2*c3+s1*s2*s3*(U5) +  -(c2*s3)+s1*s2*s3*(V5)  +  c1*s2*(W5) + px, 
          c1*s3*(U5)          +   c1*c3*(V5)             +  -s1*(W5)   + py,
          (-s2)*c3+s1*c2*s3*(U5)+   s2*s3+s1*c2*c3*(V5) +     c1*c2*(W5)    + pz,
          0  +                 0      +            0  +     1
         };
float b6[]={
          c2*c3+s1*s2*s3*(U6) +  -(c2*s3)+s1*s2*s3*(V6)  +  c1*s2*(W6) + px, 
          c1*s3*(U6)          +   c1*c3*(V6)             +  -s1*(W6)   + py,
          (-s2)*c3+s1*c2*s3*(U6)+   s2*s3+s1*c2*c3*(V6) +     c1*c2*(W6)    + pz,
          0  +                 0      +            0  +     1
         };      
         
float L1=sqrt(pow(abs(a1[0]-b1[0]),2)+pow(abs(a1[1]-b1[1]),2)+pow(abs(a1[2]-b1[2]),2));
float L2=sqrt(pow(abs(a2[0]-b2[0]),2)+pow(abs(a2[1]-b2[1]),2)+pow(abs(a2[2]-b2[2]),2));
float L3=sqrt(pow(abs(a3[0]-b3[0]),2)+pow(abs(a3[1]-b3[1]),2)+pow(abs(a3[2]-b3[2]),2));
float L4=sqrt(pow(abs(a4[0]-b4[0]),2)+pow(abs(a4[1]-b4[1]),2)+pow(abs(a4[2]-b4[2]),2));
float L5=sqrt(pow(abs(a5[0]-b5[0]),2)+pow(abs(a5[1]-b5[1]),2)+pow(abs(a5[2]-b5[2]),2));
float L6=sqrt(pow(abs(a6[0]-b6[0]),2)+pow(abs(a6[1]-b6[1]),2)+pow(abs(a6[2]-b6[2]),2));

          

static float L[]={L1,L2,L3,L4,L5,L6};
//cout<<L1<<" "<<L2<<" "<<" "<<L3<<" "<<L4<<" "<<" "<<L5<<" "<<L6<<endl;
return L;
}

int main(){
    float *fkd;
    fkd=inv(7,8,6,30,0,0);
    for(int i =0; i<6; i++){
        cout<<fkd[i]<<endl;
    }
}
