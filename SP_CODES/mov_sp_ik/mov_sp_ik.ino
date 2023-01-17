 /*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

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
int delayTime = 2*(6000/180)* 180;
int V_max=10;
int warning_pin_1 = 18; // red led
int warning_pin_2 = 19; // green led
int warning_pin_3 = 20; // blue led
int button_pin_1 = 16;
float goal_pos_0[6]={120,240,120,240,120,240};
float goal_pos_1[6]={60,300,60,300,60,300};
float goal_pos_2[6]={180,180,180,180,180,180};
float goal_pos_3[6]={10,310,10,300,60,300};
//float goal_pos_4[6]={90,270,90,270,90,270};
float goal_pos_4[6]={90,270,60,230,130,300};
int n = 6;
float L[6];
float joints[6];
   // 2.2 protocal..............

const float DXL_PROTOCOL_VERSION = 2.0;

   // 2.3 actuator variables....



Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;



void setup() {
  // put your setup code here, to run once:
 pinMode(warning_pin_1,OUTPUT); // 18 to 20 led in epansion board, 
 pinMode(warning_pin_2,OUTPUT); // 18 to 20 led in epansion board, 
 pinMode(warning_pin_3,OUTPUT); // 18 to 20 led in epansion board, 
 pinMode(button_pin_1,OUTPUT); // 16 to 17 button pin in epansion board, 


 
  // Use UART port of DYNAMIXEL Shield to debug.
//  Serial.begin(115200);
  DEBUG_SERIAL.begin(115200);
     Serial.print("DYNAMIXEL SERIAL IS: ");
  Serial.println(DXL_SERIAL);
     Serial.print("DYNAMIXEL DIR PIN IS: ");
   Serial.println(DXL_DIR_PIN);

        Serial.print("DYNAMIXEL PROTOCOL_VERSION IS: ");
   Serial.println(DXL_PROTOCOL_VERSION);
  while(!DEBUG_SERIAL);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  Serial.println("Conection Checking ('1' for conected and '0' for not conected) for the actuators 1 to 6: ");
  
  for(int i=0; i<6; i++){
    Serial.print("Actuator 1 is: ");
     Serial.println(dxl.ping(DXL_ID[0]));
  }

   Serial.println("The model numbers for the actuators 1 to 6: ");
      Serial.println("                                                      1020 for [DYNAMIXEL XM540-W270-R] ");
            Serial.println("                                                      1030 for [DYNAMIXEL XM540-W150-R] ");
   for(int i=1; i<=6; i++){
    Serial.print("Model numbers for the Actuator 1 is: ");
     Serial.println(dxl.getModelNumber(i));
   }




  Serial.println("");
  Serial.println("");
  Serial.println("==========================================================================================");
    Serial.print("The maximum velocity in Position Control Mode: =  ");
      Serial.print(V_max);
      Serial.println(" deg/sec");


  for(int i=0; i<n; i++){
  // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_POSITION);     //This function changes operating mode of DYNAMIXEL.
    dxl.torqueOn(DXL_ID[i]);
 
    // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[i], V_max);
  }
}
  


void loop(){ 
//mov(V_max, goal_pos_0);
//delay(delayTime);
//mov(V_max, goal_pos_1);
//delay(delayTime);
//mov(V_max, goal_pos_2);
//delay(delayTime);
//mov(V_max, goal_pos_3);
//delay(delayTime);
//mov(V_max, goal_pos_4);
//delay(delayTime);
//inv(2,3,4,40,25,30,L);
inv(0,0,0,0,0,0,L);
print_L(L);
crank_ang(L[0],L[1],L[2],L[3],L[4],L[5],joints);
print_joints(joints);
}

void print_L(float* L){
Serial.print(L[0]);
Serial.print(" ");
Serial.print(L[1]);
Serial.print(" ");
Serial.print(L[2]);
Serial.print(" ");
Serial.print(L[3]);
Serial.print(" ");
Serial.print(L[4]);
Serial.print(" ");
Serial.print(L[5]);
Serial.println(" ");

}


void print_joints(float* joints){
Serial.print(joints[0]);
Serial.print(" ");
Serial.print(joints[1]);
Serial.print(" ");
Serial.print(joints[2]);
Serial.print(" ");
Serial.print(joints[3]);
Serial.print(" ");
Serial.print(joints[4]);
Serial.print(" ");
Serial.print(joints[5]);
Serial.println(" ");

}
void crank_ang(float L1,float L2,float L3,float L4,float L5,float L6, float* joints){

  float rod=28.0;
  float crank= 4.5  ;

  float joints1 = acos(((crank*crank)+(L1*L1)-(rod*rod))/(2*crank*L1));
  float joints2 = acos(((crank*crank)+(L2*L2)-(rod*rod))/(2*crank*L2));
  float joints3 = acos(((crank*crank)+(L3*L3)-(rod*rod))/(2*crank*L3));
  float joints4 = acos(((crank*crank)+(L4*L4)-(rod*rod))/(2*crank*L4));
  float joints5 = acos(((crank*crank)+(L5*L5)-(rod*rod))/(2*crank*L5));
  float joints6 = acos(((crank*crank)+(L6*L6)-(rod*rod))/(2*crank*L6));
  
joints[0] = joints1*(180/3.14);
joints[1] = joints2*(180/3.14);
joints[2] = joints3*(180/3.14);
joints[3] = joints4*(180/3.14);
joints[4] = joints5*(180/3.14);
joints[5] = joints6*(180/3.14);  
}



void mov(int V_max, float goal_pos[]){
   
  int flag = 0;
  for(int i =0; i<n; i++){
    if((i+1)%2!=0){
      if(goal_pos[i]>=60 and goal_pos[i]<=180){
        flag++; //flag++ --> flag=flag+1
        }
      else{  
      Serial.print(goal_pos[i]); 
      Serial.print(" degrees rotation of the actuator "); 
      Serial.print(i+1); 
      Serial.println(" is out of range [60,180] ");
      }
//      cout<<goal_pos[i]<<' of actuator' i+1 ' is out of range [60,180]'<<endl;
    }
    else if((i+1)%2==0){
            if(goal_pos[i]>=180 and goal_pos[i]<=300){
            flag++;
            }
            else{
             Serial.print(goal_pos[i]);
             Serial.print(" degrees rotation of the actuator "); 
             Serial.print(i+1); 
             Serial.println(" is out of range [180,300]");
            }
//        cout<<goal_pos[i]<<" of actuator"<<i+1<< " is out of range [180,300]"<<endl;
    }
  }
 Serial.println(flag);
  if(flag==6){
     Serial.println("devin");
    for(int i=0; i<n; i++){
      dxl.setGoalPosition(DXL_ID[i], goal_pos[i], UNIT_DEGREE);
      }
    digitalWrite(warning_pin_1,HIGH);
    digitalWrite(warning_pin_3,HIGH);
    digitalWrite(warning_pin_2,LOW);
    delay(50);
      Serial.println("devout");
    }
    else{
    Serial.println("devout without in");
    digitalWrite(warning_pin_2,HIGH);
    digitalWrite(warning_pin_3,HIGH);
    for(int j=0; j<100; j++){
    digitalWrite(warning_pin_1,LOW);
    delay(50);
    digitalWrite(warning_pin_1,HIGH);
    delay(50);
    }
    }
  }
void inv(float px,float py,float pz,float alpha,float bita,float gama, float* L){
//float a=12;
//float b=17;
//float d=4;
//float t=2.5;
float height = 26;
float a=8.75;
float b=12.0;
float d=10.95;
float t=5.5;
pz = pz+height;
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
          (c2*c3+s1*s2*s3)*(U1) +  (-(c2*s3)+s1*s2*s3)*(V1)  +  c1*s2*(W1) + px, 
          c1*s3*(U1)          +   c1*c3*(V1)             +  -s1*(W1)   + py,
          ((-s2)*c3+s1*c2*s3)*(U1)+   (s2*s3+s1*c2*c3)*(V1) +     c1*c2*(W1)    + pz,
          0  +                 0      +            0  +     1
         };
      
float b2[]={
          (c2*c3+s1*s2*s3)*(U2) +  (-(c2*s3)+s1*s2*s3)*(V2)  +  c1*s2*(W2) + px, 
          c1*s3*(U2)          +   c1*c3*(V2)             +  -s1*(W2)   + py,
          ((-s2)*c3+s1*c2*s3)*(U2)+   (s2*s3+s1*c2*c3)*(V2) +     c1*c2*(W2)    + pz,
          0  +                 0      +            0  +     1
         };
        

float b3[]={
          (c2*c3+s1*s2*s3)*(U3) +  (-(c2*s3)+s1*s2*s3)*(V3)  +  c1*s2*(W3) + px, 
          c1*s3*(U3)          +   c1*c3*(V3)             +  -s1*(W3)   + py,
          ((-s2)*c3+s1*c2*s3)*(U3)+   (s2*s3+s1*c2*c3)*(V3) +     c1*c2*(W3)    + pz,
          0  +                 0      +            0  +     1
         };
         

float b4[]={
          (c2*c3+s1*s2*s3)*(U4) +  (-(c2*s3)+s1*s2*s3)*(V4)  +  c1*s2*(W4) + px, 
          c1*s3*(U4)          +   c1*c3*(V4)             +  -s1*(W4)   + py,
          ((-s2)*c3+s1*c2*s3)*(U4)+   (s2*s3+s1*c2*c3)*(V4) +     c1*c2*(W4)    + pz,
          0  +                 0      +            0  +     1
         };
        

float b5[]={
          (c2*c3+s1*s2*s3)*(U5) +  (-(c2*s3)+s1*s2*s3)*(V5)  +  c1*s2*(W5) + px, 
          c1*s3*(U5)          +   c1*c3*(V5)             +  -s1*(W5)   + py,
          ((-s2)*c3+s1*c2*s3)*(U5)+   (s2*s3+s1*c2*c3)*(V5) +     c1*c2*(W5)    + pz,
          0  +                 0      +            0  +     1
         };
         

float b6[]={
          (c2*c3+s1*s2*s3)*(U6) +  (-(c2*s3)+s1*s2*s3)*(V6)  +  c1*s2*(W6) + px, 
          c1*s3*(U6)          +   c1*c3*(V6)             +  -s1*(W6)   + py,
          ((-s2)*c3+s1*c2*s3)*(U6)+   (s2*s3+s1*c2*c3)*(V6) +     c1*c2*(W6)    + pz,
          0  +                 0      +            0  +     1
         };   
         

         
float L1=sqrt(pow(abs(a1[0]-b1[0]),2)+pow(abs(a1[1]-b1[1]),2)+pow(abs(a1[2]-b1[2]),2));
float L2=sqrt(pow(abs(a2[0]-b2[0]),2)+pow(abs(a2[1]-b2[1]),2)+pow(abs(a2[2]-b2[2]),2));
float L3=sqrt(pow(abs(a3[0]-b3[0]),2)+pow(abs(a3[1]-b3[1]),2)+pow(abs(a3[2]-b3[2]),2));
float L4=sqrt(pow(abs(a4[0]-b4[0]),2)+pow(abs(a4[1]-b4[1]),2)+pow(abs(a4[2]-b4[2]),2));
float L5=sqrt(pow(abs(a5[0]-b5[0]),2)+pow(abs(a5[1]-b5[1]),2)+pow(abs(a5[2]-b5[2]),2));
float L6=sqrt(pow(abs(a6[0]-b6[0]),2)+pow(abs(a6[1]-b6[1]),2)+pow(abs(a6[2]-b6[2]),2));

L[0] = L1;
L[1] = L2;
L[2] = L3;
L[3] = L4;
L[4] = L5;
L[5] = L6;
//cout<<L1<<" "<<L2<<" "<<" "<<L3<<" "<<L4<<" "<<" "<<L5<<" "<<L6<<endl;
//Serial.println(L[0]);
//Serial.println(L[1]);
//Serial.println(L[2]);
//Serial.println(L[3]);
//Serial.println(L[4]);
//Serial.println(L[5]);
//Serial.println(L[6]);

}

//    fkd=inv(2,3,4,40,25,30);




 
