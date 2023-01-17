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
int DXL_ID_dev = 8;
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
  //while(!DEBUG_SERIAL);

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
    dxl.torqueOff(DXL_ID_dev);
    dxl.setOperatingMode(DXL_ID_dev, OP_VELOCITY);     //This function changes operating mode of DYNAMIXEL.
    dxl.torqueOn(DXL_ID_dev);
}
  


void loop(){ 
dxl.setGoalVelocity(DXL_ID_dev, 10 , UNIT_RPM);
mov(V_max, goal_pos_0);
delay(delayTime);
mov(V_max, goal_pos_1);
delay(delayTime);
//mov(V_max, goal_pos_2);
//delay(delayTime);
mov(V_max, goal_pos_3);
delay(delayTime);
mov(V_max, goal_pos_4);
delay(delayTime);

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

 
