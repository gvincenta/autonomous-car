// avoid obstacles
// night lights
// moves 100m and terminate
// move faster when distance too far.
// smooth stop
// avoid falling
//stop every 5 min, take pic.
// bound is 3 to 300 cm for ultrasonic sensor.
// asynchronous approach - Interrupt service routine used. 
//#include <NewPing.h> // for ultrasonic sensor
#include <LiquidCrystal.h>
#include <Wire.h>
#define motorSpeedR 160
#define motorSpeedL 175
#define refresh

using namespace std;
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
float angle_pitch, angle_roll, angle_yaw;
long loop_timer;
long acc_x, acc_y, acc_z;
int temperature;
int tempSpeed;


//LiquidCrystal lcd(13,12,11,10,9,8); // pins for LCD Connection

//joystick ports and directions
const int X_pin = 0;
const int Y_pin = 1;
const int FWD = 0;
const int LEFT = 0;
const int RIGHT = 1023;
const int RVS = 1023;


// 2 DC motors
class Wheel
{
  
    public:
        //ports 

        
        
        //basic movements
        void runFullSpeed(){
            digitalWrite(this->enable,HIGH); //enable on
            digitalWrite(this->portA,HIGH); //one way

        };

        void reverse(){
            digitalWrite(this->enable,HIGH); //enable on
            digitalWrite(this->portB,HIGH); //one way
        }
         
        void stopping(){
            digitalWrite(this->enable,LOW); //enable off
            digitalWrite(this->portB,LOW); //one way
           
            digitalWrite(this->portA,LOW); 
        }

        void go(int analogSpeed=10){
          if (analogSpeed >=140){
            analogWrite(this->enable,analogSpeed); //enable on
            
            analogWrite(this->portA,analogSpeed); //half speed
          }
          else{
            analogWrite(this->enable,140); //enable on
            
            analogWrite(this->portA,140); //half speed
          }
  
        
        }
        void checkStatus();
        void copy(Wheel src);
        Wheel(int a, int b, int enable){
          this->portA = a;
          this->portB = b;
          this->enable = enable; 
          pinMode(this->portA,OUTPUT);
          pinMode(this->portB,OUTPUT);
          pinMode(this->enable,OUTPUT);
          Serial.begin(9600);
          
        }
        Wheel(){portA=0;portB=0;enable=0;}
   private:
       int analogSpeed = 0;
       bool digitalSpeed;
       int portA;
       int portB;
       int enable; 
};

class Car
{
    public:
        static const int R; // top right
        static const int L; // top left


        //basic movements
        void turnLeft(){
            wheels[L].reverse();
            
            wheels[R].runFullSpeed(); 
        }
        void turnRight(){
            wheels[R].reverse();  
            wheels[L].runFullSpeed();   
        }
      
        void goStraight(int analogSpeed){
            for(int i = 0; i < 2; i++){
                wheels[i].go(analogSpeed);
            }
        }
        void reverse(){
            for(int i = 0; i < 2; i++){
                wheels[i].reverse();
            }
        }
        void goStraightFullSpeed(){
            for(int i = 0; i < 2; i++){
                wheels[i].runFullSpeed();
                
            }
        }

        

        
        
        void stopping(){
            for(int i = 0; i < 2; i++){
                wheels[i].stopping();
            }
        }

        Car(Wheel *wheels){
            this->wheels = wheels;
        }
        

  private:
       Wheel *wheels;
       
};


/*class Eye{
    public:
        
        static const  int TRIGGER_PIN;  // Arduino pin tied to trigger pin on the ultrasonic sensor.
        static const  int ECHO_PIN;  // Arduino pin tied to echo pin on the ultrasonic sensor.
        static const  int MAX_DISTANCE; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
        Eye(int t=12, int e=11, int m=200): sonar(t,e,m){} ;
    private:
         NewPing sonar; // NewPing setup of pins and maximum distance.
};
*/

Wheel  wheelL(4,3,1);// bottom left
Wheel  wheelR(5,6,7); // bottom right

Wheel wheels[2] = { wheelL, wheelR};

const int Car::L =0;
const int Car::R =1;

//const  int TRIGGER_PIN = ;  // Arduino pin tied to trigger pin on the ultrasonic sensor.
//const  int ECHO_PIN = ;  // Arduino pin tied to echo pin on the ultrasonic sensor.
const  int MAX_DISTANCE = 200; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

Car car(wheels);

void setup() {
  
 
  Wire.begin();                                                        //Start I2C as master
  Serial.begin(57600);                                               //Use only for debugging
  pinMode(13, OUTPUT);                                                 //Set output 13 (LED) as output
  
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro


 
  Serial.print("  MPU-6050 IMU");                                         //Print text to screen
  Serial.print("/n");
  Serial.print("     V1.0");                                              //Print text to screen
  Serial.print("/n");
  delay(1500);                                                         //Delay 1.5 second to display the text
  
  Serial.print("Calibrating gyro");                                       //Print text to screen
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    if(cal_int % 125 == 0)Serial.print(".");                              //Print a dot on the LCD every 125 readings
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  //gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  //gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal = gyro_z_cal / 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset

  Serial.print("/n");    
  
  loop_timer = micros();  
   
  /*
  int distance = 0;
  int light = 1;
  pinMode(light, OUTPUT);
  pinMode(ultra, INPUT);
  */
  
}

void loop() {
  read_mpu_6050_data();
  gyro_z -= gyro_z_cal;   

                      //Calculate the traveled roll angle and add this to the angle_roll variable
  angle_yaw += gyro_z * 0.00003363;
  
  Serial.print("yaw : ");
  Serial.print(angle_yaw);
  Serial.print("\n");
  
  
  wheelL.go(motorSpeedL + (int)(angle_yaw));
  wheelR.go(motorSpeedR - (int)(angle_yaw));
  /*if (angle_yaw >= 0){
   
    tempSpeed = outputSpeed(angle_yaw);
    wheelL.go(tempSpeed + motorSpeed);
    wheelR.go(motorSpeed);
  }
  else if (angle_yaw < 0){
    Serial.print("+ Right Motor speed");
    Serial.print("/n");
//    tempSpeed = outputSpeed(angle_yaw * -1);
    wheelR.go(tempSpeed + motorSpeed);
    
    wheelL.go(motorSpeed);
  }
   */
}
  
  

  
  
 
  /*
  delay(delaytime);
  distance += speed;
  if (distance == 100) {
    end;
  }
  if (light sensor == HIGH) {
    digitalWrite(light, HIGH);
  }

  if (ultrareadings <= 10 ) {
    turn the car;
  }
  */

  void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

}

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}


/*int outputSpeed(float angle){
  if (angle < 10){
    return 80;
  }
  if (angle < 20){
    return 80;
  }
}*/
