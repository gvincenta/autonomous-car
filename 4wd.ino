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
#include "SR04.h"
#define motorSpeed 170
#define OFFSET 20                  //The higher the offset, the faster the car will return to a straight line but will oscillate more
#define REFRESH 0.000540         //orginal equation is angle_yaw = gyro_z * 0.0000611, 
#define TRIG_PIN 12
#define ECHO_PIN 11                // V1.0 0.00003363 base model with 1 motor driver, 1 MPU-6050, 
#define StoppingDistance 28        // V1.1 0.00048043 with ultrasonic sensor
                                   // V1.2 0.000540 
using namespace std;

// sensor that reads both accelerometer and gyroscope -- MPU 6050. 
class AccelGyro{
    private:
        int gyro_x, gyro_y, gyro_z; // reads gyroscope in x,y,z directions.
        long gyro_x_cal, gyro_y_cal, gyro_z_cal; // calculation for gyroscope in x,y,z directions. 
        float angle_pitch, angle_roll, angle_yaw; // reads change in angle for gyroscope in x,y,z directions. 
        long loop_timer; // reads loop timer .
        long acc_x, acc_y, acc_z;  // reads accelerometer in x,y,z directions.
        int temperature; // this module reads temp. too. 
        
    public:
        static int refresh; 
        AccelGyro(int gx, int gy, int gz, long gxc, long gyc, long gzc, float ap, float ar, float ay, long lt, long acx, long acy, long acz, int temp ){
        this->gyro_x = gx; 
        this->gyro_y=gy;
        this->gyro_z = gz;
        this->gyro_x_cal = gxc; this->gyro_y_cal = gyc; this->gyro_z_cal = gzc;
        this->angle_pitch= ap; this->angle_roll=ar; this->angle_yaw=ay;
        this->loop_timer = lt;
        this->acc_x = acx; this->acc_y = acy; this->acc_z = acz;
        this->temperature = temp;
        setup_mpu_6050_registers();
        }

        void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
            Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
            Wire.write(0x3B);                                                    //Send the requested starting register
            Wire.endTransmission();                                              //End the transmission
            Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  
            while(Wire.available() < 14);                                        //Wait until all the bytes are received

            this->acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
            this->acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
            this->acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
            this->temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
            this->gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
            this->gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
            this->gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

            
        }

        static void setRefreshRate(){};
        void setup_mpu_6050_registers(){
          
            Wire.begin();                                                        //Start I2C as master
            Serial.begin(57600);                                               //Use only for debugging
            pinMode(13, OUTPUT);                                                 //Set output 13 (LED) as output
            
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

            calibrate();
       }

       void calibrate(){
                     
            Serial.print("  MPU-6050 IMU");                                         
            Serial.print("/n");
            Serial.print("     V1.0");                                              
            Serial.print("/n");
            
            delay(1500);                                                         //Delay 1.5 second to display the text
            
            Serial.print("Calibrating gyro");                                       
            for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  
              if(cal_int % 125 == 0)Serial.print(".");                              //Print a dot on the LCD every 125 readings
              read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
              this->gyro_z_cal += this->gyro_z;                                             
              delay(3);                                                         
            }
            
            this->gyro_z_cal = this->gyro_z_cal / 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
          
            Serial.print("\n");    
       }
       void updateGyroZ(){
           this->gyro_z -= gyro_z_cal;   
    
                              
           this->angle_yaw += gyro_z * REFRESH;
          
           Serial.print("yaw : ");
           Serial.print(angle_yaw);
           Serial.print("\n");
        
       }
       
       
}
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
float angle_pitch, angle_roll, angle_yaw;
long loop_timer;
long acc_x, acc_y, acc_z;
int temperature;
int tempSpeed;
long a;


//LiquidCrystal lcd(13,12,11,10,9,8); // pins for LCD Connection

//joystick ports and directions
const int X_pin = 0;
const int Y_pin = 1;
const int FWD = 0;
const int LEFT = 0;
const int RIGHT = 1023;
const int RVS = 1023;


// 2 DC motors consists of 3 pins each: forward, backward, and enable. 
class Wheel
{ 
    public:
        //
          static const int MAX_SPEED;
          static const int MIN_SPEED;
             
        //basic movements:

        //run at full speed. 
        void runFullSpeed(){
            digitalWrite(this->enable,HIGH); //enable on
            digitalWrite(this->forward,HIGH); 

        };

        //moves backwards. 
        void reverse(int analogSpeed){
             analogWrite(this->enable,analogSpeed); //enable on
            
            analogWrite(this->backward,analogSpeed); //half speed
        }

        //stops the wheel. 
        void stopping(){
            digitalWrite(this->enable,LOW); //enable off
            digitalWrite(this->backward,LOW); 
           
            digitalWrite(this->forward,LOW); 
        }

        void go(int analogSpeed){
          if (analogSpeed >=MIN_SPEED){
            analogWrite(this->enable,analogSpeed); //enable on
            
            analogWrite(this->forward,analogSpeed); //half speed
          }
          else if (analogSpeed >= MAX_SPEED){
            analogWrite(this->enable,MAX_SPEED); //enable on
            
            analogWrite(this->forward,MAX_SPEED); 
          }
          else if (analogSpeed < MIN_SPEED){
            analogWrite(this->enable,MIN_SPEED); //enable on
            
            analogWrite(this->forward,MIN_SPEED); //half speed
            }
        
        }
        
        Wheel(int a, int b, int enable){
          this->forward = a;
          this->backward = b;
          this->enable = enable; 
          pinMode(this->forward,OUTPUT);
          pinMode(this->backward,OUTPUT);
          pinMode(this->enable,OUTPUT);
          Serial.begin(9600); 
        }
        
        Wheel(){forward=0;backward=0;enable=0;}
   private:
       int forward;
       int backward;
       int enable; 
};

class Car
{
    public:
        static const int R; // right wheel index
        static const int L; // left wheel index
        static const int TURN_SPEED; // analog speed for turning left or right (analog).
        static const int TURN_DELAY; // delay time for turning (ms). 
        
        //basic movements:

        // car turns left then stops.
        void turnLeft(){
            wheels[L].reverse(TURN_SPEED);
            
            wheels[R].go(TURN_SPEED); 
            delay(TURN_DELAY);
            stopping();
            delay(1000);
        }

        // car turns right then stops. 
        void turnRight(){
            wheels[R].reverse(TURN_SPEED);  
            wheels[L].go(TURN_SPEED);  
            delay(TURN_DELAY);
            stopping(); 
            delay(1000);
        }

        //180 degree turn. 
        void UTurn(){
          }

        // car goes straight at given (analog) speed. 
        void goStraight(int analogSpeed){
            for(int i = 0; i < 2; i++){
                wheels[i].go(analogSpeed);
            }

            changeSpeed(analogSpeed,1);
            
        }

        // car reverses at given (analog) speed.
        void reverse(int analogSpeed){
            for(int i = 0; i < 2; i++){
                wheels[i].reverse(analogSpeed);
            }
            changeSpeed(analogSpeed,-1);
        }

        void changeSpeed(int speed, int op){
        
        }
        

        // car goes straight at full (digital) speed
        void goStraightFullSpeed(){
            for(int i = 0; i < 2; i++){
                wheels[i].runFullSpeed();
            }
        }

        // car comes to a stop. 
        void stopping(){
            // delay(...); // for a smoother stop ?
            for(int i = 0; i < 2; i++){
                wheels[i].stopping();
            }
        }

        // attaches car to the wheel. 
        Car(Wheel *wheels, Eye *eye, AccelGyro *gyroscope){
            this->wheels = wheels;
            this->gyroscope = gyroscope;
            this->eye =eye;
            this->leftCounter = 0;
            this->y = 0;
            this->x = 0;
            this->rightCounter = 0;

            this->forward = true;
            this->left = false; 

        } 

        

        

  private:
       Wheel *wheels;
       int leftCounter; //counts how many times it has turned left
       int rightCounter; // counts how many times it has turned right
       int x,y; // maintains Cartesian coordinate
       bool forward, left; // maintains car orientation: moving forward? facing left? 
       AccelGyro gyroscope;
       Eye eye;
       
       
       
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

Wheel  wheelL(4,3,1);//  left wheel
Wheel  wheelR(5,6,7); //  right wheel

Wheel wheels[2] = { wheelL, wheelR}; 
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);

const int Car::L =0;
const int Car::R =1;
const int Car::TURN_SPEED =140;
const int Car::TURN_DELAY =480;


const int Wheel::MAX_SPEED =255;
const int Wheel::MIN_SPEED =128;

//const  int TRIGGER_PIN = 12;  // Arduino pin tied to trigger pin on the ultrasonic sensor.
//const  int ECHO_PIN = 11;  // Arduino pin tied to echo pin on the ultrasonic sensor.
const  int MAX_DISTANCE = 200; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

Car car(wheels);

void setup() {
  
 
 
  
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro


 
  
  
}

void loop() {
 
  read_mpu_6050_data();
  updateGyroZ();
  a=sr04.Distance();

  

  if(a <= StoppingDistance){
    car.stopping();
    delay(1500);
    car.turnLeft();
    angle_yaw=0;
 
  
    /*
    
      car.left = true;

    
    if (sr04.Distance()  < StoppingDistance || sr04.Distance() > RANGE) { 
     car.UTurn(); 

   
     //check if car doomed or not. 
     if (sr04.Distance()  < StoppingDistance || sr04.Distance() > RANGE) { car.stopping(); exit(0);};
     
     car.right = true; 
     car.left = false;
    }

    car.checkOrientation();
    left = -1; right = 1;   
    if left -> right = go straight (+y); 0 // safe 
    if right -> left = go straight (+y); 0  // safe 
    
    if if left -> left / right -> right = go reverse (-y); // -2 or 2  
  
    if left -> left -> left / right -> right -> right = go back to origin ( +x / -x) // -3 or 3 x
    if left -> -x; // -1
    if right -> +x;  // 1
    
    

    
    */
  }
  
  int tmp = angle_yaw;
  if (tmp > 0){
     tmp = ((int)(angle_yaw) + OFFSET);
  }
  else if(tmp < 0) {
    tmp = ((int)(angle_yaw) - OFFSET);
  }

  
 
  wheelL.go(motorSpeed + tmp);
  wheelR.go(motorSpeed - tmp);

  // car.add(); 
  
}
 
