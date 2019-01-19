
// avoid obstacles
// night lights
// moves 100m and terminate
// move faster when distance too far.
// smooth stop
// avoid falling
//stop every 5 min, take pic.
// bound is 3 to 300 cm for ultrasonic sensor.
// asynchronous approach - Interrupt service routine used.
#include <NewPing.h> // for ultrasonic sensor
#include <UTFT.h > // for 2.2 inch display 
using namespace std;

//joystick ports and directions
const int X_pin = 0;
const int Y_pin = 1;
const int FWD = 0;
const int LEFT = 0;
const int RIGHT = 1023;
const int RVS = 1023;


// LCD -
// IMU -

// 2 DC motors
class Wheel
{

  public:
    //ports



    //basic movements
    void runFullSpeed() {
      digitalWrite(this->enable, HIGH); //enable on
      digitalWrite(this->portA, HIGH); //one way

    };

    void reverse() {
      digitalWrite(this->enable, HIGH); //enable on
      digitalWrite(this->portB, HIGH); //one way
    }

    void stopping() {
      digitalWrite(this->enable, LOW); //enable off
      digitalWrite(this->portB, LOW); //one way

      digitalWrite(this->portA, LOW);
    }

    void go(int analogSpeed = 10) {
      analogWrite(this->enable, analogSpeed); //enable on

      analogWrite(this->portA, analogSpeed); //half speed


    }
    void checkStatus();
    void copy(Wheel src);
    Wheel(int a, int b, int enable) {
      this->portA = a;
      this->portB = b;
      this->enable = enable;
      pinMode(this->portA, OUTPUT);
      pinMode(this->portB, OUTPUT);
      pinMode(this->enable, OUTPUT);
      Serial.begin(9600);

    }
    Wheel() {
      portA = 0;
      portB = 0;
      enable = 0;
    }
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
    void turnLeft() {
      wheels[L].reverse();

      wheels[R].runFullSpeed();
    }
    void turnRight() {
      wheels[R].reverse();
      wheels[L].runFullSpeed();
    }

    void goStraight(int analogSpeed) {
      for (int i = 0; i < 2; i++) {
        wheels[i].go(analogSpeed);
      }
    }
    void reverse() {
      for (int i = 0; i < 2; i++) {
        wheels[i].reverse();
      }
    }
    void goStraightFullSpeed() {
      for (int i = 0; i < 2; i++) {
        wheels[i].runFullSpeed();

      }
    }





    void stopping() {
      for (int i = 0; i < 2; i++) {
        wheels[i].stopping();
      }
    }

    Car(Wheel *wheels) {
      this->wheels = wheels;
    }


  private:
    Wheel *wheels;

};


class Eye {
  public:

    static const  int TRIGGER_PIN;  // Arduino pin tied to trigger pin on the ultrasonic sensor.
    static const  int ECHO_PIN;  // Arduino pin tied to echo pin on the ultrasonic sensor.
    static const  int MAX_DISTANCE; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
    Eye(int t = 12, int e = 11, int m = 200): sonar(t, e, m) {} ;
  private:
    NewPing sonar; // NewPing setup of pins and maximum distance.
};


Wheel  wheelL(4, 3, 1); // bottom left
Wheel  wheelR(5, 6, 7); // bottom right

Wheel wheels[2] = { wheelL, wheelR};

const int Car::L = 0;
const int Car::R = 1;

//const  int TRIGGER_PIN = ;  // Arduino pin tied to trigger pin on the ultrasonic sensor.
//const  int ECHO_PIN = ;  // Arduino pin tied to echo pin on the ultrasonic sensor.
const  int MAX_DISTANCE = 200; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

Car car(wheels);

void setup() {




  Serial.begin(9600);
  delay(5000);
  /*
    int distance = 0;
    int light = 1;
    pinMode(light, OUTPUT);
    pinMode(ultra, INPUT);
  */

}

void loop() {
 delay(5000);
 car.turnLeft();
 delay(300);
 car.goStraightFullSpeed();
 delay(20000);
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
