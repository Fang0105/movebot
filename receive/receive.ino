#include <SoftwareSerial.h>
#include <Servo.h>
// SoftwareSerial mySerial(10, 11);  // RX, TX (not needed for actual Nano, but useful for debugging)
Servo myservo[6];

int count;
int angles[5];
bool end;


void turn_around(int start, int ending, int angle,int time_delay){
    if(angle<0){
      for(int i=start;i>=ending;i+=angle){
          myservo[5].write(i);
          delay(time_delay);
      }
    }
    else{
      for(int i=start;i<=ending;i+=angle){
          myservo[5].write(i);
          delay(time_delay);
      }
    }
  }
void wave(){
    turn_around(90,120,15,100);
    for(int i=0;i<5;i++){
      turn_around(120,60,-15,100);
      turn_around(60,120,15,100);
    }
    turn_around(120,90,-15,100);
}




void setup() {
    Serial.begin(9600);  // Main UART (connected to Raspberry Pi)

    count = 0;
    // mySerial.begin(9600);  // Debugging with SoftwareSerial
    myservo[0].attach(2);  // attaches the servo on pin 9 to the servo object
    myservo[1].attach(3);   // 90
    myservo[2].attach(4);
    myservo[3].attach(5);
    myservo[4].attach(6);
    myservo[5].attach(7);   // 90


    myservo[0].write(90);
    myservo[1].write(90);
    myservo[2].write(90);
    myservo[3].write(90);
    myservo[4].write(90);
    myservo[5].write(90);

    // myservo[0].write(0);
    // myservo[1].write(0);
    // myservo[2].write(0);
    // myservo[3].write(0);
    // myservo[4].write(0);
    // myservo[5].write(0);

    end = false;
}

void arm_move(){
  myservo[0].write(90);
  for(int i=0;i<5;i++){
    myservo[5-i].write(constrain(angles[i], 0, 180));
  }
}

void loop() {
    if(end == false){
      if ( Serial.available() > 0) {
        String receivedNumber = Serial.readStringUntil('\n');  // Read incoming data
        receivedNumber.trim();  // Remove extra spaces or newlines
        int number = receivedNumber.toInt();  // Convert to integer

        if(number == 300){
          end = true;
          // wave();
        }

        angles[count++] = number;
        if(count == 5){
          count = 0;
          arm_move();
        }
      }
    }
    else if(end == true){
        for(int i=0;i<=180;i++){
          myservo[0].write(i);
          delay(100);
        }
        for(int i=180;i>=0;i--){
          myservo[0].write(i);
          delay(100);
        }
    }
}