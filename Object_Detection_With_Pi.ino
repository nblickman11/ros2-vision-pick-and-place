#include <Servo.h>
  
// ---- Create servo objects ----
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoGripper;

// ---- Servo pin assignments ----
const int PIN_BASE = 3;
const int PIN_SHOULDER = 5;
const int PIN_ELBOW = 6;
const int PIN_GRIPPER = 9;

// VARIABLES
const int GRIPPER_BLOCK_WIDTH = 85;
const int GRIPPER_OPEN = 130;

void setup() {

  Serial.begin(115200);
  servoBase.attach(PIN_BASE);
  servoShoulder.attach(PIN_SHOULDER);
  servoElbow.attach(PIN_ELBOW);
  servoGripper.attach(PIN_GRIPPER);

  servoGripper.write(130);
  // servoBase.write(170); 
  // servoShoulder.write(170); 
  // servoElbow.write(100); 
  // servoGripper.write(130);


  // delay(1000);

  //Base
  // servoBase.write(0);
  // delay(1000);
  // // Shoulder
  // servoShoulder.write(0);
  // delay(1000);
  // //Elbow
  // servoElbow.write(0);
  // delay(1000);

  //   //Base
  // servoBase.write(30);
  // delay(1000);
  // // Shoulder
  // servoShoulder.write(120);
  // delay(1000);
  // //Elbow
  // servoElbow.write(90);
  // delay(1000);

  //   //Base
  // servoBase.write(80);
  // delay(1000);
  // // Shoulder
  // servoShoulder.write(170);
  // delay(1000);
  // //Elbow
  // servoElbow.write(180);
  // delay(1000);



  // //   // Elbow
  // servoElbow.write(90);
  // delay(1000);

  // Ease down the arm
  // servoShoulder.write(100);
  // delay(500);

  // delay(500);
  // servoBase.write(0);
  // delay(500);
  // servoShoulder.write(165);
  //   delay(500);
  // servoElbow.write(0);
  //   delay(500);

  // for (int base = 0; base <= 60; base += 10) {
  //   servoBase.write(base);
  //   delay(400);

    // for (int shoulder = 180; shoulder > 0; shoulder -= 10) {
    //   servoShoulder.write(shoulder);
    //   delay(400);  // Pause at each combination
    // }
    // for (int elbow = 20; elbow < 120; elbow += 10) {
    //   servoElbow.write(elbow);
    //   delay(400);  // Pause at each combination
    // }
  //}
  
}

void loop() {
  // Say this command sent from ROS, over the USB, to the Arduino:
  // Note Serial.available() doesn't mean full command arrived yet.
  // String cmd = "";
  // // if (Serial.available()) {
  // //   cmd = Serial.readStringUntil('\n');
  // // } 

  // String cmd = "";
  // while (Serial.available() > 0) {
  //   char c = Serial.read();
  //    if (c == '\n') 
  //    { 
  //         break;
  //     } 
  //     else {
  //        cmd += c; 
  //        } 
  //   }
  //   Serial.println("ACK: " + cmd); 
  //   if (cmd == "") {
  //     return;
  //   }

  // String cmd = "";
  // while (Serial.available()) {
  //       cmd = Serial.readStringUntil('\n'); // wait for full line, I think all chars should get
  //                                           // .. to hear before the timeout.
  //       Serial.println("ACK: " + cmd);           // now it prints properly
  //   }
  //   if (cmd == "") {
  //     return;
  //   }

servoGripper.write(130);

String cmd = "";
while (Serial.available()) {
    cmd = Serial.readStringUntil('\n'); // blocks for timeout if needed
    cmd.trim();                          // removes leading/trailing whitespace, including \r
    if (cmd.length() > 0) {
        Serial.println("ACK: " + cmd);  // only print if non-empty
    }
}

//String cmd = "";
// ERROR occuring, the Serial.available is becoming 0 temporarily and exiting loop
// since we receive something like MOV.. but rest of characters hasn't gotten to arduino yet.
// SO, we need bigger delay.
  // while (Serial.available() > 0) {
  //     char c = Serial.read();


  //     if (c == '\n') {
  //         Serial.println("ACK: " + cmd);
  //         break;
  //     } else {
  //         cmd += c;
  //     }
  // }
  // if (cmd == ""){
  //   return;
  // }
// int spaceCount = 0;
//   for (int i = 0; i < cmd.length(); i++) {
//     if (cmd.charAt(i) == ' ') {
//       spaceCount++;
//     }
//   }
//   if (spaceCount < 3) {
//   return;
// }

  //cmd = "MOVE 10 90 0";
  
  int start = 0;
  int spaceIndex = cmd.indexOf(' ');
  int counter = 1;
  while (counter <=4 and cmd.length() > 0) {
    String token = cmd.substring(start, spaceIndex);
    //Serial.println("\nACK Processing Command: " + cmd);
    //Serial.println("\nACK Token Value: " + cmd);
    int token_int = token.toInt();
    if (counter == 1) {
      // Just skip, increment counter below
    }
    else if (counter == 2) {
      //Serial.println("Token int base is "); 
      //Serial.println(token_int); 
      servoBase.write(token_int); 
      delay(1000);
    }
    else if (counter == 3) {
      //Serial.println("Token int shoulder is "); 
      //Serial.println(token_int); 
      servoShoulder.write(token_int); 
      delay(1000);
    }
    else if (counter == 4) {
      servoElbow.write(token_int); 
      delay(1000);
    }
    counter += 1;
    start = spaceIndex + 1;
    spaceIndex = cmd.indexOf(' ', start);
  }

  delay(200);
  servoGripper.write(85);
  delay(1000);
  servoElbow.write(100); 
  delay(1000);
  servoShoulder.write(170); 
  delay(1000);
  servoBase.write(170); 
  delay(2000);
  servoGripper.write(130);
  delay(2000);

}

