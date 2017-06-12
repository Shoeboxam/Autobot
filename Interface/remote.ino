#define pinUD 4
#define pinLR 5 
#define pinButton 2
#define pinBTRX 10
#define pinBTTX 11

#define BLUETOOTH_SPEED 57600
#include <SoftwareSerial.h>
SoftwareSerial serialBT(pinBTTX, pinBTRX);

uint8_t stateUD = 0; 
uint8_t stateLR = 0;
int stateButton = 0;
String command = "";

void setup() {
  // Joystick
  pinMode(pinButton,INPUT);
  
  Serial.begin(9600);
  serialBT.begin(BLUETOOTH_SPEED);
  delay(1000);

  // Should respond with OK
  serialBT.print("AT");
  waitForResponse();

  // Set pin to 5438
  serialBT.print("AT+PIN5438");
  waitForResponse();

  // Set the name to 'Remote'
  serialBT.print("AT+NAME");
  serialBT.print("Remote");
  waitForResponse();

  // Set baudrate to 57600
  serialBT.print("AT+BAUD7");
  waitForResponse();
}

void waitForResponse() {
    delay(1000);
    while (serialBT.available()) {
      Serial.write(serialBT.read());
    }
    Serial.write("\n");
}

void loop() {
  // Process local sensor inputs
  stateUD = analogRead(pinUD) / 4; 
  stateLR = analogRead(pinLR) / 4;
  Serial.print(stateUD);
  Serial.print(' ');
  Serial.println(stateLR);

//  // Receive Bluetooth data
//  if (serialBT.available()) {
//    while(serialBT.available()) {
//      command += (char)serialBT.read();
//    }
//    Serial.println(command);
//    command = "";
//  }
  
  // Send Bluetooth data
  serialBT.write(stateUD);
  serialBT.write(stateLR);
  
  /* Receiving code:
  byte UD1 = Serial.read();
  byte UD2 = Serial.read();
  byte LR1 = Serial.read();
  byte LR2 = Serial.read();
  
  int stateUD = UD1 * 256 + UD2;
  int stateLR = LR1 * 256 + LR2;
  */
}
