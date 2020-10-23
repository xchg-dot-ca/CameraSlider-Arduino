
#include <AccelStepper.h>
#include <bluefruit.h>
#include <Adafruit_DotStar.h>
#include "cameraslider.h"

// Default pins
#define DIR 7             // Stepper driver direction signal
#define STEP 9            // Stepper driver step signal
#define ENABLE_PIN 10          // Stepper driver Enable
#define LIMIT_PIN 0           // Home limit switch
#define ACTION 1          // External action trigger

// Type, step, dir
AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);

//Use these pin definitions for the ItsyBitsy M4
#define DATAPIN    8
#define CLOCKPIN   6
#define NUMPIXELS 1

Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);

// OTA DFU service
BLEDfu bledfu;
// Uart over BLE service
BLEUart bleuart;

#define BLE_READPACKET_TIMEOUT 2

// Packet buffer
extern uint8_t packetbuffer[];

// States
bool gooo = false;
bool runHome = false;
bool runStartPosition = false;

// Indicates that slider was recently homed
bool homed = false;
float timetoRun = DEFAULT_RUN_TIME * 60; // in seconds
float distancetoRun = DEFAULT_LENGTH; // in mm
// take in account 16t pulley
double stepsPerMm = DEFAULT_STEPS_PER_MM; 
double totalSteps = stepsPerMm * distancetoRun;
// in steps per second
float servoSpeed = totalSteps / timetoRun;
int startPosition, stopPosition;

void setup()
{

  Scheduler.startLoop(loop2);
  
  // Setup BLE
  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Camera Slider");
  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();
  // Configure and start the BLE Uart service
  bleuart.begin();
  // Set up and start advertising
  startAdv();  

  Serial.begin(115200);
  Serial.println("Controller demo ready!");
  Serial.print("Servo data. ");
  Serial.print("stepsPerMm: ");
  Serial.print(stepsPerMm, 2);
  Serial.print(" totalSteps: ");
  Serial.print(totalSteps);
  Serial.print(" servoSpeed: ");
  Serial.println(servoSpeed);

  // Setup home limit switch
  pinMode(LIMIT_PIN, INPUT_PULLUP);
  digitalWrite(LIMIT_PIN, HIGH); // Pullup

 pinMode(ENABLE_PIN, OUTPUT);
 digitalWrite(ENABLE_PIN, LOW); // LOW to Enable
 
 // Setting DIR pin as inverted to invert direction
 stepper.setPinsInverted(true, false, false);
 //stepper.setMinPulseWidth(3);
 stepper.setMaxSpeed(2200);

  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP
  strip.setPixelColor(0, 255, 0, 0); strip.show(); // RED
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  
  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */ 
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void loop()
{    
  // Get new data if there is something
  if(bleuart.available()) {
    readPacketNew(&bleuart, BLE_READPACKET_TIMEOUT);

    char command = packetbuffer[0];
    Serial.print ("Command "); Serial.println(command, HEX);

    // Process commands

    // Process HOME command
    if (command == HOME_COMMAND) {      
      goHome();
    }

    // Process START command
    if (command == START_COMMAND) {      
         
         if(!homed) {
            char message[] = "Home first!";
            bleuart.write(message, strlen(message));
            return;
         }
         
         strip.setPixelColor(0, 0, 0, 255); strip.show(); // Blue

         startPacketPayload *payload = new startPacketPayload();
         if(parseStartPayload(payload)) {
          calculateStepper(payload);
          
          if(startPosition != 0) {
            gotoStartPosition();
           } else {
            gotoTimeLapse();
           }
         } else {
            Serial.print ("Failed to parse Start payload");
         }
    }

    // Process STOP command
    if (command == STOP_COMMAND) {      
         digitalWrite(ENABLE_PIN, HIGH); // HIGH to !Enable
         Serial.print("Stopping.");
         gooo = false;
         runHome = false;
         homed = false;
         runStartPosition = false;
         stepper.stop();
    }
  }

  // Process only if going home
  if(runHome) {    
    // Read if HOME is reached
    // home = false - if button pressed
    bool home = digitalRead(LIMIT_PIN);
  
    if(!home) {
      Serial.println("Home Reached");
      runHome = false;

      strip.setPixelColor(0, 0, 255, 0); strip.show(); // GREEN
      digitalWrite(ENABLE_PIN, HIGH); // HIGH to !Enable

      char message[] = "Home Reached";
      Serial.print("Writing:");
      Serial.print(message);
      Serial.print(" bytes ");
      Serial.println(strlen(message));
      bleuart.write(message, strlen(message));
      homed = true;
      stepper.setCurrentPosition(0);
    }
  
    if(runHome) {
      Serial.println("Going home");
      stepper.runSpeed();
    }
  }
  
  if(gooo) {
    //Serial.println("Going");
    if (stepper.distanceToGo() == 0) {
      gooo = false;
      Serial.println("Arrived to the desination");
      char message[] = "Finished";
      bleuart.write(message, strlen(message));
      digitalWrite(ENABLE_PIN, HIGH); // Disabling motor driver
      strip.setPixelColor(0, 0, 255, 0); strip.show(); // GREEN
      return;
    }
  }
}

// All stepper code and logic goes here
void loop2() {
  if(gooo) {
    stepper.runSpeed();
  }

  if(runStartPosition) {
    if (stepper.distanceToGo() != 0) {
      stepper.runSpeed();
    } else {
      runStartPosition = false;
      stepper.setCurrentPosition(0);
      gotoTimeLapse();
    }
  }
}

void calculateStepper(startPacketPayload *payload) {
  if(payload != NULL) {
    timetoRun = payload->timeToRun * 60; // in seconds
    // TODO: Calculate stop - start
    stopPosition = payload->stopPosition;
    startPosition = payload->startPosition;
    distancetoRun = stopPosition - startPosition; // in mm
    // take in account 16t pulley
    stepsPerMm = DEFAULT_STEPS_PER_MM; 
    totalSteps = stepsPerMm * distancetoRun;
    // in steps per second
    servoSpeed = totalSteps / timetoRun;
  }
}

/**
 Command servo to go home
 */
void goHome() {
  digitalWrite(ENABLE_PIN, LOW); // LOW to Enable
  Serial.println("Go Home");
  runHome = true;
  gooo = false;
  stepper.setSpeed(-2000);
}

/**
 * Prepares move to start position
 */
void gotoStartPosition() {
  digitalWrite(ENABLE_PIN, LOW); // LOW to Enable
  Serial.println("Going to start position");
  runStartPosition = true;
  gooo = false;
  stepper.move(DEFAULT_STEPS_PER_MM * startPosition);
  stepper.setSpeed(2000);
}

/**
 * Prepares and starts main timelapse routine
 */
void gotoTimeLapse() {
  digitalWrite(ENABLE_PIN, LOW); // LOW to Enable
  Serial.print("Set to go. Servo speed: ");
  Serial.print(servoSpeed);
  Serial.print(" Total steps: ");
  Serial.print(totalSteps);
  Serial.print(" Time to run, sec: ");
  Serial.print(timetoRun);
  Serial.print(" Distance to run, mm: ");
  Serial.println(distancetoRun);
  
  gooo = true;
  homed = false;
  stepper.moveTo(totalSteps);
  stepper.setSpeed(servoSpeed);
}
