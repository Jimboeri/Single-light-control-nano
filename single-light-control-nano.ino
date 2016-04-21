/*
   This sketch will control a single LED
   It will turn on when motion is detected. The Passive Infrared ONBOARD_LED is a HC-SR501
   A light dependent resistor is used to detect the ambient light levels and a variable
   potentiometer to set the cut off when the light will turn on
   The processor is an ATMEL328 on an Arduiono Nano compatable clone
   A number of variable are kept which helps with decoding and could be used to get stats from the device in future
*/

/*
   Define libraries used here
*/
#include <EEPROM.h>

/*
   Pre-processor definitions
*/

#define VERSION_STRING "Single Light Control V1.0"
#define PIR_INPUT 8
#define LED 9
#define LDR A0
#define POT A4
#define ONBOARD_LED 13

// define the states for the LED
#define LED_OFF 0
#define LED_RAMP_UP 1
#define LED_ON 2
#define LED_RAMP_DOWN 3

// Define the states for the PIR
#define PIR_MOTION 1
#define PIR_STILL 0

// If set to 1 will generate stats to the serial port
#define REPORTING 1

// EEPROM Parameter offsets
#define PARAM_START_DELAY 1
#define PARAM_RAMP_UP 5
#define PARAM_RAMP_DOWN 7
#define PARAM_LED_ON 9
#define TOTAL_TIME 11
#define TOTAL_TIME_ON 13
#define INITIALISATIONS 15
#define ACTIVATIONS 17

#define MIN_LDR 100   // This is the minimum value the LDR will register

/*
   Define variables
*/
int motion = 0;                 // holds the status of the PIR output
int led_status = LED_RAMP_UP;   // The first thing the LED will do is turn on, regardless of light levels
int ramp_up = 2500;             // milliseconds needed to bring LED up to full brightness
int ramp_down = 30000;          // milliseconds needed to dim the LED off
unsigned long led_on_time = 150000L;       // milliseconds the light will stay on before checking the PIR again
long var1 = 0L;
int brightness;                 // used to calculate the LED brightness doring ramp up/down
long temp1;
int sensorValue = 0;            // value read from the light dependent resistor
int potValue = 0;               // value from the potentiometer
int potMapped = 0;              // this will store the adjusted potentiometer value so it can be compared to the LDR
long report_delay = 10000;       // millisec between reporting on sensor value
float disp_temp = 0.0;
int activations = 0;            // times the LED has been activated by the PIR
int initialisations = 0;        // Times the unit has been turned on
long total_time_on = 0L;         // total time on in ms
long curr_time_on = 0L;
long duration = 0L;

unsigned long start_delay = 100000L;  // The PIR takes about 1 minute to settle down
unsigned long next_check = 0L;         // This is the next time the PIR will be checked
unsigned long start = 0;
unsigned long current = 0;
unsigned long report_sensor = 0;
unsigned long t1 = 0L;

int t2 = 0;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(VERSION_STRING);
  pinMode(PIR_INPUT, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(LED, LOW);
  analogReference(EXTERNAL);
  next_check = millis() + start_delay + led_on_time;
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  // the EEPROM setup is in a procedure to keep the code tidy
  setup_param();
}

void loop() {
  // put your main code here, to run repeatedly:
  motion = digitalRead(PIR_INPUT);        // get the PIR status
  current = millis();                     // store the current ms time
  sensorValue = analogRead(LDR);          // read the LDR
  potValue = analogRead(POT);             // read the potentiometer
  potMapped = map(potValue, 0, 1023, MIN_LDR, 1023);  // it was useful to adjust the Pot value as the LDR never gets down to 0
  serialEvent();            // call the serial processing function
  if (stringComplete)       // keep all serial processing in one place
    process_serial();

  // This section lights the onboard LED if the light level is too high to allow the LED to be triggered
  if (sensorValue > potMapped)
  {
    digitalWrite(ONBOARD_LED, HIGH);
  }
  else
  {
    digitalWrite(ONBOARD_LED, LOW);
  }

  // output sensor value to serial channel
  if (report_sensor < current) {
    report_sensor = current + report_delay;
    if (REPORTING)
    {
      debug_info("Regular check");
    }
  }

  // 4 sections of code depending of the LED status
  switch (led_status) {
    case LED_RAMP_UP:
      if (current > (start + ramp_up)) {      // this is the end of the ramp up, ensure the LED is on and change status
        digitalWrite(LED, HIGH);
        led_status = LED_ON;

        // update the activations counter and write it to the EEPROM
        activations += 1;
        EEPROM.put(ACTIVATIONS, activations);
        Serial.print("Ramp up complete - number of times triggered = ");
        Serial.println(activations);
        Serial.print("Number of times light turned on = ");
        Serial.println(initialisations);
        break;
      }
      var1 = (current - start) * 255L;        // brightness increases during ramp up
      brightness = int(var1 / ramp_up);
      analogWrite(LED, brightness);
      delay(1);
      break;

    case LED_RAMP_DOWN:
      // if motion is detected while the LED is turning off it needs to go back on
      if (motion == PIR_MOTION) {
        led_status = LED_RAMP_UP;
        Serial.println("Motion detected - turning on");
      }

      if (current > (start + ramp_down)) {
        // end of the ramp down
        digitalWrite(LED, LOW);
        led_status = LED_OFF;
        Serial.println("Ramp down complete");
        duration = current - curr_time_on;
        Serial.print("Light was on for ");
        Serial.print(duration / 1000);
        Serial.println(" seconds");
        // update total time on and write to EEPROM
        total_time_on += duration;
        EEPROM.put(PARAM_LED_ON, total_time_on);
        break;
      }
      // calculate brightness duruing ramp down
      var1 = int(current - start);
      temp1 = var1 * 255L;
      brightness = 255 - (temp1 / ramp_down);
      analogWrite(LED, brightness);
      delay(1);
      break;

    case LED_ON:
      // if motion is detected while on, update the next check into the future
      if (motion == PIR_MOTION)
      {
        next_check = current + led_on_time;    // if the PIR detects motion reset the next check time
        break;
      }
      else
      {
        if (next_check < current)              // if no motion has been detected by the next check, start the ramp down
        {
          led_status = LED_RAMP_DOWN;
          start = millis();
          Serial.println("NO Motion detected - turning off");
        }
      }     // end of LED_ON section
      break;

    case LED_OFF:
      if (motion == PIR_MOTION)                 // motion detected
      {
        // if (report_sensor - 3 < current)
        //   Serial.println("Motion detected - check for light level");
        if (sensorValue < potMapped) {     // we only turn on if the ambient light is below this value
          next_check = millis() + led_on_time + ramp_up;
          led_status = LED_RAMP_UP;
          start = millis();
          Serial.println("Ambient light level low enough - turning on");
        }
      }   // end of LED_OFF section
      break;
  }       // end of case statement
}         // end of the main loop

/*
  SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) { // keep on reading while info is available
    // get the new byte:
    char inByte = Serial.read();

    // add it to the inputString:
    if ((inByte >= 65 && inByte <= 90) || (inByte >= 97 && inByte <= 122) || (inByte >= 48 && inByte <= 57) || inByte == 43 || inByte == 61 || inByte == 63) {
      inputString.concat(inByte);
    }
    if (inByte == 10 || inByte == 13) {
      // user hit enter or such like
      Serial.println(inputString);
      stringComplete = true;
    }
  }
}

void process_serial() {
  inputString.toUpperCase();

  if (inputString == "SHOW") {
    disp_temp = ramp_up / 1000;
    Serial.print("Ramp up time:           ");
    Serial.print(disp_temp);
    Serial.println("s");
    disp_temp = ramp_down / 1000;
    Serial.print("Ramp down time:         ");
    Serial.print(disp_temp);
    Serial.println("s");
    disp_temp = led_on_time / 1000;
    Serial.print("Light on time:          ");
    Serial.print(disp_temp);
    Serial.println("s");

    Serial.print("Times light turned on:  ");
    Serial.println(initialisations);

    Serial.print("Times light activated on:  ");
    Serial.println(activations);

  }
  else Serial.println("Clueless");

  // clear the string:
  inputString = "";
  stringComplete = false;

}

void debug_info(String msg) {
  // this section prints useful debugging info to the serial channel
  Serial.println(msg);
  Serial.print("LED status indicator ");
  Serial.print(led_status);
  switch (led_status) {
    case LED_ON:
      Serial.println("  LED ON");
      break;
    case LED_OFF:
      Serial.println("  LED OFF");
      break;
    case LED_RAMP_UP:
      Serial.println("  LED RAMP UP");
      break;
    case LED_RAMP_DOWN:
      Serial.println("  LED RAMP DOWN");
      break;
  }
  Serial.print("Light dependent Resistor reading ");
  Serial.println(sensorValue);
  Serial.print("Potentiometer reading ");
  Serial.print(potValue);
  Serial.print("  Potentiometer adjusted reading ");
  Serial.println(potMapped);
  Serial.print("Motion detection ");
  Serial.println(motion);
  Serial.print("Seconds since motion last detected ");
  Serial.println((current + led_on_time - next_check) / 1000);
  Serial.print("Number of times powered on ");
  Serial.println(initialisations);
  Serial.print("Number of times triggered ");
  Serial.println(activations);
  Serial.print("Total time spent on ");
  Serial.println(total_time_on / 1000);

  Serial.println(" ");
}

void setup_param()
// this is only called once, put here for simpler code
{

  Serial.print("Start delay is ");
  Serial.print(start_delay);
  Serial.println(" millseconds");

  Serial.print("Ramp up time is ");
  Serial.print(ramp_up);
  Serial.println(" millseconds");

  Serial.print("Ramp down time is ");
  Serial.print(ramp_down);
  Serial.println(" millseconds");

  Serial.print("Time light is to stay on is ");
  Serial.print(led_on_time);
  Serial.println(" milliseconds");

  //EEPROM.put(TOTAL_TIME_ON, 0L);      // Just the first time the sketch runs, then comment it out
  EEPROM.get(TOTAL_TIME_ON, total_time_on);      // The total time the LED has been on
  Serial.print("Total ON time is ");
  Serial.print(total_time_on);
  Serial.println(" milliseconds");

  EEPROM.get(INITIALISATIONS, initialisations);      // initialisations
  initialisations += 1;
  EEPROM.put(INITIALISATIONS, initialisations);

  EEPROM.get(ACTIVATIONS, activations);      // initialisations
}


