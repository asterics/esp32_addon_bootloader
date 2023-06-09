/*
 *  RP2040 - ESP32 serial passthrough
 *  connect Arduino Nano pin D2 (GPIO 25) to GND to create am ESP32-reset / boot condition
 */

#define ACTIVATE_GPIO0  // comment if GPIO0 should not be pressed before reset of ESP32 !

#define ORANGE_LED_PIN  6
#define ESP32_RESET_PIN 3
#define ESP32_GPIO0_PIN 2
#define PUSHBUTTON_PIN  25

#define RESET_BYPASS_TIME 100000
#define GPIO0_HOLD_TIME    50000
#define LED_TOGGLE_TIME    20000


void setup() {
  
  pinMode(ORANGE_LED_PIN, OUTPUT); 
  pinMode(PUSHBUTTON_PIN, INPUT_PULLUP); 
  digitalWrite(ESP32_RESET_PIN,HIGH); pinMode(ESP32_RESET_PIN, OUTPUT);
  #ifdef ACTIVATE_GPIO0
    digitalWrite(ESP32_GPIO0_PIN,HIGH); pinMode(ESP32_GPIO0_PIN, OUTPUT);
  #endif

  Serial.begin(115200);
  Serial2.begin(115200);
  // delay (5000);
  // Serial.println("Hi - ESP passthrough!");
}

long cnt=0;
long bypass_reset=0;

// the loop function runs over and over again forever
void loop() {

  if (!(cnt++ % LED_TOGGLE_TIME)) 
    digitalWrite(ORANGE_LED_PIN,!digitalRead(ORANGE_LED_PIN));
  
  if ((digitalRead(PUSHBUTTON_PIN) == LOW) && (!bypass_reset)) {
    // Serial.println("reset ESP");
    #ifdef ACTIVATE_GPIO0
      digitalWrite(ESP32_GPIO0_PIN,LOW);   // connect GPIO0 to GND
    #endif
    digitalWrite(ESP32_RESET_PIN,LOW);   // reset ESP32
    delay(10); digitalWrite(ESP32_RESET_PIN,HIGH);
    bypass_reset++;
  }

  if (bypass_reset) { 
    #ifdef ACTIVATE_GPIO0
      if (bypass_reset == GPIO0_HOLD_TIME) digitalWrite(ESP32_GPIO0_PIN,HIGH);   // end GPIO0 activation
    #endif
    if (++bypass_reset == RESET_BYPASS_TIME) bypass_reset=0;
  }


  // pass serial transfer to Serial2 (ESP)
  
  if (Serial.available()) {      // If anything comes in Serial (USB),
    Serial2.write(Serial.read());   // read it and send it out Serial1 (pins 0 & 1)
  }

  if (Serial2.available()) {     // If anything comes in Serial1 (pins 0 & 1)
    Serial.write(Serial2.read());   // read it and send it out Serial (USB)
  }
  
}
