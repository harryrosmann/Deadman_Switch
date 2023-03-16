// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ       434.0   // 434 MHz
#define DEBOUNCE_DELAY  20      // 20ms debounce delay
#define GO              0x1     // Go status
#define KILL            0x0     // Kill status

#define GO_MESSAGE      "SEND IT"
#define KILL_MESSAGE    "STOP"
#define UNKNWON_MESSAGE "UNKNOWN"

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
  #define BUTTON        12
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission
unsigned long start_ms = 0;  // time counter when the system starts up

int BUTTON_STATUS   = LOW; // start as unpressed

void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}

/**
* @brief Checks if the button is pressed and takes into account the time to debounce
*
* @return true if the button is pressed, otherwise false
*/
bool Button_Pressed() {
  int button_state = digitalRead(BUTTON);
  if (button_state == LOW) {
    return false;
  }

  /* Wait for delay to allow button to debounce */
  unsigned long press_start = millis();
  while (millis() - press_start < DEBOUNCE_DELAY) {
    /* Spin */
  }

  button_state = digitalRead(BUTTON);
  return button_state == HIGH;
}

bool Send_Message(char* radiopacket) {
  bool acknowledged = false; // tracks whether message was echoed or not

  Serial.print("Sending "); Serial.println(radiopacket);
  
  // Send a message!
  rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
  rf69.waitPacketSent();

  // Now wait for a reply
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf69.waitAvailableTimeout(500))  { 
    // Should be a reply message for us now   
    if (rf69.recv(buf, &len)) {
      if (strstr((char*)buf, radiopacket)) {
        acknowledged = true;
        Serial.println("Go acknowledged");
      } else {
        Serial.println("RX did not understand");
      }
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply");
  }

  if (acknowledged) {
    Blink(LED, 50, 3); //blink LED 3 times, 50ms between blinks
    return true;
  }
  Blink(LED, 150, 1); //blink LED once
  return false;
}

void loop() {
  // Send the kill signal
  if (!Button_Pressed()) {
    // Reset variables
    if (BUTTON_STATUS == HIGH) {
      BUTTON_STATUS = LOW;
      start_ms = 0;
    }

    if (!Send_Message(KILL_MESSAGE)) {
      Serial.println("Kill signal not received");
    }
  } 
  // Send the go signal
  else {
    if (BUTTON_STATUS == LOW) {
      BUTTON_STATUS = HIGH;
    }
    
    if (!Send_Message(GO_MESSAGE)) {
      Serial.println("Go signal not received");
    }

    unsigned long duration = millis() - start_ms;
    Serial.print("Duration: "); Serial.println(duration, DEC);
  }

}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    if (i != loops - 1) {
      delay(DELAY_MS);
    }
  }
}
