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
#include <string.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.0         // 434 MHz
#define DEBOUNCE_DELAY_MS 20    // 20ms debounce delay
#define HEARTBEAT_DELAY_MS 100  // Delay between heartbeat signals
#define REPLY_TIMEOUT_MS   20   // Reply timeout before sending another message
#define CONNECTION_TIMEOUT_MS 2000 // Number of ms to pass without reply
                                   // before determining connection is lost
#define GO 0x1                  // Go status
#define STOP 0x0                // STOP status
#define CONNECT 0x2             // Connection status
#define UNKNOWN 0x3             // Unknown status

// Messages to send the car for each state
#define CONNECTION_MSG "CONNECT"
#define GO_MSG "SEND IT"
#define STOP_MSG "STOP"
#define UNKNOWN_MSG "UNKNOWN"

#if defined(__AVR_ATmega32U4__) // Feather 32u4 w/Radio
#define RFM69_CS 8
#define RFM69_INT 7
#define RFM69_RST 4
#define LED 13
#define BUTTON 12
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

/** @brief Track ms between first transmission and reply */
unsigned long reply_ms = 0UL;

/** @brief Heartbeat counter, increment every go transmission */
unsigned long n_heartbeat = 0;

/** @brief Tracks ms between each heartbeat signal */
unsigned long heartbeat_ms = 0UL;

/** @brief Active high button state */
short button_state = LOW;

/** @brief Car state, initialize to unkown */
short car_state = UNKNOWN;

/** @brief Current connection status */
bool connected = false;

/**
 * @brief Blinks output at defined at PIN LOOPS times with DELAY_MS in between.
 *
 * @param PIN The pin to blink
 * @param DELAY_MS The delay in ms between blinks
 * @param loops The number of loops between blinks
 */
void blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i = 0; i < loops; i++) {
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN, LOW);
    if (i != loops - 1) {
      delay(DELAY_MS);
    }
  }
}

/**
 * @brief Toggles the LED high or low
 * 
 * @param[in] on The desired state of the LED
*/
void toggle_led(bool on) {
  byte state = on ? HIGH : LOW;
  digitalWrite(LED, state);
}

/**
 * @brief Checks if the button is pressed and takes into account the time to debounce
 *
 * @return true if the button is pressed, otherwise false
 */
bool button_pressed() {
  int button = digitalRead(BUTTON);
  
  if (button == LOW) {
    return false;
  } else if (car_state == GO) {
    // If button already pressed, no need to debounce
    return true;
  }

  /* Wait for delay to allow button to debounce */
  unsigned long press_start_ms = millis();
  while ((millis() - press_start_ms) < DEBOUNCE_DELAY_MS) {
    /* Spin */
  }

  button = digitalRead(BUTTON);
  return (button == HIGH);
}

/** 
 * @brief Attempts to establish conection between the two 
 */
bool establish_connection() {
  if (!send_message(CONNECT)) {
    return;
  }

  connected = true;
  car_state = UNKNOWN;
  toggle_led(true);
}

/** 
 * @brief Sets connected flag to disconnected, car state to unknown
*/
void disconnect() {
  // Message sending unsuccessful
  connected = false;
  car_state = UNKNOWN;
  toggle_led(false);
  Serial.println("Stop signal not received");
  Serial.println("Disconnecting...")
}

/**
 * @brief Sets up  the digital inputs and outputs
 */
void setup_digital() {
  pinMode(BUTTON, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
}

/**
 * @brief Initializes the RFM69 module
 *
 * @return True if initialization successful, otherwise false
 */
bool setup_RFM69() {
  // Manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  // Initialize RF Module
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    blink(LED, 100, 3);
    return false;
  }
  blink(LED, 1000, 1); // Indicates power on
  Serial.println("RFM69 radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
    return false;
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true); // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  return true;
}

/**
 * @brief Initializes the RF transceiver, establishes the connection
 * between the two RF modules. Also sets the encrpytion key.
 *
 * For debugging purposes, sets up the serial commication.
 */
void setup()
{
  Serial.begin(115200);

  setup_digital();

  Serial.println("Feather RFM69 TX Test!");
  Serial.println();

  while (!setup_RFM69()) {
    Serial.println("Feather RFM69 initialization failed");
  }
  Serial.println("Feather RFM69 initialization successful!");

  Serial.print("RFM69 radio @");
  Serial.print((int)RF69_FREQ);
  Serial.println(" MHz");
}

/**
 * @brief Takes in a message type and returns the string associated with 
 * that message
 * 
 * @param[in] msg_type The type of message
 * @return The string value associated with that message
*/
char *get_message_base(byte msg_type) {
  switch (msg_type) {
    case (GO):
      return GO_MSG;
    case (STOP):
      return STOP_MSG;
    case (CONNECT):
      return CONNECTION_MSG;
    default:
      return UNKNOWN_MSG;
  }
}

/**
 * @brief Sends a message to the receiver. Attempts to send a message 
 * until it is received, or connection times out.
 * 
 * @param[in] msg_type The type of message to send
 * @return True if message was received and validated, otherwise false
*/
bool send_message(short msg_type) {
  char *msg;
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  unsigned long msg_send_start_ms; // When we first send the message

  // Get the message to send
  msg = get_message_base(msg_type);

  // Triple the message and place it into packet
  size_t msg_len = strlen(msg);
  size_t pckt_len = 3 * (msg_len + 1);
  char pckt[msg_len];
  size_t i;
  for (i = 0; i < msg_len; i += msg_len) {
    strncpy(pckt + i, msg, msg_len);
  }

  msg_send_start_ms = millis();

  // Keep sending the message until receives reply or connection is lost
  while ((msg_send_start_ms - millis()) < CONNECTION_TIMEOUT_MS) {
    // Send the message
    Serial.print("Sending: "); Serial.println(pckt);
    rf69.send((uint8_t *)pckt, strlen(msg_len));
    rf69.waitPacketSent();
    
    // Wait for reply
    if (rf69.waitAvailableTimeout(REPLY_TIMEOUT_MS)) {
      // Should be a reply message for us now
      if (rf69.recv(buf, &len)) {
        if (strstr((char *)buf, msg)) {
          return true;
        }
      }
    }
  }

  return false;
}

/**
 * @brief Sends the first message to the car during a state change.
 * This message runs asynchronously, and is not dependent on the 
 * heartbeat.
 * 
 * @param[in] next_car_state The new desired car state
*/
void send_init_message(short next_car_state) {
  if (car_state == next_car_state) {
    // Car state is not different
    // Should be sending heartbeat
    return;
  }

  if (!send_message(next_car_state)) {
    disconnect();
    return;
  }

  // Message received, update car state
  car_state = next_car_state;

  // Reset heartbeat state
  heartbeat_ms = millis();
  n_heartbeat = 0;
}

/**
 * @brief Sends a hearbeat signal with the current state of the car
*/
void send_heartbeat() {
  // Not time to send a heartbeat, continue
  if ((heartbeat_ms - millis()) < HEARTBEAT_DELAY_MS) {
    return;
  }

  // Send heartbeat
  if (!send_message(car_state)) {
    disconnect();
  }

  Serial.println("Heartbeat %lu received", n_heartbeat);

  // Update heartbeat counters
  n_heartbeat++;
  heartbeat_ms = millis();
}

/**
 * @brief The continous loop. If not connected, tries to establish connection.
 * Sends an intitial message for any state change in the button. Otherwise
 * sends heartbeat with the current car state to maintain connection.
*/
void loop()
{
  // Try to establish connection if not already established
  if (!connected) {
    if (!establish_connection()) {
      return;
    }
  }

  // Send the stop signal
  if (!button_pressed()) {
    // If button state already low, just continue
    if (car_state != STOP) {
      send_init_message(STOP);
    }
  } else {
    if (car_state != GO) {
      send_init_message(GO);
    }
  }

  // Continue sending the heartbeat signal for current state
  send_heartbeat();
}
