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
#include <packetEngine.h>

/************ Radio Setup ***************/
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.0         // 434 MHz
#define HEARTBEAT_DELAY_MS 100  // Delay between heartbeat signals
#define REPLY_TIMEOUT_MS   50   // Reply timeout before sending another message
#define CONNECTION_TIMEOUT_MS 2000 // Number of ms to pass without reply
                                   // before determining connection is lost
#define GO 0x1                  // Go status
#define STOP 0x0                // STOP status
#define CONNECT 0x2             // Connection status
#define UNKNOWN 0x3             // Unknown status

/************ MESSAGES ***************/
#define CONNECTION_MSG "CONNECT"
#define GO_MSG "SENDIT"
#define STOP_MSG "STOP"
#define UNKNOWN_MSG "UNKNOWN"

#define PAYLOAD_LEN 20 // Length used for every payload
#define PACKET_LEN RH_RF69_MAX_MESSAGE_LEN // Used for every packet

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
#endif

#define ON_OFF        12 // Relay output pin

/** @brief Track ms between first transmission and reply */
unsigned long reply_ms = 0UL;

/** @brief Tracks ms between each transmission signal */
unsigned long heartbeat_ms = 0UL;

/** @brief Car state, initialize to unkown */
short car_state = UNKNOWN;

/** @brief Current connection status */
bool connected = false;

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

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
  // uint8_t key[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
  //                  0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  // rf69.setEncryptionKey(key);

  return true;
}

/**
 * @brief Sets up  the digital inputs and outputs
 */
void setup_digital() {
  pinMode(ON_OFF, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
}

/**
 * @brief Initializes the RF transceiver.
 *
 * For debugging purposes, sets up the serial commication.
 */
void setup() 
{
  Serial.begin(115200);
  
  setup_digital();

  Serial.println("Feather RFM69 RX Test!");
  Serial.println();

  if (!setup_RFM69()) {
    Serial.println("Feather RFM69 initialization failed. Trying again...");
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
 * @brief Sets the connection flag, toggles led
 */
bool set_connection(bool is_connected) {
  if (is_connected == connected) {
    return;
  }

  if (!is_connected) {
    // Turn off the motor
    toggle_power(false);
    Serial.println("Disconnecting...");    
  } else {
    Serial.println("Connected!");
  }

  connected = is_connected;
  toggle_led(is_connected);
}

/**
 * @brief Processes the received message. Updates the current state of
 * the vehicle and the connection to button.
*/
void process_message(short msg_type) {
  // Send the reply (echo the message)
  send_reply(msg_type);

  // Set the connection
  switch (msg_type) {
    case GO:
    case STOP:
    case CONNECT:
      set_connection(true);
      break;
    default:
      return;
  }

  // If change of state, update accordingly
  if (msg_type != car_state) {
    if (msg_type == GO) {
      toggle_power(true);
    } else {
      toggle_power(false);
    }
  }
}

/**
 * @brief Sends a message reply to the transmitting module
*/
void send_reply(short msg_type) {
  char *msg;
  uint8_t send_payload[PAYLOAD_LEN];
  uint8_t send_packet[PACKET_LEN];
  unsigned long msg_send_start_ms; // When we first send the message

  // Get the message to send
  msg = get_message_base(msg_type);

  // Place as many copies in the payload as possible
  size_t msg_len = strlen(msg);
  size_t i;
  for (i = 0; i < PAYLOAD_LEN; i += msg_len) {
      strncpy(((char *)send_payload)+ i, msg, msg_len);
  }
  ((char*)send_payload)[i] = '\0';

  Serial.print("Sending a reply: "); Serial.println((char*)send_payload);

  // Construct the packet and send
  if (constructPacket(send_payload, PAYLOAD_LEN, send_packet, PACKET_LEN) < 0) {
    Serial.println("Could not construct packet");
    return;
  }

  rf69.send(send_packet, PACKET_LEN);
  rf69.waitPacketSent();
}

/**
* @brief Reads the message and sends a reply (echoes the message)
* @return the true if a valid message received, otherwise false
*/
void receive_message() {
  short msg_state = UNKNOWN;

  if (rf69.waitAvailableTimeout(CONNECTION_TIMEOUT_MS)) {
    // Should be a message for us now
  uint8_t rec_payload[PAYLOAD_LEN];
  uint8_t rec_packet[PACKET_LEN];
  uint8_t rec_len = PACKET_LEN;

    if (rf69.recv(rec_packet, &rec_len)) {
      if (decodePacket(rec_packet, PACKET_LEN, rec_payload, PAYLOAD_LEN) < 0) {
        Serial.println("Could not receive packet");
        process_message(UNKNOWN);
        return;
      }
      rec_payload[PAYLOAD_LEN - 1] = '\0';

      Serial.print("Received: "); Serial.println((char *)rec_payload);

      if (strstr((char *)rec_payload, GO_MSG)) {
        msg_state = GO;
      } else if (strstr((char *)rec_payload, STOP_MSG)) {
        msg_state = STOP;
      } else if (strstr((char *)rec_payload, CONNECTION_MSG)) {
        msg_state = CONNECT;
      }

      process_message(msg_state);
      return;
    }
  }

  // Timed out - disconnect
  set_connection(false);
}

/**
 * @brief Infinite loop. Just receives message when available, and responds
 * to the messages as they come in.
*/
void loop() {
  receive_message();
}

/**
 * @brief Toggles the LED high or low
 * 
 * @param[in] on The desired state of the LED
*/
void toggle_led(bool on) {
  short state = on ? HIGH : LOW;
  digitalWrite(LED, state);
}

/**
 * @brief Toggles the LED high or low
 * 
 * @param[in] on The desired state of the LED
*/
void toggle_power(bool on) {
  short state = on ? HIGH : LOW;
  digitalWrite(ON_OFF, state);
}
