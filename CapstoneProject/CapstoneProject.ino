//Created by Drew Barclay
//Code uses thotro's dwm1000-arduino project
//Protocol: one byte for ID, then five bytes for the timestamp of the sending of the message, then a list of device specific stuff.
//For each device, append one byte for the ID of the device, one byte for the shared counter (used to detect lost transmissions), five bytes for timestamp of last received message from them, and four bytes for last calculated range

#include <SPI.h>
#define ANTENNA_DELAY 16384
#include <DW1000.h>

class Device  {
  public:
    byte id = 0;
    byte transmissionCount;
    DW1000Time timeDevicePrevSent;
    DW1000Time timePrevReceived;
    DW1000Time timeSent;
    DW1000Time timeDeviceReceived;
    DW1000Time timeDeviceSent;
    DW1000Time timeReceived;
    float lastComputedRange;
    bool hasReplied;

    Device() : lastComputedRange(0.0f), id(0), hasReplied(false) {}

    void computeRange();

    float getLastComputedRange() {
      return this->lastComputedRange;
    }
};

// CONSTANTS AND DATA START
//number of devices that can form a network at once
#define NUM_DEVICES 6
Device devices[NUM_DEVICES]; //Should be kept in order of increasing ID
int curNumDevices;
#define TX_ORDER_SIZE() (curNumDevices + 2)
byte txOrder[NUM_DEVICES + 2]; //+2 because we include our own ID and a dummy ID here

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// data buffer
#define LEN_DATA 256
byte data[LEN_DATA];

//id for this device
const byte OUR_ID = 7;
const byte DUMMY_ID = 255; //for use with ending a round

unsigned long timerStart; //from millis() for use with various code needing a reference start time
unsigned long txTimerStart; //same deal for transmission round orders
unsigned long timeTransmissionOrdered = 0;

byte lastReceivedID = 0;
boolean lastReceivedWasNew = false;
byte expectedIDIdx = 0;

// delay time before sending a message, should be at least 3ms (3000us)
const unsigned int DELAY_TIME_US = 2048 + 1000 + NUM_DEVICES * 83 + 200; //should be equal to preamble symbols (each take ~1us to transmit) + 1000 (base time to communicate and start transmitting and calculating a delay timestamp) + 4.5*bytes of data to send. add a little fudge room too. (experimentally found.) in microseconds.
const unsigned long DELAY_UNTIL_ASSUMED_LOST = 8000 + 1200 + DELAY_TIME_US + 1000; //estimated max parse time + transmit time + delay time on transmit + fudge factor

volatile bool received; //Set when we are interrupted because we have received a transmission
volatile bool sent = false; //Set when we successfully send

bool tookTurn = false;
// CONSTANTS AND DATA END

void Device::computeRange() {
  // only call this when timestamps are correct, otherwise strangeness may result
  // asymmetric two-way ranging (more computationly intense, less error prone)
  DW1000Time round1 = (timeDeviceReceived - timeDevicePrevSent).wrap();
  DW1000Time reply1 = (timeSent - timePrevReceived).wrap();
  DW1000Time round2 = (timeReceived - timeSent).wrap();
  DW1000Time reply2 = (timeDeviceSent - timeDeviceReceived).wrap();

  DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
  this->lastComputedRange = tof.getAsMeters();
}

unsigned long max_ul(unsigned long a, unsigned long b) {
  if (a > b) {
    return a;
  } else {
    return b;
  }
}

void setup() {
  txOrder[0] = OUR_ID;
  txOrder[1] = DUMMY_ID; //dummy value

  received = false;
  curNumDevices = 0;
  timerStart = millis();

  Serial.begin(19200);
  delay(1000);
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(OUR_ID);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY);
  //const byte mode[] = {DW1000.TRX_RATE_850KBPS, DW1000.TX_PULSE_FREQ_16MHZ, DW1000.TX_PREAMBLE_LEN_512};
  //DW1000.enableMode(mode);
  DW1000.commitConfiguration(16600);
  Serial.println(F("Committed configuration ..."));

  // attach callback for (successfully) sent and received messages
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachErrorHandler(handleError);
  DW1000.attachReceiveFailedHandler(handleReceiveFailed);

  receiver(); //start receiving
}

void handleError() {
  Serial.println("Error!");
}

void handleReceiveFailed() {
  Serial.println("Receive failed!");
}

void handleSent() {
  sent = true;
}

void handleReceived() {
  received = true;
}

void receiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

void parseReceived() {
  unsigned long parseTimer = micros();

  unsigned int len = DW1000.getDataLength();
  DW1000Time timeReceived;
  DW1000.getData(data, len);
  DW1000.getReceiveTimestamp(timeReceived);

  if (len < 6) {
    Serial.println("Received message with length <6, error.");
    return;
  }

  //Parse data
  //First byte, ID
  byte fromID = data[0];
  lastReceivedID = fromID;

  //Second byte, 5 byte timestamp when the transmission was sent (their clock)
  DW1000Time timeDeviceSent(data + 1);

  //If this device is not in our list, add it now.
  int idx = -1;
  for (int i = 0; i < curNumDevices; i++) {
    if (devices[i].id == fromID) {
      idx = i;
    }
  }
  if (idx == -1) { //If we haven't seen this device before...
    lastReceivedWasNew = true;
    Serial.print("New device found. ID: "); Serial.println(fromID);
    if (curNumDevices == NUM_DEVICES) {
      Serial.println("Max # of devices exceeded. Returning early from receive.");
      return;
    }

    //Find a place to put it in the transmission order
    int txIdx = 0;
    for (txIdx = 0; txIdx < TX_ORDER_SIZE(); txIdx++) {
      if (fromID < txOrder[txIdx]) {
        break;
      }
    }
    //Move everything up by 1 position to make room for this
    memmove(txOrder + txIdx + 1, txOrder + txIdx, sizeof(byte) * (TX_ORDER_SIZE() - txIdx));
    txOrder[txIdx] = fromID;

    //Add it to the device list
    devices[curNumDevices].id = fromID;
    devices[curNumDevices].timeDeviceSent = timeDeviceSent;
    devices[curNumDevices].transmissionCount = 1;
    idx = curNumDevices;
    curNumDevices++;
  } else {
    lastReceivedWasNew = false;
  }

  devices[idx].hasReplied = true;

  //Now, a list of device-specific stuff
  for (int i = 6; i < len;) {
    //First byte, device ID
    byte deviceID = data[i];
    i++;

    //Second byte, transmission counter
    byte transmissionCount = data[i];
    i++;

    //Next five bytes are the timestamp of when the device received our last transmit
    DW1000Time timeDeviceReceived(data + i);
    i += 5;

    //Next four bytes are a float representing the last calculated range
    float range;
    memcpy(&range, data + i, 4);
    i += 4;

    //Is this our device? If so, we have an update to do and a range to report!
    if (deviceID == OUR_ID) {
      //Mark down the two timestamps it included, as well as the time we received it
      devices[idx].timeDeviceReceived = timeDeviceReceived;
      devices[idx].timeDeviceSent = timeDeviceSent;
      devices[idx].timeReceived = timeReceived;

      //Serial.print("Transmission received from tag "); Serial.print(devices[idx].id); Serial.print(" with transmission count "); Serial.println(devices[idx].transmissionCount);

      //If everything looks good, we can compute the range!
      if (transmissionCount == 0) {
        //Error sending, reset everything.
        devices[idx].transmissionCount = 1;
      } else if (devices[idx].transmissionCount == transmissionCount) {
        if (devices[idx].transmissionCount > 1) {
          devices[idx].computeRange();
          Serial.print("!range "); Serial.print(OUR_ID); Serial.print(" "); Serial.print(devices[idx].id); Serial.print(" "); Serial.println(devices[idx].getLastComputedRange());
        }
        devices[idx].transmissionCount++;
      } else {
        //Error in transmission!
        devices[idx].transmissionCount = 0;
        //Serial.println("Transmission count does not match.");
      }

      devices[idx].timeDevicePrevSent = timeDeviceSent;
      devices[idx].timePrevReceived = timeReceived;
    }

    Serial.print("!range "); Serial.print(fromID); Serial.print(" "); Serial.print(deviceID); Serial.print(" "); Serial.println(range);
  }

  Serial.print("Receive time: "); Serial.println(micros() - parseTimer);
}

void doTransmit() {
  unsigned long transmitTimer = micros();

  data[0] = OUR_ID;

  //Normally we would set the timestamp for when we send here (starting at the second byte), but we'll do that later because this can take a while to calculate and we want the delay before sending to be short

  int curByte = 6;
  for (int i = 0; i < curNumDevices; i++) {
    data[curByte] = devices[i].id;
    curByte++;
    data[curByte] = devices[i].transmissionCount;
    curByte++;
    devices[i].timeReceived.getTimestamp(data + curByte); //last timestamp will contain the time we last received a transmission
    curByte += 5;
    float range = devices[i].getLastComputedRange();
    memcpy(data + curByte, &range, 4); //floats are 4 bytes
    curByte += 4;

    if (devices[i].hasReplied) {
      devices[i].transmissionCount++; //Increment for every transmission, the other device will also increment and we can check to see if they're the same to ensure the transmission was received
      devices[i].hasReplied = false; //Set to not for this round
    }
  }

  //Do the actual transmission
  DW1000.newTransmit();
  DW1000.setDefaults();

  timeTransmissionOrdered = micros();
  //Now we figure out the time to send this message!
  DW1000Time deltaTime = DW1000Time(DELAY_TIME_US, DW1000Time::MICROSECONDS);
  DW1000Time timeSent = DW1000.setDelay(deltaTime);
  timeSent.getTimestamp(data + 1); //set second byte (5 bytes will be written) to the timestamp

  DW1000.setData(data, curByte);
  DW1000.startTransmit();

  for (int i = 0; i < NUM_DEVICES; i++) {
    devices[i].timeSent = timeSent;
  }

  Serial.print("Transmit time: "); Serial.println(micros() - transmitTimer);
}

enum state_t {START_UP, ENTERING_NETWORK, IN_THE_ROUND} state;

void debug() {
  Serial.print("curNumDevices: "); Serial.print(curNumDevices);
  Serial.print(", state: "); Serial.print(state);
  Serial.print(", OUR_ID: "); Serial.print(OUR_ID);
  Serial.print(", devices[0].id: "); Serial.println(devices[0].id);
  Serial.print("Devices: ");
  for (int i = 0; i < TX_ORDER_SIZE(); i++) {
    Serial.print(txOrder[i]); Serial.print(" " );
  }
  Serial.println();
}

void checkTxOrderTime() {
  //Check timer and update if the last device was too slow
  if (micros() - txTimerStart > DELAY_UNTIL_ASSUMED_LOST + curNumDevices * 2000) {
    expectedIDIdx = (expectedIDIdx + 1) % TX_ORDER_SIZE();
    txTimerStart = micros();
    tookTurn = false;
  }
}

void updateExpectedTx() {
  //Find the index
  int idx = -1;

  if (txOrder[expectedIDIdx] == lastReceivedID) {
    idx = expectedIDIdx; //optimization to avoid a lookup. while everything's operating smooth this will be the path the code takes.
  } else {
    for (int i = 0; i < TX_ORDER_SIZE(); i++) {
      if (txOrder[i] == lastReceivedID) {
        idx = i;
        break;
      }
    }
  }

  if (idx == -1) {
    Serial.println("Fatal error on expected TX index. Not found.");
    return;
  }

  expectedIDIdx = (idx + 1) % TX_ORDER_SIZE();
  tookTurn = false;
}

void loop() {
  if (received) {
    received = false;
    unsigned long start = micros(); //This marks the beginning, since the delay used is always receive + transmit + delay time we must do this before parsing the receive
    parseReceived();
    if (!lastReceivedWasNew) {
      updateExpectedTx();
      txTimerStart = start; //Only update for non-new devices
    }
  }

  if (sent) {
    sent = false;
    tookTurn = false;
    txTimerStart = micros(); //Other devices will receive it at this point
    if (txOrder[expectedIDIdx] == OUR_ID) {
      expectedIDIdx++; //Can be done safely because of the dummy value
    }
  }

  checkTxOrderTime();

  switch (state) {
    case START_UP:
      if (millis() - timerStart > NUM_DEVICES * 100) {
        state = ENTERING_NETWORK; //Wait a little while before transmitting so this device doesn't interfere with the rest of the network
      }
      break;
    case ENTERING_NETWORK:
      if (txOrder[expectedIDIdx] == DUMMY_ID) {
        //Round is over, we can jump in!
        doTransmit();
        state = IN_THE_ROUND;
      }
      break;
    case IN_THE_ROUND:
      if (!tookTurn && OUR_ID == txOrder[expectedIDIdx]) {
        //Our turn to transmit!
        doTransmit();
        //Timer is not set here; if successfully sent, it is set in the if (sent) block above. Otherwise, we will move our expected ID up when the round expires.
        tookTurn = true;
        Serial.print("!id "); Serial.println(OUR_ID);
      }
      break;
  }
}

