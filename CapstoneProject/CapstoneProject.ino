//Created by Drew Barclay

#include <SPI.h>
#include <DW1000.h>

//number of devices that can form a network at once
#define NUM_DEVICES 7

class Device  {
public:
  byte id;
  DW1000Time timestamps[6]; //in chronological order
  
  Device() {
    for (int i = 0; i < 6; i++) {
      timestamps[i] = DW1000Time((int64_t)1); //prevent division by zero
    } 
  }
  
  float computeRange() {    
    // asymmetric two-way ranging (more computationly intense, less error prone)
    DW1000Time round1 = (timestamps[3] - timestamps[0]).wrap();
    DW1000Time reply1 = (timestamps[2] - timestamps[1]).wrap();
    DW1000Time round2 = (timestamps[5] - timestamps[2]).wrap();
    DW1000Time reply2 = (timestamps[4] - timestamps[3]).wrap();
    DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
    // set tof timestamp
    return tof.getAsMeters();
  }
};

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

//Protocol: one byte for ID, then five bytes for the timestamp of the sending of the message, then a list of device specific stuff. 
//For each device, append one byte for the ID of the device, five bytes for timestamp of last received message from them, and four bytes for last calculated range

// data buffer
#define LEN_DATA 256
byte data[LEN_DATA];

Device devices[NUM_DEVICES];
int curNumDevices;

//id for this device
byte ourID;

//length of data
unsigned int len;

long lastTransmission; //from millis()

//time when message was received on our end
DW1000Time timeReceived;

DW1000Time timeDeviceSent;

// reply times (same on both sides for symm. ranging)
unsigned int replyDelayTimeUS = 3000;

bool received;

int findDeviceIndex(int id) {
  for (int i = 0; i < curNumDevices; i++) {
     if (devices[i].id == id) {
       return i;
     }
  }
  return -1; //not found
}

void setup() {
    received = false;
    ourID = 2; //change per device    
    curNumDevices = 0;
    lastTransmission = millis();

    Serial.begin(115200);
    delay(1000);
    // initialize the driver
    DW1000.begin(PIN_IRQ, PIN_RST);
    DW1000.select(PIN_SS);
    Serial.println(F("DW1000 initialized ..."));
    // general configuration
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(2);
    DW1000.setNetworkId(10);
    DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY);
    DW1000.commitConfiguration();
    Serial.println(F("Committed configuration ..."));

    // attach callback for (successfully) sent and received messages
    DW1000.attachSentHandler(handleSent);
    DW1000.attachReceivedHandler(handleReceived);
    
    receiver(); //start receiving
}

void handleSent() {
  
}

void handleReceived() {
  received = true;
  Serial.println("Received, set.");
}

void receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    // so we don't need to restart the receiver manually
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

void loop() {
  unsigned long curMillis = millis();
  
  if (received) {
    received = false;
    len = DW1000.getDataLength();
    Serial.println("Rec"); Serial.println(len);
    DW1000.getData(data, len);
    DW1000.getReceiveTimestamp(timeReceived);
    
    if (len < 6) {
       Serial.println("Received message with length <6, error."); 
       return;
    }
    
    //Parse data
    //First byte, ID
    byte fromID = data[0];
    //Second byte, 5 byte timestamp when the transmission was sent (their clock)
    timeDeviceSent.setTimestamp(data + 1);
    
    //If this device is not in our list, add it now.
    int idx = findDeviceIndex(fromID);
    if (idx == -1) {
      devices[curNumDevices].id = fromID;
      devices[curNumDevices].timestamps[5] = timeDeviceSent;
      idx = curNumDevices;
      curNumDevices++;
    } 
    
    //Now, a list of device-specific stuff
    /*for (int i = 6; i < len;) {
      //First byte, device ID
      byte deviceID = data[i];
      i++; 
      
      //Next five bytes are the timestamp of when the device received our last transmit
      DW1000Time timeDeviceReceived(data + i);
      i += 5; 
      
      //Next four bytes are a float representing the last calculated range
      float range;
      memcpy(&range, data + i, 4);
      i+= 4;
      
      //Is this our device? If so, we have an update to do and a range to report!
      if (deviceID == ourID) {
        //Mark down the two timestamps it included, as well as the time we received it
        //Move everything down by three places
        memmove(devices[idx].timestamps, devices[idx].timestamps + 3, sizeof(DW1000Time) * 3);
        devices[idx].timestamps[3] = timeDeviceReceived;
        devices[idx].timestamps[4] = timeDeviceSent;
        devices[idx].timestamps[5] = timeReceived; 
        Serial.print("New range! ID: "); Serial.print(devices[idx].id); Serial.print(" Range: "); Serial.println(devices[idx].computeRange());
      } else {
        //devices[idx].updateRangeForID(deviceID, range);
        //todo send this over serial to cellphone
      }
    }*/
  }
  
  if (curMillis - lastTransmission > 300) {
    //Transmit every 30 ms 
    DW1000.newTransmit();
    DW1000.setDefaults();
    
    data[0] = ourID;
    
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    DW1000Time timeSent = DW1000.setDelay(deltaTime);
    timeSent.getTimestamp(data + 1); //set second byte (5 bytes will be written) to the timestamp
    
    int curByte = 6;
    for (int i = 0; i < curNumDevices; i++) {
      data[curByte] = devices[i].id;
      curByte++;
      devices[i].timestamps[5].getTimestamp(data + curByte); //last timestamp will contain the time we last received a transmission
      curByte += 5;
      float range = devices[i].computeRange();
      memcpy(data + curByte, &range, 4); //floats are 4 bytes
      curByte += 4;
      
      //Bookkeeping: move down the timestamps, append the time we sent this message to the last place
      memmove(devices[i].timestamps, devices[i].timestamps + 1, sizeof(DW1000Time) * 5);
      devices[i].timestamps[5] = timeSent;
    }
    
    DW1000.setData(data, curByte);
    DW1000.startTransmit();
    lastTransmission = curMillis;
  }
}

