//Created by Drew Barclay

#include <SPI.h>
#include <DW1000.h>

//number of devices that can form a network at once
#define NUM_DEVICES 4

class Device  {
public:
  byte id;
  byte transmissionCount;
  DW1000Time timeDevicePrevSent;
  DW1000Time timePrevReceived;
  DW1000Time timeSent;
  DW1000Time timeDeviceReceived;
  DW1000Time timeDeviceSent;
  DW1000Time timeReceived;
  float lastComputedRange;
  byte hasReplied;
  
  Device() : lastComputedRange(0.0f) {}
  
  void computeRange() {    
    // only call this when timestamps are correct, otherwise strangeness may result
    // asymmetric two-way ranging (more computationly intense, less error prone)
    DW1000Time round1 = (timeDeviceReceived - timeDevicePrevSent).wrap();
    DW1000Time reply1 = (timeSent - timePrevReceived).wrap();
    DW1000Time round2 = (timeReceived - timeSent).wrap();
    DW1000Time reply2 = (timeDeviceSent - timeDeviceReceived).wrap();
    
    if (round1.getTimestamp() > reply1.getTimestamp() && round2.getTimestamp() > reply2.getTimestamp()) { //Sanity check to ensure we're not doing incorrect math
      DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
      this->lastComputedRange = tof.getAsMeters();
    }
  }
  
  float getLastComputedRange() {
    return this->lastComputedRange;
  }
};

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

//Protocol: one byte for ID, then five bytes for the timestamp of the sending of the message, then a list of device specific stuff. 
//For each device, append one byte for the ID of the device, one byte for the shared counter (used to detect lost transmissions), five bytes for timestamp of last received message from them, and four bytes for last calculated range

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

volatile bool received;

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
    ourID = 1; //change per device    
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
      devices[curNumDevices].timeDeviceSent = timeDeviceSent;
      devices[curNumDevices].transmissionCount = 1;
      idx = curNumDevices;
      curNumDevices++;
      Serial.print("New device found. ID: "); Serial.println(fromID); 
    } 
    
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
      i+= 4;
      
      //Is this our device? If so, we have an update to do and a range to report!
      if (deviceID == ourID) {
        //Mark down the two timestamps it included, as well as the time we received it
        devices[idx].timeDeviceReceived = timeDeviceReceived;
        devices[idx].timeDeviceSent = timeDeviceSent;
        devices[idx].timeReceived = timeReceived;
        
        Serial.print("Transmission received from tag "); Serial.print(devices[idx].id); Serial.print(" with transmission count "); Serial.println(devices[idx].transmissionCount);
                
        //If everything looks good, we can compute the range! 
        if (devices[idx].transmissionCount == transmissionCount) {
          if (transmissionCount != 0) {
            devices[idx].computeRange();
            Serial.print("New range from this tag to "); Serial.print(devices[idx].id); Serial.print(". Meters: "); Serial.println(devices[idx].getLastComputedRange());
          }
          devices[idx].transmissionCount++;
        } else {
          //Error in transmission!
          devices[idx].transmissionCount = 0; 
          Serial.println("Transmission count does not match. Resetting device info.");
        }
        devices[idx].timeDevicePrevSent = timeDeviceSent;
        devices[idx].timePrevReceived = timeReceived;
      } else {
        Serial.print("Reported range from "); Serial.print(fromID); Serial.print("<->"); Serial.print(deviceID); Serial.println(", meters: "); Serial.println(range);
      }
    }
    //TODO if our device was not in the list and we think it should have been, raise an error/reset the ranging stuff. Being robust is important!
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
      data[curByte] = devices[i].transmissionCount;
      curByte++;
      devices[i].timeReceived.getTimestamp(data + curByte); //last timestamp will contain the time we last received a transmission
      curByte += 5;
      float range = devices[i].getLastComputedRange();
      Serial.print("Range, ID: "); Serial.print(range); Serial.print(" "); Serial.println(devices[i].id);
      memcpy(data + curByte, &range, 4); //floats are 4 bytes
      curByte += 4;
      
      devices[i].transmissionCount++; //Increment for every tranmission, the other device will also increment and we can check to see if they're the same to ensure the transmission was received
    }
    
    for (int i = 0; i < NUM_DEVICES; i++) {
      devices[i].timeSent = timeSent;
    }
    
    DW1000.setData(data, curByte);
    DW1000.startTransmit();
    lastTransmission = curMillis;
  }
}

