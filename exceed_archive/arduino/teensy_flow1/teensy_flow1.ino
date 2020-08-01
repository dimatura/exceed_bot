
#include <Metro.h>
#include <CircularBuffer.h>
#include <PacketSerial.h>

#include "Bitcraze_PMW3901.h"

#define SPEED_INTERVAL_MS 20
#define SPEED_BUFFER_SIZE 20

// Using digital pin 10 for chip select
Bitcraze_PMW3901 flow(10);
int16_t deltaX = 0, deltaY = 0;
PacketSerial packets;

Metro flow_check(SPEED_INTERVAL_MS);

void setup() {
  Serial.begin(115200);
  packets.setStream(&Serial);
  //packets.setPacketHandler(&on_packet);

  if (!flow.begin()) {
    Serial.println("Initialization of the flow sensor failed");
    while(1) { }
  }
}


void send_status() {
  uint8_t buff[sizeof(int32_t)];
  int32_t delta = deltaX;
  memcpy(&buff[0], (uint8_t *) &delta, sizeof(buff));
  packets.send(buff, sizeof(buff));
}


void loop() {
  // Get motion count since last call
  if (flow_check.check()) {
    flow.readMotionCount(&deltaX, &deltaY);
    send_status();
  }
  packets.update();
}
