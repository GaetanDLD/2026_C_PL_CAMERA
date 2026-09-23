#include <Arduino.h>
#include "Camera.h"
#include "Bigbrother.h"

#define CAM_AERO_TOP 0b01  // blue spacers
#define CAM_SEPMECH 0b10   // yellow spacers
#define CAM_AERO_BOT 0b00  // red spacers

#define REC_TIME 60000     // Record time for standalone camera 
#define check_delay 5000   // Delay between two cam state checks

typedef enum {             // Controler states
  ST_WAIT_CAN,
  ST_STANDBY,
  ST_PWR_CAM_ON,
  ST_START_REC,
  ST_STOP_REC,
  ST_PWR_CAM_OFF,
  ST_FAIL,
  ST_FIN
} states;

states currentState = ST_PWR_CAM_ON;

bool alone = false;         // Check control method
bool shouldRec = false;     // Expected camera state
bool isFirstToggle = true;  // First toggle flag
bool end_done = false;      // Is sequence finished flag

uint32_t toggle_time = 0;   // !!! uint32_t (unsigned long) required to avoid overflow with millis() !!!
uint32_t last_check = 0;
uint8_t try_rec = 0;

Camera cam;
Bigbrother bigbro(CAM_SEPMECH);   // Check CAN adrr when upload !

void setup() {
  bigbro.begin();
  cam.begin(bigbro);

  alone = bigbro.get_mode();
}

void loop() {
  switch (currentState) {

    case ST_WAIT_CAN:   // Read CAN bus and follow command
      { auto msg = bigbro.listen_CAN();

        if (msg == Bigbrother::MessageType::START_REC) currentState = ST_START_REC;
        else if (msg == Bigbrother::MessageType::STOP_REC) currentState = ST_STOP_REC;
        else if (msg == Bigbrother::MessageType::HEALTH) {
          bigbro.send_health_packet(bigbro.isRec());
          currentState = ST_STANDBY;
        } 
        else currentState = ST_STANDBY;
      }
      break;

    case ST_STANDBY:
      // Check camera is in expected state
      { uint32_t t = millis();

        if (alone && (t - toggle_time >= REC_TIME)) currentState = ST_STOP_REC;
        else if (t - last_check >= check_delay) {
          bool isActuallyRec = bigbro.isRec();
          last_check = t;
          if (shouldRec && !isActuallyRec) currentState = ST_START_REC;
          else if (!shouldRec && isActuallyRec) currentState = ST_STOP_REC;
        } 
        else currentState = alone ? ST_STANDBY : ST_WAIT_CAN;
      }
      break;

    case ST_PWR_CAM_ON:
      bigbro.cameraON();
      currentState = alone ? ST_START_REC : ST_WAIT_CAN;
      break;

    case ST_START_REC:
      { bool state = bigbro.isRec();  // brackets needed to define "state" scope

        if (!state && try_rec < 10) {
          cam.toggleRecording();
          try_rec++;
        } else if (state) {
          bigbro.blinkLED(2, 100);
          shouldRec = true;
          try_rec = 0;

          if (isFirstToggle) {
            toggle_time = millis();  // Save initial record start time
            last_check = toggle_time;
          }
          isFirstToggle = false;
          currentState = alone ? ST_STANDBY : ST_WAIT_CAN;
        } else {
          currentState = ST_FAIL;
        }
      }
      break;

    case ST_STOP_REC:
      { bool state = bigbro.isRec();

        if (state && try_rec < 10) {
          cam.toggleRecording();
          try_rec++;
        } else if (!state) {
          bigbro.blinkLED(2, 100);
          shouldRec = false;     // Update expected state for verifications
          isFirstToggle = true;  // Reinitialize toggle flag
          try_rec = 0;
          currentState = alone ? ST_PWR_CAM_OFF : ST_WAIT_CAN;
        } else {
          currentState = ST_FAIL;
        }
      }
      break;

    case ST_PWR_CAM_OFF:
      bigbro.cameraOFF();
      currentState = ST_FIN;
      break;

    case ST_FAIL:
      try_rec = 0;
      bigbro.cameraOFF();
      delay(500);
      currentState = ST_PWR_CAM_ON;
      break;

    case ST_FIN:
      if (!end_done) {
        bigbro.end();
        end_done = true;
      }
      break;
  }
}