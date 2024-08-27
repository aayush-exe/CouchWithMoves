#include <ESP32Servo.h>
#include "ESP32Wiimote.h"

// Controls idea?

// Hold back button to do anything, if released auto brake
// Hold 2 to go forward, speed ramps up gradually
// Hold 1 to brake slowly (releasing back button brakes hard for safety)

// Hold A button for boost
// Left/Right D pad is turn signals

// +/- buttons, home button, and up/down D pad left empty (could be used for gimmicks)

ESP32Wiimote wiimote;

static bool logging = true;
static long last_ms = 0;
static int num_run = 0, num_updates = 0;

const double ppmMin = 1000, ppmMax = 2000, ppmMid = 1500;

const double wiimMin = 102, wiimMax = 153, wiimMid = 127.5, wiimDead = 3; // deadzone of one side, ex. 3 = 3 on each side, not 3 total
double wiiPos;

Servo Lmotor;
Servo Rmotor;

const uint8_t Lmotor_pin = 5, Rmotor_pin = 4;

double Mthrottle = 1500, Lthrottle = 1500, Rthrottle = 1500;
const double accelRate = 100, decelRate = 200; // per second
long previousMillis;
bool wiimoteAvailable = false;

double maxTurnRatio = 0.1; // when turning, one side spins at Mthrottle, second spins up to Mthrottle*maxTurnRatio

bool accel, brake, safety, boost, reverse;

void update_wiimote()
{
    wiimote.task();
    num_run++;
    wiimoteAvailable = wiimote.available() > 0;

    if (wiimoteAvailable) 
    {
        ButtonState  button  = wiimote.getButtonState();
        AccelState   cur_accel   = wiimote.getAccelState();
        NunchukState nunchuk = wiimote.getNunchukState();

        num_updates++;
        if (logging)
        {
            bool ca     = (button & BUTTON_A);
            bool cb     = (button & BUTTON_B);
            bool cc     = (button & BUTTON_C);
            bool cz     = (button & BUTTON_Z);
            bool c1     = (button & BUTTON_ONE);
            bool c2     = (button & BUTTON_TWO);
            bool cminus = (button & BUTTON_MINUS);
            bool cplus  = (button & BUTTON_PLUS);
            bool chome  = (button & BUTTON_HOME);
            bool cleft  = (button & BUTTON_LEFT);
            bool cright = (button & BUTTON_RIGHT);
            bool cup    = (button & BUTTON_UP);
            bool cdown  = (button & BUTTON_DOWN);

            accel = c2;
            brake = c1;
            safety = cb;
            boost = ca;

      
            // Serial.printf(", wiimote.axis: %3d/%3d/%3d", cur_accel.xAxis, cur_accel.yAxis, cur_accel.zAxis);
            wiiPos = cur_accel.yAxis;
            // Serial.printf("NEW: %.1f\n", wiiPos);
        }
    }

    if (! logging)
    {
        long ms = millis();
        if (ms - last_ms >= 1000)
        {
            Serial.printf("Run %d times per second with %d updates\n", num_run, num_updates);
            num_run = num_updates = 0;
            last_ms += 1000;
        }
    }
}

void setup()
{
  Serial.begin(115200);

  Serial.println("Initializing Wiimote");
  wiimote.init();
  if (! logging)
      wiimote.addFilter(ACTION_IGNORE, FILTER_ACCEL); // optional
  
  Serial.println("Wiimote Started");
  last_ms = millis();

  Serial.println("Starting PPM");
  // Lmotor.attach(Lmotor_pin);
  // Rmotor.attach(Rmotor_pin);

  Serial.println("Ready");
  //vesc1.writeMicroseconds(1500);

}


void loop() {
  
  update_wiimote();

    // standard forward/back controls
    // change acceleration values at the top
    if (safety){
      if (brake){
        Mthrottle -= decelRate / 100;
        if (Mthrottle<ppmMid) Mthrottle = ppmMid;
      }
      else if (accel){
        Mthrottle += (accelRate + (boost ? accelRate : 0))/100;
        if (Mthrottle>ppmMax) Mthrottle=ppmMax;
      }
    }
    else{
      Mthrottle = 1500;
    }


    double steeringValue;
    if (wiiPos < 1){
      Serial.println("Wiimote doing something bad");
      Mthrottle = 1500;
      Lthrottle = 1500;
      Rthrottle = 1500;
    }


    else{
      Serial.printf("wiiPos: %.1f\n", wiiPos);
      if (wiiPos < wiimMid-wiimDead || wiiPos > wiimMid+wiimDead){
          if (wiiPos < wiimMin) wiiPos = wiimMin;
          if (wiiPos > wiimMax) wiiPos = wiimMax;
          bool isLeft;

          if (wiiPos > wiimMid+wiimDead) {
            isLeft = true;
            // Steering left
            // maxTurnRatio < steeringValue < 1 (percent)
            // 1 = turn the slower side at 100% power, lower it is, slower the other side spins
            // If closer to the resting value, output 1; if closer to max turn, output maxTurnRatio

            // all the way = 0
            // none = 1
            // wiimMax = 152
            // wiimMin = 102
            // wiimMid = 127.5
            float ratio = 1 - (wiiPos - wiimMid) / (wiimMid - wiimMin);
            steeringValue = ratio;
          } 
          else if (wiiPos < wiimMid-wiimDead){
            isLeft = false;
            // Steering right
            // If closer to the resting value, output 1; if closer to max turn, output maxTurnRatio
            float ratio = 1 + (wiiPos - wiimMid) / (wiimMax - wiimMid);
            steeringValue = ratio;
          }

          
          double small_throttle = (fabs(steeringValue) * (Mthrottle - ppmMid))+ppmMid;
          if (small_throttle<ppmMid) small_throttle = ppmMid;

          if (isLeft){
            Lthrottle = small_throttle;
            Rthrottle = Mthrottle;
          }
          else{
            Lthrottle = Mthrottle;
            Rthrottle = small_throttle;
          }
        }
        else{
          steeringValue = 1;
          Lthrottle = Mthrottle;
          Rthrottle = Mthrottle;
        }
      }

    Serial.printf("throttle L|R: %.1f|%.1f \n", Lthrottle, Rthrottle);
    // Serial.printf("steeringValue = %.3f \n", steeringValue);
    // Serial.println(wiiPos);
    //Lmotor.writeMicroseconds(Lthrottle);
    //Rmotor.writeMicroseconds(Rthrottle);

  delay(10); // wiimote limited to 100hz

}