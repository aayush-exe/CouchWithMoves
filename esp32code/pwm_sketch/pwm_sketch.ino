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

double ppmMin = 1000, ppmMax = 2000;
double ppmMid = (ppmMax+ppmMid)/2;

double wiimMin = 0, wiimMax = 52;
double wiimMid = (wiimMid + wiimMax)/2;
double wiimDead = 3; // deadzone of one side, ex. 3 = 3 on each side, not 3 total
double wiiAccel;

Servo Lmotor;
Servo Rmotor;

const uint8_t Lmotor_pin = 5, Rmotor_pin = 4;

double Mthrottle = 1500, Lthrottle = 1500, Rthrottle = 1500;
double accelRate = 100, decelRate = 200; // per second
long previousMillis;

double maxTurnRatio = 0.1; // when turning, one side spins at Mthrottle, second spins up to Mthrottle*maxTurnRatio

bool accel, brake, safety, boost;

void update_wiimote()
{
    wiimote.task();
    num_run++;

    if (wiimote.available() > 0) 
    {
        ButtonState  button  = wiimote.getButtonState();
        AccelState   accel   = wiimote.getAccelState();
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

      
            Serial.printf(", wiimote.axis: %3d/%3d/%3d", accel.xAxis, accel.yAxis, accel.zAxis);
            wiiAccel = accel.xAxis;
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
  Lmotor.attach(Lmotor_pin);
  Rmotor.attach(Rmotor_pin);

  Serial.println("Ready");
  //vesc1.writeMicroseconds(1500);

}


void loop() {
  update_wiimote();

  if (safety && wiimote.available() > 0){
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

  if (wiiAccel < wiimMid-wiimDead || wiiAccel > wiimMid+wiimDead){
    if (wiiAccel < wiimMin) wiiAccel = wiimMin;
    if (wiiAccel > wiimMax) wiiAccel = wiimMax;

    double steeringValue;

    if (wiiAccel < wiimMid) {
        // turning left
        steeringValue = -1 + (wiiAccel - wiimMin) / (wiimMid - wiimMin);
    } else {
        // turning right
        steeringValue = (wiiAccel - wiimMid) / (wiimMax - wiimMid);
    }

    double mult_ratio = maxTurnRatio * steeringValue;

    Lthrottle = 
  }
  else{
    Lthrottle = Mthrottle;
    Rthrottle = Mthrottle;
  }

  Serial.printf("throttle L|R: %.1f|%.1f", Lthrottle, Rthrottle);
  //Lmotor.writeMicroseconds(Lthrottle);
  //Rmotor.writeMicroseconds(Rthrottle);

  delay(10); // wiimote limited to 100hz

}