#include <FastLED.h>

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

const int LEDPIN_L = 26;
const int LEDPIN_R = 27;

const int LED_NUM = 72;

CRGB ledL[LED_NUM];
CRGB ledR[LED_NUM];

static bool logging = true;
static long last_ms = 0;
static int num_run = 0, num_updates = 0;

const double ppmMin = 1000, ppmMax = 1800, ppmMid = 1500;
const double ppmMaxNormal = 1800;   // cruise limit
const double ppmMaxBoost  = 2000;   // full-boost limit

const double wiimMin = 102, wiimMax = 153, wiimMid = 127.5, wiimDead = 3; // deadzone of one side, ex. 3 = 3 on each side, not 3 total
double wiiPos;
bool isLeft;

Servo Lmotor;
Servo Rmotor;

const uint8_t Lmotor_pin = 5, Rmotor_pin = 4;

double Mthrottle = 1500, Lthrottle = 1500, Rthrottle = 1500;
const double accelRate = 100, decelRate = 200; // per second
long prev_millis, cur_millis;
bool wiimoteAvailable = false;

double maxTurnRatio = 0.1; // when turning, one side spins at Mthrottle, second spins up to Mthrottle*maxTurnRatio

bool accel, brake, safety, boost, reverse, leftb, rightb, leftsignal, rightsignal, brakesignal;

/* ---------- steering cancel thresholds ---------- */
const float RIGHT_ENTER = 113.5;   // must go this far *into* a right turn
const float RIGHT_EXIT  = 122;   // come back past here to cancel

const float LEFT_ENTER  = 141.5;   // must go this far into a left turn
const float LEFT_EXIT   = 133;   // come back past here to cancel

/* ---------- internal state (persists between calls) ---------- */
static bool rightArmed = false;    // true = wheel has gone deep right
static bool leftArmed  = false;    // true = wheel has gone deep left

bool left_detect = false, right_detect = false;

static void ledRainbow(CRGB ledL[])
{
    const uint8_t speed = 1;
    const uint16_t NUM_LEDS = LED_NUM;
    static uint8_t baseHue  = 0;

    baseHue += speed;

    for (uint16_t i = 0; i < NUM_LEDS; ++i)
    {
        // Reverse the order by indexing from end
        uint8_t hue = baseHue + ((NUM_LEDS - 1 - i) * 256 / NUM_LEDS);
        ledL[i]     = CHSV(hue, 255, 255);
    }
}
// static void ledRainbow(CRGB ledL[])
// {
//     const uint8_t speed = 1;
//     const uint16_t NUM_LEDS = LED_NUM;
//     static uint8_t baseHue = 0;
//     baseHue += speed;
//     // Ensure baseHue stays within the range of 0 to 255
//     baseHue %= 256;
    
//     // Cycle through red, white, and blue with smaller bars
//     for (uint16_t i = 0; i < NUM_LEDS; ++i)
//     {
//         // Calculate hue based on the position in the array
//         uint8_t hue = baseHue + ((NUM_LEDS - 1 - i) * 256 / NUM_LEDS);
        
//         // Create smaller bars - each color gets about 28 units instead of 85
//         // This creates more repetitions of the pattern
//         if (hue < 28) {
//             ledL[i] = CHSV(0, 255, 255);  // Red
//         } else if (hue < 56) {
//             ledL[i] = CHSV(160, 255, 255);  // Blue
//         } else if (hue < 84) {
//             ledL[i] = CHSV(0, 0, 255);  // White
//         } else if (hue < 112) {
//             ledL[i] = CHSV(0, 255, 255);  // Red
//         } else if (hue < 140) {
//             ledL[i] = CHSV(160, 255, 255);  // Blue
//         } else if (hue < 168) {
//             ledL[i] = CHSV(0, 0, 255);  // White
//         } else if (hue < 196) {
//             ledL[i] = CHSV(0, 255, 255);  // Red
//         } else if (hue < 224) {
//             ledL[i] = CHSV(160, 255, 255);  // Blue
//         } else {
//             ledL[i] = CHSV(0, 0, 255);  // White
//         }
//     }
// }


static void ledTurnSignal(CRGB ledL[])
{
    const CRGB  SIGNAL_COLOR = CRGB(255, 200, 20);   // amber
    const uint16_t GROW_MS  = 300;
    const uint16_t HOLD_MS  = 300;
    const uint16_t OFF_MS   = 200;

    const uint16_t CYCLE_MS = GROW_MS + HOLD_MS + OFF_MS;
    const uint16_t NUM_LEDS = LED_NUM;

    static uint32_t cycleStart = millis();
    uint32_t now        = millis();
    uint32_t cycleTime  = (now - cycleStart) % CYCLE_MS;

    fill_solid(ledL, NUM_LEDS, CRGB::Black);

    if (cycleTime < GROW_MS) {
        uint16_t lit = (cycleTime * NUM_LEDS) / GROW_MS;
        for (uint16_t i = 0; i <= lit && i < NUM_LEDS; ++i) {
            ledL[i] = SIGNAL_COLOR;   // forward wipe from LED 0 to LED 71
        }
    }
    else if (cycleTime < (GROW_MS + HOLD_MS)) {
        fill_solid(ledL, NUM_LEDS, SIGNAL_COLOR);
    }
    // blackout follows
}


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
            leftb = cup;
            rightb = cdown;

      
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
  last_ms = millis();

  Serial.println("Starting PPM");
  Lmotor.attach(Lmotor_pin);
  Rmotor.attach(Rmotor_pin);

  Serial.println("Starting LEDs");
  FastLED.addLeds<WS2812, LEDPIN_L, GRB>(ledL, LED_NUM);
  FastLED.addLeds<WS2812, LEDPIN_R, GRB>(ledR, LED_NUM);
  FastLED.setBrightness(5);

  Serial.println("Ready");

}

static bool getLeftNewPress()
{
  static bool prev = false;
  bool edge = leftb && !prev;
  prev = leftb;
  return edge;
}

static bool getRightNewPress()
{
  static bool prev = false;
  bool edge = rightb && !prev;
  prev = rightb;
  return edge;
}

static bool getLeftDetect()
{
    static bool prev = false;                 // last sampled value
    bool edge = (!left_detect) && prev;       // falling edge
    prev = left_detect;                       // update history
    return edge;
}

static bool getRightDetect()
{
    static bool prev = false;
    bool edge = (!right_detect) && prev;      // falling edge
    prev = right_detect;
    return edge;
}


void loop() {

  prev_millis = cur_millis;
  cur_millis = millis();
  
  update_wiimote();



  if (safety) {
      /* -------- BRAKE ---------- */
      if (brake) {
          Mthrottle -= decelRate / 100;
          if (Mthrottle < ppmMid) Mthrottle = ppmMid;
      }
      /* -------- ACCEL ---------- */
      else if (accel) {
          double inc    = (accelRate + (boost ? accelRate : 0)) / 100;
          double limit  = boost ? ppmMaxBoost : ppmMaxNormal;

          /*  Only apply the limit if we’re currently below it.
              If we’re already above the “no-boost” limit (1800)
              and the rider lets go of A, we’ll just coast there
              until they slow down.                                  */
          if (Mthrottle < limit) {
              Mthrottle += inc;
              if (Mthrottle > limit) Mthrottle = limit;
          }
      }
  }
  /* ------------- NO SAFETY ------------- */
  else {
      Mthrottle = 1500;               // failsafe idle
  }

    // update lights

    //left signal
    if (getLeftNewPress()) 
    {
      if (leftsignal == true) leftsignal = false;
      else 
      {
        leftsignal = true;
        rightsignal = false;
      }
    }
    // right signal
    if (getRightNewPress())
    {
      if (rightsignal == true) rightsignal = false;
      else
      {
        leftsignal = false;
        rightsignal = true;
      }
    }

    if (getLeftDetect()) leftsignal = false;
    if (getRightDetect()) rightsignal = false;


    // brake light check
    // if (!safety){
    //   leftsignal = false;
    //   rightsignal = false;
    // }
    if (!safety || brake || Mthrottle == 1500) brakesignal = true;
    else brakesignal = false;

    //leftsignal, rightsignal, and brakesignal are now good to go. type shit

    if (leftsignal) ledTurnSignal(ledL);
    else if (brakesignal) fill_solid(ledL, LED_NUM, CRGB::Red);
    else ledRainbow(ledL);

    if (rightsignal) ledTurnSignal(ledR);
    else if (brakesignal) fill_solid(ledR, LED_NUM, CRGB::Red);
    else ledRainbow(ledR);

    // if wiimote not connected, tweak out
    double steeringValue;
    if (wiiPos < 1){
      Serial.println("Wiimote doing something bad");
      Mthrottle = 1500;
      Lthrottle = 1500;
      Rthrottle = 1500;
    }

    // MAIN stuff HERE
    else{

      if (wiiPos < wiimMid-wiimDead || wiiPos > wiimMid+wiimDead){
          if (wiiPos < wiimMin) wiiPos = wiimMin;
          if (wiiPos > wiimMax) wiiPos = wiimMax;
        /*****************   HYSTERESIS - BASED TURN-CANCEL   *****************/
        right_detect = false;          // default: no event this frame
        left_detect  = false;

        /* ---------- RIGHT side ---------- */
        if (!rightArmed) {
            /* We are *not* armed yet – wait until wheel goes deep enough */
            if (wiiPos <= RIGHT_ENTER) rightArmed = true;      // arm it
        }
        else {  /* rightArmed == true */
            /* We *are* armed – wait until wheel comes back past EXIT */
            if (wiiPos >= RIGHT_EXIT) {
                rightArmed  = false;     // disarm
                right_detect = true;     // one-shot pulse – cancel signal
            }
        }

        /* ---------- LEFT side ---------- */
        if (!leftArmed) {
            if (wiiPos >= LEFT_ENTER) leftArmed = true;
        }
        else {  /* leftArmed == true */
            if (wiiPos <= LEFT_EXIT) {
                leftArmed   = false;
                left_detect = true;      // one-shot – cancel signal
            }
        }



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

          double ad_mid = ((Mthrottle - ppmMid) * 0.5) + 1425;

          
          double small_throttle = (fabs(steeringValue) * (Mthrottle - ad_mid))+ad_mid;
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

    // Serial.printf("throttle L|R: %.1f|%.1f \n", Lthrottle, Rthrottle);
    // Serial.print("signals: ");
    // if (leftsignal) Serial.print("L ");
    // if (rightsignal) Serial.print("R ");
    // if (brakesignal) Serial.print("B ");
    // Serial.println("");

    // Serial.printf("steeringValue = %.3f \n", steeringValue);
    // Serial.printf("wiiPos: %.1f\n", wiiPos);

    FastLED.show();
    Lmotor.writeMicroseconds(Lthrottle);
    Rmotor.writeMicroseconds(Rthrottle);

  delay(10); // wiimote limited to 100hz

}
