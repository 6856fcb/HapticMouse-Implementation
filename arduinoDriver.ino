#include <Arduino.h>
#include "driver/ledc.h"

// ---- Pins ----
const int PIN_PWM   = 25;   // Q1 base (Drive)  -> 330–470Ω
const int PIN_BRAKE = 27;   // Q2 base (Brake)  -> 820Ω–1k + 10k pulldown to M-

// ---- LEDC ----
#if defined(LEDC_LOW_SPEED_MODE)
  #define LEDC_MODE_USED LEDC_LOW_SPEED_MODE
#else
  #define LEDC_MODE_USED LEDC_HIGH_SPEED_MODE
#endif
const ledc_timer_t     LEDC_TIMER   = LEDC_TIMER_0;
const ledc_channel_t   LEDC_CHANNEL = LEDC_CHANNEL_0;
const uint32_t         PWM_FREQ     = 200;                 // 150–250 Hz feels good for ERM
const ledc_timer_bit_t PWM_RES      = LEDC_TIMER_8_BIT;    // duty 0..255

// ---- Behavior ----
const uint8_t  MAX_DUTY           = 200;   // fixed cap (no SET)
bool           defaultBrakeAtEnd  = true;  // MODE BRAKE/COAST
const uint16_t HARD_STOP_MS       = 40;    // STOP brake duration
const uint16_t MIN_CMD_GAP_MS     = 150; 

// Click guard: ignore STOP for a short time after a click so clicks are felt
static uint32_t clickGuardUntil = 0;       // millis timestamp until which STOP is ignored

// ---- Engine state machine ----
enum Action : uint8_t { ACT_DRIVE, ACT_COAST, ACT_BRAKE };
struct Step { Action act; uint8_t duty; uint16_t dur_ms; };

static const uint8_t MAX_STEPS = 24;
Step steps[MAX_STEPS];
uint8_t  stepCount = 0;
uint8_t  stepIndex = 0;
bool     engineActive = false;
uint32_t stepDeadlineMs = 0;

// STOP state (nonblocking)
bool     hardStopActive = false;
uint32_t hardStopReleaseAt = 0;

// command dedupe
// add to enums
enum Cmd   { CMD_NONE, CMD_P1, CMD_P2, CMD_P3, CMD_P4, CMD_P5, CMD_P6 };
enum CmdId { ID_NONE,  ID_P1,  ID_P2,  ID_P3,  ID_P4,  ID_P5,  ID_P6  };

Cmd      lastStartedCmd = CMD_NONE;
uint32_t lastCmdStartMs = 0;

// for parser
String rx;

// ---- Low-level motor ----
static void pwmInit(){
  ledc_timer_config_t t = {};
  t.speed_mode = LEDC_MODE_USED;
  t.timer_num = LEDC_TIMER;
  t.duty_resolution = PWM_RES;
  t.freq_hz = PWM_FREQ;
  t.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&t);

  ledc_channel_config_t c = {};
  c.gpio_num = PIN_PWM;
  c.speed_mode = LEDC_MODE_USED;
  c.channel = LEDC_CHANNEL;
  c.intr_type = LEDC_INTR_DISABLE;
  c.timer_sel = LEDC_TIMER;
  c.duty = 0;
  c.hpoint = 0;
  ledc_channel_config(&c);
}
inline void pwmWrite(uint8_t d){ if(d>MAX_DUTY) d=MAX_DUTY; ledc_set_duty(LEDC_MODE_USED,LEDC_CHANNEL,d); ledc_update_duty(LEDC_MODE_USED,LEDC_CHANNEL); }
inline void brakeOn()  { pwmWrite(0); digitalWrite(PIN_BRAKE, HIGH); }
inline void brakeOff() { digitalWrite(PIN_BRAKE, LOW); }
inline void driveDuty(uint8_t d){ brakeOff(); pwmWrite(d); }
inline void coast(){ pwmWrite(0); brakeOff(); }

// ---- Engine helpers ----
inline void engineClear(){ engineActive=false; stepCount=stepIndex=0; stepDeadlineMs=0; }
inline void engineApply(const Step& s){
  switch(s.act){
    case ACT_DRIVE: driveDuty(s.duty); break;
    case ACT_COAST: coast();           break;
    case ACT_BRAKE: brakeOn();         break;
  }
}

void engineStart(const Step* src, uint8_t n, Cmd tag){
  uint32_t now = millis();
  if(tag!=CMD_NONE && tag==lastStartedCmd && (now - lastCmdStartMs) < MIN_CMD_GAP_MS) {
    return; // de-dupe spam (same pattern too fast)
  }
  lastStartedCmd = tag;
  lastCmdStartMs = now;

  // Preempt immediately
  engineClear();
  hardStopActive = false;

  stepCount = min<uint8_t>(n, MAX_STEPS);
  for(uint8_t i=0;i<stepCount;i++) steps[i] = src[i];
  stepIndex = 0;

  if(stepCount){
    engineActive = true;
    engineApply(steps[0]);
    stepDeadlineMs = now + steps[0].dur_ms;
  } else {
    coast();
  }
}

void engineAbortAndBrake(uint16_t ms){
  engineClear();
  brakeOn();
  hardStopActive = true;
  hardStopReleaseAt = millis() + ms;
}

// ---- Short patterns ----
// P1: single tight tick ~55ms + brief brake 15ms
uint8_t buildP1(Step*out){
  uint8_t k=0;
  out[k++] = {ACT_DRIVE, MAX_DUTY, 55};
  if(defaultBrakeAtEnd) out[k++] = {ACT_BRAKE, 0, 15};
  else                  out[k++] = {ACT_COAST, 0, 1};
  return k;
}
// P2: double micro-tick: 35ms on, 70ms off, 35ms on, brake 15ms (~120ms total)
uint8_t buildP2(Step*out){
  uint8_t k=0;
  out[k++] = {ACT_DRIVE, MAX_DUTY, 35};
  out[k++] = {ACT_COAST, 0,       70};
  out[k++] = {ACT_DRIVE, MAX_DUTY, 35};
  if(defaultBrakeAtEnd) out[k++] = {ACT_BRAKE, 0, 15};
  else                  out[k++] = {ACT_COAST, 0, 1};
  return k;
}
// P3: sharp buzzlet ~70ms + brake 20ms
uint8_t buildP3(Step*out){
  uint8_t k=0;
  out[k++] = {ACT_DRIVE, MAX_DUTY, 70};
  if(defaultBrakeAtEnd) out[k++] = {ACT_BRAKE, 0, 20};
  else                  out[k++] = {ACT_COAST, 0, 1};
  return k;
}

// ---- Longer/Harder patterns (P4–P6) ----
// P4: firm pulse ~120ms + brake 25ms
uint8_t buildP4(Step*out){
  uint8_t k=0;
  out[k++] = {ACT_DRIVE, MAX_DUTY, 120};
  if(defaultBrakeAtEnd) out[k++] = {ACT_BRAKE, 0, 25};
  else                  out[k++] = {ACT_COAST, 0, 1};
  return k;
}
// P5: long pulse ~200ms + brake 30ms
uint8_t buildP5(Step*out){
  uint8_t k=0;
  out[k++] = {ACT_DRIVE, MAX_DUTY, 200};
  if(defaultBrakeAtEnd) out[k++] = {ACT_BRAKE, 0, 30};
  else                  out[k++] = {ACT_COAST, 0, 1};
  return k;
}
// P6: ramp + hold (smooth start, then ~100ms hold) + brake 30ms
uint8_t buildP6(Step*out){
  uint8_t k=0;
  const uint8_t startDuty = 120;                 // softer engage
  for (uint8_t d = startDuty; d < MAX_DUTY && k < MAX_STEPS-3; d = (uint8_t)min<int>(MAX_DUTY, d + 16)) {
    out[k++] = {ACT_DRIVE, d, 20};               // short ramp steps
  }
  out[k++] = {ACT_DRIVE, MAX_DUTY, 100};         // hold to get ERM spinning
  if(defaultBrakeAtEnd) out[k++] = {ACT_BRAKE, 0, 30};
  else                  out[k++] = {ACT_COAST, 0, 1};
  return k;
}

// Click: crisp 50ms + brake 20ms (stronger feel)
uint8_t buildClick(Step*out){
  uint8_t k=0;
  uint8_t d = (MAX_DUTY<190)?MAX_DUTY:190;
  out[k++] = {ACT_DRIVE, d, 50};
  if(defaultBrakeAtEnd) out[k++] = {ACT_BRAKE, 0, 20};
  else                  out[k++] = {ACT_COAST, 0, 1};
  return k;
}

void engineTick(){
  uint32_t now = millis();

  // Release STOP brake when its time elapses
  if(hardStopActive && (int32_t)(now - hardStopReleaseAt) >= 0){
    hardStopActive = false;
    brakeOff();
  }

  if(!engineActive) return;
  if((int32_t)(now - stepDeadlineMs) < 0) return;  // still in current step

  // Next step
  stepIndex++;
  if(stepIndex >= stepCount){
    engineClear();
    if(!defaultBrakeAtEnd) coast();
    return;
  }
  engineApply(steps[stepIndex]);
  stepDeadlineMs = now + steps[stepIndex].dur_ms;
}

// ---- Serial commands ----


void startPattern(CmdId id){
  Step tmp[MAX_STEPS]; uint8_t n=0; Cmd tag=CMD_NONE;
  if(id==ID_P1){ n=buildP1(tmp); tag=CMD_P1; }
  else if(id==ID_P2){ n=buildP2(tmp); tag=CMD_P2; }
  else if(id==ID_P3){ n=buildP3(tmp); tag=CMD_P3; }
  else if(id==ID_P4){ n=buildP4(tmp); tag=CMD_P4; }
  else if(id==ID_P5){ n=buildP5(tmp); tag=CMD_P5; }
  else if(id==ID_P6){ n=buildP6(tmp); tag=CMD_P6; }

  if(n) engineStart(tmp, n, tag);
}

void handleClick(const String& args){
  int sp = args.indexOf(' ');
  String st = (sp==-1) ? "" : args.substring(sp+1);
  st.trim(); st.toLowerCase();

  if(st == "down"){
    Step tmp[6]; uint8_t n = buildClick(tmp);
    engineStart(tmp, n, CMD_NONE);     // PREEMPT anything
    clickGuardUntil = millis() + 120;  // ignore STOP for 120 ms
    Serial.println(F("ACK CLICK down"));
  } else if (st == "up"){
    Serial.println(F("ACK CLICK up"));
  } else {
    Serial.println(F("ERR CLICK"));
  }
}

void printHelp(){
  Serial.println(F("OK HELP"));
  Serial.println(F("  P1 | P2 | P3 (short patterns)"));
  Serial.println(F("  CLICK <left|right|middle|xbutton> <down|up>"));
  Serial.println(F("  STOP  (immediate, respects click guard)"));
  Serial.println(F("  MODE BRAKE | MODE COAST"));
  Serial.println(F("  BRAKE <ms>"));
}

void handleLine(const String& line){
  if(!line.length()) return;
  String s = line; s.trim();
  int sp = s.indexOf(' ');
  String op   = (sp==-1) ? s : s.substring(0,sp);
  String rest = (sp==-1) ? "" : s.substring(sp+1);
  op.toUpperCase();

  if(op == "P1") startPattern(ID_P1);
  else if(op == "P2") startPattern(ID_P2);
  else if(op == "P3") startPattern(ID_P3);
  else if(op == "P4") startPattern(ID_P4);
  else if(op == "P5") startPattern(ID_P5);
  else if(op == "P6") startPattern(ID_P6);
  else if(op == "CLICK") handleClick(rest);
  else if(op == "STOP"){
    if((int32_t)(millis() - clickGuardUntil) < 0){
      Serial.println(F("IGN STOP (click guard)"));
    } else {
      engineAbortAndBrake(HARD_STOP_MS);
      Serial.println(F("ACK STOP"));
    }
  }
  else if(op == "MODE"){
    rest.trim(); rest.toUpperCase();
    if      (rest == "BRAKE") { defaultBrakeAtEnd = true;  Serial.println(F("OK MODE BRAKE")); }
    else if (rest == "COAST") { defaultBrakeAtEnd = false; Serial.println(F("OK MODE COAST")); }
    else                      { Serial.println(F("ERR MODE")); }
  }
  else if(op == "BRAKE"){
    uint16_t ms = (uint16_t)rest.toInt();
    engineAbortAndBrake(ms);
    Serial.printf("OK BRAKE %u\n", ms);
  }
  else if(op == "HELP" || op == "?") printHelp();
  else { Serial.print(F("ERR Unknown: ")); Serial.println(s); }
}

void setup(){
  Serial.begin(115200);
  delay(20);
  pinMode(PIN_BRAKE, OUTPUT);
  brakeOff();
  pwmInit();
  coast();
  Serial.println(F("READY (nonblocking, short patterns, click-preempt, STOP-guard)."));
}

void loop(){
  // Parse serial quickly
  while(Serial.available()){
    char c = (char)Serial.read();
    if(c=='\r') continue;
    if(c=='\n'){ rx.trim(); handleLine(rx); rx=""; }
    else { rx += c; if(rx.length()>120) rx.remove(0, rx.length()-120); }
  }
  // Advance engine
  engineTick();
  delay(1);
}
