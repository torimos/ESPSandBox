#include <Arduino.h>
#include <ESP.h>
#include <algorithm>    // std::min

// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0
// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT  13
// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     5000
// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN            2

void led_init()
{
    ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);
}

void led_write(uint32_t value) 
{
 // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / 255) * std::min(value, (uint32_t)255);
  // write duty to LEDC
  ledcWrite(LEDC_CHANNEL_0, duty);
}