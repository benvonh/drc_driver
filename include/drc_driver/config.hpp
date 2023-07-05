#ifndef __CONFIG_HPP__
#define __CONFIG_HPP__

// Pin for controlling speed
#define SPEED_PIN 12
// Pin for controlling steering
#define STEER_PIN 13

/**
 * WARNING: Do not set the minimum pulsewidth
 * below 500 and maximum pulsewidth above 2500.
 */

// Minimum pulsewidth in microseconds
#define PULSE_MIN 1000
// Maximum pulsewidth in microseconds
#define PULSE_MAX 2000

// Uncomment for debug mode
//#define DEBUG

#endif/*__CONFIG_HPP__*/
