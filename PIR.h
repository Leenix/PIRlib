#ifndef PIRLIB_H
#define PIRLIB_H

#include "Arduino.h"

/**
* PIR Library
*
* Author: Leenix
* Date: 2017-03-06
*
* A library for PIR motion sensors with options to manage software cooldowns and event callbacks.
*/

typedef void (*event_callback)(void); /**< Callback function structure - must have no parameters. */

class PIR {
   public:
    /**
    * Create a new PIR sensor object
    * @param pin Number of the pin that the sensor's data line is connected to
    * @param cooldown Cooldown of the sensor in ms
    * @param height Mounting height of the sensor in metres
    * @param fov Viewing angle of the sensor in degrees
    */
    PIR(int pin, unsigned long cooldown = DEFAULT_COOLDOWN, float height = 0, int fov = 0);

    /**
    * Initialise the sensor for use.
    * Runtime variables are reset.
    */
    void start();

    /**
    * Reset the runtime variables and counters of the sensor
    * Does not need recalibration after reset.
    * Sensor power is not toggled as part of the reset and changes are software based only.
    */
    void reset();

    /**
    * Leave the sensor undisturbed to allow it to calibrate.
    * @param calibration_time Amount of time given to the sensor for calibration in ms
    */
    void calibrate(long calibration_time = DEFAULT_CALIBRATION_TIME);

    /**
    * Check the sensor for any new events
    * Should be called regularly (~100ms) as to not miss events and to maintain timely information.
    * Callbacks are triggered from this function.
    */
    void update();

    /**
    * Get the period of time that the current event has been active for
    * @return Length of time of the current event in ms
    */
    long get_current_event_time();

    /**
    * Assign a function to be called at the start of new events
    * @param callback Function to be called
    */
    void set_event_start_callback(event_callback callback);

    /**
    * Assign a function to be called when events finish
    * @param callback Function to be called
    */
    void set_event_end_callback(event_callback callback);

    /**
    * Assign a function to be called when the sensor cooldown expires
    * @param callback Function to be called
    */
    void set_cooldown_over_callback(event_callback callback);

    /**
    * Set the active trigger state of the sensor
    * @param trigger_state HIGH if the sensor provides an active HIGH when detecting objects
    */
    void set_trigger_state(bool trigger_state);

    /**
    * Set the trigger mode of the sensor
    * @param trigger_mode {PIR_REPEATING | PIR_DISCRETE}
    *
    * Trigger mode of the sensor.
    * PIR_REPEATING - sensor records repeat events as long as the trigger state remains high.
    * PIR_DISCRETE - The sensor trigger state must go low before recording a new detection event.
    */
    void set_trigger_mode(int trigger_mode);

    bool state; /**< Detection state of the sensor; HIGH if a detection event is currently taking place*/
    unsigned long num_detections;       /**< The number of detections recorded by the sensor */
    unsigned long detection_start_time; /**< Start time of the latest detection event (in ms since program start time)*/
    unsigned long last_untriggered_time; /**< Last time that the sensor state was LOW (in ms since program start time)*/
    unsigned long event_width;           /**< Amount of time that the sensor state has remained HIGH in ms*/
    unsigned long cooldown_time;         /**< The minimum amount of time required between detections in ms*/
    float radius;                        /**< Radius of the sensor's detection area in metres*/
    bool is_in_cooldown;                 /**< Flag to indicate whether or not the sensor is in software cooldown */

    /**
    * Trigger mode of the sensor.
    * PIR_REPEATING - sensor records repeat events as long as the trigger state remains high.
    * PIR_DISCRETE - The sensor trigger state must go low before recording a new detection event.
    * PIR_DISCRETE_NO_COOLDOWN - Trigger state must go low, but cooldowns are ignored
    */
    enum TRIGGER_MODES { PIR_REPEATING = 0, PIR_DISCRETE = 1, PIR_DISCRETE_NO_COOLDOWN = 2 };

   private:
    const static long DEFAULT_COOLDOWN = 5000; /**< Default cooldown period of the sensor in ms */
    const static long DEFAULT_CALIBRATION_TIME =
        10000; /**< Default time the sensor must be left alone to calibrate in ms */
    const static bool DEFAULT_MOTION_TRIGGER_STATE = HIGH; /**< Default trigger state of the sensor */

    int _trigger_mode;             /**< Current trigger mode of the sensor (See: TRIGGER_MODES)*/
    event_callback _event_start;   /**< Function to call when a detection event starts */
    event_callback _event_end;     /**< Function to call when the trigger state falls LOW after a detection event */
    event_callback _cooldown_over; /**< Function to call when the cooldown expires after a detection event */
    int _pin;                      /**< The data pin assignment of the sensor */
    float _height;                 /**< Mounting height of the sensor in metres */
    int _fov;                      /**< Field of view of the sensor in degrees */
    bool _trigger_state; /**< The state of the data pin that indicates that motion has been detected (usually HIGH)*/
};
#endif
