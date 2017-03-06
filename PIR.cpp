#include "PIR.h"

PIR::PIR(int pin, long cooldown, float height, int fov) {
    /**
    * Create a new PIR sensor object
    * @param pin Number of the pin that the sensor's data line is connected to
    * @param cooldown Cooldown of the sensor in ms
    * @param height Mounting height of the sensor in metres
    * @param fov Viewing angle of the sensor in degrees
    */
    _pin = pin;
    _trigger_mode = PIR_REPEATING;

    if (cooldown_time > 0) {
        cooldown_time = cooldown;
    }

    // Calculate sensor radius
    if (height > 0 && fov > 0) {
        _height = height;
        _fov = fov;
        radius = _height * tan(fov * DEG_TO_RAD);
    } else {
        radius = -1;
    }
}

void PIR::start() {
    /**
    * Initialise the sensor for use.
    * Runtime variables are reset.
    */
    pinMode(_pin, INPUT);
    reset();
}

void PIR::reset() {
    /**
    * Reset the runtime variables and counters of the sensor
    * Does not need recalibration after reset.
    * Sensor power is not toggled as part of the reset and changes are software based only.
    */
    state = false;
    is_in_cooldown = false;
    num_detections = 0;
    detection_start_time = 0;
    last_untriggered_time = millis();
    event_width = 0;
}

void PIR::calibrate(long calibration_time) {
    /**
    * Leave the sensor undisturbed to allow it to calibrate.
    * @param calibration_time Amount of time given to the sensor for calibration in ms
    */
    delay(calibration_time);
}

void PIR::update() {
    /**
    * Check the sensor for any new events
    * Should be called regularly (~100ms) as to not miss events and to maintain timely information.
    * Callbacks are triggered from this function.
    */
    // Check cooldown status
    if (is_in_cooldown) {
        if (millis() - detection_start_time > cooldown_time) {
            is_in_cooldown = false;

            if (_cooldown_over) {
                (*_cooldown_over)();
            }
        }
    }

    // Check for motion if not in cooldown
    if (!is_in_cooldown) {
        // Has the sensor been triggered?
        if (digitalRead(_pin) == _trigger_state) {
            /* Trigger modes:
            * REPEATING: Sensor events will be counted after every cooldown if the sensor stays triggered
            * DISCRETE: The sensor must go low before counting another event
            */
            if (_trigger_mode == PIR_REPEATING || (_trigger_mode == PIR_DISCRETE && !state)) {
                state = true;
                detection_start_time = millis();
                is_in_cooldown = true;
                num_detections++;

                // New event - trigger the event callback
                if (_event_start) {
                    (*_event_start)();
                }
            }
        }

        // Sensor not triggered - end the current event if this is the first low since the detection
        else {
            if (state) {
                event_width = get_current_event_time();
                if (_event_end) {
                    (*_event_end)();
                }
            }
            state = false;
            last_untriggered_time = millis();
        }
    }
}

long PIR::get_current_event_time() {
    /**
    * Get the period of time that the current event has been active for
    * @return Length of time of the current event in ms
    */
    return millis() - last_untriggered_time;
}

void PIR::set_event_start_callback(event_callback callback) {
    /**
    * Assign a function to be called at the start of new events
    * @param callback Function to be called
    */
    _event_start = callback;
}

void PIR::set_event_end_callback(event_callback callback) {
    /**
    * Assign a function to be called when events finish
    * @param callback Function to be called
    */
    _event_end = callback;
}

void PIR::set_cooldown_over_callback(event_callback callback) {
    /**
    * Assign a function to be called when the sensor cooldown expires
    * @param callback Function to be called
    */
    _cooldown_over = callback;
}

void PIR::set_trigger_state(bool trigger_state) {
    /**
    * Set the active trigger state of the sensor
    * @param trigger_state HIGH if the sensor provides an active HIGH when detecting objects
    */
    _trigger_state = trigger_state;
}

void PIR::set_trigger_mode(bool trigger_mode) {
    /**
    * Set the trigger mode of the sensor
    * @param trigger_mode {PIR_REPEATING | PIR_DISCRETE}
    *
    * Trigger mode of the sensor.
    * PIR_REPEATING - sensor records repeat events as long as the trigger state remains high.
    * PIR_DISCRETE - The sensor trigger state must go low before recording a new detection event.
    */
    _trigger_mode = trigger_mode;
}
