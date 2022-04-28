#include <Arduino.h>
#include <stepper.h>

stepper_driver::stepper_driver() { }

// test comms and return bool
bool stepper_driver::test_setup() {
    if (tmc.isSetupAndCommunicating()) {
        Serial.println("Stepper driver is setup and communicating!");
        return true;

    }
    else if (tmc.isCommunicatingButNotSetup()) {
        Serial.println("Stepper driver is communicating but not setup!");
        Serial.println("Running setup again...");
        tmc.setup(serial_stream);
        return false;
    }
    else {
        Serial.println("Stepper driver is not communicating!");
        Serial.println("Try turning driver power on to see what happens.");
        return false;
    }
    Serial.println("");
}

void stepper_driver::set_ustep(int u_step) {
    tmc.setMicrostepsPerStep(u_step);
}

void stepper_driver::set_run_current(uint8_t percent) {
    tmc.setRunCurrent(percent);
}

void stepper_driver::set_hold_current(uint8_t percent) {
    tmc.setHoldCurrent(percent);
}

void stepper_driver::connect_motor_pins(int step, int dir, int en) {
    // tell Flexy Stepper what pins to use for step and dir
    stepper.connectToPins(step, dir);

    // set pin for enable() and disable()
    MOTOR_EN_PIN = en;
}

void stepper_driver::enable() {
    tmc.enable(); 
    digitalWrite(MOTOR_EN_PIN, LOW);
}

void stepper_driver::disable() {
    tmc.disable();
    digitalWrite(MOTOR_EN_PIN, HIGH);
}

// Flexy Step functions
void stepper_driver::set_target(float angle, float max) {
    stepper.setTargetPositionInRevolutions(angle * max);
}

void stepper_driver::set_speed(int steps_per_second) {
    stepper.setSpeedInStepsPerSecond(steps_per_second);
}

void stepper_driver::set_accel(int accel) {
    stepper.setAccelerationInStepsPerSecondPerSecond(accel);
    stepper.setDecelerationInStepsPerSecondPerSecond(accel);
}

// set TMC and Flexy Stepper parameters
void stepper_driver::set_motor_parameters(int u_step, int steps_per_rev, int accel, int speed, uint8_t RUN_CURRENT_PERCENT) {
    set_ustep(u_step);
    stepper.setStepsPerRevolution(steps_per_rev * u_step);
    set_run_current(RUN_CURRENT_PERCENT);
    set_hold_current(RUN_CURRENT_PERCENT);
    set_speed(speed);
    set_accel(accel);
}


// print tmc2209 info
void stepper_driver::test_connection() {
    Serial.println("*************************");
    TMC2209::Settings settings = tmc.getSettings();
    Serial.print("settings.is_communicating = ");
    Serial.println(settings.is_communicating);
    Serial.print("settings.is_setup = ");
    Serial.println(settings.is_setup);
    Serial.print("settings.enabled = ");
    Serial.println(settings.enabled);
    Serial.print("settings.microsteps_per_step = ");
    Serial.println(settings.microsteps_per_step);
    Serial.print("settings.inverse_motor_direction_enabled = ");
    Serial.println(settings.inverse_motor_direction_enabled);
    Serial.print("settings.stealth_chop_enabled = ");
    Serial.println(settings.stealth_chop_enabled);
    Serial.print("settings.standstill_mode = ");
    switch (settings.standstill_mode)
    {
        case TMC2209::NORMAL:
        Serial.println("normal");
        break;
        case TMC2209::FREEWHEELING:
        Serial.println("freewheeling");
        break;
        case TMC2209::STRONG_BRAKING:
        Serial.println("strong_braking");
        break;
        case TMC2209::BRAKING:
        Serial.println("braking");
        break;
    }
    Serial.print("settings.irun_percent = ");
    Serial.println(settings.irun_percent);
    Serial.print("settings.irun_register_value = ");
    Serial.println(settings.irun_register_value);
    Serial.print("settings.ihold_percent = ");
    Serial.println(settings.ihold_percent);
    Serial.print("settings.ihold_register_value = ");
    Serial.println(settings.ihold_register_value);
    Serial.print("settings.iholddelay_percent = ");
    Serial.println(settings.iholddelay_percent);
    Serial.print("settings.iholddelay_register_value = ");
    Serial.println(settings.iholddelay_register_value);
    Serial.print("settings.automatic_current_scaling_enabled = ");
    Serial.println(settings.automatic_current_scaling_enabled);
    Serial.print("settings.automatic_gradient_adaptation_enabled = ");
    Serial.println(settings.automatic_gradient_adaptation_enabled);
    Serial.print("settings.pwm_offset = ");
    Serial.println(settings.pwm_offset);
    Serial.print("settings.pwm_gradient = ");
    Serial.println(settings.pwm_gradient);
    Serial.print("settings.cool_step_enabled = ");
    Serial.println(settings.cool_step_enabled);
    Serial.print("settings.analog_current_scaling_enabled = ");
    Serial.println(settings.analog_current_scaling_enabled);
    Serial.print("settings.internal_sense_resistors_enabled = ");
    Serial.println(settings.internal_sense_resistors_enabled);
    Serial.println("");

    bool disabled_by_input_pin = tmc.disabledByInputPin();
    Serial.print("disabled_by_input_pin = ");
    Serial.println(disabled_by_input_pin);
    Serial.println("");

    TMC2209::Status status = tmc.getStatus();
    Serial.print("status.over_temperature_warning = ");
    Serial.println(status.over_temperature_warning);
    Serial.print("status.over_temperature_shutdown = ");
    Serial.println(status.over_temperature_shutdown);
    Serial.print("status.short_to_ground_a = ");
    Serial.println(status.short_to_ground_a);
    Serial.print("status.short_to_ground_b = ");
    Serial.println(status.short_to_ground_b);
    Serial.print("status.low_side_short_a = ");
    Serial.println(status.low_side_short_a);
    Serial.print("status.low_side_short_b = ");
    Serial.println(status.low_side_short_b);
    Serial.print("status.open_load_a = ");
    Serial.println(status.open_load_a);
    Serial.print("status.open_load_b = ");
    Serial.println(status.open_load_b);
    Serial.print("status.over_temperature_120c = ");
    Serial.println(status.over_temperature_120c);
    Serial.print("status.over_temperature_143c = ");
    Serial.println(status.over_temperature_143c);
    Serial.print("status.over_temperature_150c = ");
    Serial.println(status.over_temperature_150c);
    Serial.print("status.over_temperature_157c = ");
    Serial.println(status.over_temperature_157c);
    Serial.print("status.current_scaling = ");
    Serial.println(status.current_scaling);
    Serial.print("status.stealth_chop_mode = ");
    Serial.println(status.stealth_chop_mode);
    Serial.print("status.standstill = ");
    Serial.println(status.standstill);
    Serial.println("");

    Serial.println("*************************");
    Serial.println("");
}

void stepper_driver::print_parameters() {
    Serial.print("getMicrostepsPerStep() = ");
    Serial.println(tmc.getMicrostepsPerStep());

    uint32_t interstep_duration = tmc.getInterstepDuration();
    Serial.print("interstep_duration = ");
    Serial.println(interstep_duration);
    Serial.println("");
}
