package io.github.captainsoccer.basicmotor.control;

import edu.wpi.first.math.MathUtil;
import io.github.captainsoccer.basicmotor.config.ImmutableBasicMotorConfig;
import io.github.captainsoccer.basicmotor.config.ImmutableConstraintsConfig;
import io.github.captainsoccer.basicmotor.measurements.Measurements;

/**
 * A class representing the constraints of the motor and houses the config and the calculations
 */
public class Constraints {
    public final ImmutableConstraintsConfig constraints;

    public Constraints(ImmutableBasicMotorConfig config){
        this.constraints = config.constraints;
    }

    public void calculate(Measurements.Measurement measurement, ControlFrame controlFrame) {
        switch (constraints.constraint.type()) {
            case NONE -> nothing();
            case LIMITED -> calculateLimited(measurement, controlFrame);
            case CONTINUOUS -> calculateContinuous(measurement, controlFrame);
        }
    }

    private void calculateLimited(Measurements.Measurement measurement, ControlFrame controlFrame) {
        if (controlFrame.controlMode() == ControlMode.PROFILED_VELOCITY) {
            // Profiled velocity does not support soft limits, so we return
            return;
        }

        var controlMode = controlFrame.controlMode();
        // check if the request is a position control (then apply the limits to the setpoint)
        if (controlMode.isPositionControl()) {
            // check if the request is in the limits of the motor
            if (controlFrame.goal().position >= constraints.constraint.maxValue()) {
                controlFrame.goal().position = constraints.constraint.maxValue();
                //resets the velocity to make sure a motion profile doesn't continue.
                controlFrame.goal().velocity = 0;
            }
            if (controlFrame.goal().position <= constraints.constraint.minValue()) {
                controlFrame.goal().position = constraints.constraint.minValue();
                //resets the velocity to make sure a motion profile doesn't continue.
                controlFrame.goal().velocity = 0;
            }
        }

        // if not position control, that means the goal set is the direction of the motor,
        // then check if the measurement is in the limits of the motor
        // and make sure the direction is back to the zone
        else {
            //if below soft limit and moving backwards, set to zero
            if (measurement.position() <= constraints.constraint.minValue() && controlFrame.goal().position < 0) {
                controlFrame.goal().position = 0;
            }
            //if above soft limit and moving forwards, set to zero
            if (measurement.position() >= constraints.constraint.maxValue() && controlFrame.goal().position > 0) {
                controlFrame.goal().position = 0;
            }
        }
    }

    private void calculateContinuous(Measurements.Measurement measurement, ControlFrame controlFrame) {
        // check if the request is a position control (continuous constraints only work for position
        // control)
        if (!controlFrame.controlMode().isPositionControl()) return;

        double errorBound = (constraints.constraint.maxValue() - constraints.constraint.minValue()) / 2.0;

        double originalPosition = controlFrame.goal().position;

        // wrap the goal around the limits
        controlFrame.goal().position =
                MathUtil.inputModulus(
                        controlFrame.goal().position - measurement.position(), -errorBound, errorBound)
                        + measurement.position();

        // if the goal is in the opposite direction of the original position, reverse the velocity
        // only affects motion profiles
        if (Math.signum(controlFrame.goal().position - measurement.position())
                != Math.signum(originalPosition - measurement.position())) {
            controlFrame.goal().velocity *= -1;
        }
    }

    private void nothing() {

    }
}
