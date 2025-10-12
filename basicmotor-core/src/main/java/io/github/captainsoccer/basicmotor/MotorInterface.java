package io.github.captainsoccer.basicmotor;

import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;

public abstract class MotorInterface {

    public final String name;

    protected MotorInterface(String name) {
        this.name = name;
    }

    /**
     * Gets the default measurements for the motor.
     * This is used to get the original source of measurements for the motor.
     *
     * @return The default measurements for the motor.
     */
    public abstract Measurements getDefaultMeasurements();

    /**
     * Gets the loop time of the internal PID loop.
     * This is used to convert the pid gains to the motor controller's loop time.
     *
     * @return The loop time of the internal PID loop in seconds.
     */
    public abstract double getInternalPIDLoopTime();

    /**
     * Sets if the motor should be inverted.
     * The motor's default positive direction is counter-clockwise.
     * This can be changed by setting this function to true.
     *
     * @param inverted If the motor should be inverted (true for inverted, false for normal).
     */
    public abstract void setInverted(boolean inverted);

    /**
     * Sets the idle mode of the motor.
     *
     * @param mode The idle mode to set the motor to.
     */
    public abstract void setIdleMode(BasicMotor.IdleMode mode);

    /**
     * Sends the PID gains to the motor controller.
     * This is used to update the PID gains of the motor controller.
     * These PID gains sent should be in motor units (in volts).
     *
     * @param pidGains The PID gains to set on the motor controller.
     * @param slot     The slot to set the PID gains to (0, 1, or 2).
     */
    public abstract void updatePIDGainsToMotor(PIDGains pidGains, int slot, MotorManager.ControllerLocation location);

    /**
     * Sets the constraints of the motor controller.
     * This is used to update the constraints of the motor controller.
     * The constraints sent should be in motor units.
     *
     * @param constraints The constraints to set on the motor controller.
     */
    public abstract void updateConstraintsGainsToMotor(ConstraintsGains constraints);
}
