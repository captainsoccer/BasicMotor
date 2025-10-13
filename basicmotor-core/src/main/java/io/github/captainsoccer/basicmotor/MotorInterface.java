package io.github.captainsoccer.basicmotor;

import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;
import io.github.captainsoccer.basicmotor.measurements.Measurements;

/**
 * This class is an interface for the different motor controllers.
 * It is used in order to initialize and configure the motor controllers.
 * all other functions of the motor controller are handled in the BasicMotor implementation.
 */
public abstract class MotorInterface {

    /** The name of the motor controller */
    public final String name;

    /**
     * Creates a MotorInterface with the provided name.
     * @param name the name of the motor
     */
    protected MotorInterface(String name) {
        this.name = name;
    }

    /**
     * Creates a MotorInterface with the name provided in the configuration.
     * @param config the configuration for the motor
     */
    protected MotorInterface(BasicMotorConfig config) {
        this.name = config.motorConfig.name;
    }

    /**
     * Gets the default measurements for the motor.
     * This is used to get the original source of measurements for the motor.
     *
     * @return The default measurements for the motor.
     */
    public abstract Measurements getDefaultMeasurements();

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
    public abstract void updatePIDGainsToMotor(PIDGains pidGains, int slot);

    /**
     * Sets the constraints of the motor controller.
     * This is used to update the constraints of the motor controller.
     * The constraints sent should be in motor units.
     *
     * @param constraints The constraints to set on the motor controller.
     */
    public abstract void updateConstraintsGainsToMotor(ConstraintsGains constraints);
}
