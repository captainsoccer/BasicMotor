package com.basicMotor.measurements;

import com.basicMotor.BasicMotor;

/**
 * This class is used to get the measurements of the motor. It is used to get the position,
 * velocity, and acceleration of the motor. It is used to get the measurements of the motor. this
 * class is used by {@link BasicMotor} to get the measurements of the motor.
 * Different encoders will extend this class to provide their own implementation of the measurements.
 */
public abstract class Measurements {
    /**
     * The measurements of the motor.
     *
     * @param position     The position of the motor in position units (e.g., meters, rotations)
     * @param velocity     The velocity of the motor in velocity units (e.g., meters per second, rotations per second)
     * @param acceleration The acceleration of the motor in acceleration units (e.g., meters per second squared, rotations per second squared)
     */
    public record Measurement(double position, double velocity, double acceleration) {
        /**
         * an empty measurement (all zeros)
         */
        public static final Measurement EMPTY = new Measurement(0, 0, 0);
    }

    /**
     * The gear ratio is the ratio between the motor's rotations and the mechanism's rotations.
     * For example, if the motor has a gear ratio of 2:1, it means that for every 2 rotations of the motor, the mechanism rotates once.
     * A number larger than 1 indicates a reduction (e.g., 2:1 gear ratio means the motor turns twice for every rotation of the mechanism).
     * Don't use this value to convert the measurements to the desired units, use {@link #unitConversion} for that.
     */
    private final double gearRatio;

    /**
     * The unit conversion is the value that will be multiplied by to convert the measurements to the desired units.
     * This will be desired units per rotation.
     * For example, if the desired units are meters and the motor has a gear ratio of 2:1, then the unit conversion should be 2 * Math.PI (the circumference of a circle with radius 1).
     */
    private final double unitConversion;

    /**
     * The latest measurement of the encoder since last update.
     */
    private Measurement latestMeasurement = Measurement.EMPTY;

    /**
     * Creates a new measurements object with the given gear ratio
     *
     * @param gearRatio      The ratio between the motor and the mechanism (the measurements are divided by this).
     *                       A number larger than 1 means the motor is geared down (e.g., 10:1).
     *                       This is used to convert the measurements from the motor's rotations to the mechanism's rotations.
     *                       For example, if the motor has a gear ratio of 2:1, it means that for every 2 rotations of the motor, the mechanism rotates once.
     * @param unitConversion The value that will be multiplied by to convert the measurements to the desired units.
     *                       This will be desired units per rotation.
     *                       For example, if the desired units are meters and the motor has a gear ratio of 2:1,
     *                       then the unit conversion should be 2 * Math.PI (the circumference of a circle with radius 1).
     *                       More info at {@link com.basicMotor.configuration.BasicMotorConfig.MotorConfig#unitConversion}.
     */
    public Measurements(double gearRatio, double unitConversion) {
        if (gearRatio <= 0) {
            throw new IllegalArgumentException("Gear ratio must be greater than 0");
        }
        this.gearRatio = gearRatio;

        if (unitConversion <= 0) {
            throw new IllegalArgumentException("Unit conversion must be greater than 0");
        }
        this.unitConversion = unitConversion;
    }

    /**
     * Creates a new measurements object with the default gear ratio of 1
     */
    public Measurements() {
        this(1, 1);
    }

    /**
     * Updates the measurements of the motor.
     * This method should be called periodically to update the measurements of the motor.
     *
     * @param dt The time since the last update in seconds.
     * @return The latest measurement of the motor, which includes position, velocity, and acceleration.
     */
    public Measurement update(double dt) {
        // update the measurements
        double position = getUpdatedPosition();
        double velocity = getUpdatedVelocity();
        double acceleration = getUpdatedAcceleration();

        // calculate the new measurements
        position /= gearRatio;
        velocity /= gearRatio;
        acceleration /= gearRatio;

        position *= unitConversion;
        velocity *= unitConversion;
        acceleration *= unitConversion;

        latestMeasurement = new Measurement(position, velocity, acceleration);

        return latestMeasurement;
    }

    /**
     * Gets the gear ratio of the motor.
     * This is the ratio between the motor's rotations and the mechanism's rotations.
     *
     * @return The gear ratio of the motor
     */
    public double getGearRatio() {
        return gearRatio;
    }

    /**
     * Gets the unit conversion of the motor.
     * This is the value that will be multiplied by to convert the measurements to the desired units.
     *
     * @return The unit conversion of the motor.
     */
    public double getUnitConversion() {
        return unitConversion;
    }

    /**
     * Gets the latest measurement of the motor.
     * This is the latest measurement of the motor since the last update.
     *
     * @return The latest measurement of the motor, which includes position, velocity, and acceleration.
     */
    public Measurement getMeasurement() {
        return latestMeasurement;
    }

    /**
     * Gets the latest position of the motor.
     * Default units are rotations, but can be converted to other units using the unit conversion factor.
     *
     * @return The latest position of the motor in the desired units.
     */
    public double getPosition() {
        return latestMeasurement.position;
    }

    /**
     * Gets the latest velocity of the motor.
     * Default units are rotations per second, but can be converted to other units using the unit conversion factor.
     *
     * @return The latest velocity of the motor in the desired units.
     */
    public double getVelocity() {
        return latestMeasurement.velocity;
    }

    /**
     * Gets the latest acceleration of the motor.
     * Default units are rotations per second squared, but can be converted to other units using the unit conversion factor.
     *
     * @return The latest acceleration of the motor in the desired units.
     */
    public double getAcceleration() {
        return latestMeasurement.acceleration;
    }

    /**
     * Gets the updated position of the motor.
     * This method is only called by the update method to get the latest position of the motor.
     * This exists to make sure thread safety is maintained and the position is updated correctly.
     *
     * @return The new position of the motor in the raw units (e.g., rotations, radians).
     */
    protected abstract double getUpdatedPosition();

    /**
     * Updates the velocity of the motor.
     * This method is only called by the update method to get the latest velocity of the motor.
     * This exists to make sure thread safety is maintained and the velocity is updated correctly.
     *
     * @return The new velocity of the motor in the raw units (e.g., rotations per second, radians per second).
     */
    protected abstract double getUpdatedVelocity();

    /**
     * Updates the acceleration of the motor.
     * This method is only called by the update method to get the latest acceleration of the motor.
     * This exists to make sure thread safety is maintained and the acceleration is updated correctly.
     *
     * @return The new acceleration of the motor in the raw units (e.g., rotations per second squared, radians per second squared).
     */
    protected abstract double getUpdatedAcceleration();

    /**
     * This sets the position of the measurements.
     * Units are native units (Most cases are rotations).
     * @param position The new position of the measurements in native units.
     */
    public abstract void setPosition(double position);
}
