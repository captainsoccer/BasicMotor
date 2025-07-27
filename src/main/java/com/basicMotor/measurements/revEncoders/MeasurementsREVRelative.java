package com.basicMotor.measurements.revEncoders;

import com.basicMotor.measurements.Measurements;
import com.revrobotics.RelativeEncoder;

/**
 * This class provides measurements from a REV Relative Encoder.
 * A relative encoder is the one found built-in to rev brushless motors.
 * Also, it can be the through bore relative encoder.
 */
public class MeasurementsREVRelative extends Measurements {
    /**
     * The relative encoder used for taking measurements.
     */
    private final RelativeEncoder encoder;

    /**
     * The previous velocity of the motor controller.
     * Used to calculate the acceleration.
     */
    private double previousVelocity = 0;
    /**
     * The current velocity of the motor controller.
     */
    private double currentVelocity = 0;
    /**
     * The acceleration of the motor controller.
     * Calculated as the change in velocity over time.
     */
    private double acceleration = 0;

    /**
     * Creates a new measurements object with the given relative encoder, gear ratio, and unit conversion factor.
     *
     * @param encoder        The relative encoder used to get the measurements from the motor controller.
     * @param gearRatio      The gear ratio of the encoder, this is the ratio of the encoder's output to the mechanism's output.
     *                       A number larger than 1 indicates a reduction (e.g., 2:1 gear ratio means the encoder turns twice for every rotation of the mechanism).
     * @param unitConversion The value that will be multiplied by to convert the measurements to the desired units.
     *                       This will be desired units per rotation.
     *                       More info at {@link com.basicMotor.configuration.BasicMotorConfig.MotorConfig#unitConversion}.
     */
    public MeasurementsREVRelative(RelativeEncoder encoder, double gearRatio, double unitConversion) {
        super(gearRatio, unitConversion);
        this.encoder = encoder;
    }

    @Override
    public Measurement update(double dt) {
        currentVelocity = encoder.getVelocity() / 60; // Convert to rotations per second

        acceleration = (currentVelocity - previousVelocity) / dt;

        previousVelocity = currentVelocity;

        return super.update(dt);
    }

    @Override
    protected double getUpdatedPosition() {
        return encoder.getPosition();
    }

    @Override
    protected double getUpdatedVelocity() {
        return currentVelocity;
    }

    @Override
    protected double getUpdatedAcceleration() {
        return acceleration;
    }

    @Override
    public void setPosition(double position) {
        encoder.setPosition(position);
    }
}
