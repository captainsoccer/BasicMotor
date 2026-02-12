package io.github.captainsoccer.basicmotor.rev.encoders;

import com.revrobotics.AbsoluteEncoder;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.measurements.Measurements;

/**
 * A class that provides measurements from a REV Absolute Encoder.
 * It extends the Measurements class and implements methods to update
 * position, velocity, and acceleration based on the encoder readings.
 */
public class RevAbsoluteEncoder extends Measurements {
    /**
     * The absolute encoder used for taking measurements
     */
    private final AbsoluteEncoder encoder;

    /**
     * The current velocity of the encoder in rotations per second
     */
    private double currentVelocity;
    /**
     * The previous velocity of the encoder in rotations per second. This is used for acceleration calculation
     */
    private double previousVelocity;
    /**
     * The acceleration of the encoder, calculated from the velocity.
     */
    private double acceleration;

    /**
     * Creates a new measurements object with the given absolute encoder and unit conversion factor.
     * There is no gear ratio as the absolute encoder will always be connected in a 1:1 ratio to the mechanism.
     *
     * @param encoder        The absolute encoder used to get the measurements from the motor controller
     * @param gearRatio      The ratio between the encoder and the mechanism. this is the divider for the encoder reading
     *                       For example, if the motor has a gear ratio of 2:1, it means that for every 2 rotations of the motor, the mechanism rotates once.
     * @param unitConversion The value that will be multiplied by to convert the measurements to the desired units.
     *                       This will be desired units per rotation.
     *                       For example, if the desired units are meters and the motor has a gear ratio of 2:1,
     *                       then the unit conversion should be 2 * Math.PI (the circumference of a circle with radius 1).
     *                       More info at {@link BasicMotorConfig.MotorConfig#unitConversion}.
     */
    public RevAbsoluteEncoder(AbsoluteEncoder encoder, double gearRatio, double unitConversion) {
        super(gearRatio, unitConversion);
        this.encoder = encoder;
    }

    @Override
    public Measurement update(double dt) {
        // converts rpm to rps
        currentVelocity = encoder.getVelocity();

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
        //Does nothing, REV Absolute Encoders do not support setting position directly.
    }
}
