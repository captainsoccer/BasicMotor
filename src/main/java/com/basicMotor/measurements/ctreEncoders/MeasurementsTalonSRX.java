package com.basicMotor.measurements.ctreEncoders;

import com.basicMotor.measurements.Measurements;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * A class that provides measurements for the TalonSRX motor controller.
 * Used for using the TalonSRX as a measurement source in the BasicMotor library.
 * The TalonSRX must be configured to use a sensor, such as a quadrature encoder, for this to work properly.
 */
public class MeasurementsTalonSRX extends Measurements {
    /**
     * The TalonSRX motor controller instance used by this MeasurementsTalonSRX.
     * This is the motor controller that will provide the measurements.
     */
    private final TalonSRX talonSRX;
    /**
     * The number of ticks per revolution of the TalonSRX encoder.
     * This is used to convert the sensor position and velocity to rotations.
     */
    public final double tickPerRevolution;

    /**
     * The velocity of the motor in rotations per second.
     */
    private double velocity = 0;
    /**
     * The previous velocity of the motor in rotations per second.
     * This is used to calculate acceleration.
     */
    private double previousVelocity = 0;
    /**
     * The acceleration of the motor in rotations per second squared.
     * This is calculated as the change in velocity over time.
     */
    private double acceleration = 0;

    /**
     * Creates A MeasurementsTalonSRX instance with the provided TalonSRX, ticks per revolution, gear ratio, and unit conversion.
     * The motor needs to be configured to use a sensor, such as a quadrature encoder, for this to work properly.
     * @param talonSRX The TalonSRX used to get the measurements from the motor controller
     * @param tickPerRevolution The number of ticks per revolution of the TalonSRX encoder.
     * @param gearRatio The gear ratio of the encoder, this is the ratio of the encoder's output to the mechanism's output.
     * @param unitConversion The value that will be multiplied by to convert the measurements to the desired units.
     * @see com.ctre.phoenix.motorcontrol.can.TalonSRX#configSelectedFeedbackSensor(FeedbackDevice)
     */
    public MeasurementsTalonSRX(TalonSRX talonSRX, double tickPerRevolution, double gearRatio, double unitConversion) {
        super(gearRatio, unitConversion);
        this.talonSRX = talonSRX;
        this.tickPerRevolution = tickPerRevolution;

        // Initialize the TalonSRX's selected sensor position to 0
        talonSRX.setSelectedSensorPosition(0);
    }

    /**
     * Creates A MeasurementsTalonSRX instance with the provided TalonSRX, ticks per revolution and gear ratio.
     * This constructor uses a default unit conversion of 1.0.
     * The motor needs to be configured to use a sensor, such as a quadrature encoder, for this to work properly.
     * @param talonSRX The TalonSRX used to get the measurements from the motor controller
     * @param tickPerRevolution The number of ticks per revolution of the TalonSRX encoder.
     * @param gearRatio The gear ratio of the encoder, this is the ratio of the encoder's output to the mechanism's output.
     * @see com.ctre.phoenix.motorcontrol.can.TalonSRX#configSelectedFeedbackSensor(FeedbackDevice)
     */
    public MeasurementsTalonSRX(TalonSRX talonSRX, double tickPerRevolution, double gearRatio) {
        this(talonSRX, tickPerRevolution, gearRatio, 1.0);
    }

    /**
     * Creates A MeasurementsTalonSRX instance with the provided TalonSRX and ticks per revolution.
     * The gear ratio is set to 1 (no reduction) and the unit conversion is 1 (no conversion).
     * The motor needs to be configured to use a sensor, such as a quadrature encoder, for this to work properly.
     * @param talonSRX The TalonSRX used to get the measurements from the motor controller
     * @param tickPerRevolution The number of ticks per revolution of the TalonSRX encoder.
     * @see com.ctre.phoenix.motorcontrol.can.TalonSRX#configSelectedFeedbackSensor(FeedbackDevice)
     */
    public MeasurementsTalonSRX(TalonSRX talonSRX, double tickPerRevolution) {
        this(talonSRX, tickPerRevolution, 1.0, 1.0);
    }

    @Override
    public Measurement update(double dt) {
        // Update the velocity and acceleration based on the TalonSRX's sensor position
        velocity = (talonSRX.getSelectedSensorVelocity() * 10) / tickPerRevolution; // Convert ticks per 100ms to rotations per second
        acceleration = (velocity - previousVelocity) / dt; // Calculate acceleration as the change in
        // velocity over time

        previousVelocity = velocity;

        return super.update(dt);
    }

    @Override
    protected double getUpdatedPosition() {
        return talonSRX.getSelectedSensorPosition() / tickPerRevolution;
    }

    @Override
    protected double getUpdatedVelocity() {
        return velocity;
    }

    @Override
    protected double getUpdatedAcceleration() {
        return acceleration;
    }

    @Override
    public void setPosition(double position) {
        talonSRX.setSelectedSensorPosition(position * tickPerRevolution);
    }
}
