package io.github.captainsoccer.basicmotor.ctre.talonfx;
import io.github.captainsoccer.basicmotor.BasicMotorConfig.MotorConfig;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * This class is used to store and update the measurements of a TalonFX motor controller.
 * It handles updating the position, velocity, and acceleration of the motor.
 * It also provides latency compensation for velocity and position measurements.
 */
public class TalonFXMeasurements extends Measurements {
    /**
     * The timeout for waiting for all signals to update.
     * Used only when Pro features are enabled.
     * This is the refresh rate divided by {@link TalonFXSensors#TIMEOUT_REFRESH_MULTIPLIER}.
     */
    private final double timeout;

    /**
     * The TalonFX motor controller to get the measurements from.
     * This is used to get the position, velocity, and acceleration signals.
     */
    private final TalonFX talonFX;

    /**
     * The motor position signal.
     */
    private final StatusSignal<Angle> motorPosition;
    /**
     * The motor velocity signal.
     */
    private final StatusSignal<AngularVelocity> motorVelocity;
    /**
     * The motor acceleration signal.
     */
    private final StatusSignal<AngularAcceleration> motorAcceleration;

    /**
     * The latency compensated value of the motor position.
     */
    private double positionLatencyCompensatedValue = 0;
    /**
     * The latency compensated value of the motor velocity.
     */
    private double velocityLatencyCompensatedValue = 0;

    /**
     * Flag to enable Pro features of the measurements.
     * If true, the measurements will wait for all signals to update before returning values.
     * Use this only if the motor is connected to a CANivore and is licensed for Pro features.
     */
    private boolean timeSync = false;

    /**
     * Creates a new MeasurementsTalonFX object with the given signals and parameters.
     *
     * @param motor          The TalonFX motor controller to get the measurements from
     * @param gearRatio      The ratio between the motor and the mechanism (the measurements are divided by this).
     *                       A number larger than 1 means the motor is geared down (e.g., 10:1),
     * @param unitConversion The value that will be multiplied by to convert the measurements to the desired units.
     *                       This will be desired units per rotation.
     *                       More info at {@link MotorConfig#unitConversion}.
     */
    public TalonFXMeasurements(TalonFX motor, double gearRatio, double unitConversion) {
        super(gearRatio, unitConversion);

        this.talonFX = motor;

        motorPosition = motor.getPosition(false);
        motorVelocity = motor.getVelocity(false);
        motorAcceleration = motor.getAcceleration(false);

        double refreshHZ = MotorManager.ControllerLocation.MOTOR.getHZ();

        motorPosition.setUpdateFrequency(refreshHZ);
        motorVelocity.setUpdateFrequency(refreshHZ);
        motorAcceleration.setUpdateFrequency(refreshHZ);

        timeout = 1 / (refreshHZ * TalonFXSensors.TIMEOUT_REFRESH_MULTIPLIER);
    }

    /**
     * Sets the update frequency of the signals.
     *
     * @param refreshHZ The refresh rate of the signals (how often to update the signals)
     */
    public void setUpdateFrequency(double refreshHZ) {
        motorPosition.setUpdateFrequency(refreshHZ);
        motorVelocity.setUpdateFrequency(refreshHZ);
        motorAcceleration.setUpdateFrequency(refreshHZ);
    }

    /**
     * Sets whether to enable the Pro features of the measurements
     *
     * @param timeSync True to enable waiting for all signals to update before returning values,
     */
    public void setTimeSync(boolean timeSync) {
        this.timeSync = timeSync;
    }

    @Override
    public Measurement update(double dt) {
        if (timeSync) BaseStatusSignal.waitForAll(timeout, motorPosition, motorVelocity, motorAcceleration);
        else BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorAcceleration);

        var position = BaseStatusSignal.getLatencyCompensatedValue(motorPosition, motorVelocity);
        positionLatencyCompensatedValue = position.in(Units.Rotations);

        var velocity = BaseStatusSignal.getLatencyCompensatedValue(motorVelocity, motorAcceleration);
        velocityLatencyCompensatedValue = velocity.in(Units.RotationsPerSecond);

        return super.update(dt);
    }

    @Override
    protected double getUpdatedPosition() {
        return positionLatencyCompensatedValue;
    }

    @Override
    protected double getUpdatedVelocity() {
        return velocityLatencyCompensatedValue;
    }

    @Override
    protected double getUpdatedAcceleration() {
        return motorAcceleration.getValueAsDouble();
    }

    @Override
    public void setPosition(double position) {
        talonFX.setPosition(position);
    }
}
