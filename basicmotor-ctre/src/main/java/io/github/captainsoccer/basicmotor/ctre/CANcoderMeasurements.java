package io.github.captainsoccer.basicmotor.ctre;

import io.github.captainsoccer.basicmotor.ctre.talonfx.BasicTalonFX;
import io.github.captainsoccer.basicmotor.BasicMotorConfig.MotorConfig;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import io.github.captainsoccer.basicmotor.ctre.talonfx.TalonFXSensors;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;

/**
 * A class that provides measurements for the CANCoder sensor.
 * Used for using the CANCoder as a measurement source in the BasicMotor library.
 * If you want to use a CANCoder with a TalonFX motor,
 * please use the {@link BasicTalonFX#useRemoteCanCoder(CANcoder, double, double, double)}
 */
public class CANcoderMeasurements extends Measurements {
    /**
     * The timeout for the signals to update.
     * This stores how much time to wait for the signals to update before returning the values.
     */
    private final double timeout;

    /**
     * The position status signal
     */
    private final StatusSignal<Angle> motorPosition;
    /**
     * The velocity status signal
     */
    private final StatusSignal<AngularVelocity> motorVelocity;

    /**
     * The latency compensated position.
     * Phoenix6 provides a method to get the latency compensated value of the position using the velocity signal.
     */
    double positionLatencyCompensatedValue = 0;

    /**
     * The current velocity of the motor.
     * This is updated in the update method.
     * Used to calculate the acceleration.
     */
    private double currentVelocity = 0;
    /**
     * The last velocity of the motor.
     * This is used to calculate the acceleration.
     */
    private double lastVelocity = 0;
    /**
     * The acceleration of the motor.
     * This is calculated as the change in velocity over time.
     */
    private double acceleration = 0;

    /**
     * This flag is used to enable waiting for all signals to update before returning the values.
     * This feature works only with a licensed version of Phoenix Pro connected to a canivore.
     * If this is set to true without a canivore, the robot code will slow down significantly.
     */
    private final boolean timeSync;

    /**
     * Creates a new measurements object with the given signals sets the refresh rate of the signals.
     * This method does not optimize the canbus usage of the canCoder.
     *
     * @param positionSignal           The position signal of the canCoder
     * @param velocitySignal           The velocity signal of the canCoder
     * @param canCoderToMechanismRatio The ratio of the canCoder rotations to the mechanism rotations.
     *                                 A number larger than 1 means the canCoder spins more than the mechanism.
     *                                 In most cases, this will be 1.
     * @param unitConversion           The value that the mehcnasims rotations will be multiplied by to convert the measurements to the desired units.
     *                                 See {@link MotorConfig#unitConversion} for more information.
     * @param throughRIO               If true, the canCoder will use the timings of a roboRIO pid controller,
     *                                 else it will use the timings of a motor controller pid controller.
     *                                 If you are using this function outside the BasicMotor library,
     *                                 you should set this to true.
     * @param timeSync                 If true, the measurements will wait for all signals to update before returning the values.
     *                                 Use this only if you have a licensed version of Phoenix Pro connected to a canivore.
     *                                 Otherwise, it will slow down the robot code significantly.
     */
    public CANcoderMeasurements(StatusSignal<Angle> positionSignal,
                                StatusSignal<AngularVelocity> velocitySignal,
                                double canCoderToMechanismRatio,
                                double unitConversion,
                                boolean throughRIO,
                                boolean timeSync) {

        super(canCoderToMechanismRatio, unitConversion);

        this.timeSync = timeSync;

        motorPosition = positionSignal;
        motorVelocity = velocitySignal;

        double refreshHZ =
                throughRIO ? MotorManager.ControllerLocation.RIO.getHZ() : MotorManager.ControllerLocation.MOTOR.getHZ();

        positionSignal.setUpdateFrequency(refreshHZ);
        velocitySignal.setUpdateFrequency(refreshHZ);

        timeout = 1 / (refreshHZ * TalonFXSensors.TIMEOUT_REFRESH_MULTIPLIER);
    }

    /**
     * Creates a new measurements object with the given signals sets the refresh rate of the signals.
     * This method does not optimize the canbus usage of the canCoder.
     *
     * @param positionSignal           The position signal of the canCoder
     * @param velocitySignal           The velocity signal of the canCoder
     * @param canCoderToMechanismRatio The ratio of the canCoder rotations to the mechanism rotations.
     *                                 A number larger than 1 means the canCoder spins more than the mechanism.
     *                                 In most cases, this will be 1.
     * @param unitConversion           The value that the mehcnasims rotations will be multiplied by to convert the measurements to the desired units.
     *                                 See {@link MotorConfig#unitConversion} for more information.
     * @param throughRIO               If true, the canCoder will use the timings of a roboRIO pid controller,
     *                                 else it will use the timings of a motor controller pid controller.
     *                                 If you are using this function outside the BasicMotor library,
     *                                 you should set this to true.
     */
    public CANcoderMeasurements(StatusSignal<Angle> positionSignal,
                                StatusSignal<AngularVelocity> velocitySignal,
                                double canCoderToMechanismRatio,
                                double unitConversion,
                                boolean throughRIO) {
        this(positionSignal, velocitySignal, canCoderToMechanismRatio, unitConversion, throughRIO, false);
    }

    /**
     * Creates a new measurements object with the given CANCoder
     *
     * @param cancoder                 The CANCoder to use for the measurements.
     * @param canCoderToMechanismRatio The ratio of the canCoder rotations to the mechanism rotations.
     *                                 A number larger than 1 means the canCoder spins more than the mechanism.
     *                                 In most cases, this will be 1.
     * @param unitConversion           The value that the mehcnasims rotations will be multiplied by to convert the measurements to the desired units.
     *                                 See {@link MotorConfig#unitConversion} for more information.
     * @param throughRIO               If true, the canCoder will use the timings of a roboRIO pid controller,
     *                                 else it will use the timings of a motor controller pid controller.
     *                                 If you are using this function outside the BasicMotor library,
     *                                 you should set this to true.
     * @param timeSync                 If true, the measurements will wait for all signals to update before returning the values.
     *                                 Use this only if you have a licensed version of Phoenix Pro connected to a canivore.
     *                                 Otherwise, it will slow down the robot code significantly.
     */
    public CANcoderMeasurements(CANcoder cancoder, double canCoderToMechanismRatio, double unitConversion, boolean throughRIO, boolean timeSync) {
        this(cancoder.getPosition(false), cancoder.getVelocity(false), canCoderToMechanismRatio, unitConversion, throughRIO, timeSync);
    }

    /**
     * Creates a new measurements object with the given CANCoder.
     *
     * @param cancoder                 The CANCoder to use for the measurements.
     * @param canCoderToMechanismRatio The ratio of the canCoder rotations to the mechanism rotations.
     *                                 A number larger than 1 means the canCoder spins more than the mechanism.
     *                                 In most cases, this will be 1.
     * @param unitConversion           The value that the mehcnasims rotations will be multiplied by to convert the measurements to the desired units.
     *                                 See {@link MotorConfig#unitConversion} for more information.
     * @param throughRIO               If true, the canCoder will use the timings of a roboRIO pid controller,
     *                                 else it will use the timings of a motor controller pid controller.
     *                                 If you are using this function outside the BasicMotor library,
     *                                 you should set this to true.
     */
    public CANcoderMeasurements(CANcoder cancoder, double canCoderToMechanismRatio, double unitConversion, boolean throughRIO) {
        this(cancoder, canCoderToMechanismRatio, unitConversion, throughRIO, false);
    }

    /**
     * Creates a new measurements object with the given CANCoder.
     *
     * @param cancoder                 The CANCoder to use for the measurements.
     * @param canCoderToMechanismRatio The ratio of the canCoder rotations to the mechanism rotations.
     *                                 A number larger than 1 means the canCoder spins more than the mechanism.
     *                                 In most cases, this will be 1.
     * @param unitConversion           The value that the mehcnasims rotations will be multiplied by to convert the measurements to the desired units.
     *                                 See {@link MotorConfig#unitConversion} for more information.
     */
    public CANcoderMeasurements(CANcoder cancoder, double canCoderToMechanismRatio, double unitConversion) {
        this(cancoder, canCoderToMechanismRatio, unitConversion, true);
    }

    @Override
    public Measurement update(double dt) {
        if (timeSync) BaseStatusSignal.waitForAll(timeout, motorPosition, motorVelocity);
        else BaseStatusSignal.refreshAll(motorPosition, motorVelocity);

        var position = BaseStatusSignal.getLatencyCompensatedValue(motorPosition, motorVelocity);
        positionLatencyCompensatedValue = position.in(Units.Rotations);

        currentVelocity = motorVelocity.getValueAsDouble();

        acceleration = (currentVelocity - lastVelocity) / dt;

        lastVelocity = currentVelocity;

        return super.update(dt);
    }

    /**
     * Sets the update frequency of the signals.
     * This will update the frequency of the position and velocity signals.
     *
     * @param refreshHZ The refresh rate of the signals (how often to update the signals)
     */
    public void setUpdateFrequency(double refreshHZ) {
        motorPosition.setUpdateFrequency(refreshHZ);
        motorVelocity.setUpdateFrequency(refreshHZ);
    }

    @Override
    protected double getUpdatedPosition() {
        return positionLatencyCompensatedValue;
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
        //Does nothing, CANCoder does not support setting position directly.
    }
}
