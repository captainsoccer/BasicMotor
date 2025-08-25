package io.github.captainsoccer.basicmotor.ctre.talonsrx;


import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.wpilibj.DriverStation;
import io.github.captainsoccer.basicmotor.BasicMotor;
import com.ctre.phoenix.motorcontrol.ControlMode;
import io.github.captainsoccer.basicmotor.LogFrame;
import io.github.captainsoccer.basicmotor.controllers.Controller;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.gains.ControllerGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;
import io.github.captainsoccer.basicmotor.measurements.EmptyMeasurements;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;
import io.github.captainsoccer.basicmotor.gains.CurrentLimits;

/**
 * A basic motor implementation for the TalonSRX motor controller.
 * This class provides a simple interface to control the TalonSRX motor controller
 * and provides methods to set the PID gains, current limits, idle mode, and more.
 */
public class BasicTalonSRX extends BasicMotor {
    /**
     * The type of encoder that is connected directly to the TalonSRX motor controller.
     * This is used to configure the TalonSRX motor controller to use the correct encoder
     */
    public enum EncoderType{
        /**
         * No encoder is connected to the TalonSRX motor controller.
         * This is used when the TalonSRX is used in a simple open loop control mode.
         */
        NONE(FeedbackDevice.None),
        /**
         * A quadrature encoder is connected to the TalonSRX motor controller.
         * This is the most common type of encoder used with the TalonSRX.
         */
        QUAD_ENCODER(FeedbackDevice.QuadEncoder),
        /**
         * An analog encoder is connected to the TalonSRX motor controller.
         * This is used when an analog encoder is connected to the TalonSRX.
         */
        ANALOG(FeedbackDevice.Analog),
        /**
         * A tachometer is connected to the TalonSRX motor controller.
         * This only measures the speed of the motor, not the position.
         * Used commonly for flywheels or other high-speed applications.
         */
        TACHOMETER(FeedbackDevice.Tachometer),
        /**
         * An encoder that uses PWM signals to report position is connected to the TalonSRX motor controller.
         * This is usually an absolute encoder that provides a position value in PWM format.
         */
        ABSOLUTE(FeedbackDevice.PulseWidthEncodedPosition);

        /**
         * The feedback device type used by the TalonSRX motor controller.
         * This is used to configure the TalonSRX motor controller to use the correct encoder type.
         */
        public final FeedbackDevice feedbackDevice;

        /**
         * Constructor for the EncoderType enum.
         * @param feedbackDevice The feedback device type used by the TalonSRX motor controller.
         */
        EncoderType(FeedbackDevice feedbackDevice){
            this.feedbackDevice = feedbackDevice;
        }
    }

    /**
     * The TalonSRX motor controller instance used by this BasicTalonSRX.
     */
    private final TalonSRX motor;

    /**
     * The configuration for the TalonSRX motor controller.
     * This is used to configure the TalonSRX motor controller with the correct settings.
     */
    private final TalonSRXConfiguration config = new TalonSRXConfiguration();

    /**
     * The default measurements for the TalonSRX motor controller.
     * This is used to provide measurements for the TalonSRX motor controller.
     * If no measurements are provided, it will use the default empty measurements.
     */
    private final Measurements defaultMeasurements;

    /**
     * The current PID gains in the motor native units.
     * Used to calculate the output of the PID controller.
     */
    private PIDGains motorGains;

    /**
     * The currently selected PID slot.
     * This is used to determine which PID slot to use when setting the PID gains.
     */
    private int selectedSlot = 0;

    /**
     * Creates a BasicTalonSRX instance with the provided motor ID and name.
     * This will make the motor only open loop controllable,
     * Until changed by the {@link #setMeasurements(Measurements)} method or the {@link #setEncoderType(EncoderType, int)}.
     * @param id The ID of the TalonSRX motor controller
     * @param name The name of the motor controller
     */
    public BasicTalonSRX(int id, String name){
        super(new ControllerGains(), name);

        this.motor = new TalonSRX(id);
        this.motor.configFactoryDefault();
        config.voltageCompSaturation = MotorManager.config.motorIdealVoltage;

        applyConfig();

        defaultMeasurements = new EmptyMeasurements();
    }

    /**
     * Creates a BasicTalonSRX instance with the provided motor ID, name, controller gains, controller location, encoder type, and ticks per revolution.
     * This constructor is used to create a TalonSRX motor controller with the specified settings
     * @param id The ID of the TalonSRX motor controller
     * @param name The name of the motor controller
     * @param controllerGains The controller gains to use for the motor controller
     * @param encoderType The type of encoder that is connected to the TalonSRX motor controller.
     * @param tickPerRevolution The number of ticks per revolution of the encoder.
     */
    public BasicTalonSRX(int id, String name, ControllerGains controllerGains, EncoderType encoderType, int tickPerRevolution) {
        super(controllerGains, name);

        this.motor = new TalonSRX(id);
        this.motor.configFactoryDefault();
        this.config.voltageCompSaturation = MotorManager.config.motorIdealVoltage;

        this.config.primaryPID.selectedFeedbackSensor = encoderType.feedbackDevice;
        this.config.primaryPID.selectedFeedbackCoefficient = 1.0 / tickPerRevolution;

        applyConfig();

        defaultMeasurements = new TalonSRXMeasurements(motor, tickPerRevolution);
    }

    /**
     * Creates a BasicTalonSRX instance with the provided configuration.
     * This constructor is used to create a TalonSRX motor controller with the specified settings.
     * @param motorConfig The configuration for the TalonSRX motor controller.
     */
    public BasicTalonSRX(BasicTalonSRXConfig motorConfig){
        super(motorConfig);

        this.motor = new TalonSRX(motorConfig.motorConfig.id);
        this.motor.configFactoryDefault();
        this.config.voltageCompSaturation = MotorManager.config.motorIdealVoltage;

        this.config.primaryPID.selectedFeedbackSensor = motorConfig.encoderConfig.type.feedbackDevice;
        this.config.primaryPID.selectedFeedbackCoefficient = 1.0 / motorConfig.encoderConfig.tickPerRevolution;

        applyConfig();

        defaultMeasurements = new TalonSRXMeasurements(motor, motorConfig.encoderConfig.tickPerRevolution,
                motorConfig.motorConfig.gearRatio, motorConfig.motorConfig.unitConversion);

        setCurrentLimits(motorConfig.currentLimitConfig.toCurrentLimits());
    }

    @Override
    protected void updatePIDGainsToMotor(PIDGains pidGains, int slot) {
        motorGains = pidGains.convertToDutyCycle();

        var pidConfig = switch (slot){
            case 0 -> config.slot0;
            case 1 -> config.slot1;
            case 2 -> config.slot2;

            default -> config.slot0;
        };

        pidConfig.kP = motorGains.getK_P();
        pidConfig.kI = motorGains.getK_I();
        pidConfig.kD = motorGains.getK_D();

        pidConfig.allowableClosedloopError = motorGains.getTolerance();
        pidConfig.integralZone = motorGains.getI_Zone();
        pidConfig.maxIntegralAccumulator = motorGains.getI_MaxAccum();

        applyConfig();
    }

    @Override
    protected double getInternalPIDLoopTime() {
        return 0.001; // TalonSRX has a fixed internal loop time of 1ms
        //According to a chief delphi post:
        // https://www.chiefdelphi.com/t/control-loop-timing-of-various-motor-controllers/370356/4
    }

    @Override
    protected void updateConstraintsGainsToMotor(ConstraintsGains constraints) {
        config.peakOutputForward = constraints.getMaxMotorOutput() / MotorManager.config.motorIdealVoltage;
        config.peakOutputReverse = -constraints.getMaxMotorOutput() / MotorManager.config.motorIdealVoltage;

        config.nominalOutputForward = constraints.getVoltageDeadband() / MotorManager.config.motorIdealVoltage;
        config.nominalOutputReverse = -constraints.getVoltageDeadband() / MotorManager.config.motorIdealVoltage;

        // How much time to go from 0 to 100% output in seconds, based on the voltage ramp rate
        config.closedloopRamp = constraints.getRampRate();
        config.openloopRamp = constraints.getRampRate();

        if (constraints.getConstraintType() == ConstraintsGains.ConstraintType.LIMITED
                && getDefaultMeasurements() instanceof TalonSRXMeasurements talonMeasurements) {
            config.forwardSoftLimitEnable = true;
            config.forwardSoftLimitThreshold = constraints.getMaxValue() * talonMeasurements.tickPerRevolution;

            config.reverseSoftLimitEnable = true;
            config.reverseSoftLimitThreshold = constraints.getMinValue() * talonMeasurements.tickPerRevolution;
        } else {
            config.forwardSoftLimitEnable = false;
            config.reverseSoftLimitEnable = false;
        }

        applyConfig();
    }

    @Override
    protected Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }

    @Override
    public void setCurrentLimits(CurrentLimits currentLimits) {
        config.continuousCurrentLimit = currentLimits.getCurrentLimit();

        if (currentLimits instanceof TalonSRXCurrentLimits talonCurrentLimits &&
                talonCurrentLimits.peakCurrentLimit() != 0 && talonCurrentLimits.peakCurrentDuration() != 0) {

            config.peakCurrentLimit = talonCurrentLimits.peakCurrentLimit();
            config.peakCurrentDuration = talonCurrentLimits.peakCurrentDuration() * 1000; // Convert seconds to milliseconds
        } else {
            // If not talonSRX current limits, use the continuous current limit for peak current limit
            config.peakCurrentDuration = 0;
        }

        applyConfig();
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        var idleMode = switch (mode) {
            case COAST -> com.ctre.phoenix.motorcontrol.NeutralMode.Coast;
            case BRAKE -> com.ctre.phoenix.motorcontrol.NeutralMode.Brake;
        };

        motor.setNeutralMode(idleMode);
    }

    @Override
    public void setMotorInverted(boolean inverted) {
        motor.setInverted(inverted);
    }

    @Override
    protected void stopRecordingMeasurements() {
        // Does nothing, as the TalonSRX does not support changing timings of can bus signals
        DriverStation.reportWarning("motor: " + this.name + " does not stop recording the measurements", false);
    }

    @Override
    protected void startRecordingMeasurements(double HZ) {
        //Does nothing, as the TalonSRX does not support changing timings of can bus signals
    }

    @Override
    protected void updateMainLoopTiming(MotorManager.ControllerLocation location) {
        //Does nothing as the TalonSRX does not support changing timings of can bus signals
    }

    @Override
    protected void setMotorFollow(BasicMotor master, boolean inverted) {
        BasicTalonSRX masterMotor = (BasicTalonSRX) master;

        motor.setInverted(inverted != masterMotor.motor.getInverted());

        motor.follow(masterMotor.motor);
    }

    @Override
    protected void stopMotorFollow() {
        stopRecordingMeasurements();
    }

    @Override
    protected void setMotorOutput(double setpoint, double feedForward, Controller.ControlMode mode, int slot) {

        if(slot != this.selectedSlot){
            this.selectedSlot = slot;
            motor.selectProfileSlot(slot, 0);
        }

        switch (mode) {
            // Converts voltage to percent output based on the ideal voltage of the motor
            case VOLTAGE -> motor.set(TalonSRXControlMode.PercentOutput, setpoint / MotorManager.config.motorIdealVoltage);

            // Converts the voltage feedforward to percent output based on the ideal voltage of the motor
            case POSITION, PROFILED_POSITION -> motor.set(TalonSRXControlMode.Position, setpoint,
                    DemandType.ArbitraryFeedForward, feedForward / MotorManager.config.motorIdealVoltage);

            // Converts the velocity feedforward to percent output based on the ideal voltage of the motor
            case VELOCITY, PROFILED_VELOCITY -> motor.set(TalonSRXControlMode.Velocity, setpoint,
                    DemandType.ArbitraryFeedForward, feedForward / MotorManager.config.motorIdealVoltage);

            case STOP -> stopMotorOutput();

            case PERCENT_OUTPUT ->  motor.set(TalonSRXControlMode.PercentOutput, setpoint);

            case CURRENT, TORQUE -> {
                DriverStation.reportError("motor: " + super.name + " does not support current or torque control", false);
                stopMotorOutput();
            }
        }
    }


    @Override
    protected void stopMotorOutput() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    protected LogFrame.SensorData getLatestSensorData() {
        double temperature = motor.getTemperature();

        double inputVoltage = motor.getBusVoltage();
        double outputVoltage = motor.getMotorOutputVoltage();

        double currentOutput = motor.getStatorCurrent();
        double currentDraw = motor.getSupplyCurrent();

        double dutyCycle = motor.getMotorOutputPercent();

        return new LogFrame.SensorData(
                temperature,
                currentDraw,
                currentOutput,
                outputVoltage,
                inputVoltage,
                inputVoltage * currentDraw,
                outputVoltage * currentOutput,
                dutyCycle
        );
    }

    @Override
    protected LogFrame.PIDOutput getPIDLatestOutput() {
        double iAccum = motor.getIntegralAccumulator();
        double derivative = motor.getErrorDerivative();
        double error = motor.getClosedLoopError();
        //Allowed since on the same thread as the getSensorData call
        double busVoltage = super.logFrame.sensorData.voltageInput();

        double pOutput = motorGains.getK_P() * error * busVoltage;
        double iOutput = motorGains.getK_I() * iAccum * busVoltage;
        double dOutput = motorGains.getK_D() * derivative * busVoltage;

        return new LogFrame.PIDOutput(
                pOutput,
                iOutput,
                dOutput,
                pOutput + iOutput + dOutput
        );
    }

    /**
     * Applies the current configuration to the TalonSRX motor controller.
     * This method will apply the current configuration to the TalonSRX motor controller.
     */
    private void applyConfig(){
        var error = motor.configAllSettings(config);
        if (error.value != 0) {
            DriverStation.reportWarning("issues applying config for motor: " + super.name + ". Error: " + error.name(), false);
        }
    }

    /**
     * Sets the encoder type for the TalonSRX motor controller.
     * This method will configure the encoder that is connected directly to the TalonSRX motor controller.
     * @param encoderType The type of encoder that is connected to the TalonSRX motor controller.
     * @param tickPerRevolution The number of ticks per revolution of the encoder.
     * @param gearRatio The gear ratio of the encoder, this is the ratio of the encoder's output to the mechanism's output.
     * @param unitConversion The value that will be multiplied by to convert the measurements to the desired units.
     *                       This will be desired units per rotation.
     */
    public void setEncoderType(EncoderType encoderType, int tickPerRevolution, double gearRatio, double unitConversion) {
        config.primaryPID.selectedFeedbackSensor = encoderType.feedbackDevice;
        config.primaryPID.selectedFeedbackCoefficient = 1.0 / tickPerRevolution;

        applyConfig();

        if(defaultMeasurements instanceof TalonSRXMeasurements) {
            setDefaultMeasurements();
        } else {
            setMeasurements(new TalonSRXMeasurements(motor, tickPerRevolution, gearRatio, unitConversion), false);
        }
    }

    /**
     * Sets the encoder type for the TalonSRX motor controller.
     * This method will configure the encoder that is connected directly to the TalonSRX motor controller.
     * @param encoderType The type of encoder that is connected to the TalonSRX motor controller.
     * @param tickPerRevolution The number of ticks per revolution of the encoder.
     */
    public void setEncoderType(EncoderType encoderType, int tickPerRevolution) {
        setEncoderType(encoderType, tickPerRevolution, 1.0, 1.0);
    }
}
