package com.basicMotor.motors;

import com.basicMotor.BasicMotor;
import com.basicMotor.LogFrame;
import com.basicMotor.configuration.BasicTalonSRXConfig;
import com.basicMotor.controllers.Controller;
import com.basicMotor.gains.ControllerConstraints;
import com.basicMotor.gains.ControllerGains;
import com.basicMotor.gains.PIDGains;
import com.basicMotor.gains.currentLimits.CurrentLimits;
import com.basicMotor.gains.currentLimits.CurrentLimitsTalonSRX;
import com.basicMotor.measurements.EmptyMeasurements;
import com.basicMotor.measurements.Measurements;
import com.basicMotor.measurements.ctreEncoders.MeasurementsTalonSRX;
import com.basicMotor.motorManager.MotorManager;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.wpilibj.DriverStation;

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
    private final TalonSRXConfiguration motorConfig = new TalonSRXConfiguration();

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
     * Creates a BasicTalonSRX instance with the provided motor ID and name.
     * This will make the motor only open loop controllable,
     * Until changed by the {@link #setMeasurements(Measurements)} method or the {@link #setEncoderType(EncoderType, int)}.
     * @param id The ID of the TalonSRX motor controller
     * @param name The name of the motor controller
     * @param controllerLocation The location of the PID controller, RIO or Motor Controller.
     */
    public BasicTalonSRX(int id, String name, MotorManager.ControllerLocation controllerLocation){
        super(new ControllerGains(), name, controllerLocation);

        this.motor = new TalonSRX(id);
        this.motor.configFactoryDefault();
        motorConfig.voltageCompSaturation = MotorManager.config.motorIdealVoltage;

        this.motor.configAllSettings(motorConfig);

        defaultMeasurements = new EmptyMeasurements();
    }

    /**
     * Creates a BasicTalonSRX instance with the provided motor ID and name.
     * This will make the motor only open loop controllable,
     * Until changed by the {@link #setMeasurements(Measurements)} method or the {@link #setEncoderType(EncoderType, int)}.
     * Using this constructor will place the motor in the RIO controller location.
     * @param id The ID of the TalonSRX motor controller
     * @param name The name of the motor controller
     */
    public BasicTalonSRX(int id, String name){
        this(id, name, MotorManager.ControllerLocation.RIO);
    }

    /**
     * Creates a BasicTalonSRX instance with the provided motor ID, name, controller gains, controller location, encoder type, and ticks per revolution.
     * This constructor is used to create a TalonSRX motor controller with the specified settings
     * @param id The ID of the TalonSRX motor controller
     * @param name The name of the motor controller
     * @param controllerGains The controller gains to use for the motor controller
     * @param controllerLocation The location of the PID controller, RIO or Motor Controller.
     * @param encoderType The type of encoder that is connected to the TalonSRX motor controller.
     * @param tickPerRevolution The number of ticks per revolution of the encoder.
     */
    public BasicTalonSRX(int id, String name, ControllerGains controllerGains, MotorManager.ControllerLocation controllerLocation, EncoderType encoderType, int tickPerRevolution) {
        super(controllerGains, name, controllerLocation);

        this.motor = new TalonSRX(id);
        this.motor.configFactoryDefault();
        this.motorConfig.voltageCompSaturation = MotorManager.config.motorIdealVoltage;

        this.motorConfig.primaryPID.selectedFeedbackSensor = encoderType.feedbackDevice;
        this.motorConfig.primaryPID.selectedFeedbackCoefficient = 1.0 / tickPerRevolution;

        motor.configAllSettings(motorConfig);

        defaultMeasurements = new MeasurementsTalonSRX(motor, tickPerRevolution);
    }

    /**
     * Creates a BasicTalonSRX instance with the provided configuration.
     * This constructor is used to create a TalonSRX motor controller with the specified settings.
     * @param config The configuration for the TalonSRX motor controller.
     */
    public BasicTalonSRX(BasicTalonSRXConfig config){
        super(config);

        this.motor = new TalonSRX(config.motorConfig.id);
        this.motor.configFactoryDefault();
        this.motorConfig.voltageCompSaturation = MotorManager.config.motorIdealVoltage;

        this.motorConfig.primaryPID.selectedFeedbackSensor = config.encoderConfig.type.feedbackDevice;
        this.motorConfig.primaryPID.selectedFeedbackCoefficient = 1.0 / config.encoderConfig.tickPerRevolution;

        this.motor.configAllSettings(motorConfig);

        defaultMeasurements = new MeasurementsTalonSRX(motor, config.encoderConfig.tickPerRevolution,
                config.motorConfig.gearRatio, config.motorConfig.unitConversion);

        setCurrentLimits(config.currentLimitConfig.toCurrentLimits());
    }

    @Override
    protected void updatePIDGainsToMotor(PIDGains pidGains) {
        motorGains = pidGains.convertToDutyCycle();

        var pidConfig = motorConfig.slot0;

        pidConfig.kP = motorGains.getK_P();
        pidConfig.kI = motorGains.getK_I();
        pidConfig.kD = motorGains.getK_D();

        pidConfig.allowableClosedloopError = motorGains.getTolerance();
        pidConfig.integralZone = motorGains.getI_Zone();
        pidConfig.maxIntegralAccumulator = motorGains.getI_MaxAccum();

        var error = motor.configureSlot(pidConfig);
        if (error.value != 0) {
            DriverStation.reportWarning("issues setting pid for motor: " + super.name + ". Error: " + error.name(), false);
        }
    }

    @Override
    protected double getInternalPIDLoopTime() {
        return 0.001; // TalonSRX has a fixed internal loop time of 1ms
        //According to a chief delphi post:
        // https://www.chiefdelphi.com/t/control-loop-timing-of-various-motor-controllers/370356/4
    }

    @Override
    protected void updateConstraints(ControllerConstraints constraints) {
        motorConfig.peakOutputForward = constraints.getMaxMotorOutput() / MotorManager.config.motorIdealVoltage;
        motorConfig.peakOutputReverse = -constraints.getMaxMotorOutput() / MotorManager.config.motorIdealVoltage;

        motorConfig.nominalOutputForward = constraints.getVoltageDeadband() / MotorManager.config.motorIdealVoltage;
        motorConfig.nominalOutputReverse = -constraints.getVoltageDeadband() / MotorManager.config.motorIdealVoltage;

        if (constraints.getConstraintType() == ControllerConstraints.ConstraintType.LIMITED
                && getDefaultMeasurements() instanceof MeasurementsTalonSRX talonMeasurements) {
            motorConfig.forwardSoftLimitEnable = true;
            motorConfig.forwardSoftLimitThreshold = constraints.getMaxValue() * talonMeasurements.tickPerRevolution;

            motorConfig.reverseSoftLimitEnable = true;
            motorConfig.reverseSoftLimitThreshold = constraints.getMinValue() * talonMeasurements.tickPerRevolution;
        } else {
            motorConfig.forwardSoftLimitEnable = false;
            motorConfig.reverseSoftLimitEnable = false;
        }

        var error = motor.configAllSettings(motorConfig);
        if (error.value != 0) {
            DriverStation.reportWarning("issues setting constraints for motor: " + super.name + ". Error: " + error.name(), false);
        }
    }

    @Override
    protected Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }

    @Override
    public void setCurrentLimits(CurrentLimits currentLimits) {
        motorConfig.continuousCurrentLimit = currentLimits.getCurrentLimit();

        if (currentLimits instanceof CurrentLimitsTalonSRX talonCurrentLimits &&
                talonCurrentLimits.getPeakCurrentLimit() != 0 && talonCurrentLimits.getPeakCurrentDuration() != 0) {

            motorConfig.peakCurrentLimit = talonCurrentLimits.getPeakCurrentLimit();
            motorConfig.peakCurrentDuration = talonCurrentLimits.getPeakCurrentDuration() * 1000; // Convert seconds to milliseconds
        } else {
            // If not talonSRX current limits, use the continuous current limit for peak current limit
            motorConfig.peakCurrentDuration = 0;
        }

        var error = motor.configAllSettings(motorConfig);
        if (error.value != 0) {
            DriverStation.reportWarning("issues setting current limits for motor: " + super.name + ". Error: " + error.name(), false);
        }
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
        BasicTalonSRX maserMotor = (BasicTalonSRX) master;

        motor.setInverted(inverted != maserMotor.motor.getInverted());

        motor.follow(maserMotor.motor);
    }

    @Override
    protected void stopMotorFollow() {
        stopRecordingMeasurements();
    }

    @Override
    protected void setMotorOutput(double setpoint, double feedForward, Controller.ControlMode mode) {
        switch (mode) {
            // Converts voltage to percent output based on the ideal voltage of the motor
            case VOLTAGE -> motor.set(TalonSRXControlMode.PercentOutput, setpoint / MotorManager.config.motorIdealVoltage);

            // Converts the voltage feedforward to percent output based on the ideal voltage of the motor
            case POSITION, PROFILED_POSITION -> motor.set(TalonSRXControlMode.Position, setpoint,
                    DemandType.ArbitraryFeedForward, feedForward / MotorManager.config.motorIdealVoltage);

            // Converts the velocity feedforward to percent output based on the ideal voltage of the motor
            case VELOCITY, PROFILED_VELOCITY -> motor.set(TalonSRXControlMode.Velocity, setpoint,
                    DemandType.ArbitraryFeedForward, feedForward / MotorManager.config.motorIdealVoltage);

            // For percent output, just set the percent output directly
            default ->  motor.set(TalonSRXControlMode.PercentOutput, setpoint);
        }
    }

    @Override
    protected void stopMotorOutput() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    protected LogFrame.SensorData getSensorData() {
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
     * Sets the encoder type for the TalonSRX motor controller.
     * This method will configure the encoder that is connected directly to the TalonSRX motor controller.
     * @param encoderType The type of encoder that is connected to the TalonSRX motor controller.
     * @param tickPerRevolution The number of ticks per revolution of the encoder.
     * @param gearRatio The gear ratio of the encoder, this is the ratio of the encoder's output to the mechanism's output.
     * @param unitConversion The value that will be multiplied by to convert the measurements to the desired units.
     *                       This will be desired units per rotation.
     */
    public void setEncoderType(EncoderType encoderType, int tickPerRevolution, double gearRatio, double unitConversion) {
        motorConfig.primaryPID.selectedFeedbackSensor = encoderType.feedbackDevice;
        motorConfig.primaryPID.selectedFeedbackCoefficient = 1.0 / tickPerRevolution;

        var error = motor.configAllSettings(motorConfig);
        if (error.value != 0) {
            DriverStation.reportWarning("issues setting encoder type for motor: " + super.name + ". Error: " + error.name(), false);
        }

        if(defaultMeasurements instanceof MeasurementsTalonSRX) {
            setDefaultMeasurements();
        } else {
            setMeasurements(new MeasurementsTalonSRX(motor, tickPerRevolution, gearRatio, unitConversion));
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
