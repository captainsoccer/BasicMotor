package com.basicMotor.motors.sparkBase;

import com.basicMotor.BasicMotor;
import com.basicMotor.configuration.BasicMotorConfig;
import com.basicMotor.configuration.BasicSparkBaseConfig;
import com.basicMotor.controllers.Controller;
import com.basicMotor.gains.ControllerConstraints;
import com.basicMotor.gains.ControllerGains;
import com.basicMotor.gains.currentLimits.CurrentLimits;
import com.basicMotor.gains.currentLimits.CurrentLimitsSparkBase;
import com.basicMotor.gains.PIDGains;
import com.basicMotor.LogFrame;
import com.basicMotor.measurements.Measurements;
import com.basicMotor.measurements.revEncoders.MeasurementsREVAbsolute;
import com.basicMotor.measurements.revEncoders.MeasurementsREVRelative;
import com.basicMotor.motorManager.MotorManager;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.basicMotor.configuration.BasicSparkBaseConfig.AbsoluteEncoderConfig.AbsoluteEncoderRange;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A class that includes common functionality for Spark Base motor controllers
 * (e.g., SparkFlex, SparkMax, etc.).
 * This class assumes that the motor is brushless.
 */
public abstract class BasicSparkBase extends BasicMotor {
    /**
     * The spark base motor instance (SparkFlex, SparkMax, etc.)
     * This is provided by the derived class.
     */
    private final SparkBase motor;
    /**
     * The configuration of the Spark Base motor controller.
     * Stores all the configuration parameters for the motor controller.
     */
    private final SparkBaseConfig config;

    /**
     * The default measurements for the Spark Base motor controller.
     * This is the built-in encoder of any brushless motor connected to the Spark Base motor controller.
     */
    private final Measurements defaultMeasurements;

    /**
     * The idle power draw of the motor controller.
     * This value is used to estimate the power draw of the motor.
     * This value is based on chatGPT's answer to the question about the idle power draw of the Spark MAX motor controller.
     */
    private static final double SPARK_MAX_IDLE_POWER_DRAW = 0.72;

    /**
     * Creates a new Spark Base motor controller with the provided parameters.
     *
     * @param motor          The new instance of the spark base motor controller (SparkFlex, SparkMax, etc.).
     *                       This needs to be a brand-new instance of the motor controller without any configuration.
     * @param config         The empty configuration of the Spark Base motor controller (SparkFlexConfig, SparkMaxConfig, etc.).
     *                       This should be an empty configuration that will be applied to the motor controller.
     * @param gains          The controller gains for the motor controller.
     * @param name           The name of the motor controller (used for logging and debugging).
     * @param gearRatio      The gear ratio of the motor controller.
     * @param unitConversion The conversion factor for the motor's position units.
     *                       This will be multiplied by the motor's rotation to get the position with the desired units.
     */
    public BasicSparkBase(
            SparkBase motor,
            SparkBaseConfig config,
            ControllerGains gains,
            String name,
            double gearRatio,
            double unitConversion) {

        super(gains, name);

        this.motor = motor;
        this.config = config.voltageCompensation(MotorManager.config.motorIdealVoltage); // set the voltage compensation to the idle voltage
        // all configs should be stored in code and not on motor
        applyConfig();

        configurePeriodicFrames(MotorManager.ControllerLocation.MOTOR.getHZ());

        defaultMeasurements =
                new MeasurementsREVRelative(motor.getEncoder(), gearRatio, unitConversion);
    }

    /**
     * Creates a new Spark Base motor controller with the config and motor.
     *
     * @param motor       The new instance of the spark base motor controller (SparkFlex, SparkMax, etc.).
     *                    This needs to be a brand-new instance of the motor controller without any configuration.
     * @param config      The empty configuration of the Spark Base motor controller (SparkFlexConfig, SparkMaxConfig, etc.).
     *                    This should be an empty configuration that will be applied to the motor controller.
     * @param motorConfig The configuration of the motor controller.
     */
    public BasicSparkBase(SparkBase motor, SparkBaseConfig config, BasicMotorConfig motorConfig) {
        super(motorConfig);

        this.motor = motor;

        this.config = config.voltageCompensation(MotorManager.config.motorIdealVoltage); // set the voltage compensation to the idle voltage
        // all configs should be stored in code and not on motor
        applyConfig();

        configurePeriodicFrames(MotorManager.ControllerLocation.MOTOR.getHZ());

        defaultMeasurements = new MeasurementsREVRelative(
                motor.getEncoder(),
                motorConfig.motorConfig.gearRatio,
                motorConfig.motorConfig.unitConversion);

        if (!(motorConfig instanceof BasicSparkBaseConfig sparkBaseConfig)) {
            DriverStation.reportWarning("not using specific Spark Base config for motor: " + name, false);
            return;
        }

        setCurrentLimits(sparkBaseConfig.currentLimitConfig.getCurrentLimits());

        if (sparkBaseConfig.externalEncoderConfig.useExternalEncoder
                && sparkBaseConfig.absoluteEncoderConfig.useAbsoluteEncoder) {
            //spark max (and maybe other future Spark Base motors)
            // cannot use both an absolute encoder and an external encoder at the same time,
            // so we will not configure the encoders and let the derived class handle this
            return;
        }

        // if the motor is using only an absolute encoder, configure it
        if (sparkBaseConfig.absoluteEncoderConfig.useAbsoluteEncoder) {
            var absoluteEncoderConfig = sparkBaseConfig.absoluteEncoderConfig;

            useAbsoluteEncoder(
                    absoluteEncoderConfig.inverted,
                    absoluteEncoderConfig.zeroOffset,
                    absoluteEncoderConfig.sensorToMotorRatio,
                    absoluteEncoderConfig.absoluteEncoderRange);
        }

        // if the motor is using only an external encoder, configure it
        if (sparkBaseConfig.externalEncoderConfig.useExternalEncoder) {
            var externalEncoderConfig = sparkBaseConfig.externalEncoderConfig;

            useExternalEncoder(
                    externalEncoderConfig.inverted,
                    externalEncoderConfig.sensorToMotorRatio,
                    externalEncoderConfig.mechanismToSensorRatio);
        }
    }

    /**
     * Gets the spark base motor controller instance.
     *
     * @return The SparkBase motor controller instance (SparkFlex, SparkMax, etc.).
     */
    public SparkBase getMotor() {
        return motor;
    }

    /**
     * Gets the configuration of the Spark Base motor controller.
     *
     * @return The SparkBaseConfig of the motor controller.
     */
    public SparkBaseConfig getSparkConfig() {
        return config;
    }

    @Override
    protected Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }

    @Override
    protected LogFrame.SensorData getLatestSensorData() {
        double voltage = motor.getBusVoltage();
        double current = motor.getOutputCurrent();
        double temperature = motor.getMotorTemperature();
        double dutyCycle = motor.getAppliedOutput();

        double outputVoltage = motor.getAppliedOutput() * MotorManager.config.motorIdealVoltage;
        double powerOutput = outputVoltage * current;

        double powerDraw = SPARK_MAX_IDLE_POWER_DRAW + powerOutput; // idle power draw + output power
        double currentDraw = powerDraw / voltage; // current draw is power draw / voltage

        return new LogFrame.SensorData(
                temperature,
                currentDraw,
                current,
                outputVoltage,
                voltage,
                powerDraw,
                powerOutput,
                dutyCycle);
    }

    @Override
    protected void setMotorOutput(double setpoint, double feedForward, Controller.ControlMode mode) {
        switch (mode) {
            // stop the motor output
            case STOP -> stopMotorOutput();
            // set the motor output to a voltage
            case VOLTAGE -> motor.setVoltage(setpoint);
            // set the motor output to a duty cycle
            case PRECENT_OUTPUT -> motor.set(setpoint);
            // set the motor output to a position
            case POSITION, PROFILED_POSITION -> setClosedLoopOutput(
                    setpoint, feedForward, SparkBase.ControlType.kPosition);
            // set the motor output to a velocity
            case VELOCITY, PROFILED_VELOCITY -> setClosedLoopOutput(
                    setpoint, feedForward, SparkBase.ControlType.kVelocity);

            case TORQUE, CURRENT -> setClosedLoopOutput(setpoint, 0, SparkBase.ControlType.kCurrent);
        }
    }

    /**
     * Sets the closed loop output for the Spark Base motor controller.
     * This is used only when there is a request that needs pid control and
     * the pid controller is running on the Spark Base motor controller.
     *
     * @param setpoint    The setpoint for the closed loop output.
     * @param feedForward The feed forward value for the closed loop output. (in volts)
     * @param mode        The control type for the closed loop output.
     */
    private void setClosedLoopOutput(double setpoint, double feedForward, SparkBase.ControlType mode) {
        //sets the closed loop output for the Spark MAX motor controller
        var errorSignal = motor.getClosedLoopController().setReference(setpoint, mode, ClosedLoopSlot.kSlot0, feedForward);

        //if there was an error setting the closed loop output, report it
        if (errorSignal != REVLibError.kOk) {
            DriverStation.reportError(
                    "Failed to set closed loop output for Spark MAX motor: "
                            + name
                            + ". Error: "
                            + errorSignal.name(),
                    false);
        }
    }

    @Override
    protected void stopMotorOutput() {
        motor.stopMotor();
    }

    @Override
    protected LogFrame.PIDOutput getPIDLatestOutput() {
        // spark max supports only integral accumulation, so we will return the I accumulator value
        double iAccum = motor.getClosedLoopController().getIAccum();

        //the ideal voltage is used to convert the duty cycle to voltage
        //it is on the same thread as the sensor data so it works fine
        double inputVoltage = logFrame.sensorData.voltageInput();

        //total output is zero because it is incomplete in Spark MAX
        return new LogFrame.PIDOutput(0, iAccum * inputVoltage, 0, 0);
    }

    @Override
    protected void updatePIDGainsToMotor(PIDGains pidGains) {
        var gains = pidGains.convertToDutyCycle();

        // sets the PID gains for the closed loop controller
        config.closedLoop.pid(gains.getK_P(), gains.getK_I(), gains.getK_D());
        config.closedLoop.iZone(gains.getI_Zone());
        config.closedLoop.iMaxAccum(gains.getI_MaxAccum());

        if (gains.getTolerance() != 0 && getControllerLocation() == MotorManager.ControllerLocation.MOTOR) {
            DriverStation.reportWarning(
                    "Spark MAX does not use tolerance in the PID controller (works on rio PID Controller), so it is ignored: "
                            + name,
                    false);
        }

        applyConfig();
    }

    @Override
    protected void updateConstraints(ControllerConstraints constraints) {
        double idealVoltage = MotorManager.config.motorIdealVoltage;

        // sets the max voltage to the max motor output
        config.closedLoop.maxOutput(constraints.getMaxMotorOutput() / idealVoltage);
        config.closedLoop.minOutput(constraints.getMinMotorOutput() / idealVoltage);

        //if the constraints are continuous, ignores them.
        //the rio code handles the continuous constraints.
        if (constraints.getConstraintType() == ControllerConstraints.ConstraintType.LIMITED) {
            // sets the soft limits to the max and min values
            config.softLimit.forwardSoftLimit(constraints.getMaxValue());
            config.softLimit.reverseSoftLimit(constraints.getMinValue());
            // enables the soft limits
            config.softLimit.forwardSoftLimitEnabled(true);
            config.softLimit.reverseSoftLimitEnabled(true);
        } else {
            // disables the soft limits
            config.softLimit.forwardSoftLimitEnabled(false);
            config.softLimit.reverseSoftLimitEnabled(false);
        }

        if (constraints.getVoltageDeadband() != 0 && getControllerLocation() ==  MotorManager.ControllerLocation.MOTOR) {
            DriverStation.reportWarning(
                    "Spark MAX does not use voltage deadband (works on RIO PID controller), so it is ignored: "
                            + name,
                    false);
        }

        applyConfig();
    }

    @Override
    public void setCurrentLimits(CurrentLimits currentLimits) {

        if (currentLimits instanceof CurrentLimitsSparkBase limits) {
            //if both the normal and secondary current limits are 0, do not set any current limits
            if (limits.getCurrentLimit() == 0 && limits.getSecondaryCurrentLimit() == 0) return;

            //if there is a secondary current limit, set it
            if (limits.getSecondaryCurrentLimit() != 0) config.secondaryCurrentLimit(limits.getSecondaryCurrentLimit());

            //if there is a free speed current limit and a stall current limit, set both
            if (limits.getCurrentLimit() != 0 && limits.getStallCurrentLimit() != 0) {
                config.smartCurrentLimit(limits.getStallCurrentLimit(), limits.getCurrentLimit(), limits.getFreeSpeedRPM());
            }
            //if there is only free set it.
            else if (limits.getCurrentLimit() != 0) {
                config.smartCurrentLimit(limits.getCurrentLimit());
            }

        }
        // if the current limits are not REV specific, use the normal current limits
        else {
            //if there are no current limits, do not set any current limits
            if (currentLimits.getCurrentLimit() == 0) return;

            config.smartCurrentLimit(currentLimits.getCurrentLimit());

            DriverStation.reportWarning(
                    "using non Spark Base current limits for motor: " + name, false);
        }

        applyConfig();
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        SparkBaseConfig.IdleMode value =
                switch (mode) {
                    case BRAKE -> SparkBaseConfig.IdleMode.kBrake;
                    case COAST -> SparkBaseConfig.IdleMode.kCoast;
                };

        config.idleMode(value);
        applyConfig();
    }

    @Override
    public void setMotorInverted(boolean inverted) {
        config.inverted(inverted);

        applyConfig();
    }

    @Override
    protected void setMotorPosition(double position) {
        if (!(getMeasurements() instanceof MeasurementsREVRelative encoder)) {
            DriverStation.reportWarning(
                    "motor: "
                            + name
                            + " does not use anymore an encoder that supports setting position,so the position cannot be set",
                    false);
            return;
        }

        encoder.setEncoderPosition(position);

        motor.getClosedLoopController().setIAccum(0);
    }

    @Override
    protected void updateMainLoopTiming(MotorManager.ControllerLocation location) {
        configurePeriodicFrames(location.getHZ());
    }

    /**
     * Configures the periodic frames according to the main loop frequency.
     * Also applies the sensor loop frequency.
     * Note that not all periodic frames will be changed due to the limitations of the Spark base motor controllers.
     * For more information about periodic frames, see the
     * <a href="https://docs.revrobotics.com/brushless/spark-max/control-interfaces#periodic-status-frames">rev website</a>
     *
     * @param mainLoopHZ The frequency of the main loop in Hz.
     */
    private void configurePeriodicFrames(double mainLoopHZ) {
        var signals = config.signals;
        int sensorLoopPeriodMs =
                (int) ((1 / MotorManager.config.SENSOR_LOOP_HZ) * 1000); // convert to milliseconds
        int mainLoopPeriodMs = (int) ((1 / mainLoopHZ) * 1000); // convert to milliseconds

        signals.busVoltagePeriodMs(sensorLoopPeriodMs); // currently does nothing
        signals.motorTemperaturePeriodMs(sensorLoopPeriodMs); // currently does nothing
        signals.iAccumulationPeriodMs(sensorLoopPeriodMs); // currently unknown if it does anything
        signals.outputCurrentPeriodMs(sensorLoopPeriodMs); // currently does nothing

        signals.appliedOutputPeriodMs(mainLoopPeriodMs); // currently does something
        signals.primaryEncoderPositionAlwaysOn(false);
        signals.primaryEncoderVelocityAlwaysOn(false);
        signals.primaryEncoderPositionPeriodMs(mainLoopPeriodMs); // currently does something
        signals.primaryEncoderVelocityPeriodMs(mainLoopPeriodMs); // currently does something

        applyConfig();
    }

    @Override
    protected void stopRecordingMeasurements() {
        var signals = config.signals;

        int maxPeriodMs =
                32767; // maximum period in milliseconds for the Spark MAX according to the documentation
        // check the docs at
        // https://docs.revrobotics.com/brushless/spark-max/control-interfaces#periodic-status-frames

        signals.primaryEncoderPositionPeriodMs(maxPeriodMs);
        signals.primaryEncoderVelocityPeriodMs(maxPeriodMs);

        applyConfig();
    }

    @Override
    protected void startRecordingMeasurements(double HZ) {
        var signals = config.signals;

        int periodMs = (int) ((1 / HZ) * 1000); // convert to milliseconds

        signals.primaryEncoderPositionPeriodMs(periodMs);
        signals.primaryEncoderVelocityPeriodMs(periodMs);

        applyConfig();
    }

    @Override
    protected void setMotorFollow(BasicMotor master, boolean inverted) {
        var motor = (BasicSparkBase) master;

        config.follow(motor.motor, inverted);
        applyConfig();

    }

    @Override
    protected void stopMotorFollow() {
        motor.pauseFollowerMode();
    }

    /**
     * This applies the config file to the spark base motor controller.
     * If there is an error applying the configuration,
     * it will report the error to the driver station.
     */
    private void applyConfig() {
        var okSignal =
                motor.configure(
                        config,
                        SparkBase.ResetMode.kResetSafeParameters,
                        SparkBase.PersistMode.kNoPersistParameters);

        if (okSignal != REVLibError.kOk) {
            DriverStation.reportError(
                    "Failed to apply configuration to Spark MAX motor: "
                            + name
                            + ". Error: "
                            + okSignal.name(),
                    false);
        }
    }


    @Override
    public void setDefaultMeasurements() {
        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);

        int maxPeriodMs =
                32767; // maximum period in milliseconds for the Spark MAX according to the documentation
        // check the docs at
        // https://docs.revrobotics.com/brushless/spark-max/control-interfaces#periodic-status-frames

        config.signals.externalOrAltEncoderVelocity(maxPeriodMs);
        config.signals.externalOrAltEncoderPosition(maxPeriodMs);

        config.signals.absoluteEncoderPositionPeriodMs(maxPeriodMs);
        config.signals.absoluteEncoderVelocityPeriodMs(maxPeriodMs);

        super.setDefaultMeasurements();
    }

    @Override
    protected double getInternalPIDLoopTime(){
        return 0.001; // Spark max and spark flex have a fixed internal PID loop time of 1ms
        // This is according to the documentation:
        // https://docs.revrobotics.com/revlib/spark/closed-loop
    }

    /**
     * Configures the spark base motor controller to use an absolute encoder connected directly to the motor controller.
     *
     * @param inverted             If the absolute encoder is inverted.
     *                             The default positive direction of the encoder is clockwise.
     *                             This is opposite to the default positive direction of a motor.
     * @param zeroOffset           The position the encoder reports that should be considered zero.
     *                             This is affected by the absoluteEncoderRange.
     * @param sensorToMotorRatio   The number which the reading of the absolute encoder should be
     *                             multiplied by to get the motor output.
     *                             This does not include unit conversion.
     *                             This will be larger than 1.0 if the sensor is in a reduction to the motor.
     *                             This is not affected by sensorToMotorRatio.
     * @param absoluteEncoderRange The range that the absolute encoder should report.
     *                             This will be in the encoders rotations.
     *                             (0 to 1, -0.5 to 0.5, etc.)
     */
    public void useAbsoluteEncoder(
            boolean inverted,
            double zeroOffset,
            double sensorToMotorRatio,
            AbsoluteEncoderRange absoluteEncoderRange) {
        // sets the absolute encoder configuration
        setAbsoluteEncoderConfig(inverted, zeroOffset, sensorToMotorRatio, absoluteEncoderRange);
        // sets the feedback sensor for the closed loop controller
        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);

        int periodMs = (int) (MotorManager.ControllerLocation.MOTOR.getSeconds() * 1000); // convert to milliseconds
        // sets the period for the absolute encoder position and velocity
        // (the default encoder period is automatically disabled)
        config.signals.absoluteEncoderPositionPeriodMs(periodMs);
        config.signals.absoluteEncoderVelocityPeriodMs(periodMs);

        // apply the configuration to the motor
        applyConfig();

        // set the measurements to the absolute encoder measurements
        setMeasurements(new MeasurementsREVAbsolute(motor.getAbsoluteEncoder(), 1), false);
    }

    /**
     * Configures the motor controller to use an absolute encoder connected directly to the motor controller.
     *
     * @param inverted           If the absolute encoder is inverted.
     *                           The default positive direction of the encoder is clockwise.
     *                           This is opposite to the default positive direction of a motor.
     * @param zeroOffset         The position the encoder reports that should be considered zero.
     * @param sensorToMotorRatio The number which the reading of the absolute encoder should be
     *                           multiplied by to get the motor output.
     *                           This does not include unit conversion.
     *                           This will be larger than 1.0 if the sensor is in a reduction to the motor.
     */
    public void useAbsoluteEncoder(boolean inverted, double zeroOffset, double sensorToMotorRatio) {
        useAbsoluteEncoder(inverted, zeroOffset, sensorToMotorRatio, AbsoluteEncoderRange.ZERO_TO_ONE);
    }

    /**
     * Sets the configuration for an absolute encoder connected directly to the motor controller.
     * This method does not set the absolute encoder as the feedback sensor.
     * It does not apply the configuration to the motor.
     *
     * @param inverted             If the absolute encoder is inverted.
     *                             The default positive direction of the encoder is clockwise.
     *                             This is opposite to the default positive direction of a motor.
     * @param zeroOffset           The position the encoder reports that should be considered zero.
     *                             This is affected by the absoluteEncoderRange.
     *                             This is the position in rotations.
     * @param sensorToMotorRatio   The number which the reading of the absolute encoder should be
     *                             multiplied by to get the motor output.
     *                             This does not include unit conversion.
     *                             This will be larger than 1.0 if the sensor is in a reduction to the motor.
     * @param absoluteEncoderRange If the encoder should report a range of 0 to 1 or -0.5 to 0.5.
     */
    protected void setAbsoluteEncoderConfig(
            boolean inverted,
            double zeroOffset,
            double sensorToMotorRatio,
            AbsoluteEncoderRange absoluteEncoderRange) {
        // sets whether the absolute encoder is inverted or not
        config.absoluteEncoder.inverted(inverted);
        // sets the conversion factor for the absolute encoder position and velocity
        config.absoluteEncoder.positionConversionFactor(sensorToMotorRatio);
        config.absoluteEncoder.velocityConversionFactor(sensorToMotorRatio);
        // sets the zero centered range of the absolute encoder
        config.absoluteEncoder.zeroCentered(absoluteEncoderRange.zeroCentered());
        // sets the zero offset of the absolute encoder
        config.absoluteEncoder.zeroOffset(zeroOffset);
    }

    /**
     * Configures the motor controller to use an external encoder for the pid loop.
     *
     * @param inverted               If the external encoder is inverted (the position is reversed).
     *                               The default positive direction of the encoder is clockwise.
     *                               This is opposite to the default positive direction of a motor.
     * @param sensorToMotorRatio     The number which the reading of the external encoder should be
     *                               multiplied by to get the motor output.
     *                               A value bigger than 1.0 is a reduction in the motor output.
     *                               (the motor spins more than the sensor)
     * @param unitConversion         The value that will be multiplied by to convert the measurements to the
     *                               desired units.
     *                               This is not affected by sensorToMotorRatio.
     *                               This will be multiplied by the encoders rotations to get the position with the desired units.
     *                               (after mechanismToSensorRatio)
     * @param mechanismToSensorRatio The number that the reading of the external encoder should be
     *                               divided by to get the mechanism output.
     *                               A value bigger than 1.0 is a reduction in the mechanism output.
     *                               Usually this will be 1.0, but if the sensor is mounted not directly on the mechanism.
     */
    public void useExternalEncoder(
            boolean inverted,
            double sensorToMotorRatio,
            double unitConversion,
            double mechanismToSensorRatio) {
        // sets the feedback sensor for the closed loop controller
        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAlternateOrExternalEncoder);

        configExternalEncoder(inverted, sensorToMotorRatio);

        int periodMs = (int) (MotorManager.ControllerLocation.MOTOR.getSeconds() * 1000); // convert to milliseconds
        // sets the period for the absolute encoder position and velocity
        // (the default encoder period is automatically disabled)
        config.signals.externalOrAltEncoderPosition(periodMs);
        config.signals.externalOrAltEncoderVelocity(periodMs);

        // apply the configuration to the motor
        applyConfig();

        // set the measurements to the absolute encoder measurements
        setMeasurements(new MeasurementsREVRelative(getExternalEncoder(), mechanismToSensorRatio, unitConversion), false);
    }

    /**
     * Configures the motor controller to use an external encoder for the pid loop.
     *
     * @param inverted           If the external encoder is inverted (the position is reversed).
     *                           The default positive direction of the encoder is clockwise.
     *                           This is opposite to the default positive direction of a motor.
     * @param sensorToMotorRatio The number which the reading of the external encoder should be
     *                           multiplied by to get the motor output.
     *                           A value bigger than 1.0 is a reduction in the motor output.
     *                           (the motor spins more than the sensor)
     * @param unitConversion     The value that will be multiplied by to convert the measurements to the
     *                           desired units.
     *                           This is not affected by sensorToMotorRatio.
     *                           This will be multiplied by the encoders rotations to get the position with the desired units.
     *                           (after mechanismToSensorRatio)
     */
    public void useExternalEncoder(
            boolean inverted, double sensorToMotorRatio, double unitConversion) {
        useExternalEncoder(inverted, sensorToMotorRatio, unitConversion, 1);
    }

    /**
     * Configures the motor controller to use an external encoder for the pid loop.
     *
     * @param inverted           If the external encoder is inverted (the position is reversed).
     *                           The default positive direction of the encoder is clockwise.
     *                           This is opposite to the default positive direction of a motor.
     * @param sensorToMotorRatio The number which the reading of the external encoder should be
     *                           multiplied by to get the motor output.
     *                           A value bigger than 1.0 is a reduction in the motor output.
     *                           (the motor spins more than the sensor)
     */
    public void useExternalEncoder(boolean inverted, double sensorToMotorRatio) {
        useExternalEncoder(inverted, sensorToMotorRatio, 1);
    }

    /**
     * Configures the motor controller to use an external encoder for the pid loop.
     * This is abstract due to the fact that different Spark Base motor controllers handle external encoders differently.
     *
     * @param inverted              Whether the external encoder is inverted (the position is reversed).
     *                              The default positive direction of the encoder is clockwise.
     *                              This is opposite to the default positive direction of a motor.
     * @param sensorToMotorRatio    The number that the reading of the external encoder should be multiplied by to get the motor output.
     *                              A value bigger than 1.0 is a reduction in the motor output.
     */
    protected abstract void configExternalEncoder(boolean inverted, double sensorToMotorRatio);

    /**
     * Gets the external encoder of the Spark Base motor controller.
     * This will be the relative encoder connected directly to the motor controller.
     * This is abstract due to the fact that different Spark Base motor controllers handle external encoders differently.
     *
     * @return The external encoder of the Spark Base motor controller.
     */
    protected abstract RelativeEncoder getExternalEncoder();
}
