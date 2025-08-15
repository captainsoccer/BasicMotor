package io.github.captainsoccer.basicmotor.ctre.talonfx;

import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.LogFrame;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.controllers.Controller;
import io.github.captainsoccer.basicmotor.ctre.CANcoderMeasurements;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.gains.ControllerGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;
import io.github.captainsoccer.basicmotor.gains.CurrentLimits;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class represents a basic TalonFX motor controller.
 * It extends the BasicMotor class and provides
 * functionality specific to the TalonFX motor controller.
 */
public class BasicTalonFX extends BasicMotor {
    /**
     * The name of the default can bus chain.
     * This is the can bus chain built into the robo rio.
     */
    public static final String defaultCanBusName = "rio";

    /**
     * The TalonFX motor controller instance.
     */
    private final TalonFX motor;
    /**
     * The configuration for the TalonFX motor controller.
     */
    private final TalonFXConfiguration config;

    /**
     * The object that handles the sensors for the TalonFX motor controller.
     */
    private final TalonFXSensors sensors;

    /**
     * The default measurements for the TalonFX motor controller. (built in encoder)
     */
    private final Measurements defaultMeasurements;

    /**
     * The velocity request for the motor controller.
     * Used when using the built-in pid controller of the TalonFX.
     */
    private final VelocityVoltage velocityRequest =
            new VelocityVoltage(0).withEnableFOC(false).withUpdateFreqHz(0);
    /**
     * The position request for the motor controller.
     * Used when using the built-in pid controller of the TalonFX.
     */
    private final PositionVoltage positionRequest =
            new PositionVoltage(0).withEnableFOC(false).withUpdateFreqHz(0);

    /**
     * The voltage request for the motor controller.
     */
    private final VoltageOut voltageRequest = new VoltageOut(0).withUpdateFreqHz(0);

    /**
     * The duty cycle request for the motor controller.
     */
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withUpdateFreqHz(0);

    /**
     * The torque current request for the motor controller.
     * Used when using the built-in FOC controller of the TalonFX.
     */
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0).withUpdateFreqHz(0);

    /**
     * Constructor for the TalonFX motor controller
     *
     * @param controllerGains    The gains for the motor controller.
     * @param id                 The id of the motor controller.
     * @param gearRatio          The gear ratio of the motor.
     *                           the gear ratio is how many rotations of the motor are a rotation of the mechanism
     *                           A value larger than 1 means the motor is geared down,
     *                           e.g., a 10 gear ratio means the motor turns 10 times for every rotation of the mechanism.
     * @param unitConversion     The value that will be multiplied by to convert the measurements to the desired units.
     *                           This will be desired units per rotation.
     *                           This will be multiplied after the gear ratio is applied.
     * @param name               The name of the motor controller (used for logging and debugging).
     */
    public BasicTalonFX(ControllerGains controllerGains, int id, double gearRatio, double unitConversion, String name) {

        super(controllerGains, name);

        motor = new TalonFX(id);
        config = new TalonFXConfiguration();

        applyConfig();

        defaultMeasurements = new TalonFXMeasurements(motor, gearRatio, unitConversion);

        sensors = new TalonFXSensors(motor, super::getControllerLocation);

        motor.optimizeBusUtilization();
    }

    /**
     * Constructor for the TalonFX motor controller
     *
     * @param controllerGains    The gains for the motor controller.
     * @param id                 The id of the motor controller.
     * @param gearRatio          The gear ratio of the motor.
     *                           the gear ratio is how many rotations of the motor are a rotation of the mechanism
     *                           A value larger than 1 means the motor is geared down,
     *                           e.g., a 10 gear ratio means the motor turns 10 times for every rotation of the mechanism.
     * @param name               The name of the motor controller (used for logging and debugging).
     */
    public BasicTalonFX(
            ControllerGains controllerGains,
            int id,
            double gearRatio,
            String name) {
        this(controllerGains, id, gearRatio, 1, name);
    }

    /**
     * Constructor for the TalonFX motor controller.
     * This constructor uses the configuration provided.
     *
     * @param config The configuration for the motor controller.
     */
    public BasicTalonFX(BasicMotorConfig config) {
        super(config);

        boolean isSpecificConfig = config instanceof BasicTalonFXConfig;

        String canbusName = isSpecificConfig ? ((BasicTalonFXConfig) config).canBusName : defaultCanBusName;

        motor = new TalonFX(config.motorConfig.id, canbusName);
        this.config = new TalonFXConfiguration();

        applyConfig();

        defaultMeasurements = new TalonFXMeasurements(motor, config.motorConfig.gearRatio, config.motorConfig.unitConversion);

        sensors = new TalonFXSensors(motor, super::getControllerLocation);

        motor.optimizeBusUtilization();

        if (!isSpecificConfig) return;

        BasicTalonFXConfig specificConfig = (BasicTalonFXConfig) config;

        setCurrentLimits(specificConfig.currentLimitConfig.getCurrentLimits());

        enableFOC(specificConfig.enableFOC);

        enableTimeSync(specificConfig.waitForAllSignals);
    }

    @Override
    protected void updatePIDGainsToMotor(PIDGains pidGains) {
        config.Slot0.kP = pidGains.getK_P();
        config.Slot0.kI = pidGains.getK_I();
        config.Slot0.kD = pidGains.getK_D();

        if (getControllerLocation() == MotorManager.ControllerLocation.MOTOR) {
            // changes made in phoenix 6 api
            // https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/feature-replacements-guide.html#integral-zone-and-max-integral-accumulator

            if (pidGains.getI_MaxAccum() != Double.POSITIVE_INFINITY)
                DriverStation.reportWarning(
                        name
                                + " does not need i max accum when running on motor therefor not used (TalonFX check phoenix 6 docs)",
                        false);

            if (pidGains.getTolerance() != 0)
                DriverStation.reportWarning(
                        name
                                + " does not need tolerance when running on motor therefor not used (TalonFX check phoenix 6 docs)",
                        false);

            if (pidGains.getI_Zone() != Double.POSITIVE_INFINITY)
                DriverStation.reportWarning(
                        name
                                + " does not need i zone when running on motor therefor not used (TalonFX check phoenix 6 docs)",
                        false);
        }

        applyConfig();
    }

    @Override
    protected double getInternalPIDLoopTime() {
        return 0.001; // TalonFX has a fixed internal PID loop time of 1ms
        // This is according to https://www.chiefdelphi.com/t/control-loop-timing-of-various-motor-controllers/370356/4
    }

    @Override
    protected void updateMainLoopTiming(MotorManager.ControllerLocation location) {
        sensors.updateControllerLocation();

        if (getMeasurements() instanceof TalonFXMeasurements measurements) {
            measurements.setUpdateFrequency(location.getHZ());
        }

        if (getMeasurements() instanceof CANcoderMeasurements measurements) {
            measurements.setUpdateFrequency(location.getHZ());
        }
    }

    @Override
    public void setCurrentLimits(CurrentLimits currentLimits) {
        var currentConfig = config.CurrentLimits;

        currentConfig.StatorCurrentLimitEnable = currentLimits.getCurrentLimit() != 0;
        currentConfig.StatorCurrentLimit = currentLimits.getCurrentLimit();

        if (currentLimits instanceof TalonFXCurrentLimits limits) {
            currentConfig.SupplyCurrentLimitEnable = limits.getSupplyCurrentLimit() != 0;

            currentConfig.SupplyCurrentLimit = limits.getSupplyCurrentLimit();
            currentConfig.SupplyCurrentLowerLimit = limits.getSupplyLowerLimit();
            currentConfig.SupplyCurrentLowerTime = limits.getSupplyLowerTime();
        } else {
            //reports a warning if the current limits are not for a TalonFX motor controller
            DriverStation.reportWarning("Using non-TalonFX current limits on TalonFX motor controller: " + super.name, false);

            currentConfig.SupplyCurrentLimitEnable = false;
        }
        // applies the current limits to the motor controller
        applyConfig();
    }

    @Override
    protected void updateConstraintsGainsToMotor(ConstraintsGains constraints) {
        // sets the max voltage to the max motor output
        config.Voltage.PeakForwardVoltage = constraints.getMaxMotorOutput();
        config.Voltage.PeakReverseVoltage = constraints.getMinMotorOutput();

        // sets the max duty cycle to the max motor output (same as voltage)
        config.MotorOutput.PeakForwardDutyCycle =
                constraints.getMaxMotorOutput() / MotorManager.config.motorIdealVoltage;
        config.MotorOutput.PeakReverseDutyCycle =
                constraints.getMinMotorOutput() / MotorManager.config.motorIdealVoltage;

        // sets the voltage deadband to the voltage deadband
        config.MotorOutput.DutyCycleNeutralDeadband =
                constraints.getVoltageDeadband() / MotorManager.config.motorIdealVoltage;

        // sets continuous wrap to false (it is calculated on the rio if needed)
        config.ClosedLoopGeneral.ContinuousWrap = false;

        // checks if it needs to apply soft limits
        if (constraints.getConstraintType() == ConstraintsGains.ConstraintType.LIMITED) {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constraints.getMaxValue();
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constraints.getMinValue();
        } else {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        }

        // applies the config to the motor
        applyConfig();
    }

    @Override
    protected Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }

    @Override
    protected void setMotorOutput(double setpoint, double feedForward, Controller.ControlMode mode) {
        StatusCode error =
                switch (mode) {
                    case POSITION, PROFILED_POSITION -> motor.setControl(
                            positionRequest.withPosition(setpoint).withFeedForward(feedForward));

                    case VELOCITY, PROFILED_VELOCITY -> motor.setControl(
                            velocityRequest.withVelocity(setpoint).withFeedForward(feedForward));

                    case VOLTAGE -> motor.setControl(voltageRequest.withOutput(setpoint));

                    case PRECENT_OUTPUT -> motor.setControl(dutyCycleRequest.withOutput(setpoint));

                    case STOP -> {
                        stopMotorOutput();
                        yield StatusCode.OK; // no error when stopping the motor
                    }

                    case CURRENT, TORQUE -> motor.setControl(torqueCurrentRequest.withOutput(setpoint));

                };

        if (error != StatusCode.OK) {
            DriverStation.reportError(
                    "Failed to set motor output for motor: " + super.name + " Error: " + error.name(), false);
        }
    }

    @Override
    protected void stopMotorOutput() {
        motor.stopMotor();
    }

    @Override
    protected LogFrame.SensorData getLatestSensorData() {
        return sensors.getSensorData();
    }

    @Override
    protected LogFrame.PIDOutput getPIDLatestOutput() {
        return sensors.getPIDLatestOutput();
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        config.MotorOutput.NeutralMode =
                switch (mode) {
                    case COAST -> NeutralModeValue.Coast;
                    case BRAKE -> NeutralModeValue.Brake;
                };

        applyConfig();
    }

    @Override
    public void setMotorInverted(boolean inverted) {
        config.MotorOutput.Inverted =
                inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        applyConfig();
    }

    @Override
    protected void stopRecordingMeasurements() {
        if (getMeasurements() instanceof TalonFXMeasurements measurements) {
            measurements.setUpdateFrequency(0);
        }

        if (getMeasurements() instanceof CANcoderMeasurements measurements) {
            measurements.setUpdateFrequency(0);
        }
    }

    @Override
    protected void startRecordingMeasurements(double HZ) {
        if (getMeasurements() instanceof TalonFXMeasurements measurements) {
            measurements.setUpdateFrequency(HZ);
        }

        if (getMeasurements() instanceof CANcoderMeasurements measurements) {
            measurements.setUpdateFrequency(HZ);
        }
    }

    @Override
    protected void setMotorFollow(BasicMotor master, boolean inverted) {
        BasicTalonFX motor = (BasicTalonFX) master;

        sensors.setDutyCycleToDefaultRate(true);

        Follower follower = new Follower(motor.motor.getDeviceID(), inverted);

        this.motor.setControl(follower);
    }

    @Override
    protected void stopMotorFollow() {
        sensors.setDutyCycleToDefaultRate(false);
        motor.stopMotor();
    }

    /**
     * This enables or disables the Field Oriented Control (FOC) for the TalonFX motor controller.
     * This works only if the motor controller is licensed with Phoenix Pro.
     * If the motor controller is not licensed with Phoenix Pro, it will not have any effect.
     *
     * @param enable Whether to enable or disable FOC.
     */
    public void enableFOC(boolean enable) {
        velocityRequest.EnableFOC = enable;
        positionRequest.EnableFOC = enable;
        voltageRequest.EnableFOC = enable;
        dutyCycleRequest.EnableFOC = enable;
    }

    /**
     * Enables or disables time synchronization for the TalonFX signals.
     * Use this only when the motor controller is licensed with Phoenix Pro and is connected to a canivore.
     * @param enable Whether to enable time synchronization.
     */
    public void enableTimeSync(boolean enable) {
        sensors.setWaitForAll(enable);

        if(!(defaultMeasurements instanceof TalonFXMeasurements measurements)) {
            throw new RuntimeException("Default measurements are not instance of MeasurementsTalonFX");
        }

        measurements.setTimeSync(enable);
    }

    /**
     * Applies the configuration to the motor controller.
     * If the configuration fails to apply, it will report an error to the driver station.
     */
    private void applyConfig() {
        var error = motor.getConfigurator().apply(config);

        if (error != StatusCode.OK) {
            DriverStation.reportError(
                    "Failed to apply config to motor: " + super.name + " Error: " + error.name(), false);
        }
    }

    /**
     * Configures the motor to use a CANcoder as the feedback sensor.
     *
     * @param canCoder               The CANcoder to use as the remote encoder.
     * @param sensorToMotorRatio     The ratio between the sensor and the motor (the value of the can coder to get the motor value).
     * @param unitConversion         The value that will be multiplied by to convert the measurements to the desired units.
     * @param mechanismToSensorRatio The ratio between the mechanism and the sensor (the value of the can coder to get the mechanism value).
     * @param feedbackSensorSource   The feedback sensor source value (RemoteCANcoder or FusedCANcoder).
     */
    private void configureCanCoder(CANcoder canCoder, double sensorToMotorRatio, double unitConversion, double mechanismToSensorRatio, FeedbackSensorSourceValue feedbackSensorSource) {
        if (canCoder == null) {
            DriverStation.reportError("CAN coder is null, cannot use remote encoder", false);
            return;
        }

        config.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
        config.Feedback.RotorToSensorRatio = sensorToMotorRatio;
        config.Feedback.FeedbackSensorSource = feedbackSensorSource;
        applyConfig();

        setMeasurements(new CANcoderMeasurements(canCoder, mechanismToSensorRatio, unitConversion), false);
    }

    /**
     * uses a remote CAN coder as the encoder for the motor controller the user must ensure that the
     * CAN coder is configured correctly and zeroed before using this method.
     * If your motor controller and canCoder are connected to a canivore and you have a Phoenix Pro license,
     * use the {@link #useFusedCanCoder(CANcoder, double, double, double)} method instead.
     *
     * @param canCoder               the CAN coder to use as the remote encoder
     * @param sensorToMotorRatio     this value multiplies the ratio between the sensor and the motor (the
     *                               value of the can coder to get the motor value)
     * @param unitConversion         the value the rotations of the can coder will be multiplied by to convert
     *                               the measurements to the desired units
     * @param mechanismToSensorRatio this value divides the ratio between the mechanism and the sensor
     *                               (the value of the can coder to get the mechanism value)
     */
    public void useRemoteCanCoder(CANcoder canCoder, double sensorToMotorRatio, double unitConversion, double mechanismToSensorRatio) {
        configureCanCoder(canCoder, sensorToMotorRatio, unitConversion, mechanismToSensorRatio, FeedbackSensorSourceValue.RemoteCANcoder);
    }

    /**
     * uses a remote CAN coder as the encoder for the motor controller the user must ensure that the
     * CAN coder is configured correctly and zeroed before using this method.
     * If your motor controller and canCoder are connected to a canivore and you have a Phoenix Pro license,
     * use the {@link #useFusedCanCoder(CANcoder, double, double, double)} method instead.
     *
     * @param canCoder               the CAN coder to use as the remote encoder
     * @param sensorToMotorRatio     this value multiplies the ratio between the sensor and the motor (the
     *                               value of the can coder to get the motor value)
     * @param unitConversion         the value the rotations of the can coder will be multiplied by to convert
     *                               the measurements to the desired units
     */
    public void useRemoteCanCoder(CANcoder canCoder, double sensorToMotorRatio, double unitConversion) {
        useRemoteCanCoder(canCoder, sensorToMotorRatio, unitConversion, 1);
    }

    /**
     * uses a remote CAN coder as the encoder for the motor controller the user must ensure that the
     * CAN coder is configured correctly and zeroed before using this method.
     * If your motor controller and canCoder are connected to a canivore and you have a Phoenix Pro license,
     * use the {@link #useFusedCanCoder(CANcoder, double, double, double)} method instead.
     *
     * @param canCoder               the CAN coder to use as the remote encoder
     * @param sensorToMotorRatio     this value multiplies the ratio between the sensor and the motor (the
     *                               value of the can coder to get the motor value)
     */
    public void useRemoteCanCoder(CANcoder canCoder, double sensorToMotorRatio) {
        useRemoteCanCoder(canCoder, sensorToMotorRatio, 1.0);
    }

    /**
     * Uses a fused CAN coder as the encoder for the motor controller.
     * This method is only available if the motor controller and the CAN coder are connected to a canivore and are pro licensed.
     * The user must ensure that the CAN coder is configured correctly and zeroed before using this method.
     * @param canCoder               the CAN coder to use as the remote encoder
     * @param sensorToMotorRatio     this value multiplies the ratio between the sensor and the motor (the
     *                               value of the can coder to get the motor value)
     * @param unitConversion         the value the rotations of the can coder will be multiplied by to convert
     *                               the measurements to the desired units
     * @param mechanismToSensorRatio this value divides the ratio between the mechanism and the sensor
     *                               (the value of the can coder to get the mechanism value)
     */
    public void useFusedCanCoder(CANcoder canCoder, double sensorToMotorRatio, double unitConversion, double mechanismToSensorRatio){
        configureCanCoder(canCoder, sensorToMotorRatio, unitConversion, mechanismToSensorRatio, FeedbackSensorSourceValue.FusedCANcoder);
    }

    /**
     * Uses a fused CAN coder as the encoder for the motor controller.
     * This method is only available if the motor controller and the CAN coder are connected to a canivore and are pro licensed.
     * The user must ensure that the CAN coder is configured correctly and zeroed before using this method.
     * @param canCoder               the CAN coder to use as the remote encoder
     * @param sensorToMotorRatio     this value multiplies the ratio between the sensor and the motor (the
     *                               value of the can coder to get the motor value)
     * @param unitConversion         the value the rotations of the can coder will be multiplied by to convert
     *                               the measurements to the desired units
     */
    public void useFusedCanCoder(CANcoder canCoder, double sensorToMotorRatio, double unitConversion){
        useFusedCanCoder(canCoder, sensorToMotorRatio, unitConversion, 1);
    }

    /**
     * Uses a fused CAN coder as the encoder for the motor controller.
     * This method is only available if the motor controller and the CAN coder are connected to a canivore and are pro licensed.
     * The user must ensure that the CAN coder is configured correctly and zeroed before using this method.
     * @param canCoder               the CAN coder to use as the remote encoder
     * @param sensorToMotorRatio     this value multiplies the ratio between the sensor and the motor (the
     *                               value of the can coder to get the motor value)
     */
    public void useFusedCanCoder(CANcoder canCoder, double sensorToMotorRatio){
        useFusedCanCoder(canCoder, sensorToMotorRatio, 1);
    }
}
