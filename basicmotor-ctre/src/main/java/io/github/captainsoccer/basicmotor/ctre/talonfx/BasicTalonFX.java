package io.github.captainsoccer.basicmotor.ctre.talonfx;

import com.ctre.phoenix6.hardware.TalonFX;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.LogFrame;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.MotorInterface;
import io.github.captainsoccer.basicmotor.controllers.Controller;
import io.github.captainsoccer.basicmotor.ctre.CANcoderMeasurements;
import io.github.captainsoccer.basicmotor.gains.ControllerGains;
import io.github.captainsoccer.basicmotor.gains.CurrentLimits;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import java.util.Objects;

/**
 * This class represents a basic TalonFX motor controller.
 * It extends the BasicMotor class and provides
 * functionality specific to the TalonFX motor controller.
 */
public class BasicTalonFX extends BasicMotor {

    /**
     * The interface that houses the motor itself.
     * It is used to give to the BasicMotor Constructor to create the motor before the child constructor runs.
     */
    private final TalonFXInterface motorInterface;

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
     * @param name               The name of the motor controller (used for logging and debugging).
     * @param id                 The id of the motor controller.
     * @param gearRatio          The gear ratio of the motor.
     *                           the gear ratio is how many rotations of the motor are a rotation of the mechanism
     *                           A value larger than 1 means the motor is geared down,
     *                           e.g., a 10 gear ratio means the motor turns 10 times for every rotation of the mechanism.
     * @param unitConversion     The value that will be multiplied by to convert the measurements to the desired units.
     *                           This will be desired units per rotation.
     *                           This will be multiplied after the gear ratio is applied.
     */
    public BasicTalonFX(ControllerGains controllerGains, String name, int id, double gearRatio, double unitConversion) {
        super(new TalonFXInterface(name, id, gearRatio, unitConversion), controllerGains);

        motorInterface = (TalonFXInterface) super.motorInterface;
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
    public BasicTalonFX(ControllerGains controllerGains, String name, int id, double gearRatio) {
        this(controllerGains, name, id, gearRatio, 1);
    }

    /**
     * Constructor for the TalonFX motor controller.
     * This constructor uses the configuration provided.
     *
     * @param motorConfig The configuration for the motor controller.
     */
    public BasicTalonFX(BasicMotorConfig motorConfig) {
        super(new TalonFXInterface(motorConfig), motorConfig);

        this.motorInterface = (TalonFXInterface) super.motorInterface;

        if(motorConfig instanceof BasicTalonFXConfig talonConfig) {

            setCurrentLimits(talonConfig.currentLimitConfig.getCurrentLimits());

            enableFOC(talonConfig.enableFOC);

            enableTimeSync(talonConfig.waitForAllSignals);
        }
    }

    @Override
    protected void updateMainLoopTiming(MotorManager.ControllerLocation location) {
        motorInterface.sensors.updateControllerLocation(location);

        if (getMeasurements() instanceof TalonFXMeasurements measurements) {
            measurements.setUpdateFrequency(location.getHZ());
        }

        if (getMeasurements() instanceof CANcoderMeasurements measurements) {
            measurements.setUpdateFrequency(location.getHZ());
        }
    }

    @Override
    public void setCurrentLimits(CurrentLimits currentLimits) {
        var currentConfig = motorInterface.config.CurrentLimits;

        currentConfig.StatorCurrentLimitEnable = currentLimits.getCurrentLimit() != 0;
        currentConfig.StatorCurrentLimit = currentLimits.getCurrentLimit();

        if (currentLimits instanceof TalonFXCurrentLimits limits) {
            currentConfig.SupplyCurrentLimitEnable = limits.supplyCurrentLimit() != 0;

            currentConfig.SupplyCurrentLimit = limits.supplyCurrentLimit();
            currentConfig.SupplyCurrentLowerLimit = limits.supplyLowerLimit();
            currentConfig.SupplyCurrentLowerTime = limits.supplyLowerTime();
        } else {
            //reports a warning if the current limits are not for a TalonFX motor controller
            errorHandler.logWarning("Using non-TalonFX current limits on TalonFX motor controller");

            currentConfig.SupplyCurrentLimitEnable = false;
        }
        // applies the current limits to the motor controller
        motorInterface.applyConfig();
    }

    @Override
    protected void setMotorOutput(double setpoint, double feedForward, Controller.ControlMode mode, int slot) {
        var motor = motorInterface.motor;

        StatusCode error =
                switch (mode) {
                    case POSITION, PROFILED_POSITION -> motor.setControl(
                                positionRequest.withPosition(setpoint).withFeedForward(feedForward).withSlot(slot));

                    case VELOCITY, PROFILED_VELOCITY -> motor.setControl(
                                velocityRequest.withVelocity(setpoint).withFeedForward(feedForward).withSlot(slot));

                    case VOLTAGE -> motor.setControl(voltageRequest.withOutput(setpoint));

                    case PERCENT_OUTPUT -> motor.setControl(dutyCycleRequest.withOutput(setpoint));

                    case STOP -> {
                        stopMotorOutput();
                        yield StatusCode.OK; // no error when stopping the motor
                    }

                    case CURRENT, TORQUE -> motor.setControl(torqueCurrentRequest.withOutput(setpoint));
                };

        if (error != StatusCode.OK) {
            errorHandler.logAndReportError("Failed to set motor output, StatusCode: " + error.name());
        }
    }

    @Override
    protected void stopMotorOutput() {
        motorInterface.motor.stopMotor();
    }

    @Override
    protected LogFrame.SensorData getLatestSensorData() {
        return motorInterface.sensors.getSensorData();
    }

    @Override
    protected LogFrame.PIDOutput getPIDLatestOutput() {
        return motorInterface.sensors.getPIDLatestOutput();
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
    protected void setMotorFollow(MotorInterface master, boolean inverted) {
        TalonFXInterface motor = (TalonFXInterface) master;

        motor.sensors.setDutyCycleToDefaultRate(true);

        Follower follower = new Follower(motor.motor.getDeviceID(), inverted);

        this.motorInterface.motor.setControl(follower);
    }

    @Override
    protected void stopMotorFollow() {
       stopMotorOutput();
    }

    @Override
    public void setDefaultMeasurements(){
        motorInterface.config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorInterface.applyConfig();

        super.setDefaultMeasurements();
    }

    /**
     * Gets the TalonFX of this motor.
     * Useful when you need to interact directly with the talonFX
     * @return The talonFX instance of this motor
     */
    public TalonFX getMotor() {
        return motorInterface.motor;
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
        motorInterface.sensors.setWaitForAll(enable);

        motorInterface.getDefaultMeasurements().setTimeSync(enable);
    }

    /**
     * uses a remote CAN coder as the encoder for the motor controller the user must ensure that the
     * CAN coder is configured correctly and zeroed before using this method.
     * If your motor controller and canCoder are connected to a canivore and you have a Phoenix Pro license,
     * use the {@link #useFusedCanCoder(CANcoder, double, double, double)} method instead.
     *
     * @param canCoder               the CAN coder to use as the remote encoder
     * @param unitConversion         the value the rotations of the can coder will be multiplied by to convert
     *                               the measurements to the desired units
     * @param mechanismToSensorRatio this value divides the ratio between the mechanism and the sensor
     *                               (the value of the can coder to get the mechanism value)
     */
    public void useRemoteCanCoder(CANcoder canCoder, double unitConversion, double mechanismToSensorRatio) {
        Objects.requireNonNull(canCoder);

        var config = motorInterface.config;

        config.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motorInterface.applyConfig();

        //because remote canCoder uses the canCoder position directly without
        // conversion we convert the pid gains back to the 1:1 gains
        if(getMeasurements().getGearRatio() != 1){
            for(int i = 0; i < 3; i ++){

                var newGains = getController().getControllerGains().getPidGains(i)
                        .convertToMotorGains(1 / getMeasurements().getGearRatio(), 1);

                getController().getControllerGains().setPidGains(newGains, i);
            }
        }

        setMeasurements(new CANcoderMeasurements(canCoder, mechanismToSensorRatio, unitConversion), false);
    }

    /**
     * uses a remote CAN coder as the encoder for the motor controller the user must ensure that the
     * CAN coder is configured correctly and zeroed before using this method.
     * If your motor controller and canCoder are connected to a canivore and you have a Phoenix Pro license,
     * use the {@link #useFusedCanCoder(CANcoder, double, double, double)} method instead.
     *
     * @param canCoder               the CAN coder to use as the remote encoder
     * @param unitConversion         the value the rotations of the can coder will be multiplied by to convert
     *                               the measurements to the desired units
     */
    public void useRemoteCanCoder(CANcoder canCoder, double unitConversion) {
        useRemoteCanCoder(canCoder, unitConversion, 1);
    }

    /**
     * uses a remote CAN coder as the encoder for the motor controller the user must ensure that the
     * CAN coder is configured correctly and zeroed before using this method.
     * If your motor controller and canCoder are connected to a canivore and you have a Phoenix Pro license,
     * use the {@link #useFusedCanCoder(CANcoder, double, double, double)} method instead.
     *
     * @param canCoder               the CAN coder to use as the remote encoder
     */
    public void useRemoteCanCoder(CANcoder canCoder) {
        useRemoteCanCoder(canCoder, 1.0);
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
        Objects.requireNonNull(canCoder);

        var config = motorInterface.config;

        config.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
        config.Feedback.RotorToSensorRatio = sensorToMotorRatio;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorInterface.applyConfig();

        setMeasurements(new CANcoderMeasurements(canCoder, mechanismToSensorRatio, unitConversion), false);
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
