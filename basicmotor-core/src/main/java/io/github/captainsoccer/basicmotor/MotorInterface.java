package io.github.captainsoccer.basicmotor;

import io.github.captainsoccer.basicmotor.config.*;
import io.github.captainsoccer.basicmotor.config.slots.FeedForwardConfig;
import io.github.captainsoccer.basicmotor.config.slots.MotionProfileConfig;
import io.github.captainsoccer.basicmotor.config.slots.PIDConfig;
import io.github.captainsoccer.basicmotor.config.slots.SlotConfig;
import io.github.captainsoccer.basicmotor.control.ControlFrame;
import io.github.captainsoccer.basicmotor.errorHandling.ErrorHandler;
import io.github.captainsoccer.basicmotor.measurements.Measurements;

import java.util.Objects;

/**
 * This class is an interface for the different motor controllers.
 * It is used in order to initialize and configure the motor controllers.
 * all other functions of the motor controller are handled in the BasicMotor implementation.
 */
public abstract class MotorInterface {

    /**
     * The config of the motor
     */
    private BasicMotorConfig config;

    protected final ErrorHandler errorHandler;

    /**
     * Creates a MotorInterface with the name provided in the configuration.
     *
     * @param config the configuration for the motor
     */
    protected MotorInterface(BasicMotorConfig config) {
        this.config = config;
        errorHandler = new ErrorHandler(config.motorBasics.motorName);
    }

    /**
     * Gets the default measurements for the motor.
     * This is used to get the original source of measurements for the motor.
     *
     * @return The default measurements for the motor.
     */
    public abstract Measurements getDefaultMeasurements();

    /**
     * sets the output of the motor
     * @param controlFrame the output to set
     * @return the answer from the motor (error code if there was)
     */
    public abstract BasicError setMotorOutput(ControlFrame controlFrame);

    /**
     * sets the output of the motor, used by voltage control mode / when running pid on a custom measurements
     * @param volts how many volts to apply
     * @return the answer from the motor (error code if there was)
     */
    public abstract BasicError setMotorOutput(double volts);

    /**
     * logs an error through the error handler
     * @param error the error to log
     * @param printStackTrace if to print the stack trace (use when error is linked to an action that happened
     */
    public void logError(BasicError error, boolean printStackTrace) {
        errorHandler.logError(error.getName(), printStackTrace);
    }

    /**
     * logs an error through the error handaler
     * @param error the error to log
     */
    public void logError(BasicError error) {
        logError(error, false);
    }

    /**
     * checks if the motor is alive and connected to the canbus
     * @return if the motor is alive or not
     */
    public abstract boolean isMotorConnected();

    /**
     * gets the latest data of the sensors of the motor
     * @return the newest data from the motor sensors
     */
    public abstract LogFrame.SensorData getLatestSensorData();

    /**
     * gets the latest pid output of the motor.
     * not supported on all motors, used to log the pid calculations that are done on the motor controller itself.
     * @return the pid output data
     */
    public abstract LogFrame.PIDOutput getLatestPIDOutput();

    /**
     * @return the config of the motor
     */
    public BasicMotorConfig getConfig() {
        return config;
    }

    /**
     * @return the motor basics config
     */
    public MotorBasicsConfig getMotorBasicsConfig() {
        return config.motorBasics;
    }

    /**
     * @return the constraints of the motor
     */
    public ConstraintsConfig getConstraintsConfig() {
        return config.constraints;
    }

    /**
     * @param slot which slot to get
     * @return the slot config
     */
    public SlotConfig getSlotConfig(int slot){
        return switch (slot){
            case 0 -> config.slot0;
            case 1 -> config.slot1;
            case 2 -> config.slot2;
            default -> throw new IllegalStateException("Ivalid slot value: " + slot);
        };
    }

    /**
     * @return gets the follower config
     */
    public FollowerConfig getFollowerConfig(){
        return config.follower;
    }

    /**
     * @param config the config to set
     * @return the message back from the motor, can be an error
     */
    public BasicError setConfig(BasicMotorConfig config) {
        checkBasicMotorConfig(config);
        this.config = config;
        return applyConfig(config);
    }

    /**
     * @param config the config to set
     * @return the message back from the motor, can be an error
     */
    public BasicError setConfig(MotorBasicsConfig config) {
        checkMotorBasicsConfig(config, this.config.motorBasics);
        this.config.motorBasics = config;
        return applyConfig(config);
    }

    /**
     * @param config the config to set
     * @return the message back from the motor, can be an error
     */
    public BasicError setConfig(ConstraintsConfig config) {
        checkConstraintsConfig(config);
        this.config.constraints = config;
        return applyConfig(config);
    }

    /**
     * @param config the config to set
     * @param slot the slot to set
     * @return the message back from the motor, can be an error
     */
    public BasicError setConfig(SlotConfig config, int slot) {
        checkSlotConfig(config, slot);
        switch (slot) {
            case 0:
                this.config.slot0 = config;
            case 1:
                this.config.slot1 = config;
            case 2:
                this.config.slot2 = config;
        }
        return applyConfig(config, slot);
    }

    /**
     * @param config the config to set
     * @return the message back from the motor, can be an error
     */
    protected BasicError setConfig(FollowerConfig config) {
        checkFollowerConfig(config, this.config.motorBasics);
        this.config.follower = config;
        return applyConfig(config);
    }

    /**
     * @param config the config to set
     * @return the message back from the motor, can be an error
     */
    protected BasicError setConfig(LoopTimingConfig config) {
        checkLoopTimingConfig(config);
        this.config.loopTiming = config;
        double measurementsHz = this.config.customMeasurements == null ? config.mainLoopFrequency : 0;
        return applyConfig(measurementsHz, config.sensorLoopFrequency);
    }

    protected abstract BasicError applyConfig(BasicMotorConfig config);

    protected abstract BasicError applyConfig(MotorBasicsConfig config);

    protected abstract BasicError applyConfig(SlotConfig slotConfig, int slot);

    protected abstract BasicError applyConfig(ConstraintsConfig config);

    protected abstract BasicError applyConfig(FollowerConfig config);

    protected abstract BasicError applyConfig(double measurementsHz, double sensorHz);

    private void checkBasicMotorConfig(BasicMotorConfig config) {
        checkConstraintsConfig(config.constraints);

        checkSlotConfig(config.slot0, 0);
        checkSlotConfig(config.slot1, 1);
        checkSlotConfig(config.slot2, 2);

        checkMotorBasicsConfig(config.motorBasics, this.config.motorBasics);

        checkConstraintsConfig(config.constraints);
    }

    private void checkConstraintsConfig(ConstraintsConfig config) {
        if (config.constraint.type() != ConstraintsConfig.ConstraintType.NONE &&
                config.constraint.maxValue() < config.constraint.minValue()) {
            throw new IllegalArgumentException("Invalid constraint values");
        }

        if (config.maxOutput < 0)
            throw new IllegalArgumentException("max output cannot be negative");
        if (config.minOutput > 0)
            config.minOutput = -config.minOutput;

        if (config.deadband < 0)
            throw new IllegalArgumentException("deadband cannot be negative");

        if (config.rampRate < 0)
            throw new IllegalArgumentException("rampRate cannot be negative");
    }

    private void checkSlotConfig(SlotConfig config, int slot) {
        if (slot < 0 || slot > 2) {
            throw new IllegalArgumentException("Invalid slot index");
        }

        checkFFConfig(config.feedforward);
        checkPIDConfig(config.pid);
        checkMotionProfileConfig(config.motionProfile);
    }

    private void checkFFConfig(FeedForwardConfig config) {
        if (config.kA < 0) throw new IllegalArgumentException("kA cannot be negative");
        if (config.kV < 0) throw new IllegalArgumentException("kV cannot be negative");
        if (config.kS < 0) throw new IllegalArgumentException("kS cannot be negative");
        if (config.kSDeadband < 0) throw new IllegalArgumentException("kSDeadband cannot be negative");
    }

    private void checkPIDConfig(PIDConfig config) {
        if (config.kP < 0) throw new IllegalArgumentException("kP cannot be negative");
        if (config.kI < 0) throw new IllegalArgumentException("kI cannot be negative");
        if (config.kD < 0) throw new IllegalArgumentException("kD cannot be negative");
        if (config.iZone < 0) throw new IllegalArgumentException("iZone cannot be negative");
        if (config.IMaxAccum < 0) throw new IllegalArgumentException("IMaxAccum cannot be negative");
        if (config.IMinAccum > 0) config.IMinAccum = -config.IMaxAccum;
        if (config.tolerance < 0) throw new IllegalArgumentException("tolerance cannot be negative");
    }

    private void checkMotionProfileConfig(MotionProfileConfig config) {
        if (config.cruiseVelocity < 0)
            throw new IllegalArgumentException("cruiseVelocity cannot be negative");
        if (config.maxAcceleration < 0)
            throw new IllegalArgumentException("maxAcceleration cannot be negative");
    }

    private void checkMotorBasicsConfig(MotorBasicsConfig config, MotorBasicsConfig oldConfig) {
        if(config.CANID != oldConfig.CANID) throw new IllegalArgumentException("cannot change CAN ID of motor");
        if(!config.CANNetworkName.equals(oldConfig.CANNetworkName)) throw new IllegalArgumentException("cannot change CAN Network Name of motor");
        if(!Objects.equals(config.motorName, oldConfig.motorName)) throw new IllegalArgumentException("cannot change Motor Name of motor");
        if(config.gearRatio != oldConfig.gearRatio) throw new IllegalArgumentException("cannot change gear Ratio of motor");
        if(config.unitConversion != oldConfig.unitConversion) throw new IllegalArgumentException("cannot change unit Conversion of motor");
        if(!config.motorType.equals(oldConfig.motorType)) throw new IllegalArgumentException("cannot change Motor Type of motor");
    }

    private void checkFollowerConfig(FollowerConfig config, MotorBasicsConfig motorBasicsConfig) {
        if(config.masterID < 0 || motorBasicsConfig.CANID == config.masterID)
            throw new IllegalArgumentException("master ID cannot be negative or the same as this motor");
    }

    private void checkLoopTimingConfig(LoopTimingConfig config) {
        if(config.feedbackLoopFrequency < 0)
            throw new IllegalArgumentException("feedbackLoopFrequency cannot be negative");
        if(config.mainLoopFrequency  < 0)
            throw new IllegalArgumentException("mainLoopFrequency cannot be negative");
        if(config.sensorLoopFrequency  < 0)
            throw new IllegalArgumentException("sensorLoopFrequency cannot be negative");
    }
}
