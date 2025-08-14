package io.github.captainsoccer.basicmotor;

import io.github.captainsoccer.basicmotor.measurements.EmptyMeasurements;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;
import io.github.captainsoccer.basicmotor.controllers.Controller;
import io.github.captainsoccer.basicmotor.gains.ControllerConstraints;
import io.github.captainsoccer.basicmotor.gains.ControllerGains;
import io.github.captainsoccer.basicmotor.gains.CurrentLimits;
import io.github.captainsoccer.basicmotor.gains.PIDGains;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager.ControllerLocation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;

import java.util.Objects;

/**
 * The Basic motor base class.
 * It is used to generalize the motor functionality, ease of use and automatic logging.
 * Check the <a href="https://github.com/captainsoccer/BasicMotor/wiki/Usage" >Usage wiki page</a>
 * for more information on how to use this class.
 */
public abstract class BasicMotor {
    /**
     * An enum representing the state of the motor.
     */
    public enum MotorState {
        /**
         * The motor is stopped by command.
         */
        STOPPED,
        /**
         * The motor is running a command.
         */
        RUNNING,
        /**
         * The motor is following another motor.
         */
        FOLLOWING,

        /**
         * The motor is disabled.
         * Means that the motor is stopped due to the robot being disabled.
         */
        DISABLED;
    }

    /**
     * The idle mode of the motor.
     * This is the mode the motor is when not running a command.
     */
    public enum IdleMode {
        /**
         * The motor is in brake mode.
         * Means that the motor will try to hold its position when not running a command.
         */
        BRAKE,
        /**
         * The motor is in coast mode.
         * Means that the motor will not do anything when not running a command.
         */
        COAST
    }

    /**
     * The controller of the motor.
     * This handles constraints, PID control, feedforward, and motion profiles.
     */
    private final Controller controller;

    /**
     * The measurements of the motor.
     * This is the source of the motor's position, velocity, and acceleration.
     * Used for control loops and logging.
     */
    private volatile Measurements measurements;

    /**
     * The log frame of the motor.
     * Updated on different threads and used on the main thread for logging.
     */
    protected final LogFrame.LogFrameAutoLogged logFrame = new LogFrame.LogFrameAutoLogged();

    /**
     * The location of the pid controller.
     * This is used to determine if the pid controller is on the motor or on the RIO.
     */
    private volatile ControllerLocation controllerLocation = ControllerLocation.MOTOR;

    /**
     * The state of the motor.
     */
    private MotorState motorState;

    /**
     * If the PID gains have changed (then it updates the motor controller on the slower thread).
     * The controller updates this value when the PID gains change.
     */
    private volatile boolean hasPIDGainsChanged = false;

    /**
     * Sends the PID gains to the motor controller.
     * This is used to update the PID gains of the motor controller.
     * These PID gains sent should be in motor units (in volts).
     *
     * @param pidGains The PID gains to set on the motor controller.
     */
    protected abstract void updatePIDGainsToMotor(PIDGains pidGains);

    /**
     * If the constraints have changed (then it updates the motor controller on the slower thread).
     * The controller updates this value when the constraints change.
     */
    private volatile boolean hasConstraintsChanged = false;

    /**
     * Sets the constraints of the motor controller.
     * This is used to update the constraints of the motor controller.
     * The constraints sent should be in motor units.
     *
     * @param constraints The constraints to set on the motor controller.
     */
    protected abstract void updateConstraints(ControllerConstraints constraints);

    /**
     * The name of the motor.
     * used for logging and error messages.
     */
    public final String name;

    /**
     * The configuration of the motor controller.
     * Used to store the configuration of the motor controller.
     * May be null if the user goes with a bare minimum configuration.
     */
    private final BasicMotorConfig config;

    /**
     * If the motor has been initialized.
     * This is used due to the motor needing to be initialized after the constructor is called.
     * This will be set true after the motor is initialized.
     * Set by {@link #initializeMotor()}
     */
    private volatile boolean initialized = false;

    /**
     * Creates the motor.
     *
     * @param controllerGains    The gains of the controller.
     * @param name               The name of the motor (used for logging).
     */
    public BasicMotor(ControllerGains controllerGains, String name) {
        // config is null due to the constructor being used for the bare minimum configuration
        this(controllerGains, name, null);
    }

    /**
     * Creates the motor with the given configuration.
     *
     * @param config The configuration for the motor controller.
     */
    public BasicMotor(BasicMotorConfig config) {
        this(config.getControllerGains(), config.motorConfig.name, config);
    }

    /**
     * Creates the motor with the given controller gains, name, and configuration.
     *
     * @param controllerGains    The gains of the controller (used for PID control, feedforward, and constraints).
     * @param name               The name of the motor (used for logging and debugging).
     * @param config             The configuration for the motor controller. (used for idle mode, inverted, and current limits)
     */
    private BasicMotor(ControllerGains controllerGains, String name, BasicMotorConfig config) {
        // checking for null values
        Objects.requireNonNull(controllerGains);
        Objects.requireNonNull(name);

        Runnable setHasPIDGainsChanged = () -> hasPIDGainsChanged = true;
        Runnable setHasConstraintsChanged = () -> hasConstraintsChanged = true;
        controller = new Controller(controllerGains, setHasPIDGainsChanged, setHasConstraintsChanged);

        this.name = name;

        this.config = config != null ? config.copy() : null;

        //register the motor with the motor manager
        MotorManager.getInstance()
                .registerMotor(name, this::run, this::updateSensorData, this::getLatestFrame);
    }

    /**
     * Initializes the motor.
     * Does extra set up like setting the idle mode, inverted, and current limits.
     *
     * @return The default measurements of the motor. (built-in encoder)
     */
    private Measurements initializeMotor() {
        if (initialized) return getDefaultMeasurements();

        // because the motor is on a different thread, it might not be ready yet
        // so it will return null and try again later
        if (getDefaultMeasurements() == null) return null;

        double gearRatio = getDefaultMeasurements().getGearRatio();
        double unitConversion = getDefaultMeasurements().getUnitConversion();

        var controllerGains = controller.getControllerGains();

        updatePIDGainsToMotor(controllerGains.getPidGains().convertToMotorGains(gearRatio, unitConversion, getInternalPIDLoopTime()));

        updateConstraints(controllerGains.getControllerConstrains().convertToMotorConstraints(gearRatio, unitConversion));

        // if the user uses a bare minimum configuration, then we will not set the idle mode, inverted, or current limits
        if (config == null) {
            initialized = true;
            return getDefaultMeasurements();
        }

        setIdleMode(config.motorConfig.idleMode);
        setMotorInverted(config.motorConfig.inverted);

        initialized = true;

        // if the user is using an external encoder, sets the measurements to the external encoder
        return config.usingExternalEncoder() ? getMeasurements() : getDefaultMeasurements();
    }

    // getters and setters

    /**
     * Sets a new measurements source for the motor.
     * Use this when you want the motor to use an external measurements source.
     * Check the specific motor documentation to check if it supports on motor external measurements source.
     * This effects the motor only if the pid controller is on the robo rio {@link ControllerLocation#RIO}.
     * If you regret setting the measurements, you can call {@link #setDefaultMeasurements()} to reset it to the default measurements.
     *
     * @param measurements The new measurements source for the motor.
     */
    public void setMeasurements(Measurements measurements) {
        setMeasurements(measurements, true);
    }

    /**
     * Sets a new measurements source for the motor.
     * Use this when you want the motor to use an external measurements source.
     * Check the specific motor documentation to check if it supports on motor external measurements source.
     * This effects the motor only if the pid controller is on the robo rio {@link ControllerLocation#RIO}.
     * If you regret setting the measurements, you can call {@link #setDefaultMeasurements()} to reset it to the default measurements.
     *
     * @param measurements The new measurements source for the motor.
     * @param throughRIO If the measurements should be set through the RIO or directly on the motor controller.
     */
    protected void setMeasurements(Measurements measurements, boolean throughRIO){
        stopRecordingMeasurements();
        this.measurements = measurements;

        if(throughRIO)
            setControllerLocation(ControllerLocation.RIO);
        else
            setControllerLocation(ControllerLocation.MOTOR);
    }

    /**
     * Sets the motor to use the default measurements source.
     * This is used to reset the measurements to the default measurements of the motor.
     */
    public void setDefaultMeasurements() {
        this.measurements = getDefaultMeasurements();
        startRecordingMeasurements(controllerLocation.getHZ());

        setControllerLocation(ControllerLocation.MOTOR);
    }

    /**
     * Gets the default measurements of the motor.
     * This will be the motor's built-in encoder or a simulated encoder.
     * The motor should store this source in its class.
     *
     * @return The default measurements of the motor.
     */
    protected abstract Measurements getDefaultMeasurements();

    /**
     * Gets the current measurements source of the motor.
     *
     * @return The current measurements source of the motor.
     */
    public Measurements getMeasurements() {
        return measurements;
    }

    /**
     * Gets the latest recorded measurement of the motor.
     * This is updated with the main loop of the motor.
     *
     * @return The latest recorded measurement of the motor.
     */
    public Measurements.Measurement getMeasurement() {
        return measurements.getMeasurement();
    }

    /**
     * Gets the controller of the motor.
     * You can use this to set a command directly to the controller, or send it to the dashboard.
     *
     * @return The controller of the motor.
     */
    public Controller getController() {
        return controller;
    }

    /**
     * Gets the loop time of the internal PID loop.
     * This is used to convert the pid gains to the motor controller's loop time.
     * @return The loop time of the internal PID loop in seconds.
     */
    protected abstract double getInternalPIDLoopTime();

    /**
     * Gets the latest log frame of the motor.
     * The frame is updated on multiple threads, so some of the data might be desynchronized.
     *
     * @return The latest log frame of the motor.
     */
    private LogFrame.LogFrameAutoLogged getLatestFrame() {
        return logFrame;
    }

    /**
     * Stops the motor.
     * Sets a new command to the controller to stop the motor.
     */
    public void stop() {
        controller.setControl(new Controller.ControllerRequest());
    }

    /**
     * Sets the current limits of the motor.
     * This will apply the current limits to the motor controller.
     * You should use a current limits object that is compatible with the motor controller.
     *
     * @param currentLimits The new current limits for the motor.
     */
    public abstract void setCurrentLimits(CurrentLimits currentLimits);

    /**
     * Sets the idle mode of the motor.
     *
     * @param mode The idle mode to set the motor to.
     */
    public abstract void setIdleMode(IdleMode mode);

    /**
     * Sets if the motor should be inverted.
     * The motor's default positive direction is counter-clockwise.
     * This can be changed by setting this function to true.
     *
     * @param inverted If the motor should be inverted (true for inverted, false for normal).
     */
    public abstract void setMotorInverted(boolean inverted);

    /**
     * Sets the new position of the motor.
     * This should be in the motor rotations (before gear ratio and unit conversion).
     *
     * @param position The new position of the motor (in motor rotations).
     */
    private void setMotorPosition(double position){
        if (measurements == null) {
            throw new IllegalStateException("Motor measurements are not initialized. Call setMeasurements() first.");
        }
        measurements.setPosition(position);
    };

    /**
     * Used when there is no need to record the motors built in measurements.
     * If the motor needs to record measurements again, should call {@link #startRecordingMeasurements(double)}.
     */
    protected abstract void stopRecordingMeasurements();

    /**
     * Starts recording the measurements of the motor.
     * This will make the motor record its built-in measurements.
     *
     * @param HZ The frequency of the measurements in Hz. (should be the main thread frequency)
     */
    protected abstract void startRecordingMeasurements(double HZ);

    /**
     * Gets the location of the PID controller.
     * This is used to determine if the PID controller is on the motor or on the RIO.
     * @return The location of the PID controller.
     */
    public ControllerLocation getControllerLocation(){
        return controllerLocation;
    }

    /**
     * Sets the location of the PID controller.
     * This will change the location of the PID controller and update the motor manager.
     * This will change the main loop frequency of the motor.
     *
     * @param controllerLocation The new location of the PID controller (RIO or motor controller).
     */
    public void setControllerLocation(ControllerLocation controllerLocation){
        if (controllerLocation == null) {
            throw new IllegalArgumentException("Controller location cannot be null");
        }

        // if the controller location is the same as the current one, then we don't need to do anything
        if(controllerLocation == this.controllerLocation) return;

        this.controllerLocation = controllerLocation;
        MotorManager.getInstance().setControllerLocation(name, controllerLocation);

        updateMainLoopTiming(controllerLocation);
    }

    /**
     * Updates the main loop timing of the motor.
     * This is used so each motor implementation can update its main loop timing.
     * This is called when the controller location is updated
     * @param location The new location of the PID controller (RIO or motor controller).
     */
    protected abstract void updateMainLoopTiming(ControllerLocation location);

    /**
     * Starts following another motor.
     * This will disable the pid control on this motor and stop recording measurements.
     * This works only for motors of the same type (e.g. SparkMax, TalonFX, etc.).
     *
     * @param master   The motor to follow.
     * @param inverted If the motor should be inverted (opposite direction of the master).
     *                 Used for example when both of the motors are connected to one gear.
     */
    public void followMotor(BasicMotor master, boolean inverted) {
        // if it's the same motor, then we don't need to do anything
        if (master == this) {
            return;
        }

        if (this.getClass() != master.getClass()) {
            throw new IllegalArgumentException("Cannot follow a motor of a different type");
        }

        motorState = MotorState.FOLLOWING;
        setMotorFollow(master, inverted);
        stopRecordingMeasurements();
    }

    /**
     * Stops following another motor.
     * This is used if the user no longer wants the motor to follow another motor.
     */
    public void stopFollowing() {
        motorState = MotorState.STOPPED;
        stopMotorFollow();
        stopMotorOutput();
        startRecordingMeasurements(controllerLocation.getHZ());
    }

    /**
     * Sets the motor to follow another motor.
     * This will be implemented by the specific motor type.
     *
     * @param master   The motor to follow.
     * @param inverted If the motor should be inverted (opposite direction of the master).
     */
    protected abstract void setMotorFollow(BasicMotor master, boolean inverted);

    /**
     * Stops the motor from following another motor.
     * This will be implemented by the specific motor type.
     */
    protected abstract void stopMotorFollow();

    // forwarded functions (for ease of use)

    /**
     * Sets a new control request for the motor.
     * This will send a command to the motor.
     *
     * @param request The control request to set on the motor.
     */
    public void setControl(Controller.ControllerRequest request) {
        controller.setControl(request);
    }

    /**
     * Sets the control of the motor.
     * This will send a command to the motor with the given setpoint and mode.
     *
     * @param setpoint The setpoint of the motor (units depending on the mode).
     * @param mode     The control mode of the motor (position, velocity, voltage, percent output).
     */
    public void setControl(double setpoint, Controller.ControlMode mode) {
        controller.setControl(setpoint, mode);
    }

    /**
     * Sets the control of the motor with an arbitrary feed forward.
     *
     * @param setpoint             The setpoint of the motor (units depending on the mode).
     * @param mode                 The control mode of the motor (position, velocity, voltage, percent output).
     * @param arbitraryFeedForward The arbitrary feed forward to apply to the motor output.
     */
    public void setControl(double setpoint, Controller.ControlMode mode, double arbitraryFeedForward) {
        controller.setControl(new Controller.ControllerRequest(setpoint, mode, arbitraryFeedForward));
    }

    /**
     * Sets a command to output a specific voltage to the motor.
     * This is the same as any of the setControl methods, but specifically for voltage control.
     *
     * @param volts The voltage to output to the motor.
     */
    public void setVoltage(double volts) {
        controller.setControl(volts, Controller.ControlMode.VOLTAGE);
    }

    /**
     * Sets the motor to output a percentage of the maximum output.
     * This is also known as duty cycle control.
     * This is the same as any of the setControl methods, but specifically for percent output.
     *
     * @param percentOutput The percentage of the maximum output to set the motor to.
     */
    public void setPrecentOutput(double percentOutput) {
        controller.setControl(percentOutput, Controller.ControlMode.PRECENT_OUTPUT);
    }

    /**
     * Gets the current position of the motor.
     * This will change according to the gear ratio and unit conversion.
     * Default units are rotations.
     *
     * @return The current position of the motor.
     */
    public double getPosition() {
        return measurements.getMeasurement().position();
    }

    /**
     * Gets the current velocity of the motor.
     * This will change according to the gear ratio and unit conversion.
     * Default units are rotations per second.
     *
     * @return the current velocity of the motor
     */
    public double getVelocity() {
        return measurements.getMeasurement().velocity();
    }

    /**
     * Gets the latest sensor data from the motor.
     * The sensor data holds the following data:
     * <ul>
     *   <li>output current (Amps)</li>
     *   <li>current draw (Amps)</li>
     *   <li>voltage input (Volts)</li>
     *   <li>voltage output (Volts)</li>
     *   <li>power output (Watts)</li>
     *   <li>power draw (Watts)</li>
     *   <li>temperature (°C)</li>
     *   <li>duty cycle (ratio)</li>
     * </ul>
     * @return The latest sensor data from the motor.
     */
    public LogFrame.SensorData getSensorData() {
        return logFrame.sensorData;
    }

    /**
     * If the motor is at the requested setpoint.
     * This will check if the error is within the defined tolerance of the controller.
     * This is only impossible if the motor is using a closed loop control mode (like position or velocity control).
     * If using a motion profile, use {@link #atGoal()} to check if the motor is at the goal.
     *
     * @return True if the motor is at the setpoint, false otherwise.
     */
    public boolean atSetpoint() {
        return logFrame.atSetpoint;
    }

    /**
     * If the motor is at the goal of the motion profile.
     * This will check if the motor is at the goal of the motion profile.
     * This is only relevant if the motor is using a motion profile control mode.
     * If not using a profiled control mode, this will return the same as {@link #atSetpoint()}.
     *
     * @return True if the motor is at the goal of the motion profile, false otherwise.
     */
    public boolean atGoal() {
        return logFrame.atGoal;
    }

    /**
     * Resets the motors position to a specific value.
     * This should be in the units the user configured the motor to use.
     *
     * @param newPosition The new position of the motor.
     */
    public void resetEncoder(double newPosition) {
        if (controller.getControlMode().isVelocityControl())
            controller.reset(0, 0);
        else
            controller.reset(newPosition, 0);

        setMotorPosition((newPosition / measurements.getUnitConversion()) * measurements.getGearRatio());
    }

    /**
     * Resets the motors encoder to a specific position and velocity.
     * This should be used if the motor is using a motion profile.
     * This should be in the units the user configured the motor to use.
     *
     * @param newPosition The new position of the motor
     * @param newVelocity The new velocity of the motor
     */
    public void resetEncoder(double newPosition, double newVelocity) {
        if (!controller.getControlMode().isVelocityControl()) controller.reset(newPosition, newVelocity);
        else controller.reset(newVelocity, 0);

        setMotorPosition((newPosition / measurements.getUnitConversion()) * measurements.getGearRatio());
    }

    // periodic functions

    /**
     * This is the main loop of the motor.
     * It handles taking measurements, checking the constraints, calculating the feedforward,
     * calculating a motion profile, and calculating the PID output (if needed).
     */
    private void run() {
        // if the motor is not initialized, then we need to initialize it
        if (!initialized) {
            measurements = initializeMotor();
            // if the measurements are still null, it means the motor has not been initialized yet
            if (measurements == null) return;
        }

        // doesn't need to do anything if the motor is following another motor
        if (motorState == MotorState.FOLLOWING) {
            return;
        }

        // updates the measurements
        var measurement = updateMeasurements();
        logFrame.measurement = measurement;

        // checks if the motor is stopped or needs to be stopped
        boolean shouldRun = shouldRunLoop(controller.getControlMode(), measurement);
        if (shouldRun) motorState = MotorState.RUNNING;
        else return;

        //calculates the motor output
        var motorOutput =
                runController(measurement, controllerLocation.getSeconds(), controller.getRequest());

        double tolerance = controller.getControllerGains().getPidGains().getTolerance();

        boolean atSetpoint = Math.abs(motorOutput.error()) <= tolerance;
        logFrame.atSetpoint = atSetpoint;

        if (motorOutput.mode().isProfiled()) {
            // checking if the goal is the same as the setpoint
            logFrame.atGoal = (motorOutput.goal() == motorOutput.setpoint()) && atSetpoint;
        }
        // if not using a motion profile, then the atGoal is the same as atSetpoint
        else logFrame.atGoal = atSetpoint;

        // updates the log frame with the motor output
        logFrame.controllerFrame = motorOutput;
        // sets the motor output
        if (controllerLocation == ControllerLocation.RIO)
            // all the pid and feedforward outputs are already calculated in the controller frame
            setMotorOutput(motorOutput.totalOutput(), 0, Controller.ControlMode.VOLTAGE);
        else
            setMotorOutput(
                    checkMotorSetpoint(motorOutput),
                    motorOutput.feedForwardOutput().totalOutput(),
                    motorOutput.mode());
    }

    /**
     * Checks the motor setpoint based on the controller frame.
     * This will convert the setpoint to the motor units if needed.
     * @param controllerFrame The controller frame to check the setpoint from.
     * @return The checked motor setpoint in motor units.
     */
    private double checkMotorSetpoint(LogFrame.ControllerFrame controllerFrame) {
        if (!controllerFrame.mode().requiresPID())
            return controllerFrame.setpoint();

        if (controllerFrame.mode().isCurrentControl()){
            if (controllerFrame.mode() == Controller.ControlMode.CURRENT)
                return controllerFrame.setpoint();
            else
                return getCurrentFromTorque(controllerFrame.setpoint());
        }

        return (controllerFrame.setpoint() / measurements.getUnitConversion()) * measurements.getGearRatio();
    }

    /**
     * Takes the latest measurements of the motor.
     *
     * @return The updated measurements of the motor.
     */
    private Measurements.Measurement updateMeasurements() {
        // checks if the measurements are null, if so, then it will try to get the default measurements
        if (measurements == null) {
            measurements = getDefaultMeasurements();
        }

        if(measurements == null){
            // if the measurements are still null, then we cannot update the measurements
            return Measurements.Measurement.EMPTY;
        }

        return measurements.update(controllerLocation.getSeconds());
    }

    /**
     * Checks if the motor needs to run the main loop.
     * This checks for disabled state, stopped state, and if the controller is not in stop mode.
     *
     * @param controlMode The control mode of the controller.
     * @return True if the motor should run the loop, false otherwise.
     */
    private boolean shouldRunLoop(Controller.ControlMode controlMode, Measurements.Measurement measurement) {
        // if the robot is disabled, then we need to stop the motor
        if (RobotState.isDisabled()) {
            if (motorState == MotorState.DISABLED) {
                // if the motor is already disabled, then we don't need to do anything
                return false;
            }
            stopMotorOutput();
            motorState = MotorState.DISABLED;
            return false;
        }

        // If the measurements are empty and the control mode requires PID control,
        // then we cannot run the controller.
        if((controlMode.requiresPID() && !controlMode.isCurrentControl())&& measurements instanceof EmptyMeasurements){
            DriverStation.reportError("Using empty measurements with a controller that requires PID control. " +
                    "Please set the measurements to a valid source.", false);

            return false;
        }

        // if the robot is enabled but the motor is disabled, then we need to reset the controller
        if (motorState == MotorState.DISABLED && controlMode != Controller.ControlMode.STOP) {
            if (controlMode.isVelocityControl()) controller.reset(measurement.velocity(), measurement.acceleration());
            else controller.reset(measurement.position(), measurement.velocity());
            // continue running the loop
            return true;
        }

        // if the request type is stop, then we need to stop the motor
        if (controlMode == Controller.ControlMode.STOP) {
            if (motorState != MotorState.STOPPED) {
                motorState = MotorState.STOPPED;
                stopMotorOutput();
                logFrame.controllerFrame = LogFrame.ControllerFrame.EMPTY;
                logFrame.pidOutput = LogFrame.PIDOutput.EMPTY;
            }
            return false;
        }

        return true;
    }

    /**
     * Runs the controller for the motor.
     * This will calculate the constraints, feedforward, and PID output (if needed).
     *
     * @param measurement       The latest measurement of the motor.
     * @param dt                The time since the last update in seconds.
     * @param controllerRequest The latest controller request for the motor.
     * @return The controller frame with the calculated outputs.
     */
    private LogFrame.ControllerFrame runController(
            Measurements.Measurement measurement,
            double dt,
            Controller.ControllerRequest controllerRequest) {

        var controlMode = controllerRequest.controlMode();

        // cheks if motor is within the constraints
        controller.calculateConstraints(measurement, controllerRequest);
        double goal = controllerRequest.goal().position;

        // if the controller is not using PID, we can just set the output directly
        if (!controlMode.requiresPID()) {
            double output = controlMode == Controller.ControlMode.VOLTAGE
                            ? goal
                            // estimates the duty cycle output
                            : goal * logFrame.sensorData.voltageInput();

            return new LogFrame.ControllerFrame(
                    output,
                    goal,
                    controlMode);
        }

        // if the controller is using current control, then we need to calculate the current output
        if (controlMode.isCurrentControl()){
            double currentOutput = logFrame.sensorData.currentOutput();

            // if using torque control, converts the current output to torque
            double referenceMeasurement = controlMode == Controller.ControlMode.CURRENT ?
                    currentOutput :
                    getTorqueFromCurrent(currentOutput);


            return new LogFrame.ControllerFrame(
                    logFrame.sensorData.voltageOutput(),
                    LogFrame.FeedForwardOutput.EMPTY,
                    goal,
                    referenceMeasurement,
                    goal - referenceMeasurement,
                    goal,
                    controlMode
            );
        }

        // if using a motion profile, then calculate the profile
        if (controlMode.isProfiled()) controller.calculateProfile(dt);
        else controller.setSetpointToGoal();

        double setpoint = controller.getSetpointAsDouble();

        // calculate the direction of travel
        double directionOfTravel =
                Math.signum(
                        controlMode.isPositionControl()
                                ? setpoint - measurement.position()
                                : setpoint);

        // calculate the feedforward output
        var FFOutput = controller.calculateFeedForward(directionOfTravel);

        // calculates the error of the controller
        double referenceMeasurement =
                controlMode.isVelocityControl()
                        ? measurement.velocity()
                        : measurement.position();

        double error = setpoint - referenceMeasurement;

        double totalOutput;
        if(controllerLocation == ControllerLocation.MOTOR){
            var pidOutput = logFrame.pidOutput;
            totalOutput = pidOutput.totalOutput() + FFOutput.totalOutput();
        }
        else{
            var pidOutput = controller.calculatePID(referenceMeasurement, dt);
            logFrame.pidOutput = pidOutput;
            totalOutput = controller.checkMotorOutput(FFOutput.totalOutput() + pidOutput.totalOutput());
        }

        // returns the combined controller frame and cheks the motor output
        return new LogFrame.ControllerFrame(
                totalOutput,
                FFOutput,
                setpoint,
                referenceMeasurement,
                error,
                goal,
                controlMode);
    }

    /**
     * Converts the motor torque to current.
     * This uses the config to convert the torque to current.
     * If the config is null, it will report an error to the driver station.
     * @param torque The torque to convert to current.
     * @return The current in amps that corresponds to the given torque.
     */
    private double getCurrentFromTorque(double torque) {
        if (config == null) {
            DriverStation.reportError("motor: " + name + " Trying to convert torque and current without config set.", false);
            return 0;
        }
        return config.motorConfig.motorType.getCurrent(torque);
    }

    /**
     * Converts the current to motor torque.
     * This uses the config to convert the current to torque.
     * If the config is null, it will report an error to the driver station.
     *
     * @param current The current in amps to convert to torque.
     * @return The torque in motor units that corresponds to the given current.
     */
    private double getTorqueFromCurrent(double current) {
        if (config == null) {
            DriverStation.reportError("motor: " + name + " Trying to convert torque and current without config set.", false);
            return 0;
        }
        return config.motorConfig.motorType.getTorque(current);
    }

    /**
     * Sets the motor output.
     * This is used to directly set the output of the motor.
     *
     * @param setpoint    The setpoint of the motor needs to be in the motor units (before gear ratio and unit conversion).
     *                    This will depend on the control mode of the motor.
     * @param feedForward The voltage feedforward to apply to the motor output.
     *                    This is used only when the pid controller is on the motor controller.
     * @param mode        The control mode of the motor.
     */
    protected abstract void setMotorOutput(double setpoint, double feedForward, Controller.ControlMode mode);

    /**
     * Stops the motor output.
     */
    protected abstract void stopMotorOutput();

    /**
     * Gets the latest sensor data from the motor.
     * This will update the sensor data in the log frame.
     * Also, will apply the latest pid gains and constraints if they have changed.
     * If the pid controller is on the motor controller, it will also update the pid output in the log frame.
     */
    private void updateSensorData() {
        if (!initialized) return;
        logFrame.sensorData = getLatestSensorData();

        if (controllerLocation == ControllerLocation.MOTOR) logFrame.pidOutput = getPIDLatestOutput();

        if(config != null){
            logFrame.appliedTorque = getTorqueFromCurrent(logFrame.sensorData.currentOutput());
        }

        // if the pid has changed, then update the built-in motor pid
        if (hasPIDGainsChanged) {
            hasPIDGainsChanged = false;

            var convertedGains =
                    controller
                            .getControllerGains()
                            .getPidGains()
                            .convertToMotorGains(measurements.getGearRatio(), measurements.getUnitConversion(), getInternalPIDLoopTime());

            updatePIDGainsToMotor(convertedGains);
        }

        // if the constraints have changed, then update the built-in motor pid
        if (hasConstraintsChanged) {
            hasConstraintsChanged = false;

            double gearRatio = getDefaultMeasurements().getGearRatio();
            double unitConversion = getDefaultMeasurements().getUnitConversion();

            var convertedConstraints =
                    controller
                            .getControllerGains()
                            .getControllerConstrains()
                            .convertToMotorConstraints(gearRatio, unitConversion);

            updateConstraints(convertedConstraints);
        }
    }

    /**
     * Gets the latest sensor data from the motor.
     *
     * @return The latest sensor data from the motor.
     */
    protected abstract LogFrame.SensorData getLatestSensorData();

    /**
     * Gets the latest PID output from the motor.
     * This should be used only when the pid controller is on the motor controller.
     * This would be called in the {@link #updateSensorData()} method
     *
     * @return The latest PID output from the motor.
     */
    protected abstract LogFrame.PIDOutput getPIDLatestOutput();
}
