package io.github.captainsoccer.basicmotor.control;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.captainsoccer.basicmotor.BasicMotorOld;
import io.github.captainsoccer.basicmotor.LogFrame;
import io.github.captainsoccer.basicmotor.errorHandling.ErrorHandler;
import io.github.captainsoccer.basicmotor.gains.ControllerGains;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * This class is used to control the {@link BasicMotorOld}.
 * It handles the PID loop, feedforward, constraints, and profiling of the motor.
 */
public class Controller implements Sendable {

    /**
     * The error handler of the controller.
     * This is used to log errors and warnings of the controller.
     */
    private final ErrorHandler errorHandler;

    /**
     * The gains of the controller.
     * This stores the PID gains, feedforward, constraints, and profile of the controller.
     */
    private final ControllerGains controllerGains;

    /**
     * The PID controller used to calculate the PID output of the controller.
     */
    private final BasicPIDController[] pidController = new BasicPIDController[3];

    /**
     * The controllable control mode for the dashboard.
     * This is used to change the control mode of the controller from the dashboard.
     * only enabled when called from {@link #initSendable(SendableBuilder)}.
     */
    private final SendableChooser<ControlMode> controlModeChooser = new SendableChooser<>();

    /**
     * The latest request of the controller this contains the control mode and the goal.
     * This is set by the user to control the motor.
     */
    private ControlFrame controlFrame = new ControlFrame();
    /**
     * The setpoint of the controller.
     * Most of the time it is the same as the goal of the request,
     * but when using motion profiling it will be calculated each loop until the goal is reached.
     */
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    /**
     * Creates a controller with the given gains
     *
     * @param controllerGains              The gains of the controller
     * @param hasPIDGainsChangeRunnable    The function to run when the PID gains are changed.
     *                                     This function is to flag when the PID gains are changed
     *                                     so the motor can send the updates to the motor controller on a slower thread.
     * @param hasConstraintsChangeRunnable The function to run when the constraints are changed.
     *                                     This function is to flag when the constraints are changed
     *                                     so the motor can send the updates to the motor controller on a slower thread.
     * @param errorHandler                 The error handler of the controller, used to log errors and warnings.
     * @param measurementSupplier          The supplier for the current measurement of the motor, used for resting the
     *                                     controller when control mode changes
     */
    public Controller(
            ControllerGains controllerGains,
            Consumer<Integer> hasPIDGainsChangeRunnable,
            Runnable hasConstraintsChangeRunnable,
            ErrorHandler errorHandler,
            Supplier<Measurements.Measurement> measurementSupplier) {
        this.controllerGains = controllerGains;
        //sets the callbacks for when the PID gains or constraints are changed
        this.controllerGains.setHasPIDGainsChanged(hasPIDGainsChangeRunnable);
        this.controllerGains.setHasConstraintsChanged(hasConstraintsChangeRunnable);

        this.measurementSupplier = measurementSupplier;
        this.errorHandler = errorHandler;

        //creates the PID controller with the given gains
        for (int i = 0; i < pidController.length; i++) {
            this.pidController[i] = new BasicPIDController(this.controllerGains.getPidGains(i));
        }

        // registers the controller with the sendable registry (used when sending to the dashboard)
        SendableRegistry.add(this, errorHandler.name + " Controller");
    }

    /**
     * Gets the controller gains of the controller
     *
     * @return The controller gains of the controller
     */
    public ControllerGains getControllerGains() {
        return controllerGains;
    }

    /**
     * Sets the reference of the controller
     * The request must be non-null and must have a valid control mode and goal.
     *
     * @param request The new request for the controller
     */
    public void setControl(ControlFrame request) {
        Objects.requireNonNull(request);
        Objects.requireNonNull(request.controlMode());
        Objects.requireNonNull(request.goal());

        if (request.slot() < 0 || request.slot() >= pidController.length) {
            throw new IllegalArgumentException("Invalid slot: " + request.slot());
        }

        if (request.controlMode().isProfiled() && !controllerGains.isProfiled(request.slot())) {
            errorHandler.logWarning("Using a profiled control mode without a profile set in the controller gains. using normal request");
        }

        if (request.controlMode() != this.controlFrame.controlMode()) {
            Measurements.Measurement measurement = measurementSupplier.get();
            if (request.controlMode().isVelocityControl()) reset(measurement.velocity(), measurement.acceleration());
            else reset(measurement.position(), measurement.velocity());
        }

        this.controlFrame = request;
    }

    /**
     * Sets the reference of the controller
     *
     * @param setpoint    The new setpoint (the goal if using a profiled control).
     * @param controlMode The mode of control.
     * @param slot        Which slot to use for Feedback and Feedforwards.
     */
    public void setControl(double setpoint, ControlMode controlMode, int slot) {
        setControl(new ControlFrame(setpoint, controlMode, slot, 0));
    }

    /**
     * Sets the reference of the controller
     *
     * @param setpoint    The new setpoint (the goal if using a profiled control).
     * @param controlMode The mode of control.
     */
    public void setControl(double setpoint, ControlMode controlMode) {
        setControl(setpoint, controlMode, 0);
    }

    /**
     * Sets the reference of the controller.
     * This function is used when you want to control the end velocity of the movement.
     * use this function only when using profiled control.
     *
     * @param goal         the new goal
     * @param goalVelocity The setpoint velocity of the controller (used for profiled position and
     *                     velocity)
     * @param controlMode  The control mode of the controller. (must be a profiled control mode)
     * @param slot         Which slot to use for Feedback and Feedforwards.
     */
    public void setControl(double goal, double goalVelocity, ControlMode controlMode, int slot) {
        if (!controlMode.isProfiled()) {
            errorHandler.logWarning("Using a function made for profiled control for a non profiled control mode");
        }

        setControl(new ControlFrame(goal, goalVelocity, controlMode, slot, 0));
    }

    /**
     * Sets the reference of the controller.
     * This function is used when you want to control the end velocity of the movement.
     * use this function only when using profiled control.
     *
     * @param goal         the new goal
     * @param goalVelocity The setpoint velocity of the controller (used for profiled position and
     *                     velocity)
     * @param controlMode  The control mode of the controller. (must be a profiled control mode)
     */
    public void setControl(double goal, double goalVelocity, ControlMode controlMode) {
        setControl(goal, goalVelocity, controlMode, 0);
    }

    /**
     * Sets the reference of the controller.
     * Use this only when using profiled control.
     *
     * @param goal        The new goal for the controller.
     * @param controlMode The control mode of the controller. (must be a profiled control mode)
     * @param slot        Which slot to use for Feedback and Feedforwards.
     */
    public void setControl(TrapezoidProfile.State goal, ControlMode controlMode, int slot) {
        if (!controlMode.isProfiled()) {
            errorHandler.logWarning("Using a function made for profiled control for a non profiled control mode");
        }

        setControl(new ControlFrame(goal, controlMode, slot, 0));
    }

    /**
     * Sets the reference of the controller.
     * Use this only when using profiled control.
     *
     * @param goal        The new goal for the controller.
     * @param controlMode The control mode of the controller. (must be a profiled control mode)
     */
    public void setControl(TrapezoidProfile.State goal, ControlMode controlMode) {
        setControl(goal, controlMode, 0);
    }

    /**
     * Gets the current setpoint of the controller
     * Usually this will be the same as the setpoint set by the user,
     * but if using profiled control it will be calculated each loop until the goal is reached.
     * When using profiled control, use {@link #getGoal()} to get the goal of the controller.
     *
     * @return The current setpoint of the controller
     */
    public TrapezoidProfile.State getSetpoint() {
        return setpoint;
    }

    /**
     * Gets the current setpoint of the controller as a double.
     * This is the position of the setpoint in units of measurement.
     * If using profiled control, this will be the position of the setpoint calculated each loop until the goal is reached.
     *
     * @return The current setpoint of the controller as a double
     */
    public double getSetpointAsDouble() {
        return setpoint.position;
    }

    /**
     * Gets the goal of the controller
     * This is the goal set by the user in the request.
     * If not using profiled control, this will be the same as the setpoint.
     *
     * @return The goal of the controller
     */
    public TrapezoidProfile.State getGoal() {
        return controlFrame.goal();
    }

    /**
     * Gets the goal of the controller as a double.
     * This is the position of the goal in units of measurement.
     * If not using profiled control, this will be the same as the setpoint.
     *
     * @return The goal of the controller as a double
     */
    public double getGoalAsDouble() {
        return controlFrame.goal().position;
    }

    /**
     * Gets the current control mode of the controller.
     *
     * @return The current control mode of the controller
     */
    public ControlMode getControlMode() {
        return controlFrame.controlMode();
    }

    /**
     * Gets the latest request of the controller.
     *
     * @return The current request of the controller
     */
    public ControlFrame getLatestFrame() {
        return controlFrame;
    }

    /**
     * This will reset the PID controller (last error and integral gain)
     * and will reset the motion profile to the current position.
     * This will be called automatically when the robot exists disable to make sure the motion profile won't freak out.
     *
     * @param measurement         The current measurement of the controller (depending on the control mode).
     *                            If Position control is used, this should be the current position of the motor.
     *                            If Velocity control is used, this should be the current velocity of the motor.
     * @param measurementVelocity The current measurement velocity of the controller (used for profiled control).
     *                            If Position control is used, this should be the current velocity of the motor.
     *                            If Velocity control is used, this should be the current acceleration of the motor.
     */
    public void reset(double measurement, double measurementVelocity) {
        for (var pid : pidController) {
            pid.reset();
        }
        this.setpoint = new TrapezoidProfile.State(measurement, measurementVelocity);
    }

    // calculations

    /**
     * Calculates the output of the PID controller.
     * This uses the saved setpoint for calculating the PID output.
     * so make sure to set the setpoint before calling this function.
     * {@link #setSetpointToGoal()} or {@link #calculateProfile(double dt)} to update the setpoint.
     *
     * @param measurement The measurement of the controller. This will change depending on the control mode.
     * @param dt          The time since the last calculation (in seconds), used to calculate the derivative and integral of the error.
     * @return The PID output of the controller. (in volts)
     */
    public LogFrame.PIDOutput calculatePID(double measurement, double dt) {
        return this.pidController[controlFrame.slot()].calculate(this.setpoint.position, measurement, dt);
    }

    /**
     * Sets the setpoint to the goal.
     * This is used when the controller is not using a motion profile.
     * if using a motion profile, use {@link #calculateProfile(double dt)} to update the setpoint.
     */
    public void setSetpointToGoal() {
        this.setpoint = controlFrame.goal();
    }

    /**
     * Calculates the feed forward of the controller.
     * This calculates the outputs of the feed forwards.
     * It uses the saved setpoint for calculating the feed forward output.
     * This includes the arbitrary feed forward set by the user.
     *
     * @param measurement The measurement of the controller (depending on the control mode).
     *                    Used only when using position control for calculating the direction of travel.
     * @return The feed forward of the controller in volts
     */
    public LogFrame.FeedForwardOutput calculateFeedForward(double measurement) {
        var feedForwards = controllerGains.getControllerFeedForwards(controlFrame.slot());

        return feedForwards.calculateFeedForwardOutput(this.setpoint, measurement, controlFrame.controlMode(), controlFrame.arbFeedforward());
    }

    /**
     * Calculates the motion profile of the controller.
     * use this if you are using a profiled control mode.
     * else use {@link #setSetpointToGoal()}.
     * this updates the setpoint to the next position in the profile
     *
     * @param dt The time since the last calculation
     */
    public void calculateProfile(double dt) {
        var profile = this.controllerGains.getMotionProfile(controlFrame.slot());

        setpoint = profile.calculate(dt, setpoint, controlFrame.goal());
    }

    /**
     * Checks the motor output.
     * This checks if the motor output exceeds the max output of the controller.
     * This also checks if the output is above the deadband of the controller.
     *
     * @param output The calculated output of the controller in volts.
     * @return The output after checking the constraints.
     */
    public double checkMotorOutput(double output) {
        return controllerGains.getControllerConstrains().checkMotorOutput(output);
    }

    /**
     * Calculates the constraints on the goal or setpoint of the controller.
     * If using a soft limit or continuous warp, this function will check accordingly.
     *
     * @param measurement The measurement of the controller (depending on the control mode).
     * @param request     The latest request of the controller. (the function will update the goal in the request if needed)
     */
    public void calculateConstraints(
            Measurements.Measurement measurement, ControlFrame controlFrame) {
        this.controllerGains.getControllerConstrains().calculateConstraints(measurement, controlFrame);
    }

    // maintenance things
    @Override
    public void initSendable(SendableBuilder builder) {
        boolean isProfiled = controllerGains.initSendable(builder);

        String setPointName = isProfiled ? "goal" : "setpoint";

        for (ControlMode mode : ControlMode.values()) {
            controlModeChooser.addOption(mode.name(), mode);
        }

        controlModeChooser.setDefaultOption(ControlMode.STOP.name(), ControlMode.STOP);

        //this acts both as the setpoint and the goal of the controller
        builder.addDoubleProperty(setPointName, () -> setpoint.position,
                (value) -> setControl(value, controlModeChooser.getSelected(), controlFrame.slot()));

        SmartDashboard.putData(SendableRegistry.getName(this) + "/controlMode", controlModeChooser);
    }

    /**
     * sets the slot that gets sent to the dashboard.
     * This method needs to be called before sending the controller to the dashboard.
     *
     * @param slot The slot to send to the dashboard
     */
    public void setSendableSlot(int slot) {
        controllerGains.setSendableSlot(slot);
    }
}
