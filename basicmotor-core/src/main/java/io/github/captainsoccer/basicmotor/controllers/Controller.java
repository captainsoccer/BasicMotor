package io.github.captainsoccer.basicmotor.controllers;

import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.LogFrame;
import io.github.captainsoccer.basicmotor.errorHandling.ErrorHandler;
import io.github.captainsoccer.basicmotor.gains.ControllerGains;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import io.github.captainsoccer.basicmotor.gains.FeedForwardsGains;

import java.util.Objects;
import java.util.function.Consumer;

/**
 * This class is used to control the {@link BasicMotor}.
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
     * The controlled mode used by the pid controller on the smartdashboard.
     */
    private int sendableControlMode = 0;

    /**
     * The latest request of the controller this contains the control mode and the goal.
     * This is set by the user to control the motor.
     */
    private ControllerRequest request = new ControllerRequest();
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
     */
    public Controller(
            ControllerGains controllerGains,
            Consumer<Integer> hasPIDGainsChangeRunnable,
            Runnable hasConstraintsChangeRunnable,
            ErrorHandler errorHandler) {
        this.controllerGains = controllerGains;
        //sets the callbacks for when the PID gains or constraints are changed
        this.controllerGains.setHasPIDGainsChanged(hasPIDGainsChangeRunnable);
        this.controllerGains.setHasConstraintsChanged(hasConstraintsChangeRunnable);

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
    public void setControl(ControllerRequest request) {
        Objects.requireNonNull(request);
        Objects.requireNonNull(request.controlMode);
        Objects.requireNonNull(request.goal);

        this.sendableControlMode = request.controlMode.ordinal();

        if (request.slot < 0 || request.slot >= pidController.length) {
            throw new IllegalArgumentException("Invalid slot: " + request.slot);
        }

        if (request.controlMode.isProfiled() && !controllerGains.isProfiled(request.slot)) {
            errorHandler.logWarning("Using a profiled control mode without a profile set in the controller gains. using normal request");
        }

        this.request = request;
    }

    /**
     * Sets the reference of the controller
     *
     * @param setpoint    The new setpoint (the goal if using a profiled control).
     * @param controlMode The mode of control.
     * @param slot        Which slot to use for Feedback and Feedforwards.
     */
    public void setControl(double setpoint, ControlMode controlMode, int slot) {
        setControl(new ControllerRequest(setpoint, controlMode, slot));
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

        setControl(new ControllerRequest(goal, goalVelocity, controlMode, slot));
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

        setControl(new ControllerRequest(goal, controlMode, slot));
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
        return request.goal;
    }

    /**
     * Gets the goal of the controller as a double.
     * This is the position of the goal in units of measurement.
     * If not using profiled control, this will be the same as the setpoint.
     *
     * @return The goal of the controller as a double
     */
    public double getGoalAsDouble() {
        return request.goal.position;
    }

    /**
     * Gets the current control mode of the controller.
     *
     * @return The current control mode of the controller
     */
    public ControlMode getControlMode() {
        return request.controlMode;
    }

    /**
     * Gets the latest request of the controller.
     *
     * @return The current request of the controller
     */
    public ControllerRequest getRequest() {
        return request;
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
        return this.pidController[request.slot].calculate(this.setpoint.position, measurement, dt);
    }

    /**
     * Sets the setpoint to the goal.
     * This is used when the controller is not using a motion profile.
     * if using a motion profile, use {@link #calculateProfile(double dt)} to update the setpoint.
     */
    public void setSetpointToGoal() {
        this.setpoint = request.goal;
    }

    /**
     * Calculates the feed forward of the controller.
     * This calculates the outputs of the feed forwards.
     * It uses the saved setpoint for calculating the feed forward output.
     * This includes the arbitrary feed forward set by the user.
     *
     * @param directionOfTravel the direction of travel of the motor (used to calculate the friction
     *                          feed forward)
     * @return The feed forward of the controller in volts
     */
    public LogFrame.FeedForwardOutput calculateFeedForward(double directionOfTravel) {
        return controllerGains
                .getControllerFeedForwards(request.slot)
                .calculateFeedForwardOutput(
                        this.setpoint.position, directionOfTravel, request.arbFeedForward);
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
        var profile = this.controllerGains.getMotionProfile(request.slot);

        setpoint = profile.calculate(dt, setpoint, request.goal);
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
            Measurements.Measurement measurement, ControllerRequest request) {
        this.controllerGains.getControllerConstrains().calculateConstraints(measurement, request);
    }

    // maintenance things
    @Override
    public void initSendable(SendableBuilder builder) {
        boolean isProfiled = controllerGains.initSendable(builder);

        String setPointName = isProfiled ? "setpoint" : "goal";

        //this acts both as the setpoint and the goal of the controller
        builder.addDoubleProperty(setPointName, () -> setpoint.position,
                (value) -> setControl(value, ControlMode.values()[this.sendableControlMode], request.slot));

        builder.addIntegerProperty("controlMode", () -> request.controlMode.ordinal(),
                (mode) -> this.sendableControlMode = (int)mode);
    }

    /**
     * An enum that represents the control mode of the controller.
     */
    public enum ControlMode {
        /**
         * Stops motor output.
         */
        STOP(false, false),
        /**
         * Control directly the voltage applied to the coils.
         */
        VOLTAGE(false, false),
        /**
         * Controls the duty cycle of the motor output. (a fraction of the input voltage).
         * Known as (precent output and duty cycle output)
         */
        PERCENT_OUTPUT(false, false),
        /**
         * Controls the motor with a closed loop position control.
         * Needs pid gains to be set in the controller gains.
         * If you want to use a motion profile, use {@link #PROFILED_POSITION} instead.
         */
        POSITION(false, true),
        /**
         * Controls the motor with a profiled position output.
         * This is the same as {@link #POSITION} but uses a motion profile to smooth the movement.
         * The motion profile is set in the controller gains.
         * The motion profile is always calculated on the rio (not using the built-in controllers features).
         */
        PROFILED_POSITION(true, true),
        /**
         * Controls the motor with a closed loop velocity control.
         * Needs pid gains to be set in the controller gains.
         * Also recommended to use a setpoint feedforward to help the motor stay in the requested velocity.
         * If you want to use a motion profile, use {@link #PROFILED_VELOCITY} instead.
         */
        VELOCITY(false, true),
        /**
         * Controls the motor with a profiled velocity output.
         * This is the same as {@link #VELOCITY} but uses a motion profile to smooth the movement.
         * The motion profile is set in the controller gains.
         * The motion profile is always calculated on the rio (not using the built-in controllers features).
         */
        PROFILED_VELOCITY(true, true),

        /**
         * Controls the motor with a closed loop current control.
         * Needs pid gains to be set in the controller gains.
         * This is used to control the current output of the motor.
         * Will work the same as {@link #TORQUE}.
         * This is used where a mechanism needs to apply a specific current to the motor.
         */
        CURRENT(false, true),

        /**
         * Controls the motor with a closed loop torque control.
         * Needs pid gains to be set in the controller gains.
         * This is used to control the torque output of the motor.
         * Will work the same as {@link #CURRENT}.
         * This is used where a mechanism needs to apply a specific torque to the motor.
         */
        TORQUE(false, true);

        /**
         * If the control mode is profiled.
         */
        private final boolean profiled;

        /**
         * If the control mode requires a PID controller.
         * This is used to check if the controller needs to calculate the PID output.
         */
        private final boolean requiresPID;

        ControlMode(boolean profiled, boolean requiresPID) {
            this.profiled = profiled;
            this.requiresPID = requiresPID;
        }

        /**
         * Checks if the control mode is a position control
         * This function is used for calculating the constraints of the controller.
         *
         * @return True if the control mode is a position control
         */
        public boolean isPositionControl() {
            return this == POSITION || this == PROFILED_POSITION;
        }

        /**
         * Checks if the control mode is a velocity control
         * This function is used for calculating the constraints of the controller.
         *
         * @return True if the control mode is a velocity control
         */
        public boolean isVelocityControl() {
            return this == VELOCITY || this == PROFILED_VELOCITY;
        }

        /**
         * Checks if the control mode is a current control.
         * Current control is both CURRENT and TORQUE.
         *
         * @return True if the control mode is a current control
         */
        public boolean isCurrentControl() {
            return this == CURRENT || this == TORQUE;
        }

        /**
         * Checks if the control mode is a profiled control
         * This function is used for checking if the controller is using a motion profile.
         *
         * @return True if the control mode is a profiled control
         */
        public boolean isProfiled() {
            return this.profiled;
        }

        /**
         * Checks if the control mode requires a pid controller
         * Used to check if there is a need to calculate the PID output.
         *
         * @return True if the control mode requires a pid controller
         */
        public boolean requiresPID() {
            return this.requiresPID;
        }
    }

    /**
     * The request used to control the motor.
     * This will be set by the user to control the motor.
     *
     * @param goal           The goal of the controller. (if not using a motion profile, this will be the setpoint).
     * @param controlMode    The control mode used to control the motor.
     * @param arbFeedForward A voltage feedforward given by the user that will be added to the motor output.
     *                       Useful when using outside calculations easily.
     *                       Similar to the function feedForward in the {@link FeedForwardsGains}
     * @param slot           Which slot to use for Feedback and Feedforwards.
     */
    public record ControllerRequest(
            TrapezoidProfile.State goal, ControlMode controlMode, double arbFeedForward, int slot) {

        /**
         * Creates a controller request with a custom goal.
         * use only when using profiled control.
         * Use this when you want to control the goal velocity.
         *
         * @param goal        The goal of the controller.
         * @param controlMode The control mode used to control the motor.
         * @param slot        Which slot to use for Feedback and Feedforwards.
         */
        public ControllerRequest(TrapezoidProfile.State goal, ControlMode controlMode, int slot) {
            this(goal, controlMode, 0, slot);
        }

        /**
         * Creates a controller request with a goal.
         *
         * @param goal        The new goal (if not using a motion profile, this will be the setpoint).
         * @param controlMode The new control mode used to control the motor.
         * @param slot        Which slot to use for Feedback and Feedforwards.
         */
        public ControllerRequest(double goal, ControlMode controlMode, int slot) {
            this(new TrapezoidProfile.State(goal, 0), controlMode, 0, slot);
        }


        /**
         * Creates a controller request with a goal and a setpoint velocity.
         * use only when using profiled control.
         * Use this when you want to control the goal velocity.
         * Same as {@link #ControllerRequest(TrapezoidProfile.State goal, ControlMode controlMode, int slot)}
         *
         * @param goal         The new goal of the controller.
         * @param goalVelocity The goal velocity of the controller.
         *                     (i.e. the velocity the motor should be at when reaching the goal)
         * @param controlMode  The control mode used to control the motor.
         * @param slot         Which slot to use for Feedback and Feedforwards.
         */
        public ControllerRequest(double goal, double goalVelocity, ControlMode controlMode, int slot) {
            this(new TrapezoidProfile.State(goal, goalVelocity), controlMode, 0, slot);
        }

        /**
         * Creates an empty controller request.
         * Used to stop the motor.
         */
        public ControllerRequest() {
            this(new TrapezoidProfile.State(), ControlMode.STOP, 0);
        }

        /**
         * Creates a controller request with a goal and an arbitrary feed forward.
         *
         * @param goal           The new goal of the controller. (if not using a motion profile, this will be the setpoint).
         * @param controlMode    The control mode used to control the motor.
         * @param arbFeedForward A voltage feedforward given by the user that will be added to the motor output.
         * @param slot           Which slot to use for Feedback and Feedforwards.
         */
        public ControllerRequest(double goal, ControlMode controlMode, double arbFeedForward, int slot) {
            this(new TrapezoidProfile.State(goal, 0), controlMode, arbFeedForward, slot);
        }

        /**
         * Creates a controller request with a goal and an arbitrary feed forward.
         *
         * @param goal           The new goal of the controller. (if not using a motion profile, this will be the setpoint).
         * @param goalVelocity   The goal velocity of the controller.
         *                       (i.e. the velocity the motor should be at when reaching the goal)
         * @param controlMode    The control mode used to control the motor.
         * @param arbFeedForward A voltage feedforward given by the user that will be added to the motor output.
         * @param slot           Which slot to use for Feedback and Feedforwards.
         */
        public ControllerRequest(double goal, double goalVelocity, ControlMode controlMode, double arbFeedForward, int slot) {
            this(new TrapezoidProfile.State(goal, goalVelocity), controlMode, arbFeedForward, slot);
        }
    }
}
