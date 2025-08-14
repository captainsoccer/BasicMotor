package io.github.captainsoccer.basicmotor.gains;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import io.github.captainsoccer.basicmotor.controllers.Controller;
import io.github.captainsoccer.basicmotor.BasicMotor;

/**
 * This class stores all the controller gains, constraints, and feed forwards for a motor controller.
 * It is used directly by the {@link Controller}.
 */
public class ControllerGains {
    /**
     * The PID gains of the controller.
     */
    private PIDGains pidGains = new PIDGains();
    /**
     * The constraints of the controller. (soft limits, deadband, etc.)
     */
    private ControllerConstraints controllerConstraints = new ControllerConstraints();
    /**
     * The feed forwards of the controller.
     */
    private ControllerFeedForwards controllerFeedForwards = new ControllerFeedForwards();

    /**
     * The constraints of the profile (used for motion profiling).
     * Has the maximum velocity and maximum acceleration.
     * (changes based on the control mode)
     */
    private TrapezoidProfile.Constraints profileConstraints =
            new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    /**
     * The trapezoid profile used for motion profiling.
     * It uses the {@link #profileConstraints} to calculate the profile.
     * changes only when the profile constraints are changed.
     */
    private TrapezoidProfile controllerProfile = new TrapezoidProfile(profileConstraints);

    /**
     * The function that is called when the PID gains are changed.
     * Used to set a flag in the {@link BasicMotor} to update the PID gains on the slower thread.
     */
    private Runnable setHasPIDGainsChanged;

    /**
     * The function that is called when the constraints are changed.
     * Used to set a flag in the {@link BasicMotor} to update the constraints on the slower thread.
     */
    private Runnable setHasConstraintsChanged;

    /**
     * Creates an empty controller gains object (no PID gains, no feed forwards, no constraints).
     */
    public ControllerGains() {
    }

    /**
     * Creates a controller gains object with the given PID gains.
     *
     * @param k_P The proportional gain (>= 0) (volts per unit of control)
     * @param k_I The integral gain (>= 0) (volts second per unit of control)
     * @param k_D The derivative gain (>= 0) (volts per unit of control per second)
     */
    public ControllerGains(double k_P, double k_I, double k_D) {
        pidGains = new PIDGains(k_P, k_I, k_D);
    }

    /**
     * Creates a controller gains object with the given PID gains
     *
     * @param pidGains The PID gains
     */
    public ControllerGains(PIDGains pidGains) {
        this.pidGains = pidGains;
    }

    /**
     * Creates a controller gains object with the given feed forwards.
     *
     * @param controllerFeedForwards The feed forwards of the controller.
     */
    public ControllerGains(ControllerFeedForwards controllerFeedForwards) {
        this.controllerFeedForwards = controllerFeedForwards;
    }

    /**
     * Creates a controller gains object with the given pid gains and constraints.
     *
     * @param pidGains              The PID gains
     * @param controllerConstraints The constraints of the controller (soft limits, deadband, etc.)
     */
    public ControllerGains(PIDGains pidGains, ControllerConstraints controllerConstraints) {
        this.pidGains = pidGains;
        this.controllerConstraints = controllerConstraints;
    }

    /**
     * Creates a controller gains object with the given pid gains and feed forwards.
     *
     * @param pidGains               The PID gains
     * @param controllerFeedForwards The feed forwards of the controller
     */
    public ControllerGains(PIDGains pidGains, ControllerFeedForwards controllerFeedForwards) {
        this.pidGains = pidGains;
        this.controllerFeedForwards = controllerFeedForwards;
    }

    /**
     * Creates a controller gains object with the given pid gains, constraints and feed forwards.
     *
     * @param pidGains               The PID gains
     * @param controllerConstraints  The constraints of the controller (soft limits, deadband, etc.)
     * @param controllerFeedForwards The feed forwards of the controller
     */
    public ControllerGains(PIDGains pidGains, ControllerConstraints controllerConstraints, ControllerFeedForwards controllerFeedForwards) {
        this.pidGains = pidGains;
        this.controllerConstraints = controllerConstraints;
        this.controllerFeedForwards = controllerFeedForwards;
    }

    /**
     * Creates a controller with all the gains, constraints, and feed forwards.
     *
     * @param pidGains               The PID gains
     * @param controllerConstraints  The constraints of the controller (soft limits, deadband, etc.)
     * @param controllerFeedForwards The feed forwards of the controller
     * @param profileConstraints     The constraints of the profile (used for motion profiling).
     */
    public ControllerGains(
            PIDGains pidGains,
            ControllerConstraints controllerConstraints,
            ControllerFeedForwards controllerFeedForwards,
            TrapezoidProfile.Constraints profileConstraints) {
        this.pidGains = pidGains;
        this.controllerConstraints = controllerConstraints;
        this.controllerFeedForwards = controllerFeedForwards;
        this.profileConstraints = profileConstraints;

        this.controllerProfile = new TrapezoidProfile(profileConstraints);
    }

    /**
     * Sets the callback function that is called when the PID gains are changed.
     * Used only once to set the callback function.
     * (Used by the {@link Controller}).
     * The callback function is not set in the constructor so the user doesn't need to set it.
     *
     * @param hasPIDGainsChanged The function that is called when the PID gains are changed.
     */
    public void setHasPIDGainsChanged(Runnable hasPIDGainsChanged) {
        if (this.setHasPIDGainsChanged != null) return;
        this.setHasPIDGainsChanged = hasPIDGainsChanged;
    }

    /**
     * Sets the callback function that is called when the constraints are changed.
     * Used only once to set the callback function.
     * (Used by the {@link Controller}).
     * The callback function is not set in the constructor so the user doesn't need to set it.
     *
     * @param hasConstraintsChanged The function that is called when the constraints are changed.
     */
    public void setHasConstraintsChanged(Runnable hasConstraintsChanged) {
        if (this.setHasConstraintsChanged != null) return;
        this.setHasConstraintsChanged = hasConstraintsChanged;
    }

    /**
     * Gets the PID gains of the controller
     *
     * @return The PID gains of the controller
     */
    public PIDGains getPidGains() {
        return pidGains;
    }

    /**
     * Gets the constraints of the controller
     *
     * @return The constraints of the controller
     */
    public ControllerConstraints getControllerConstrains() {
        return controllerConstraints;
    }

    /**
     * Gets the feed forwards of the controller
     *
     * @return The feed forwards of the controller
     */
    public ControllerFeedForwards getControllerFeedForwards() {
        return controllerFeedForwards;
    }

    /**
     * Gets the constraints of the profile
     *
     * @return The constraints of the profile
     */
    public TrapezoidProfile getControllerProfile() {
        return controllerProfile;
    }

    /**
     * Checks if the controller is profiled.
     * The Controller is profiled if both the maximum velocity and maximum acceleration are changed from the default value.
     *
     * @return True if the controller is profiled, false otherwise.
     */
    public boolean isProfiled() {
        return profileConstraints.maxVelocity != Double.POSITIVE_INFINITY
                && profileConstraints.maxAcceleration != Double.POSITIVE_INFINITY;
    }

    /**
     * Sets the PID gains of the controller.
     * Calls the {@link #setHasPIDGainsChanged} function to notify that the PID gains have changed.
     *
     * @param pidGains The PID gains of the controller
     */
    public void setPidGains(PIDGains pidGains) {
        this.pidGains = pidGains;
        setHasPIDGainsChanged.run();
    }

    /**
     * Sets the PID gains of the controller.
     * Calls the {@link #setHasPIDGainsChanged} function to notify that the PID gains have changed.
     *
     * @param k_P The proportional gain (>= 0) (volts per unit of control)
     * @param k_I The integral gain (>= 0) (volts second per unit of control)
     * @param k_D The derivative gain (>= 0) (volts per unit of control per second)
     */
    public void setPidGains(double k_P, double k_I, double k_D) {
        setPidGains(new PIDGains(k_P, k_I, k_D));
    }

    /**
     * Sets the constraints of the controller.
     * Also calls the {@link #setHasConstraintsChanged} function to notify that the constraints have changed.
     *
     * @param controllerConstraints The constraints of the controller
     */
    public void setControllerConstrains(ControllerConstraints controllerConstraints) {
        this.controllerConstraints = controllerConstraints;
        setHasConstraintsChanged.run();
    }

    /**
     * Sets the feed forwards of the controller.
     *
     * @param controllerFeedForwards The feed forwards of the controller
     */
    public void setControllerFeedForwards(ControllerFeedForwards controllerFeedForwards) {
        this.controllerFeedForwards = controllerFeedForwards;
    }

    /**
     * Sets the constraints of the profile, used for motion profiling.
     *
     * @param profileConstraints The constraints of the profile
     */
    public void setControllerProfile(TrapezoidProfile.Constraints profileConstraints) {
        this.profileConstraints = profileConstraints;
        controllerProfile = new TrapezoidProfile(profileConstraints);
    }

    /**
     * This function is used to initialize the sendable for the controller gains.
     * Used when the {@link Controller} is sent to the dashboard.
     * If you want to control the constraints of the motion profile through the dashboard,
     * you must give them an initial value before calling this function.
     *
     * @param builder The sendable builder to use for the controller gains.
     */
    public void initSendable(SendableBuilder builder) {
        if (isProfiled()) {
            builder.setSmartDashboardType("ProfiledPIDController");

            builder.addDoubleProperty(
                    "maxVelocity",
                    () -> profileConstraints.maxVelocity,
                    (x) -> setControllerProfile(new TrapezoidProfile.Constraints(x, profileConstraints.maxAcceleration)));

            builder.addDoubleProperty(
                    "maxAcceleration",
                    () -> profileConstraints.maxAcceleration,
                    (x) -> setControllerProfile(new TrapezoidProfile.Constraints(profileConstraints.maxVelocity, x)));

        } else builder.setSmartDashboardType("PIDController");

        buildPIDSendable(builder);
        buildFeedForwardSendable(builder);
    }

    /**
     * Builds the sendable for the feed forwards (simpleFeedForward, frictionFeedForward, setpointFeedForward).
     *
     * @param builder The sendable builder
     */
    private void buildFeedForwardSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
                "simpleFeedForward",
                controllerFeedForwards::getSimpleFeedForward,

                (value) ->
                        controllerFeedForwards.updateFeedForwards(
                                value, ControllerFeedForwards.ChangeType.SIMPLE_FEED_FORWARD));

        builder.addDoubleProperty(
                "frictionFeedForward",
                controllerFeedForwards::getFrictionFeedForward,

                (value) ->
                        controllerFeedForwards.updateFeedForwards(
                                value, ControllerFeedForwards.ChangeType.FRICTION_FEED_FORWARD));

        builder.addDoubleProperty(
                "setpointFeedForward",
                controllerFeedForwards::getSetpointFeedForward,

                (value) ->
                        controllerFeedForwards.updateFeedForwards(
                                value, ControllerFeedForwards.ChangeType.SETPOINT_FEED_FORWARD));
    }

    /**
     * Builds the sendable for the PID gains. (p, i, d, izone, iMaxAccum, tolerance).
     *
     * @param builder The sendable builder
     */
    private void buildPIDSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
                "p", pidGains::getK_P, (value) -> updatePIDGains(value, PIDGains.ChangeType.K_P));

        builder.addDoubleProperty(
                "i", pidGains::getK_I, (value) -> updatePIDGains(value, PIDGains.ChangeType.K_I));

        builder.addDoubleProperty(
                "d", pidGains::getK_D, (value) -> updatePIDGains(value, PIDGains.ChangeType.K_D));

        builder.addDoubleProperty(
                "izone", pidGains::getI_Zone, (value) -> updatePIDGains(value, PIDGains.ChangeType.I_ZONE));

        builder.addDoubleProperty(
                "iMaxAccum",
                pidGains::getI_MaxAccum,
                (value) -> updatePIDGains(value, PIDGains.ChangeType.I_MAX_ACCUM));

        builder.addDoubleProperty(
                "tolerance",
                pidGains::getTolerance,
                (value) -> updatePIDGains(value, PIDGains.ChangeType.TOLERANCE));
    }

    /**
     * Sets the PID gains of the controller.
     * Calls the {@link #setHasPIDGainsChanged} function to notify that the PID gains have changed.
     * @param value The value to set the gain to (must be greater than or equal to zero)
     * @param changeType Which gain to change
     */
    private void updatePIDGains(double value, PIDGains.ChangeType changeType) {
        boolean hasChanged = pidGains.updatePIDGains(value, changeType);

        if (hasChanged) setHasPIDGainsChanged.run();
    }


}
