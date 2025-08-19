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
    private ConstraintsGains constraintsGains = new ConstraintsGains();
    /**
     * The feed forwards of the controller.
     */
    private FeedForwardsGains feedForwardsGains = new FeedForwardsGains();

    /**
     * The constraints of the profile (used for motion profiling).
     * Has the maximum velocity and maximum acceleration.
     * (changes based on the control mode)
     */
    private TrapezoidProfile.Constraints motionProfileGains =
            new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    /**
     * The trapezoid profile used for motion profiling.
     * It uses the {@link #motionProfileGains} to calculate the profile.
     * changes only when the profile constraints are changed.
     */
    private TrapezoidProfile motionProfile = new TrapezoidProfile(motionProfileGains);

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
     * @param feedForwardsGains The feed forwards of the controller.
     */
    public ControllerGains(FeedForwardsGains feedForwardsGains) {
        this.feedForwardsGains = feedForwardsGains;
    }

    /**
     * Creates a controller gains object with the given pid gains and constraints.
     *
     * @param pidGains         The PID gains
     * @param constraintsGains The constraints of the controller (soft limits, deadband, etc.)
     */
    public ControllerGains(PIDGains pidGains, ConstraintsGains constraintsGains) {
        this.pidGains = pidGains;
        this.constraintsGains = constraintsGains;
    }

    /**
     * Creates a controller gains object with the given pid gains and feed forwards.
     *
     * @param pidGains          The PID gains
     * @param feedForwardsGains The feed forwards of the controller
     */
    public ControllerGains(PIDGains pidGains, FeedForwardsGains feedForwardsGains) {
        this.pidGains = pidGains;
        this.feedForwardsGains = feedForwardsGains;
    }

    /**
     * Creates a controller gains object with the given pid gains, constraints and feed forwards.
     *
     * @param pidGains          The PID gains
     * @param constraintsGains  The constraints of the controller (soft limits, deadband, etc.)
     * @param feedForwardsGains The feed forwards of the controller
     */
    public ControllerGains(PIDGains pidGains, ConstraintsGains constraintsGains, FeedForwardsGains feedForwardsGains) {
        this.pidGains = pidGains;
        this.constraintsGains = constraintsGains;
        this.feedForwardsGains = feedForwardsGains;
    }

    /**
     * Creates a controller with all the gains, constraints, and feed forwards.
     *
     * @param pidGains           The PID gains
     * @param constraintsGains   The constraints of the controller (soft limits, deadband, etc.)
     * @param feedForwardsGains  The feed forwards of the controller
     * @param motionProfileGains The constraints of the profile (used for motion profiling).
     */
    public ControllerGains(
            PIDGains pidGains,
            ConstraintsGains constraintsGains,
            FeedForwardsGains feedForwardsGains,
            TrapezoidProfile.Constraints motionProfileGains) {
        this.pidGains = pidGains;
        this.constraintsGains = constraintsGains;
        this.feedForwardsGains = feedForwardsGains;
        this.motionProfileGains = motionProfileGains;

        this.motionProfile = new TrapezoidProfile(motionProfileGains);
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
    public ConstraintsGains getControllerConstrains() {
        return constraintsGains;
    }

    /**
     * Gets the feed forwards of the controller
     *
     * @return The feed forwards of the controller
     */
    public FeedForwardsGains getControllerFeedForwards() {
        return feedForwardsGains;
    }

    /**
     * Gets the constraints of the profile
     *
     * @return The constraints of the profile
     */
    public TrapezoidProfile getMotionProfile() {
        return motionProfile;
    }

    /**
     * Checks if the controller is profiled.
     * The Controller is profiled if both the maximum velocity and maximum acceleration are changed from the default value.
     *
     * @return True if the controller is profiled, false otherwise.
     */
    public boolean isProfiled() {
        return motionProfileGains.maxVelocity != Double.POSITIVE_INFINITY
                && motionProfileGains.maxAcceleration != Double.POSITIVE_INFINITY;
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
     * @param constraintsGains The constraints of the controller
     */
    public void setConstraintsGains(ConstraintsGains constraintsGains) {
        this.constraintsGains = constraintsGains;
        setHasConstraintsChanged.run();
    }

    /**
     * Sets the feed forwards of the controller.
     *
     * @param feedForwardsGains The feed forwards of the controller
     */
    public void setFeedForwardsGains(FeedForwardsGains feedForwardsGains) {
        this.feedForwardsGains = feedForwardsGains;
    }

    /**
     * Sets the constraints of the profile, used for motion profiling.
     *
     * @param profileConstraints The constraints of the profile
     */
    public void setMotionProfileGains(TrapezoidProfile.Constraints profileConstraints) {
        this.motionProfileGains = profileConstraints;
        motionProfile = new TrapezoidProfile(profileConstraints);
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
                    () -> motionProfileGains.maxVelocity,
                    (x) -> setMotionProfileGains(new TrapezoidProfile.Constraints(x, motionProfileGains.maxAcceleration)));

            builder.addDoubleProperty(
                    "maxAcceleration",
                    () -> motionProfileGains.maxAcceleration,
                    (x) -> setMotionProfileGains(new TrapezoidProfile.Constraints(motionProfileGains.maxVelocity, x)));

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
                feedForwardsGains::getSimpleFeedForward,

                (value) ->
                        feedForwardsGains.updateFeedForwards(
                                value, FeedForwardsGains.ChangeType.SIMPLE_FEED_FORWARD));

        builder.addDoubleProperty(
                "frictionFeedForward",
                feedForwardsGains::getFrictionFeedForward,

                (value) ->
                        feedForwardsGains.updateFeedForwards(
                                value, FeedForwardsGains.ChangeType.FRICTION_FEED_FORWARD));

        builder.addDoubleProperty(
                "setpointFeedForward",
                feedForwardsGains::getSetpointFeedForward,

                (value) ->
                        feedForwardsGains.updateFeedForwards(
                                value, FeedForwardsGains.ChangeType.SETPOINT_FEED_FORWARD));
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
     *
     * @param value      The value to set the gain to (must be greater than or equal to zero)
     * @param changeType Which gain to change
     */
    private void updatePIDGains(double value, PIDGains.ChangeType changeType) {
        boolean hasChanged = pidGains.updatePIDGains(value, changeType);

        if (hasChanged) setHasPIDGainsChanged.run();
    }


}
