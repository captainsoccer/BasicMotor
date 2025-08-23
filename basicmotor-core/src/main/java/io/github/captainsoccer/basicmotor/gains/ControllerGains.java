package io.github.captainsoccer.basicmotor.gains;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import io.github.captainsoccer.basicmotor.controllers.Controller;
import io.github.captainsoccer.basicmotor.BasicMotor;

import java.util.function.Consumer;

/**
 * This class stores all the controller gains, constraints, and feed forwards for a motor controller.
 * It is used directly by the {@link Controller}.
 */
public class ControllerGains {

    /**
     * The individual slot gains of the controller.
     * Contains the PID gains, feed forwards, and motion profile constraints for each slot.
     */
    private final SlotGains[] slotGains = SlotGains.getDefaultSlotGains();

    /**
     * The constraints of the controller. (soft limits, deadband, etc.)
     */
    private ConstraintsGains constraintsGains = new ConstraintsGains();

    /**
     * The function that is called when the constraints are changed.
     * Used to set a flag in the {@link BasicMotor} to update the constraints on the slower thread.
     */
    private Runnable setHasConstraintsChanged;

    /**
     * Creates a controller gains object with the given PID gains.
     *
     * @param k_P The proportional gain (>= 0) (volts per unit of control)
     * @param k_I The integral gain (>= 0) (volts second per unit of control)
     * @param k_D The derivative gain (>= 0) (volts per unit of control per second)
     */
    public ControllerGains(double k_P, double k_I, double k_D) {
        slotGains[0].setPIDGains(new PIDGains(k_P, k_I, k_D), false);
    }

    /**
     * Creates a controller gains object with the given PID gains
     *
     * @param pidGains The PID gains
     */
    public ControllerGains(PIDGains pidGains) {
        slotGains[0].setPIDGains(pidGains, false);
    }

    /**
     * Creates a controller gains object with the given feed forwards.
     *
     * @param feedForwardsGains The feed forwards of the controller.
     */
    public ControllerGains(FeedForwardsGains feedForwardsGains) {
        slotGains[0].setFeedForwardsGains(feedForwardsGains);
    }

    /**
     * Creates a controller gains object with the given pid gains and constraints.
     *
     * @param pidGains         The PID gains
     * @param constraintsGains The constraints of the controller (soft limits, deadband, etc.)
     */
    public ControllerGains(PIDGains pidGains, ConstraintsGains constraintsGains) {
        slotGains[0].setPIDGains(pidGains, false);
        this.constraintsGains = constraintsGains;
    }

    /**
     * Creates a controller gains object with the given pid gains and feed forwards.
     *
     * @param pidGains          The PID gains
     * @param feedForwardsGains The feed forwards of the controller
     */
    public ControllerGains(PIDGains pidGains, FeedForwardsGains feedForwardsGains) {
        slotGains[0].setPIDGains(pidGains, false);
        slotGains[0].setFeedForwardsGains(feedForwardsGains);
    }

    /**
     * Creates a controller gains object with the given pid gains, constraints and feed forwards.
     *
     * @param pidGains          The PID gains
     * @param constraintsGains  The constraints of the controller (soft limits, deadband, etc.)
     * @param feedForwardsGains The feed forwards of the controller
     */
    public ControllerGains(PIDGains pidGains, ConstraintsGains constraintsGains, FeedForwardsGains feedForwardsGains) {
        slotGains[0].setPIDGains(pidGains, false);
        slotGains[0].setFeedForwardsGains(feedForwardsGains);
        this.constraintsGains = constraintsGains;
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

        slotGains[0].setPIDGains(pidGains, false);
        this.constraintsGains = constraintsGains;
        slotGains[0].setFeedForwardsGains(feedForwardsGains);
        slotGains[0].setMotionProfileGains(motionProfileGains);
    }

    /**
     * Sets the callback function that is called when the PID gains are changed.
     * Used only once to set the callback function.
     * (Used by the {@link Controller}).
     * The callback function is not set in the constructor so the user doesn't need to set it.
     *
     * @param hasPIDGainsChanged The function that is called when the PID gains are changed.
     */
    public void setHasPIDGainsChanged(Consumer<Integer> hasPIDGainsChanged) {
        if(slotGains[0].hasPIDGainsRunnableSet()) return;

        for(int i = 0; i < slotGains.length; i++) {
            final int index = i;
            slotGains[i].setHasPIDGainsChanged(() -> hasPIDGainsChanged.accept(index));
        }
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
    public PIDGains getPidGains(int slot) {
        return slotGains[slot].getPIDGains();
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
    public FeedForwardsGains getControllerFeedForwards(int slot) {
        return slotGains[slot].getFeedForwardsGains();
    }

    /**
     * Gets the constraints of the profile
     *
     * @return The constraints of the profile
     */
    public TrapezoidProfile getMotionProfile(int slot) {
        return slotGains[slot].getMotionProfile();
    }

    /**
     * Checks if the controller is profiled.
     * The Controller is profiled if both the maximum velocity and maximum acceleration are changed from the default value.
     *
     * @return True if the controller is profiled, false otherwise.
     */
    public boolean isProfiled(int slot) {
        var gains = slotGains[slot].getMotionProfileGains();
        return gains.maxVelocity != Double.POSITIVE_INFINITY && gains.maxAcceleration != Double.POSITIVE_INFINITY;
    }

    /**
     * Sets the PID gains of the controller.
     * Calls the {@link #setHasPIDGainsChanged} function to notify that the PID gains have changed.
     *
     * @param pidGains The PID gains of the controller
     */
    public void setPidGains(PIDGains pidGains, int slot) {
        slotGains[slot].setPIDGains(pidGains);
    }

    public void setPidGains(PIDGains pidGains) {
        setPidGains(pidGains, 0);
    }

    /**
     * Sets the PID gains of the controller.
     * Calls the {@link #setHasPIDGainsChanged} function to notify that the PID gains have changed.
     *
     * @param k_P The proportional gain (>= 0) (volts per unit of control)
     * @param k_I The integral gain (>= 0) (volts second per unit of control)
     * @param k_D The derivative gain (>= 0) (volts per unit of control per second)
     */
    public void setPidGains(double k_P, double k_I, double k_D, int slot) {
        setPidGains(new PIDGains(k_P, k_I, k_D), slot);
    }

    public void setPidGains(double k_P, double k_I, double k_D) {
        setPidGains(k_P, k_I, k_D, 0);
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
    public void setFeedForwardsGains(FeedForwardsGains feedForwardsGains, int slot) {
        slotGains[slot].setFeedForwardsGains(feedForwardsGains);
    }

    public void setFeedForwardsGains(FeedForwardsGains feedForwardsGains) {
        setFeedForwardsGains(feedForwardsGains, 0);
    }

    /**
     * Sets the constraints of the profile, used for motion profiling.
     *
     * @param profileConstraints The constraints of the profile
     */
    public void setMotionProfileGains(TrapezoidProfile.Constraints profileConstraints, int slot) {
        if(profileConstraints.maxVelocity < 0 || profileConstraints.maxAcceleration < 0) {
            DriverStation.reportError("motion profile constraints must be greater than or equal to zero, disabling profile", false);
            profileConstraints = new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }

        slotGains[slot].setMotionProfileGains(profileConstraints);
    }

    public void setMotionProfileGains(TrapezoidProfile.Constraints profileConstraints) {
        setMotionProfileGains(profileConstraints, 0);
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
        var slot0 = slotGains[0];

        if (isProfiled(0)) {
            builder.setSmartDashboardType("ProfiledPIDController");

            builder.addDoubleProperty(
                    "maxVelocity",
                    () -> slot0.getMotionProfileGains().maxVelocity,
                    (x) -> setMotionProfileGains(
                            new TrapezoidProfile.Constraints(x, slot0.getMotionProfileGains().maxAcceleration)));

            builder.addDoubleProperty(
                    "maxAcceleration",
                    () -> slot0.getMotionProfileGains().maxAcceleration,
                    (x) -> setMotionProfileGains(
                            new TrapezoidProfile.Constraints(slot0.getMotionProfileGains().maxVelocity, x)));

        } else builder.setSmartDashboardType("PIDController");

        buildPIDSendable(builder, slot0);
        buildFeedForwardSendable(builder, slot0);
    }

    /**
     * Builds the sendable for the feed forwards (simpleFeedForward, frictionFeedForward, setpointFeedForward).
     *
     * @param builder The sendable builder
     */
    private void buildFeedForwardSendable(SendableBuilder builder, SlotGains slot0) {
        builder.addDoubleProperty(
                "simpleFeedForward",
                () -> slot0.getFeedForwardsGains().getSimpleFeedForward(),

                (value) ->
                        slot0.updateFeedForwardsGains(
                                value, FeedForwardsGains.ChangeType.SIMPLE_FEED_FORWARD));

        builder.addDoubleProperty(
                "frictionFeedForward",
                () -> slot0.getFeedForwardsGains().getFrictionFeedForward(),

                (value) ->
                        slot0.updateFeedForwardsGains(
                                value, FeedForwardsGains.ChangeType.FRICTION_FEED_FORWARD));

        builder.addDoubleProperty(
                "setpointFeedForward",
                () -> slot0.getFeedForwardsGains().getSetpointFeedForward(),

                (value) ->
                        slot0.updateFeedForwardsGains(
                                value, FeedForwardsGains.ChangeType.SETPOINT_FEED_FORWARD));
    }

    /**
     * Builds the sendable for the PID gains. (p, i, d, izone, iMaxAccum, tolerance).
     *
     * @param builder The sendable builder
     */
    private void buildPIDSendable(SendableBuilder builder, SlotGains slot0) {
        builder.addDoubleProperty(
                "p", () -> slot0.getPIDGains().getK_P(),
                (value) -> slot0.updatePIDGains(value, PIDGains.ChangeType.K_P));

        builder.addDoubleProperty(
                "i", () -> slot0.getPIDGains().getK_I(),
                (value) -> slot0.updatePIDGains(value, PIDGains.ChangeType.K_I));

        builder.addDoubleProperty(
                "d", () -> slot0.getPIDGains().getK_D(),
                (value) -> slot0.updatePIDGains(value, PIDGains.ChangeType.K_D));

        builder.addDoubleProperty(
                "izone", () -> slot0.getPIDGains().getI_Zone(),
                (value) -> slot0.updatePIDGains(value, PIDGains.ChangeType.I_ZONE));

        builder.addDoubleProperty(
                "iMaxAccum",
                () -> slot0.getPIDGains().getI_MaxAccum(),
                (value) -> slot0.updatePIDGains(value, PIDGains.ChangeType.I_MAX_ACCUM));

        builder.addDoubleProperty(
                "tolerance",
                () -> slot0.getPIDGains().getTolerance(),
                (value) -> slot0.updatePIDGains(value, PIDGains.ChangeType.TOLERANCE));
    }
}
