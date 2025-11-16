package io.github.captainsoccer.basicmotor.gains;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
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
     * The slot that is currently being sent to the dashboard.
     * This is used to determine which slot to display in the sendable.
     * This will need to be changed before calling {@link #initSendable(SendableBuilder)}.
     */
    private int sendableSlot = 0;

    /**
     * Sets the slot that is currently being sent to the dashboard.
     * This is used to determine which slot to display in the sendable.
     * This will need to be changed before calling {@link #initSendable(SendableBuilder)}.
     *
     * @param slot The slot to send to the dashboard (0-2)
     */
    public void setSendableSlot(int slot) {
        if (slot < 0 || slot >= slotGains.length) {
            throw new IllegalArgumentException("slot must be between 0 and " + (slotGains.length - 1));
        }
        this.sendableSlot = slot;
    }

    /**
     * Creates a controller gains object with default values.
     * PID gains are all set to 0, constraints are set to default values.
     */
    public ControllerGains() {
        // Default constructor
    }

    /**
     * Creates a controller gains object with the given slot gains and constraints.
     *
     * @param slotGains        The individual slot gains of the controller.
     * @param constraintsGains The constraints of the controller (soft limits, deadband, etc.)
     */
    public ControllerGains(SlotGains[] slotGains, ConstraintsGains constraintsGains) {
        if (slotGains.length != this.slotGains.length) {
            throw new IllegalArgumentException("slotGains array must have length " + this.slotGains.length);
        }
        System.arraycopy(slotGains, 0, this.slotGains, 0, slotGains.length);
        this.constraintsGains = constraintsGains;
    }

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
        if (slotGains[0].hasPIDGainsRunnableSet()) return;

        for (int i = 0; i < slotGains.length; i++) {
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
     * Checks if the slot number is valid (0-2).
     * Throws an IllegalArgumentException if the slot number is invalid.
     *
     * @param slot The slot number to check.
     */
    private void checkArrayAccess(int slot) {
        if (slot < 0 || slot >= slotGains.length) {
            throw new IllegalArgumentException("slot must be between 0 and " + (slotGains.length - 1));
        }
    }

    /**
     * Gets the PID gains of the controller
     *
     * @param slot The slot to get the PID gains from (0-2)
     * @return The PID gains of the controller
     */
    public PIDGains getPidGains(int slot) {
        checkArrayAccess(slot);
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
     * @param slot The slot to get the feed forwards from (0-2)
     * @return The feed forwards of the controller
     */
    public FeedForwardsGains getControllerFeedForwards(int slot) {
        checkArrayAccess(slot);
        return slotGains[slot].getFeedForwardsGains();
    }

    /**
     * Gets the constraints of the profile
     *
     * @param slot The slot to get the profile constraints from (0-2)
     * @return The constraints of the profile
     */
    public TrapezoidProfile getMotionProfile(int slot) {
        checkArrayAccess(slot);
        return slotGains[slot].getMotionProfile();
    }

    /**
     * Checks if the controller is profiled.
     * The Controller is profiled if both the maximum velocity and maximum acceleration are changed from the default value.
     *
     * @param slot The slot to check if the controller is profiled (0-2)
     * @return True if the controller is profiled, false otherwise.
     */
    public boolean isProfiled(int slot) {
        checkArrayAccess(slot);
        var gains = slotGains[slot].getMotionProfileGains();
        return Double.isFinite(gains.maxVelocity) && Double.isFinite(gains.maxAcceleration);
    }

    /**
     * Sets the PID gains of the controller.
     * Calls the {@link #setHasPIDGainsChanged} function to notify that the PID gains have changed.
     *
     * @param slot     The slot to set the PID gains for (0-2)
     * @param pidGains The PID gains of the controller
     */
    public void setPidGains(PIDGains pidGains, int slot) {
        checkArrayAccess(slot);
        slotGains[slot].setPIDGains(pidGains);
    }

    /**
     * Sets the PID gains of the controller.
     * Calls the {@link #setHasPIDGainsChanged} function to notify that the PID gains have changed.
     * This will set the pid gains for slot 0.
     *
     * @param pidGains The PID gains of the controller
     */
    public void setPidGains(PIDGains pidGains) {
        setPidGains(pidGains, 0);
    }

    /**
     * Sets the PID gains of the controller.
     * Calls the {@link #setHasPIDGainsChanged} function to notify that the PID gains have changed.
     *
     * @param k_P  The proportional gain (>= 0) (volts per unit of control)
     * @param k_I  The integral gain (>= 0) (volts second per unit of control)
     * @param k_D  The derivative gain (>= 0) (volts per unit of control per second)
     * @param slot The slot to set the PID gains for (0-2)
     */
    public void setPidGains(double k_P, double k_I, double k_D, int slot) {
        checkArrayAccess(slot);
        setPidGains(new PIDGains(k_P, k_I, k_D), slot);
    }

    /**
     * Sets the PID gains of the controller.
     * Calls the {@link #setHasPIDGainsChanged} function to notify that the PID gains have changed.
     * This will set the pid gains for slot 0.
     *
     * @param k_P The proportional gain (>= 0) (volts per unit of control)
     * @param k_I The integral gain (>= 0) (volts second per unit of control)
     * @param k_D The derivative gain (>= 0) (volts per unit of control per second)
     */
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
     * @param slot              The slot to set the feed forwards for (0-2)
     * @param feedForwardsGains The feed forwards of the controller
     */
    public void setFeedForwardsGains(FeedForwardsGains feedForwardsGains, int slot) {
        checkArrayAccess(slot);
        slotGains[slot].setFeedForwardsGains(feedForwardsGains);
    }

    /**
     * Sets the feed forwards of the controller.
     * This will set the feed forwards for slot 0.
     *
     * @param feedForwardsGains The feed forwards of the controller
     */
    public void setFeedForwardsGains(FeedForwardsGains feedForwardsGains) {
        setFeedForwardsGains(feedForwardsGains, 0);
    }

    /**
     * Sets the constraints of the profile, used for motion profiling.
     *
     * @param profileConstraints The constraints of the profile
     * @param slot               The slot to set the profile constraints for (0-2)
     */
    public void setMotionProfileGains(TrapezoidProfile.Constraints profileConstraints, int slot) {
        checkArrayAccess(slot);

        if (profileConstraints.maxVelocity <= 0) {
            throw new IllegalArgumentException("maxVelocity must be greater than 0");
        }

        if (profileConstraints.maxAcceleration <= 0){
            throw new IllegalArgumentException("maxAcceleration must be greater than 0");
        }

        slotGains[slot].setMotionProfileGains(profileConstraints);
    }

    /**
     * Sets the constraints of the profile, used for motion profiling.
     * This will set the profile constraints for slot 0.
     *
     * @param profileConstraints The constraints of the profile
     */
    public void setMotionProfileGains(TrapezoidProfile.Constraints profileConstraints) {
        setMotionProfileGains(profileConstraints, 0);
    }

    /**
     * This function is used to initialize the sendable for the controller gains.
     * Used when the {@link Controller} is sent to the dashboard.
     * If you want to control the constraints of the motion profile through the dashboard,
     * you must give them an initial value before calling this function.
     * By default, this will use the gains in slot 0.
     * Can be changed by calling {@link #setSendableSlot(int)} before calling this function.
     * This function will also check if the controller is profiled or not,
     * and will set the sendable type accordingly and return if it is profiled or not.
     *
     * @param builder The sendable builder to use for the controller gains.
     * @return isProfiled, whether the pid controller is profiled or not
     */
    public boolean initSendable(SendableBuilder builder) {
        var slot = slotGains[sendableSlot];

        boolean isProfiled = isProfiled(sendableSlot);

        if (isProfiled) {
            builder.setSmartDashboardType("ProfiledPIDController");

            builder.addDoubleProperty(
                    "maxVelocity",
                    () -> slot.getMotionProfileGains().maxVelocity,
                    (x) -> slot.setMotionProfileGains(
                            new TrapezoidProfile.Constraints(x, slot.getMotionProfileGains().maxAcceleration)));

            builder.addDoubleProperty(
                    "maxAcceleration",
                    () -> slot.getMotionProfileGains().maxAcceleration,
                    (x) -> slot.setMotionProfileGains(
                            new TrapezoidProfile.Constraints(slot.getMotionProfileGains().maxVelocity, x)));

        } else builder.setSmartDashboardType("PIDController");

        buildPIDSendable(builder, slot);
        buildFeedForwardSendable(builder, slot);

        return isProfiled;
    }

    /**
     * Builds the sendable for the feed forwards (simpleFeedForward, frictionFeedForward, setpointFeedForward).
     *
     * @param builder The sendable builder
     * @param slot    The slot to build the sendable for
     */
    private void buildFeedForwardSendable(SendableBuilder builder, SlotGains slot) {
        builder.addDoubleProperty(
                "simpleFeedForward",
                () -> slot.getFeedForwardsGains().getSimpleFeedForward(),

                (value) ->
                        slot.updateFeedForwardsGains(
                                value, FeedForwardsGains.ChangeType.SIMPLE_FEED_FORWARD));

        builder.addDoubleProperty(
                "frictionFeedForward",
                () -> slot.getFeedForwardsGains().getFrictionFeedForward(),

                (value) ->
                        slot.updateFeedForwardsGains(
                                value, FeedForwardsGains.ChangeType.FRICTION_FEED_FORWARD));

        builder.addDoubleProperty(
                "setpointFeedForward",
                () -> slot.getFeedForwardsGains().getSetpointFeedForward(),

                (value) ->
                        slot.updateFeedForwardsGains(
                                value, FeedForwardsGains.ChangeType.SETPOINT_FEED_FORWARD));
    }

    /**
     * Builds the sendable for the PID gains. (p, i, d, izone, iMaxAccum, tolerance).
     *
     * @param builder The sendable builder
     * @param slot    The slot to build the sendable for
     */
    private void buildPIDSendable(SendableBuilder builder, SlotGains slot) {
        builder.addDoubleProperty(
                "p", () -> slot.getPIDGains().getK_P(),
                (value) -> slot.updatePIDGains(value, PIDGains.ChangeType.K_P));

        builder.addDoubleProperty(
                "i", () -> slot.getPIDGains().getK_I(),
                (value) -> slot.updatePIDGains(value, PIDGains.ChangeType.K_I));

        builder.addDoubleProperty(
                "d", () -> slot.getPIDGains().getK_D(),
                (value) -> slot.updatePIDGains(value, PIDGains.ChangeType.K_D));

        builder.addDoubleProperty(
                "izone", () -> slot.getPIDGains().getI_Zone(),
                (value) -> slot.updatePIDGains(value, PIDGains.ChangeType.I_ZONE));

        builder.addDoubleProperty(
                "iMaxAccum",
                () -> slot.getPIDGains().getI_MaxAccum(),
                (value) -> slot.updatePIDGains(value, PIDGains.ChangeType.I_MAX_ACCUM));

        builder.addDoubleProperty(
                "tolerance",
                () -> slot.getPIDGains().getTolerance(),
                (value) -> slot.updatePIDGains(value, PIDGains.ChangeType.TOLERANCE));
    }
}
