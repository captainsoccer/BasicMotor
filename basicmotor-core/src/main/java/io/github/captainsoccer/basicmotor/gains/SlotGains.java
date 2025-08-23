package io.github.captainsoccer.basicmotor.gains;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import io.github.captainsoccer.basicmotor.BasicMotor;

public class SlotGains {
    /**
     * The default slot gains.
     * This will include default PID gains, feed forwards, and motion profile constraints.
     * This is used to initialize the slot gains in the {@link BasicMotor}.
     */
    public static SlotGains[] getDefaultSlotGains(){
        SlotGains[] slotGains = new SlotGains[3];
        for(int i = 0; i < slotGains.length; i++){
            slotGains[i] = new SlotGains();
        }
        return slotGains;
    }

    /**
     * The PID gains of the controller.
     */
    private PIDGains pidGains = new PIDGains();

    /**
     * Gets the PID gains of the slot
     * @return the PID gains of the slot
     */
    public PIDGains getPIDGains() {
        return pidGains;
    }

    /**
     * Sets the PID gains of the slot
     * @param pidGains the new PID gains of the slot
     */
    public void setPIDGains(PIDGains pidGains) {
        setPIDGains(pidGains, true);
    }

    /**
     * Sets the PID gains of the slot
     * @param pidGains the new PID gains of the slot
     * @param needToRunRunnable if true, will run the {@link #setHasPIDGainsChanged} runnable if it is set.
     */
    public void setPIDGains(PIDGains pidGains, boolean needToRunRunnable) {
        this.pidGains = pidGains;
        if(setHasPIDGainsChanged != null && needToRunRunnable) setHasPIDGainsChanged.run();
    }

    /**
     * Updates a single PID gain of the slot
     * @param value the new value of the PID gain
     * @param changeType the type of PID gain to change
     */
    public void updatePIDGains(double value, PIDGains.ChangeType changeType) {
        if(value < 0){
            DriverStation.reportError("PID gain values cannot be negative. Ignoring value: " + value, false);
            return;
        }

        double oldValue = this.pidGains.getValue(changeType);
        if(oldValue == value) return;

        this.pidGains = this.pidGains.changeValue(value, changeType);
        setHasPIDGainsChanged.run();
    }

    /**
     * The feed forwards of the controller.
     */
    private FeedForwardsGains feedForwardsGains = new FeedForwardsGains();

    /**
     * Gets the feed forwards of the slot
     * @return the feed forwards of the slot
     */
    public FeedForwardsGains getFeedForwardsGains() {
        return feedForwardsGains;
    }

    /**
     * Sets the feed forwards of the slot
     * @param feedForwardsGains the new feed forwards of the slot
     */
    public void setFeedForwardsGains(FeedForwardsGains feedForwardsGains) {
        this.feedForwardsGains = feedForwardsGains;
    }

    /**
     * Updates a single feed forward gain of the slot
     * @param value the new value of the feed forward gain
     * @param changeType the type of feed forward gain to change
     */
    public void updateFeedForwardsGains(double value, FeedForwardsGains.ChangeType changeType) {
        if(value < 0 && changeType != FeedForwardsGains.ChangeType.SIMPLE_FEED_FORWARD){
            DriverStation.reportError("Feed forward gain values cannot be negative. Ignoring value: " + value, false);
            return;
        }

        this.feedForwardsGains = this.feedForwardsGains.changeValue(value, changeType);
    }

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
     * Gets the motion profile used for motion profiling.
     * @return the motion profile used for motion profiling.
     */
    public TrapezoidProfile getMotionProfile() {
        return motionProfile;
    }

    /**
     * Gets the motion profile constraints used for motion profiling.
     * @return the motion profile constraints used for motion profiling.
     */
    public TrapezoidProfile.Constraints getMotionProfileGains() {
        return motionProfileGains;
    }

    /**
     * Sets the motion profile constraints used for motion profiling.
     * This will create a new {@link TrapezoidProfile} with the new constraints.
     * @param motionProfileGains the new motion profile constraints used for motion profiling.
     */
    public void setMotionProfileGains(TrapezoidProfile.Constraints motionProfileGains) {
        this.motionProfileGains = motionProfileGains;
        this.motionProfile = new TrapezoidProfile(motionProfileGains);
    }

    /**
     * The function that is called when the PID gains are changed.
     * Used to set a flag in the {@link BasicMotor} to update the PID gains on the slower thread.
     */
    private Runnable setHasPIDGainsChanged;

    /**
     * Checks if the hasPIDGainsChanged runnable is set.
     * @return true if the hasPIDGainsChanged runnable is set, false otherwise.
     */
    public boolean hasPIDGainsRunnableSet() {
        return setHasPIDGainsChanged != null;
    }

    /**
     * Sets the function that is called when the PID gains are changed.
     * Used to set a flag in the {@link BasicMotor} to update the PID gains on the slower thread.
     * Only sets the function if it is not already set.
     * @param runnable the function that is called when the PID gains are changed.
     */
    public void setHasPIDGainsChanged(Runnable runnable) {
        if(hasPIDGainsRunnableSet()) return;
        this.setHasPIDGainsChanged = runnable;
    }
}
