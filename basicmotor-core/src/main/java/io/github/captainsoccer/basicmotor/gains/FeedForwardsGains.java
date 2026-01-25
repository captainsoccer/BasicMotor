package io.github.captainsoccer.basicmotor.gains;


import java.util.function.Function;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import io.github.captainsoccer.basicmotor.LogFrame;
import io.github.captainsoccer.basicmotor.controllers.Controller;

/**
 * This class is used to store and calculate feed forward gains for a motor controller.
 * This class supports a gravity feedforward, friction feedforward, velocity feedforward, acceleration feedforward and a custom feedforward function.
 * Values here can be updated from the dashboard through the {@link Controller}.
 * All feed forwards are calculated in volts.
 * You can calculate most of the feed forwards using tools like SysID.
 * For more info about the use of feed forwards and calculating them, see the <a href="https://github.com/captainsoccer/BasicMotor/wiki/Configuration#feedforwardconfig--feedforward-tuning">wiki</a>
 * Also more info about the DC motor equation that uses these feed forwards can be found in the
 * <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html">wpilib docs</a>.
 */
public class FeedForwardsGains {

    /**
     * The type of feed forward gain to change.
     * This is used by the {@link Controller} to update the feed forward gains if needed.
     * There is no support for changing the feed forward function, as it is a custom function that is set when the feed forward gains are created.
     */
    public enum ChangeType {
        /**
         * Changes the friction feed forward gain (adds constant voltage to the output based on the direction of travel)
         */
        FRICTION_FEED_FORWARD,
        /**
         * Changes the deadband for the friction feed forward when in position control mode.
         */
        FRICTION_FEED_FORWARD_DEADBAND,
        /**
         * Changes the velocity feed forward gain (multiplies the target velocity by a constant voltage)
         */
        KV,
        /**
         * Changes the acceleration feedforward gain (only applies to motion profiling)
         */
        KA
    }

    /**
     * The interface that represent a gravity feedForward for mechanisms.
     * Use static methods to create the feedForward
     */
    @FunctionalInterface
    public interface KG {
        /**
         * calculates the feedforward based on the setpoint
         *
         * @param setpoint the setpoint of the motor (default units are rotations)
         * @return the feedForward to apply in volts
         */
        double calculate(double setpoint);

        /**
         * Creates a feed forward for an elevator mechanism.
         * This does not use the setpoint as the elevator feedforward always works
         *
         * @param kG The feed forward to apply in volts
         * @return the KG to give to the motor
         */
        static KG ELEVATOR(double kG) {
            return setpoint -> kG;
        }

        /**
         * creates a feedforward for an arm system based on the cosine of the target angle
         *
         * @param kG The feedforward to apply times the cosine (volts)
         * @return the KG to give to the motor
         */
        static KG ARM_COS(double kG) {
            return setpoint -> Math.cos(rotationsToRadians(setpoint)) * kG;
        }

        /**
         * creates a feedforward for an arm system based on the sine of the target angle
         *
         * @param kG The feedforward to apply times the sine (volts)
         * @return the KG to give to the motor
         */
        static KG ARM_SIN(double kG) {
            return setpoint -> Math.sin(rotationsToRadians(setpoint)) * kG;
        }

        /**
         * Applies no feedForward
         *
         * @return an empty feedForward
         */
        static KG NONE() {
            return setpoint -> 0;
        }

        /**
         * converts rotation to radians
         *
         * @param rotation the rotation to convert
         * @return the angle in radians
         */
        static double rotationsToRadians(double rotation) {
            return rotation / (Math.PI * 2);
        }
    }

    /**
     * The KG gain, used for systems like arms and elevators.
     * this feedforward applies only to position control.
     */
    private final KG kG;

    /**
     * A constant voltage added to the motor output based on the direction of travel.(volts)
     * This is used to counteract friction in the mechanism.
     */
    private final double frictionFeedForward;

    /**
     * The deadband for the friction feed forward when in position control mode.
     * If the motor is within this deadband of the setpoint, the friction feed forward will not be applied.
     * This is to prevent the motor from oscillating around the setpoint when it is close to it.
     * Used only in position control mode.
     */
    private final double frictionFeedForwardDeadband;

    /**
     * A voltage that is multiplied by the setpoint of the motor, then added to the output.(volts per unit of control)
     * Useful for mechanisms that require a voltage that is proportional to the setpoint.
     * For example, a flywheel that requires a certain voltage to reach a certain speed.
     * Used only for velocity control.
     */
    private final double kV;

    /**
     * A voltage that is multiplied by the acceleration of the setpoint when using motion profiles.
     * This feedforward only applies when using motion profile.
     * This improves the behavior of the mechanism using a motion profile.
     */
    private final double kA;

    /**
     * A functions that takes the setpoint of the controller and returns a Voltage that is added to the output.(volts per unit of control)
     * Useful for mechanisms that require a more complex feed forward calculation.
     * For example, an arm that the force of gravity is the sine of the angle of the arm.
     */
    private final Function<Double, Double> feedForwardFunction;

    /**
     * Creates a feed forward gain with the given values.
     *
     * @param kG                          The gravity feedForward gain
     * @param frictionFeedForward         The friction feed forward gain (volts) (Greater than or equal to zero)
     * @param frictionFeedForwardDeadband The deadband for the friction feed forward when in position control mode (unit of control)
     *                                    (Greater than or equal to zero)
     * @param kV                          The setpoint feed forward gain (volts per unit of control) (Greater than or equal to zero)
     * @param kA                          The acceleration feedforward gain
     * @param feedForwardFunction         The feed forward function that takes the setpoint and returns a voltage (volts per unit of control)
     */
    public FeedForwardsGains(KG kG, double frictionFeedForward, double frictionFeedForwardDeadband,
                             double kV, double kA, Function<Double, Double> feedForwardFunction) {

        // simple feed forward can be negative, as it is a constant voltage added to the output
        this.kG = kG;

        if (frictionFeedForward < 0)
            throw new IllegalArgumentException("frictionFeedForward must be greater than or equal to zero");
        this.frictionFeedForward = frictionFeedForward;

        if (frictionFeedForwardDeadband < 0)
            throw new IllegalArgumentException("frictionFeedForwardDeadband must be greater than or equal to zero");
        this.frictionFeedForwardDeadband = frictionFeedForwardDeadband;

        if (kV < 0)
            throw new IllegalArgumentException("setpointFeedForward must be greater than or equal to zero");
        this.kV = kV;

        if (kA < 0)
            throw new IllegalArgumentException("setpointFeedForward must be greater than or equal to zero");
        this.kA = kA;

        //no checks can be done on the feedForwardFunction, as it is a custom function
        this.feedForwardFunction = feedForwardFunction;
    }

    /**
     * Creates a feed forward that is only a setpoint feed forward gain.
     * Useful for simple flywheels or other mechanisms that only require a setpoint feed forward gain.
     *
     * @param kV The setpoint feed forward gain (volts per unit of control) (Greater than or equal to zero)
     */
    public FeedForwardsGains(double kV) {
        this(KG.NONE(), 0, 0, kV, 0, (x) -> 0.0);
    }

    /**
     * Creates a feed forward that the setpoint feed forward gain and friction feed forward gain.
     * Useful for simple flywheels or other mechanism that require a setpoint feed forward gain and want to counteract friction.
     *
     * @param kV                  The setpoint feed forward gain (volts per unit of control) (Greater than or equal to zero)
     * @param frictionFeedForward The friction feed forward gain (volts) (Greater than or equal to zero)
     */
    public FeedForwardsGains(double kV, double frictionFeedForward) {
        this(KG.NONE(), frictionFeedForward, 0, kV, 0, (x) -> 0.0);
    }

    /**
     * Creates a feed forward that has a velocity feed forward gain, gravity forward gain, and friction feed forward gain.
     *
     * @param kV                  The setpoint feed forward gain (volts per unit of control) (Greater than or equal to zero)
     * @param kG                  The gravity feedforward gain
     * @param frictionFeedForward The friction feed forward gain (volts) (Greater than or equal to zero)
     */
    public FeedForwardsGains(double kV, KG kG, double frictionFeedForward) {
        this(kG, frictionFeedForward, 0, kV, 0, (x) -> 0.0);
    }

    /**
     * Creates a feed forward with no gains.
     * Means that there is no feed forward output from the controller.
     */
    public FeedForwardsGains() {
        this(KG.NONE(), 0, 0, 0, 0, (x) -> 0.0);
    }

    /**
     * Gets the gravity feedforward gain.
     *
     * @return the KG gain (function that returns voltage)
     */
    public KG getKG() {
        return kG;
    }

    /**
     * Gets the friction feed forward gain.
     *
     * @return The friction feed forward gain (volts)
     */
    public double getFrictionFeedForward() {
        return frictionFeedForward;
    }

    /**
     * Gets the deadband for the friction feed forward when in position control mode.
     *
     * @return The deadband for the friction feed forward (unit of control)
     */
    public double getFrictionFeedForwardDeadband() {
        return frictionFeedForwardDeadband;
    }

    /**
     * Gets the velocity feedforward gain
     *
     * @return The velocity feedforward gain (volts per unit of control)
     */
    public double getKV() {
        return kV;
    }

    /**
     * Gets the acceleration feedforward
     * This only applies to motion profiles
     *
     * @return The acceleration feedforward (volts per unit of control squared)
     */
    public double getKA() {
        return kA;
    }

    /**
     * Gets the feed forward function.
     *
     * @return The feed forward function that takes the setpoint and returns a voltage (volts per unit of control)
     */
    public Function<Double, Double> getFeedForwardFunction() {
        return feedForwardFunction;
    }

    /**
     * Calculates the output of the feed forward function based on the setpoint.
     * Only returns the output of the feed forward function without any other feed forward gains.
     *
     * @param setpoint The setpoint of the controller (unit of control)
     * @return The calculated feed forward output (volts)
     */
    public double getCalculatedFeedForward(double setpoint) {
        return feedForwardFunction.apply(setpoint);
    }

    /**
     * Calculates the feed forward output of the controller.
     * This includes all the feedforwards stored.
     *
     * @param setpoint             The setpoint of the controller (unit of control)
     * @param measurement          The measurement of the controller, used to calculate the friction feedforward
     * @param controlMode          The control mode of the controller, used to determine which feedforwards to use.
     * @param arbitraryFeedForward The arbitrary feed forward provided by the user (volts)
     * @return The feed forward output of the controller.
     */
    public LogFrame.FeedForwardOutput calculateFeedForwardOutput(
            TrapezoidProfile.State setpoint, double measurement, Controller.ControlMode controlMode, double arbitraryFeedForward) {

        return new LogFrame.FeedForwardOutput(
                controlMode.isPositionControl() ? kG.calculate(setpoint.position) : 0,
                calculateFrictionFeedForward(setpoint.position, measurement, controlMode),
                calculateKV(setpoint, controlMode),
                controlMode == Controller.ControlMode.PROFILED_VELOCITY ? setpoint.velocity * kA : 0,
                controlMode.requiresPID() ? getCalculatedFeedForward(setpoint.position) : 0,
                arbitraryFeedForward
        );
    }

    /**
     * Calculates the velocity feedForward based on the control mode and the setpoint.
     * the velocity feedforward is used only when using velocity control or using profiled position
     *
     * @param setpoint    the setpoint of the motor
     * @param controlMode the control mode of the motor
     * @return the calculated velocity feedForward
     */
    private double calculateKV(TrapezoidProfile.State setpoint, Controller.ControlMode controlMode) {
        if (controlMode.isVelocityControl()) return setpoint.position * kV;

        if (controlMode == Controller.ControlMode.PROFILED_POSITION) return setpoint.velocity * kV;

        return 0;
    }

    /**
     * Calculates the friction feedForward based on the setpoint measurement and control mode.
     * the friction feedforward only applies when using either position control mode or velocity.
     * if using position control mode it checks if the error is within the {@link #frictionFeedForwardDeadband}.
     * @param setpoint the setpoint of the motor
     * @param measurement the measurement of the motor
     * @param controlMode the control mode of the motor
     * @return the friction feedForward to apply
     */
    private double calculateFrictionFeedForward(double setpoint, double measurement, Controller.ControlMode controlMode) {
        if (controlMode.isPositionControl()) {
            double delta = setpoint - measurement;

            if (Math.abs(delta) <= frictionFeedForwardDeadband)
                return 0.0;
            else
                return Math.signum(delta) * frictionFeedForward;
        }
        else if(controlMode.isVelocityControl())
            return Math.signum(setpoint) * frictionFeedForward;

        return 0;
    }

    /**
     * Updates the feed forward gains based on the given value and type.
     * Used by the {@link Controller} to update the feed forward gains from the dashboard.
     *
     * @param value The value to set the feed forward gain to (must be greater than or equal to zero)
     * @param type  The type of feed forward gain to change.
     * @return A new FeedForwardsGains object with the updated feed forward gain.
     */
    public FeedForwardsGains changeValue(double value, ChangeType type) {
        var array = new Double[]{frictionFeedForward, frictionFeedForwardDeadband, kV, kA};

        if (array[type.ordinal()] == value) return this;

        array[type.ordinal()] = value;

        return new FeedForwardsGains(kG, array[0], array[1], array[2], array[3], feedForwardFunction);
    }
}
