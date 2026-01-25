package io.github.captainsoccer.basicmotor.gains;


import java.util.function.Function;

import io.github.captainsoccer.basicmotor.LogFrame;
import io.github.captainsoccer.basicmotor.controllers.Controller;

/**
 * This class is used to store and calculate feed forward gains for a motor controller.
 * This class supports simple feed forward, friction feed forward, setpoint feed forward, and a custom function for feed forward.
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
         * Changes the simple feed forward gain (adds constant voltage to the output)
         */
        KG,
        /**
         * Changes the friction feed forward gain (adds constant voltage to the output based on the direction of travel)
         */
        FRICTION_FEED_FORWARD,
        /**
         * Changes the deadband for the friction feed forward when in position control mode.
         */
        FRICTION_FEED_FORWARD_DEADBAND,
        /**
         * Changes the setpoint feed forward gain (multiplies the setpoint by a constant voltage)
         */
        KV
    }

    /**
     * The interface that represent a gravity feedForward for mechanisms.
     * Use static methods to create the feedForward
     */
    public interface KG {
        /**
         * calculates the feedforward based on the setpoint
         * @param setpoint the setpoint of the motor (default units are rotations)
         * @return the feedForward to apply in volts
         */
        double calculate(double setpoint);

        /**
         * Creates a feed forward for an elevator mechanism.
         * This does not use the setpoint as the elevator feedforward always works
         * @param kG The feed forward to apply in volts
         * @return the KG to give to the motor
         */
        static KG ELEVATOR(double kG){
            return setpoint -> kG;
        }

        /**
         * creates a feedforward for an arm system based on the cosine of the target angle
         * @param kG The feedforward to apply times the cosine (volts)
         * @return the KG to give to the motor
         */
        static KG ARM_COS(double kG){
            return setpoint -> Math.cos(rotationsToRadians(setpoint)) * kG;
        }

        /**
         * creates a feedforward for an arm system based on the sine of the target angle
         * @param kG The feedforward to apply times the sine (volts)
         * @return the KG to give to the motor
         */
        static KG ARM_SIN(double kG){
            return setpoint -> Math.sin(rotationsToRadians(setpoint)) * kG;
        }

        /**
         * Applies no feedForward
         * @return an empty feedForward
         */
        static KG NONE(){
            return setpoint -> 0;
        }

        /**
         * converts rotation to radians
         * @param rotation the rotation to convert
         * @return the angle in radians
         */
        static double rotationsToRadians(double rotation){
            return rotation / (Math.PI * 2);
        }
    }

    /**
     * The KG gain, used for systems like arms and elevators
     */
    private final KG kG = KG.NONE();

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
     * Or any closed loop that controls velocity.
     */
    private final double kV;

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
     * @param simpleFeedForward           The simple feed forward gain (volts)
     * @param frictionFeedForward         The friction feed forward gain (volts) (Greater than or equal to zero)
     * @param frictionFeedForwardDeadband The deadband for the friction feed forward when in position control mode (unit of control)
     *                                    (Greater than or equal to zero)
     * @param kV                          The setpoint feed forward gain (volts per unit of control) (Greater than or equal to zero)
     * @param feedForwardFunction         The feed forward function that takes the setpoint and returns a voltage (volts per unit of control)
     */
    public FeedForwardsGains(double simpleFeedForward, double frictionFeedForward, double frictionFeedForwardDeadband,
                             double kV, Function<Double, Double> feedForwardFunction) {

        // simple feed forward can be negative, as it is a constant voltage added to the output
        this.simpleFeedForward = simpleFeedForward;

        if (frictionFeedForward < 0)
            throw new IllegalArgumentException("frictionFeedForward must be greater than or equal to zero");
        this.frictionFeedForward = frictionFeedForward;

        if (frictionFeedForwardDeadband < 0)
            throw new IllegalArgumentException("frictionFeedForwardDeadband must be greater than or equal to zero");
        this.frictionFeedForwardDeadband = frictionFeedForwardDeadband;

        if (kV < 0)
            throw new IllegalArgumentException("setpointFeedForward must be greater than or equal to zero");
        this.kV = kV;

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
        this(0, 0, 0, kV, (x) -> 0.0);
    }

    /**
     * Creates a feed forward that the setpoint feed forward gain and friction feed forward gain.
     * Useful for simple flywheels or other mechanism that require a setpoint feed forward gain and want to counteract friction.
     *
     * @param kV                  The setpoint feed forward gain (volts per unit of control) (Greater than or equal to zero)
     * @param frictionFeedForward The friction feed forward gain (volts) (Greater than or equal to zero)
     */
    public FeedForwardsGains(double kV, double frictionFeedForward) {
        this(0, frictionFeedForward, 0, kV, (x) -> 0.0);
    }

    /**
     * Creates a feed forward that has a setpoint feed forward gain, simple feed forward gain, and friction feed forward gain.
     *
     * @param kV                  The setpoint feed forward gain (volts per unit of control) (Greater than or equal to zero)
     * @param simpleFeedForward   The simple feed forward gain (volts)
     * @param frictionFeedForward The friction feed forward gain (volts) (Greater than or equal to zero)
     */
    public FeedForwardsGains(double kV, double simpleFeedForward, double frictionFeedForward) {
        this(simpleFeedForward, frictionFeedForward, 0, kV, (x) -> 0.0);
    }

    /**
     * Creates a feed forward with no gains.
     * Means that there is no feed forward output from the controller.
     */
    public FeedForwardsGains() {
        this(0, 0, 0, 0, (x) -> 0.0);
    }

    /**
     * Gets the simple feed forward gain.
     *
     * @return The simple feed forward gain (volts)
     */
    public double getSimpleFeedForward() {
        return simpleFeedForward;
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
     * Gets the setpoint feed forward gain.
     *
     * @return The setpoint feed forward gain (volts per unit of control)
     */
    public double getkV() {
        return kV;
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
     * Calculates the direction of travel of the mechanism based on the setpoint and measurement.
     * This is used to determine the direction of travel for the friction feed forward.
     * If using position control mode, it will check if the motor is within the
     * {@link #frictionFeedForwardDeadband} of the setpoint to determine if the motor is stationary.
     *
     * @param setpoint    The setpoint of the controller (unit of control)
     * @param measurement The current measurement of the controller (unit of control)
     * @param controlMode The control mode of the controller.
     * @return The direction of travel of the mechanism (1 for forward, -1 for backward, 0 for stationary)
     */
    public double calculateDirectionOfTravel(double setpoint, double measurement, Controller.ControlMode controlMode) {
        if (controlMode.isPositionControl()) {
            double delta = setpoint - measurement;

            if (Math.abs(delta) <= frictionFeedForwardDeadband)
                return 0.0;
            else
                return Math.signum(delta);
        } else return Math.signum(setpoint);
    }

    /**
     * Calculates the feed forward output of the controller.
     * This includes the simple feed forward, friction feed forward, setpoint feed forward,
     * feed forward function output, and any arbitrary feed forward value.
     *
     * @param setpoint             The setpoint of the controller (unit of control)
     * @param directionOfTravel    The direction of travel of the mechanism (1 for forward, -1 for backward)
     * @param arbitraryFeedForward The arbitrary feed forward provided by the user (volts)
     * @return The feed forward output of the controller.
     */
    public LogFrame.FeedForwardOutput calculateFeedForwardOutput(
            double setpoint, double directionOfTravel, double arbitraryFeedForward) {
        return new LogFrame.FeedForwardOutput(
                simpleFeedForward,
                frictionFeedForward * directionOfTravel,
                kV * setpoint,
                getCalculatedFeedForward(setpoint),
                arbitraryFeedForward);
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
        var array = new Double[]{simpleFeedForward, frictionFeedForward, frictionFeedForwardDeadband, kV};

        if (array[type.ordinal()] == value) return this;

        array[type.ordinal()] = value;

        return new FeedForwardsGains(array[0], array[1], array[2], array[3], feedForwardFunction);
    }
}
