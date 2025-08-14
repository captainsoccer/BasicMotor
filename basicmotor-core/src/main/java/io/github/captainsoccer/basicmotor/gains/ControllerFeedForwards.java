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
public class ControllerFeedForwards {

    /**
     * The type of feed forward gain to change.
     * This is used by the {@link Controller} to update the feed forward gains if needed.
     * There is no support for changing the feed forward function, as it is a custom function that is set when the feed forward gains are created.
     */
    public enum ChangeType {
        /**
         * Changes the simple feed forward gain (adds constant voltage to the output)
         */
        SIMPLE_FEED_FORWARD,
        /**
         * Changes the friction feed forward gain (adds constant voltage to the output based on the direction of travel)
         */
        FRICTION_FEED_FORWARD,
        /**
         * Changes the setpoint feed forward gain (multiplies the setpoint by a constant voltage)
         */
        SETPOINT_FEED_FORWARD
    }

    /**
     * A constant voltage added to the motor output.(volts)
     * This is useful for mechanisms that have a constant force applied to them.
     * (Like an elevator)
     */
    private double simpleFeedForward;

    /**
     * A constant voltage added to the motor output based on the direction of travel.(volts)
     * This is used to counteract friction in the mechanism.
     */
    private double frictionFeedForward;

    /**
     * A voltage that is multiplied by the setpoint of the motor, then added to the output.(volts per unit of control)
     * Useful for mechanisms that require a voltage that is proportional to the setpoint.
     * For example, a flywheel that requires a certain voltage to reach a certain speed.
     * Or any closed loop that controls velocity.
     */
    private double setpointFeedForward;

    /**
     * A functions that takes the setpoint of the controller and returns a Voltage that is added to the output.(volts per unit of control)
     * Useful for mechanisms that require a more complex feed forward calculation.
     * For example, an arm that the force of gravity is the sine of the angle of the arm.
     */
    private final Function<Double, Double> feedForwardFunction;

    /**
     * Creates a feed forward gain with the given values.
     *
     * @param simpleFeedForward   The simple feed forward gain (volts)
     * @param frictionFeedForward The friction feed forward gain (volts) (Greater than or equal to zero)
     * @param setpointFeedForward The setpoint feed forward gain (volts per unit of control) (Greater than or equal to zero)
     * @param feedForwardFunction The feed forward function that takes the setpoint and returns a voltage (volts per unit of control)
     */
    public ControllerFeedForwards(double simpleFeedForward, double frictionFeedForward, double setpointFeedForward, Function<Double, Double> feedForwardFunction) {
        // simple feed forward can be negative, as it is a constant voltage added to the output
        this.simpleFeedForward = simpleFeedForward;

        if (frictionFeedForward < 0)
            throw new IllegalArgumentException("frictionFeedForward must be greater than or equal to zero");
        this.frictionFeedForward = frictionFeedForward;

        if (setpointFeedForward < 0)
            throw new IllegalArgumentException("setpointFeedForward must be greater than or equal to zero");
        this.setpointFeedForward = setpointFeedForward;

        //no checks can be done on the feedForwardFunction, as it is a custom function
        this.feedForwardFunction = feedForwardFunction;
    }

    /**
     * Creates a feed forward that is only a setpoint feed forward gain.
     * Useful for simple flywheels or other mechanisms that only require a setpoint feed forward gain.
     *
     * @param setpointFeedForward The setpoint feed forward gain (volts per unit of control) (Greater than or equal to zero)
     */
    public ControllerFeedForwards(double setpointFeedForward) {
        this(0, 0, setpointFeedForward, (x) -> 0.0);
    }

    /**
     * Creates a feed forward that the setpoint feed forward gain and friction feed forward gain.
     * Useful for simple flywheels or other mechanism that require a setpoint feed forward gain and want to counteract friction.
     *
     * @param setpointFeedForward The setpoint feed forward gain (volts per unit of control) (Greater than or equal to zero)
     * @param frictionFeedForward The friction feed forward gain (volts) (Greater than or equal to zero)
     */
    public ControllerFeedForwards(double setpointFeedForward, double frictionFeedForward) {
        this(0, frictionFeedForward, setpointFeedForward, (x) -> 0.0);
    }

    /**
     * Creates a feed forward that has a setpoint feed forward gain, simple feed forward gain, and friction feed forward gain.
     *
     * @param setpointFeedForward The setpoint feed forward gain (volts per unit of control) (Greater than or equal to zero)
     * @param simpleFeedForward   The simple feed forward gain (volts) (Greater than or equal to zero)
     * @param frictionFeedForward The friction feed forward gain (volts) (Greater than or equal to zero)
     */
    public ControllerFeedForwards(double setpointFeedForward, double simpleFeedForward, double frictionFeedForward) {
        this(frictionFeedForward, simpleFeedForward, setpointFeedForward, (x) -> 0.0);
    }

    /**
     * Creates a feed forward with no gains.
     * Means that there is no feed forward output from the controller.
     */
    public ControllerFeedForwards() {
        this(0, 0, 0, (x) -> 0.0);
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
     * Gets the setpoint feed forward gain.
     *
     * @return The setpoint feed forward gain (volts per unit of control)
     */
    public double getSetpointFeedForward() {
        return setpointFeedForward;
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
                setpointFeedForward * setpoint,
                getCalculatedFeedForward(setpoint),
                arbitraryFeedForward);
    }

    /**
     * Updates the feed forward gains based on the given value and type.
     * Used by the {@link Controller} to update the feed forward gains from the dashboard.
     *
     * @param value The value to set the feed forward gain to (must be greater than or equal to zero)
     * @param type  The type of feed forward gain to change.
     */
    public void updateFeedForwards(double value, ChangeType type) {
        if (value < 0) {
            //ignore negative values
            return;
        }

        //set the value based on the type
        switch (type) {
            case SIMPLE_FEED_FORWARD:
                simpleFeedForward = value;
                break;
            case FRICTION_FEED_FORWARD:
                frictionFeedForward = value;
                break;
            case SETPOINT_FEED_FORWARD:
                setpointFeedForward = value;
                break;
        }
    }
}
