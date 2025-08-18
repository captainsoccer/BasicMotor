package io.github.captainsoccer.basicmotor.gains;

import io.github.captainsoccer.basicmotor.motorManager.MotorManager;
import io.github.captainsoccer.basicmotor.controllers.BasicPIDController;

/**
 * This class contains the PID gains for a PID controller.
 * Used mainly by the {@link BasicPIDController}.
 */
public class PIDGains {

    /**
     * What kind of gain to change in the PID controller.
     * Used for updating the PID gains from the dashboard.
     */
    public enum ChangeType {
        /**
         * Changes the proportional gain (k_P)
         */
        K_P,
        /**
         * Changes the integral gain (k_I)
         */
        K_I,
        /**
         * Changes the derivative gain (k_D)
         */
        K_D,
        /**
         * Changes the integrator zone (i_Zone)
         */
        I_ZONE,
        /**
         * Changes the maximum accumulation of the integrator (i_maxAccum)
         */
        I_MAX_ACCUM,
        /**
         * Changes the tolerance of the PID controller
         */
        TOLERANCE
    }

    private final double loopTimeSeconds;

    /**
     * The PID elements of the PID controller.
     */
    private double k_P, k_I, K_D;

    /**
     * The integrator limiters.
     * This is used to limit the accumulation of the integrator to prevent windup.
     * The i_Zone is the zone where the integrator is active,
     * and the i_maxAccum is the maximum value that the integrator can accumulate.
     */
    private double i_Zone, i_maxAccum;

    /**
     * The error tolerance of the PID controller.
     * Any error less than this value will be considered as at setpoint.
     */
    private double tolerance;

    /**
     * Creates a PID gains object with the given values.
     * Use this constructor only if you know what you are doing.
     *
     * @param k_P        The proportional gain (>= 0) (units are volts per unit of control)
     * @param k_I        The integral gain (>= 0) (units are volt seconds per unit of control)
     * @param k_D        The derivative gain (>= 0) (units are volts per unit of control per second)
     * @param i_Zone     The integrator zone (>= 0) (units are unit of control).
     *                   If this is zero, I accumulation is disabled.
     *                   The Default value is infinity, meaning the integrator is always active.
     * @param i_maxAccum The maximum accumulation of the integrator (>= 0) (units are volts)
     *                   If this is zero, I accumulation is disabled.
     *                   The Default value is the maximum motor output of the motor controller.
     * @param tolerance  The tolerance of the PID controller (>= 0) (units are unit of control)
     * @param loopTimeSeconds The loop time of the PID controller in seconds.
     *                        This is used to convert the integral and derivative gains to the correct units.
     */
    public PIDGains(double k_P, double k_I, double k_D, double i_Zone, double i_maxAccum, double tolerance, double loopTimeSeconds) {
        if(loopTimeSeconds <= 0) {
            throw new IllegalArgumentException("loopTimeSeconds must be greater than zero");
        }
        this.loopTimeSeconds = loopTimeSeconds;

        if (k_P < 0) throw new IllegalArgumentException("k_P must be greater than zero");
        this.k_P = k_P;

        if (k_I < 0) throw new IllegalArgumentException("k_I must be greater than zero");
        this.k_I = k_I / loopTimeSeconds; // convert to volt times loop time seconds per unit of control

        if (k_D < 0) throw new IllegalArgumentException("k_D must be greater than zero");
        this.K_D = k_D * loopTimeSeconds; // convert to volts per unit of control times loop time seconds

        if (i_Zone < 0) throw new IllegalArgumentException("i_Zone must be greater than zero");
        this.i_Zone = i_Zone;

        if (i_maxAccum < 0) throw new IllegalArgumentException("i_maxAccum must be greater than zero");
        this.i_maxAccum = i_maxAccum;

        if (tolerance < 0) throw new IllegalArgumentException("tolerance must be greater than zero");
        this.tolerance = tolerance;
    }

    /**
     * Creates a PID gains object with the given values.
     * This will use the default loop time of the RIO pid controller.
     * (Will convert units automatically to the correct units for the RIO controller)
     *
     * @param k_P        The proportional gain (>= 0) (units are volts per unit of control)
     * @param k_I        The integral gain (>= 0) (units are volt seconds per unit of control)
     * @param k_D        The derivative gain (>= 0) (units are volts per unit of control per second)
     * @param i_Zone     The integrator zone (>= 0) (units are unit of control).
     *                   If this is zero, I accumulation is disabled.
     *                   The Default value is infinity, meaning the integrator is always active.
     * @param i_maxAccum The maximum accumulation of the integrator (>= 0) (units are volts)
     *                   If this is zero, I accumulation is disabled.
     *                   The Default value is the maximum motor output of the motor controller.
     * @param tolerance  The tolerance of the PID controller (>= 0) (units are unit of control)
     */
    public PIDGains(double k_P, double k_I, double k_D, double i_Zone, double i_maxAccum, double tolerance) {
        this(k_P, k_I, k_D, i_Zone, i_maxAccum, tolerance, MotorManager.ControllerLocation.RIO.getSeconds());
    }

    /**
     * Creates a PID gains object with the given values.
     *
     * @param k_P The proportional gain (>= 0) (units are volts per unit of control)
     * @param k_I The integral gain (>= 0) (units are volt seconds per unit of control)
     * @param k_D The derivative gain (>= 0) (units are volts per unit of control per second)
     */
    public PIDGains(double k_P, double k_I, double k_D) {
        this(k_P, k_I, k_D, Double.POSITIVE_INFINITY, MotorManager.config.defaultMaxMotorOutput(), 0);
    }

    /**
     * Creates a PID gains object with the given values and a tolerance.
     *
     * @param k_P       The proportional gain (>= 0) (units are volts per unit of control)
     * @param k_I       The integral gain (>= 0) (units are volt seconds per unit of control)
     * @param k_D       The derivative gain (>= 0) (units are volts per unit of control per second)
     * @param tolerance The tolerance of the PID controller (>= 0) (units are unit of control)
     */
    public PIDGains(double k_P, double k_I, double k_D, double tolerance) {
        this(k_P, k_I, k_D, Double.POSITIVE_INFINITY, MotorManager.config.defaultMaxMotorOutput(), tolerance);
    }

    /**
     * Creates an empty PID gains object.
     */
    public PIDGains() {
        this(0, 0, 0);
    }

    /**
     * Gets the proportional gain of the PID controller.
     * Units are volts per unit of control.
     *
     * @return The proportional gain of the PID controller
     */
    public double getK_P() {
        return k_P;
    }

    /**
     * Gets the integral gain of the PID controller.
     * Units are volt seconds per unit of control.
     *
     * @return The integral gain of the PID controller
     */
    public double getK_I() {
        return k_I;
    }

    /**
     * Gets the derivative gain of the PID controller.
     * Units are volts per unit of control per second.
     *
     * @return The derivative gain of the PID controller
     */
    public double getK_D() {
        return K_D;
    }

    /**
     * Gets the integrator zone of the PID controller.
     * The integrator zone is the zone where the integrator is active.
     * Units are unit of control.
     *
     * @return The integrator zone of the PID controller
     */
    public double getI_Zone() {
        return i_Zone;
    }

    /**
     * Gets the maximum accumulation of the integrator of the PID controller.
     * This is used to limit the accumulation of the integrator to prevent windup.
     * Units are volts.
     *
     * @return The maximum accumulation of the integrator of the PID controller
     */
    public double getI_MaxAccum() {
        return i_maxAccum;
    }

    /**
     * Gets the tolerance of the PID controller.
     * This is the error tolerance of the PID controller.
     * Any error less than this value will be considered as at setpoint.
     * Units are unit of control.
     *
     * @return The tolerance of the PID controller
     */
    public double getTolerance() {
        return tolerance;
    }

    /**
     * Gets the loop time of the PID controller.
     * This is used to convert the integral and derivative gains to the correct units.
     * @return The loop time of the PID controller in seconds.
     */
    public double getLoopTimeSeconds() {
        return loopTimeSeconds;
    }

    /**
     * Converts the PID gains to motor gains.
     * Needed because the motor unit of control is different from the mechanisms unit of control.
     *
     * @param gearRatio      The gear ratio of the motor (unitless)
     * @param unitConversion the value that will be divided to get rotations.
     * @param loopTimeSeconds The loop time of the new pid controller in seconds.
     *                       This is used to convert the integral and derivative gains to the correct units.
     * @return The motor gains with the same PID structure but adjusted for the motor's unit of control.
     */
    public PIDGains convertToMotorGains(double gearRatio, double unitConversion, double loopTimeSeconds) {
        //motors don't support infinite i_Zone, so we set it to 0
        double i_Zone = this.i_Zone == Double.POSITIVE_INFINITY ? 0 : this.i_Zone;

        return new PIDGains(
                (k_P / gearRatio) * unitConversion,
                ((k_I / gearRatio) * unitConversion) * this.loopTimeSeconds, //converts to volt seconds per unit of control
                ((K_D / gearRatio) * unitConversion) / this.loopTimeSeconds, //converts to volts per unit of control per second
                (i_Zone / unitConversion) * gearRatio,
                i_maxAccum, //stays at volts
                (tolerance / unitConversion) * gearRatio,
                loopTimeSeconds);
    }

    /**
     * Converts the PID gains to duty cycle gains.
     * This is used for motors that their built-in controller uses duty cycle instead of voltage.
     *
     * @return The PID gains with the same PID structure but adjusted for duty cycle.
     */
    public PIDGains convertToDutyCycle() {
        double motorIdleVoltage = MotorManager.config.motorIdealVoltage();

        return new PIDGains(
                k_P / motorIdleVoltage,
                k_I / motorIdleVoltage,
                K_D / motorIdleVoltage,
                i_Zone,
                i_maxAccum / motorIdleVoltage,
                tolerance);
    }

    /**
     * Updates the PID gains based on the change type.
     * If the value is negative, it will be ignored.
     * If the value is the same as the current value, it will not change.
     *
     * @param value      The value to set the PID gain to (must be greater than or equal to zero)
     * @param changeType Which gain to change
     * @return True if the value has changed, false otherwise.
     */
    public boolean updatePIDGains(double value, ChangeType changeType) {
        //ignore negative values for gains
        if (value < 0) return false;

        double oldValue = getValue(changeType);

        if (oldValue == value) {
            return false; // no change
        }

        setValue(value, changeType);
        return true; // value has changed
    }

    /**
     * Sets the value of the PID gain based on the change type.
     * Any value sent to here must be post checking for negative values.
     * Used by the {@link #updatePIDGains(double, ChangeType)} method
     *
     * @param value      The value to set the PID gain to (must be greater than or equal to zero)
     * @param changeType Which gain to change
     */
    private void setValue(double value, ChangeType changeType) {
        switch (changeType) {
            case K_P -> k_P = value;
            case K_I -> k_I = value;
            case K_D -> K_D = value;
            case I_ZONE -> i_Zone = value;
            case I_MAX_ACCUM -> i_maxAccum = value;
            case TOLERANCE -> tolerance = value;
        }
    }

    /**
     * Gets the current value of the PID gain based on the change type.
     * Used for the {@link #updatePIDGains(double, ChangeType)} method
     *
     * @param changeType Which gain to get the value of
     * @return The current value of the PID gain
     */
    private double getValue(ChangeType changeType) {
        return switch (changeType) {
            case K_P -> k_P;
            case K_I -> k_I;
            case K_D -> K_D;
            case I_ZONE -> i_Zone;
            case I_MAX_ACCUM -> i_maxAccum;
            case TOLERANCE -> tolerance;
        };
    }
}
