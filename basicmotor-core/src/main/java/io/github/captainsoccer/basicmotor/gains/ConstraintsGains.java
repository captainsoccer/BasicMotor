package io.github.captainsoccer.basicmotor.gains;

import io.github.captainsoccer.basicmotor.motorManager.MotorManager;
import io.github.captainsoccer.basicmotor.controllers.Controller;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import edu.wpi.first.math.MathUtil;

/**
 * This class defines the constraints for a controller.
 * It contains the soft limits, continuous wrap, output limits, and deadband for the controller.
 * Some of the constraints are not available for every controller when running pid on the motor controller,
 * but they are available when running the pid on the RIO.
 */
public class ConstraintsGains {
    /**
     * Represents the type of constraint used
     */
    public enum ConstraintType {
        /**
         * Continuous (also known as continuous wrap).
         * It means that the mechanism is in some sort of loop.
         * This means the motor will go the shortest way to the goal.
         * (range defined by the max and min values)
         * This is commonly used for swerve steer motors.
         * This constraint is only supported for position control.
         */
        CONTINUOUS,
        /**
         * Limited (also known as soft limits).
         * It means that the mechanism has a limited range of motion.
         * This means the motor will not go beyond the limits.
         * Commonly used for arm motors or other mechanisms with a limited range of motion.
         */
        LIMITED,
        /**
         * No constraints.
         * This means that the controller has no constraints.
         * good for flywheels or other mechanisms that do not need constraints.
         */
        NONE
    }

    /**
     * The type of constraint to use
     */
    private final ConstraintType constraintType;

    /**
     * The minimum value of the constraints.
     * Purpose depends on the type of constraints.
     */
    private final double minValue;

    /**
     * the maximum value of the constraints.
     * Purpose depends on the type of constraints.
     */
    private final double maxValue;

    /**
     * The maximum output of the motor (in volts).
     * This is the max voltage (power or force) the motor can output in the positive direction.
     * This is used for capping the output of the motor.
     * Default is defined in {@link MotorManager#config}.
     */
    private final double maxMotorOutput;

    /**
     * the minimum output of the motor (in volts).
     * This is the max voltage (power or force) the motor can output in the negative direction.
     * This is used for capping the output of the motor.
     * Default is defined in {@link MotorManager#config}.
     */
    private final double minMotorOutput;

    /**
     * The minimum absolute voltage the motor needs to apply.(in volts)
     * Any value below this will be ignored and the motor will not move.
     * This is used to prevent the motor from stalling for no reason or to prevent jittering.
     * This is also known as deadband.
     */
    private final double voltageDeadband;

    /**
     * The maximum rate the voltage can change per second.(in volts per second)
     * This is used to prevent the motor from changing too quickly and causing damage to the mechanism.
     */
    private final double voltageRampRate;

    /**
     * Creates a constraints object with the given type and limits.
     *
     * @param type           The type of the constraints (continuous, limited, none).
     *                       If null, it will be set to NONE.
     * @param minValue       The minimum value of the constraints.
     *                       The purpose depends on the type of constraints:
     *                       - If continuous, this is the minimum value to round to.
     *                       - If limited, this is the minimum value of the limits.
     * @param maxValue       The maximum value of the constraints.
     *                       The purpose depends on the type of constraints:
     *                       - If continuous, this is the maximum value to round to.
     *                       - If limited, this is the maximum value of the limits.
     * @param maxMotorOutput The maximum output of the motor (in volts).
     *                       This is used for capping the output of the motor.
     *                       (default is 13.0), can be defined in {@link MotorManager#config}.
     * @param minMotorOutput The minimum output of the motor (in volts).
     *                       This is used for capping the output of the motor.
     *                       (default is -13.0), can be defined in {@link MotorManager#config}.
     * @param deadband       The minimum output voltage of the motor.
     *                       any value (absolute value) below this will be ignored.
     * @param rampRate       The maximum rate the voltage can change per second.(in volts per second)
     *                       This is used to prevent the motor from changing too quickly and causing damage to the mechanism.
     */
    public ConstraintsGains(ConstraintType type, double minValue, double maxValue, double maxMotorOutput, double minMotorOutput, double deadband, double rampRate) {
        // if the type is null, set it to NONE
        if (type == null) this.constraintType = ConstraintType.NONE;
        else this.constraintType = type;

        // check if the min and max values are valid
        if (minValue > maxValue) {
            throw new IllegalArgumentException(
                    "the minimum value must be less than or equal to the maximum value");
        }
        this.minValue = minValue;
        this.maxValue = maxValue;

        this.maxMotorOutput = MathUtil.clamp(Math.abs(maxMotorOutput), 0, MotorManager.config.defaultMaxMotorOutput);
        this.minMotorOutput = -MathUtil.clamp(Math.abs(minMotorOutput), 0, MotorManager.config.defaultMaxMotorOutput);

        this.voltageDeadband = Math.abs(deadband);

        this.voltageRampRate = Math.abs(rampRate);
    }

    /**
     * Creates a constraints object with the given type and limits with the default max motor output and no deadband.
     *
     * @param type     The type of the constraints (continuous, limited, none).
     *                 If null, it will be set to NONE.
     * @param minValue The minimum value of the constraints.
     *                 The purpose depends on the type of constraints:
     *                 - If continuous, this is the minimum value to round to.
     *                 - If limited, this is the minimum value of the limits.
     * @param maxValue The maximum value of the constraints.
     *                 The purpose depends on the type of constraints:
     *                 - If continuous, this is the maximum value to round to.
     *                 - If limited, this is the maximum value of the limits.
     */
    public ConstraintsGains(ConstraintType type, double minValue, double maxValue) {
        this(type, minValue, maxValue, MotorManager.config.defaultMaxMotorOutput, -MotorManager.config.defaultMaxMotorOutput, 0, 0);
    }

    /**
     * Creates a constraints object with the given type and limits with a voltage deadband.
     *
     * @param type            The type of the constraints (continuous, limited, none).
     *                        If null, it will be set to NONE.
     * @param minValue        The minimum value of the constraints.
     *                        The purpose depends on the type of constraints:
     *                        - If continuous, this is the minimum value to round to.
     *                        - If limited, this is the minimum value of the limits.
     * @param maxValue        The maximum value of the constraints.
     *                        The purpose depends on the type of constraints:
     *                        - If continuous, this is the maximum value to round to.
     *                        - If limited, this is the maximum value of the limits.
     * @param voltageDeadband The minimum output voltage of the motor.
     *                        any value (absolute value) below this will be ignored.
     */
    public ConstraintsGains(ConstraintType type, double minValue, double maxValue, double voltageDeadband) {
        this(type, minValue, maxValue, MotorManager.config.defaultMaxMotorOutput, -MotorManager.config.defaultMaxMotorOutput, voltageDeadband, 0);
    }

    /**
     * Creates a constraints object with only output limits.
     *
     * @param maxMotorOutput The maximum output of the motor (in volts).
     *                       This is used for capping the output of the motor.
     *                       (default is 13.0), can be defined in {@link MotorManager#config}.
     * @param minMotorOutput The minimum output of the motor (in volts).
     *                       This is used for capping the output of the motor.
     *                       (default is -13.0), can be defined in {@link MotorManager#config}
     */
    public ConstraintsGains(double maxMotorOutput, double minMotorOutput) {
        this(ConstraintType.NONE, 0, 0, maxMotorOutput, minMotorOutput, 0, 0);
    }

    /**
     * Creates a constraints object with only output limits and a voltage deadband.
     *
     * @param maxMotorOutput  The maximum output of the motor (in volts).
     *                        This is used for capping the output of the motor.
     *                        (default is 13.0), can be defined in {@link MotorManager#config}.
     * @param minMotorOutput  The minimum output of the motor (in volts).
     *                        This is used for capping the output of the motor.
     *                        (default is -13.0), can be defined in {@link MotorManager#config}
     * @param voltageDeadband The minimum output voltage of the motor.
     *                        any value (absolute value) below this will be ignored.
     * @param rampRate        The maximum rate the voltage can change per second.(in volts per second)
     *                        This is used to prevent the motor from changing too quickly and causing damage to the mechanism.
     */
    public ConstraintsGains(double maxMotorOutput, double minMotorOutput, double voltageDeadband, double rampRate) {
        this(ConstraintType.NONE, 0, 0, maxMotorOutput, minMotorOutput, voltageDeadband, rampRate);
    }

    /**
     * Creates a constraints object with only output limits and a voltage deadband.
     *
     * @param maxMotorOutput  The maximum output of the motor (in volts).
     *                        This is used for capping the output of the motor.
     *                        (default is 13.0), can be defined in {@link MotorManager#config}.
     * @param minMotorOutput  The minimum output of the motor (in volts).
     *                        This is used for capping the output of the motor.
     *                        (default is -13.0), can be defined in {@link MotorManager#config}
     * @param voltageDeadband The minimum output voltage of the motor.
     *                        any value (absolute value) below this will be ignored.
     */
    public ConstraintsGains(double maxMotorOutput, double minMotorOutput, double voltageDeadband) {
        this(ConstraintType.NONE, 0, 0, maxMotorOutput, minMotorOutput, voltageDeadband, 0);
    }

    /**
     * Creates a constraints object with only the voltage deadband.
     *
     * @param voltageDeadband The minimum output voltage of the motor.
     *                        any value (absolute value) below this will be ignored.
     */
    public ConstraintsGains(double voltageDeadband) {
        this(MotorManager.config.defaultMaxMotorOutput, -MotorManager.config.defaultMaxMotorOutput, voltageDeadband);
    }

    /**
     * Creates a constraints object with no constraints, no output limits, and no deadband.
     */
    public ConstraintsGains() {
        this(MotorManager.config.defaultMaxMotorOutput, -MotorManager.config.defaultMaxMotorOutput);
    }

    /**
     * Preforms the calculations of the constraints based on the type of constraints.
     * This method should be called by the controller.
     * It will apply the constraints directly to the request, therefore it is void.
     *
     * @param measurement The current measurement of the motor.
     * @param request     The latest request of the controller.
     */
    public void calculateConstraints(Measurements.Measurement measurement, Controller.ControllerRequest request) {
        switch (constraintType) {
            case LIMITED -> calculateLimited(measurement, request);
            case CONTINUOUS -> calculateContinuous(measurement, request);
            default -> {
                // do nothing
            }
        }
    }

    /**
     * Preforms the calculations of the constraints with the limited mode.
     * This will apply the soft limits to the goal of the controller request.
     * {@link Controller.ControlMode#PROFILED_VELOCITY} does not support soft limits.
     *
     * @param measurement the measurement of the motor
     * @param request     the request of the controller
     */
    public void calculateLimited(Measurements.Measurement measurement, Controller.ControllerRequest request) {
        if (request.controlMode() == Controller.ControlMode.PROFILED_VELOCITY) {
            // Profiled velocity does not support soft limits, so we return
            return;
        }

        var controlMode = request.controlMode();
        // check if the request is a position control (then apply the limits to the setpoint)
        if (controlMode.isPositionControl()) {
            // check if the request is in the limits of the motor
            if (request.goal().position >= maxValue) {
                request.goal().position = maxValue;
                //resets the velocity to make sure a motion profile doesn't continue.
                request.goal().velocity = 0;
            }
            if (request.goal().position <= minValue) {
                request.goal().position = minValue;
                //resets the velocity to make sure a motion profile doesn't continue.
                request.goal().velocity = 0;
            }
        }

        // if not position control, that means the goal set is the direction of the motor,
        // then check if the measurement is in the limits of the motor
        // and make sure the direction is back to the zone
        else {
            //if below soft limit and moving backwards, set to zero
            if (measurement.position() <= minValue && request.goal().position < 0) {
                request.goal().position = 0;
            }
            //if above soft limit and moving forwards, set to zero
            if (measurement.position() >= maxValue && request.goal().position > 0) {
                request.goal().position = 0;
            }
        }
    }

    /**
     * Preforms the calculations of the constraints with the continuous mode.
     * This will wrap the goal of the controller request around the limits of the motor.
     * This mode supports only position control.
     *
     * @param measurement the measurement of the motor
     * @param request     the request of the controller
     */
    public void calculateContinuous(Measurements.Measurement measurement, Controller.ControllerRequest request) {
        // check if the request is a position control (continuous constraints only work for position
        // control)
        if (!request.controlMode().isPositionControl()) return;

        double errorBound = (maxValue - minValue) / 2.0;

        double originalPosition = request.goal().position;

        // wrap the goal around the limits
        request.goal().position =
                MathUtil.inputModulus(
                        request.goal().position - measurement.position(), -errorBound, errorBound)
                        + measurement.position();

        // if the goal is in the opposite direction of the original position, reverse the velocity
        // only affects motion profiles
        if (Math.signum(request.goal().position - measurement.position())
                != Math.signum(originalPosition - measurement.position())) {
            request.goal().velocity *= -1;
        }
    }

    /**
     * Clamps the given value to the range of the output limits of the motor.
     *
     * @param output The output of the motor (in volts).
     * @return The clamped output of the motor. (in volts)
     */
    public double clampMotorOutput(double output) {
        return MathUtil.clamp(output, minMotorOutput, maxMotorOutput);
    }

    /**
     * Applies the deadband to the motor output.
     * This checks if the absolute value of the output is below the deadband,
     * if it is, it returns 0.
     *
     * @param output The output of the motor (in volts).
     * @return The output of the motor after applying the deadband. (in volts)
     */
    public double deadbandMotorOutput(double output) {
        if (Math.abs(output) < voltageDeadband) {
            return 0;
        }
        return output;
    }

    /**
     * Checks the motor output by applying the deadband and clamping it to the output limits.
     *
     * @param output The output of the motor (in volts).
     * @return The corrected output of the motor after applying the deadband and clamping it to the output limits. (in volts)
     */
    public double checkMotorOutput(double output) {
        return deadbandMotorOutput(clampMotorOutput(output));
    }

    /**
     * Gets the type of constraints stored.
     *
     * @return The type of constraints stored.
     */
    public ConstraintType getConstraintType() {
        return constraintType;
    }

    /**
     * Gets the minimum value of the constraints.
     * Purpose depends on the type of constraints.
     *
     * @return The minimum value of the constraints.
     */
    public double getMinValue() {
        return minValue;
    }

    /**
     * Gets the maximum value of the constraints.
     *
     * @return The maximum value of the constraints.
     */
    public double getMaxValue() {
        return maxValue;
    }

    /**
     * Gets the maximum output of the motor.
     * This will always be positive or zero.
     *
     * @return The maximum output of the motor. (in volts)
     */
    public double getMaxMotorOutput() {
        return maxMotorOutput;
    }

    /**
     * Gets the minimum output of the motor.
     * This will always be negative or zero.
     *
     * @return the minimum output of the motor. (in volts)
     */
    public double getMinMotorOutput() {
        return minMotorOutput;
    }

    /**
     * Gets the minimum output voltage of the motor.
     * This will always be positive or zero.
     *
     * @return The minimum output voltage of the motor.
     */
    public double getVoltageDeadband() {
        return voltageDeadband;
    }

    /**
     * Gets the maximum rate the voltage can change per second.
     *
     * @return The maximum rate the voltage can change per second. (in volts per second)
     */
    public double getVoltageRampRate() {
        return voltageRampRate;
    }

    /**
     * Converts the constraints to motor constraints.
     * When setting the constraint to the motor controller, it needs to account for the gear ratio and unit conversion.
     *
     * @param gearRatio      The gear ratio of the motor.
     *                       This will be used to multiply the min and max values to convert them to motor units.
     * @param unitConversion The value that will be divided by the min and max values to convert them to the desired units.
     *                       This is used to convert the constraints to motor units.
     * @return The motor ready constraints.
     */
    public ConstraintsGains convertToMotorConstraints(double gearRatio, double unitConversion) {
        return new ConstraintsGains(
                constraintType,
                (minValue / unitConversion) * gearRatio,
                (maxValue / unitConversion) * gearRatio,
                maxMotorOutput,
                minMotorOutput,
                voltageDeadband,
                voltageRampRate);
    }
}
