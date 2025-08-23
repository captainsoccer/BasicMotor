package io.github.captainsoccer.basicmotor;

import io.github.captainsoccer.basicmotor.gains.*;
import io.github.captainsoccer.basicmotor.motorManager.MotorManagerConfig;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import java.util.function.Function;

/**
 * This class is used to store all relevant data to use the Basic Motor class.
 * Use this class when constructing a BasicMotor instance for a one-liner initialization.
 * also check out motor specific configurations to take advantage of motor specific features.
 * Refer to the <a href="https://github.com/captainsoccer/MotorUtils/wiki/Configuration">wiki</a>
 * for more information and examples.
 */
public class BasicMotorConfig {

    /**
     * The basic parameters of the motor controller
     * (id, name, gear ratio, unit conversion, inverted, idle mode, motor type)
     */
    public MotorConfig motorConfig = new MotorConfig();

    /**
     * The configuration for slot 0 of the motor controller.
     * (PID, feed forward, motion profile)
     */
    public SlotConfig slot0Config = new SlotConfig();

    /**
     * The configuration for slot 1 of the motor controller.
     * (PID, feed forward, motion profile)
     */
    public SlotConfig slot1Config = new SlotConfig();

    /**
     * The configuration for slot 2 of the motor controller.
     * (PID, feed forward, motion profile)
     */
    public SlotConfig slot2Config = new SlotConfig();

    /**
     * The Constraints configuration of the motor controller
     * (constraint type, min value, max value, max output, min output, voltage deadband)
     */
    public ConstraintsConfig constraintsConfig = new ConstraintsConfig();

    /**
     * The simulation configuration for the motor controller, used for simulating the motor in different scenarios
     * (kv, ka, moment of inertia, elevator simulation config, arm simulation config)
     */
    public SimulationConfig simulationConfig = new SimulationConfig();

    /**
     * Bunches up all the configurations into one class
     *
     * @return The controller gains which include PID gains, constraints, and feed forwards
     */
    public ControllerGains getControllerGains() {
        var slots = new SlotGains[]{slot0Config.getSlotGains(), slot1Config.getSlotGains(), slot2Config.getSlotGains()};

        return new ControllerGains(slots, constraintsConfig.getConstraints());
    }

    /**
     * If the motor controller is using an external encoder.
     * Used when initializing the motor controller.
     * When creating motor specific configurations, override this method to check if the motor is using an external
     * encoder connected directly to the motor controller
     *
     * @return True if the motor controller is using an external encoder, false otherwise
     */
    public boolean usingExternalEncoder() {
        return false;
    }

    /**
     * Creates a copy of the BasicMotorConfig instance.
     * This is useful when you want to create a new instance with the same values as the original.
     * The copy will have no references to the original instance, so changes to the copy will not affect the original.
     *
     * @param copy The BasicMotorConfig instance to copy values into.
     * @return A new instance of BasicMotorConfig with the same values as this instance with no references to the original instance.
     */
    public BasicMotorConfig copy(BasicMotorConfig copy) {

        copy.motorConfig = this.motorConfig.copy();
        copy.constraintsConfig = this.constraintsConfig.copy();
        copy.simulationConfig = this.simulationConfig.copy();

        copy.slot0Config = this.slot0Config.copy();
        copy.slot1Config = this.slot1Config.copy();
        copy.slot2Config = this.slot2Config.copy();

        return copy;
    }

    /**
     * Creates a copy of the BasicMotorConfig instance.
     * This is useful when you want to create a new instance with the same values as the original.
     * The copy will have no references to the original instance, so changes to the copy will not affect the original.
     *
     * @return A new instance of BasicMotorConfig with the same values as this instance with no references to the original instance.
     */
    public BasicMotorConfig copy() {
        return copy(new BasicMotorConfig());
    }


    /**
     * Handles the basic parameters of the motor controller.
     */
    public static class MotorConfig {
        /**
         * The CANBUS id of the motor controller
         *
         * <p>This is used in the constructor of the motor controller to identify it in the CANBUS chain.
         */
        public int id = 0;
        /**
         * The name of the motor controller
         *
         * <p>This is used to identify the motor controller in the logger, motor logs will be under: Motors/{name}/
         */
        public String name = "motor";
        /**
         * The gear ratio of the motor controller
         *
         * <p>This is used to convert the number of rotations the motor does to the number of rotations the mechanism does.
         * The different measurements (positon, velocity, acceleration) will be divided by this value
         * do not include unit conversion in this value, it is only the gear ratio (i.e reduction ratio)
         * if you want to convert the output to a different unit, use {@link #unitConversion}.
         * If using an external encoder, still set this value to the gear ratio of the motor,
         * this is important for correct conversions and torque calculations.
         */
        public double gearRatio = 1;

        /**
         * The value that will be multiplied by to convert the measurements to the desired units.
         * The default units of the motor are rotations.
         * (rotations, rotations per second, rotations per
         * second squared)
         *
         * <p>You can convert to the following units easily:
         *
         * <p>* - meters: unit conversion will be: 2π * radius (radius must be in meters)
         *
         * <p>* - degrees: unit conversion will be: 360 (1 rotation = 360 degrees)
         *
         * <p>* - radians: unit conversion will be: 2π (1 rotation = 2 * π radians)
         */
        public double unitConversion = 1;

        /**
         * If the motor direction is inverted.
         * <p>If true, positive output will result in clockwise rotation
         * <p>If false, positive output will result in counter-clockwise rotation
         */
        public boolean inverted = false;

        /**
         * The idle mode of the motor controller COAST means the motor will not try to hold its position
         * when not powered BRAKE means the motor will try to hold its position when not powered
         */
        public BasicMotor.IdleMode idleMode = BasicMotor.IdleMode.COAST;

        /**
         * The type of motor that the controller is connected to.
         * This is used to calculate the motor
         * torque.
         * Also used for simulation purposes.
         * If another motor is following this motor, include the total number of motors.
         */
        public DCMotor motorType = DCMotor.getNEO(1);

        /**
         * Copies the motor configuration into a new instance.
         *
         * @return A new instance of MotorConfig with the same values as this instance with no references to the original instance.
         */
        public MotorConfig copy() {
            var copy = new MotorConfig();
            copy.id = this.id;
            copy.name = this.name;
            copy.gearRatio = this.gearRatio;
            copy.unitConversion = this.unitConversion;
            copy.inverted = this.inverted;
            copy.idleMode = this.idleMode;
            copy.motorType = this.motorType;

            return copy;
        }
    }

    /**
     * The configuration for a specific slot of the motor controller.
     * This is used to store different configurations for different control modes.
     */
    public static class SlotConfig {
        /**
         * The PID configuration of the motor controller
         */
        public PIDConfig pidConfig = new PIDConfig();

        /**
         * The feed forward configuration of the motor controller
         */
        public FeedForwardConfig feedForwardConfig = new FeedForwardConfig();

        /**
         * The constraints configuration of the motor controller
         */
        public ProfileConfig profileConfig = new ProfileConfig();

        /**
         * Creates a copy of the SlotConfig instance.
         * This is useful when you want to create a new instance with the same values as the original.
         *
         * @return A new instance of SlotConfig with the same values as this instance with no references to the original instance.
         */
        public SlotConfig copy() {
            var copy = new SlotConfig();

            copy.pidConfig = this.pidConfig.copy();
            copy.feedForwardConfig = this.feedForwardConfig.copy();
            copy.profileConfig = this.profileConfig.copy();

            return copy;
        }

        /**
         * Bunches up all the configurations into one class
         */
        public SlotGains getSlotGains() {
            return new SlotGains(
                    pidConfig.getGains(),
                    feedForwardConfig.getFeedForwards(),
                    profileConfig.getProfileConstraints());
        }

        /**
         * Creates a SlotConfig from the given SlotGains object.
         *
         * @param slotGains The SlotGains object to convert into a SlotConfig.
         * @return The SlotConfig instance with the values from the SlotGains object.
         */
        public static SlotConfig fromSlotGains(SlotGains slotGains) {
            var config = new SlotConfig();

            config.pidConfig = PIDConfig.fromGains(slotGains.getPIDGains());
            config.feedForwardConfig = FeedForwardConfig.fromFeedForwards(slotGains.getFeedForwardsGains());
            config.profileConfig = ProfileConfig.fromProfileConstraints(slotGains.getMotionProfileGains());

            return config;
        }
    }

    /**
     * Handles the PID configuration of the motor controller
     */
    public static class PIDConfig {
        /**
         * The kp gain of the PID controller units are: (voltage / unit of control) <p>The value must be greater then zero
         */
        public double kP = 0;
        /**
         * The ki gain of the PID controller units are: (voltage * second / unit of control)  <p>The value must be greater then zero
         */
        public double kI = 0;
        /**
         * The kd gain of the PID controller units are: (voltage / unit of control * second) <p>The value must be greater then zero
         */
        public double kD = 0;
        /**
         * The iZone of the PID controller units are: (unit of control)
         *
         * <p>The value must be greater than or equal to zero (zero means no integral term)
         * <p>If the error is within this zone, the integral will accumulate, when the error is outside it will be zeroed
         */
        public double iZone = Double.POSITIVE_INFINITY;
        /**
         * The maximum accumulation and contribution of the integral term units are: (voltage)
         *
         * <p>The value must be greater than or equal to zero (zero means no integral term)
         * <p>This is the maximum value that the integral term can accumulate to, if it exceeds this
         * value, it will be clamped.
         * The default value is 13, but can be changed to any value.
         * Also, the default value can be changed in the #{@link MotorManagerConfig}
         */
        public double iMaxAccum = MotorManager.config.defaultMaxMotorOutput;
        /**
         * The tolerance of the PID controller units are: (unit of control)
         *
         * <p>The value must be greater than or equal to zero (zero means pid will never consider itself at setpoint)
         * <p>This is the tolerance of the PID controller, if the error is within this tolerance, the
         * PID controller will consider itself at setpoint and will not apply any output.
         */
        public double tolerance = 0;

        /**
         * Gets the PID gains of the controller
         *
         * @return The PID gains of the controller
         */
        public PIDGains getGains() {
            return new PIDGains(kP, kI, kD, iZone, iMaxAccum, tolerance);
        }

        /**
         * Creates a copy of the PIDConfig instance.
         * This is useful when you want to create a new instance with the same values as the original.
         *
         * @return A new instance of PIDConfig with the same values as this instance with no references to the original instance.
         */
        public PIDConfig copy() {
            var copy = new PIDConfig();
            copy.kP = this.kP;
            copy.kI = this.kI;
            copy.kD = this.kD;
            copy.iZone = this.iZone;
            copy.iMaxAccum = this.iMaxAccum;
            copy.tolerance = this.tolerance;

            return copy;
        }

        /**
         * Creates a PID configuration from the given PIDGains object.
         *
         * @param gains The PIDGains object to convert into a PIDConfig.
         * @return The PIDConfig instance with the values from the PIDGains object.
         */
        public static PIDConfig fromGains(PIDGains gains) {
            var config = new PIDConfig();

            config.kP = gains.getK_P();
            config.kI = gains.getK_I();
            config.kD = gains.getK_D();
            config.iZone = gains.getI_Zone();
            config.iMaxAccum = gains.getI_MaxAccum();
            config.tolerance = gains.getTolerance();

            return config;
        }
    }

    /**
     * Handles the feed forward configuration of the motor controller.
     * Feed forwards are always calculated on the RIO, this is to ensure reliability and repeatability.
     */
    public static class FeedForwardConfig {
        /**
         * The friction feed forward gain of the motor controller units are: (voltage)
         *
         * <p>The value must be greater than or equal to zero (zero means no friction feed forward)
         * <p>This is used to compensate for the friction in the motor, it will be applied to the output
         * in the direction of travel.
         */
        public double frictionFeedForward = 0;
        /**
         * The setpoint feed forward gain of the motor controller units are: (voltage / unit of control).
         *
         * <p>The value must be greater than or equal to zero (zero means no setpoint feed forward)
         * <p>This is usually used in velocity control, but it is just a feed forward gain that is
         * multiplied by the setpoint
         */
        public double setpointFeedForward = 0;
        /**
         * The simple feed forward gain of the motor controller units are: (voltage)
         *
         * <p>The value must be greater than or equal to zero (zero means no simple feed forward)
         * <p>It's just a constant voltage applied to the output used, for example, in elevators
         */
        public double simpleFeedForward = 0;
        /**
         * A custom feed forward function that takes the setpoint and returns a value.
         * It takes a setpoint in units of control and returns a value in volts.
         *
         * <p>The function must be O(1).
         * <p>This can be used for more complex feed forwards, like an arm with gravity compensation
         */
        public Function<Double, Double> customFeedForward = (setpoint) -> 0.0;

        /**
         * Gets the feed forwards of the controller
         *
         * @return The feed forwards of the controller
         */
        public FeedForwardsGains getFeedForwards() {
            return new FeedForwardsGains(
                    simpleFeedForward, frictionFeedForward, setpointFeedForward, customFeedForward);
        }

        /**
         * Creates a copy of the FeedForwardConfig instance.
         * This is useful when you want to create a new instance with the same values as the original.
         * The only reference that is copied is the custom feed forward function,
         * which will point to the same function as the original.
         *
         * @return A new instance of FeedForwardConfig with the same values as this instance.
         */
        public FeedForwardConfig copy() {
            var copy = new FeedForwardConfig();

            copy.simpleFeedForward = this.simpleFeedForward;
            copy.frictionFeedForward = this.frictionFeedForward;
            copy.setpointFeedForward = this.setpointFeedForward;
            copy.customFeedForward = this.customFeedForward;

            return copy;
        }

        /**
         * Creates a feed forward configuration from the given ControllerFeedForwards object.
         *
         * @param feedForwards The ControllerFeedForwards object to convert into a FeedForwardConfig.
         * @return The FeedForwardConfig instance with the values from the ControllerFeedForwards object.
         */
        public static FeedForwardConfig fromFeedForwards(FeedForwardsGains feedForwards) {
            var config = new FeedForwardConfig();

            config.simpleFeedForward = feedForwards.getSimpleFeedForward();
            config.frictionFeedForward = feedForwards.getFrictionFeedForward();
            config.setpointFeedForward = feedForwards.getSetpointFeedForward();
            config.customFeedForward = feedForwards.getFeedForwardFunction();

            return config;
        }
    }

    /**
     * Handles the constraints configuration of the motor controller
     */
    public static class ConstraintsConfig {
        /**
         * The type of constraint to apply to the controller
         *
         * <p>This can be NONE, LIMITED, or CONTINUOUS
         *
         * <p>* NONE means no constraints are applied
         *
         * <p>* LIMITED means there are soft limits in the min value and max value.
         * For example, an arm with a limited range of motion between 0 and 90 degrees.
         * Or an elevator with a base location of 0 meters and a maximum height of 2 meters.
         *
         * <p>* CONTINUOUS means the position is continuous.
         * Which means the number of rotations doesn't matter.
         * This effects only position control, where it will find the shortest path to the setpoint.
         * The most common use case is a swerve module steering motor.
         */
        public ConstraintsGains.ConstraintType constraintType =
                ConstraintsGains.ConstraintType.NONE;

        /**
         * The maximum value of the constraint units are: (unit of position (default is rotations))
         *
         * <p>This represents the maximum value of the constraint:
         *
         * <p>* For a limited constraint, this is the maximum value the motor position can reach.
         *
         * <p>* For a continuous constraint, this is the maximum value the position can reach before
         * wrapping around (for example, 1 rotation, 0.5 rotations, etc.).
         */
        public double maxValue = 0;

        /**
         * The minimum value of the constraint units are: (unit of position (default is rotations))
         *
         * <p>This represents the minimum value of the constraint:
         *
         * <p>* For a limited constraint, this is the minimum value the motor position can reach.
         *
         * <p>* For a continuous constraint, this is the minimum value the position can reach before
         * wrapping around (for example, 0 rotations, -0.5 rotations, etc.).
         */
        public double minValue = 0;

        /**
         * The maximum output of the constraint units are: (voltage)
         *
         * <p>This is the maximum output of the motor controller in the forward direction
         * This is used to save power, relax the mechanism, and prevent damage to the motor.
         */
        public double maxOutput = MotorManager.config.defaultMaxMotorOutput;

        /**
         * The minimum output of the constraint units are: (voltage)
         * The value is automatically set to a negative value, so no need to think of it.
         * This value will usually be the negative of the {@link #maxOutput} value.
         *
         * <p>This is the minimum output of the motor controller in the reverse direction.
         * This is used to save power, relax the mechanism, and prevent damage to the motor.
         */
        public double minOutput = -MotorManager.config.defaultMaxMotorOutput;
        /**
         * The voltage deadband of the constraint units are: (voltage)
         *
         * <p>This is the minimum voltage that the motor controller will apply to the motor,
         * any absolute value below this will be ignored.
         * And the motor will not apply any output.
         */
        public double voltageDeadband = 0;

        /**
         * Gets the constraints of the controller
         *
         * @return The constraints of the controller
         */
        public ConstraintsGains getConstraints() {
            return new ConstraintsGains(
                    constraintType, minValue, maxValue, maxOutput, minOutput, voltageDeadband);
        }

        /**
         * Creates a copy of the ConstraintsConfig instance.
         * This is useful when you want to create a new instance with the same values as the original.
         *
         * @return A new instance of ConstraintsConfig with the same values as this instance with no references to the original instance.
         */
        public ConstraintsConfig copy() {
            var copy = new ConstraintsConfig();

            copy.constraintType = this.constraintType;
            copy.maxValue = this.maxValue;
            copy.minValue = this.minValue;
            copy.maxOutput = this.maxOutput;
            copy.minOutput = this.minOutput;
            copy.voltageDeadband = this.voltageDeadband;

            return copy;
        }

        /**
         * Creates a ConstraintsConfig from the given ControllerConstraints object.
         *
         * @param constraints The ControllerConstraints object to convert into a ConstraintsConfig.
         * @return The ConstraintsConfig instance with the values from the ControllerConstraints object.
         */
        public static ConstraintsConfig fromConstraints(ConstraintsGains constraints) {
            var config = new ConstraintsConfig();

            config.constraintType = constraints.getConstraintType();
            config.maxValue = constraints.getMaxValue();
            config.minValue = constraints.getMinValue();
            config.maxOutput = constraints.getMaxMotorOutput();
            config.minOutput = constraints.getMinMotorOutput();
            config.voltageDeadband = constraints.getVoltageDeadband();

            return config;
        }
    }

    /**
     * The profile configuration of the motor controller used for motion profiling and trajectory
     * generation.
     * For motion profiling to work, both of the maximum velocity and maximum acceleration must be set to a finite value.
     * Default values are set to {@link Double#POSITIVE_INFINITY} which means no motion profiling will be applied.
     */
    public static class ProfileConfig {
        /**
         * The maximum velocity of the motor controller units are: (unit of control per second).
         * This is used to create a motion profile for the motor controller.
         * This setting applies only if using a profiled control mode.
         * <p>If using a profiled position control, this is the maximum velocity of the motor.
         * <p>If using a profiled velocity control, this is the maximum acceleration of the motor.
         */
        public double maximumMeasurementVelocity = Double.POSITIVE_INFINITY;

        /**
         * The maximum acceleration of the motor controller units are: (unit of control per second squared).
         * This is used to create a motion profile for the motor controller.
         * This setting applies only if using a profiled control mode.
         * <p>If using a profiled position control, this is the maximum acceleration of the motor.
         * <p>If using a profiled velocity control, this is the maximum jerk of the motor.
         */
        public double maximumMeasurementAcceleration = Double.POSITIVE_INFINITY;

        /**
         * Gets the profile constraints of the controller
         *
         * @return The profile constraints of the controller
         */
        public TrapezoidProfile.Constraints getProfileConstraints() {
            return new TrapezoidProfile.Constraints(
                    maximumMeasurementVelocity, maximumMeasurementAcceleration);
        }

        /**
         * Creates a copy of the ProfileConfig instance.
         * This is useful when you want to create a new instance with the same values as the original.
         *
         * @return A new instance of ProfileConfig with the same values as this instance with no references to the original instance.
         */
        public ProfileConfig copy() {

            var copy = new ProfileConfig();
            copy.maximumMeasurementVelocity = this.maximumMeasurementVelocity;
            copy.maximumMeasurementAcceleration = this.maximumMeasurementAcceleration;

            return copy;
        }

        /**
         * Creates a ProfileConfig from the given TrapezoidProfile.Constraints object.
         *
         * @param constraints The TrapezoidProfile.Constraints object to convert into a ProfileConfig.
         * @return The ProfileConfig instance with the values from the TrapezoidProfile.Constraints object.
         */
        public static ProfileConfig fromProfileConstraints(TrapezoidProfile.Constraints constraints) {
            var config = new ProfileConfig();

            config.maximumMeasurementVelocity = constraints.maxVelocity;
            config.maximumMeasurementAcceleration = constraints.maxAcceleration;

            return config;
        }
    }

    /**
     * The simulation configuration of the motor controller.
     * Used when you want to simulate the system the motor is controlling.
     * To learn more about the simulation configuration, refer to the wiki:
     * <a href="https://github.com/captainsoccer/MotorUtils/wiki">wiki</a> //TODO: add wiki link
     */
    public static class SimulationConfig {
        /**
         * The kV gain of the motor controller units are: (voltage / unit of velocity).
         * Use this and {@link #kA} to simulate the motor controller or use {@link #momentOfInertia}.
         * The kv must be in the mechanism rotations, not the motor rotations.
         * (i.e. after gear ratio).
         * The kv must be in SI units.
         * <p>For general use (angular mechanisms), it will be in (voltage / radians per second).
         * <p>For elevators, telescopic arms, and other linear mechanisms, it is in (voltage / meters per second).
         */
        public double kV = 0;

        /**
         * The kA gain of the motor controller units are: (voltage / unit of acceleration).
         * use this and {@link #kV} to simulate the motor controller or use {@link #momentOfInertia}.
         * The ka must be in the mechanism rotations, not the motor rotations.
         * (i.e. after gear ratio).
         * The ka must be in SI units.
         * For general use (angular mechanisms), it will be in (voltage / radians per second squared).
         * For elevators, telescopic arms, and other linear mechanisms, it will be in (voltage / meters per second squared).
         */
        public double kA = 0;

        /**
         * The moment of inertia of the motor controller units are: (kilogram * meter squared).
         * This is used to simulate the motor inertia in the simulation.
         * To simulate a mechanism, you must either use the kV and kA values or set the moment of inertia.
         * For a linear mechanism, the moment of inertia is the moment of inertia of the pulley or pinion.
         * See more about the moment of inertia on wikipedia: <a href="https://en.wikipedia.org/wiki/Moment_of_inertia">Moment of inertia</a>
         */
        public double momentOfInertia = 0.0;

        /**
         * The standard deviation of the position measurement in the simulation this is used to simulate
         * the noise in the position measurement units are: (unit of position)
         */
        public double positionStandardDeviation = 0.0;

        /**
         * The standard deviation of the velocity measurement in the simulation this is used to simulate
         * the noise in the velocity measurement units are: (unit of velocity)
         */
        public double velocityStandardDeviation = 0.0;

        /**
         * The elevator simulation configuration used if the motor is an elevator or a similar mechanism.
         * This has specific parameters for simulating an elevator system.
         */
        public ElevatorSimConfig elevatorSimConfig = new ElevatorSimConfig();

        /**
         * The arm simulation configuration used if the motor is an arm or a similar mechanism.
         * The arm simulation is for a single jointed arm where the 0 angle is parallel to the ground.
         * This has specific parameters for simulating an arm system.
         */
        public ArmSimConfig armSimConfig = new ArmSimConfig();

        /**
         * Creates a copy of the SimulationConfig instance.
         * This is useful when you want to create a new instance with the same values as the original.
         *
         * @return A new instance of SimulationConfig with the same values as this instance with no references to the original instance.
         */
        public SimulationConfig copy() {
            var copy = new SimulationConfig();

            copy.kV = this.kV;
            copy.kA = this.kA;
            copy.momentOfInertia = this.momentOfInertia;
            copy.positionStandardDeviation = this.positionStandardDeviation;
            copy.velocityStandardDeviation = this.velocityStandardDeviation;

            copy.elevatorSimConfig = this.elevatorSimConfig.copy();
            copy.armSimConfig = this.armSimConfig.copy();

            return copy;
        }

        /**
         * Handles elevator simulation parameters, used only when simulating an elevator system
         */
        public static class ElevatorSimConfig {
            /**
             * If the elevator simulation should enable gravity simulation if true, there will be a force
             * acting on the elevator due to gravity if false, the elevator will not be affected by
             * gravity
             */
            public boolean enableGravitySimulation = true;

            /**
             * The mass of the elevator in kilograms.
             * This includes all the stages of the elevator and the load it carries.
             * The mass is used only if the kV and kA are not provided.
             * If using a cascade elevator, the carriage mass (the part which is not connected to the motor directly) should be doubled.
             * This is due to the marriage moving twice the distance of the motor.
             * (includes both the portion directly driven by the motor and any parts moved as a result of the mechanism)
             */
            public double massKG = 0;

            /**
             * The radius of the pulley in meters.
             * This is used to calculate the distance the elevator moves based on the motor rotations.
             * The pully radius is not used if the kV and kA are provided.
             */
            public double pulleyRadiusMeters = 0;

            /**
             * Creates a copy of the ElevatorSimConfig instance.
             * This is useful when you want to create a new instance with the same values as the original.
             *
             * @return A new instance of ElevatorSimConfig with the same values as this instance with no references to the original instance.
             */
            public ElevatorSimConfig copy() {
                var copy = new ElevatorSimConfig();

                copy.enableGravitySimulation = this.enableGravitySimulation;
                copy.massKG = this.massKG;
                copy.pulleyRadiusMeters = this.pulleyRadiusMeters;

                return copy;
            }
        }

        /**
         * Handles the arm simulation parameters, used only when simulating an arm system.
         */
        public static class ArmSimConfig {
            /**
             * The length of the arm in meters taken from the pivot point to the end of the arm
             */
            public double armlengthMeters = 0.0;

            /**
             * Should the arm simulate gravity?
             *
             * <p>If true, there will be a force acting on the arm due to gravity
             *
             * <p>If false, the arm will not be affected by gravity
             */
            public boolean simulateGravity = true;

            /**
             * The starting angle of the arm in rotations in simulation.
             * The 0 angle is parallel to the ground (full force of gravity is applied in 0 angle).
             */
            public double startingAngle = 0.0;

            /**
             * Creates a copy of the ArmSimConfig instance.
             * This is useful when you want to create a new instance with the same values as the original.
             *
             * @return A new instance of ArmSimConfig with the same values as this instance with no references to the original instance.
             */
            public ArmSimConfig copy() {
                var copy = new ArmSimConfig();

                copy.armlengthMeters = this.armlengthMeters;
                copy.simulateGravity = this.simulateGravity;
                copy.startingAngle = this.startingAngle;

                return copy;
            }
        }
    }
}
