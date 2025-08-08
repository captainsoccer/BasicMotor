package com.basicMotor.configuration;

import com.basicMotor.BasicMotor;
import com.basicMotor.motorManager.MotorManagerConfig;
import com.basicMotor.gains.ControllerConstraints;
import com.basicMotor.gains.ControllerFeedForwards;
import com.basicMotor.gains.ControllerGains;
import com.basicMotor.gains.PIDGains;
import com.basicMotor.motorManager.MotorManager;
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
   * The pid configuration for the motor controller (kP, kI, kD, iZone, iMaxAccum, tolerance)
   */
  public PIDConfig pidConfig = new PIDConfig();
  /**
   * The feed forward configuration for the motor controller
   * (simple feed forward, friction feed forward, setpoint feed forward, feed forward function)
   */
  public FeedForwardConfig feedForwardConfig = new FeedForwardConfig();
  /**
   * The constraints configuration for the motor controller
   * (type of constraint (none, limited, continuous),
   * max value of the constraint, min value of the constraint, max output, min output, voltage deadband)
   */
  public ConstraintsConfig constraintsConfig = new ConstraintsConfig();

  /**
   * The profile configuration for the motor controller, used for motion profiling
   * (max control velocity, max control acceleration)
   */
  public ProfileConfig profileConfig = new ProfileConfig();

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
    return new ControllerGains(
        pidConfig.getGains(),
        constraintsConfig.getConstraints(),
        feedForwardConfig.getFeedForwards(),
        profileConfig.getProfileConstraints());
  }

  /**
   * If the motor controller is using an external encoder.
   * Used when initializing the motor controller.
   * When creating motor specific configurations, override this method to check if the motor is using an external
   * encoder connected directly to the motor controller
   * @return True if the motor controller is using an external encoder, false otherwise
   */
  public boolean usingExternalEncoder() {
    return false;
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
  }

  /**
   * Handles the PID configuration of the motor controller
   */
  public static class PIDConfig {
    /** The kp gain of the PID controller units are: (voltage / unit of control) <p>The value must be greater then zero */
    public double kP = 0;
    /** The ki gain of the PID controller units are: (voltage * second / unit of control)  <p>The value must be greater then zero*/
    public double kI = 0;
    /** The kd gain of the PID controller units are: (voltage / unit of control * second) <p>The value must be greater then zero */
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
    public ControllerFeedForwards getFeedForwards() {
      return new ControllerFeedForwards(
          simpleFeedForward, frictionFeedForward, setpointFeedForward, customFeedForward);
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
     *
     */
    public ControllerConstraints.ConstraintType constraintType =
        ControllerConstraints.ConstraintType.NONE;

    /**
     * The maximum value of the constraint units are: (unit of position (default is rotations))
     *
     * <p>This represents the maximum value of the constraint:
     *
     * <p>* For a limited constraint, this is the maximum value the motor position can reach.
     *
     * <p>* For a continuous constraint, this is the maximum value the position can reach before
     * wrapping around (for example, 1 rotation, 0.5 rotations, etc.).
     *
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
     *
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
    public ControllerConstraints getConstraints() {
      return new ControllerConstraints(
          constraintType, minValue, maxValue, maxOutput, minOutput, voltageDeadband);
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
    public final ElevatorSimConfig elevatorSimConfig = new ElevatorSimConfig();

    /**
     * The arm simulation configuration used if the motor is an arm or a similar mechanism.
     * The arm simulation is for a single jointed arm where the 0 angle is parallel to the ground.
     * This has specific parameters for simulating an arm system.
     */
    public final ArmSimConfig armSimConfig = new ArmSimConfig();

    /** Handles elevator simulation parameters, used only when simulating an elevator system */
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
    }

    /**
     * Handles the arm simulation parameters, used only when simulating an arm system.
     */
    public static class ArmSimConfig {
      /** The length of the arm in meters taken from the pivot point to the end of the arm */
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
    }
  }
}
