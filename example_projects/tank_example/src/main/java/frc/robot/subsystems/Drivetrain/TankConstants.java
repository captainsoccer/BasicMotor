package frc.robot.subsystems.Drivetrain;

import com.basicMotor.BasicMotor;
import com.basicMotor.configuration.BasicMotorConfig;
import com.basicMotor.configuration.BasicSparkBaseConfig;
import com.basicMotor.configuration.BasicMotorConfig.FeedForwardConfig;
import com.basicMotor.configuration.BasicMotorConfig.PIDConfig;
import com.basicMotor.gains.ControllerFeedForwards;
import com.basicMotor.gains.PIDGains;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Constants for the tank drive system.
 */
public enum TankConstants {
    LEFT(3, 4, false,
            new PIDGains(3, 0, 0), //The PID gains for the left side motors
            new ControllerFeedForwards(4), //The feed forwards for the left side motors
            0.4), //The kA constant for the left side motors (kv is the setpoint feed forward)

    RIGHT(5, 6, false,
            new PIDGains(3, 0, 0), //The PID gains for the right side motors
            new ControllerFeedForwards(4), //The feed forwards for the right side motors
            0.4); //The kA constant for the right side motors (kv is the setpoint feed forward)

    /**
     * The radius of the wheels in meters.
     */
    public static final double WHEEL_RADIUS_METERS = 0.0762; // 3 inches in meters
    /**
     * The track width of the tank drive in meters.
     * This is the distance between the left and right wheels.
     */
    public static final double TRACK_WIDTH_METERS = 0.6096; // 24 inches in meters

    /**
     * The maximum velocity of the tank drive in meters per second.
     * This is the speed at which the tank drive can operate.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 3.0; // Maximum velocity of the tank drive in meters per second

    /**
     * The gear ratio of the motors.
     * This is the ratio of the motor rotation to the wheel rotation.
     */
    public static final double GEAR_RATIO = 6.1; // Example gear ratio for the motors
    /**
     * The current limit for the motors in Amperes.
     * This is the maximum current that the motors can output.
     */
    public static final int FREE_SPEED_CURRENT_LIMIT = 50; // Current limit for the motors in Amperes

    /**
     * The CAN ID for the gyro.
     * This is the ID used to communicate with the gyro over CAN bus.
     */
    public static final int GYRO_CAN_ID = 1; // The CAN ID for the gyro (e.g., Pigeon2)

    /**
     * Whether the follower motors are inverted relative to the lead motors.
     * This is used to determine the direction of the follower motors.
     */
    public static final boolean followerInvertedToLead = true; // Follower motors are inverted relative to lead motors

    /**
     * Creates a common motor configuration for both lead and follower motors.
     * This method sets the common properties for the motors such as gear ratio, unit conversion, idle mode, and motor type.
     * @return A BasicMotorConfig object containing the common motor configuration.
     */
    private static BasicMotorConfig createCommonMotorConfig(){
        var config = new BasicSparkBaseConfig();

        config.motorConfig.gearRatio = GEAR_RATIO;
        config.motorConfig.unitConversion = WHEEL_RADIUS_METERS * 2 * Math.PI; // Convert wheel radius to circumference
        config.motorConfig.idleMode = BasicMotor.IdleMode.BRAKE;
        config.motorConfig.motorType = DCMotor.getNEO(2); // Example motor type

        config.currentLimitConfig.freeSpeedCurrentLimit = FREE_SPEED_CURRENT_LIMIT;

        return config;
    }

    // PathPlanner constants

    /**
     * The fallback RobotConfig for the tank drive system.
     * This is used if the GUI configuration fails to load.
     */
    public static final RobotConfig ROBOT_CONFIG =
            new RobotConfig(
                    30,
                    6,
                    new ModuleConfig(
                            WHEEL_RADIUS_METERS,
                            MAX_VELOCITY_METERS_PER_SECOND,
                            1, // The coefficient of friction for the wheels
                            DCMotor.getNEO(2).withReduction(GEAR_RATIO),
                            FREE_SPEED_CURRENT_LIMIT,
                            2
                    ),
                    TRACK_WIDTH_METERS
            );

    /**
     * Should pathplanner flip the path.
     * Change this function to suit your needs.
     * @return true if the path should be flipped, false otherwise.
     */
    public static boolean shouldFlipPath() {
        var alliance = DriverStation.getAlliance();

        if(alliance.isPresent()){
            return alliance.get() == DriverStation.Alliance.Red;
        }

        return false; // Default to not flipping if alliance is not set
    }

    // No need to change anything below this line

    /**
     * The lead motor configuration for the tank drive system.
     */
    public final BasicMotorConfig leadMotorConfig;

    /**
     * The follower motor configuration for the tank drive system.
     * This is used to configure the follower motors that follow the lead motors.
     */
    public final BasicMotorConfig followerMotorConfig;

    /**
     * Constructor for the TankConstants enum.
     * @param leadID the canbus ID for the lead motor
     * @param followID the canbus ID for the follower motor
     * @param inverted whether the output of this side should be inverted
     * @param pidGains the PID gains for the motor (for velocity control)
     * @param feedForwards the feed forwards for the motor (for velocity control)
     * @param kA the kA constant for the motor (for simulation) (kv is the setpoint feed forward)
     */
    TankConstants(int leadID, int followID, boolean inverted, PIDGains pidGains, ControllerFeedForwards feedForwards, double kA){
        leadMotorConfig = createCommonMotorConfig();
        leadMotorConfig.motorConfig.id = leadID;
        leadMotorConfig.motorConfig.name = this.name() + " Lead Motor";
        leadMotorConfig.motorConfig.inverted = inverted;

        leadMotorConfig.pidConfig = PIDConfig.fromGains(pidGains);
        leadMotorConfig.feedForwardConfig = FeedForwardConfig.fromFeedForwards(feedForwards);

        leadMotorConfig.simulationConfig.kV = feedForwards.getSetpointFeedForward();
        leadMotorConfig.simulationConfig.kA = kA;

        followerMotorConfig = new BasicSparkBaseConfig();

        followerMotorConfig.motorConfig.id = followID;
        followerMotorConfig.motorConfig.name = this.name() + " Follower Motor";
    }
}
