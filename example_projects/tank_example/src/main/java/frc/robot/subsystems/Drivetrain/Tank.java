package frc.robot.subsystems.Drivetrain;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.GyroIO;
import frc.Util.Pigeon2IO;
import frc.Util.SimGyroIO;
import org.littletonrobotics.junction.Logger;
import java.util.function.BiConsumer;

/**
 * The subsystem for the tank drive system.
 */
public class Tank extends SubsystemBase {
    /**
     * The IO interface for the tank drive system.
     * This interface is responsible for communicating with the hardware or simulation.
     */
    private final TankIO io;

    /**
     * The inputs for the tank drive system.
     * This class is logged by the logger.
     * When running replay mode, it will inject the inputs from the log.
     */
    private final TankInputsAutoLogged inputs = new TankInputsAutoLogged();

    /**
     * The gyro interface for the tank drive system.
     */
    private final GyroIO gyro;

    /**
     * The kinematics for the tank drive system.
     * This is used to convert between wheel speeds and chassis speeds.
     */
    private final DifferentialDriveKinematics kinematics;

    /**
     * The pose estimator for the tank drive system.
     * This is used to estimate the pose of the robot using the gyro and wheel encoders.
     * Also vision measurements if available.
     */
    private final DifferentialDrivePoseEstimator poseEstimator;

    /**
     * Constructor for the Tank subsystem.
     * Initializes the IO, gyro, kinematics, and pose estimator.
     */
    public Tank() {
        io = new TankIOBasic();

        gyro = RobotBase.isReal() ? new Pigeon2IO(TankConstants.GYRO_CAN_ID) : new SimGyroIO(this::getChassisSpeeds);

        kinematics = new DifferentialDriveKinematics(TankConstants.TRACK_WIDTH_METERS);

        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, Rotation2d.kZero, 0, 0, Pose2d.kZero);

        configureAutoBuilder();
    }

    /**
     * Configures the AutoBuilder for the tank drive system.
     * This method sets up the AutoBuilder with the necessary parameters
     */
    private void configureAutoBuilder() {
        BiConsumer<ChassisSpeeds, DriveFeedforwards> output =
                (chassisSpeeds, driveFeedforwards) -> runVelocity(chassisSpeeds);

        RobotConfig config;

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError("Unable to load config! error: " + e.getMessage(), true);
            config = TankConstants.ROBOT_CONFIG; // Fallback to default config if GUI settings fail
        }

        AutoBuilder.configure(
                this::getPose,
                this::reset,
                this::getChassisSpeeds,
                output,
                new PPLTVController(0.02, TankConstants.MAX_VELOCITY_METERS_PER_SECOND),
                config,
                TankConstants::shouldFlipPath
        );
    }

    /**
     * Resets the pose estimator to a new pose.
     * This method sets the gyro angle to the new pose's rotation
     * @param newPose The new pose to reset the estimator to.
     */
    public void reset(Pose2d newPose) {
        gyro.setAngle(newPose.getRotation());

        poseEstimator.resetPosition(newPose.getRotation(), inputs.wheelPositions, newPose);
    }

    /**
     * Resets the gyro angle to a new angle.
     * @param newAngle The new angle to reset the gyro to.
     */
    public void resetAngle(Rotation2d newAngle) {
        gyro.setAngle(newAngle);

        var newPose = new Pose2d(getPose().getX(), getPose().getY(), newAngle);

        poseEstimator.resetPosition(newAngle, inputs.wheelPositions, newPose);
    }

    /**
     * Gets the angle of the gyro.
     *
     * @return The current angle of the gyro as a Rotation object.
     */
    public Rotation2d getGyroAngle() {
        return inputs.gyroAngle;
    }

    /**
     * Gets the angle of the robot's pose.
     * This is more accurate than the gyro angle in most cases,
     * But it can have jumps in the angle or other artifacts due to vision measurements.
     *
     * @return The current angle of the robot's pose as a Rotation object.
     */
    public Rotation2d getAngle() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    /**
     * Gets the current pose of the robot.
     * This pose is estimated using the gyro and wheel encoders.
     * And vision measurements if available.
     *
     * @return The current pose of the robot as a Pose2d object.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the chassis speeds of the tank drive system.
     *
     * @return The current chassis speeds of the tank drive system.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(inputs.wheelSpeeds);
    }

    /**
     * Sets a new velocity target for the tank drive system.
     * This method converts the provided chassis speeds into wheel speeds
     * The chassis speeds need to be robot relative.
     * Because it's a tank drive, the vY is ignored.
     *
     * @param chassisSpeeds The desired chassis speeds.
     * @param dt            How much time does this target apply to the drive (how much time until the next command).
     */
    public void runVelocity(ChassisSpeeds chassisSpeeds, double dt) {
        var discretizedSpeeds = ChassisSpeeds.discretize(chassisSpeeds, dt);

        var targetWheelSpeeds = kinematics.toWheelSpeeds(discretizedSpeeds);

        checkWheelSpeeds(targetWheelSpeeds, TankConstants.MAX_VELOCITY_METERS_PER_SECOND);

        io.setTargetSpeeds(targetWheelSpeeds);

        Logger.recordOutput("Tank/ChassisSpeeds", discretizedSpeeds);
    }

    /**
     * Sets a new velocity target for the tank drive system.
     * This method converts the provided chassis speeds into wheel speeds
     * The chassis speeds need to be robot relative.
     * Because it's a tank drive, the vY is ignored.
     *
     * @param chassisSpeeds The desired chassis speeds.
     */
    public void runVelocity(ChassisSpeeds chassisSpeeds) {
        runVelocity(chassisSpeeds, 0.02); // Default dt is 20ms
    }

    /**
     * Sets the voltage for the left and right motors of the tank drive system.
     * @param leftVoltage The voltage to apply to the left motors.
     * @param rightVoltage The voltage to apply to the right motors.
     */
    public void runVoltage(double leftVoltage, double rightVoltage) {
        io.setVoltage(leftVoltage, rightVoltage);
    }

    /**
     * This function checks if the target wheel speeds exceed the maximum velocity.
     * If they do, it reduces the speeds to the maximum allowed velocity while maintaining the
     * ratio between the left and right wheel speeds.
     *
     * @param wheelSpeeds The target wheel speeds to check and potentially adjust.
     * @param maxVelocity The maximum allowed velocity for the wheels in meters per second.
     */
    private static void checkWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds, double maxVelocity) {
        double delta;

        if (Math.abs(wheelSpeeds.leftMetersPerSecond) > maxVelocity) {
            delta = Math.abs(wheelSpeeds.leftMetersPerSecond) - maxVelocity;
        } else if (Math.abs(wheelSpeeds.rightMetersPerSecond) > maxVelocity) {
            delta = Math.abs(wheelSpeeds.rightMetersPerSecond) - maxVelocity;
        } else delta = 0;

        wheelSpeeds.leftMetersPerSecond -= Math.copySign(delta, wheelSpeeds.leftMetersPerSecond);
        wheelSpeeds.rightMetersPerSecond -= Math.copySign(delta, wheelSpeeds.rightMetersPerSecond);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        inputs.gyroAngle = gyro.update();
        inputs.chassisSpeeds = getChassisSpeeds();

        Logger.processInputs("Tank", inputs);

        poseEstimator.update(getGyroAngle(), inputs.wheelPositions);

        Logger.recordOutput("Tank/EstimatedPose", poseEstimator.getEstimatedPosition());
        Logger.recordOutput("Tank/ChassisSpeeds", getChassisSpeeds());
    }
}

