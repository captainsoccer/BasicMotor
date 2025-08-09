// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.io.IOException;
import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj.RobotBase;
import frc.Util.SimGyroIO;
import frc.robot.subsystems.drivetrain.swerveModule.*;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.GyroIO;
import frc.Util.Pigeon2IO;

/**
 * This is the subsystem for the drive train of the robot.
 * It handles the swerve modules and the pose estimation.
 * Includes pathplanner auto support.
 */
public class DriveTrain extends SubsystemBase {
    @AutoLog
    public static class DriveTrainInputs {
        /**
         * Holds the current states of the modules
         * (Angle and velocity)
         */
        public SwerveModuleState[] currentStates = new SwerveModuleState[4];
        /**
         * Holds the angle of the gyro.
         * Counterclockwise positive.
         */
        public Rotation2d angle = new Rotation2d();
    }

    /**
     * This holds the inputs for the driveBase.
     * The values here are readings from the sensors without any proxying.
     */
    private final DriveTrainInputsAutoLogged inputs = new DriveTrainInputsAutoLogged();

    /**
     * This is the interface used to communicate with the modules.
     */
    private final SwerveModuleIO[] io = new SwerveModuleIO[4];
    /**
     * This is the inputs for the modules.
     */
    private final SwerveModuleInputsAutoLogged[] moduleInputs = new SwerveModuleInputsAutoLogged[4];

    /**
     * Holds the modules position, this is used for pose estimation.
     */
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    /**
     * The kinematics object used for calculating chassis speeds to modules states and back
     */
    private final SwerveDriveKinematics kinematics;
    /**
     * The pose estimator that uses the modules position and external vision measurement for pose estimation
     */
    private final SwerveDrivePoseEstimator poseEstimator;

    /**
     * The Gyro used for angle.
     */
    private final GyroIO gyro;

    /**
     * Creates a new DriveTrain.
     */
    public DriveTrain() {
        for (int i = 0; i < 4; i++) {
            if(RobotBase.isReal()){
                io[i] = new SwerveModuleReal(SwerveModuleConstants.values()[i]);
            }
            else{
                io[i] = new SwerveModuleSim(SwerveModuleConstants.values()[i]);
            }

            modulePositions[i] = new SwerveModulePosition();

            moduleInputs[i] = new SwerveModuleInputsAutoLogged();
        }

        if(RobotBase.isReal()){
            gyro = new Pigeon2IO(DriveTrainConstants.pigeonID);
        } else {
            gyro = new SimGyroIO(this::getChassisSpeeds); // Use a simulated gyro in simulation
        }

        kinematics = new SwerveDriveKinematics(SwerveModuleConstants.getTranslations());

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero, modulePositions, new Pose2d());

        configureAutoBuilder();
    }

    /**
     * Configures the Pathplanner Auto builder.
     * This takes the configuration set by the GUI.
     * If no configuration is available, uses the backup configuration stored in {@link DriveTrainConstants#FALLBACK_CONFIG}.
     */
    private void configureAutoBuilder() {

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            DriverStation.reportError("could not use pathplanner GUI config, using fallback config", true);

            config = DriveTrainConstants.FALLBACK_CONFIG;
        }

        BiConsumer<ChassisSpeeds, DriveFeedforwards> output =
                (speeds, feedForwards) -> this.runVelocity(speeds);

        AutoBuilder.configure(
                this::getEstimatedPose,
                this::reset,
                this::getChassisSpeeds,
                output,
                DriveTrainConstants.PATH_PLANNER_PID,
                config,
                DriveTrainConstants::shouldFlipPath,
                this);
    }

    /**
     * Resets the pose estimation and the gyro to the new pose
     *
     * @param newPose The new pose of the robot
     */
    public void reset(Pose2d newPose) {
        // The gyro real angle should stay in sync with the position angle
        gyro.setAngle(newPose.getRotation());

        poseEstimator.resetPosition(newPose.getRotation(), modulePositions, newPose);
    }

    /**
     * Resets the heading of the robot (gyro angle) To the given Angle
     *
     * @param newAngle The new angle of the robot
     */
    public void resetGyro(Rotation2d newAngle) {
        var newPose = new Pose2d(getEstimatedPose().getX(), getEstimatedPose().getY(), newAngle);

        reset(newPose);
    }

    /**
     * Runs the chassis at the target speed requested.
     *
     * @param targetSpeed The speeds the robot should run at, This needs to be robot oriented.
     *                    (i.e. positive X velocity is the robot going is straight.)
     * @param dt          The time the velocity is applied in. (i.e. the time between calls to this function).
     */
    public void runVelocity(ChassisSpeeds targetSpeed, double dt) {
        var discretizedSpeeds = ChassisSpeeds.discretize(targetSpeed, dt);

        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(discretizedSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveModuleConstants.MAX_WHEEL_SPEED);

        for (int i = 0; i < 4; i++) {
            targetStates[i].optimize(moduleInputs[i].state.angle);

            io[i].setTarget(targetStates[i]);
        }

        Logger.recordOutput("Drive train/Target speeds", targetSpeed);
        Logger.recordOutput("Drive train/targetStates", targetStates);
    }

    /**
     * Runs the chassis at the requested speed (robot relative) with the default time between calls of 0.02 seconds (50 Hz).
     *
     * @param targetSpeed The target speed for the drive train
     */
    public void runVelocity(ChassisSpeeds targetSpeed) {
        runVelocity(targetSpeed, 0.02);
    }

    /**
     * Adds the vision measurement to the pose estimator.
     * This function should be called when the vision has a good estimated pose.
     * This method does not need to be called periodically.
     *
     * @param pose      The estimated pose
     * @param timeStamp The timestamp of the pose (the time it was taken)
     * @param stdDevs   The standard deviation of the measurements.
     *                  A Vector with 3 parameters in the following order:
     *                  X standard deviation (in meters).
     *                  Y standard deviation (in meters).
     *                  Theta standard deviation (in radians).
     */
    public void addVisionMeasurement(Pose2d pose, double timeStamp, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(pose, timeStamp, stdDevs);
    }

    /**
     * Adds the vision measurement to the pose estimator.
     * This function should be called when the vision has a good estimated pose.
     * This method does not need to be called periodically.
     *
     * @param pose      The estimated pose
     * @param timeStamp The timestamp of the pose (the time it was taken)
     */
    public void addVisionMeasurement(Pose2d pose, double timeStamp) {
        poseEstimator.addVisionMeasurement(pose, timeStamp);
    }

    /**
     * Gets the Rotation 2d.
     * This is the combination of the gyro angle and vision measurements.
     * Use this function for the best accuracy.
     * But for driving the robot manually, use {@link #getGyroRotation2d()} for a smoother input.
     *
     * @return The rotation of the chassis
     */
    public Rotation2d getRotation2d() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    /**
     * Gets the Rotation 2d of the gyro.
     * This is only the angle of the gyro.
     * Use this when you need a smooth angle supplier (for manual drive).
     * Otherwise, use {@link #getRotation2d()}.
     *
     * @return The angle of the gyro
     */
    public Rotation2d getGyroRotation2d() {
        return inputs.angle;
    }

    /**
     * Gets the chassis speeds of the chassis.
     * This is the velocity of the chassis in the x, y and theta direction.
     * This speed is robot relative (X positive is the robot's front).
     *
     * @return The chassis speeds
     */
    @AutoLogOutput
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(inputs.currentStates);
    }

    /**
     * Gets the estimated pose of the robot.
     * This pose relays primarily on the encoders on the drive motors and gets correction data from the vision measurements.
     *
     * @return The estimated pose
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        for (int i = 0; i < 4; i++) {
            io[i].update(moduleInputs[i]);

            inputs.currentStates[i] = moduleInputs[i].state;
            modulePositions[i] = moduleInputs[i].position;

            Logger.processInputs("SwerveModule/" + io[i].getName(), moduleInputs[i]);
        }

        // Updates the gyro angle to the latest angle
        inputs.angle = gyro.update();
        Logger.processInputs("DriveTrain", inputs);

        poseEstimator.update(getGyroRotation2d(), modulePositions);

        // Records the estimated pose and rotation
        Logger.recordOutput("Drive train/Rotation", getRotation2d());
        Logger.recordOutput("Drive train/Estimated pose", poseEstimator.getEstimatedPosition());
    }
}
