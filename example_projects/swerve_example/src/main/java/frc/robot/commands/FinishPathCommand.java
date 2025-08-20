// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DriveTrain;
import io.github.captainsoccer.basicmotor.gains.PIDGains;

/**
 * The finish path command is used to finish pathPlanner paths.
 * due to how pathplanner and trajectory works, when the path finishes the robot does not try to get to the final pose.
 * This class takes the last target pose from the latest paths and runs PID to get to it.
 * It will finish when the robot will achicve the target pose within tolerance.
 */
public class FinishPathCommand extends Command {
  private final DriveTrain driveTrain;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;

  private Pose2d targetPose = Pose2d.kZero;

  /** Creates a new FinishPathCommand. */
  public FinishPathCommand(DriveTrain driveTrain, PIDGains translationConstants, PIDGains rotationConstants) {
    this.driveTrain = driveTrain;

    // Sets the PID for the XController
    xController = new PIDController(translationConstants.getK_P(), translationConstants.getK_I(), translationConstants.getK_D());
    xController.setIZone(translationConstants.getI_Zone());
    xController.setIntegratorRange(-translationConstants.getI_MaxAccum(), translationConstants.getI_MaxAccum());
    xController.setTolerance(translationConstants.getTolerance(), 0.1);

    // Sets the PID for the YController
    yController = new PIDController(translationConstants.getK_P(), translationConstants.getK_I(), translationConstants.getK_D());
    yController.setIZone(translationConstants.getI_Zone());
    yController.setIntegratorRange(-translationConstants.getI_MaxAccum(), translationConstants.getI_MaxAccum());
    yController.setTolerance(translationConstants.getTolerance(), 0.1);

    // Sets the PID for the thetaCotnroller
    thetaController = new PIDController(rotationConstants.getK_P(), rotationConstants.getK_I(), rotationConstants.getK_D());
    thetaController.setIZone(rotationConstants.getI_Zone());
    thetaController.setIntegratorRange(-rotationConstants.getI_MaxAccum(), rotationConstants.getI_MaxAccum());
    thetaController.setTolerance(rotationConstants.getTolerance(), 0.05);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();

    targetPose = driveTrain.getPathFinalPose();

    xController.setSetpoint(targetPose.getX());
    yController.setSetpoint(targetPose.getY());
    thetaController.setSetpoint(targetPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var currentPose = driveTrain.getEstimatedPose();

    double xOutput = xController.calculate(currentPose.getX());
    double yOutput = yController.calculate(currentPose.getY());
    double thetaOutput = thetaController.calculate(currentPose.getRotation().getRadians());

    Logger.recordOutput("FinishPathCommand/xError", xController.getError());
    Logger.recordOutput("FinishPathCommand/yError", yController.getError());
    Logger.recordOutput("FinishPathCommand/thetaError", thetaController.getError());

    driveTrain.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(xOutput, yOutput, thetaOutput, currentPose.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.runVelocity(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}
