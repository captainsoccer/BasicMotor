// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DriveTrain;

public class RobotContainer {
  private static DriveTrain driveTrain;

  private static LoggedDashboardChooser<Command> chooser;


  public RobotContainer() {
    driveTrain = new DriveTrain();

    configureBindings();
    
    var autoChooser = AutoBuilder.buildAutoChooser();

    chooser = new LoggedDashboardChooser<>("Auto chooser", autoChooser);
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return chooser.get();
  }
}
