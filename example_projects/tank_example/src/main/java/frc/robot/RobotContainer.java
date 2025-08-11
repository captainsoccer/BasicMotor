// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Tank;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {
  public static Tank tank;

  public static LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    tank = new Tank();

    configureBindings();

    autoChooser = new LoggedDashboardChooser<>("auto chooser", AutoBuilder.buildAutoChooser());
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
