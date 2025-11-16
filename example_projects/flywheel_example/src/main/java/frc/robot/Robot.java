// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;



public class Robot extends LoggedRobot
{
    private Command autonomousCommand;
    
    private final RobotContainer robotContainer;
    
    
    public Robot()
    {

        Logger.addDataReceiver(new NT4Publisher());

        Logger.start();

        robotContainer = new RobotContainer();

        shit Shit = new shit();

        SmartDashboard.putData("pid", Shit);
        
    }

    public static class shit implements Sendable{

        @Override
        public void initSendable(SendableBuilder builder) {

            PIDController controller = new PIDController(defaultPeriodSecs, defaultPeriodSecs, defaultPeriodSecs);

            controller.initSendable(builder);

            SendableChooser<String> chooser = new SendableChooser<>();

            chooser.setDefaultOption("sss", "saf");

            SmartDashboard.putData("pid/choose", chooser);
        }
        
    }
    
    
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
        MotorManager.getInstance().periodic();
    }
    
    
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {}
    
    
    @Override
    public void disabledExit() {}
    
    
    @Override
    public void autonomousInit()
    {
        autonomousCommand = robotContainer.getAutonomousCommand();
        
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
    }
    
    
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void autonomousExit() {}
    
    
    @Override
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
    }
    
    
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void teleopExit() {}
    
    
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    @Override
    public void testPeriodic() {}
    
    
    @Override
    public void testExit() {}
}
