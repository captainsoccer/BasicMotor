package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;
import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;

/**
 * This is the basic motor implementation of the TankIO interface.
 * It uses BasicMotor to control the left and right motors of a tank drive system.
 * It is designed to work with both real hardware and simulation environments.
 */
public class TankIOBasic implements TankIO{
    /**
     * The lead left motor of the tank drive system.
     */
    private final BasicMotor leadLeftMotor;
    /**
     * The lead right motor of the tank drive system.
     */
    private final BasicMotor leadRightMotor;

    /**
     * Constructor for the TankIOBasic class.
     */
    public TankIOBasic() {
        if(RobotBase.isReal()) {
            leadLeftMotor = new BasicSparkMAX(TankConstants.LEFT.leadMotorConfig);
            leadRightMotor = new BasicSparkMAX(TankConstants.RIGHT.leadMotorConfig);

            BasicMotor leftFollower = new BasicSparkMAX(TankConstants.LEFT.followerMotorConfig);
            BasicMotor rightFollower = new BasicSparkMAX(TankConstants.RIGHT.followerMotorConfig);

            leftFollower.followMotor(leadLeftMotor, TankConstants.followerInvertedToLead);
            rightFollower.followMotor(leadRightMotor, TankConstants.followerInvertedToLead);
        }
        else{
            leadLeftMotor = new BasicMotorSim(TankConstants.LEFT.leadMotorConfig);
            leadRightMotor = new BasicMotorSim(TankConstants.RIGHT.leadMotorConfig);
        }
    }



    @Override
    public void updateInputs(TankInputs inputs) {
        inputs.wheelSpeeds.leftMetersPerSecond = leadLeftMotor.getVelocity();
        inputs.wheelSpeeds.rightMetersPerSecond = leadRightMotor.getVelocity();

        inputs.wheelPositions.leftMeters = leadLeftMotor.getPosition();
        inputs.wheelPositions.rightMeters = leadRightMotor.getPosition();
    }

    @Override
    public void setTargetSpeeds(DifferentialDriveWheelSpeeds targetSpeeds) {
        leadLeftMotor.setControl(targetSpeeds.leftMetersPerSecond, Controller.ControlMode.VELOCITY);
        leadRightMotor.setControl(targetSpeeds.rightMetersPerSecond, Controller.ControlMode.VELOCITY);
    }

    @Override
    public void setVoltage(double leftVoltage, double rightVoltage) {
        leadLeftMotor.setVoltage(leftVoltage);
        leadRightMotor.setVoltage(rightVoltage);
    }
}
