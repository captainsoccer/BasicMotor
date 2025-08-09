package frc.robot.subsystems.drivetrain.swerveModule;

import com.basicMotor.BasicMotor;
import com.basicMotor.controllers.Controller;
import com.basicMotor.motors.simulation.BasicSimMotor;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * This class implements the SwerveModuleIO interface for a simulated swerve module.
 * It handles the simulation of the swerve module's drive and steer motors,
 * including setting the target state,
 * updating the state and position, and updating logs.
 * This class is used for testing and simulation purposes,
 * allowing for the swerve module to be simulated without actual hardware.
 */
public class SwerveModuleSim extends  SwerveModuleIO{
    private final BasicMotor driveMotor;
    private final BasicMotor steerMotor;

    public SwerveModuleSim(SwerveModuleConstants constants){
        super(constants);

        driveMotor = new BasicSimMotor(constants.DRIVE_MOTOR_CONFIG);

        steerMotor = new BasicSimMotor(constants.STEER_MOTOR_CONFIG);
    }

    @Override
    public void updateLogs(SwerveModuleInputs inputs){
    }

    @Override
    protected void updateStateAndPosition(SwerveModuleState state, SwerveModulePosition position) {
        Rotation2d angle = Rotation2d.fromRotations(steerMotor.getPosition());

        state.angle = angle;
        position.angle = angle;

        state.speedMetersPerSecond = driveMotor.getVelocity();

        position.distanceMeters = driveMotor.getPosition();
    }

    @Override
    public void setTarget(SwerveModuleState targetState) {
        driveMotor.setControl(targetState.speedMetersPerSecond, Controller.ControlMode.VELOCITY);

        steerMotor.setControl(targetState.angle.getRotations(), Controller.ControlMode.POSITION);
    }
}
