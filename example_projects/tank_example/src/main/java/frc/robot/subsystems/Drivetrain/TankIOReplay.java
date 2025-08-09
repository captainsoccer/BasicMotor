package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

/**
 * Replay mode implementation of the TankIO interface.
 * This class does not perform any actual hardware interaction.
 * It is used to replay logged inputs during simulation or testing.
 */
public class TankIOReplay implements TankIO{

    @Override
    public void updateInputs(TankInputs inputs) {

    }

    @Override
    public void setTargetSpeeds(DifferentialDriveWheelSpeeds targetSpeeds) {

    }

    @Override
    public void setVoltage(double leftVoltage, double rightVoltage) {

    }
}
