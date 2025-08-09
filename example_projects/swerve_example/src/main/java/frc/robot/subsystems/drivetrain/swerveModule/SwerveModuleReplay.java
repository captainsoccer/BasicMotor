package frc.robot.subsystems.drivetrain.swerveModule;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * This class implements the SwerveModuleIO interface for a replay swerve module.
 * When the robot code is replaying a log, this class is empty and is used to get the data from the log.
 */
public class SwerveModuleReplay extends SwerveModuleIO{
    /**
     * Constructor for the SwerveModuleIO class.
     *
     * @param constants The constants for the swerve module
     */
    public SwerveModuleReplay(SwerveModuleConstants constants) {
        super(constants);
    }

    @Override
    public void setTarget(SwerveModuleState targetState) {

    }

    @Override
    protected void updateStateAndPosition(SwerveModuleState state, SwerveModulePosition position) {

    }

    @Override
    protected void updateLogs(SwerveModuleInputs inputs) {

    }
}
