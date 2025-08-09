// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.swerveModule;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * This class serves as an interface for the swerve module IO.
 * This handles the communication between the robot and the swerve module,
 * including setting the target state, updating the state and position,
 * and updating the logs.
 */
public abstract class SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleInputs{
        /**
         * The current state of the swerve module.
         */
        public SwerveModuleState state = new SwerveModuleState();
        /**
         * The current position of the swerve module.
         */
        public SwerveModulePosition position = new SwerveModulePosition();

        /**
         * The color of the CAN coder.
         * This is used for logging purposes.
         */
        public String canCoderColor = "";
    }

    /**
     * Constants for the swerve module.
     */
    protected final SwerveModuleConstants constants;

    /**
     * Constructor for the SwerveModuleIO class.
     * @param constants The constants for the swerve module
     */
    public SwerveModuleIO(SwerveModuleConstants constants){
        this.constants = constants;
    }

    /**
     * Gets the name of the swerve module.
     * @return The name of the swerve module
     */
    public String getName(){
        return constants.NAME;
    }

    /**
     * Updates the swerve module state and position.
     * Also updates miscellaneous logs such as the can coder color.
     * @param inputs The inputs containing the state and position of the swerve module
     */
    public void update(SwerveModuleInputs inputs){
        updateStateAndPosition(inputs.state, inputs.position);

        updateLogs(inputs);
    }

    /**
     * Sets a new target for the swerve module.
     * The target should be limited to the max speed and optimized for the shortest angle
     * @param targetState The target state of the module
     */
    public abstract void setTarget(SwerveModuleState targetState);

    /**
     * Updates the state and position of the swerve module.
     * The function updates the internal state and position of the swerve module
     * @param state The current state of the swerve module
     * @param position The current position of the swerve module
     */
    protected abstract void updateStateAndPosition(SwerveModuleState state, SwerveModulePosition position);

    /**
     * Updates the logs for the swerve module.
     * This is used to log the can coder color and other miscellaneous information.
     * @param inputs The inputs containing the state and position of the swerve module
     */
    protected abstract void updateLogs(SwerveModuleInputs inputs);
}
