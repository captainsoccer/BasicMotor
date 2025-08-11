// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Util;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface GyroIO {
    /**
     * Updates the angle of the gyro, should be called periodically.
     * @return The latest angle of the gyro.
     */
    Rotation2d update();

    /**
     * Sets the angle fo the gyro, use this when resting the robot's heading.
     * @param angle The new angle of the robot.
     */
    void setAngle(Rotation2d angle);

    /**
     * Gets the latest Angle of the gyro.
     * The angle here is updated by {@link #update()}.
     * @return The latest Angle.
     */
    Rotation2d getAngle();
}