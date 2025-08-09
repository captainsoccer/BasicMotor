// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class SimGyroIO implements GyroIO{
    private final Supplier<ChassisSpeeds> speedSupplier;

    private Rotation2d angle = Rotation2d.kZero;

    public SimGyroIO(Supplier<ChassisSpeeds> speedSupplier){
        this.speedSupplier = speedSupplier;
    }

    @Override
    public Rotation2d update() {
        double theta = speedSupplier.get().omegaRadiansPerSecond;

        angle = Rotation2d.fromRadians(angle.getRadians() + theta * 0.02);

        return angle;
    }

    @Override
    public void setAngle(Rotation2d angle) {
        this.angle = angle;
    }

    @Override
    public Rotation2d getAngle() {
        return angle;
    }
    
}
