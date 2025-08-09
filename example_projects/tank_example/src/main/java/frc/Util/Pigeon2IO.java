// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Util;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Pigeon2IO implements GyroIO{
    private final Pigeon2 gyro;

    private Rotation2d angle;

    public Pigeon2IO(int id){
        gyro = new Pigeon2(id);
    }

    @Override
    public Rotation2d update(){
        angle = gyro.getRotation2d();
        return  angle;
    }

    @Override
    public Rotation2d getAngle(){
        return angle;
    }

    public void setAngle(Rotation2d angle){
        gyro.setYaw(angle.getDegrees());
    }
}
