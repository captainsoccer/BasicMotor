// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.swerveModule;

import com.basicMotor.BasicMotor;
import com.basicMotor.controllers.Controller.ControlMode;
import com.basicMotor.measurements.ctreEncoders.MeasurementsCANCoder;
import com.basicMotor.motors.simulation.BasicSimMotor;
import com.basicMotor.motors.sparkBase.BasicSparkMAX;
import com.basicMotor.motors.talonFX.BasicTalonFX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class SwerveModuleIOBasic extends SwerveModuleIO{
    private final boolean isReal;

    private final BasicMotor driveMotor;
    private final BasicMotor steerMotor;

    private final CANcoder canCoder;

    public SwerveModuleIOBasic(SwerveModuleConstants constants, boolean isReal){
        super(constants);

        this.isReal = isReal;

        if(isReal){
            driveMotor = new BasicTalonFX(constants.DRIVE_MOTOR_CONFIG);

            steerMotor = new BasicSparkMAX(constants.STEER_MOTOR_CONFIG);

            canCoder = configureCanCoder(constants);

            var canCoderMeasurements = new MeasurementsCANCoder(canCoder, 1, 1);

            canCoder.getMagnetHealth(false).setUpdateFrequency(4);

            canCoder.optimizeBusUtilization();

            steerMotor.setMeasurements(canCoderMeasurements);
        }
        else{
            canCoder = null;

            driveMotor = new BasicSimMotor(constants.DRIVE_MOTOR_CONFIG);
            steerMotor = new BasicSimMotor(constants.STEER_MOTOR_CONFIG);
        }
    }

    private static CANcoder configureCanCoder(SwerveModuleConstants constants){
        CANcoder canCoder = new CANcoder(constants.CAN_CODER_ID);

        var config = new CANcoderConfiguration();

        config.FutureProofConfigs = false;

        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        config.MagnetSensor.MagnetOffset = constants.ZERO_OFFSET;

        canCoder.getConfigurator().apply(config);

        return canCoder;
    }

    @Override
    public void updateLogs(SwerveModuleInputs inputs){
        if(isReal)
            inputs.canCoderColor = canCoder.getMagnetHealth(true).getValue().name();
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
        driveMotor.setControl(targetState.speedMetersPerSecond, ControlMode.VELOCITY);

        steerMotor.setControl(targetState.angle.getRotations(), ControlMode.POSITION);
    }
}
