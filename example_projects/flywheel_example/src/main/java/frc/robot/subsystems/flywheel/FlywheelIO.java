package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    class FlywheelIOInputs {
        public double velocityMetersPerSecond = 0.0; // Current velocity in meters per second
        public boolean atTarget = false; // Whether the flywheel is at the target velocity
    }

    void updateInputs(FlywheelIOInputs inputs);

    void setTargetVelocity(double targetMetersPerSecond);

    void setVoltage(double voltage);

    void setPrecentOutput(double percentOutput);
}
