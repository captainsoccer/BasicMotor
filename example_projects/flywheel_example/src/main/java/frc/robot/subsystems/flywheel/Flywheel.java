package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Flywheel subsystem for controlling the flywheel mechanism in a robot.
 * This subsystem uses a basic motor configuration and provides methods to set
 * target velocity, RPM, voltage, and percent output.
 */
public class Flywheel extends SubsystemBase {
    /**
     * Interface for Flywheel I/O operations.
     */
    private final FlywheelIO io;

    /**
     * Inputs for the Flywheel I/O, automatically logged for diagnostics.
     */
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    /**
     * Constructor for the Flywheel subsystem.
     */
    public Flywheel() {
        io = new FlywheelIOBasic();
    }

    /**
     * Sets the target velocity of the flywheel in meters per second.
     * @param targetMetersPerSecond the target velocity in meters per second.
     */
    public void setTargetVelocity(double targetMetersPerSecond) {
        io.setTargetVelocity(checkVelocity(targetMetersPerSecond));
    }

    /**
     * Sets the target RPM of the flywheel.
     * @param targetRPM the target RPM to set.
     */
    public void setTargetRPM(double targetRPM) {
        double targetMetersPerSecond = (targetRPM / 60.0) * (2 * Math.PI * FlywheelConstants.WHEEL_RADIUS);

        io.setTargetVelocity(checkVelocity(targetMetersPerSecond));
    }

    /**
     * Sets the voltage applied to the flywheel motor.
     * @param voltage the voltage to apply to the motor.
     */
    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    /**
     * Sets the percent output for the flywheel motor.
     * @param percentOutput the percent output to set (0.0 to 1.0).
     */
    public void setPercentOutput(double percentOutput) {
        io.setPrecentOutput(percentOutput);
    }

    /**
     * Checks and clamps the target velocity to ensure it does not exceed the maximum allowed velocity.
     * @param targetMetersPerSecond the target velocity in meters per second.
     * @return the clamped target velocity.
     */
    private static double checkVelocity(double targetMetersPerSecond) {
        return MathUtil.clamp(targetMetersPerSecond, -FlywheelConstants.MAX_VELOCITY, FlywheelConstants.MAX_VELOCITY);
    }

    /**
     * Gets the current velocity of the flywheel in meters per second.
     * @return the current velocity in meters per second.
     */
    public double getVelocityMetersPerSecond() {
        return inputs.velocityMetersPerSecond;
    }

    /**
     * Gets the current RPM of the flywheel.
     * @return the current RPM.
     */
    public double getRPM() {
        return (inputs.velocityMetersPerSecond / (2 * Math.PI * FlywheelConstants.WHEEL_RADIUS)) * 60.0;
    }

    /**
     * Checks if the flywheel is at the target velocity.
     * @return true if the flywheel is at the target velocity, false otherwise.
     */
    public boolean isAtTarget() {
        return inputs.atTarget;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
    }
}
