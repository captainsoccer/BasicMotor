package frc.robot.subsystems.flywheel;

import com.basicMotor.BasicMotor;
import com.basicMotor.configuration.BasicMotorConfig;
import com.basicMotor.configuration.BasicTalonFXConfig;

public class FlywheelConstants {
    public static final double WHEEL_RADIUS = 0.0762; // in meters (3 inches)

    public static final double MAX_VELOCITY = 100.0; // Maximum velocity in meters per second

    public static final BasicMotorConfig motorConfig = new BasicTalonFXConfig();
    static {
        motorConfig.motorConfig.name = "Flywheel Motor";
        motorConfig.motorConfig.id = 1; // Example ID, change as needed
        motorConfig.motorConfig.inverted = false; // Set to true if the motor is inverted
        motorConfig.motorConfig.idleMode = BasicMotor.IdleMode.COAST;
        motorConfig.motorConfig.gearRatio = 5; // Example gear ratio, change as needed
        motorConfig.motorConfig.unitConversion = WHEEL_RADIUS * 2 * Math.PI; // Convert radius to circumference for unit conversion

        motorConfig.pidConfig.kP = 4; // Proportional gain
        motorConfig.pidConfig.kI = 0; // Integral gain
        motorConfig.pidConfig.kD = 0; // Derivative gain

        motorConfig.feedForwardConfig.setpointFeedForward = 0.2; // Feedforward for setpoint
        motorConfig.feedForwardConfig.frictionFeedForward = 0.05; // Friction feedforward

        motorConfig.constraintsConfig.maxOutput = 10; // Maximum output in Volts
        motorConfig.constraintsConfig.minOutput = -10; // Minimum output in Volts

        var talonFXConfig = (BasicTalonFXConfig) motorConfig;
        talonFXConfig.currentLimitConfig.supplyCurrentLimit = 40; // Supply current limit in Amperes
        talonFXConfig.currentLimitConfig.statorCurrentLimit = 60; // stator current limit in Amperes
    }
}
