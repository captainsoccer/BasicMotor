package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.BasicMotor.IdleMode;
import io.github.captainsoccer.basicmotor.ctre.talonfx.BasicTalonFXConfig;

public class FlywheelConstants {
    public static final double WHEEL_RADIUS = 0.0762; // in meters (3 inches)

    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_RADIUS * 2 * Math.PI;

    public static final double MAX_VELOCITY = 20; // Maximum velocity in rotations per second

    public static final BasicMotorConfig motorConfig = new BasicTalonFXConfig();
    static {
        motorConfig.motorConfig.name = "Flywheel Motor";
        motorConfig.motorConfig.id = 1; // Example ID, change as needed
        motorConfig.motorConfig.inverted = false; // Set to true if the motor is inverted
        motorConfig.motorConfig.idleMode = IdleMode.COAST;
        motorConfig.motorConfig.gearRatio = 5; // Example gear ratio, change as needed
        motorConfig.motorConfig.motorType = DCMotor.getFalcon500(1);

        motorConfig.pidConfig.kP = 4; // Proportional gain
        motorConfig.pidConfig.kI = 0; // Integral gain
        motorConfig.pidConfig.kD = 0; // Derivative gain

        motorConfig.feedForwardConfig.setpointFeedForward = 0.56; // Feedforward for setpoint
        motorConfig.feedForwardConfig.frictionFeedForward = 0.05; // Friction feedforward

        motorConfig.constraintsConfig.maxOutput = 10; // Maximum output in Volts
        motorConfig.constraintsConfig.minOutput = -10; // Minimum output in Volts

        motorConfig.simulationConfig.kV = 0.564263323;
        motorConfig.simulationConfig.kA = 0.1;

        var talonFXConfig = (BasicTalonFXConfig) motorConfig;
        talonFXConfig.currentLimitConfig.supplyCurrentLimit = 40; // Supply current limit in Amperes
        talonFXConfig.currentLimitConfig.statorCurrentLimit = 60; // stator current limit in Amperes
    }
}
