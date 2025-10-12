package io.github.captainsoccer.basicmotor.ctre.victorspx;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DriverStation;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.MotorInterface;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;
import io.github.captainsoccer.basicmotor.measurements.EmptyMeasurements;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;

/**
 * A motor interface for the VictorSPX motor controller.
 * This class handles the configuration and communication with the VictorSPX motor controller.
 * Note that the VictorSPX does not support PID gains directly, so any closed loop control must be handled externally.
 */
public class VictorSPXInterface extends MotorInterface {
    /** The VictorSPX motor controller. */
    public final VictorSPX motor;

    /** The default measurements for the VictorSPX motor controller. */
    private final Measurements defaultMeasurements;

    /**
     * Creates a VictorSPXInterface with the provided motor ID and name.
     * @param id The ID of the VictorSPX motor controller
     * @param name The name of the motor controller
     * @param defaultMeasurements The default measurements for the motor controller. If null, EmptyMeasurements will be used.
     */
    public VictorSPXInterface(int id, String name, Measurements defaultMeasurements) {
        super(name);

        motor = new VictorSPX(id);
        motor.configFactoryDefault();
        motor.configVoltageCompSaturation(MotorManager.config.motorIdealVoltage);

        this.defaultMeasurements = defaultMeasurements == null ? new EmptyMeasurements() : defaultMeasurements;
    }

    /**
     * Creates a VictorSPXInterface with the provided motor ID and name.
     * Since no measurements are provided, it will use the default empty measurements.
     * @param id The ID of the VictorSPX motor controller
     * @param name The name of the motor controller
     * @see #VictorSPXInterface(int, String, Measurements)
     */
    public VictorSPXInterface(int id, String name) {
        this(id, name, new EmptyMeasurements());
    }

    /**
     * Creates a VictorSPXInterface with the provided configuration.
     * @param config the configuration for the motor
     * @param defaultMeasurements The default measurements for the motor controller. If null, EmptyMeasurements will be used.
     */
    public VictorSPXInterface(BasicMotorConfig config, Measurements defaultMeasurements) {
        this(config.motorConfig.id, config.motorConfig.name, defaultMeasurements);
    }

    @Override
    public Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }

    @Override
    public double getInternalPIDLoopTime() {
        return 0.001; // TalonSRX has a fixed internal loop time of 1ms
        // Also doesn't matter as the victorSPX does not support PID gains directly.
        //According to a chief delphi post:
        // https://www.chiefdelphi.com/t/control-loop-timing-of-various-motor-controllers/370356/4
    }

    @Override
    public void setInverted(boolean inverted) {
        motor.setInverted(inverted);
    }

    @Override
    public void setIdleMode(BasicMotor.IdleMode mode) {
        NeutralMode idleMode = switch (mode) {
            case COAST -> NeutralMode.Coast;
            case BRAKE -> NeutralMode.Brake;
        };

        motor.setNeutralMode(idleMode);
    }

    @Override
    public void updatePIDGainsToMotor(PIDGains pidGains, int slot) {
        // Nothing
        DriverStation.reportWarning("VictorSPX does not support PID gains directly. Ignoring PID gains for motor " + name, false);
    }

    @Override
    public void updateConstraintsGainsToMotor(ConstraintsGains constraints) {
        // Does nothing with soft limits.
        motor.configPeakOutputForward(constraints.getMaxMotorOutput() / MotorManager.config.motorIdealVoltage);
        motor.configPeakOutputReverse(-constraints.getMaxMotorOutput() / MotorManager.config.motorIdealVoltage);

        motor.configNominalOutputForward(constraints.getVoltageDeadband() / MotorManager.config.motorIdealVoltage);
        motor.configNominalOutputReverse(-constraints.getVoltageDeadband() / MotorManager.config.motorIdealVoltage);

        motor.configClosedloopRamp(constraints.getRampRate());
        motor.configOpenloopRamp(constraints.getRampRate());
    }
}
