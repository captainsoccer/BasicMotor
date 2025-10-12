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

public class VictorSPXInterface extends MotorInterface {
    public final VictorSPX motor;

    private final Measurements defaultMeasurements;

    public VictorSPXInterface(int id, String name, Measurements defaultMeasurements) {
        super(name);

        motor = new VictorSPX(id);
        motor.configFactoryDefault();
        motor.configVoltageCompSaturation(MotorManager.config.motorIdealVoltage);

        this.defaultMeasurements = defaultMeasurements == null ? new EmptyMeasurements() : defaultMeasurements;
    }

    public VictorSPXInterface(int id, String name) {
        this(id, name, new EmptyMeasurements());
    }

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
    public void updatePIDGainsToMotor(PIDGains pidGains, int slot, MotorManager.ControllerLocation location) {
        // Nothing
        if(location == MotorManager.ControllerLocation.MOTOR) {
            DriverStation.reportWarning("VictorSPX does not support PID gains directly. Ignoring PID gains for motor " + name, false);
        }
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
