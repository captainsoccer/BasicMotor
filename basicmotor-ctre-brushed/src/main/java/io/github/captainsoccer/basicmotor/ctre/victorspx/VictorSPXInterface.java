package io.github.captainsoccer.basicmotor.ctre.victorspx;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.MotorInterface;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;

public class VictorSPXInterface extends MotorInterface {
    public final VictorSPX motor;

    private final Measurements defaultMeasurements;

    protected VictorSPXInterface(String name) {
        super(name);
    }

    protected VictorSPXInterface(BasicMotorConfig config) {
        super(config);
    }

    @Override
    public Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }

    @Override
    public double getInternalPIDLoopTime() {
        return 0;
    }

    @Override
    public void setInverted(boolean inverted) {

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
