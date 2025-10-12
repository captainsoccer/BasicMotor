package io.github.captainsoccer.basicmotor.sim;

import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.MotorInterface;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;

public abstract class SimSystemInterface extends MotorInterface {

    protected SimSystemInterface(String name) {
        super(name);
    }

    protected SimSystemInterface(BasicMotorConfig config) {
        super(config);
    }

    @Override
    public double getInternalPIDLoopTime() {
        return MotorManager.ControllerLocation.RIO.getSeconds();
    }

    @Override
    public void setInverted(boolean inverted) {

    }

    @Override
    public void setIdleMode(BasicMotor.IdleMode mode) {

    }

    @Override
    public void updatePIDGainsToMotor(PIDGains pidGains, int slot) {

    }

    @Override
    public void updateConstraintsGainsToMotor(ConstraintsGains constraints) {

    }
}
