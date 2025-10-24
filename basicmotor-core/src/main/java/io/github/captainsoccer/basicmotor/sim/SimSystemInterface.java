package io.github.captainsoccer.basicmotor.sim;

import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.MotorInterface;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;

/**
 * This is an abstract class that represents a motor interface for simulation systems.
 * It ignores some of the functionality of a motor interface, as it is meant to be used in a simulation.
 */
public abstract class SimSystemInterface extends MotorInterface {

    /**
     * Creates a SimSystemInterface with the provided name.
     * @param name the name of the motor
     */
    protected SimSystemInterface(String name) {
        super(name);
    }

    /**
     * Creates a SimSystemInterface with the name provided in the configuration.
     * @param config the configuration for the motor
     */
    protected SimSystemInterface(BasicMotorConfig config) {
        super(config);
    }

    @Override
    public void setInverted(boolean inverted) {
        //Does nothing as this is a simulation
    }

    @Override
    public void setIdleMode(BasicMotor.IdleMode mode) {
        // Does nothing as this is a simulation
    }

    @Override
    public void updatePIDGainsToMotor(PIDGains pidGains, int slot) {
        // Does nothing as this is a simulation
    }

    @Override
    public void updateConstraintsGainsToMotor(ConstraintsGains constraints) {
        // Does nothing as this is a simulation
    }
}
