package io.github.captainsoccer.basicmotor.sim.flywheel;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.gains.ControllerGains;
import io.github.captainsoccer.basicmotor.sim.BasicSimSystem;

/**
 * A class that simulates a flywheel system using the FlywheelSim class. It is in the basic sim
 * motor system and has all the functionality of a basic sim system. Use this when you want to
 * simulate a flywheel in your robot code. units are in rotations per second.
 */
public class BasicFlywheelSim extends BasicSimSystem {
    /**
     * The FlywheelSim instance used by this BasicSimFlyWheel.
     */
    private final FlywheelSim flywheelSim;

    /**
     * Creates a BasicSimFlyWheel instance with the provided FlywheelSim and name.
     *
     * @param flywheelSim The FlywheelSim instance to use for the flywheel simulation
     * @param name        The name of the flywheel simulation
     * @param gains       The controller gains to use for the flywheel simulation
     */
    public BasicFlywheelSim(FlywheelSim flywheelSim, String name, ControllerGains gains) {
        super(new FlyWheelSimInterface(flywheelSim, name), gains);
        this.flywheelSim = flywheelSim;
    }

    /**
     * Creates a BasicSimFlyWheel instance with the provided configuration.
     * If the kv and ka values are set, it will use those for the simulation.
     * Otherwise, it will use the moment of inertia and gear ratio from the configuration.
     *
     * @param config The configuration for the flywheel motor
     */
    public BasicFlywheelSim(BasicMotorConfig config) {
        super(new FlyWheelSimInterface(config), config);

        this.flywheelSim = ((FlyWheelSimInterface) super.motorInterface).flywheelSim;
    }

    @Override
    protected void setInputVoltage(double voltage) {
        flywheelSim.setInputVoltage(voltage);
    }

    @Override
    protected double getCurrentDraw() {
        return flywheelSim.getCurrentDrawAmps();
    }
}
