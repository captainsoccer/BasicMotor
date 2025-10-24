package io.github.captainsoccer.basicmotor.sim.elevator;


import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.gains.ControllerGains;
import io.github.captainsoccer.basicmotor.sim.BasicSimSystem;

/**
 * A class that simulates an elevator system using the ElevatorSim class. It is part of the
 * basic sim motor system and has all the functionality of a basic sim system.
 * Use this when you want to simulate an elevator in your robot code.
 * Its units are meters and cannot be changed.
 */
public class BasicElevatorSim extends BasicSimSystem {
    /**
     * The elevator simulation instance used by this BasicSimElevator.
     */
    private final ElevatorSim elevator;

    /**
     * Creates a BasicSimElevator instance with the provided ElevatorSim and name.
     *
     * @param elevator The ElevatorSim instance to use for the elevator simulation
     * @param name     The name of the elevator simulation
     * @param gains    The controller gains to use for the elevator simulation
     */
    public BasicElevatorSim(ElevatorSim elevator, String name, ControllerGains gains) {
        super(new ElevatorSimInterface(elevator, name), gains);
        this.elevator = elevator;
    }

    /**
     * Creates a BasicSimElevator instance with the provided configuration.
     * If the kv and ka values are set, it will use those for the simulation.
     *
     * @param config The configuration for the elevator motor
     */
    public BasicElevatorSim(BasicMotorConfig config) {
        super(new ElevatorSimInterface(config), config);

        this.elevator = ((ElevatorSimInterface) super.motorInterface).elevatorSim;
    }

    @Override
    protected void setInputVoltage(double voltage) {
        elevator.setInputVoltage(voltage);
    }

    @Override
    protected double getCurrentDraw() {
        return elevator.getCurrentDrawAmps();
    }
}
