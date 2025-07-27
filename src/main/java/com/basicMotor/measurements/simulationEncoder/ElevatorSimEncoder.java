package com.basicMotor.measurements.simulationEncoder;

import com.basicMotor.measurements.Measurements;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * The ElevatorSimEncoder class simulates an encoder for an ElevatorSim. It calculates the position,
 * velocity, and acceleration of the elevator based on the simulation.
 */
public class ElevatorSimEncoder extends Measurements {
    /**
     * The ElevatorSim instance used by this ElevatorSimEncoder.
     */
    private final ElevatorSim elevator;

    /**
     * The current velocity of the elevator in meters per second.
     */
    private double currentVelocity = 0.0;
    /**
     * The previous velocity of the elevator in meters per second. Used to calculate acceleration.
     */
    private double previousVelocity = 0.0;

    /**
     * The acceleration of the elevator in meters per second squared.
     * Calculated as the change in velocity over time.
     */
    private double acceleration = 0.0;

    /**
     * Creates a new ElevatorSimEncoder for the given ElevatorSim.
     *
     * @param elevator The ElevatorSim to use for simulation
     */
    public ElevatorSimEncoder(ElevatorSim elevator) {
        this.elevator = elevator;
    }

    @Override
    public Measurement update(double dt) {
        elevator.update(dt);

        currentVelocity = elevator.getVelocityMetersPerSecond();

        // Calculate acceleration as the change in velocity over time
        acceleration = (currentVelocity - previousVelocity) / dt;

        previousVelocity = currentVelocity;

        return super.update(dt);
    }

    @Override
    protected double getUpdatedPosition() {
        return elevator.getPositionMeters();
    }

    @Override
    protected double getUpdatedVelocity() {
        return currentVelocity;
    }

    @Override
    protected double getUpdatedAcceleration() {
        return acceleration;
    }

    @Override
    public void setPosition(double position) {
        elevator.setState(position, elevator.getVelocityMetersPerSecond());
    }
}
