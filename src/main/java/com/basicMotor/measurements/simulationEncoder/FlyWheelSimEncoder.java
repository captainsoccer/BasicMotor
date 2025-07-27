package com.basicMotor.measurements.simulationEncoder;

import com.basicMotor.measurements.Measurements;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * A class that simulates a flywheel encoder using the FlywheelSim class.
 * It is used to get the position, velocity, and acceleration of the flywheel in a simulation environment.
 * As the flywheels main component is its velocity, it will sum the velocity over time to get the position.
 */
public class FlyWheelSimEncoder extends Measurements {
    /**
     * The FlywheelSim instance used by this FlyWheelSimEncoder.
     */
    private final FlywheelSim flywheelSim;

    /**
     * The current position of the flywheel in rotations.
     * This is updated in the update method.
     */
    private double position = 0.0;

    /**
     * The current velocity of the flywheel in rotations per second.
     */
    private double velocity = 0.0;

    /**
     * Creates a FlyWheelSimEncoder instance with the provided FlywheelSim.
     *
     * @param flywheelSim The FlywheelSim to use.
     */
    public FlyWheelSimEncoder(FlywheelSim flywheelSim) {
        this.flywheelSim = flywheelSim;
    }

    @Override
    public Measurement update(double dt) {
        flywheelSim.update(dt);

        velocity = flywheelSim.getAngularVelocity().in(Units.RotationsPerSecond);

        position += velocity * dt; // Update position based on velocity and time step

        return super.update(dt);
    }

    @Override
    protected double getUpdatedPosition() {
        return position;
    }

    @Override
    protected double getUpdatedVelocity() {
        return velocity;
    }

    @Override
    protected double getUpdatedAcceleration() {
        return flywheelSim.getAngularAcceleration().in(Units.RotationsPerSecondPerSecond);
    }

    @Override
    public void setPosition(double position) {
        //Does nothing as the flywheel sim does not support setting position directly.
    }
}
