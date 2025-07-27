package com.basicMotor.measurements.simulationEncoder;

import com.basicMotor.measurements.Measurements;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * A class that simulates an arm encoder using the SingleJointedArmSim class.
 * It is used to get the
 * position, velocity, and acceleration of the arm in a simulation environment.
 * Where the 0 angle of the arm is at the horizontal position.
 */
public class ArmSimEncoder extends Measurements {
    /**
     * The SingleJointedArmSim instance used by this ArmSimEncoder.
     */
    private final SingleJointedArmSim armSim;

    /**
     * The current velocity of the arm in rotations per second.
     * This is updated in the update method.
     */
    private double currentVelocity = 0.0;
    /**
     * The previous velocity of the arm in rotations per second.
     * This is used to calculate acceleration.
     */
    private double previousVelocity = 0.0;

    /**
     * The acceleration of the arm in rotations per second squared.
     * This is calculated as the change in velocity over time.
     */
    private double acceleration = 0.0;

    /**
     * Creates an ArmSimEncoder instance with the provided SingleJointedArmSim.
     *
     * @param armSim The SingleJointedArmSim to use for simulation.
     */
    public ArmSimEncoder(SingleJointedArmSim armSim) {
        this.armSim = armSim;
    }

    @Override
    public Measurement update(double dt) {
        armSim.update(dt);

        var angle = edu.wpi.first.units.Units.RadiansPerSecond.of(armSim.getVelocityRadPerSec());
        currentVelocity = angle.in(edu.wpi.first.units.Units.RotationsPerSecond);

        // Calculate acceleration as the change in velocity over time
        acceleration = (currentVelocity - previousVelocity) / dt;

        previousVelocity = currentVelocity;

        return super.update(dt);
    }

    @Override
    protected double getUpdatedPosition() {
        return Units.radiansToRotations(armSim.getAngleRads());
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
        double angleInRadians = Units.rotationsToRadians(position);
        armSim.setState(angleInRadians, armSim.getVelocityRadPerSec());
    }
}
