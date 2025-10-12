package io.github.captainsoccer.basicmotor.sim.motor;


import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.gains.ControllerGains;
import io.github.captainsoccer.basicmotor.sim.BasicSimSystem;
import io.github.captainsoccer.basicmotor.sim.elevator.BasicElevatorSim;

/**
 * A class that simulates a DC motor system using the DCMotorSim class.
 * It is part of the basic sim motor system and has all the functionality of a basic sim system.
 * Use this when you want to simulate a DC motor in your robot code.
 * It is better to use specific mechanisms like {@link BasicElevatorSim} when possible.
 */
public class BasicMotorSim extends BasicSimSystem {
    /**
     * The DCMotorSim instance used by this BasicSimMotor.
     */
    private final DCMotorSim motor;

    /**
     * Creates a BasicSimMotor instance with the provided DCMotorSim and name.
     *
     * @param motor          The DCMotorSim instance to use for the motor simulation
     * @param name           The name of the motor simulation
     * @param gains          The controller gains to use for the motor simulation
     * @param unitConversion The conversion factor for the motor's position units.
     *                       This will be multiplied by the motors rotation to get the position with the desired units.
     *                       The unit for this value is desired position unit per rotation.
     */
    public BasicMotorSim(DCMotorSim motor, String name, ControllerGains gains, double unitConversion) {
        super(new MotorSimInterface(motor, name, unitConversion), gains);

        this.motor = motor;
    }

    /**
     * Creates a BasicSimMotor instance with the provided configuration.
     * If the kv and ka values are set, it will use those for the simulation.
     * Otherwise, it will use the moment of inertia and gear ratio from the configuration.
     *
     * @param config The configuration for the motor
     */
    public BasicMotorSim(BasicMotorConfig config) {
        super(new MotorSimInterface(config), config);

        this.motor = ((MotorSimInterface)super.motorInterface).motor;
    }

    @Override
    protected void setInputVoltage(double voltage) {
        motor.setInputVoltage(voltage);
    }

    @Override
    protected double getCurrentDraw() {
        return motor.getCurrentDrawAmps();
    }

    /**
     * Gets the DCMotorSim instance used by this BasicMotorSim.
     *
     * @return The DCMotorSim instance used by this BasicMotorSim.
     */
    public DCMotorSim getMotorSim() {
        return motor;
    }
}
