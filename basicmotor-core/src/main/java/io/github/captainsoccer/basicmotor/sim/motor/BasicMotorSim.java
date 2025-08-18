package io.github.captainsoccer.basicmotor.sim.motor;


import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.gains.ControllerGains;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
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
     * The default measurements for the motor simulation.
     */
    private final Measurements defaultMeasurements;

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
        super(name, gains);
        this.motor = motor;

        defaultMeasurements = new MotorSimEncoder(motor, unitConversion);
    }

    /**
     * Creates a BasicSimMotor instance with the provided configuration.
     * If the kv and ka values are set, it will use those for the simulation.
     * Otherwise, it will use the moment of inertia and gear ratio from the configuration.
     *
     * @param config The configuration for the motor
     */
    public BasicMotorSim(BasicMotorConfig config) {
        super(config);

        this.motor = createSimMotor(config);

        this.defaultMeasurements = new MotorSimEncoder(motor, config.motorConfig.unitConversion);
    }

    @Override
    protected void setInputVoltage(double voltage) {
        motor.setInputVoltage(voltage);
    }

    /**
     * Creates a DCMotorSim based on the provided configuration.
     * This method initializes the motor simulation.
     * The configuration must have either the moment of inertia or the kv and ka values set in the
     * {@link BasicMotorConfig.SimulationConfig} for the motor simulation to work correctly.
     *
     * @param config The configuration for the motor
     * @return A DCMotorSim instance configured according to the provided BasicMotorConfig
     */
    private static DCMotorSim createSimMotor(BasicMotorConfig config) {
        var simConfig = config.simulationConfig;

        if (simConfig.momentOfInertia == 0 && simConfig.kV == 0 && simConfig.kA == 0)
            throw new IllegalArgumentException(
                    "you must provide either a moment of inertia or kV and kA for the simulation motor");

        var plant =
                simConfig.momentOfInertia == 0
                        ? LinearSystemId.createDCMotorSystem(simConfig.kV, simConfig.kA)
                        : LinearSystemId.createDCMotorSystem(
                        config.motorConfig.motorType,
                        simConfig.momentOfInertia,
                        config.motorConfig.gearRatio);

        return new DCMotorSim(
                plant,
                config.motorConfig.motorType.withReduction(config.motorConfig.gearRatio),
                simConfig.positionStandardDeviation,
                simConfig.velocityStandardDeviation);
    }

    @Override
    protected double getCurrentDraw() {
        return motor.getCurrentDrawAmps();
    }

    @Override
    protected Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }
}
