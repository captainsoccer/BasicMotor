package io.github.captainsoccer.basicmotor.sim.elevator;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.sim.SimSystemInterface;

/**
 * The interface for a simulated elevator system using the ElevatorSim class.
 * It extends the SimSystemInterface and provides the ElevatorSim instance and default measurements.
 */
public class ElevatorSimInterface extends SimSystemInterface {
    /** The ElevatorSim instance used by this ElevatorSimInterface. */
    public final ElevatorSim elevatorSim;

    /** The default measurements for this ElevatorSimInterface. */
    private final Measurements defaultMeasurements;

    /**
     * Creates a BasicSimElevator instance with the provided ElevatorSim and name.
     *
     * @param elevator The ElevatorSim instance to use for the elevator simulation
     * @param name     The name of the elevator simulation
     */
    protected ElevatorSimInterface(ElevatorSim elevator, String name) {
        super(name);

        this.elevatorSim = elevator;

        this.defaultMeasurements = new ElevatorSimEncoder(elevator);
    }

    /**
     * Creates a BasicSimElevator instance with the provided configuration.
     * If the kv and ka values are set, it will use those for the simulation.
     *
     * @param config The configuration for the elevator motor
     */
    protected ElevatorSimInterface(BasicMotorConfig config) {
        super(config);

        double minHeight;
        double maxHeight;
        if (config.constraintsConfig.constraintType == ConstraintsGains.ConstraintType.LIMITED) {
            minHeight = config.constraintsConfig.minValue;
            maxHeight = config.constraintsConfig.maxValue;
        } else {
            minHeight = 0;
            maxHeight = Double.POSITIVE_INFINITY;
        }

        var simConfig = config.simulationConfig;

        if (simConfig.kV == 0 && simConfig.kA == 0) {
            elevatorSim = new ElevatorSim(
                    config.motorConfig.motorType,
                    config.motorConfig.gearRatio,
                    simConfig.elevatorSimConfig.massKG,
                    simConfig.elevatorSimConfig.pulleyRadiusMeters,
                    minHeight,
                    maxHeight,
                    simConfig.elevatorSimConfig.enableGravitySimulation,
                    minHeight,
                    simConfig.positionStandardDeviation,
                    simConfig.velocityStandardDeviation);
        } else {
            elevatorSim = new ElevatorSim(
                    simConfig.kV,
                    simConfig.kA,
                    config.motorConfig.motorType,
                    minHeight,
                    maxHeight,
                    simConfig.elevatorSimConfig.enableGravitySimulation,
                    minHeight,
                    simConfig.positionStandardDeviation,
                    simConfig.velocityStandardDeviation);
        }

        this.defaultMeasurements = new ElevatorSimEncoder(elevatorSim);
    }

    @Override
    public Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }
}
