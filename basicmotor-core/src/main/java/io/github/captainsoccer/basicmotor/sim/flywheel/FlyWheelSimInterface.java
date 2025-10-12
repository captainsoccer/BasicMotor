package io.github.captainsoccer.basicmotor.sim.flywheel;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.sim.SimSystemInterface;

public class FlyWheelSimInterface extends SimSystemInterface {

    private final Measurements defaultMeasurements;

    public final FlywheelSim flywheelSim;

    /**
     * Creates a BasicSimFlyWheel instance with the provided FlywheelSim and name.
     *
     * @param flywheelSim The FlywheelSim instance to use for the flywheel simulation
     * @param name        The name of the flywheel simulation
     */
    public FlyWheelSimInterface(FlywheelSim flywheelSim, String name) {
        super(name);

        this.flywheelSim = flywheelSim;
        this.defaultMeasurements = new FlywheelSimEncoder(flywheelSim);
    }

    /**
     * Creates a BasicSimFlyWheel instance with the provided configuration.
     * If the kv and ka values are set, it will use those for the simulation.
     * Otherwise, it will use the moment of inertia and gear ratio from the configuration.
     *
     * @param config The configuration for the flywheel motor
     */
    public FlyWheelSimInterface(BasicMotorConfig config) {
        super(config);

        var simConfig = config.simulationConfig;

        if (simConfig.momentOfInertia == 0 && simConfig.kV == 0 && simConfig.kA == 0)
            throw new IllegalArgumentException(
                    "you must provide either a moment of inertia or kV and kA for the simulation motor");

        // gives priority to the kv and ka values if they are set
        if (simConfig.kV == 0 && simConfig.kA == 0) {
            flywheelSim = new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            config.motorConfig.motorType,
                            simConfig.momentOfInertia,
                            config.motorConfig.gearRatio),
                    config.motorConfig.motorType,
                    simConfig.velocityStandardDeviation);
        } else {
            double unitConversion = config.motorConfig.unitConversion;

            Per<VoltageUnit, AngularVelocityUnit> kV =
                    Units.Volts.per(Units.RotationsPerSecond).ofNative(simConfig.kV * unitConversion);

            Per<VoltageUnit, AngularAccelerationUnit> kA =
                    Units.Volts.per(Units.RotationsPerSecondPerSecond).ofNative(simConfig.kA * unitConversion);

            LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(kV.in(Units.VoltsPerRadianPerSecond), kA.in(Units.VoltsPerRadianPerSecondSquared));

            flywheelSim = new FlywheelSim(
                    plant,
                    config.motorConfig.motorType,
                    simConfig.velocityStandardDeviation);


        }

        this.defaultMeasurements = new FlywheelSimEncoder(flywheelSim);
    }

    @Override
    public Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }
}
