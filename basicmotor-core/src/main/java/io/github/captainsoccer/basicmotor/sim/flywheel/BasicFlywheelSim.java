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
import io.github.captainsoccer.basicmotor.gains.ControllerGains;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
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
     * The default measurements for the flywheel simulation.
     */
    private final Measurements defaultMeasurements;

    /**
     * Creates a BasicSimFlyWheel instance with the provided FlywheelSim and name.
     *
     * @param flywheelSim The FlywheelSim instance to use for the flywheel simulation
     * @param name        The name of the flywheel simulation
     * @param gains       The controller gains to use for the flywheel simulation
     */
    public BasicFlywheelSim(FlywheelSim flywheelSim, String name, ControllerGains gains) {
        super(name, gains);
        this.flywheelSim = flywheelSim;

        defaultMeasurements = new FlywheelEncoder(flywheelSim);
    }

    /**
     * Creates a BasicSimFlyWheel instance with the provided configuration.
     * If the kv and ka values are set, it will use those for the simulation.
     * Otherwise, it will use the moment of inertia and gear ratio from the configuration.
     *
     * @param config The configuration for the flywheel motor
     */
    public BasicFlywheelSim(BasicMotorConfig config) {
        super(config);

        this.flywheelSim = createFlywheelSim(config);

        defaultMeasurements = new FlywheelEncoder(flywheelSim);
    }

    /**
     * Creates a FlywheelSim based on the provided configuration.
     *
     * @param config The configuration for the flywheel motor
     * @return A FlywheelSim instance configured according to the provided BasicMotorConfig
     */
    private static FlywheelSim createFlywheelSim(BasicMotorConfig config) {
        var simConfig = config.simulationConfig;

        if (simConfig.momentOfInertia == 0 && simConfig.kV == 0 && simConfig.kA == 0)
            throw new IllegalArgumentException(
                    "you must provide either a moment of inertia or kV and kA for the simulation motor");

        // gives priority to the kv and ka values if they are set
        if (simConfig.kV == 0 && simConfig.kA == 0) {
            return new FlywheelSim(
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

            return new FlywheelSim(
                    plant,
                    config.motorConfig.motorType,
                    simConfig.velocityStandardDeviation);

        }
    }

    @Override
    protected void setInputVoltage(double voltage) {
        flywheelSim.setInputVoltage(voltage);
    }

    @Override
    protected double getCurrentDraw() {
        return flywheelSim.getCurrentDrawAmps();
    }

    @Override
    protected Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }
}
