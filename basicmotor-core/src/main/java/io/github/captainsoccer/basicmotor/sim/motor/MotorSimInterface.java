package io.github.captainsoccer.basicmotor.sim.motor;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.sim.SimSystemInterface;

/**
 * This is the interface for a simulated DC motor system using the DCMotorSim class.
 * It extends the SimSystemInterface and provides the DCMotorSim instance and default measurements.
 */
public class MotorSimInterface extends SimSystemInterface {
    /** The DCMotorSim instance used by this MotorSimInterface. */
    public final DCMotorSim motor;

    /** The default measurements for this MotorSimInterface. */
    private final Measurements defaultMeasurements;

    /**
     * Creates a MotorSimInterface instance with the provided DCMotorSim and name.
     *
     * @param motor          The DCMotorSim instance to use for the motor simulation
     * @param name           The name of the motor simulation
     * @param unitConversion The conversion factor for the motor's position units.
     *                       This will be multiplied by the motors rotation to get the position with the desired units.
     *                       The unit for this value is desired position unit per rotation.
     */
    public MotorSimInterface(DCMotorSim motor, String name, double unitConversion) {
        super(name);
        this.motor = motor;
        this.defaultMeasurements = new MotorSimEncoder(motor, unitConversion);
    }

    /**
     * Creates a BasicSimMotor instance with the provided configuration.
     * If the kv and ka values are set, it will use those for the simulation.
     * Otherwise, it will use the moment of inertia and gear ratio from the configuration.
     *
     * @param config The configuration for the motor
     */
    public MotorSimInterface(BasicMotorConfig config) {
        super(config);

        var simConfig = config.simulationConfig;

        if (simConfig.momentOfInertia == 0 && simConfig.kV == 0 && simConfig.kA == 0)
            throw new IllegalArgumentException(
                    "you must provide either a moment of inertia or kV and kA for the simulation motor");

        LinearSystem<N2, N1, N2> plant;
        if (simConfig.momentOfInertia == 0) {
            double unitConversion = config.motorConfig.unitConversion;

            Per<VoltageUnit, AngularVelocityUnit> kV =
                    Units.Volts.per(Units.RotationsPerSecond).ofNative(simConfig.kV * unitConversion);

            Per<VoltageUnit, AngularAccelerationUnit> kA =
                    Units.Volts.per(Units.RotationsPerSecondPerSecond).ofNative(simConfig.kA * unitConversion);

            plant = LinearSystemId.createDCMotorSystem(kV.in(Units.VoltsPerRadianPerSecond), kA.in(Units.VoltsPerRadianPerSecondSquared));
        } else {
            plant = LinearSystemId.createDCMotorSystem(config.motorConfig.motorType, simConfig.momentOfInertia, config.motorConfig.gearRatio);
        }

        motor = new DCMotorSim(
                plant,
                config.motorConfig.motorType.withReduction(config.motorConfig.gearRatio),
                simConfig.positionStandardDeviation,
                simConfig.velocityStandardDeviation);

        this.defaultMeasurements = new MotorSimEncoder(motor, config.motorConfig.unitConversion);
    }

    @Override
    public Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }
}
