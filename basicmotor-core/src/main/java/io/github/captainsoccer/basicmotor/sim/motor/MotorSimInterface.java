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

public class MotorSimInterface extends SimSystemInterface {
    public final DCMotorSim motor;

    private final Measurements defaultMeasurements;

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
