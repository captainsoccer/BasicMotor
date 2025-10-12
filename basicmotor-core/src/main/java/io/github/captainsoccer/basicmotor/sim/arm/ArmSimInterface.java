package io.github.captainsoccer.basicmotor.sim.arm;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.sim.SimSystemInterface;

/**
 * The interface for a simulated arm system using the SingleJointedArmSim class.
 * It extends the SimSystemInterface and provides the SingleJointedArmSim instance and default measurements.
 */
public class ArmSimInterface extends SimSystemInterface {

    /** The SingleJointedArmSim instance used by this ArmSimInterface. */
    public final SingleJointedArmSim armSim;

    /** The default measurements for this ArmSimInterface. */
    private final Measurements defaultMeasurements;

    /**
     * Creates a BasicSimArm instance with the provided SingleJointedArmSim and name.
     *
     * @param armSim The SingleJointedArmSim instance to use for the arm simulation
     * @param name   The name of the arm simulation
     */
    public ArmSimInterface(SingleJointedArmSim armSim, String name) {
        super(name);

        this.armSim = armSim;

        this.defaultMeasurements = new ArmSimEncoder(armSim);
    }

    /**
     * Creates a BasicSimArm instance with the provided configuration.
     * the config must have the {@link BasicMotorConfig.SimulationConfig#momentOfInertia} set.
     *
     * @param config The configuration for the arm motor
     */
    public ArmSimInterface(BasicMotorConfig config) {
        super(config);

        // Create the plant model for the arm system based on the motor type, moment of inertia, and gear ratio
        var plant =
                LinearSystemId.createSingleJointedArmSystem(
                        config.motorConfig.motorType,
                        config.simulationConfig.momentOfInertia,
                        config.motorConfig.gearRatio);


        // apply constraints based on the configuration
        double minAngle;
        double maxAngle;
        if (config.constraintsConfig.constraintType == ConstraintsGains.ConstraintType.LIMITED) {
            minAngle = Units.rotationsToRadians(config.constraintsConfig.minValue / config.motorConfig.unitConversion);
            maxAngle = Units.rotationsToRadians(config.constraintsConfig.maxValue / config.motorConfig.unitConversion);
        } else {
            minAngle = Double.NEGATIVE_INFINITY;
            maxAngle = Double.POSITIVE_INFINITY;
        }

        var simConfig = config.simulationConfig;

        double startingAngle = Units.rotationsToRadians(simConfig.armSimConfig.startingAngle);

        double positionSTD = Units.rotationsToRadians(simConfig.positionStandardDeviation);

        double velocitySTD =
                Units.rotationsPerMinuteToRadiansPerSecond(simConfig.velocityStandardDeviation * 60);

        armSim = new SingleJointedArmSim(
                plant,
                config.motorConfig.motorType,
                config.motorConfig.gearRatio,
                simConfig.armSimConfig.armlengthMeters,
                minAngle,
                maxAngle,
                simConfig.armSimConfig.simulateGravity,
                startingAngle,
                positionSTD,
                velocitySTD);

        defaultMeasurements = new ArmSimEncoder(armSim);
    }

    @Override
    public Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }
}
