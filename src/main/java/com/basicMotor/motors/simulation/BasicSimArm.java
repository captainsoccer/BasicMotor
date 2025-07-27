package com.basicMotor.motors.simulation;

import com.basicMotor.configuration.BasicMotorConfig;
import com.basicMotor.gains.ControllerConstraints;
import com.basicMotor.gains.ControllerGains;
import com.basicMotor.measurements.Measurements;
import com.basicMotor.measurements.simulationEncoder.ArmSimEncoder;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * A class that simulates a single-jointed arm system using the SingleJointedArmSim class. It is in
 * the basic sim motor system and has all the functionality of a basic sim system. Use this when you
 * want to simulate an arm in your robot code. units are in rotations.
 */
public class BasicSimArm extends BasicSimSystem {
  /** The SingleJointedArmSim instance used by this BasicSimArm. */
  private final SingleJointedArmSim armSim;

  /** The default measurements for the arm simulation. */
  private final Measurements defaultMeasurements;

  /**
   * Creates a BasicSimArm instance with the provided SingleJointedArmSim and name.
   *
   * @param armSim The SingleJointedArmSim instance to use for the arm simulation
   * @param name The name of the arm simulation
   * @param gains The controller gains to use for the arm simulation
   */
  public BasicSimArm(SingleJointedArmSim armSim, String name, ControllerGains gains) {
    super(name, gains);
    this.armSim = armSim;

    defaultMeasurements = new ArmSimEncoder(armSim);
  }

  /**
   * Creates a BasicSimArm instance with the provided configuration.
   * the config must have the {@link BasicMotorConfig.SimulationConfig#momentOfInertia} set.
   *
   * @param config The configuration for the arm motor
   */
  public BasicSimArm(BasicMotorConfig config) {
    super(config);

    this.armSim = createArmSim(config);

    defaultMeasurements = new ArmSimEncoder(armSim);
  }

  /**
   * Creates a SingleJointedArmSim based on the provided configuration.
   * This method initializes the arm simulation
   *
   * @param config The configuration for the arm motor
   * @return A SingleJointedArmSim instance configured according to the provided BasicMotorConfig
   */
  private static SingleJointedArmSim createArmSim(BasicMotorConfig config) {
    var plant =
        LinearSystemId.createSingleJointedArmSystem(
            config.motorConfig.motorType,
            config.simulationConfig.momentOfInertia,
            config.motorConfig.gearRatio);

    double minAngle;
    double maxAngle;
    if (config.constraintsConfig.constraintType == ControllerConstraints.ConstraintType.LIMITED) {
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
        Units.rotationsPerMinuteToRadiansPerSecond(simConfig.velocityStandardDeviation* 60);

    return new SingleJointedArmSim(
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
  }

  @Override
  protected void setInputVoltage(double voltage) {
    armSim.setInputVoltage(voltage);
  }

  @Override
  protected double getCurrentDraw() {
    return armSim.getCurrentDrawAmps();
  }

  @Override
  protected Measurements getDefaultMeasurements() {
    return defaultMeasurements;
  }
}
