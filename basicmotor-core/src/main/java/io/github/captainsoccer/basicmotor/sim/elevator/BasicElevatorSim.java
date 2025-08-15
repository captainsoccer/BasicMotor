package io.github.captainsoccer.basicmotor.sim.elevator;


import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.gains.ControllerGains;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.sim.BasicSimSystem;

/**
 * A class that simulates an elevator system using the ElevatorSim class. It is part of the
 * basic sim motor system and has all the functionality of a basic sim system.
 * Use this when you want to simulate an elevator in your robot code.
 * Its units are meters and cannot be changed.
 */
public class BasicElevatorSim extends BasicSimSystem {
  /** The elevator simulation instance used by this BasicSimElevator. */
  private final ElevatorSim elevator;

  /** The default measurements for the elevator simulation. */
  private final Measurements defaultMeasurements;

  /**
   * Creates a BasicSimElevator instance with the provided ElevatorSim and name.
   *
   * @param elevator The ElevatorSim instance to use for the elevator simulation
   * @param name The name of the elevator simulation
   * @param gains The controller gains to use for the elevator simulation
   */
  public BasicElevatorSim(ElevatorSim elevator, String name, ControllerGains gains) {
    super(name, gains);
    this.elevator = elevator;

    defaultMeasurements = new ElevatorSimEncoder(elevator);
  }

  /**
   * Creates a BasicSimElevator instance with the provided configuration.
   * If the kv and ka values are set, it will use those for the simulation.
   *
   * @param config The configuration for the elevator motor
   */
  public BasicElevatorSim(BasicMotorConfig config) {
    super(config);

    this.elevator = createElevatorSim(config);

    defaultMeasurements = new ElevatorSimEncoder(elevator);
  }

  @Override
  protected void setInputVoltage(double voltage) {
    elevator.setInputVoltage(voltage);
  }

  /**
   * Creates an ElevatorSim based on the provided configuration.
   * The configuration must have the all the parameters in the
   * {@link BasicMotorConfig.SimulationConfig.ElevatorSimConfig} set.
   *
   * @param config The configuration for the elevator motor
   * @return A new ElevatorSim instance configured according to the provided BasicMotorConfig
   */
  private static ElevatorSim createElevatorSim(BasicMotorConfig config) {
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
      return new ElevatorSim(
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
      return new ElevatorSim(
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
  }

  @Override
  protected double getCurrentDraw() {
    return elevator.getCurrentDrawAmps();
  }

  @Override
  protected Measurements getDefaultMeasurements() {
    return defaultMeasurements;
  }
}
