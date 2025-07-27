package com.basicMotor.measurements.simulationEncoder;

import com.basicMotor.measurements.Measurements;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** The encoder for a simulated motor. */
public class MotorSimEncoder extends Measurements {
  /** The DCMotorSim instance used by this MotorSimEncoder. */
  private final DCMotorSim motor;

  /**
   * Creates a MotorSimEncoder instance with the provided DCMotorSim and unit conversion factor.
   *
   * @param motor the DCMotorSim to use
   * @param unitConversion the conversion factor for the motor's position units
   */
  public MotorSimEncoder(DCMotorSim motor, double unitConversion) {
    super(1, unitConversion);
    this.motor = motor;
  }

  /**
   * Creates a MotorSimEncoder instance with the provided DCMotorSim and a default unit conversion
   * of 1.
   *
   * @param motor the DCMotorSim to use
   */
  public MotorSimEncoder(DCMotorSim motor) {
    this(motor, 1);
  }

  @Override
  public Measurement update(double dt) {
    motor.update(dt);

    return super.update(dt);
  }

  @Override
  protected double getUpdatedPosition() {
    return motor.getAngularPosition().in(Units.Rotations);
  }

  @Override
  protected double getUpdatedVelocity() {
    return motor.getAngularVelocity().in(Units.RotationsPerSecond);
  }

  @Override
  protected double getUpdatedAcceleration() {
    return motor.getAngularAcceleration().in(Units.RotationsPerSecondPerSecond);
  }

  @Override
  public void setPosition(double position) {
    motor.setAngle(edu.wpi.first.math.util.Units.rotationsToRadians(position));
  }
}
