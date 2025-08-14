package io.github.captainsoccer.basicmotor.controllers;

import io.github.captainsoccer.basicmotor.LogFrame;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.gains.PIDGains;
import edu.wpi.first.math.MathUtil;

/**
 * This is a very simplified PID controller.
 * It is used to calculate the output of the PID algorithm.
 * It checks for I-zone, integral clamping, and tolerance.
 * It used mainly for the {@link BasicMotor} in the {@link Controller}.
 */
public class BasicPIDController {
  /**
   * The gains of the PID controller
   * They can Update when the user changes the PID gains.
   */
  private PIDGains gains;

  /**
   * The last error of the PID controller
   * This is used to calculate the derivative of the error.
   */
  private double lastError = 0;

  /**
   * The integral of the PID controller
   * This is used to calculate the integral of the error.
   */
  private double integral;

  /**
   * Creates a PID controller with the given gains
   *
   * @param gains The gains of the PID controller
   */
  public BasicPIDController(PIDGains gains) {
    this.gains = gains;
  }

  /**
   * Calculates the PID output based on the setpoint and measurement
   *
   * @param setpoint The target of the PID controller (unit of control)
   * @param measurement the current measurement of the PID controller (unit of control)
   * @param dt the time since the last calculation (in seconds)
   * @return The PID output of the PID controller.
   * The units should be volts.
   * but are determined by the gains.
   */
  public LogFrame.PIDOutput calculate(double setpoint, double measurement, double dt) {
    double error = setpoint - measurement;

    // Calculate the derivative of the error
    double derivative = (error - lastError) / dt;
    lastError = error;

    // checks if the error is within the I_Zone
    if (Math.abs(error) > gains.getI_Zone()) {
      integral = 0;
    }
    // If the error is within the I_Zone, accumulate the integral and clamp it
    else if (gains.getK_I() != 0) {
      integral =
          MathUtil.clamp(
              integral + error * dt, //the units of the integral are unit of control * seconds (not volts!)
              -gains.getI_MaxAccum() / gains.getK_I(), //divides by k_I to get the units of control
              gains.getI_MaxAccum() / gains.getK_I()); //divides by k_I to get the units of control
    }

    double pOutput = gains.getK_P() * error;
    double iOutput = gains.getK_I() * integral;
    double dOutput = gains.getK_D() * derivative;

    // Calculate the total output
    double totalOutput = pOutput + iOutput + dOutput;

    // If the error is within the tolerance, set the output to 0
    if (Math.abs(error) <= gains.getTolerance()) {
      totalOutput = 0;
    }

    // create the PID output
    return new LogFrame.PIDOutput(pOutput, iOutput, dOutput, totalOutput);
  }

  /**
   * Resets the PID controller.
   * This will reset the last error to zero and the integral gain to zero.
   */
  public void reset() {
    lastError = 0;
    integral = 0;
  }

  /**
   * Sets the gains of the PID controller
   *
   * @param gains The new gains of the PID controller
   */
  public void setGains(PIDGains gains) {
    this.gains = gains;
  }
}
