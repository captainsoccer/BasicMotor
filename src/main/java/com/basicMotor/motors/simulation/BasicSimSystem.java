package com.basicMotor.motors.simulation;

import com.basicMotor.BasicMotor;
import com.basicMotor.configuration.BasicMotorConfig;
import com.basicMotor.controllers.Controller;
import com.basicMotor.gains.ControllerConstraints;
import com.basicMotor.gains.ControllerGains;
import com.basicMotor.gains.currentLimits.CurrentLimits;
import com.basicMotor.gains.PIDGains;
import com.basicMotor.LogFrame;
import com.basicMotor.motorManager.MotorManager.ControllerLocation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

/**
 * This is an abstract class that represents a basic simulation system for motors.
 * This ignores some of the functionality of a basic motor, as it is meant to be used in a simulation.
 * Use specific mechanisms like {@link BasicSimElevator} or {@link BasicSimArm} when possible.
 */
public abstract class BasicSimSystem extends BasicMotor {
  /** The voltage output of the motor */
  private double voltageOutput = 0.0;

  /**
   * Creates a BasicSimSystem instance with the provided name and controller gains.
   *
   * @param name The name of the motor simulation
   * @param gains The controller gains to use for the motor simulation
   */
  public BasicSimSystem(String name, ControllerGains gains) {
    super(gains, name, ControllerLocation.RIO);
  }

  /**
   * Creates a BasicSimSystem instance with the provided configuration.
   * forces controller location to RIO, as simulation systems are always on the RoboRIO.
   *
   * @param config The configuration for the motor simulation
   */
  public BasicSimSystem(BasicMotorConfig config) {
    super(checkConfig(config));
  }

  /**
   * Forces the controller location to RIO in the provided configuration.
   */
  private static BasicMotorConfig checkConfig(BasicMotorConfig config) {
    config.motorConfig.location = ControllerLocation.RIO;
    return config;
  }

  @Override
  protected void updatePIDGainsToMotor(PIDGains pidGains) {
    // does nothing, as this is a simulation system
  }

  @Override
  protected void updateConstraints(ControllerConstraints constraints) {
    // does nothing, as this is a simulation system
  }

  @Override
  public void setCurrentLimits(CurrentLimits currentLimits) {
    // does nothing, as this is a simulation system
  }

  @Override
  public void setIdleMode(IdleMode mode) {
    // does nothing, as this is a simulation system
  }

  @Override
  public void setMotorInverted(boolean inverted) {
    // does nothing, as this is a simulation system
  }

  @Override
  protected void stopRecordingMeasurements() {
    // does nothing, as this is a simulation system
  }

  @Override
  protected void startRecordingMeasurements(double HZ) {
    // does nothing, as this is a simulation system
  }

  @Override
  protected void setMotorFollow(BasicMotor master, boolean inverted) {
    // does nothing, as this is a simulation system
  }

  @Override
  protected void stopMotorFollow() {
    // does nothing, as this is a simulation system
  }

  @Override
  protected void updateMainLoopTiming(ControllerLocation location){
    // does nothing, as this is a simulation system
    // the main loop timing is always on the RIO for simulation systems
  }

  @Override
  public void setControllerLocation(ControllerLocation location){
    throw new UnsupportedOperationException("Can't change controller location for simulation systems.");
  }

  @Override
  protected double getInternalPIDLoopTime(){
    return ControllerLocation.RIO.getSeconds();
    // All sim motors run their PID loop on the RoboRIO, so this is always the RIO loop time.
  }

  @Override
  protected void setMotorOutput(double setpoint, double feedForward, Controller.ControlMode mode) {
    if (mode.requiresPID()){
      DriverStation.reportError("Simulation systems do not support direct PID control.", true);
      return;
    }

    double output =
        switch (mode) {
          case STOP -> 0;
          //converts duty cycle to voltage output
          case PRECENT_OUTPUT -> setpoint * RobotController.getBatteryVoltage();
          default -> setpoint;
        };

    setOutput(output);
  }

  /**
   * Sets the output voltage of the motor simulation.
   *
   * @param output The output voltage to set, in volts.
   */
  private void setOutput(double output) {
    voltageOutput = output;
    setInputVoltage(voltageOutput);
  }

  @Override
  protected void stopMotorOutput() {
    setOutput(0);
  }

  /**
   * Sets the input voltage for the motor simulation.
   * This directly sets the voltage output of the motor simulation.
   *
   * @param voltage The voltage to set, in volts.
   */
  protected abstract void setInputVoltage(double voltage);

  @Override
  protected LogFrame.SensorData getLatestSensorData() {
    double voltageInput = RobotController.getBatteryVoltage();
    double voltageOutput = this.voltageOutput;

    double currentDraw = getCurrentDraw();

    double powerDraw =
        voltageInput * currentDraw; // also power output because this is a simulation system

    double currentOutput = powerDraw / voltageOutput;

    double temp = 0; // no temperature in simulation

    double dutyCycle =
        voltageOutput / voltageInput; // duty cycle is the ratio of output to input voltage

    return new LogFrame.SensorData(
        temp,
        currentDraw,
        currentOutput,
        voltageOutput,
        voltageInput,
        powerDraw,
        powerDraw,
        dutyCycle);
  }

  /**
   * Gets the current draw of the motor simulation.
   * This method should be overridden by subclasses to provide the specific current draw logic.
   * Some simulations calculate current draw based on the input voltage and other factors,
   *
   * @return The current draw of the motor simulation in amps.
   */
  protected abstract double getCurrentDraw();

  @Override
  protected LogFrame.PIDOutput getPIDLatestOutput() {
    //this will never be called in a simulation system, as it does not support PID control
    return LogFrame.PIDOutput.EMPTY;
  }
}
