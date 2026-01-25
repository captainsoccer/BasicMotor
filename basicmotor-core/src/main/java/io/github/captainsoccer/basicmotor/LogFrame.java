package io.github.captainsoccer.basicmotor;

import io.github.captainsoccer.basicmotor.controllers.Controller;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * This is the class that holds the logged data of the motor.
 * This will be sent to the logger and can be used to analyze the motor's performance.
 */
public class LogFrame {
    /**
     * The latest controller frame of the motor.
     * This contains the essential data of the controller,
     */
    public ControllerFrame controllerFrame = ControllerFrame.EMPTY;
    /**
     * The latest PID output of the controller.
     * This holds the output of the PID controller.
     * Useful for debugging the output of the PID controller.
     */
    public PIDOutput pidOutput = PIDOutput.EMPTY;
    /**
     * The latest sensor data of the motor.
     * This holds the data of the sensors of the motor.
     */
    public SensorData sensorData = SensorData.EMPTY;
    /**
     * The latest measurement of the motor.
     * This will be in the units that the user has set for the motor.
     */
    public Measurements.Measurement measurement = Measurements.Measurement.EMPTY;

    /**
     * If the motor is at setpoint.
     * This is only applicable when using a closed-loop controller.
     * This will be true if the error is within the tolerance of the controller.
     */
    public boolean atSetpoint = false;

    /**
     * If the motor is at goal.
     * This is only applicable when using a profile controller.
     * This will be true if the motor has reached the goal of the profile.
     * If the controller is not using a profile, this will always be the same as {@link #atSetpoint}.
     */
    public boolean atGoal = false;

    /**
     * The applied torque of the motor.
     * This is the torque that the motor is applying.
     * This is only calculated if using a motor configuration with the correct gear ratio and motor type.
     */
    public double appliedTorque = 0;

    //Helper classes

    /**
     * The record holding the sensor data of the motor
     * This is in a record due to multithreading and performance reasons.
     *
     * @param temperature   The temperature of the motor (in degrees Celsius)
     * @param currentDraw   The current draw of the motor (in amps).
     *                      This is how much current the motor is drawing from the battery.
     * @param currentOutput The current output of the motor (in amps).
     *                      This is how much current is in the motor coils.
     * @param voltageOutput The voltage output of the motor (in volts).
     *                      This is how much voltage is being applied to the motor coils.
     * @param voltageInput  The voltage input of the motor (in volts).
     *                      This will be the battery voltage available to the motor.
     * @param powerDraw     This is how much power the motor is drawing from the battery (in watts).
     *                      This is used to check how much energy the motor is consuming.
     * @param powerOutput   The power output of the motor (in watts).
     *                      This is used to check how much energy loss the motor has.
     * @param dutyCycle     The duty cycle of the motor (in -1 to 1 range).
     *                      This is the ratio of the voltage output to the voltage input.
     */
    public record SensorData(
            double temperature,
            double currentDraw,
            double currentOutput,
            double voltageOutput,
            double voltageInput,
            double powerDraw,
            double powerOutput,
            double dutyCycle) {

        /**
         * An empty sensor data used when no sensors are available or when the motor is not initialized.
         */
        public static final SensorData EMPTY = new SensorData(0, 0, 0, 0, 0, 0, 0, 0);
    }

    /**
     * The record holding the PID output of the controller.
     * This is in a record due to multithreading and performance reasons.
     * All the units are in volts.
     *
     * @param pOutput     The P output of the controller
     * @param iOutput     The I output of the controller
     * @param dOutput     The D output of the controller
     * @param totalOutput The total output of the controller (the sum of all outputs)
     */
    public record PIDOutput(double pOutput, double iOutput, double dOutput, double totalOutput) {
        /**
         * An empty PID output used when no PID data is available.
         */
        public static final PIDOutput EMPTY = new PIDOutput(0, 0, 0, 0);
    }

    /**
     * The record holding the feedforward output of the controller.
     * This is in a record due to multithreading and performance reasons.
     * all the units are in volts.
     *
     * @param kG                    The gravity feedforward output of the controller
     * @param frictionFeedForward   The friction feedforward output of the controller.
     * @param kV                    The velocity feedforward output of the controller.
     * @param kA                    The acceleration feedforward output of the controller.
     * @param calculatedFeedForward The feedforward calculated from the feedforward function.
     * @param arbitraryFeedForward  The feedforward set by the user with the control request.
     * @param totalOutput           The total output of the feedforward (the sum of all feedforward outputs).
     */
    public record FeedForwardOutput(
            double kG,
            double frictionFeedForward,
            double kV,
            double kA,
            double calculatedFeedForward,
            double arbitraryFeedForward,
            double totalOutput) {

        /**
         * An empty feedforward output used when no feedforward data is available.
         */
        public static final FeedForwardOutput EMPTY = new FeedForwardOutput(0, 0, 0, 0, 0, 0);

        /**
         * The record holding the feedforward output of the controller.
         * This is in a record due to multithreading and performance reasons.
         * all the units are in volts.
         * This will calculate the total output of the feedforward
         *
         * @param kG                    The gravity feedforward output of the controller
         * @param frictionFeedForward   The friction feedforward output of the controller.
         * @param kV                    The velocity feedforward output of the controller.
         * @param kA                    The acceleration feedforward output of the controller.
         * @param calculatedFeedForward The feedforward calculated from the feedforward function.
         * @param arbitraryFeedForward  The feedforward set by the user with the control request.
         */
        public FeedForwardOutput(
                double kG,
                double frictionFeedForward,
                double kV,
                double kA,
                double calculatedFeedForward,
                double arbitraryFeedForward) {
            this(
                    kG,
                    frictionFeedForward,
                    kV,
                    kA,
                    calculatedFeedForward,
                    arbitraryFeedForward,

                    kG
                            + frictionFeedForward
                            + kV
                            + kA
                            + calculatedFeedForward
                            + arbitraryFeedForward);
        }
    }

    /**
     * The record holding the controller frame of the motor.
     *
     * @param totalOutput       The total output of the controller (in volts).
     * @param feedForwardOutput The feedforward output of the controller.
     * @param setpoint          The setpoint of the controller in units of control.
     * @param measurement       The measurement used to calculate the error of the controller in units of control.
     * @param error             The error of the controller in units of control.
     * @param goal              The goal of the controller in units of control.
     *                          This is the end goal of the motion profile.
     *                          If the controller is not using a motion profile, this will be the same as the setpoint.
     * @param mode              The control mode of the controller.
     */
    public record ControllerFrame(
            double totalOutput,
            FeedForwardOutput feedForwardOutput,
            double setpoint,
            double measurement,
            double error,
            double goal,
            Controller.ControlMode mode) {

        /**
         * An empty controller frame used when no controller data is available or when the motor is stopped.
         */
        public static final ControllerFrame EMPTY =
                new ControllerFrame(0, FeedForwardOutput.EMPTY, 0, 0, 0, 0, Controller.ControlMode.STOP);

        /**
         * The record holding the controller frame of the motor.
         * This is used when using an open loop controller.
         *
         * @param outputVolts The output in volts.
         * @param output      The output in units of control.
         * @param mode        The control mode of the controller.
         */
        public ControllerFrame(double outputVolts, double output, Controller.ControlMode mode) {
            this(outputVolts, FeedForwardOutput.EMPTY, output, 0, 0, 0, mode);
        }
    }

    /**
     * The logged version of the LogFrame.
     * This is sent to the logger.
     * This was automatically generated to implement the LoggableInputs interface.
     */
    public static class LogFrameAutoLogged extends LogFrame implements LoggableInputs, Cloneable {
        @Override
        public void toLog(LogTable table) {
            table.put("ControllerFrame", controllerFrame);
            table.put("PidOutput", pidOutput);
            table.put("SensorData", sensorData);
            table.put("Measurement", measurement);
            table.put("AtSetpoint", atSetpoint);
            table.put("AtGoal", atGoal);
            table.put("AppliedTorque", appliedTorque);
        }

        @Override
        public void fromLog(LogTable table) {
            controllerFrame = table.get("ControllerFrame", controllerFrame);
            pidOutput = table.get("PidOutput", pidOutput);
            sensorData = table.get("SensorData", sensorData);
            measurement = table.get("Measurement", measurement);
            atSetpoint = table.get("AtSetpoint", atSetpoint);
            atGoal = table.get("AtGoal", atGoal);
            appliedTorque = table.get("AppliedTorque", appliedTorque);
        }

        @SuppressWarnings("MethodDoesntCallSuperMethod")
        @Override
        public LogFrameAutoLogged clone() {
            LogFrameAutoLogged copy = new LogFrameAutoLogged();
            copy.controllerFrame = this.controllerFrame;
            copy.pidOutput = this.pidOutput;
            copy.sensorData = this.sensorData;
            copy.measurement = this.measurement;
            copy.atSetpoint = this.atSetpoint;
            copy.atGoal = this.atGoal;
            copy.appliedTorque = this.appliedTorque;
            return copy;
        }
    }
}
