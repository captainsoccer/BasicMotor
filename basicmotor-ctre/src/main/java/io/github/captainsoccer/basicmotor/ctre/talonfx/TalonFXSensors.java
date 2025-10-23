package io.github.captainsoccer.basicmotor.ctre.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import io.github.captainsoccer.basicmotor.LogFrame;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;

import java.util.Arrays;
import java.util.stream.Stream;

/**
 * This class is used to manage the sensors of a TalonFX motor controller.
 * It handles updating the sensor data at a specified refresh rate
 * and provides methods to retrieve the sensor data.
 */
public class TalonFXSensors {
    /**
     * The multiplier for the timeout when using waitForAll.
     * If the user is using waitForAll (pro with canivore), this will be the multiplier for the timeout time.
     * This will be multiplied by the refresh rate to determine the timeout time.
     * Change this only if you know what you are doing.
     */
    public static int TIMEOUT_REFRESH_MULTIPLIER = 4;

    /**
     * The timeout for waiting for all signals to update.
     * Used only when Pro features are enabled.
     * This is the refresh rate divided by {@link #TIMEOUT_REFRESH_MULTIPLIER}.
     */
    private final double timeout;

    /**
     * The location of the pid controller of the motor.
     * This is used to determine which signals to update.
     */
    private MotorManager.ControllerLocation location = MotorManager.ControllerLocation.MOTOR;

    /**
     * The status signal containing the temperature
     */
    private final StatusSignal<Temperature> temperatureSignal;
    /**
     * The current draw of the motor controller
     */
    private final StatusSignal<Current> supplyCurrentSignal;
    /**
     * The current output of the motor controller
     */
    private final StatusSignal<Current> statorCurrentSignal;
    /**
     * The voltage output of the motor controller
     */
    private final StatusSignal<Voltage> motorVoltageSignal;
    /**
     * The voltage input of the motor controller
     */
    private final StatusSignal<Voltage> supplyVoltageSignal;
    /**
     * The duty cycle of the motor controller
     */
    private final StatusSignal<Double> dutyCycleSignal;

    /**
     * The total output of the pid controller
     */
    private final StatusSignal<Double> totalOutput;
    /**
     * The P output of the pid controller
     */
    private final StatusSignal<Double> kpOutput;
    /**
     * The I output of the pid controller
     */
    private final StatusSignal<Double> kiOutput;
    /**
     * The D output of the pid controller
     */
    private final StatusSignal<Double> kdOutput;

    /**
     * The collection of the sensor status signals.
     * These signals are always used and updated regardless of the controller location.
     */
    private final BaseStatusSignal[] sensorsSignals;

    /**
     * The collection of the PID status signals.
     * These signals are only used if the controller is on the motor controller.
     * Otherwise, they are not used and will not be updated.
     */
    private final BaseStatusSignal[] pidSignals;

    /**
     * The collection of status signals that will be updated at the refresh rate.
     */
    private BaseStatusSignal[] allSignals;

    /**
     * The latest PID output from the motor controller.
     * This is used for logging purposes when the controller is on the motor controller.
     */
    private LogFrame.PIDOutput latestPIDOutput = LogFrame.PIDOutput.EMPTY;

    /**
     * Whether to wait for all status signals to update before returning the sensor data.
     * This requires Pro features to be enabled and is only used if the controller is on a CANivore.
     */
    private boolean waitForAll = false;

    /**
     * Constructs a TalonFXSensors object with the given motor and refresh rate.
     *
     * @param motor    The TalonFX motor controller to get the sensors from.
     */
    public TalonFXSensors(TalonFX motor) {
        double refreshHZ = MotorManager.config.SENSOR_LOOP_HZ;

        timeout = 1 / (refreshHZ * TIMEOUT_REFRESH_MULTIPLIER);

        temperatureSignal = motor.getDeviceTemp(false);
        supplyCurrentSignal = motor.getSupplyCurrent(false);
        statorCurrentSignal = motor.getStatorCurrent(false);
        motorVoltageSignal = motor.getMotorVoltage(false);
        supplyVoltageSignal = motor.getSupplyVoltage(false);
        dutyCycleSignal = motor.getDutyCycle(false);

        sensorsSignals = new BaseStatusSignal[]{
                temperatureSignal,
                supplyCurrentSignal,
                statorCurrentSignal,
                motorVoltageSignal,
                supplyVoltageSignal,
                dutyCycleSignal
        };

        kpOutput = motor.getClosedLoopProportionalOutput(false);
        kiOutput = motor.getClosedLoopIntegratedOutput(false);
        kdOutput = motor.getClosedLoopDerivativeOutput(false);
        totalOutput = motor.getClosedLoopOutput(false);

        pidSignals = new BaseStatusSignal[]{
                totalOutput,
                kpOutput,
                kiOutput,
                kdOutput
        };

        updateControllerLocation(MotorManager.ControllerLocation.MOTOR);
    }

    /**
     * Gets the sensor data from the motor controller.
     * This refreshes the status signals based on the configured refresh rate.
     *
     * @return The sensor data.
     */
    public LogFrame.SensorData getSensorData() {
        if (waitForAll) BaseStatusSignal.waitForAll(timeout, allSignals);
        else BaseStatusSignal.refreshAll(allSignals);

        double temperature = temperatureSignal.getValueAsDouble();
        double currentDraw = supplyCurrentSignal.getValueAsDouble();
        double currentOutput = statorCurrentSignal.getValueAsDouble();
        double voltageOutput = motorVoltageSignal.getValueAsDouble();
        double voltageInput = supplyVoltageSignal.getValueAsDouble();
        double powerDraw = currentDraw * voltageInput;
        double powerOutput = currentOutput * voltageOutput;
        double dutyCycle = dutyCycleSignal.getValueAsDouble();

        // updates the latest pid output if the controller is on the motor controller
        // used for logging
        if (location == MotorManager.ControllerLocation.MOTOR) {
            double pOutput = kpOutput.getValueAsDouble();
            double iOutput = kiOutput.getValueAsDouble();
            double dOutput = kdOutput.getValueAsDouble();
            double pidOutput = totalOutput.getValueAsDouble();

            latestPIDOutput = new LogFrame.PIDOutput(pOutput, iOutput, dOutput, pidOutput);
        }

        return new LogFrame.SensorData(
                temperature,
                currentDraw,
                currentOutput,
                voltageOutput,
                voltageInput,
                powerDraw,
                powerOutput,
                dutyCycle);
    }

    /**
     * Sets the update frequency of the duty cycle signal.
     * This is used when the motor is followed by another motor and needs a fast duty cycle update.
     *
     * @param defaultRate Whether to set the duty cycle to the default rate (100 Hz) or to the refresh rate.
     */
    public void setDutyCycleToDefaultRate(boolean defaultRate) {
        dutyCycleSignal.setUpdateFrequency(defaultRate ? 100 : MotorManager.config.SENSOR_LOOP_HZ);
    }

    /**
     * Updates the controller location for the sensors.
     * If the controller is on the motor controller, it updates all signals.
     * If the controller is on the RIO, it only updates the sensor signals
     *
     * @param location The location of the controller (MOTOR or RIO).
     */
    public void updateControllerLocation(MotorManager.ControllerLocation location) {
        this.location = location;
        if (location == MotorManager.ControllerLocation.MOTOR) {
            allSignals = Stream.concat(Arrays.stream(sensorsSignals), Arrays.stream(pidSignals))
                    .toArray(BaseStatusSignal[]::new);
        } else {
            allSignals = sensorsSignals;
            latestPIDOutput = LogFrame.PIDOutput.EMPTY;

            for (BaseStatusSignal signal : pidSignals) {
                signal.setUpdateFrequency(0);
            }
        }

        for(BaseStatusSignal signal : allSignals) {
            signal.setUpdateFrequency(MotorManager.config.SENSOR_LOOP_HZ);
        }
    }

    /**
     * Sets whether to enable waiting for all signals to update before returning values.
     * This is only used if the controller is on a CANivore and Pro features are enabled.
     * If enabled without a CANivore, it will slow down the system significantly
     *
     * @param enable True to enable waiting for all signals to update before returning values,
     */
    public void setWaitForAll(boolean enable) {
        this.waitForAll = enable;
    }

    /**
     * Gets the latest PID output from the motor controller.
     *
     * @return The latest PID output.
     */
    public LogFrame.PIDOutput getPIDLatestOutput() {
        return latestPIDOutput;
    }
}
