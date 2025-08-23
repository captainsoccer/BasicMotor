package io.github.captainsoccer.basicmotor.ctre;

import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.LogFrame;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.controllers.Controller;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.gains.ControllerGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;
import io.github.captainsoccer.basicmotor.gains.CurrentLimits;
import io.github.captainsoccer.basicmotor.measurements.EmptyMeasurements;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A basic motor implementation for the brushed Victor SPX motor controller.
 * This class has limitation due to the fact that the victorSPX does not have a built-in encoder
 * If you want full functionality, provide a measurements object.
 */
public class BasicVictorSPX extends BasicMotor {
    /**
     * The VictorSPX motor controller instance used by this BasicVictorSPX.
     */
    private final VictorSPX motor;

    /**
     * The default measurements for the VictorSPX motor controller.
     */
    private final Measurements defaultMeasurements;

    /**
     * Creates a BasicVictorSPX instance with the provided motor ID and name.
     * Since no measurements are provided, it will use the default empty measurements.
     * That means that any closed loop control will not work.
     * You can add a measurements object later using the {@link #setMeasurements(Measurements)} method.
     * @param id The ID of the VictorSPX motor controller
     * @param name The name of the motor controller
     * @see #BasicVictorSPX(int, String, Measurements, ControllerGains)
     */
    public BasicVictorSPX(int id, String name) {
        super(new ControllerGains(), name);

        motor = new VictorSPX(id);
        motor.configFactoryDefault();
        motor.configVoltageCompSaturation(MotorManager.config.motorIdealVoltage);

        defaultMeasurements = new EmptyMeasurements();
    }

    /**
     * Creates a BasicVictorSPX instance with the provided motor ID, name, measurements, and controller gains.
     * If the measurements are null, it will use the default empty measurements.
     * The pid controller location is always set to RIO, as the victorSPX does not support PID gains directly.
     * @param id The ID of the VictorSPX motor controller
     * @param name The name of the motor controller
     * @param measurements The measurements to use for the motor controller
     * @param controllerGains The controller gains to use for the motor controller
     */
    public BasicVictorSPX(int id, String name, Measurements measurements, ControllerGains controllerGains) {
        super(controllerGains, name);

        motor = new VictorSPX(id);
        motor.configFactoryDefault();

        motor.configVoltageCompSaturation(MotorManager.config.motorIdealVoltage);

        if(measurements != null) {
            defaultMeasurements = measurements;
            setControllerLocation(MotorManager.ControllerLocation.RIO);
        }
        else{
            defaultMeasurements = new EmptyMeasurements();
            DriverStation.reportWarning("provided measurements for motor: " + name + " is null", false);
        }
    }

    /**
     * Creates a BasicVictorSPX instance with the provided configuration and measurements.
     * This also sets the controller location to RIO, as the victorSPX does not support PID gains directly.
     * @param config The configuration for the motor controller
     * @param measurements The measurements to use for the motor controller
     */
    public BasicVictorSPX(BasicMotorConfig config, Measurements measurements) {
        super(config);

        motor = new VictorSPX(config.motorConfig.id);
        motor.configFactoryDefault();

        motor.configVoltageCompSaturation(MotorManager.config.motorIdealVoltage);

        if(measurements != null) {
            defaultMeasurements = measurements;
            setControllerLocation(MotorManager.ControllerLocation.RIO);
        }
        else{
            defaultMeasurements = new EmptyMeasurements();
            DriverStation.reportWarning("provided measurements for motor: " + name + " is null", false);
        }
    }

    @Override
    protected void updatePIDGainsToMotor(PIDGains pidGains, int slot) {
        // Does nothing, as the victorSPX does not support PID gains directly.
    }

    @Override
    protected void updateConstraintsGainsToMotor(ConstraintsGains constraints) {
        // Does nothing with soft limits.
        motor.configPeakOutputForward(constraints.getMaxMotorOutput() / MotorManager.config.motorIdealVoltage);
        motor.configPeakOutputReverse(-constraints.getMaxMotorOutput() / MotorManager.config.motorIdealVoltage);

        motor.configNominalOutputForward(constraints.getVoltageDeadband() / MotorManager.config.motorIdealVoltage);
        motor.configNominalOutputReverse(-constraints.getVoltageDeadband() / MotorManager.config.motorIdealVoltage);
    }

    @Override
    protected Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }

    @Override
    public void setCurrentLimits(CurrentLimits currentLimits) {
        DriverStation.reportWarning("motor: " + this.name + " does not support current limits.", false);
    }

    @Override
    public void setIdleMode(IdleMode mode) {
         NeutralMode idleMode = switch (mode) {
            case COAST -> NeutralMode.Coast;
            case BRAKE -> NeutralMode.Brake;
        };

        motor.setNeutralMode(idleMode);
    }

    @Override
    public void setMotorInverted(boolean inverted) {
        motor.setInverted(inverted);
    }

    @Override
    protected void stopRecordingMeasurements() {
        //Does nothing, as the victorSPX does not support recording measurements.
    }

    @Override
    protected void startRecordingMeasurements(double HZ) {
        // Does nothing, as the victorSPX does not support recording measurements.
    }

    @Override
    protected void updateMainLoopTiming(MotorManager.ControllerLocation location) {
        //Does nothing, as the victorSPX does not support main loop timing.
        if(location == MotorManager.ControllerLocation.MOTOR){
            DriverStation.reportError("motor: " + this.name + " does not support closed loop control on motor", true);
        }
    }

    @Override
    protected double getInternalPIDLoopTime() {
        return 0.001; // TalonSRX has a fixed internal loop time of 1ms
        // Also doesn't matter as the victorSPX does not support PID gains directly.
        //According to a chief delphi post:
        // https://www.chiefdelphi.com/t/control-loop-timing-of-various-motor-controllers/370356/4
    }

    @Override
    protected void setMotorFollow(BasicMotor master, boolean inverted) {
        BasicVictorSPX motor = (BasicVictorSPX) master;

        boolean masterInverted = motor.motor.getInverted();

        this.motor.setInverted(inverted != masterInverted);

        this.motor.follow(motor.motor);
    }

    @Override
    protected void stopMotorFollow() {
        stopMotorOutput();
    }

    @Override
    protected void setMotorOutput(double setpoint, double feedForward, Controller.ControlMode mode) {
        // Because victorSPX does not have a built-in encoder and does not have a connection for an external encoder,
        // it cannot support direct PID control.
        // but it can support percent output, voltage, current, and torque control modes.
        if(mode.requiresPID()){
            DriverStation.reportError("motor: " + this.name + " does not support direct PID control.", true);
        }

        switch (mode) {
            case PERCENT_OUTPUT -> motor.set(VictorSPXControlMode.PercentOutput, setpoint);

            case VOLTAGE ->  motor.set(VictorSPXControlMode.PercentOutput, setpoint / MotorManager.config.motorIdealVoltage);

            case STOP -> stopMotorOutput();
        }
    }

    @Override
    protected void stopMotorOutput() {
        this.motor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    protected LogFrame.SensorData getLatestSensorData() {
        //VictorSPX does not support current sensing.

        double busVoltage = motor.getBusVoltage();
        double outputVoltage = motor.getMotorOutputVoltage();

        double temperature = motor.getTemperature();
        double dutyCycle = motor.getMotorOutputPercent();

        return new LogFrame.SensorData(
                temperature,
                0,
                0,
                outputVoltage,
                busVoltage,
                0,
                0,
                dutyCycle
        );
    }

    @Override
    protected LogFrame.PIDOutput getPIDLatestOutput() {
        return LogFrame.PIDOutput.EMPTY;
    }
}
