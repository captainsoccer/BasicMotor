package io.github.captainsoccer.basicmotor.ctre.talonfx;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.MotorInterface;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;

public class TalonFXInterface extends MotorInterface {
    /**
     * The name of the default can bus chain.
     * This is the can bus chain built into the robo rio.
     */
    public static final String defaultCanBusName = "rio";

    public final TalonFX motor;

    public final TalonFXConfiguration config;

    public final TalonFXSensors sensors;

    private final TalonFXMeasurements defaultMeasurements;

    public TalonFXInterface(String name, int id, double gearRatio, double unitConversion) {
        super(name);

        motor = new TalonFX(id);
        config = new TalonFXConfiguration();

        sensors = new TalonFXSensors(motor);

        applyConfig();

        defaultMeasurements = new TalonFXMeasurements(motor, gearRatio, unitConversion);

        motor.optimizeBusUtilization();
    }

    public TalonFXInterface(BasicMotorConfig config){
        super(config);

        final String canBusName;
        if(config instanceof BasicTalonFXConfig talonConfig)
            canBusName = talonConfig.canBusName;
        else
            canBusName = defaultCanBusName;

        motor = new TalonFX(config.motorConfig.id, canBusName);
        this.config = new TalonFXConfiguration();

        applyConfig();

        defaultMeasurements = new TalonFXMeasurements(motor, config.motorConfig.gearRatio, config.motorConfig.unitConversion);

        sensors = new TalonFXSensors(motor);

        motor.optimizeBusUtilization();
    }

    @Override
    public TalonFXMeasurements getDefaultMeasurements() {
        return defaultMeasurements;
    }

    @Override
    public double getInternalPIDLoopTime() {
        return 0.001; // TalonFX has a fixed internal PID loop time of 1ms
        // This is according to https://www.chiefdelphi.com/t/control-loop-timing-of-various-motor-controllers/370356/4
    }

    @Override
    public void setInverted(boolean inverted) {
        config.MotorOutput.Inverted =
                inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        applyConfig();
    }

    @Override
    public void setIdleMode(BasicMotor.IdleMode mode) {
        config.MotorOutput.NeutralMode =
                switch (mode) {
                    case COAST -> NeutralModeValue.Coast;
                    case BRAKE -> NeutralModeValue.Brake;
                };

        applyConfig();
    }

    @Override
    public void updatePIDGainsToMotor(PIDGains pidGains, int slot, MotorManager.ControllerLocation location) {
// Thanks ctre for making them different types of configs for each slot
        switch (slot) {
            case 0 -> {
                config.Slot0.kP = pidGains.getK_P();
                config.Slot0.kI = pidGains.getK_I();
                config.Slot0.kD = pidGains.getK_D();
            }

            case 1 -> {
                config.Slot1.kP = pidGains.getK_P();
                config.Slot1.kI = pidGains.getK_I();
                config.Slot1.kD = pidGains.getK_D();
            }

            case 2 -> {
                config.Slot2.kP = pidGains.getK_P();
                config.Slot2.kI = pidGains.getK_I();
                config.Slot2.kD = pidGains.getK_D();
            }
        }

        if (location == MotorManager.ControllerLocation.MOTOR) {
            // changes made in phoenix 6 api
            // https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/feature-replacements-guide.html#integral-zone-and-max-integral-accumulator

            if (pidGains.getI_MaxAccum() != Double.POSITIVE_INFINITY)
                DriverStation.reportWarning(
                        name
                                + " does not need i max accum when running on motor therefor not used (TalonFX check phoenix 6 docs)",
                        false);

            if (pidGains.getTolerance() != 0)
                DriverStation.reportWarning(
                        name
                                + " does not need tolerance when running on motor therefor not used (TalonFX check phoenix 6 docs)",
                        false);

            if (pidGains.getI_Zone() != Double.POSITIVE_INFINITY)
                DriverStation.reportWarning(
                        name
                                + " does not need i zone when running on motor therefor not used (TalonFX check phoenix 6 docs)",
                        false);
        }

        applyConfig();
    }

    @Override
    public void updateConstraintsGainsToMotor(ConstraintsGains constraints) {
// sets the max voltage to the max motor output
        config.Voltage.PeakForwardVoltage = constraints.getMaxMotorOutput();
        config.Voltage.PeakReverseVoltage = constraints.getMinMotorOutput();

        // sets the max duty cycle to the max motor output (same as voltage)
        config.MotorOutput.PeakForwardDutyCycle =
                constraints.getMaxMotorOutput() / MotorManager.config.motorIdealVoltage;
        config.MotorOutput.PeakReverseDutyCycle =
                constraints.getMinMotorOutput() / MotorManager.config.motorIdealVoltage;

        // sets the voltage deadband to the voltage deadband
        config.MotorOutput.DutyCycleNeutralDeadband =
                constraints.getVoltageDeadband() / MotorManager.config.motorIdealVoltage;

        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = constraints.getRampRate();
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = constraints.getRampRate();
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = constraints.getRampRate();
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = constraints.getRampRate();

        // sets continuous wrap to false (it is calculated on the rio if needed)
        config.ClosedLoopGeneral.ContinuousWrap = false;

        // checks if it needs to apply soft limits
        if (constraints.getConstraintType() == ConstraintsGains.ConstraintType.LIMITED) {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constraints.getMaxValue();
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constraints.getMinValue();
        } else {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        }

        // applies the config to the motor
        applyConfig();
    }

    /**
     * Applies the configuration to the motor controller.
     * If the configuration fails to apply, it will report an error to the driver station.
     */
    public void applyConfig() {
        var error = motor.getConfigurator().apply(config);

        if (error != StatusCode.OK) {
            DriverStation.reportError(
                    "Failed to apply config to motor: " + super.name + " Error: " + error.name(), false);
        }
    }
}