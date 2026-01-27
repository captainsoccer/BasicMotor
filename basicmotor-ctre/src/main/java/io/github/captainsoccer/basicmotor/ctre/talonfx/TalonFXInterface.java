package io.github.captainsoccer.basicmotor.ctre.talonfx;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.MotorInterface;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;

/**
 * A motor interface for the TalonFX motor controller.
 * This class handles the configuration and communication with the TalonFX motor controller.
 */
public class TalonFXInterface extends MotorInterface {
    /**
     * The name of the default can bus chain.
     * This is the can bus chain built into the robo rio.
     */
    public static final CANBus defaultCanBusName = CANBus.roboRIO();

    /** The TalonFX motor controller. */
    public final TalonFX motor;

    /** The configuration for the TalonFX motor controller. */
    public final TalonFXConfiguration config;

    /** The sensors for the TalonFX motor controller. */
    public final TalonFXSensors sensors;

    /** The default measurements for the TalonFX motor controller. */
    private final TalonFXMeasurements defaultMeasurements;

    /**
     * Creates a TalonFXInterface with the provided name, id, gear ratio, and unit conversion.
     * @param name the name of the motor
     * @param id the CAN ID of the motor
     * @param gearRatio the gear ratio of the motor
     * @param unitConversion the unit conversion factor for the motor
     */
    public TalonFXInterface(String name, int id, double gearRatio, double unitConversion) {
        super(name);

        motor = new TalonFX(id);
        config = new TalonFXConfiguration();

        sensors = new TalonFXSensors(motor);

        applyConfig();

        defaultMeasurements = new TalonFXMeasurements(motor, gearRatio, unitConversion);

        motor.optimizeBusUtilization();
    }

    /**
     * Creates a TalonFXInterface with the provided configuration.
     * If the config is not a {@link BasicTalonFXConfig}, it will use the default can bus name.
     * @param config the configuration for the motor
     */
    public TalonFXInterface(BasicMotorConfig config) {
        super(config);

        final CANBus canBus;
        if (config instanceof BasicTalonFXConfig talonConfig)
            canBus = talonConfig.canBus;
        else
            canBus = defaultCanBusName;

        motor = new TalonFX(config.motorConfig.id, canBus);
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
    public void updatePIDGainsToMotor(PIDGains pidGains, int slot) {

        if(config.Feedback.FeedbackSensorSource != FeedbackSensorSourceValue.RotorSensor)
            pidGains = unConvertPIDgains(pidGains);

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


        // changes made in phoenix 6 api
        // https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/feature-replacements-guide.html#integral-zone-and-max-integral-accumulator

        if (pidGains.getI_MaxAccum() != MotorManager.getConfig().DEFAULT_MAX_OUTPUT)
            errorHandler.logAndReportWarning("TalonFX does not use i max accum therefore not used (check phoenix 6 docs)");

        if (pidGains.getTolerance() != 0)
            errorHandler.logAndReportWarning("TalonFX does not use tolerance therefore not used (check phoenix 6 docs)");

        if (pidGains.getI_Zone() != 0)
            errorHandler.logAndReportWarning("TalonFX does not use i zone therefore not used (check phoenix 6 docs)");


        applyConfig();
    }

    /**
     * Returns the pid gains to be in the real units (not motor units)
     * because if using a remote canCoder it will be the real units (as the canCoder reads real and not motor)
     * @param pidGains the motor units pid gains
     * @return the real units pid gains
     */
    private PIDGains unConvertPIDgains(PIDGains pidGains) {
        return pidGains.convertToMotorGains(1 / defaultMeasurements.getGearRatio(),
                1 /  defaultMeasurements.getUnitConversion());
    }

    @Override
    public void updateConstraintsGainsToMotor(ConstraintsGains constraints) {

        if(config.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.RemoteCANcoder)
            constraints = unConvertConstraintsGains(constraints);

// sets the max voltage to the max motor output
        config.Voltage.PeakForwardVoltage = constraints.getMaxMotorOutput();
        config.Voltage.PeakReverseVoltage = constraints.getMinMotorOutput();

        // sets the max duty cycle to the max motor output (same as voltage)
        config.MotorOutput.PeakForwardDutyCycle =
                constraints.getMaxMotorOutput() / MotorManager.getConfig().DEFAULT_IDEAL_VOLTAGE;
        config.MotorOutput.PeakReverseDutyCycle =
                constraints.getMinMotorOutput() / MotorManager.getConfig().DEFAULT_IDEAL_VOLTAGE;

        // sets the voltage deadband to the voltage deadband
        config.MotorOutput.DutyCycleNeutralDeadband =
                constraints.getVoltageDeadband() / MotorManager.getConfig().DEFAULT_IDEAL_VOLTAGE;

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
     * Returns the constraint gains to be in the real units (not motor units)
     * because if using a remote canCoder it will be the real units (as the canCoder reads real and not motor)
     * @param constraints the motor units constraints
     * @return the real units constraints
     */
    private ConstraintsGains unConvertConstraintsGains(ConstraintsGains constraints) {
        return constraints.convertToMotorConstraints(1 / defaultMeasurements.getGearRatio(),
                1 / defaultMeasurements.getUnitConversion());
    }

    /**
     * Applies the configuration to the motor controller.
     * If the configuration fails to apply, it will report an error to the driver station.
     */
    public void applyConfig() {
        var error = motor.getConfigurator().apply(config);

        if (error != StatusCode.OK) {
            errorHandler.logAndReportError("Failed to apply config to motor: " + super.name + " Error: " + error.name());
        }
    }
}