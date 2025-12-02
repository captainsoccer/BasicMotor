package io.github.captainsoccer.basicmotor.ctre.talonsrx;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.MotorInterface;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;
import io.github.captainsoccer.basicmotor.measurements.EmptyMeasurements;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;

/**
 * A motor interface for the TalonSRX motor controller.
 * This class handles the configuration and communication with the TalonSRX motor controller.
 */
public class TalonSRXInterface extends MotorInterface {
    /** The TalonSRX motor controller. */
    public final TalonSRX motor;

    /** The configuration for the TalonSRX motor controller. */
    public final TalonSRXConfiguration config = new TalonSRXConfiguration();

    /** The PID gains for the TalonSRX motor controller. */
    public PIDGains motorGains = new PIDGains();

    /** The default measurements for the TalonSRX motor controller. */
    private final Measurements defaultMeasurements;

    /**
     * Creates a TalonSRXInterface with the provided motor ID and name.
     * @param id The ID of the TalonSRX motor controller
     * @param name The name of the motor controller
     */
    public TalonSRXInterface(int id, String name) {
        super(name);

        motor = new TalonSRX(id);
        this.motor.configFactoryDefault();
        config.voltageCompSaturation = MotorManager.getConfig().DEFAULT_IDEAL_VOLTAGE;

        applyConfig();

        defaultMeasurements = new EmptyMeasurements();
    }

    /**
     * Creates a BasicTalonSRX instance with the provided motor ID, name, controller gains, controller location, encoder type, and ticks per revolution.
     * This constructor is used to create a TalonSRX motor controller with the specified settings
     * @param id The ID of the TalonSRX motor controller
     * @param name The name of the motor controller
     * @param encoderType The type of encoder that is connected to the TalonSRX motor controller.
     * @param tickPerRevolution The number of ticks per revolution of the encoder.
     */
    public TalonSRXInterface(int id, String name, BasicTalonSRX.EncoderType encoderType, int tickPerRevolution) {
        super(name);

        this.motor = new TalonSRX(id);
        this.motor.configFactoryDefault();
        this.config.voltageCompSaturation = MotorManager.getConfig().DEFAULT_IDEAL_VOLTAGE;

        this.config.primaryPID.selectedFeedbackSensor = encoderType.feedbackDevice;
        this.config.primaryPID.selectedFeedbackCoefficient = 1.0 / tickPerRevolution;

        applyConfig();

        defaultMeasurements = new TalonSRXMeasurements(motor, tickPerRevolution);
    }

    /**
     * Creates a BasicTalonSRX instance with the provided configuration.
     * This constructor is used to create a TalonSRX motor controller with the specified settings.
     * @param motorConfig The configuration for the TalonSRX motor controller.
     */
    public TalonSRXInterface(BasicTalonSRXConfig motorConfig){
        super(motorConfig);

        this.motor = new TalonSRX(motorConfig.motorConfig.id);
        this.motor.configFactoryDefault();
        this.config.voltageCompSaturation = MotorManager.getConfig().DEFAULT_IDEAL_VOLTAGE;

        this.config.primaryPID.selectedFeedbackSensor = motorConfig.encoderConfig.type.feedbackDevice;
        this.config.primaryPID.selectedFeedbackCoefficient = 1.0 / motorConfig.encoderConfig.tickPerRevolution;

        applyConfig();

        defaultMeasurements = new TalonSRXMeasurements(motor, motorConfig.encoderConfig.tickPerRevolution,
                motorConfig.motorConfig.gearRatio, motorConfig.motorConfig.unitConversion);
    }

    /**
     * Applies the current configuration to the TalonSRX motor controller.
     * This method will apply the current configuration to the TalonSRX motor controller.
     */
    public void applyConfig(){
        var error = motor.configAllSettings(config);
        if (error.value != 0) {
            errorHandler.logAndReportError("could not apply config, error: " + error.name());
        }
    }

    @Override
    public Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }

    @Override
    public void setInverted(boolean inverted) {
        motor.setInverted(inverted);
    }

    @Override
    public void setIdleMode(BasicMotor.IdleMode mode) {
        var idleMode = switch (mode) {
            case COAST -> com.ctre.phoenix.motorcontrol.NeutralMode.Coast;
            case BRAKE -> com.ctre.phoenix.motorcontrol.NeutralMode.Brake;
        };

        motor.setNeutralMode(idleMode);
    }

    @Override
    public void updatePIDGainsToMotor(PIDGains pidGains, int slot) {
        motorGains = pidGains.convertToDutyCycle();

        var pidConfig = switch (slot){
            case 0 -> //noinspection DuplicateBranchesInSwitch
                    config.slot0;
            case 1 -> config.slot1;
            case 2 -> config.slot2;

            default -> config.slot0;
        };

        pidConfig.kP = motorGains.getK_P();
        pidConfig.kI = motorGains.getK_I();
        pidConfig.kD = motorGains.getK_D();

        pidConfig.allowableClosedloopError = motorGains.getTolerance();
        pidConfig.integralZone = motorGains.getI_Zone();
        pidConfig.maxIntegralAccumulator = motorGains.getI_MaxAccum();

        applyConfig();
    }

    @Override
    public void updateConstraintsGainsToMotor(ConstraintsGains constraints) {
        config.peakOutputForward = constraints.getMaxMotorOutput() / MotorManager.getConfig().DEFAULT_IDEAL_VOLTAGE;
        config.peakOutputReverse = -constraints.getMaxMotorOutput() / MotorManager.getConfig().DEFAULT_IDEAL_VOLTAGE;

        config.nominalOutputForward = constraints.getVoltageDeadband() / MotorManager.getConfig().DEFAULT_IDEAL_VOLTAGE;
        config.nominalOutputReverse = -constraints.getVoltageDeadband() / MotorManager.getConfig().DEFAULT_IDEAL_VOLTAGE;

        // How much time to go from 0 to 100% output in seconds, based on the voltage ramp rate
        config.closedloopRamp = constraints.getRampRate();
        config.openloopRamp = constraints.getRampRate();

        if (constraints.getConstraintType() == ConstraintsGains.ConstraintType.LIMITED
                && getDefaultMeasurements() instanceof TalonSRXMeasurements talonMeasurements) {
            config.forwardSoftLimitEnable = true;
            config.forwardSoftLimitThreshold = constraints.getMaxValue() * talonMeasurements.tickPerRevolution;

            config.reverseSoftLimitEnable = true;
            config.reverseSoftLimitThreshold = constraints.getMinValue() * talonMeasurements.tickPerRevolution;
        } else {
            config.forwardSoftLimitEnable = false;
            config.reverseSoftLimitEnable = false;
        }

        applyConfig();
    }
}
