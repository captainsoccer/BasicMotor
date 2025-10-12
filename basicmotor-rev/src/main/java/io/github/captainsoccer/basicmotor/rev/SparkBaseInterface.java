package io.github.captainsoccer.basicmotor.rev;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj.DriverStation;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.MotorInterface;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;
import io.github.captainsoccer.basicmotor.measurements.EmptyMeasurements;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;
import io.github.captainsoccer.basicmotor.rev.encoders.RevRelativeEncoder;

public class SparkBaseInterface extends MotorInterface {
    private final Measurements defaultMeasurements;

    public final SparkBase motor;

    public final SparkBaseConfig config;

    public SparkBaseInterface(SparkBase motor,
                                 SparkBaseConfig motorConfig,
                                 String name,
                                 double gearRatio,
                                 double unitConversion) {
        super(name);

        this.motor = motor;
        this.config = motorConfig.voltageCompensation(MotorManager.config.motorIdealVoltage); // set the voltage compensation to the idle voltage
        // all configs should be stored in code and not on motor
        applyConfig();

        configurePeriodicFrames(MotorManager.ControllerLocation.MOTOR.getHZ());

        defaultMeasurements = motor.getMotorType() == SparkLowLevel.MotorType.kBrushless ?
                new RevRelativeEncoder(motor.getEncoder(), gearRatio, unitConversion) :
                new EmptyMeasurements();
    }

    public SparkBaseInterface(SparkBase motor, SparkBaseConfig config, BasicMotorConfig motorConfig){
        super(motorConfig);
        this.motor = motor;
        this.config = config.voltageCompensation(MotorManager.config.motorIdealVoltage); // set the voltage compensation to the idle voltage
        // all configs should be stored in code and not on motor
        applyConfig();

        configurePeriodicFrames(MotorManager.ControllerLocation.MOTOR.getHZ());

        double gearRatio = motorConfig.motorConfig.gearRatio;
        double unitConversion = motorConfig.motorConfig.unitConversion;

        if (!(motorConfig instanceof BasicSparkConfig sparkBaseConfig)) {
            DriverStation.reportWarning("not using specific Spark Base config for motor: " + name, false);
            defaultMeasurements = new RevRelativeEncoder(motor.getEncoder(), gearRatio, unitConversion);
            return;
        }

        var primaryEncoderConfig = sparkBaseConfig.primaryEncoderConfig;
        defaultMeasurements = primaryEncoderConfig.usePrimaryEncoder ?
                new RevRelativeEncoder(
                        motor.getEncoder(),
                        gearRatio,
                        unitConversion) :
                new EmptyMeasurements();

        if (primaryEncoderConfig.countsPerRevolution != 0)
            config.encoder.countsPerRevolution(primaryEncoderConfig.countsPerRevolution);
    }

    @Override
    public Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }

    @Override
    public double getInternalPIDLoopTime() {
        return 0.001; // Spark max and spark flex have a fixed internal PID loop time of 1ms
        // This is according to the documentation:
        // https://docs.revrobotics.com/revlib/spark/closed-loop
    }

    @Override
    public void setInverted(boolean inverted) {
        config.inverted(inverted);

        applyConfig();
    }

    @Override
    public void setIdleMode(BasicMotor.IdleMode mode) {
        SparkBaseConfig.IdleMode value =
                switch (mode) {
                    case BRAKE -> SparkBaseConfig.IdleMode.kBrake;
                    case COAST -> SparkBaseConfig.IdleMode.kCoast;
                };

        config.idleMode(value);
        applyConfig();
    }

    @Override
    public void updatePIDGainsToMotor(PIDGains pidGains, int slot) {
        var gains = pidGains.convertToDutyCycle();

        ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.values()[slot];

        // sets the PID gains for the closed loop controller
        config.closedLoop.pid(gains.getK_P(), gains.getK_I(), gains.getK_D(), closedLoopSlot);
        config.closedLoop.iZone(gains.getI_Zone(), closedLoopSlot);
        config.closedLoop.iMaxAccum(gains.getI_MaxAccum(), closedLoopSlot);

        if (gains.getTolerance() != 0) {
            DriverStation.reportWarning(
                    "Spark MAX does not use tolerance in the PID controller (works on rio PID Controller), so it is ignored: "
                            + name,
                    false);
        }

        applyConfig();
    }


    @Override
    public void updateConstraintsGainsToMotor(ConstraintsGains constraints) {
        double idealVoltage = MotorManager.config.motorIdealVoltage;

        // sets the max voltage to the max motor output
        config.closedLoop.maxOutput(constraints.getMaxMotorOutput() / idealVoltage);
        config.closedLoop.minOutput(constraints.getMinMotorOutput() / idealVoltage);

        config.closedLoopRampRate(constraints.getRampRate());
        config.openLoopRampRate(constraints.getRampRate());

        //if the constraints are continuous, ignores them.
        //the rio code handles the continuous constraints.
        if (constraints.getConstraintType() == ConstraintsGains.ConstraintType.LIMITED) {
            // sets the soft limits to the max and min values
            config.softLimit.forwardSoftLimit(constraints.getMaxValue());
            config.softLimit.reverseSoftLimit(constraints.getMinValue());
            // enables the soft limits
            config.softLimit.forwardSoftLimitEnabled(true);
            config.softLimit.reverseSoftLimitEnabled(true);
        } else {
            // disables the soft limits
            config.softLimit.forwardSoftLimitEnabled(false);
            config.softLimit.reverseSoftLimitEnabled(false);
        }

        if (constraints.getVoltageDeadband() != 0) {
            DriverStation.reportWarning(
                    "Spark motor controllers do not use voltage deadband (works on RIO PID controller), so it is ignored: "
                            + name,
                    false);
        }

        applyConfig();
    }

    /**
     * Configures the periodic frames according to the main loop frequency.
     * Also applies the sensor loop frequency.
     * Note that not all periodic frames will be changed due to the limitations of the Spark base motor controllers.
     * For more information about periodic frames, see the
     * <a href="https://docs.revrobotics.com/brushless/spark-max/control-interfaces#periodic-status-frames">rev website</a>
     *
     * @param mainLoopHZ The frequency of the main loop in Hz.
     */
    public void configurePeriodicFrames(double mainLoopHZ) {
        var signals = config.signals;
        int sensorLoopPeriodMs =
                (int) ((1 / MotorManager.config.SENSOR_LOOP_HZ) * 1000); // convert to milliseconds
        int mainLoopPeriodMs = (int) ((1 / mainLoopHZ) * 1000); // convert to milliseconds

        signals.busVoltagePeriodMs(sensorLoopPeriodMs); // currently does nothing
        signals.motorTemperaturePeriodMs(sensorLoopPeriodMs); // currently does nothing
        signals.iAccumulationPeriodMs(sensorLoopPeriodMs); // currently unknown if it does anything
        signals.outputCurrentPeriodMs(sensorLoopPeriodMs); // currently does nothing

        signals.appliedOutputPeriodMs(mainLoopPeriodMs); // currently does something
        signals.primaryEncoderPositionAlwaysOn(false);
        signals.primaryEncoderVelocityAlwaysOn(false);
        signals.primaryEncoderPositionPeriodMs(mainLoopPeriodMs); // currently does something
        signals.primaryEncoderVelocityPeriodMs(mainLoopPeriodMs); // currently does something

        applyConfig();
    }

    /**
     * This applies the config file to the spark base motor controller.
     * If there is an error applying the configuration,
     * it will report the error to the driver station.
     */
    public void applyConfig() {
        var okSignal =
                motor.configure(
                        config,
                        SparkBase.ResetMode.kResetSafeParameters,
                        SparkBase.PersistMode.kNoPersistParameters);

        if (okSignal != REVLibError.kOk) {
            DriverStation.reportError(
                    "Failed to apply configuration to Spark MAX motor: "
                            + name
                            + ". Error: "
                            + okSignal.name(),
                    false);
        }
    }
}
