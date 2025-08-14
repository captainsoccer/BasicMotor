package io.github.captainsoccer.basicmotor.rev;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.gains.ControllerGains;
import io.github.captainsoccer.basicmotor.rev.BasicSparkBaseConfig.AbsoluteEncoderConfig.AbsoluteEncoderRange;

/**
 * This class represents a basic spark max motor controller.
 * It extends the BasicSparkBase class and provides
 * functionality specific to the Spark Max motor controller.
 * This class assumes that the motor is brushless.
 */
public class BasicSparkMAX extends BasicSparkBase {
    /**
     * Creates a basic spark max motor controller with the given gains, id, name, gear ratio,
     *
     * @param gains          The gains of the motor controller
     * @param id             The id of the motor controller
     * @param name           The name of the motor controller (used for logging and debugging)
     * @param gearRatio      The gear ratio of the motor controller (how many rotations of the motor are a rotation of the mechanism)
     * @param unitConversion The conversion factor for the motor's position units.
     *                       This will be multiplied by the motor's rotation to get the position with the desired units.
     *                       The unit for this value is desired position unit per rotation.
     * @param brushless      Whether the motor is brushless or not.
     *                       Brushless motor have an integrated encoder that can be used for position and velocity control.
     *                       those include: (Neo, Neo 550, Neo vortex (and probably others)).
     *                       brushed controllers do not have an integrated encoder and
     *                       require an external encoder to be used for position and velocity control.
     *                       those include: (CIM, mini-CIM, and many more).
     */
    public BasicSparkMAX(
            ControllerGains gains,
            int id,
            String name,
            double gearRatio,
            double unitConversion,
            boolean brushless) {

        super(
                // creates a new SparkMax motor controller with the given id and type
                new SparkMax(id, brushless ? MotorType.kBrushless : MotorType.kBrushed),
                new SparkMaxConfig(),
                gains,
                name,
                gearRatio,
                unitConversion);
    }

    /**
     * Creates a basic spark max motor controller with the given gains, id, name, gear ratio,
     *
     * @param gains     The gains of the motor controller
     * @param id        The id of the motor controller
     * @param name      The name of the motor controller (used for logging and debugging)
     * @param gearRatio The gear ratio of the motor controller (how many rotations of the motor are a rotation of the mechanism)
     * @param brushless Whether the motor is brushless or not.
     *                  Brushless motor have an integrated encoder that can be used for position and velocity control.
     *                  those include: (Neo, Neo 550, Neo vortex (and probably others)).
     *                  brushed controllers do not have an integrated encoder and
     *                  require an external encoder to be used for position and velocity control.
     *                  those include: (CIM, mini-CIM, and many more).
     *
     */
    public BasicSparkMAX(
            ControllerGains gains,
            int id,
            String name,
            double gearRatio,
            boolean brushless) {

        this(gains, id, name, gearRatio, 1, brushless);
    }

    /**
     * Creates a basic spark max motor controller with the given configuration.
     *
     * @param config The configuration for the motor controller.
     */
    public BasicSparkMAX(BasicMotorConfig config) {
        super(new SparkMax(config.motorConfig.id, BasicSparkBase.getMotorType(config)), new SparkMaxConfig(), config);

        if (config instanceof BasicSparkBaseConfig sparkBaseConfig) {
            // checks if both absolute and external encoders are being used
            if (sparkBaseConfig.externalEncoderConfig.useExternalEncoder
                    && sparkBaseConfig.absoluteEncoderConfig.useAbsoluteEncoder) {
                throw new IllegalArgumentException(
                        "motor: "
                                + config.motorConfig.name
                                + " cannot use both absolute and external encoders at the same time");
            }
        }
    }

    @Override
    public void useAbsoluteEncoder(boolean inverted, double zeroOffset, double sensorToMotorRatio, AbsoluteEncoderRange absoluteEncoderRange) {
        // sets the absolute encoder configuration
        getSparkConfig().absoluteEncoder.setSparkMaxDataPortConfig();

        //calls on the normal useAbsoluteEncoder method
        super.useAbsoluteEncoder(inverted, zeroOffset, sensorToMotorRatio, absoluteEncoderRange);
    }

    @Override
    protected void configExternalEncoder(boolean inverted, double sensorToMotorRatio) {

        if (!(getSparkConfig() instanceof SparkMaxConfig config)) {
            throw new RuntimeException("SparkMax motor is not an instance of SparkMaxConfig");
        }

        // sets the absolute encoder configuration
        config.alternateEncoder.setSparkMaxDataPortConfig();
        // sets whether the absolute encoder is inverted or not
        config.alternateEncoder.inverted(inverted);
        // sets the conversion factor for the absolute encoder position and velocity
        config.alternateEncoder.positionConversionFactor(sensorToMotorRatio);
        config.alternateEncoder.velocityConversionFactor(sensorToMotorRatio);
    }

    @Override
    protected RelativeEncoder getExternalEncoder() {
        if (!(getMotor() instanceof SparkMax sparkMax)) {
            throw new RuntimeException("SparkMax motor is not an instance of SparkMax");
        }

        return sparkMax.getAlternateEncoder();
    }
}
