package com.basicMotor.motors.sparkBase;

import com.basicMotor.configuration.BasicMotorConfig;
import com.basicMotor.configuration.BasicSparkBaseConfig;
import com.basicMotor.gains.ControllerGains;
import com.basicMotor.MotorManager.MotorManager;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.basicMotor.configuration.BasicSparkBaseConfig.AbsoluteEncoderConfig.AbsoluteEncoderRange;
import com.revrobotics.spark.config.SparkFlexConfig;

/**
 * This class represents a basic spark flex motor controller.
 * It extends the BasicSparkBase class and provides
 * functionality specific to the Spark Flex motor controller.
 * This class assumes that the motor is brushless.
 */
public class BasicSparkFlex extends BasicSparkBase {
    /**
     * Creates a basic spark flex motor controller with the given gains, id, name, gear ratio,
     *
     * @param gains          The gains of the motor controller
     * @param id             The id of the motor controller
     * @param name           The name of the motor controller (used for logging and debugging)
     * @param gearRatio      The gear ratio of the motor controller (how many rotations of the motor are a rotation of the mechanism)
     * @param unitConversion The conversion factor for the motor's position units.
     *                       This will be multiplied by the motor's rotation to get the position with the desired units.
     *                       The unit for this value is desired position unit per rotation.
     * @param location       The location of the pid controller (RIO or MOTOR).
     */
    public BasicSparkFlex(
            ControllerGains gains,
            int id,
            String name,
            double gearRatio,
            double unitConversion,
            MotorManager.ControllerLocation location) {

        super(
                //creates a new SparkFlex motor controller with the given id and type
                new SparkFlex(id, SparkLowLevel.MotorType.kBrushless),
                new SparkFlexConfig(),
                gains,
                name,
                gearRatio,
                unitConversion,
                location);
    }

    /**
     * Creates a basic spark flex motor controller with the given gains, id, name, gear ratio,
     *
     * @param gains     The gains of the motor controller
     * @param id        The id of the motor controller
     * @param name      The name of the motor controller (used for logging and debugging)
     * @param gearRatio The gear ratio of the motor controller (how many rotations of the motor are a rotation of the mechanism)
     * @param location  The location of the pid controller (RIO or MOTOR).
     */
    public BasicSparkFlex(
            ControllerGains gains,
            int id,
            String name,
            double gearRatio,
            MotorManager.ControllerLocation location) {

        this(gains, id, name, gearRatio, 1, location);
    }

    /**
     * Creates a basic spark flex motor controller with the given configuration.
     *
     * @param config The configuration for the motor controller.
     */
    public BasicSparkFlex(BasicMotorConfig config) {
        super(
                // creates a new SparkFlex motor controller with the given id and type
                new SparkFlex(config.motorConfig.id, SparkLowLevel.MotorType.kBrushless),
                new SparkFlexConfig(),
                config);

        if (config instanceof BasicSparkBaseConfig sparkBaseConfig) {
            //if the user configured to use an external encoder with an absolute encoder,
            // then we will use the external encoder with the absolute encoder
            if (sparkBaseConfig.externalEncoderConfig.useExternalEncoder
                    && sparkBaseConfig.absoluteEncoderConfig.useAbsoluteEncoder) {
                var absoluteEncoderConfig = sparkBaseConfig.absoluteEncoderConfig;

                useExternalEncoderWithAbsoluteEncoder(
                        absoluteEncoderConfig.inverted,
                        absoluteEncoderConfig.sensorToMotorRatio,
                        absoluteEncoderConfig.zeroOffset,
                        absoluteEncoderConfig.absoluteEncoderRange);
            }
        }
    }

    @Override
    protected void configExternalEncoder(boolean inverted, double sensorToMotorRatio) {
        assert getSparkConfig() instanceof SparkFlexConfig;

        var config = (SparkFlexConfig) getSparkConfig();

        // sets whether the absolute encoder is inverted or not
        config.externalEncoder.inverted(inverted);
        // sets the conversion factor for the absolute encoder position and velocity
        config.externalEncoder.positionConversionFactor(sensorToMotorRatio);
        config.externalEncoder.velocityConversionFactor(sensorToMotorRatio);
    }

    @Override
    protected RelativeEncoder getExternalEncoder() {
        assert getMotor() instanceof SparkFlex;

        var sparkMax = (SparkFlex) getMotor();

        return sparkMax.getExternalEncoder();
    }

    /**
     * Use this when you have a through-bore encoder connected to the mechanism in a 1:1 ratio with the mechanism.
     * This will provide better accuracy than using the absolute encoder directly.
     * This works only if the relative and absolute encoder is connected to the spark flex data port.
     *
     * @param inverted             Whether the through-bore encoder is inverted or not.
     *                             The default positive direction of the encoder is clockwise.
     *                             This is opposite to the default positive direction of a motor.
     * @param sensorToMotorRatio   The number of rotations of the sensor for each rotation of the motor.
     *                             This will be the same as the gear ratio of the mechanism.
     * @param zeroOffset           The zero offset of the absolute encoder.
     *                             when the absolute encoder reads this value, it will report a position of 0.
     *                             This is affected by absoluteEncoderRange.
     * @param absoluteEncoderRange The range of the absolute encoder.
     *                             This will determine the values that the absolute encoder can report.
     *                             (0 to 1, -0.5 to 0.5, etc.)
     */
    public void useExternalEncoderWithAbsoluteEncoder(
            boolean inverted,
            double sensorToMotorRatio,
            double zeroOffset,
            AbsoluteEncoderRange absoluteEncoderRange) {

        // configures the absolute encoder
        // but does not set it as the feedback sensor
        setAbsoluteEncoderConfig(inverted, zeroOffset, sensorToMotorRatio, absoluteEncoderRange);

        // sets the external encoder configuration
        // and sets the external encoder as the feedback sensor
        // also applies the configuration to the motor
        useExternalEncoder(inverted, sensorToMotorRatio);

        assert getMotor() instanceof SparkFlex;

        var sparkFlex = (SparkFlex) getMotor();

        sparkFlex.getExternalEncoder().setPosition(sparkFlex.getAbsoluteEncoder().getPosition());
    }
}
