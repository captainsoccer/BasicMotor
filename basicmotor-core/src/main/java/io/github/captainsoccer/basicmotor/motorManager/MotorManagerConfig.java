package io.github.captainsoccer.basicmotor.motorManager;

/**
 * This class holds the configuration for the MotorManager.
 * It includes the frequency of the PID loop, profile loop, and sensor loop,
 * as well as the ideal voltage fed into the motor when it is not moving.
 * Change these values only if you know what you are doing.
 * Replace the default config instance in the MotorManager class before constructing any motors.
 *
 * @param PID_LOOP_HZ           The frequency of the PID loop (in Hz) used for when the pid controller runs on the rio.
 *                              should be faster than the profile loop and sensor loop.
 * @param PROFILE_LOOP_HZ       The frequency of the profile loop (in Hz) used for when the pid controller runs on the motor.
 *                              This can be slower than the PID loop and sensor loop.
 * @param SENSOR_LOOP_HZ        The frequency of the sensor loop (in Hz) used to update the sensors.
 *                              This can be very slow, as it is only used to update the sensors.
 * @param motorIdealVoltage     The ideal voltage fed into the motor when it is not moving (in volts).
 *                              This should be the voltage the battery is rated for.
 * @param defaultMaxMotorOutput The maximum output of the motor (in volts).
 *                              can be slightly higher than the battery voltage.
 */
public record MotorManagerConfig(double PID_LOOP_HZ, double PROFILE_LOOP_HZ, double SENSOR_LOOP_HZ,
                                 double defaultMaxMotorOutput, double motorIdealVoltage) {

    /**
     * The default configuration for the motor manager.
     * This is used if no configuration is provided when constructing the MotorManager.
     * It is recommended to replace this with a custom configuration before constructing any motors.
     */
    public static final MotorManagerConfig DEFAULT_CONFIG = new MotorManagerConfig();

    /**
     * Checks if the provided configuration is valid.
     */
    public MotorManagerConfig {
        if (PID_LOOP_HZ <= 0 || PROFILE_LOOP_HZ <= 0 || SENSOR_LOOP_HZ <= 0) {
            throw new IllegalArgumentException("Loop frequencies must be greater than zero.");
        }
        if (defaultMaxMotorOutput <= 0 || motorIdealVoltage <= 0) {
            throw new IllegalArgumentException("Motor output and ideal voltage must be greater than zero.");
        }
    }

    /**
     * Constructor for the motor manager config with default max motor output and ideal voltage.
     *
     * @param PID_LOOP_HZ     The frequency of the PID loop (in Hz) used for the pid controller on the rio.
     * @param PROFILE_LOOP_HZ The frequency of the profile loop (in Hz) used for the pid controller on the motor.
     * @param SENSOR_LOOP_HZ  The frequency of the sensor loop (in Hz) used to update the sensors.
     */
    public MotorManagerConfig(double PID_LOOP_HZ, double PROFILE_LOOP_HZ, double SENSOR_LOOP_HZ) {
        this(PID_LOOP_HZ, PROFILE_LOOP_HZ, SENSOR_LOOP_HZ, DEFAULT_CONFIG.defaultMaxMotorOutput, DEFAULT_CONFIG.motorIdealVoltage);
    }

    /**
     * The default constructor for the motor manager config.
     * This is hardcoded to the default values.
     */
    public MotorManagerConfig() {
        this(100, 50, 4, 12, 13);
    }

}
