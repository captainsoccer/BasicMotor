package io.github.captainsoccer.basicmotor.motorManager;

/**
 * This class holds the configuration for the MotorManager.
 * It includes the frequency of the PID loop, profile loop, and sensor loop,
 * as well as the ideal voltage fed into the motor when it is not moving.
 * Change these values only if you know what you are doing.
 * Replace the default config instance in the MotorManager class before constructing any motors.
 */
public class MotorManagerConfig {

    /**
     * The frequency of the PID loop (in Hz) used for when the pid controller runs on the rio.
     * should be faster than the profile loop and sensor loop.
     */
    public final double PID_LOOP_HZ;
    /**
     * The frequency of the profile loop (in Hz) used for when the pid controller runs on the motor.
     * This can be slower than the PID loop and sensor loop.
     */
    public final double PROFILE_LOOP_HZ;
    /**
     * The frequency of the sensor loop (in Hz) used to update the sensors.
     * This can be very slow, as it is only used to update the sensors.
     */
    public final double SENSOR_LOOP_HZ;

    /**
     * The ideal voltage fed into the motor when it is not moving (in volts).
     * This should be the voltage the battery is rated for.
     */
    public final double motorIdealVoltage;

    /**
     * The maximum output of the motor (in volts).
     * can be slightly higher than the battery voltage.
     */
    public final double defaultMaxMotorOutput;

    /**
     * The time in seconds that the motor loops starts
     */
    public final double STARTUP_DELAY_SECONDS;

    /**
     * The default configuration for the motor manager.
     * This is used if no configuration is provided when constructing the MotorManager.
     * It is recommended to replace this with a custom configuration before constructing any motors.
     */
    public static final MotorManagerConfig DEFAULT_CONFIG = new MotorManagerConfig();

    /**
     * Constructor for the motor manager config.
     *
     * @param PID_LOOP_HZ           The frequency of the PID loop (in Hz) used for the pid controller on the rio.
     * @param PROFILE_LOOP_HZ       The frequency of the profile loop (in Hz) used for the pid controller on the motor.
     * @param SENSOR_LOOP_HZ        The frequency of the sensor loop (in Hz) used to update the sensors.
     * @param defaultMaxMotorOutput The maximum output of the motor (in volts).
     * @param motorIdealVoltage     The ideal voltage fed into the motor when it is not moving (in volts).
     * @param startupDelaySeconds   The delay in seconds the motor loop will wait before starting (after motor initialization)
     */
    public MotorManagerConfig(double PID_LOOP_HZ, double PROFILE_LOOP_HZ, double SENSOR_LOOP_HZ, double defaultMaxMotorOutput, double motorIdealVoltage, double startupDelaySeconds) {
        if (PID_LOOP_HZ <= 0 || PROFILE_LOOP_HZ <= 0 || SENSOR_LOOP_HZ <= 0) {
            throw new IllegalArgumentException("Loop frequencies must be greater than zero.");
        }
        this.PID_LOOP_HZ = PID_LOOP_HZ;
        this.PROFILE_LOOP_HZ = PROFILE_LOOP_HZ;
        this.SENSOR_LOOP_HZ = SENSOR_LOOP_HZ;

        if (defaultMaxMotorOutput <= 0 || motorIdealVoltage <= 0) {
            throw new IllegalArgumentException("Motor output and ideal voltage must be greater than zero.");
        }
        this.defaultMaxMotorOutput = defaultMaxMotorOutput;
        this.motorIdealVoltage = motorIdealVoltage;

        if (startupDelaySeconds <= 0) {
            throw new IllegalArgumentException("Startup delay must be greater than zero.");
        }

        this.STARTUP_DELAY_SECONDS = startupDelaySeconds;
    }

    /**
     * Constructor for the motor manager config with default max motor output and ideal voltage.
     *
     * @param PID_LOOP_HZ     The frequency of the PID loop (in Hz) used for the pid controller on the rio.
     * @param PROFILE_LOOP_HZ The frequency of the profile loop (in Hz) used for the pid controller on the motor.
     * @param SENSOR_LOOP_HZ  The frequency of the sensor loop (in Hz) used to update the sensors.
     */
    public MotorManagerConfig(double PID_LOOP_HZ, double PROFILE_LOOP_HZ, double SENSOR_LOOP_HZ) {
        this(PID_LOOP_HZ, PROFILE_LOOP_HZ, SENSOR_LOOP_HZ, DEFAULT_CONFIG.defaultMaxMotorOutput, DEFAULT_CONFIG.motorIdealVoltage, DEFAULT_CONFIG.STARTUP_DELAY_SECONDS);
    }

    /**
     * The default constructor for the motor manager config.
     */
    public MotorManagerConfig() {
        this.PID_LOOP_HZ = 100;
        this.PROFILE_LOOP_HZ = 50;
        this.SENSOR_LOOP_HZ = 4;
        this.defaultMaxMotorOutput = 12;
        this.motorIdealVoltage = 13;
        this.STARTUP_DELAY_SECONDS = 5;
    }

}
