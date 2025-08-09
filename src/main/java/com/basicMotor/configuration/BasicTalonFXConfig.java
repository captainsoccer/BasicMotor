package com.basicMotor.configuration;

import com.basicMotor.gains.currentLimits.CurrentLimitsTalonFX;

/**
 * This class represents the configuration for a basic TalonFX motor controller.
 * It extends the BasicMotorConfig class and provides specific configurations for TalonFX motors.
 * Use this class when using a TalonFX motor controller.
 * (Falcon 500, Kraken X60, Kraken X44).
 * See the <a href="https://github.com/captainsoccer/MotorUtils/wiki">wiki</a> //TODO: add wiki link
 * for more information on how to use this class.
 */
public class BasicTalonFXConfig extends  BasicMotorConfig {
    /**
     * The current limits configuration for the TalonFX motor controller.
     * Use this to protect the motor from overheating and drawing too much current.
     */
    public CurrentLimitConfig currentLimitConfig = new CurrentLimitConfig();

    /**
     * The name of the CAN bus that the TalonFX motor controller is connected to.
     * This is only useful when using a canivore and the motor is connected to a canivore.
     * Otherwise, do not change this value.
     */
    public String canBusName = "rio";

    /**
     * This enables the Field Oriented Control (FOC) for the TalonFX motor controller.
     * This is supported only on the TalonFX, with a licensed phoenix pro firmware.
     * For more information on FOC, see the FOC section in the
     * <a href="https://v6.docs.ctr-electronics.com/en/latest/docs/migration/new-to-phoenix.html">phoenix documentation</a>.
     */
    public boolean enableFOC = false;

    /**
     * This flag determines whether the measurements will wait for all signals to update before returning values.
     * This will work only if the device is connected to a CANivore and the Pro features are enabled.
     * Otherwise, it will be ignored.
     */
    public boolean waitForAllSignals = false;

    @Override
    public BasicTalonFXConfig copy() {
        var copy = new BasicTalonFXConfig();

        // Copy the basic motor configuration
        super.copy(copy);

        copy.currentLimitConfig = this.currentLimitConfig.copy();
        copy.canBusName = this.canBusName;
        copy.enableFOC = this.enableFOC;
        copy.waitForAllSignals = this.waitForAllSignals;

        return copy;
    }

    /**
     * Handles the configuration for the current limits.
     */
    public static class CurrentLimitConfig {
        /**
         * The maximum current output of the motor controller (in amps).
         * This is different from the supply current limit, and will usually be higher.
         * This can be used to limit the force output of the motor, to prevent damaging the mechanism.
         */
        public int statorCurrentLimit = 0;
        /**
         * The maximum current draw of the motor controller (in amps).
         * This is the current that the motor controller will draw from the battery.
         * If the motor draws this amount of current for more then {@link #lowerLimitTime} seconds,
         * it will lower to {@link #lowerCurrentLimit}.
         * Use this if there are brownouts or breakers tripping.
         * Otherwise, it is recommended to use {@link #statorCurrentLimit} instead.
         */
        public int supplyCurrentLimit = 0;
        /**
         * The time (in seconds) that the motor controller will stay in the supply current limit before lowering to the lower current limit.
         * This is useful to prevent the motor from overheating and tripping breakers.
         */
        public double lowerLimitTime = 0;
        /**
         * The current the motor drops to after the supply current limit is reached for {@link #lowerLimitTime}.
         * Works only if {@link #supplyCurrentLimit} and {@link #lowerLimitTime} is set.
         */
        public int lowerCurrentLimit = 0;

        /**
         * Creates the current limit configuration with the given values
         *
         * @return The current limits of the motor controller
         */
        public CurrentLimitsTalonFX getCurrentLimits() {
            return new CurrentLimitsTalonFX(
                    statorCurrentLimit, supplyCurrentLimit, lowerLimitTime, lowerCurrentLimit);
        }

        /**
         * Sets the current limits of the motor controller from the given current limits.
         *
         * @param currentLimits The current limits to set
         */
        public void fromCurrentLimits(CurrentLimitsTalonFX currentLimits) {
            this.statorCurrentLimit = currentLimits.getCurrentLimit();
            this.supplyCurrentLimit = currentLimits.getSupplyCurrentLimit();
            this.lowerLimitTime = currentLimits.getSupplyLowerTime();
            this.lowerCurrentLimit = currentLimits.getSupplyLowerLimit();
        }

        /**
         * Creates a copy of the current limit configuration.
         *
         * @return A new CurrentLimitConfig object with the same values
         */
        public CurrentLimitConfig copy() {
            var copy = new CurrentLimitConfig();

            copy.statorCurrentLimit = this.statorCurrentLimit;
            copy.supplyCurrentLimit = this.supplyCurrentLimit;
            copy.lowerLimitTime = this.lowerLimitTime;
            copy.lowerCurrentLimit = this.lowerCurrentLimit;

            return copy;
        }
    }
}
