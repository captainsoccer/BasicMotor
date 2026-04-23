package io.github.captainsoccer.basicmotor.config.slots;

import java.util.function.Function;

public class FeedForwardConfig {
    /**
     * The interface that represent a gravity feedForward for mechanisms.
     * Use static methods to create the feedForward
     */
    @FunctionalInterface
    public interface KG {
        /**
         * calculates the feedforward based on the setpoint
         *
         * @param setpoint the setpoint of the motor (default units are rotations)
         * @return the feedForward to apply in volts
         */
        double calculate(double setpoint);

        /**
         * Creates a feed forward for an elevator mechanism.
         * This does not use the setpoint as the elevator feedforward always works
         *
         * @param kG The feed forward to apply in volts
         * @return the KG to give to the motor
         */
        static KG ELEVATOR(double kG) {
            return setpoint -> kG;
        }

        /**
         * creates a feedforward for an arm system based on the cosine of the target angle
         *
         * @param kG The feedforward to apply times the cosine (volts)
         * @return the KG to give to the motor
         */
        static KG ARM_COS(double kG) {
            return setpoint -> Math.cos(rotationsToRadians(setpoint)) * kG;
        }

        /**
         * creates a feedforward for an arm system based on the sine of the target angle
         *
         * @param kG The feedforward to apply times the sine (volts)
         * @return the KG to give to the motor
         */
        static KG ARM_SIN(double kG) {
            return setpoint -> Math.sin(rotationsToRadians(setpoint)) * kG;
        }

        /**
         * Applies no feedForward
         *
         * @return an empty feedForward
         */
        static KG NONE() {
            return setpoint -> 0;
        }

        /**
         * converts rotation to radians
         *
         * @param rotation the rotation to convert
         * @return the angle in radians
         */
        static double rotationsToRadians(double rotation) {
            return rotation / (Math.PI * 2);
        }
    }

    public double kV = 0;
    public double kA = 0;

    public KG kG = KG.NONE();

    public double kS = 0;
    public double kSDeadband = 0;

    public Function<Double, Double> custom = (setpoint) -> 0.0;
}
