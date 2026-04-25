package io.github.captainsoccer.basicmotor.config;

public class ConstraintsConfig {

    public Constraint constraint = Constraint.none();

    public double maxOutput;

    public double minOutput = -maxOutput;

    public double deadband = 0;

    public double rampRate = 0;

    public enum ConstraintType{
        CONTINUOUS,
        LIMITED,
        NONE
    }

    public record Constraint(
            ConstraintType type,
            double minValue,
            double maxValue
    ){
        /**
         * No constraints.
         * This means that the controller has no constraints.
         * good for flywheels or other mechanisms that do not need constraints.
         */
        public static Constraint none(){
            return new Constraint(ConstraintType.NONE, 0, 0);
        }

        /**
         * Limited (also known as soft limits).
         * It means that the mechanism has a limited range of motion.
         * This means the motor will not go beyond the limits.
         * Commonly used for arm motors or other mechanisms with a limited range of motion.
         */
        public static Constraint limited(double minPosition, double maxPosition){
            return new Constraint(ConstraintType.LIMITED, minPosition, maxPosition);
        }

        /**
         * Continuous (also known as continuous wrap).
         * It means that the mechanism is in some sort of loop.
         * This means the motor will go the shortest way to the goal.
         * (range defined by the max and min values)
         * This is commonly used for swerve steer motors.
         * This constraint is only supported for position control.
         */
        public static Constraint continuous(double startOfCircle, double endOfCircle){
            return  new Constraint(ConstraintType.CONTINUOUS, startOfCircle, endOfCircle);
        }
    }
}
