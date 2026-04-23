package io.github.captainsoccer.basicmotor.config;

public class SimulationConfig {

    public SimulationType simulationType = SimulationType.flyWheel();

    public double kV = 0;

    public double kA = 0;

    public double momentOfInertia = 0;

    public double positionSTD = 0;

    public double velocitySTD = 0;

    private enum SimulationMechanism{
        FLY_WHEEL,
        ARM,
        ELEVATOR
    }

    public record SimulationType(
            SimulationMechanism mechanism,
            boolean simulateGravity,
            double value0,
            double value1
    ){
        public static SimulationType flyWheel(){
            return new SimulationType(SimulationMechanism.FLY_WHEEL, false, 0, 0);
        }

        public static SimulationType arm(boolean simulateGravity, double armLength, double startingAngle){
            return new SimulationType(SimulationMechanism.ARM, simulateGravity, armLength, startingAngle);
        }

        public static SimulationType elevator(boolean simulateGravity, double mass, double pulleyRadius){
            return new SimulationType(SimulationMechanism.ELEVATOR, simulateGravity, mass, pulleyRadius);
        }
    }
}
