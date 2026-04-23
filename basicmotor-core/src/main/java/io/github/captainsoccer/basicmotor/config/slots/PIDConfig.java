package io.github.captainsoccer.basicmotor.config.slots;

public class PIDConfig {
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;

    public double iZone = Double.POSITIVE_INFINITY;
    public double IMaxAccum = 0;
    public double IMinAccum = -IMaxAccum;

    public double tolerance = 0;
    public boolean disableOutputWhileInTolerance = true;
}
