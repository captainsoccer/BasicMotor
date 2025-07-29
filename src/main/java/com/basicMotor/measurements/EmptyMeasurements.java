package com.basicMotor.measurements;

/**
 * An empty implementation of the Measurements class.
 * This class is used when no measurements are available.
 * If this is used, any closed loop control will be disabled.
 * Used by brushed motors like the VictorSPX, which do not have built-in encoders.
 * It is also used by the TalonSRX class when no measurements are provided.
 */
public class EmptyMeasurements extends Measurements{
    @Override
    protected double getUpdatedPosition() {
        return 0;
    }

    @Override
    protected double getUpdatedVelocity() {
        return 0;
    }

    @Override
    protected double getUpdatedAcceleration() {
        return 0;
    }

    @Override
    public void setPosition(double position) {
        //Does nothing, as this is an empty implementation.
    }
}
