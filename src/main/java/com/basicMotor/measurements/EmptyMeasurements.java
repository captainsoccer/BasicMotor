package com.basicMotor.measurements;

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

    }
}
