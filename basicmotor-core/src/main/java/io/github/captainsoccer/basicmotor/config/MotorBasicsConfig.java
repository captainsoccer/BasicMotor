package io.github.captainsoccer.basicmotor.config;

import edu.wpi.first.math.system.plant.DCMotor;

public class MotorBasicsConfig {
    public enum IdleMode{
        COAST,
        BRAKE
    }

    public String CANNetworkName = "rio";

    public int CANID = 0;

    public String motorName = "Motor";

    public DCMotor motorType = DCMotor.getKrakenX60(1);

    public double gearRatio = 1.0;

    public double unitConversion = 1.0;

    public boolean inverted = false;

    public IdleMode idleMode = IdleMode.BRAKE;
}
