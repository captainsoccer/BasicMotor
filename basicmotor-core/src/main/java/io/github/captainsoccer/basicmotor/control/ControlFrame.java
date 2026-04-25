package io.github.captainsoccer.basicmotor.control;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public record ControlFrame(TrapezoidProfile.State goal, ControlMode controlMode, int slot, double arbFeedforward) {
    public ControlFrame(double setpoint, ControlMode controlMode, int slot, double arbFeedforward){
        this(new TrapezoidProfile.State(setpoint, 0), controlMode, slot, arbFeedforward);
    }

    public ControlFrame(){
        this(new TrapezoidProfile.State(), ControlMode.STOP, 0, 0);
    }

    public ControlFrame(double goal, double goalVelocity, ControlMode controlMode, int slot, double arbFeedforward){
        this(new TrapezoidProfile.State(goal, goalVelocity), controlMode, slot, arbFeedforward);
    }
}
