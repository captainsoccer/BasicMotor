package io.github.captainsoccer.basicmotor.config.slots;

import io.github.captainsoccer.basicmotor.config.MotorBasicsConfig;

public class SlotConfig {
    public PIDConfig pid = new PIDConfig();

    public FeedForwardConfig feedforward = new FeedForwardConfig();

    public MotionProfileConfig motionProfile = new MotionProfileConfig();
}
