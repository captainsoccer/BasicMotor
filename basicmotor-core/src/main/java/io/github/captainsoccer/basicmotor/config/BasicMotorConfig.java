package io.github.captainsoccer.basicmotor.config;

import io.github.captainsoccer.ImmutableConfigProcessor;
import io.github.captainsoccer.basicmotor.config.slots.SlotConfig;
import io.github.captainsoccer.basicmotor.measurements.Measurements;

import java.util.function.Supplier;

@ImmutableConfigProcessor.Immutable
public class BasicMotorConfig {

    public MotorBasicsConfig motorBasics = new MotorBasicsConfig();

    public SlotConfig slot0 = new SlotConfig();

    public SlotConfig slot1 = new SlotConfig();

    public SlotConfig slot2 = new SlotConfig();

    public ConstraintsConfig constraints = new ConstraintsConfig();

    public SimulationConfig simulation = new SimulationConfig();

    public LogLevelConfig logLevel;

    public LoopTimingConfig loopTiming = new LoopTimingConfig();

    public FollowerConfig follower = new FollowerConfig();

    public Supplier<Measurements.Measurement> customMeasurements = null;

    public ImmutableBasicMotorConfig immutable(){
        return new ImmutableBasicMotorConfig(this);
    }
}
