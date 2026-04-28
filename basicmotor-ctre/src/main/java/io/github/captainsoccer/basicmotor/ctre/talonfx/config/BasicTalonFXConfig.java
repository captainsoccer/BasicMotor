package io.github.captainsoccer.basicmotor.ctre.talonfx.config;

import io.github.captainsoccer.basicmotor.config.BasicMotorConfig;

public class BasicTalonFXConfig extends BasicMotorConfig {
    public BasicTalonFXConfig() {
        super.currentLimit = this.currentLimit;
    }

    public CanCoderConfig canCoder = new CanCoderConfig();

    public CTRECurrentLimitsConfig currentLimit = new CTRECurrentLimitsConfig();

    public boolean waitForAllStatusSignals = false;

    public FOCMode focMode = FOCMode.OFF;
}
