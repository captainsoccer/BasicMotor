package io.github.captainsoccer.basicmotor.ctre.talonfx;

import com.ctre.phoenix6.controls.*;
import io.github.captainsoccer.basicmotor.control.ControlFrame;
import io.github.captainsoccer.basicmotor.ctre.talonfx.config.FOCMode;

import java.util.Objects;

public class TalonFXOutputManager {
    /**
     * The velocity request for the motor controller.
     * Used when using the built-in pid controller of the TalonFX.
     */
    private final VelocityVoltage velocityVoltage;

    private final VelocityTorqueCurrentFOC velocityFOC;

    /**
     * The position request for the motor controller.
     * Used when using the built-in pid controller of the TalonFX.
     */
    private final PositionVoltage positionVoltage;

    private final PositionTorqueCurrentFOC positionFOC;

    /**
     * The voltage request for the motor controller.
     */
    private final VoltageOut voltageRequest = new VoltageOut(0).withUpdateFreqHz(0);

    /**
     * The duty cycle request for the motor controller.
     */
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withUpdateFreqHz(0);

    /**
     * The torque current request for the motor controller.
     * Used when using the built-in FOC controller of the TalonFX.
     */
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0).withUpdateFreqHz(0);

    public TalonFXOutputManager(FOCMode mode) {
        Objects.requireNonNull(mode);
        switch (mode) {
            case CURRENT -> {
                velocityFOC = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
                positionFOC = new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
                velocityVoltage = null;
                positionVoltage = null;
            }

            case VOLTAGE_HYBRID -> {
                velocityVoltage = new VelocityVoltage(0).withUpdateFreqHz(0).withEnableFOC(true);
                positionVoltage = new PositionVoltage(0).withUpdateFreqHz(0).withEnableFOC(true);
                velocityFOC = null;
                positionFOC = null;
            }

            default -> {
                velocityVoltage = new VelocityVoltage(0).withUpdateFreqHz(0).withEnableFOC(false);
                positionVoltage = new PositionVoltage(0).withUpdateFreqHz(0).withEnableFOC(false);
                velocityFOC = null;
                positionFOC = null;
            }
        }
    }

    public ControlRequest getRequest(ControlFrame controlFrame){
        return switch (controlFrame.controlMode()){
            case PERCENT_OUTPUT -> dutyCycleRequest.withOutput(controlFrame.goal().position);
            case VOLTAGE -> voltageRequest.withOutput(controlFrame.goal().position);
            case POSITION, PROFILED_POSITION -> getPositionRequest(controlFrame);
            case VELOCITY, PROFILED_VELOCITY -> getVelocityRequest(controlFrame);
            case STOP -> s
        };
    }

    private ControlRequest getPositionRequest(ControlFrame controlFrame){
        if (positionVoltage == null)
            return positionFOC.withPosition(controlFrame.goal().position).withSlot(controlFrame.slot());
        else
            return positionVoltage.withPosition(controlFrame.goal().position).withSlot(controlFrame.slot());
    }

    private ControlRequest getVelocityRequest(ControlFrame controlFrame){
        if (velocityVoltage == null)
            return velocityFOC.withVelocity(controlFrame.goal().velocity).withSlot(controlFrame.slot());
        else
            return velocityVoltage.withVelocity(controlFrame.goal().velocity).withSlot(controlFrame.slot());
    }
}
