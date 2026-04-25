package io.github.captainsoccer.basicmotor.control;

/**
 * An enum that represents the control mode of the controller.
 */
public enum ControlMode {
    /**
     * Stops motor output.
     */
    STOP(false, false),
    /**
     * Control directly the voltage applied to the coils.
     */
    VOLTAGE(false, false),
    /**
     * Controls the duty cycle of the motor output. (a fraction of the input voltage).
     * Known as (precent output and duty cycle output)
     */
    PERCENT_OUTPUT(false, false),
    /**
     * Controls the motor with a closed loop position control.
     * Needs pid gains to be set in the controller gains.
     * If you want to use a motion profile, use {@link #PROFILED_POSITION} instead.
     */
    POSITION(false, true),
    /**
     * Controls the motor with a profiled position output.
     * This is the same as {@link #POSITION} but uses a motion profile to smooth the movement.
     * The motion profile is set in the controller gains.
     * The motion profile is always calculated on the rio (not using the built-in controllers features).
     */
    PROFILED_POSITION(true, true),
    /**
     * Controls the motor with a closed loop velocity control.
     * Needs pid gains to be set in the controller gains.
     * Also recommended to use a setpoint feedforward to help the motor stay in the requested velocity.
     * If you want to use a motion profile, use {@link #PROFILED_VELOCITY} instead.
     */
    VELOCITY(false, true),
    /**
     * Controls the motor with a profiled velocity output.
     * This is the same as {@link #VELOCITY} but uses a motion profile to smooth the movement.
     * The motion profile is set in the controller gains.
     * The motion profile is always calculated on the rio (not using the built-in controllers features).
     */
    PROFILED_VELOCITY(true, true),

    /**
     * Controls the motor with a closed loop current control.
     * Needs pid gains to be set in the controller gains.
     * This is used to control the current output of the motor.
     * Will work the same as {@link #TORQUE}.
     * This is used where a mechanism needs to apply a specific current to the motor.
     */
    CURRENT(false, true),

    /**
     * Controls the motor with a closed loop torque control.
     * Needs pid gains to be set in the controller gains.
     * This is used to control the torque output of the motor.
     * Will work the same as {@link #CURRENT}.
     * This is used where a mechanism needs to apply a specific torque to the motor.
     */
    TORQUE(false, true);

    /**
     * If the control mode is profiled.
     */
    private final boolean profiled;

    /**
     * If the control mode requires a PID controller.
     * This is used to check if the controller needs to calculate the PID output.
     */
    private final boolean requiresPID;

    ControlMode(boolean profiled, boolean requiresPID) {
        this.profiled = profiled;
        this.requiresPID = requiresPID;
    }

    /**
     * Checks if the control mode is a position control
     * This function is used for calculating the constraints of the controller.
     *
     * @return True if the control mode is a position control
     */
    public boolean isPositionControl() {
        return this == POSITION || this == PROFILED_POSITION;
    }

    /**
     * Checks if the control mode is a velocity control
     * This function is used for calculating the constraints of the controller.
     *
     * @return True if the control mode is a velocity control
     */
    public boolean isVelocityControl() {
        return this == VELOCITY || this == PROFILED_VELOCITY;
    }

    /**
     * Checks if the control mode is a current control.
     * Current control is both CURRENT and TORQUE.
     *
     * @return True if the control mode is a current control
     */
    public boolean isCurrentControl() {
        return this == CURRENT || this == TORQUE;
    }

    /**
     * Checks if the control mode is a profiled control
     * This function is used for checking if the controller is using a motion profile.
     *
     * @return True if the control mode is a profiled control
     */
    public boolean isProfiled() {
        return this.profiled;
    }

    /**
     * Checks if the control mode requires a pid controller
     * Used to check if there is a need to calculate the PID output.
     *
     * @return True if the control mode requires a pid controller
     */
    public boolean requiresPID() {
        return this.requiresPID;
    }
}
