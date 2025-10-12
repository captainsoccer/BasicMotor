package io.github.captainsoccer.basicmotor.sim.arm;


import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.gains.ControllerGains;
import io.github.captainsoccer.basicmotor.sim.BasicSimSystem;

/**
 * A class that simulates a single-jointed arm system using the SingleJointedArmSim class. It is in
 * the basic sim motor system and has all the functionality of a basic sim system. Use this when you
 * want to simulate an arm in your robot code. units are in rotations.
 */
public class BasicArmSim extends BasicSimSystem {
    /**
     * The SingleJointedArmSim instance used by this BasicSimArm.
     */
    private final SingleJointedArmSim armSim;

    /**
     * Creates a BasicSimArm instance with the provided SingleJointedArmSim and name.
     *
     * @param armSim The SingleJointedArmSim instance to use for the arm simulation
     * @param name   The name of the arm simulation
     * @param gains  The controller gains to use for the arm simulation
     */
    public BasicArmSim(SingleJointedArmSim armSim, String name, ControllerGains gains) {
        super(new ArmSimInterface(armSim, name), gains);
        this.armSim = armSim;
    }

    /**
     * Creates a BasicSimArm instance with the provided configuration.
     * the config must have the {@link BasicMotorConfig.SimulationConfig#momentOfInertia} set.
     *
     * @param config The configuration for the arm motor
     */
    public BasicArmSim(BasicMotorConfig config) {
        super(new ArmSimInterface(config), config);

        this.armSim = ((ArmSimInterface) super.motorInterface).armSim;
    }


    @Override
    protected void setInputVoltage(double voltage) {
        armSim.setInputVoltage(voltage);
    }

    @Override
    protected double getCurrentDraw() {
        return armSim.getCurrentDrawAmps();
    }
}
