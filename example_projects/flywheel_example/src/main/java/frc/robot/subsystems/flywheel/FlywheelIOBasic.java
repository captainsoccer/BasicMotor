package frc.robot.subsystems.flywheel;


import edu.wpi.first.wpilibj.RobotBase;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.ctre.talonfx.BasicTalonFX;
import io.github.captainsoccer.basicmotor.sim.flywheel.BasicFlywheelSim;

public class FlywheelIOBasic implements  FlywheelIO {
    private final BasicMotor motor;

    public FlywheelIOBasic() {
        this.motor = RobotBase.isReal() ? new BasicTalonFX(FlywheelConstants.motorConfig) :
                        new BasicFlywheelSim(FlywheelConstants.motorConfig);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.velocityMetersPerSecond = motor.getVelocity();
        inputs.atTarget = motor.atSetpoint() && motor.getController().getControlMode().requiresPID();
    }

    @Override
    public void setTargetVelocity(double targetMetersPerSecond) {
        motor.setControl(targetMetersPerSecond, ControlMode.PROFILED_VELOCITY);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setPercentOutput(double percentOutput) {
        motor.setPercentOutput(percentOutput);
    }
}
