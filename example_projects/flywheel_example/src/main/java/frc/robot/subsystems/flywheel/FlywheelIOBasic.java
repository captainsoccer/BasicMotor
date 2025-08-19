package frc.robot.subsystems.flywheel;


import edu.wpi.first.wpilibj.RobotBase;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.ctre.talonfx.BasicTalonFX;
import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;

public class FlywheelIOBasic implements  FlywheelIO {
    private final BasicMotor motor;

    public FlywheelIOBasic() {
        this.motor = RobotBase.isReal() ? new BasicTalonFX(FlywheelConstants.motorConfig) :
                        new BasicMotorSim(FlywheelConstants.motorConfig);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.velocityMetersPerSecond = motor.getVelocity();
        inputs.atTarget = motor.atSetpoint() && motor.getController().getControlMode().requiresPID();
    }

    @Override
    public void setTargetVelocity(double targetMetersPerSecond) {
        motor.setControl(targetMetersPerSecond, ControlMode.VELOCITY);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setPrecentOutput(double percentOutput) {
        motor.setPrecentOutput(percentOutput);
    }
}
