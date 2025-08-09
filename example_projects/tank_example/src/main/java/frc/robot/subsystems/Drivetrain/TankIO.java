package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import org.littletonrobotics.junction.AutoLog;

public interface TankIO {
    @AutoLog
    class TankInputs {
        public DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds();

        public DifferentialDriveWheelPositions wheelPositions = new DifferentialDriveWheelPositions(0, 0);

        public Rotation2d gyroAngle = new Rotation2d();

        public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    }

    /**
     * Updates the inputs for the tank drive system.
     * The function will insert the latest values to the objects in the inputs class.
     * @param inputs The inputs to update.
     */
    void updateInputs(TankInputs inputs);

    /**
     * Sets the target speeds for the tank drive system.
     * This function is used to set the desired speeds for the left and right wheels.
     * @param targetSpeeds The target speeds for the left and right wheels.
     */
    void setTargetSpeeds(DifferentialDriveWheelSpeeds targetSpeeds);

    /**
     * Sets the voltage for the left and right motors of the tank drive system.
     * This function is used to control the speed of the motors directly by applying a voltage.
     * @param leftVoltage The voltage to apply to the left motors.
     * @param rightVoltage The voltage to apply to the right motors.
     */
    void setVoltage(double leftVoltage, double rightVoltage);
}
