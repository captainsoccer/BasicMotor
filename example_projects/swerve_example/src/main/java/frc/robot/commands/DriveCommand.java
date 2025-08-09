package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.drivetrain.swerveModule.SwerveModuleConstants;

import java.util.function.DoubleSupplier;


public class DriveCommand extends Command {
    /**
     * The deadband for the inputs.
     * Any absolute value below this will be considered 0.
     */
    private static final double DEADBAND = 0.1; // Adjust deadband as needed

    /**
     * The radius of the drive train wheel base.
     * This assumes the wheels are placed in a circle, then calculates the distance of the front left module to the center
     */
    private static final double CHASSIS_RADIUS_METERS = SwerveModuleConstants.FRONT_LEFT.location.getNorm();

    /**
     * The minimum speed of the drive train in metres per second.
     * This will be the maximum speed of the robot when the brake is at 0.
     */
    private static final double MIN_SPEED = 1; // Minimum speed to consider for driving (metres per second)
    /**
     * The minumim Rotational speed of the chassis, defind by {@link #MIN_SPEED}
     */
    private static final double MIN_ROTATION_SPEED = MIN_SPEED / CHASSIS_RADIUS_METERS;

    /**
     * The maximum speed of the drive train in metres per second.
     * Will be the same as the maximum wheel speed of the swerve modules.
     */
    private static final double MAX_SPEED = SwerveModuleConstants.MAX_WHEEL_SPEED;
    /**
     * The maximum Rotational speed of the chassis, defind by {@link #MAX_SPEED}
     */
    private static final double MAX_ROTATION_SPEED = MAX_SPEED / CHASSIS_RADIUS_METERS;

    /**
     * The drive train subsystem that this command will control.
     */
    private final DriveTrain driveTrain;

    /**
     * The x velocity input supplier (forward/backward) (field relative) (forward is positive)
     */
    private final DoubleSupplier xVelocity;
    /**
     * The y velocity input supplier (left/right) (field relative) (left is positive)
     */
    private final DoubleSupplier yVelocity;
    /**
     * The rotation input supplier (counterClockwise is positive)
     */
    private final DoubleSupplier rotation;
    /**
     * This is the value the maximum speed will be multiplied by to determine the brake value.
     * When brake is 1 it will be the maximum speed, when brake is 0 it will be the minimum speed.
     */
    private final DoubleSupplier brake;

    /**
     * Creates a drive command that uses the given drive train and velocity inputs.
     * @param driveTrain The drive train subsystem to control
     * @param xVelocity The x velocity input supplier (forward/backward) (filed relative) (forward is positive)
     * @param yVelocity The y velocity input supplier (left/right) (field relative) (left is positive)
     * @param rotation The rotation input supplier (counterClockwise is positive)
     * @param brake The brake input supplier (0 is minimum speed, 1 is maximum speed)
     */
    public DriveCommand(DriveTrain driveTrain, DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier rotation, DoubleSupplier brake) {
        this.driveTrain = driveTrain;

        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.rotation = rotation;
        this.brake = brake;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain);
    }

    /**
     * Creates a drive command that uses the given drive train and PS5 controller inputs.
     * @param driveTrain The drive train subsystem to control
     * @param controller The PS5 controller to use for inputs
     */
    public DriveCommand(DriveTrain driveTrain, CommandPS5Controller controller) {
        this(driveTrain,
                () -> -controller.getLeftY(), // X velocity from left joystick)
                () -> -controller.getLeftX(), // Y velocity from left joystick
                () -> -controller.getRightX(), // Rotation from right joystick
                () -> 1 - controller.getR2Axis() // Brake value from R2 trigger
        );
    }

    /**
     * Creates a drive command that uses the given drive train and PS4 controller inputs.
     * @param driveTrain The drive train subsystem to control
     * @param controller The PS4 controller to use for inputs
     */
    public DriveCommand(DriveTrain driveTrain, CommandPS4Controller controller) {
        this(driveTrain,
                () -> -controller.getLeftY(), // X velocity from left joystick
                () -> -controller.getLeftX(), // Y velocity from left joystick
                () -> -controller.getRightX(), // Rotation from right joystick
                () -> 1 - controller.getR2Axis() // Custom brake value supplier
        );
    }

    /**
     * Creates a drive command that uses the given drive train and Xbox controller inputs.
     * @param driveTrain The drive train subsystem to control
     * @param controller The Xbox controller to use for inputs
     */
    public DriveCommand(DriveTrain driveTrain, CommandXboxController controller){
        this(driveTrain,
                () -> -controller.getLeftY(), // X velocity from left joystick
                () -> -controller.getLeftX(), // Y velocity from left joystick
                () -> -controller.getRightX(), // Rotation from right joystick
                () -> 1 - controller.getRightTriggerAxis() // Brake value from right trigger
        );
    }

    @Override
    public void initialize() {
        driveTrain.runVelocity(new ChassisSpeeds());
    }

    @Override
    public void execute() {
        double brake = this.brake.getAsDouble();

        double lerp = lerpVelocity(brake);

        double xVel = applyDeadband(xVelocity.getAsDouble()) * lerp;
        double yVel = applyDeadband(yVelocity.getAsDouble()) * lerp;
        double rot = applyDeadband(rotation.getAsDouble()) * lerpRotationVelocity(brake);

        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rot, driveTrain.getGyroRotation2d());

        driveTrain.runVelocity(robotRelativeSpeeds);
    }

    /**
     * Applies a deadband to the given value.
     * @param value The value to apply the deadband to
     * @return The value after applying the deadband, 0 if within deadband range
     */
    private static double applyDeadband(double value) {
        if (Math.abs(value) < DEADBAND) {
            return 0.0;
        }
        return value;
    }

    /**
     * Linearly interpolates the velocity based on the brake value.
     * This will scale the speed between the minimum and maximum speeds.
     * @param value The value to interpolate, 0 to 1
     * @return  The interpolated velocity in metres per second
     */
    private static double lerpVelocity(double value) {
        return MIN_SPEED + (MAX_SPEED - MIN_SPEED) * value;
    }

    /**
     * Linearly interpolates the velocity based on the brake value.
     * This will scale the speed between the minimum angular velocity and maximum angular velocity.
     * @param value The value to interpolate, 0 to 1
     * @return  The interpolated velocity in metres per second
     */
    public static double lerpRotationVelocity(double value){
        return MIN_ROTATION_SPEED + (MAX_ROTATION_SPEED - MIN_ROTATION_SPEED) * value;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.runVelocity(new ChassisSpeeds());
    }
}
