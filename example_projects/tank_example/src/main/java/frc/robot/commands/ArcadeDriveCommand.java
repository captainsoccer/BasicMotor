package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain.Tank;
import frc.robot.subsystems.Drivetrain.TankConstants;

import java.util.function.Supplier;

/**
 * This command implements arcade drive for a tank drive system.
 * This uses velocity, not precent output.
 */
public class ArcadeDriveCommand extends Command {
    /**
     * The max velocity for the tank drive system in meters per second.
     */
    private static final double MAX_VELOCITY = TankConstants.MAX_VELOCITY_METERS_PER_SECOND;
    /**
     * The min velocity for the tank drive system in meters per second.
     * This is used to prevent the robot from stopping completely when the joystick is at rest.
     */
    private static final double MIN_VELOCITY = 1;

    /**
     * The max angular velocity for the tank drive system in radians per second.
     * This is calculated based on the max velocity and the track width of the tank drive.
     */
    private static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (TankConstants.TRACK_WIDTH_METERS / 2);
    /**
     * The min angular velocity for the tank drive system in radians per second.
     * This is used to prevent the robot from stopping completely when the joystick is at rest.
     */
    private static final double MIN_ANGULAR_VELOCITY = MIN_VELOCITY / (TankConstants.TRACK_WIDTH_METERS / 2);

    /**
     * The tank drive system that this command controls.
     * This is the subsystem that will be used to run the drive command.
     */
    private final Tank tank;

    /**
     * Suppliers for the velocity, rotation, and brake inputs.
     * These are used to get the current values from the joystick or other input devices.
     * This returns a value between -1 and 1, where 0 is no input.
     */
    private final Supplier<Double> velocitySupplier;
    /**
     * The rotation supplier for the arcade drive command.
     * This is used to get the current rotation value from the joystick or other input devices.
     * This returns a value between -1 and 1, where 0 is no rotation.
     */
    private final Supplier<Double> rotationSupplier;

    /**
     * The brake supplier for the arcade drive command.
     * This is used to get the current brake value from the joystick or other input devices.
     * This returns a value between 0 and 1, where 0 is no brake and 1 is full brake.
     */
    private final Supplier<Double> brakeSupplier;

    /**
     * Constructor for the arcadeDriveCommand.
     * @param tank the tank drive system that this command controls
     * @param velocitySupplier The supplier for the velocity input. (This should return a value between -1 and 1, where 0 is no input)
     *                         positive is forward, negative is backward.
     * @param rotationSupplier The supplier for the rotation input. (This should return a value between -1 and 1, where 0 is no rotation)
     *                         positive is counter-clockwise, negative is clockwise.
     * @param brakeSupplier The supplier for the brake input. (This should return a value between 0 and 1, where 0 is no brake and 1 is full brake)
     *                      This is used to control the speed of the robot.
     */
    public ArcadeDriveCommand(Tank tank, Supplier<Double> velocitySupplier, Supplier<Double> rotationSupplier, Supplier<Double> brakeSupplier) {
        this.tank = tank;

        this.velocitySupplier = velocitySupplier;
        this.rotationSupplier = rotationSupplier;
        this.brakeSupplier = brakeSupplier;

        addRequirements(tank);
    }

    /**
     * Constructor for the arcadeDriveCommand.
     * @param tank the tank drive system that this command controls
     * @param controller The PS5 controller used to control the tank drive system.
     */
    public ArcadeDriveCommand(Tank tank, CommandPS5Controller controller){
        this(tank,
                () -> -controller.getLeftY(),
                () -> -controller.getRightX(),
                controller::getR2Axis);
    }

    /**
     * Constructor for the arcadeDriveCommand.
     * @param tank the tank drive system that this command controls
     * @param controller The PS4 controller used to control the tank drive system.
     */
    public ArcadeDriveCommand(Tank tank, CommandPS4Controller controller){
        this(tank,
                () -> -controller.getLeftY(),
                () -> -controller.getRightX(),
                controller::getR2Axis);
    }

    /**
     * Constructor for the arcadeDriveCommand.
     * @param tank the tank drive system that this command controls
     * @param controller The Xbox controller used to control the tank drive system.
     */
    public ArcadeDriveCommand(Tank tank, CommandXboxController controller){
        this(tank,
                () -> -controller.getLeftY(),
                () -> -controller.getRightX(),
                controller::getRightTriggerAxis);
    }

    @Override
    public void initialize() {
        // Initialization logic for the drive command
        tank.runVelocity(new ChassisSpeeds());
    }

    @Override
    public void execute() {
        // inverse the brake value to make it work like a throttle
        // where 0 is full speed and 1 is full brake
        double brake = 1 - brakeSupplier.get();

        double velocity = velocitySupplier.get() * lerpVelocity(brake);
        double rotation = rotationSupplier.get() * lerpAngularVelocity(brake);

        tank.runVelocity(new ChassisSpeeds(velocity, 0, rotation));
    }

    /**
     * Linearly interpolates the velocity based on the brake input.
     * @param brake The brake input value, where 0 is full speed and 1 is full brake.
     * @return The interpolated velocity in meters per second.
     */
    private static double lerpVelocity(double brake){
        return MIN_VELOCITY + (MAX_VELOCITY - MIN_VELOCITY) * brake;
    }

    /**
     * Linearly interpolates the angular velocity based on the brake input.
     * @param brake The brake input value, where 0 is full speed and 1 is full brake.
     * @return The interpolated angular velocity in radians per second.
     */
    private static double lerpAngularVelocity(double brake){
        return MIN_ANGULAR_VELOCITY + (MAX_ANGULAR_VELOCITY - MIN_ANGULAR_VELOCITY) * brake;
    }

    @Override
    public void end(boolean interrupted) {
        tank.runVelocity(new ChassisSpeeds());
    }
}
