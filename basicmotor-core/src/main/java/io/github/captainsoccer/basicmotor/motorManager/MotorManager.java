package io.github.captainsoccer.basicmotor.motorManager;

import io.github.captainsoccer.basicmotor.errorHandling.ErrorHandler;
import io.github.captainsoccer.basicmotor.LogFrame;
import io.github.captainsoccer.basicmotor.BasicMotor;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/**
 * This class is used to manage the {@link BasicMotor} instances.
 * It handles each motor's threads for the PID loop and sensor loop.
 * It also periodically logs the data of the motors.
 * You need to call the {@link #periodic()} method in the periodic method of your robot.java.
 * If you want to change some of the default values or the frequency of the loops, check {@link MotorManagerConfig}.
 */
public class MotorManager {

    /**
     * The configuration for the motor manager.
     * This stores the default values for some of the parameters of the motor manager.
     * It Also stores the frequency of the sensor loop and the PID loop.
     * If you want to change one of the parameters, You must change the value before constructing the motors.
     * This is static to make simpler calls to the motor manager.
     */
    public static MotorManagerConfig config = MotorManagerConfig.DEFAULT_CONFIG;

    /**
     * The singleton instance of the MotorManager.
     */
    private static MotorManager instance;

    /**
     * The map that stores the motors and their functions.
     * This map uses motor names (as strings) as keys and MotorFunctions objects as values.
     */
    private final Map<String, MotorFunctions> motorMap = new HashMap<>(20);
    //The current max number of motors in an FRC robot is 20, so we set the initial capacity to 20.

    /**
     * Private constructor to enforce singleton pattern.
     * Use {@link #getInstance()} to get the instance of the MotorManager.
     */
    private MotorManager() {
    }

    /**
     * Gets the singleton instance of the MotorManager.
     *
     * @return The instance of the MotorManager.
     */
    public static MotorManager getInstance() {
        if (instance == null) {
            instance = new MotorManager();
        }
        return instance;
    }

    /**
     * The periodic method that should be called in the periodic method of your robot.java.
     * This method sends all the motor data to the logger.
     * This method is called with the codes periodic due to the logger not supporting multithreading.
     */
    public void periodic() {
        // goes through all the motors and logs their data.
        for (Map.Entry<String, MotorFunctions> entry : motorMap.entrySet()) {
            var name = entry.getKey();

            LogFrame.LogFrameAutoLogged frame = entry.getValue().frameSupplier.get();

            Logger.processInputs("Motors/" + name, frame);

            ErrorHandler.ErrorLogFrame errorFrame = entry.getValue().errorFrameSupplier.get();
            Logger.processInputs("Motors/" + name + "/Messages", errorFrame);
        }
    }

    /**
     * Stops all the motor loops.
     */
    public void stopAllMotorLoops() {
        for (MotorFunctions motor : motorMap.values()) {
            motor.thread.stop();
        }
    }

    /**
     * Sets the location of the controller for a specific motor.
     * This updates the motor's main loop to run at the specified frequency.
     * This should not be used in the user code!,
     * only by the {@link BasicMotor#setControllerLocation(ControllerLocation)} function.
     * @param name The name of the motor to set the location for.
     * @param location The location of the pid loop.
     */
    public void setControllerLocation(String name, ControllerLocation location) {
        var functions = motorMap.get(name);
        if (functions != null) {
            functions.thread.setMainLoopTiming(location.getSeconds());
        } else {
            DriverStation.reportError("Motor with name " + name + " not found in MotorManager." +
                    "\n did you call outside the basicMotor class?", false);
        }
    }

    /**
     * Registers a motor with the motor manager.
     * Use only once per motor.
     *
     * @param name               The name of the motor. (will be used in the logs)
     * @param run                The function to run for the main loop.
     * @param sensorLoopFunction The function to run for the sensor loop.
     * @param frameSupplier      The function to get the latest frame for the motor.
     * @param errorFrameSupplier The function to get the latest error frame for the motor.
     */
    public void registerMotor(
            String name,
            Runnable run,
            Runnable sensorLoopFunction,
            Supplier<LogFrame.LogFrameAutoLogged> frameSupplier,
            Supplier<ErrorHandler.ErrorLogFrame> errorFrameSupplier) {

        var motorProcess = new MotorProcess(run, sensorLoopFunction, name);

        var functions = new MotorFunctions(motorProcess, frameSupplier);

        // since this function only happens in the start of the robot code and is on the main thread, throws the exception if the motor name already exists.
        if (motorMap.containsKey(name)){
            throw new IllegalArgumentException("Motor with name " + name + " already exists.");
        }

        motorMap.put(name, functions);
    }

    /**
     * This class stores the threads for the main loop and the sensor loop for a motor.
     * It also stores the frame supplier and the name of the motor.
     */
    public static class MotorFunctions{

        /**
         * The object that handles the main loop and the sensor loop for the motor.
         */
        private final MotorProcess thread;

        /**
         * The supplier for the frame of the motor.
         */
        private final Supplier<LogFrame.LogFrameAutoLogged> frameSupplier;

        /**
         * The supplier for the error frame of the motor.
         */
        private final Supplier<ErrorHandler.ErrorLogFrame> errorFrameSupplier;

        /**
         * Constructor for the MotorFunctions class.
         * This creates the threads for the main loop and the sensor loop.
         * But does not start them.
         * @param motorProcess The motor process that handles the loops.
         * @param frameSupplier The function to get the latest frame for the motor.
         * @param errorFrameSupplier The function to get the latest error frame for the motor.
         */
        public MotorFunctions(MotorProcess motorProcess, Supplier<LogFrame.LogFrameAutoLogged> frameSupplier) {

            this.thread = motorProcess;

            this.frameSupplier = frameSupplier;
            this.errorFrameSupplier = errorFrameSupplier;
        }
    }

    /**
     * An enum that represents the location of the pid controller.
     * This is used to determine how fast the pid loop runs.
     * Check the specific location for more information.
     */
    public enum ControllerLocation {
        /**
         * The pid controller is running on the motor controller.
         * This means that the code that is running on the rio is only checking for constraints, feedforwards and motion profiling.
         * This is the default location for most motors.
         * For most mechanisms, this is the best location to run the pid controller.
         * But it lacks control over the measurement source of the pid controller.
         * Takes the frequency from the {@link MotorManagerConfig#PROFILE_LOOP_HZ} which is the default profile loop frequency.
         */
        MOTOR(() -> config.PROFILE_LOOP_HZ),
        /**
         * The pid controller is running on the rio.
         * This means that the code on the rio is calculating everything and sending the output to the motor controller.
         * This is useful when you want to have more control over the measurement source of the pid controller.
         * But it can lead to more canbus traffic and robo rio cpu usage.
         * Takes the frequency from the {@link MotorManagerConfig#PID_LOOP_HZ} which is the default pid loop frequency.
         */
        RIO(() -> config.PID_LOOP_HZ);

        /**
         * The supplier of the frequency of the controller loop in Hz.
         */
        private final DoubleSupplier hzSupplier;

        /**
         * Constructor for the ControllerLocation enum.
         *
         * @param hzSupplier The supplier of the frequency of the controller loop in Hz.
         */
        ControllerLocation(DoubleSupplier hzSupplier) {
            this.hzSupplier = hzSupplier;
        }

        /**
         * Gets the frequency of the controller loop in Hz.
         *
         * @return The frequency of the controller loop in Hz.
         */
        public double getHZ() {
            return hzSupplier.getAsDouble();
        }

        /**
         * Gets the time in seconds for one iteration of the controller loop.
         * @return The time in seconds for one iteration of the controller loop.
         */
        public double getSeconds() {
            return 1.0 / hzSupplier.getAsDouble();
        }
    }
}
