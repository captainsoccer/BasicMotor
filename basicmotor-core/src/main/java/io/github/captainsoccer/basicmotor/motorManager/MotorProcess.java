package io.github.captainsoccer.basicmotor.motorManager;

import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.RobotController;

import java.util.concurrent.locks.ReentrantLock;

/**
 * This class manages the motor control and sensor loops using a notifier.
 * It runs the main loop and sensor loop at specified intervals.
 */
public class MotorProcess {

    /**
     * The thread that runs the motor process loop.
     */
    private final Thread motorProcessThread;

    /**
     * The main loop to be executed periodically.
     */
    private final Runnable mainLoop;

    /**
     * The sensor loop to be executed periodically.
     */
    private final Runnable sensorLoop;

    /**
     * The period of the sensor loop in microseconds.
     * Taken from {@link MotorManagerConfig#SENSOR_LOOP_HZ}.
     */
    private final long sensorLoopMicroSeconds;

    /**
     * The period of the main loop in microseconds.
     */
    private long mainLoopMicroSeconds;

    /**
     * Lock to protect access to the notifier and timing variables.
     */
    private final ReentrantLock lock = new ReentrantLock();

    /**
     * The notifier handle for scheduling alarms.
     */
    private final int notifier = NotifierJNI.initializeNotifier();

    /**
     * The next alarm time for the main loop.
     * This is in microseconds, relative to the fpga time.
     */
    private long mainLoopAlarmTime;

    /**
     * The next alarm time for the sensor loop.
     * This is in microseconds, relative to the fpga time.
     */
    private long sensorLoopAlarmTime;

    /**
     * Constructs a MotorProcess with specified main and sensor loops and main loop period.
     * @param mainLoop The main loop to run periodically.
     * @param sensorLoop The sensor loop to run periodically.
     * @param name The name of the motor used for the notifier.
     */
    public MotorProcess(Runnable mainLoop, Runnable sensorLoop, String name) {
        this.mainLoop = mainLoop;
        this.sensorLoop = sensorLoop;

        sensorLoopMicroSeconds = secondsToMicroseconds(1.0 / MotorManager.config.SENSOR_LOOP_HZ);

        this.mainLoopMicroSeconds = secondsToMicroseconds(MotorManager.ControllerLocation.MOTOR.getSeconds());

        motorProcessThread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted()) {
                loop();
            }
        });

        NotifierJNI.setNotifierName(notifier, name + " MotorProcess");
        motorProcessThread.setName(name + " MotorProcess Thread");

        // makes both of the loops run together in the set startup delay
        long delay = secondsToMicroseconds(MotorManager.config.STARTUP_DELAY_SECONDS) + RobotController.getFPGATime();

        mainLoopAlarmTime = delay;
        sensorLoopAlarmTime = delay;

        NotifierJNI.updateNotifierAlarm(notifier, delay);

        motorProcessThread.start();
    }

    /**
     * Converts seconds to microseconds.
     * @param seconds The time in seconds.
     * @return The time in microseconds.
     */
    private static long secondsToMicroseconds(double seconds) {
        return (long) (seconds * 1e6);
    }

    /**
     * Sets the timing for the main loop.
     * @param periodSeconds The period of the main loop in seconds.
     */
    public void setMainLoopTiming(double periodSeconds) {
        try {
            lock.lock();
            this.mainLoopMicroSeconds = secondsToMicroseconds(periodSeconds);
        } finally {
            lock.unlock();
        }
    }

    /**
     * Stops the motor process by canceling the notifier alarm.
     */
    public void stop() {
        try {
            lock.lock();
            NotifierJNI.cancelNotifierAlarm(notifier);
            NotifierJNI.cleanNotifier(notifier);

            motorProcessThread.interrupt();
        }
        finally {
            lock.unlock();
        }
    }

    /**
     * The function the thread runs in a loop.
     */
    private void loop() {
        try {
            lock.lock();

            // gets the notifier handle
            if(notifier == 0) {
                return;
            }

            // waits for the next alarm
            long currentTime = NotifierJNI.waitForNotifierAlarm(notifier);
            if (currentTime == 0) {
                return;
            }

            // check if the alarm was for the main loop
            if(currentTime >= mainLoopAlarmTime) {
                mainLoop.run();
                mainLoopAlarmTime = currentTime + mainLoopMicroSeconds;
            }

            // check if the alarm was for the sensor loop
            if(currentTime >= sensorLoopAlarmTime) {
                sensorLoop.run();
                sensorLoopAlarmTime = currentTime + sensorLoopMicroSeconds;
            }

            // sets the next alarm time to the earliest of the two loops
            NotifierJNI.updateNotifierAlarm(notifier, Math.min(mainLoopAlarmTime, sensorLoopAlarmTime));
        }
        finally {
            lock.unlock();
        }
    }
}
