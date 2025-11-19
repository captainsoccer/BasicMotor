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
     * Default value is taken from {@link MotorManagerConfig#SENSOR_LOOP_HZ}.
     */
    private long sensorLoopMicroseconds;

    /**
     * The period of the main loop in microseconds.
     */
    private long mainLoopMicroseconds;

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
     * @param mainLoopPeriodSeconds The period of the main loop in seconds, can be changed later.
     * @param sensorLoopPeriodSeconds The period of the sensor loop in seconds, can be changed later.
     */
    public MotorProcess(Runnable mainLoop, Runnable sensorLoop, String name, double mainLoopPeriodSeconds, double sensorLoopPeriodSeconds) {
        this.mainLoop = mainLoop;
        this.sensorLoop = sensorLoop;


        this.mainLoopMicroseconds = secondsToMicroseconds(mainLoopPeriodSeconds);

        this.sensorLoopMicroseconds = secondsToMicroseconds(sensorLoopPeriodSeconds);

        motorProcessThread = new Thread(() -> {
            // delays execution for the set amount of time
            try {
                Thread.sleep((long)(MotorManager.getConfig().STARTUP_DELAY_SECONDS * 1000));
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            //sets the next alarm time
            try{
                lock.lock();

                long currentTime = RobotController.getFPGATime();

                mainLoopAlarmTime = currentTime + mainLoopMicroseconds;
                sensorLoopAlarmTime = currentTime + sensorLoopMicroseconds;

                NotifierJNI.updateNotifierAlarm(notifier, Math.min(mainLoopAlarmTime, sensorLoopAlarmTime));
            }
            finally {
                lock.unlock();
            }

            while (!Thread.currentThread().isInterrupted()) {
                loop();
            }
        });

        NotifierJNI.setNotifierName(notifier, name + " MotorProcess");
        motorProcessThread.setName(name + " MotorProcess Thread");

        motorProcessThread.start();
    }

    /**
     * Constructs a MotorProcess with specified main and sensor loops and main loop period.
     * @param mainLoop The main loop to run periodically.
     * @param sensorLoop The sensor loop to run periodically.
     * @param name The name of the motor used for the notifier.
     *
     */
    public MotorProcess(Runnable mainLoop, Runnable sensorLoop, String name) {
        this(mainLoop, sensorLoop, name, MotorManager.ControllerLocation.MOTOR.getSeconds(),
                1.0 / MotorManager.getConfig().SENSOR_LOOP_HZ);
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
            this.mainLoopMicroseconds = secondsToMicroseconds(periodSeconds);
        } finally {
            lock.unlock();
        }
    }

    /**
     * Sets the timing for the sensor loop.
     * @param periodSeconds The period of the sensor loop in seconds.
     */
    public void setSensorLoopTiming(double periodSeconds) {
        try {
            lock.lock();
            this.sensorLoopMicroseconds = secondsToMicroseconds(periodSeconds);
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

            // checks if the notifier id is valid
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
                mainLoopAlarmTime = currentTime + mainLoopMicroseconds;
            }

            // check if the alarm was for the sensor loop
            if(currentTime >= sensorLoopAlarmTime) {
                sensorLoop.run();
                sensorLoopAlarmTime = currentTime + sensorLoopMicroseconds;
            }

            // sets the next alarm time to the earliest of the two loops
            NotifierJNI.updateNotifierAlarm(notifier, Math.min(mainLoopAlarmTime, sensorLoopAlarmTime));
        }
        finally {
            lock.unlock();
        }
    }
}
