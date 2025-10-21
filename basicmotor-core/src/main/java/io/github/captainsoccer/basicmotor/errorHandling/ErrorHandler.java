package io.github.captainsoccer.basicmotor.errorHandling;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.Arrays;

/**
 * Handles error and warning for each motor.
 * Uses {@link MessageHandler} to store and update messages.
 */
public class ErrorHandler {
    /** The message handler for errors */
    private final MessageHandler errorHandler = new MessageHandler();
    /** The message handler for warnings */
    private final MessageHandler warningHandler = new MessageHandler();

    /** The error log frame used for logging */
    private final ErrorLogFrame frame = new ErrorLogFrame();

    /**
     * The name of the motor or system this error handler is for.
     * Used for driverStation reporting.
     */
    public final String name;

    /**
     * Creates an ErrorHandler for the given motor or system name
     * @param name the name of the motor or system this error handler is for
     */
    public ErrorHandler(String name) {
        this.name = name;
    }

    /**
     * Logs an error message
     * @param msg the error message to log (automatically adds timestamp and brackets)
     */
    public void logError(String msg) {
        errorHandler.addMessage(msg);
    }

    /**
     * Logs an error message and reports it to the driver station
     * message will look like: "motor [name] had error: [msg]"
     * @param msg the error message to log (automatically adds timestamp and brackets)
     * @param printStackTrace whether to print the stack trace to the driver station
     */
    public void logAndReportError(String msg, boolean printStackTrace) {
        logError(msg);
        if(printStackTrace) {
            DriverStation.reportError("motor " + name + " had error: " + msg, getStackTrace());
        }
        else{
            DriverStation.reportError("motor " + name + " had error: " + msg, false);
        }

    }

    /**
     * Logs an error message and reports it to the driver station
     * message will look like: "motor [name] had error: [msg]"
     * @param msg the error message to log (automatically adds timestamp and brackets)
     */
    public void logAndReportError(String msg) {
        logAndReportError(msg, false);
    }

    /**
     * Logs a warning message
     * @param msg the warning message to log (automatically adds timestamp and brackets)
     */
    public void logWarning(String msg) {
        warningHandler.addMessage(msg);
    }

    /**
     * Logs a warning message and reports it to the driver station
     * message will look like: "motor [name] had warning: [msg]"
     * @param msg the warning message to log (automatically adds timestamp and brackets)
     * @param printStackTrace whether to print the stack trace to the driver station
     */
    public void logAndReportWarning(String msg, boolean printStackTrace) {
        logWarning(msg);

        if(printStackTrace) {
            DriverStation.reportWarning("motor " + name + " had warning: " + msg, getStackTrace());
        }
        else{
            DriverStation.reportWarning("motor " + name + " had warning: " + msg, false);
        }
    }

    /**
     * Gets the current stack trace, excluding the first two elements (getStackTrace and this method)
     * @return the current stack trace
     */
    private StackTraceElement[] getStackTrace() {
        var stackTrace = Thread.currentThread().getStackTrace();
        return Arrays.copyOfRange(stackTrace, 2, stackTrace.length);
    }

    /**
     * Logs a warning message and reports it to the driver station
     * message will look like: "motor [name] had warning: [msg]"
     * @param msg the warning message to log (automatically adds timestamp and brackets)
     */
    public void logAndReportWarning(String msg) {
        logAndReportWarning(msg, false);
    }

    /** Updates the error and warning messages */
    private String getErrors() {
        if(errorHandler.isEmpty())
            return "";

        errorHandler.updateMessages();
        return errorHandler.getMessages();
    }

    /** Updates the warning messages */
    private String getWarnings() {
        if(warningHandler.isEmpty())
            return "";

        warningHandler.updateMessages();
        return warningHandler.getMessages();
    }

    /**
     * Gets the error log frame for logging
     * @return the error log frame
     */
    public ErrorLogFrame getErrorFrame() {
        frame.errors = getErrors();
        frame.warnings = getWarnings();
        return frame;
    }

    /**
     * The log frame for errors and warnings.
     * Used for logging.
     */
    public static class ErrorLogFrame implements LoggableInputs, Cloneable {
        /**
         * The error messages
         */
        public String errors = "";
        /**
         * The warning messages
         */
        public String warnings = "";

        @Override
        public void toLog(LogTable table) {
            table.put("Errors", errors);
            table.put("Warnings", warnings);
        }

        @Override
        public void fromLog(LogTable table) {
            //Does nothing, as we don't need to read errors/warnings from logs
        }
    }
}
