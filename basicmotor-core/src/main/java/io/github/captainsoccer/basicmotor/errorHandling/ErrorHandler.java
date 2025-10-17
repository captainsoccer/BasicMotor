package io.github.captainsoccer.basicmotor.errorHandling;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

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
     * Logs an error message
     * @param msg the error message to log (automatically adds timestamp and brackets)
     */
    public void logError(String msg) {
        errorHandler.addMessage(msg);
    }

    /**
     * Logs an error message and optionally prints the stack trace
     * @param msg the error message to log (automatically adds timestamp and brackets)
     * @param printStackTrace whether to print the stack trace to the driver station
     */
    public void logError(String msg, boolean printStackTrace){
        logError(msg);

        if(printStackTrace)
            DriverStation.reportError(msg, true);
    }

    /**
     * Logs a warning message
     * @param msg the warning message to log (automatically adds timestamp and brackets)
     */
    public void logWarning(String msg) {
        warningHandler.addMessage(msg);
    }

    /**
     * Logs a warning message and optionally prints the stack trace
     * @param msg the warning message to log (automatically adds timestamp and brackets)
     * @param printStackTrace whether to print the stack trace to the driver station
     */
    public void logWarning(String msg, boolean printStackTrace){
        logWarning(msg);

        if(printStackTrace)
            DriverStation.reportWarning(msg, true);
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

    /** Gets the error log frame for logging */
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
        public String errors = "";
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
