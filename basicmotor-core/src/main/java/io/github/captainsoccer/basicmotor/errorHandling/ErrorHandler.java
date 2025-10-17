package io.github.captainsoccer.basicmotor.errorHandling;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 *
 */
public class ErrorHandler {
    private final MessageHandler errorHandler = new MessageHandler();
    private final MessageHandler warningHandler = new MessageHandler();

    private final ErrorLogFrame frame = new ErrorLogFrame();

    public void logError(String msg) {
        errorHandler.addMessage(msg);
    }

    public void logError(String msg, boolean printStackTrace){
        logError(msg);

        if(printStackTrace)
            DriverStation.reportError(msg, true);
    }

    public void logWarning(String msg) {
        warningHandler.addMessage(msg);
    }

    public void logWarning(String msg, boolean printStackTrace){
        logWarning(msg);

        if(printStackTrace)
            DriverStation.reportWarning(msg, true);
    }

    private String getErrors() {
        if(errorHandler.isEmpty())
            return "";

        errorHandler.updateMessages();
        return errorHandler.getMessages();
    }

    private String getWarnings() {
        if(warningHandler.isEmpty())
            return "";

        warningHandler.updateMessages();
        return warningHandler.getMessages();
    }

    public ErrorLogFrame getErrorFrame() {
        frame.errors = getErrors();
        frame.warnings = getWarnings();
        return frame;
    }

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
