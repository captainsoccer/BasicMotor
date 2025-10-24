package io.github.captainsoccer.basicmotor.errorHandling;

import edu.wpi.first.wpilibj.Timer;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;

import java.util.LinkedList;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Handles messages used for logging
 * Automatically removes old messages.
 * supports advantage kit logging for easy logging.
 * Used by {@link ErrorHandler}
 * <p>
 * Messages are stored in a linked list that the first messages is always the oldest one.
 */
public class MessageHandler {
    /**
     * The node used to keep the messages in check
     */
    private static class MessageNode {
        /**
         * The length of this message, used to remove from the message builder
         */
        final int messageLength;

        /**
         * The timer that keeps track of how long the message has been playing
         */
        double startTime;

        /**
         * The hash code of the message, used to prevent duplicate messages
         */
        final int hashCode;

        /**
         * Creates the node of the message with the length of the message and the hash code
         *
         * @param messageLength the length of the message string
         * @param hashCode      the hash code of the message
         */
        private MessageNode(int messageLength, int hashCode) {
            this.hashCode = hashCode;
            this.messageLength = messageLength;
            this.startTime = Timer.getTimestamp();
        }
    }

    /**
     * The string builder that holds the messages
     */
    private final StringBuilder messages = new StringBuilder();

    /**
     * The list of messages
     * Used to keep track of the messages and their timers
     * And check for duplicates
     * The first message is always the oldest one
     */
    private final LinkedList<MessageNode> messageList = new LinkedList<>();

    /**
     * The lock used to synchronize access to the messages
     * Prevents concurrent modification exceptions
     */
    private final ReentrantLock lock = new ReentrantLock();

    /**
     * Adds a message to the list and the string builder
     *
     * @param msg the message to add (automatically gets enclosed in brackets for readability)
     */
    public void addMessage(String msg) {
        try {
            lock.lock();

            int messageHash = msg.hashCode();

            // check for existing message
            var existingMessage = hasCopy(messageHash);

            if (existingMessage != null){
                // reset the timer
                existingMessage.startTime = Timer.getTimestamp();
                return;
            }

            String messageStr = "[" + msg + "]";

            messages.append(messageStr);

            messageList.add(new MessageNode(messageStr.length(), messageHash));
        } finally {
            lock.unlock();
        }
    }

    /**
     * Gets the message node if it exists
     *
     * @param messageHash the hash of the message to check (without brackets)
     * @return the message node if it exists, null otherwise
     */
    private MessageNode hasCopy(int messageHash) {

        if(messageList.isEmpty())
            return null;

        for (var node : messageList) {
            if (node.hashCode == messageHash) {
                return node;
            }
        }

        return null;
    }

    /**
     * Updates the messages
     * should be called periodically, unless there are no messages to update
     * Use {@link #isEmpty()} to check
     */
    public void updateMessages() {
        try {
            int removeLength = 0;

            double currentTime = Timer.getTimestamp();

            double MESSAGE_DISPLAY_SECONDS = MotorManager.config.MESSAGE_DISPLAY_TIMEOUT_SECONDS;

            lock.lock();

            while (!messageList.isEmpty() && currentTime - messageList.peek().startTime > MESSAGE_DISPLAY_SECONDS) {
                // removes the rotten message
                var message = messageList.poll();
                removeLength += message.messageLength;
            }

            if (removeLength > 0) {
                messages.delete(0, removeLength);
            }

        } finally {
            lock.unlock();
        }
    }

    /**
     * checks if there are any messages to update
     *
     * @return true if there is any node existing, false otherwise
     */
    public boolean isEmpty() {
        return messageList.isEmpty();
    }

    /**
     * Gets the messages
     *
     * @return a string of all the active messages
     */
    public String getMessages() {
        return messages.toString();
    }
}
