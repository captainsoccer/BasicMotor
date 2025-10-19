package io.github.captainsoccer.basicmotor.errorHandling;

import edu.wpi.first.wpilibj.Timer;

import java.util.LinkedList;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Handles messages used for logging
 * Automatically removes old messages.
 * supports advantage kit logging for easy logging.
 * Used by {@link ErrorHandler}
 *
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
        final double startTime;

        /**
         * The hash code of the message, used to prevent duplicate messages
         */
        final int hashCode;
        /**
         * Creates the node of the message with the length of the message
         * @param messageLength the length of the message string
         */
        private MessageNode(int messageLength, int hashCode) {
            this.hashCode = hashCode;
            this.messageLength = messageLength;
            this.startTime = Timer.getTimestamp();
        }
    }

    /**
     * The number of seconds a message is displayed for.
     */
    private static final int MESSAGE_DISPLAY_SECONDS = 2;

    /**
     * The string builder that holds the messages
     */
    private final StringBuilder messages = new StringBuilder();

    /**
     * The latest message added
     */
    private String latestMessage = "";

    /**
     * The list of messages
     */
    private final LinkedList<MessageNode> messageList = new LinkedList<>();

    /**
     * The lock used to synchronize access to the messages
     */
    private final ReentrantLock lock = new ReentrantLock();

    /**
     * Adds a message to the string and node
     * @param msg the message to add (automatically gets enclosed in brackets for readability)
     */
    public void addMessage(String msg) {
        try {
            lock.lock();

            int messageHash = msg.hashCode();

            // prevent duplicate messages
            if(containsMessage(messageHash))
                return;

            String messageStr = "[" + msg + "]";

            messages.append(messageStr);

            messageList.add(new MessageNode(messageStr.length(), messageHash));
        }
        finally {
            lock.unlock();
        }
    }

    /**
     * Checks if the message is already in the list
     * @param messageHash the hash of the message to check
     * @return whether the message is already in the list
     */
    private boolean containsMessage(int messageHash) {
        for(var node : messageList) {
            if(node.hashCode == messageHash) {
                return true;
            }
        }

        return false;
    }

    /**
     * Updates the messages
     * should be called periodically, unless there are no messages to update
     * Use {@link #isEmpty()} to check
     */
    public void updateMessages() {

        latestMessage = messages.toString();

        try {
            int removedLength = 0;

            double currentTime = Timer.getTimestamp();

            lock.lock();

            while (!messageList.isEmpty() && currentTime - messageList.peek().startTime > MESSAGE_DISPLAY_SECONDS) {
                var message = messageList.poll();
                removedLength += message.messageLength;
            }

            if(removedLength > 0) {
                messages.delete(0, removedLength);
            }
        }
        finally {
            lock.unlock();
        }
    }

    /**
     * Cheks if there are any messages to update
     * @return whether there are any messages
     */
    public boolean isEmpty() {
        return messageList.isEmpty();
    }

    /**
     * Gets the messages
     * @return a string of all the active messages
     */
    public String getMessages() {
        return latestMessage;
    }
}
