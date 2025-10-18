package io.github.captainsoccer.basicmotor.errorHandling;

import edu.wpi.first.wpilibj.Timer;

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
         * The next message in the list
         */
        MessageNode next = null;

        /**
         * Creates the node of the message with the length of the message
         * @param messageLength the length of the message string
         */
        private MessageNode(int messageLength) {
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
     * The first message node (the oldest)
     */
    private MessageNode firstMessage = null;

    /**
     * THe newest message node (the youngest)
     */
    private MessageNode lastMessage = null;

    /**
     * Adds a message to the string and node
     * @param msg the message to add (automatically gets enclosed in brackets for readability)
     */
    public void addMessage(String msg) {
        String messageStr = "[" + msg + "]";

        if(messages.toString().contains(messageStr))
            return;

        messages.append(messageStr);

        MessageNode newMessage = new MessageNode(messageStr.length());

        if (firstMessage == null) {
            firstMessage = newMessage;
            lastMessage = firstMessage;
        } else {
            lastMessage.next = newMessage;
            lastMessage = lastMessage.next;
        }
    }

    /**
     * Updates the messages
     * should be called periodically, unless there are no messages to update
     * Use {@link #isEmpty()} to check
     */
    public void updateMessages() {
        int removedLength = 0;

        double currentTime = Timer.getTimestamp();

        while (firstMessage != null && currentTime - firstMessage.startTime <= MESSAGE_DISPLAY_SECONDS) {
            removedLength += firstMessage.messageLength;
            firstMessage = firstMessage.next;
        }

        if(removedLength > 0) {
            messages.delete(0, removedLength);
        }

        if(firstMessage == null || firstMessage.next == null) {
            lastMessage = null;
        }
    }

    /**
     * Cheks if there are any messages to update
     * @return whether there are any messages
     */
    public boolean isEmpty() {
        return firstMessage == null;
    }

    /**
     * Gets the messages
     * @return a string of all the active messages
     */
    public String getMessages() {
        return messages.toString();
    }
}
