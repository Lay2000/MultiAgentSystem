package tileworld.agent;

/**
 * Customized Message class for transferring a list of blocks.
 */
public class MessageCust extends Message {

    private Object[] messageContent;
    /**
     * Creates a message
     * @param from  sender's name.
     * @param to    receiver's name. 'All' if to all.
     * @param type  message type. 
     * @param content   message content.
     */
    public MessageCust(String from, String to, String type, Object[] content) {
        super(from, to, type);
        messageContent = content;
    }

    public Object[] getMessageContent() {
        return messageContent;
    }

}
