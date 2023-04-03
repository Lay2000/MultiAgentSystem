package tileworld.agent;

/**
 * Customized Message class for transferring a list of blocks.
 */
public class Message {
	public enum msgType {
		mapInfo, goalInfo, contractInfo_tile, contractInfo_hole
	}

	private int sender;

	public int getSender() {
		return sender;
	}

	private int receiver;

	public int getReceiver() {
		return receiver;
	}

	private msgType messageType;

	public msgType getMessageType() {
		return messageType;
	}

	private Object[] messageContent;

	public Object[] getMessageContent() {
		return messageContent;
	}

	/**
	 * Create a customized message
	 * 
	 * @param sender         sender's id.
	 * @param receiver       receiver's id. -1 if this message is to all agents.
	 * @param messageType    one of the msgType, indicating message purpose.
	 * @param messageContent object list so that anything can fit into it.
	 */
	public Message(int sender, int receiver, msgType messageType, Object[] messageContent) {
		this.sender = sender;
		this.receiver = receiver;
		this.messageType = messageType;
		this.messageContent = messageContent;
	}

}
