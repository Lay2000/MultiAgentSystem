package tileworld.agent;

import java.util.HashMap;
import java.util.Map;

import sim.util.Bag;
import sim.util.Int2D;

public class Message {
	private String from; // the sender
	private String to; // the recepient
	Map<String, Object> message;
	
	public Message(String from, String to) {
		this.from = from;
		this.to = to;
		this.message = new HashMap<String, Object>();
	}
	
	public Message(String from, String to, Map<String, Object> message){
		this.from = from;
		this.to = to;
		this.message = message;
	}

	public String getFrom() {
		return from;
	}

	public String getTo() {
		return to;
	}

	public Map<String, Object> getMessage() {
		return this.message;
	}
	
	public void addMessage(String Key, Object Value) {
		this.message.put(Key, Value);
	}
	
	public void addFuelStationPosition(Int2D fuelStationPosition) {
		this.addMessage("fuelStationPosition", fuelStationPosition);
	}
	
	public Int2D getFuelStationPosition() {
		if (this.message.containsKey("fuelStationPosition")) {
			return (Int2D) this.message.get("fuelStationPosition");
		}
		else return null;
	}
	
	public void addSensedObjects(Bag sensedObjects, Int2D agentPosition) {
		this.addMessage("sensedObjects", sensedObjects);
		this.addMessage("agentPosition", agentPosition);
	}
	
	public Bag getSensedObjects() {
		if (this.message.containsKey("sensedObjects")) {
			return (Bag) this.message.get("sensedObjects");
		}
		else return null;
	}
	
	public Int2D getAgentPosition() {
		if (this.message.containsKey("agentPosition")) {
			return (Int2D) this.message.get("agentPosition");
		}
		else return null;
	}
}
