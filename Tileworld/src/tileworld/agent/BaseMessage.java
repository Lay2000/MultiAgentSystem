package tileworld.agent;

import sim.util.Bag;
import sim.util.Int2D;

public class BaseMessage extends Message {
	Int2D fuelStation;
	Bag sensedObjects;
	Int2D posAgent;
	
	public BaseMessage(String from, String to) {
		super(from, to);

		this.fuelStation = fuelStation;
	}

	public Int2D getFuelStation() {
		return fuelStation;
	}
	
	public Bag getSensedObjects() {
		return sensedObjects;
	}
	
	public Int2D getPosAgent() {
		return posAgent;
	}

}
