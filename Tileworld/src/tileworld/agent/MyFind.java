package tileworld.agent;

import tileworld.environment.TWEnvironment;

public class MyFind extends TWAgent {

	private static final long serialVersionUID = 1L;

	public MyFind(int xpos, int ypos, TWEnvironment env, double fuelLevel) {
		super(xpos, ypos, env, fuelLevel);
	}

	@Override
	protected TWThought think() {
		return null;
	}

	@Override
	protected void act(TWThought thought) {
	}

	@Override
	public String getName() {
		return null;
	}
}
