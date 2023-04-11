/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package tileworld.planners;

import java.util.ArrayList;

import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.agent.TWAgent;
import tileworld.environment.TWDirection;

/**
 * DefaultTWPlanner
 *
 * @author michaellees
 * Created: Apr 22, 2010
 *
 * Copyright michaellees 2010
 *
 * Here is the skeleton for your planner. Below are some points you may want to
 * consider.
 *
 * Description: This is a simple implementation of a Tileworld planner. A plan
 * consists of a series of directions for the agent to follow. Plans are made,
 * but then the environment changes, so new plans may be needed
 *
 * As an example, your planner could have 4 distinct behaviors:
 *
 * 1. Generate a random walk to locate a Tile (this is triggered when there is
 * no Tile observed in the agents memory
 *
 * 2. Generate a plan to a specified Tile (one which is nearby preferably,
 * nearby is defined by threshold - @see TWEntity)
 *
 * 3. Generate a random walk to locate a Hole (this is triggered when the agent
 * has (is carrying) a tile but doesn't have a hole in memory)
 *
 * 4. Generate a plan to a specified hole (triggered when agent has a tile,
 * looks for a hole in memory which is nearby)
 *
 * The default path generator might use an implementation of A* for each of the behaviors
 *
 */
public class DefaultTWPlanner implements TWPlanner {
	private ArrayList<Int2D> goals;
	private TWPath plan;
	private TWAgent agent;
	private AstarPathGenerator pathGenerator;

	public DefaultTWPlanner(TWAgent agent) {
		this.agent = agent;
		this.plan = null;
		this.goals = new ArrayList<Int2D>(0);
		this.pathGenerator = new AstarPathGenerator(agent.getEnvironment(), agent, Parameters.xDimension + Parameters.yDimension);
	}
	
//	public TWPath generatePlan(Int2D startPos, Int2D endPos) {
//		voidPlan();
//		// this.plan = pathGenerator.findPath(startPos.getX(), startPos.getY(), goals.get(0).x, goals.get(0).y);
//		this.plan = pathGenerator.findPath(startPos.getX(), startPos.getY(), endPos.getX(), endPos.getY());
//
//		return plan;
//	}
	
//	@Override
	public TWPath generatePlan() {
		assert(!goals.isEmpty());
		assert(this.agent.getEnvironment().isInBounds(goals.get(0).x, goals.get(0).y));
		this.plan = pathGenerator.findPath(agent.getX(), agent.getY(), goals.get(0).x, goals.get(0).y);
		return plan;
	}

	@Override
	public boolean hasPlan() {
		return (plan != null) && plan.hasNext() ? true : false;
	}

	@Override
	public void voidPlan() {
		plan = null;
		// System.out.println(agent.getName() + " voided current plan.");
	}

	public ArrayList<Int2D> getGoals() {
		return goals;
	}
	
	public void voidGoals() {
		goals.clear();
	}
	
	@Override
	public Int2D getCurrentGoal() {
		if (goals.isEmpty())
			return null;
		else
			return goals.get(0);
	}

	@Override
	public TWDirection execute() {
		return plan.popNext().getDirection();
	}

	public void printPlan() {
		System.out.println("path: "+plan);
		// TODO Auto-generated method stub
	}
}

//	public TWPath generatePlan() {
//	throw new UnsupportedOperationException("Not supported yet.");
//	}
//    public boolean hasPlan() {
//        throw new UnsupportedOperationException("Not supported yet.");
//    }
//
//    public void voidPlan() {
//        throw new UnsupportedOperationException("Not supported yet.");
//    }
//
//    public Int2D getCurrentGoal() {
//        throw new UnsupportedOperationException("Not supported yet.");
//    }
//
//    public TWDirection execute() {
//        throw new UnsupportedOperationException("Not supported yet.");
//    }


