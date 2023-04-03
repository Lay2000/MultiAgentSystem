/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package tileworld.planners;

import java.security.Policy.Parameters;
import java.util.ArrayList;

import sim.util.Int2D;
import tileworld.agent.TWAgent;
import tileworld.environment.TWDirection;

/**
 * DefaultTWPlanner
 *
 * @author michaellees
 *         Created: Apr 22, 2010
 *
 *         Copyright michaellees 2010
 *
 *         Here is the skeleton for your planner. Below are some points you may
 *         want to
 *         consider.
 *
 *         Description: This is a simple implementation of a Tileworld planner.
 *         A plan
 *         consists of a series of directions for the agent to follow. Plans are
 *         made,
 *         but then the environment changes, so new plans may be needed
 *
 *         As an example, your planner could have 4 distinct behaviors:
 *
 *         1. Generate a random walk to locate a Tile (this is triggered when
 *         there is
 *         no Tile observed in the agents memory
 *
 *         2. Generate a plan to a specified Tile (one which is nearby
 *         preferably,
 *         nearby is defined by threshold - @see TWEntity)
 *
 *         3. Generate a random walk to locate a Hole (this is triggered when
 *         the agent
 *         has (is carrying) a tile but doesn't have a hole in memory)
 *
 *         4. Generate a plan to a specified hole (triggered when agent has a
 *         tile,
 *         looks for a hole in memory which is nearby)
 *
 *         The default path generator might use an implementation of A* for each
 *         of the behaviors
 *
 */
public class DefaultTWPlanner implements TWPlanner {

    private ArrayList<Int2D> goals;
    private TWPath plan;
    private TWAgent agent;
    private AstarPathGenerator pathGenerator;

    private int maxSearchx;
    private int maxSearchy;

    public DefaultTWPlanner(TWAgent agent) {
        this.agent = agent;
        this.plan = null;
        this.goals = new ArrayList<Int2D>(0);
        this.pathGenerator = new AstarPathGenerator(agent.getEnvironment(), agent,
                maxSearchx + maxSearchy);
    }

    // TSP resource-bounded time-limited scenario.
    // Rank nearby tiles and holes by both distance and expected time left.
    @Override
    public TWPath generatePlan() {
        plan = pathGenerator.findPath(agent.getX(), agent.getY(), goals.get(0).x, goals.get(0).y);
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

}
