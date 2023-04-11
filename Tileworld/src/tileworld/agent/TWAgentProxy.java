package tileworld.agent;

import java.awt.Color;
import java.util.ArrayList;
import sim.display.GUIState;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.portrayal.Inspector;
import sim.portrayal.LocationWrapper;
import sim.portrayal.Portrayal;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;

/**
 * TWHandleProxy
 *
 * @author michaellees
 * Created: Apr 21, 2010
 *
 * Copyright michaellees 2010
 *
 *
 * Description:
 *
 * Abstract class used for implementing Steppable.
 *
 */
public abstract class TWAgentProxy extends TWAgent implements Steppable {

    protected int score;

    public int getScore() {
        return score;
    }

    public TWAgentProxy(int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env,fuelLevel);
        this.score = 0;
        this.fuelLevel = fuelLevel;
        this.carriedTiles = new ArrayList<TWTile>();
        this.sensor = new TWAgentSensor(this, Parameters.defaultSensorRange);
        this.memory = new TWAgentWorkingMemory(this, env.schedule, env.getxDimension(), env.getyDimension());
    	
    }
    /**
     * Fuel level, automatically decremented once per move.
     */
    protected double fuelLevel;
    /**
     * List of carried tiles - will have a set capacity
     */
    protected ArrayList<TWTile> carriedTiles;
    /**
     * Sensor class, used for getting information about the environment.
     */
    protected TWAgentSensor sensor;
    /**
     * Memory which stores sensed facts in the form of tuples (see TWAgentMemoryFact)
     */
    protected TWAgentWorkingMemory memory;

    //THE THREE METHODS YOU SHOULD EXTEND - SENSE, THINK, ACT
    /**
     * Sense procedure of the agent simply stores observed objects into memory.
     *
     */
    public void sense() {
        sensor.sense();
    }
    
    public void communicate() {
        Message message = new Message("","");
        this.getEnvironment().receiveMessage(message); // this will send the message to the broadcast channel of the environment
    }

    /**
     * This is the heart of your implementation. This is where the agent can
     * use what it has sensed to make a decision. Don't put everything here -
     * you can add some other methods too.
     *
     */
    abstract protected TWThought think();

    /**
     * Act currently involves moving only, you should modify this.
     */
    abstract protected void act(TWThought thought);

    //----------------------------------------------------------------
    //----------------------------------------------------------------
    //OTHER METHODS YOU MAY WANT TO USE
    /**
     * Call this to move your agent in a specified direction
     */
    @Override
    protected void move(TWDirection d) throws CellBlockedException {
        if (fuelLevel <= 0) {
        	System.out.println("Agent ran out of fuel, Score: " + this.score);
            //Bad news, causes runtime exception
            //throw new InsufficientFuelException("Agent ran out of fuel, Score: " + this.score);
        } else {
        	moveDir(d);
        }
    }

    /**
     * Gets the fuel level of the agent
     * @return
     */
    public double getFuelLevel() {
        return fuelLevel;
    }


    /**
     * This method actually moves the agent in the currently selected direction.
     * You shouldn't modify this, be aware of the CellBlockedException
     * @throws CellBlockedException
     */
    protected void moveDir(TWDirection direction) throws CellBlockedException {
        //get current location
        int localX = this.getX();
        int localY = this.getY();
        int oldx = localX, oldy = localY;
        //alter position according to direction
        localX += direction.dx;
        localY += direction.dy;

        //update location in grid (can throw exception)
        if (this.getEnvironment().isCellBlocked(localX, localY)) {
            throw new CellBlockedException();
        } else {
            //think this is necessary with Object2DGrid
            this.getEnvironment().getAgentGrid().set(oldx, oldy, null);
            this.setLocation(localX, localY);
            //remove fuel, unless we stay still.
            if (direction != TWDirection.Z) {
                fuelLevel--;
            }
        }
    }

   
    /**
     * This is the portrayal for the agent. If you want a different coloured
     * agent you can modify this.
     *
     * @return
     */
    public static Portrayal getPortrayal() {
        //red filled box.
        return new TWAgentPortrayal(Color.blue, Parameters.defaultSensorRange) {

            @Override
            public Inspector getInspector(LocationWrapper wrapper, GUIState state) {
                // make the inspector
                return new AgentInspector(super.getInspector(wrapper, state), wrapper, state);
            }
        };
    }

    /**
     * Indicates if this agent has at least one tile carried
     * 
     * @return true if the agent is carrying at least 1 tile
     */
    public boolean hasTile() {
        return carriedTiles.size() > 0;
    }

    /**
     * Returns the working memory of this agent
     * @return working memory
     */
    public TWAgentWorkingMemory getMemory() {
        return memory;
    }

    /**
     * Update the agents location on the agent grid.
     * @param xpos
     * @param ypos
     */
    @Override
    protected void setLocation(int xpos, int ypos) {
        x = xpos;
        y = ypos;
        //Set location of entity when it's created
        this.getEnvironment().getAgentGrid().set(x, y, this);
    }
    
    /**
     * A name for the agent. Can be used for debugging and right now used for
     * memory portrayal
     * 
     */
    public abstract String getName();
}
