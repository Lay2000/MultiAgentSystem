package tileworld.agent;

import sim.display.GUIState;
import sim.portrayal.Inspector;
import sim.portrayal.LocationWrapper;
import sim.portrayal.Portrayal;
import tileworld.Parameters;
import tileworld.environment.TWEntity;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEnvironment;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.*;

import java.awt.*;


public class MyTWAgent extends TWAgent{
    protected TWPathGenerator pathGenerator;
    private String name;
    private TWPath curPath = null;
    private boolean rethink = false;
    private String curGoal = "RANDOM";
    public MyTWAgent(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos,ypos,env,fuelLevel);
        this.name = name;
        this.pathGenerator = new AstarPathGenerator(this.getEnvironment(), this, 50);
    }

    protected TWThought think() {
        // getMemory().getClosestObjectInSensorRange(Tile.class);
        System.out.println("[" + this.getName() +"]" + "Current Goal: " + this.curGoal + ", Score: " + this.score + ", Current FuelLevel: " + this.getFuelLevel() + ", fuelX: " + this.memory.getFuelX());

        // If the current location is on an object: TWTile, TWHole or FuelStation, then do something
        Object tempObject = this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
        if (this.carriedTiles.size() < 3 && tempObject != null && tempObject instanceof TWTile){
            return new TWThought(TWAction.PICKUP);
        }
        if (this.carriedTiles.size() > 0 && tempObject != null && tempObject instanceof TWHole){
            return new TWThought(TWAction.PUTDOWN);
        }
        if (this.getEnvironment().inFuelStation(this) && this.getFuelLevel() <= 490){
            return new TWThought(TWAction.REFUEL);
        }

        // get a random direction as default
        TWDirection direction = getRandomDirection();

        if (this.curPath == null) {
            // Find a random point and set a path to it
            // Because if just move by random direction, the agent may just circle in a small area
            int targetX = 1;
            int targetY = 1;
            while (!this.getEnvironment().isValidCreationLocation(
                    targetX = this.getEnvironment().random.nextInt(this.getEnvironment().getxDimension()),
                    targetY = this.getEnvironment().random.nextInt(this.getEnvironment().getyDimension()))) {
            }
            this.curPath = this.pathGenerator.findPath(this.getX(), this.getY(), targetX, targetY);
            this.curGoal = "RANDOM";

            // If the state satisifies some conditions, then do something meanful. 
            // The priority: Go to refuel > Go to pickup > Go to put down

            // Go to refuel
            if (this.memory.getFuelX() != -1 && this.getFuelLevel() <= 100) {
                this.curPath = this.pathGenerator.findPath(this.getX(), this.getY(), this.memory.getFuelX(), this.memory.getFuelY());
                this.curGoal = "REFUEL";
            }

            // Go to pickup
            else if (this.carriedTiles.size() < 3 && this.getMemory().getNearbyTile(this.getX(), this.getY(), 10000) != null) {
                TWTile targetTile = this.getMemory().getNearbyTile(this.getX(), this.getY(), 5);
                this.curPath = this.pathGenerator.findPath(this.getX(), this.getY(), targetTile.getX(), targetTile.getY());
                this.curGoal = "PICKUP";
            }

            // Go to put down
            else if (this.carriedTiles.size() == 3 && this.getMemory().getNearbyHole(this.getX(), this.getY(), 10000) != null) {
                TWHole targetHole = this.getMemory().getNearbyHole(this.getX(), this.getY(), 5);
                this.curPath = this.pathGenerator.findPath(this.getX(), this.getY(), targetHole.getX(), targetHole.getY());
                this.curGoal = "PUTDOWN";
            }
        }

        // If there is a target position, and not rethink (CellBlockedException), then move following the curPath.
        // Otherwise, move in a random direction
        if (this.curPath != null && this.curPath.hasNext() && this.rethink == false) {
            TWPathStep nextStep = this.curPath.popNext();
            direction = nextStep.getDirection();
        }
        else if (this.curPath != null && !this.curPath.hasNext()) {
            this.curPath = null;
        }

        return new TWThought(TWAction.MOVE,direction);
        // There should be a Planner to Plan the Path, Instead of using getRandomDirection
    }

    @Override
    protected void act(TWThought thought) {
        Object tempObject = this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
        switch(thought.getAction()){

            case PICKUP:
                pickUpTile((TWTile) tempObject);
                this.getMemory().removeObject((TWEntity) tempObject);
                break;

            case PUTDOWN:
                putTileInHole((TWHole) tempObject);
                this.getMemory().removeObject((TWEntity) tempObject);
                break;

            case REFUEL:
                this.refuel();
                break;

            case MOVE:
                try {
                    this.move(thought.getDirection());
                } catch (CellBlockedException ex) {
                    this.rethink = true;
                    this.act(this.think());
                    this.rethink = false;
                }
        }
    }


    private TWDirection getRandomDirection(){

        TWDirection randomDir = TWDirection.values()[this.getEnvironment().random.nextInt(5)];

        if(this.getX()>=this.getEnvironment().getxDimension() ){
            randomDir = TWDirection.W;
        }else if(this.getX()<=1 ){
            randomDir = TWDirection.E;
        }else if(this.getY()<=1 ){
            randomDir = TWDirection.S;
        }else if(this.getY()>=this.getEnvironment().getxDimension() ){
            randomDir = TWDirection.N;
        }

       return randomDir;

    }

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
    @Override
    public String getName() {
        return name;
    }
}
