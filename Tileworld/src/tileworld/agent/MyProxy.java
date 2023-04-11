package tileworld.agent;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

import sim.util.Bag;
import sim.util.Int2D;
import sim.util.IntBag;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWObject;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.DefaultTWPlanner;



public class MyProxy extends TWAgent {
	
	private static final long serialVersionUID = 1L;
	
	private int count;
	private String name;
	private MyStorage storage;
	private DefaultTWPlanner dtPlanner;
	private AgentMode.Mode mode;
	private double fuelThreshold;
	
	TWObject neighborTile;
	TWObject neighborHole;
	
	public MyProxy(int count, String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos,ypos,env,fuelLevel);
        this.count = count;
        this.name = name;
        this.fuelThreshold = this.getFuelLevel() / 10 ;
        this.dtPlanner = new DefaultTWPlanner(this);
		this.sensor = new TWAgentSensor(this, Parameters.defaultSensorRange);
        this.storage = new MyStorage(this, env.schedule, env.getxDimension(), env.getyDimension());
	}
	
	
	public MyProxy(int xpos, int ypos, TWEnvironment env, double fuelLevel) {
		super(xpos, ypos, env, fuelLevel);
		this.dtPlanner = new DefaultTWPlanner(this);
	}
	
	
    public void communicate() {
     
    	Message msg = new Message(this.getName(), "");
    	Bag sensedObjects = new Bag();
    	IntBag objectXCoords = new IntBag();
    	IntBag objectYCoords = new IntBag();
    	this.storage.getMemoryGrid().getNeighborsMaxDistance(x, y, Parameters.defaultSensorRange, false, sensedObjects, objectXCoords, objectYCoords);

  		msg.addFuelStationPosition(this.storage.getFuelStation());
  		msg.addSensedObjects(sensedObjects, new Int2D(x, y));
  		this.getEnvironment().receiveMessage(msg);
 	
    }
    
   
	@Override
	protected TWThought think() {
	
		ArrayList<Message> messages = this.getEnvironment().getMessages();
		for (Message m : messages) {
  			if (m == null || m.message == null || m.getFrom() == this.getName()) continue;
  			Int2D fuelStationPos = m.getFuelStationPosition();
  			if (this.storage.getFuelStation() == null && fuelStationPos != null) {
  				this.storage.setFuelStation(fuelStationPos.x, fuelStationPos.y);
  			}
  			Bag sharedObjects = m.getSensedObjects();
  			Int2D posAgent = m.getAgentPosition();
  			if (sharedObjects != null) {
  				this.storage.merStorage(sharedObjects, posAgent);
  			}
  		}
    	
    	
    	PriorityQueue<TWEntity> neighborTiles;
    	PriorityQueue<TWEntity> neighborHoles;
    	
		neighborTiles = this.storage.getNearbyAllSortedObjects(x, y, 50, TWTile.class);
		neighborHoles = this.storage.getNearbyAllSortedObjects(x, y, 50, TWHole.class);
		
		
		ArrayList<TWEntity> tiles = Any_collision(neighborTiles, this.storage.neighbouringAgents);
		ArrayList<TWEntity> holes = Any_collision(neighborHoles, this.storage.neighbouringAgents);

		TWTile tileTarget = null;
		TWHole holeTarget = null;
		
		if (tiles != null) {
			tileTarget = (TWTile) tiles.get(0);
		}
		if (holes != null) {
			holeTarget = (TWHole) holes.get(0);
		}

		tileTarget = this.getMemory().getNearbyTile(x, y, 50);
		holeTarget = this.getMemory().getNearbyHole(x, y, 50);
		
		
	
		mode = AgentMode.Mode.EXPLORE;
		if (storage.getFuelStation() == null) {
			mode = AgentMode.Mode.FIND_FUEL_STATION;
		} else if (storage.getFuelStation() != null 
				&& (this.getFuelLevel() < this.fuelThreshold 
						|| this.getFuelLevel() * 0.9 < this.getDistanceTo(storage.getFuelStation().x, storage.getFuelStation().y))) {
			mode = AgentMode.Mode.REFUEL;
		} else if (this.getFuelLevel() < this.fuelThreshold){
			mode = AgentMode.Mode.WAIT;
			return new TWThought(TWAction.MOVE, TWDirection.Z);
		} else if (!(mode == AgentMode.Mode.FIND_FUEL_STATION)) {
			if (this.hasTile() && holeTarget != null) {
				mode = AgentMode.Mode.FILL;
			} else if (!this.hasTile() && tileTarget != null) {
				mode = AgentMode.Mode.COLLECT;
			} else mode = AgentMode.Mode.EXPLORE;
		} else mode = AgentMode.Mode.EXPLORE;

		
		Object curLocObj = this.storage.getMemoryGrid().get(x, y);
		if (curLocObj instanceof TWFuelStation &&
				 (this.fuelLevel < (0.75 * Parameters.defaultFuelLevel))) {
			return new TWThought(TWAction.REFUEL, null);
		} else if (curLocObj instanceof TWHole && 
				this.getEnvironment().canPutdownTile((TWHole)curLocObj, this) &&
				this.hasTile()) 
		{
				return new TWThought(TWAction.PUTDOWN, null);
	
		} else if (curLocObj instanceof TWTile &&
				this.getEnvironment().canPickupTile((TWTile) curLocObj, this) &&
				this.carriedTiles.size() < 3) 
		{
				return new TWThought(TWAction.PICKUP, null);
		} 
		

		if (mode == AgentMode.Mode.FIND_FUEL_STATION) {
			if (this.dtPlanner.getGoals().isEmpty()) {
				FuelGoal();
			}
			else {
				if (this.dtPlanner.getGoals().contains(new Int2D(this.x, this.y))) {
					int count = dtPlanner.getGoals().indexOf(new Int2D(this.x, this.y));
					if (count != -1) dtPlanner.getGoals().remove(0);
				}			
			}
			
			for (int i = 0; i < dtPlanner.getGoals().size(); i++) {
				System.out.println("Goals " + i + ": " + dtPlanner.getGoals().get(i));			
			}	
		}
		else {
			dtPlanner.voidGoals();
			dtPlanner.voidPlan();
			if (mode == AgentMode.Mode.REFUEL) {
				dtPlanner.getGoals().add(storage.getFuelStation());
			}  else if ((mode == AgentMode.Mode.FILL)){
				dtPlanner.getGoals().add(new Int2D(holeTarget.getX(), holeTarget.getY()));
			} else if ((mode == AgentMode.Mode.COLLECT)){
				dtPlanner.getGoals().add(new Int2D(tileTarget.getX(), tileTarget.getY()));
			} else if (mode == AgentMode.Mode.WAIT){
				return new TWThought(TWAction.MOVE, TWDirection.Z);
			} else if (mode == AgentMode.Mode.EXPLORE) {
				return RandomMoveThought();
			}	
		}
		
		for (int i = 0; i < dtPlanner.getGoals().size(); i++) {
		}	
		if (this.dtPlanner.getGoals().isEmpty()){
			return RandomMoveThought();
		}	
		
		this.dtPlanner.generatePlan();
		
		if (!dtPlanner.hasPlan()) {
			if (this.mode == AgentMode.Mode.FIND_FUEL_STATION) {
				Int2D newGoal = CreateNCell(dtPlanner.getGoals().get(0));
				dtPlanner.getGoals().set(0, newGoal);
				dtPlanner.generatePlan();
			} else {
				return new TWThought(TWAction.MOVE, TWDirection.Z);
			}
		}
		if (!dtPlanner.hasPlan()) {
			return RandomMoveThought();
		} 
		
		TWDirection dir = this.dtPlanner.execute();
		return new TWThought(TWAction.MOVE, dir);
		

	}
private Int2D CreateNCell(Int2D goalPos) {
	  ArrayList<Int2D> seedi = new ArrayList<Int2D>();
	  int x = goalPos.getX();
	  int y = goalPos.getY();
	  
	  if ((y-1) >= 0 
			  && !this.storage.isCellBlocked(x, y-1)) {
		  seedi.add(new Int2D(x, y-1));
	  }
	  if ((y+1) < this.getEnvironment().getyDimension() 
			  && !this.storage.isCellBlocked(x, y+1)) {
		  seedi.add(new Int2D(x, y+1));
	  }
	  if ((x+1) < this.getEnvironment().getxDimension() 
			  && !this.storage.isCellBlocked(x+1, y)) {
		  seedi.add(new Int2D(x+1, y));
	  }
	  if ((x-1) >= 0 
			  && !this.storage.isCellBlocked(x-1, y)) {
		  seedi.add(new Int2D(x-1, y));
	  }
	  
	  if (seedi.size() > 0) {
		   int random_num = this.getEnvironment().random.nextInt(seedi.size());
		   return seedi.get(random_num);
	  }
	  else {
		  System.out.println("No where to go!");
		  return null;
	  }
	}
	
private Int2D getPositionAdd(Int2D base, Int2D position) {
		return new Int2D (base.x + position.x, base.y + position.y) ;
	}

public void FuelGoal() {
	
	Int2D[] basePos = {new Int2D(0, 0), 
			new Int2D(Parameters.xDimension/2, 0), 
			new Int2D(0, Parameters.yDimension/2), 
			new Int2D(Parameters.xDimension/2, Parameters.yDimension/2),
			new Int2D(Parameters.xDimension/2, Parameters.yDimension/2)};

	Int2D base = basePos[this.count];
	Int2D position = new Int2D (Parameters.defaultSensorRange, Parameters.defaultSensorRange);
	int depth = Parameters.defaultSensorRange;
	
	while(depth <= Parameters.xDimension/2) {
		int posX = position.x;
		int posY = position.y;
		dtPlanner.getGoals().add(getPositionAdd(base, position));
		posX = Parameters.xDimension/2 - Parameters.defaultSensorRange-1;
		position = new Int2D (posX, position.y);
		dtPlanner.getGoals().add(getPositionAdd(base, position));
		posY += Parameters.defaultSensorRange * 2+1;
		depth = depth + Parameters.defaultSensorRange * 2+1;
		if (depth > Parameters.xDimension/2) {
			break;
		}
		position = new Int2D (position.x, posY);
		dtPlanner.getGoals().add(getPositionAdd(base, position));
		posX = Parameters.defaultSensorRange;
		position = new Int2D (posX, position.y);
		dtPlanner.getGoals().add(getPositionAdd(base, position));
		depth = depth + (Parameters.defaultSensorRange * 2+1);
		posY += Parameters.defaultSensorRange * 2+1;
		position = new Int2D (position.x, posY);
	}
}

	private ArrayList<TWEntity> Any_collision(PriorityQueue<TWEntity> neighborObjects, List<TWAgent> neighbouringAgents) {
		ArrayList<TWEntity> tiles = new ArrayList<TWEntity>();
		while(!neighborObjects.isEmpty()) {
			TWEntity tile = neighborObjects.poll();
			for (int i = 0; i < neighbouringAgents.size(); i++) {
				if (!(neighbouringAgents.get(i) instanceof MyCheckProxy) 
						&& this.getDistanceTo(tile)/3 >= neighbouringAgents.get(i).getDistanceTo(tile)) {
					continue;
				} 
				tiles.add(tile);
			}
		}
	

		if (tiles.size() > 0) return tiles;
		else return null;
	}


	@Override
	protected void act(TWThought thought) {
		
		Int2D curGoal = dtPlanner.getCurrentGoal();
		
		try {
			switch (thought.getAction()) {
			case MOVE:
				move(thought.getDirection());
				break;
			case PICKUP:
				TWTile tile = (TWTile) storage.getMemoryGrid().get(this.x, this.y);
				pickUpTile(tile);
				dtPlanner.getGoals().clear();
				break;
			case PUTDOWN:
				TWHole hole = (TWHole) storage.getMemoryGrid().get(this.x, this.y);
				putTileInHole(hole);
				dtPlanner.getGoals().clear();
				break;
			case REFUEL:
				refuel();
				dtPlanner.getGoals().clear();
				break;
			}
            
        } catch (CellBlockedException ex) {
    		System.out.println("Goal_size: "+this.dtPlanner.getGoals().size());
    		System.out.println("North: " + this.storage.isCellBlocked(x, y-1));
    		System.out.println("South: " + this.storage.isCellBlocked(x, y+1));
    		System.out.println("East: " + this.storage.isCellBlocked(x+1, y));
    		System.out.println("West: " + this.storage.isCellBlocked(x-1, y));
			System.out.println("Blocked" + Integer.toString(this.x) + ", " + Integer.toString(this.y));
        }
		System.out.print(name + " score: " + this.score);
		System.out.print(" coordinate: " + "[" + Integer.toString(this.x) + ", " + Integer.toString(this.y)+ "]");
		System.out.print(" carry tiles: " + this.carriedTiles.size());
		System.out.print(" fuel level: " + this.fuelLevel);
		System.out.print(" fuel station: " + this.storage.getFuelStation());
		System.out.println("");

	}
	private TWThought RandomMoveThought() {
	  ArrayList<TWDirection> seedi = new ArrayList<TWDirection>();
	  int x = this.getX();
	  int y = this.getY();
	  
	  if ((y-1) >= 0 
			  && !this.storage.isCellBlocked(x, y-1)) {
		  seedi.add(TWDirection.N);
	  }
	  if ((y+1) < this.getEnvironment().getyDimension() 
			  && !this.storage.isCellBlocked(x, y+1)) {
		  seedi.add(TWDirection.S);
	  }
	  if ((x+1) < this.getEnvironment().getxDimension() 
			  && !this.storage.isCellBlocked(x+1, y)) {
		  seedi.add(TWDirection.E);
	  }
	  if ((x-1) >= 0 
			  && !this.storage.isCellBlocked(x-1, y)) {
		  seedi.add(TWDirection.W);
	  }
	  
	  if (seedi.size() > 0) {
	   int random_num = this.getEnvironment().random.nextInt(seedi.size());
	   return new TWThought(TWAction.MOVE, seedi.get(random_num));
	  }
	  else {
	   System.out.println("no road");
	   return new TWThought(TWAction.MOVE, TWDirection.Z);
	  }
    }
	
	private TWDirection getRandomDirection(int X, int Y){
	
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
	@Override  
	public TWAgentWorkingMemory getMemory() {
        return this.storage;
    }
	@Override
	public String getName() {
		return name;
	}
		
	

}
