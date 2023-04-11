package tileworld.agent;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
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
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.DefaultTWPlanner;


public class MyCheckProxy extends TWAgent {
	private static final long serialVersionUID = 1L;
	
	private int count;
	private String name;
	private MyStorage storage;
	private DefaultTWPlanner dtPlanner;
	private AgentMode.Mode mode;
	private double fuelThreshold;
	
	TWEntity neighborTile;
	TWEntity neighborHole;
	
	PriorityQueue<TWEntity> neighborTiles;
	PriorityQueue<TWEntity> neighborHoles;
	
	
	public MyCheckProxy(int count, String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos,ypos,env,fuelLevel);
        this.count = count;
        this.name = name;
        this.fuelThreshold = this.getFuelLevel() / 10 ;
        this.dtPlanner = new DefaultTWPlanner(this);
		this.sensor = new TWAgentSensor(this, Parameters.defaultSensorRange);
        this.storage = new MyStorage(this, env.schedule, env.getxDimension(), env.getyDimension());
	}
	
	
	public MyCheckProxy(int xpos, int ypos, TWEnvironment env, double fuelLevel) {
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
			  
		
		
		TWTile tileTarget = this.getMemory().getNearbyTile(x, y, 50);
		TWHole holeTarget = this.getMemory().getNearbyHole(x, y, 50);
		
		
		
		mode = AgentMode.Mode.INSPECT;
		if (storage.getFuelStation() == null) {
			mode = AgentMode.Mode.FIND_FUEL_STATION;
		} 
		else if (storage.getFuelStation() != null 
				&& (this.getFuelLevel() < this.fuelThreshold 
						|| this.getFuelLevel() * 0.9 < this.getDistanceTo(storage.getFuelStation().x, storage.getFuelStation().y))) {
			mode = AgentMode.Mode.REFUEL;
		} 
		else if (this.getFuelLevel() < this.fuelThreshold){
			mode = AgentMode.Mode.WAIT;
			return new TWThought(TWAction.MOVE, TWDirection.Z);
		} 

		Object curLocObj = this.storage.getMemoryGrid().get(x, y);
		if (curLocObj instanceof TWFuelStation &&
				 (this.fuelLevel < (0.75 * Parameters.defaultFuelLevel))) {
			dtPlanner.getGoals().add(0, new Int2D(this.x, this.y));
			return new TWThought(TWAction.REFUEL, null);
		} else if (curLocObj instanceof TWHole && 
				this.getEnvironment().canPutdownTile((TWHole)curLocObj, this) &&
				this.hasTile()) 
		{
				dtPlanner.getGoals().add(0, new Int2D(this.x, this.y));
				return new TWThought(TWAction.PUTDOWN, null);
	
		} else if (curLocObj instanceof TWTile &&
				this.getEnvironment().canPickupTile((TWTile) curLocObj, this) &&
				this.carriedTiles.size() < 3) 
		{
				dtPlanner.getGoals().add(0, new Int2D(this.x, this.y));
				return new TWThought(TWAction.PICKUP, null);
		} 

		if (mode == AgentMode.Mode.FIND_FUEL_STATION) {
			if (this.dtPlanner.getGoals().isEmpty()) {
				FuelGoal();
			}
			else {
				if (dtPlanner.getGoals().contains(new Int2D(this.x, this.y))) {
					int count = dtPlanner.getGoals().indexOf(new Int2D(this.x, this.y));
					if (count != -1) dtPlanner.getGoals().remove(count);
				}
			}	
		}
		
		else if (mode == AgentMode.Mode.INSPECT){

			if (this.dtPlanner.getGoals().size() > 20) {
				dtPlanner.voidGoals();
				dtPlanner.voidPlan();
			}
			if (this.dtPlanner.getGoals().isEmpty()) {
				InspectGoal();
				if (this.count == 1) {
					Collections.reverse(this.dtPlanner.getGoals()); 
				}
			}
			if (!dtPlanner.getGoals().isEmpty()) {
				if (dtPlanner.getGoals().contains(new Int2D(this.x, this.y))) {
					int temp = dtPlanner.getGoals().indexOf(new Int2D(this.x, this.y));
					if (count != -1) dtPlanner.getGoals().remove(temp);
				}			
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
		if (this.dtPlanner.getGoals().isEmpty()){
			return RandomMoveThought();
		}
		
		dtPlanner.generatePlan();
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
		
		TWDirection dir = dtPlanner.execute();
		return new TWThought(TWAction.MOVE, dir);
	}
	
private Int2D getPositionAdd(Int2D base, Int2D position) {
		return new Int2D (base.x + position.x, base.y + position.y) ;
	}
public void FuelGoal() {
	dtPlanner.voidGoals();
	dtPlanner.voidPlan();
	
	Int2D[] basePos = {new Int2D(0, 0), 
			new Int2D(Parameters.xDimension/2, 0), 
			new Int2D(0, Parameters.yDimension/2), 
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
		posY += Parameters.defaultSensorRange * 2 + 1;
		depth = depth + Parameters.defaultSensorRange * 2 + 1;
		if (depth > Parameters.xDimension/2) {
			break;
		}
		position = new Int2D (position.x, posY);
		dtPlanner.getGoals().add(getPositionAdd(base, position));
		posX = Parameters.defaultSensorRange;
		position = new Int2D (posX, position.y);
		dtPlanner.getGoals().add(getPositionAdd(base, position));
		depth = depth + (Parameters.defaultSensorRange * 2 + 1);
		posY += Parameters.defaultSensorRange * 2 + 1;
		position = new Int2D (position.x, posY);
	}
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
		  System.out.println("no road");
		  return null;
	  }
	}

public void InspectGoal() {
	Int2D position = new Int2D (Parameters.defaultSensorRange, Parameters.defaultSensorRange);
	int depth = Parameters.defaultSensorRange;
	
	while(depth <= Parameters.xDimension) {
		int posX = position.x;
		int posY = position.y;
		dtPlanner.getGoals().add(position);
		posX = Parameters.xDimension - Parameters.defaultSensorRange - 1;
		
		if (this.getEnvironment().isInBounds(posX, posY)) {
			position = new Int2D (posX, position.y);
			dtPlanner.getGoals().add(position);
		}
		posY += Parameters.defaultSensorRange * 2 + 1;
		depth = depth + Parameters.defaultSensorRange * 2 + 1;
		if (depth > Parameters.xDimension) {
			break;
		}
		position = new Int2D (position.x, posY);
		dtPlanner.getGoals().add(position);
		posX = Parameters.defaultSensorRange;;
		position = new Int2D (posX, position.y);
		dtPlanner.getGoals().add(position);
		depth = depth + (Parameters.defaultSensorRange * 2 + 1);
		posY += Parameters.defaultSensorRange * 2 + 1;
		position = new Int2D (position.x, posY);
	}
}



	private TWEntity Any_collision(PriorityQueue<TWEntity> neighborObjects, List<TWAgent> neighbouringAgents) {
		LinkedList<TWEntity> tiles = null;
		while(!neighborObjects.isEmpty()) {
			TWEntity tile = neighborObjects.poll();
			for (int i = 0; i < neighbouringAgents.size(); i++) {
				if (this.getDistanceTo(tile) >= neighbouringAgents.get(i).getDistanceTo(tile)) {
					continue;
				}
			}
			return tile;
		}
		return null;
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
				break;
			case PUTDOWN:
				TWHole hole = (TWHole) storage.getMemoryGrid().get(this.x, this.y);
				putTileInHole(hole);
				break;
			case REFUEL:
				refuel();
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
	   System.out.println("No where to go!");
	   return new TWThought(TWAction.MOVE, TWDirection.Z);
	  }
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
