package tileworld.agent;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;

import com.sun.org.apache.xalan.internal.xsltc.runtime.Parameter;

import sim.engine.Schedule;
import sim.field.grid.ObjectGrid2D;
import sim.util.Bag;
import sim.util.Int2D;
import sim.util.IntBag;
import tileworld.Parameters;
import tileworld.environment.NeighbourSpiral;
import tileworld.environment.TWEntity;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWObject;
import tileworld.environment.TWObstacle;
import tileworld.environment.TWTile;

public class MyStorage extends TWAgentWorkingMemory {
	protected Schedule schedule;
	protected TWAgent me;
	protected final static int MAX_TIME = Parameters.lifeTime;
	public ObjectGrid2D memoryGrid;
	public TWAgentPercept[][] objects;
	public int memorySize;
	protected HashMap<Class<?>, TWEntity> closestInSensorRange;
	static protected List<Int2D> spiral = new NeighbourSpiral(Parameters.defaultSensorRange * 4).spiral();
	protected List<TWAgent> neighbouringAgents = new ArrayList<TWAgent>();

	protected Int2D fuelStation;
	
	protected Int2D getFuelStation() {
		return fuelStation;
	}
	protected void setFuelStation(int x, int y) {
		this.fuelStation = new Int2D(x, y); 
	}

	public MyStorage(TWAgent moi, Schedule schedule, int x, int y) {
		super(moi, schedule, x, y);
		closestInSensorRange = new HashMap<Class<?>, TWEntity>(4);
		this.me = moi;
		this.objects = new TWAgentPercept[x][y];
		this.schedule = schedule;
		this.memoryGrid = new ObjectGrid2D(me.getEnvironment().getxDimension(), me.getEnvironment().getyDimension());
	}
	public void merStorage(Bag sensedObjects, Int2D posAgent) {
		
		int minX = Math.max(0, posAgent.getX() - Parameters.defaultSensorRange);
		int maxX = Math.min(Parameters.xDimension, posAgent.getX() + Parameters.defaultSensorRange);
		int minY = Math.max(0, posAgent.getY() - Parameters.defaultSensorRange);
		int maxY = Math.min(Parameters.yDimension, posAgent.getY() + Parameters.defaultSensorRange);

		for (int i = minX; i < maxX; i++) {
			for(int j = minY; j < maxY; j++) {
				if (objects[i][j] != null) {
					objects[i][j] = null;
					memoryGrid.set(i, j, null);
					memorySize--;
				}
			}
		}
		for (int i = 0; i < sensedObjects.size(); i++) {
			TWEntity mega = (TWEntity) sensedObjects.get(i);
			if (!(mega instanceof TWEntity)) {
				continue;
			} 
			if (this.fuelStation == null && mega instanceof TWFuelStation) {
				setFuelStation(mega.getX(), mega.getY());
			}				
			
			objects[mega.getX()][mega.getY()] = new TWAgentPercept(mega, this.getSimulationTime());
			memoryGrid.set(mega.getX(), mega.getY(), mega);
		}
	}
	@Override
	public void updateMemory(Bag sensedObjects, IntBag objectXCoords, IntBag objectYCoords, Bag sensedAgents, IntBag agentXCoords, IntBag agentYCoords) {
		
		closestInSensorRange = new HashMap<Class<?>, TWEntity>(4);

		assert (sensedObjects.size() == objectXCoords.size() && sensedObjects.size() == objectYCoords.size());

		this.decayMemory();
		int minX = Math.max(0, me.getX() - Parameters.defaultSensorRange);
		int maxX = Math.min(Parameters.xDimension, me.getX() + Parameters.defaultSensorRange);
		int minY = Math.max(0, me.getY() - Parameters.defaultSensorRange);
		int maxY = Math.min(Parameters.yDimension, me.getY() + Parameters.defaultSensorRange);

		for (int i = minX; i < maxX; i++) {
			for(int j = minY; j < maxY; j++) {
				if (objects[i][j] != null) {
					objects[i][j] = null;
					memoryGrid.set(i, j, null);
					memorySize--;
				}
			}
		}
		for (int i = 0; i < sensedObjects.size(); i++) {
			TWEntity mega = (TWEntity) sensedObjects.get(i);
			if (!(mega instanceof TWEntity)) {	
				continue;
			}
			if (this.fuelStation == null && mega instanceof TWFuelStation) {
				System.out.println(this.me.getName() + ": Found fuel" + mega.getX() + ", " + mega.getY());
				setFuelStation(mega.getX(), mega.getY());
			}				

			objects[mega.getX()][mega.getY()] = new TWAgentPercept(mega, this.getSimulationTime());
			memoryGrid.set(mega.getX(), mega.getY(), mega);
			updateClosest(mega);
		}
        neighbouringAgents.clear();
		for (int i = 0; i < sensedAgents.size(); i++) {
            if (!(sensedAgents.get(i) instanceof TWAgent)) {
                assert false;
            }
            TWAgent a = (TWAgent) sensedAgents.get(i);
            if(a == null || a.equals(me)){
                continue;
            }
            neighbouringAgents.add(a);
        }
	}
	
	public TWAgent getNeighbour(){
        if(neighbouringAgents.isEmpty()){
            return null;
        }else{
            return neighbouringAgents.get(0);
        }
    }
	
	@Override
	public void updateMemory(TWEntity[][] sensed, int xOffset, int yOffset) {
		for (int x = 0; x < sensed.length; x++) {
			for (int y = 0; y < sensed[x].length; y++) {
				objects[x + xOffset][y + yOffset] = new TWAgentPercept(sensed[x][y], this.getSimulationTime());
			}
		}
	}
	@Override
	public void decayMemory() {
	    for (int x = 0; x < this.objects.length; x++) {
	       for (int y = 0; y < this.objects[x].length; y++) {
	           TWAgentPercept currentMemory =  objects[x][y];
	           if(currentMemory!=null && !(currentMemory.getO() instanceof TWFuelStation) && currentMemory.getT() < schedule.getTime()-MAX_TIME){ 
	        	   objects[x][y] = null;
	        	   memoryGrid.set(x, y, null);
	               memorySize--;
	           }
	       }
	    }
	}
	
	@Override
	public ObjectGrid2D getMemoryGrid() {
		return this.memoryGrid;
	}
	@Override
	public boolean isCellBlocked(int tx, int ty) {
		if (objects[tx][ty] == null) {
			return false;
		}

		TWEntity e = objects[tx][ty].getO();
		return (e instanceof TWObstacle);
	}
	@Override
	public void removeAgentPercept(int x, int y){
		objects[x][y] = null;
		memoryGrid.set(x, y, null);
	}
	@Override
	public void removeObject(TWEntity mega){
		removeAgentPercept(mega.getX(), mega.getY());
	}
	protected double getSimulationTime() {
		return schedule.getTime();
	}
	protected void updateClosest(TWEntity mega) {
		assert (mega != null);
		if (closestInSensorRange.get(mega.getClass()) == null || me.closerTo(mega, closestInSensorRange.get(mega.getClass()))) {
			closestInSensorRange.put(mega.getClass(), mega);
			
		}
	}
	class ObjectComparator implements Comparator<TWEntity>{
        public int compare(TWEntity o1, TWEntity o2) {
            int dis1 = (int) ((MyProxy)me).getDistanceTo(o1.getX(),o1.getY());
            int dis2 = (int) ((MyProxy)me).getDistanceTo(o2.getX(),o2.getY());
            if (dis1 < dis2)
                return 1;
            else if (dis1 > dis2)
                return -1;
                            return 0;
            }
    }
	protected PriorityQueue<TWEntity> getNearbyAllSortedObjects(int sx, int sy, double threshold, Class<?> type) {
		double maxTimestamp = 0;
		TWEntity mega = null;
		double time = 0;
		PriorityQueue<TWEntity> ret = new PriorityQueue<TWEntity>(10, new ObjectComparator());
		int x, y;
		for (Int2D offset : spiral) {
			x = offset.x + sx;
			y = offset.y + sy;

			if (me.getEnvironment().isInBounds(x, y) && 
					objects[x][y] != null &&
					!(objects[x][y].getO() instanceof TWFuelStation)) {
				mega = (TWObject) objects[x][y].getO();
				if (type.isInstance(mega)) {
					time = objects[x][y].getT();
					if (this.getSimulationTime() - time <= threshold) {
						ret.add(mega);
					} else if (time > maxTimestamp) {
						ret.add(mega);
						maxTimestamp = time;
					}
				}
			}
		}
		return ret;
	}
	protected TWObject getNearbyObject(int sx, int sy, double threshold, Class<?> type) {
		double maxTimestamp = 0;
		TWObject mega = null;
		double time = 0;
		TWObject ret = null;
		int x, y;
		for (Int2D offset : spiral) {
			x = offset.x + sx;
			y = offset.y + sy;
			if (me.getEnvironment().isInBounds(x, y) 
					&& this.objects[x][y] != null
					&& !(objects[x][y].getO() instanceof TWFuelStation)) {
				mega = (TWObject) this.objects[x][y].getO();
				if (type.isInstance(mega))
				{
					time = this.objects[x][y].getT();
					if (this.getSimulationTime() - time <= threshold) {
						return mega;
					} else if (time > maxTimestamp) {
						ret = mega;
						maxTimestamp = time;
					}
				}
			}
		}
		return ret;
	}

	@Override
	public TWTile getNearbyTile(int x, int y, double threshold) {
		return (TWTile) this.getNearbyObject(x, y, threshold, TWTile.class);
	}
	@Override
	public TWHole getNearbyHole(int x, int y, double threshold) {
		return (TWHole) this.getNearbyObject(x, y, threshold, TWHole.class);
	}
	
	
	public static void main(String[] args) {

	}
	

}
