package tileworld.agent;

import java.util.*;

import sim.engine.Schedule;
import sim.field.grid.ObjectGrid2D;
import sim.util.Bag;
import sim.util.Int2D;
import sim.util.IntBag;
import tileworld.environment.*;
import tileworld.Parameters;

/**
 * 修改后的agent记忆模块
 * 包括衰减
 * 不一定全用得上
 */
public class TWAgentWorkingMemory {

	// ==============================================================================
	// 成员变量和其getter/setter方法
	// ==============================================================================

	// --------------
	/**
	 * TWEnvironment中定义的schedule，可以通过它获取当前时间戳/步数
	 */
	private Schedule schedule;

	/**
	 * 获取当前时间戳
	 * 
	 * @return 浮点数时间
	 */
	private double getSimulationTime() {
		return schedule.getTime();
	}

	// --------------
	/**
	 * agent自己，可以用它获取位置等信息
	 */
	private TWAgent mySelf;

	// --------------
	/**
	 * 由参数定义好的存活时间，适用于tile/hole/etc
	 */
	private final static int MAX_TIME = Parameters.lifeTime;

	// --------------
	/**
	 * 主记忆存储区，可以调用其set()方法，设定特定坐标处的对象（hole/tile/etc）
	 */
	private ObjectGrid2D memoryGrid;

	/**
	 * 获取主记忆存储网格
	 * 
	 * @return 网格对象
	 */
	public ObjectGrid2D getMemoryGrid() {
		return this.memoryGrid;
	}

	// --------------
	/**
	 * 副记忆存储区，存储‘观测’，它将普通的对象（hole/tile/etc）包装上了时间戳
	 * 原代码中两个都用了，感觉可以精简合并到主记忆区中？
	 */
	private TWAgentPercept[][] objects;

	/**
	 * 获取副观测列表
	 * 
	 * @return agent的观测，包装了时间戳的对象
	 */
	public TWAgentPercept[][] getAgentPercept() {
		return this.objects;
	}

	// --------------
	/**
	 * 加油站位置
	 */
	private Int2D fuelStation;

	/**
	 * 返回加油站坐标
	 * 
	 * @return 加油站坐标
	 */
	public Int2D getFuelStation() {
		return this.fuelStation;
	}

	/**
	 * 设置加油站坐标
	 * 
	 * @param x 加油站x
	 * @param y 加油站y
	 */
	public void setFuelStation(int x, int y) {
		this.fuelStation = new Int2D(x, y);
	}

	// --------------
	/**
	 * 存储感受野中最近的对象（hole/tile/etc）
	 * 感觉似乎大概没必要用hashmap
	 */
	private HashMap<Class<?>, TWEntity> closestInSensorRange;

	/**
	 * 根据对象类型返回最近对象（hole/tile/etc）
	 *
	 * @param type 对象类型（hole/tile/etc）
	 * @return 对象
	 */
	public TWEntity getClosestObjectInSensorRange(Class<?> type) {
		return closestInSensorRange.get(type);
	}

	/**
	 * 更新感受野中最近的物品
	 * 
	 * @param o 物体实例
	 */
	private void updateClosest(TWEntity o) {
		assert (o != null);
		if (closestInSensorRange.get(o.getClass()) == null
				|| mySelf.closerTo(o, closestInSensorRange.get(o.getClass()))) {
			closestInSensorRange.put(o.getClass(), o);
		}
	}

	// --------------
	/**
	 * 源代码有，在getNearbyObject使用，但getNearbyObject方法本身并未被调用
	 * 没啥用
	 */
	static private List<Int2D> spiral = new NeighbourSpiral(Parameters.defaultSensorRange * 4).spiral();

	// --------------
	/**
	 * 邻近的agents
	 */
	private List<TWAgent> neighbouringAgents = new ArrayList<TWAgent>();

	/**
	 * 返回最近的一个agent
	 * 
	 * @return 那个agent
	 */
	public TWAgent getNeighbour() {
		if (neighbouringAgents.isEmpty()) {
			return null;
		} else {
			return neighbouringAgents.get(0);
		}
	}

	// --------------
	/**
	 * 探索得分/优先级
	 * 越没探索越大
	 * 源代码：未探索 = 正无穷, 正在探索 = 0, 每次衰减 += 1
	 */
	private Double[][] explorationScore;

	// ==============================================================================
	// 构造函数
	// ==============================================================================

	/**
	 * @param me       本agent
	 * @param schedule TWEnvironment中的schedule
	 */
	public TWAgentWorkingMemory(TWAgent me, Schedule schedule) {
		int mapx = me.getEnvironment().getxDimension();
		int mapy = me.getEnvironment().getyDimension();

		// 探索得分初始化为正无穷
		this.explorationScore = new Double[mapx][mapy];
		for (int i = 0; i < mapx; i++) {
			for (int j = 0; j < mapy; j++) {
				explorationScore[i][j] = Double.POSITIVE_INFINITY;
			}

		}

		this.mySelf = me;
		this.objects = new TWAgentPercept[mapx][mapy];
		this.schedule = schedule;
		this.memoryGrid = new ObjectGrid2D(mapx, mapy);

		this.closestInSensorRange = new HashMap<>();
	}

	// ==============================================================================
	// 主要逻辑方法
	// ==============================================================================

	/**
	 * 每步都自动调用的记忆更新方法
	 * 提供一个大概的思路，实现同等功能即可
	 * 主要是将当前感受到的物品与记忆中的相比较并更新记忆中的物品
	 * 原本的英文注释
	 * Called at each time step, updates the memory map of the agent.
	 * Note that some objects may disappear or be moved, in which case part of
	 * sensed may contain null objects
	 *
	 * Also note that currently the agent has no sense of moving objects, so
	 * an agent may remember the same object at two locations simultaneously.
	 * 
	 * Other agents in the grid are sensed and passed to this function. But it
	 * is currently not used for anything. Do remember that an agent sense itself
	 * too.
	 *
	 * @param sensedObjects bag containing the sensed objects
	 * @param objectXCoords bag containing x coordinates of objects
	 * @param objectYCoords bag containing y coordinates of object
	 * @param sensedAgents  bag containing the sensed agents
	 * @param agentXCoords  bag containing x coordinates of agents
	 * @param agentYCoords  bag containing y coordinates of agents
	 */
	public void updateMemory(Bag sensedObjects, IntBag objectXCoords, IntBag objectYCoords, Bag sensedAgents,
			IntBag agentXCoords, IntBag agentYCoords) {
		/*
		 * 首先需要衰减记忆
		 */
		// ..........................
		decayMemory();

		/*
		 * 遍历记忆中此感受野中的物体，将其放入previousSensedObj中供之后比较用，同时清除记忆中的这些物品。
		 */
		int senseRadius = Parameters.defaultSensorRange;
		TWAgentPercept[][] previousSensedObj = new TWAgentPercept[senseRadius * 2 + 1][senseRadius * 2 + 1];
		int visibleX_min = mySelf.getX() - Parameters.defaultSensorRange;
		int visibleY_min = mySelf.getY() - Parameters.defaultSensorRange;
		for (int i = 0; i <= Parameters.defaultSensorRange * 2; i++) {
			for (int j = 0; j <= Parameters.defaultSensorRange * 2; j++) {
				// .................
				if (mySelf.getEnvironment().isInBounds(i + visibleX_min, j + visibleY_min)) {
					previousSensedObj[i][j] = objects[i + visibleX_min][j + visibleY_min];
					this.explorationScore[i + visibleX_min][j + visibleY_min] = 0.0;
					objects[i + visibleX_min][j + visibleY_min] = null;
					memoryGrid.set(i + visibleX_min, j + visibleY_min, null);
				}
			}
		}
//		System.out.printf("Cur Location: %d %d, Cur ExplorationScore: %f\n", mySelf.getX(), mySelf.getY(), explorationScore[mySelf.getX()][mySelf.getY()]);

		/*
		 * 遍历参数里提供的，此轮感受野中感受到的物品，并放入记忆中。
		 * 注意处理加油站
		 * 注意更新物品的时间（如果记忆previousSensedObj中物品与当前感受到的相同，保持不变；如果不同，以此此时间为准）
		 * 注意更新最近物品updateClosest
		 */
		for (int i = 0; i < sensedObjects.size(); i++) {
			// .....................
			Object o = sensedObjects.get(i);
			int x = objectXCoords.get(i);
			int y = objectYCoords.get(i);
			if (o instanceof TWEntity) {
				if (o instanceof TWFuelStation && fuelStation == null) {
					setFuelStation(x, y);
				}
				TWEntity obj = (TWEntity) o;
				if (objects[x][y] == null) {
					objects[x][y] = new TWAgentPercept(obj, schedule.getTime());
				} else if (objects[x][y].getO() == obj) {
					objects[x][y].setT(schedule.getTime());
				}
				memoryGrid.set(x, y, objects[x][y]);
				updateClosest(obj);
			}
		}

		/*
		 * 遍历参数中提供的agent，更新neighbouringAgents
		 */
		// ..........................
		neighbouringAgents.clear();
		for (int k = 0; k < sensedAgents.size(); k++) {
			Object o = sensedAgents.get(k);
			assert o instanceof TWAgent;
			TWAgent agent = (TWAgent) o;
			int x = agentXCoords.get(k);
			int y = agentYCoords.get(k);
			if (agent != mySelf) {
				neighbouringAgents.add(agent);
			}
			for (int i = x - Parameters.defaultSensorRange; i <= x + Parameters.defaultSensorRange; i++) {
				for (int j = y - Parameters.defaultSensorRange; j <= y + Parameters.defaultSensorRange; j++) {
					if (mySelf.getEnvironment().isInBounds(i, j)) {
						explorationScore[i][j] = 0.0;
					}
				}
			}
		}
	}

	/**
	 * 整合记忆。根据参数中提供的别的agent的记忆更新自己的记忆。
	 * 注意处理交叉冲突部分，比如优先自己的感知而非他人记忆
	 * 注意处理特殊点位比如加油站
	 * 
	 * @param objectsShared 别人的记忆，是全图的大小的数组
	 * @param agentPos      别人的位置
	 */
	public void mergeMemory(TWAgentPercept[][] objectsShared, Int2D agentPos) {
		// ...............
		for (int i = 0; i < memoryGrid.getWidth(); i++) {
			for (int j = 0; j < memoryGrid.getHeight(); j++) {
				if (objectsShared[i][j] != null) {
					if (objectsShared[i][j].getO() instanceof TWFuelStation && fuelStation == null) {
						setFuelStation(i, j);
					} else if (objects[i][j] == null) {
						objects[i][j] = objectsShared[i][j];
						memoryGrid.set(i, j, objects[i][j]);
					} else if (objects[i][j].newerFact(objectsShared[i][j])) {
						objects[i][j].setT(objectsShared[i][j].getT());
					}
				}
			}
		}
		int x = agentPos.x;
		int y = agentPos.y;
		for (int i = x - Parameters.defaultSensorRange; i <= x + Parameters.defaultSensorRange; i++) {
			for (int j = y - Parameters.defaultSensorRange; j <= y + Parameters.defaultSensorRange; j++) {
				if (mySelf.getEnvironment().isInBounds(i, j)) {
					explorationScore[i][j] = 0.0;
				}
			}
		}
	}

	/**
	 * 遍历所有记忆中的物品，并加上适当的衰减效果（增加explorationScore）
	 * 清除掉所有超时的物品
	 */
	public void decayMemory() {
		// .......................
		for (int i = 0; i < memoryGrid.getWidth(); i++) {
			for (int j = 0; j < memoryGrid.getHeight(); j++) {
				if (objects[i][j] != null) {
					if (getEstimatedRemainingLifetime(objects[i][j].getO(), 1.) > 0) {
						explorationScore[i][j] += 1;
					} else {
						objects[i][j] = null;
						memoryGrid.set(i, j, null);
						explorationScore[i][j] = Double.MAX_VALUE;
					}
				}
			}
		}
	}

	/**
	 * 获取自己负责区域内特定块的优先列表
	 * 比如一个包含了自己区域内所有tile的列表
	 * 列表为PriorityQueue，用于根据tile的属性（比如距离agent多远）来排序
	 * 源代码中使用下文提供的getTSPDistance()来判断比较
	 * 
	 * @param bounds 包含了区域四个角坐标的数组（左上-右上-右下-左下）
	 * @param type   类型，比如Tile类或hole类
	 * @return 排好序的优先队列（也可以考虑其他实现）
	 */
	protected PriorityQueue<TWEntity> getNearbyObjectsWithinBounds(Int2D[] bounds, Class<?> type) {
		// ...............
		PriorityQueue<TWEntity> entities = new PriorityQueue<>(Comparator.comparingDouble(o -> getTSPDistance(mySelf, o, true)));
		int x1 = bounds[0].x;
		int y1 = bounds[0].y;
		int x2 = bounds[2].x;
		int y2 = bounds[2].y;
		for (int i = x1; i <= x2; i++) {
			for (int j = y1; j <= y2; j++) {
				if (mySelf.getEnvironment().isInBounds(i,j) && objects[i][j] != null && !(objects[i][j].getO() instanceof TWFuelStation)) {
					TWEntity o = objects[i][j].getO();
					if (type.isInstance(o)) {
						entities.add(o);
					}
				}
			}
		}
		return entities;
	}

	/**
	 * 遍历一个锚点区域内所有块的探索的得分来得出锚点整体的探索得分
	 * 用于评估一个锚点区域的探索度，以便优先探索
	 * 
	 * @param anchor 锚点坐标
	 * @return 浮点数探索得分
	 */
	public Double getAnchorExplorationScore(Int2D anchor) {
		// ......................
		double score = 0;
		int exploreCnt = 0;
		for (int i = anchor.x - Parameters.defaultSensorRange; i <= anchor.x + Parameters.defaultSensorRange; i++) {
			for (int j = anchor.y - Parameters.defaultSensorRange; j <= anchor.y + Parameters.defaultSensorRange; j++) {
				if (i >= 0 && i < memoryGrid.getWidth() && j >= 0 && j < memoryGrid.getHeight()) {
					exploreCnt++;
//					if (anchor.x == mySelf.getX() && anchor.y == mySelf.getY()) {
//						System.out.printf("Position: %d, %d ExplorationScore: %f\n", i, j, explorationScore[i][j]);
//					}
					score = score * (1. - 1. / exploreCnt) + explorationScore[i][j] * (1. / exploreCnt);
				}
			}
		}
//		if (anchor.x == mySelf.getX() && anchor.y == mySelf.getY()) {
//			System.out.printf("Cur Location: %d %d, Cur ExplorationScore: %f\n", mySelf.getX(), mySelf.getY(), explorationScore[mySelf.getX()][mySelf.getY()]);
//			System.out.printf("Anchors: %d, %d CurExplorationScore: %f\n", anchor.x, anchor.y, score);
//		}
		return score;
	}

	/**
	 * 返回预估剩余时间。（原代码，可根据需求更改）
	 *
	 * @param o         物体实例
	 * @param threshold 超参数
	 * @return 浮点数时间
	 */
	public double getEstimatedRemainingLifetime(TWEntity o, double threshold) {
		if (objects[o.getX()][o.getY()] == null)
			return 0;
		else
			return (Parameters.lifeTime * threshold) - (this.getSimulationTime() - objects[o.getX()][o.getY()].getT());
	}

	// ==============================================================================
	// 工具函数，按需使用
	// ==============================================================================

	/**
	 * 用预估时间来调整距离因子，比如让快消失对象的距离因子变短以便提前探索
	 *
	 * @param a            agent
	 * @param b            对象
	 * @param TSPHeuristic 是否加入因子修正
	 * @return （修正后的）距离因子
	 */
	public double getTSPDistance(TWEntity a, TWEntity b, boolean TSPHeuristic) {
		double oDist = a.getDistanceTo(b);
		// Modifies Manhattan distance by lifetime remaining, so between two equidistant
		// objects, the one with a shorter lifetime is closer
		if (TSPHeuristic) {
			oDist *= getEstimatedRemainingLifetime(b, 1.0) / MAX_TIME;
		}
		return oDist;
	}

	/**
	 * 没用到
	 * updates memory using 2d array of sensor range - currently not used
	 *
	 * @see TWAgentWorkingMemory updateMemory(sim.util.Bag, sim.util.IntBag,
	 *      sim.util.IntBag)
	 */
	public void updateMemory(TWEntity[][] sensed, int xOffset, int yOffset) {
		for (int x = 0; x < sensed.length; x++) {
			for (int y = 0; y < sensed[x].length; y++) {
				objects[x + xOffset][y + yOffset] = new TWAgentPercept(sensed[x][y], this.getSimulationTime());
			}
		}
	}

	/**
	 * 清除‘观测’记忆中特定位置的东西
	 *
	 * @param x x坐标
	 * @param y y坐标
	 */
	public void removeAgentPercept(int x, int y) {
		objects[x][y] = null;
	}

	/**
	 * 清除清除‘观测’记忆中一个特定的物品实例。
	 *
	 * @param o 物品实例
	 */
	public void removeObject(TWEntity o) {
		removeAgentPercept(o.getX(), o.getY());
	}

	/**
	 * 他们没用到
	 * Finds a nearby tile we have seen less than threshold timesteps ago
	 *
	 * @see TWAgentWorkingMemory#getNearbyObject(int, int, double, java.lang.Class)
	 */
	public TWTile getNearbyTile(int x, int y, double threshold) {
		return (TWTile) this.getNearbyObject(x, y, threshold, TWTile.class);
	}

	/**
	 * 他们没用到
	 * Finds a nearby hole we have seen less than threshold timesteps ago
	 *
	 * @see TWAgentWorkingMemory#getNearbyObject(int, int, double, java.lang.Class)
	 */
	public TWHole getNearbyHole(int x, int y, double threshold) {
		return (TWHole) this.getNearbyObject(x, y, threshold, TWHole.class);
	}

	/**
	 * 他们没用到
	 * Returns the nearest object that has been remembered recently where recently
	 * is defined by a number of timesteps (threshold)
	 *
	 * If no Object is in memory which has been observed in the last threshold
	 * timesteps it returns the most recently observed object. If there are no
	 * objects in
	 * memory the method returns null. Note that specifying a threshold of one
	 * will always return the most recently observed object. Specifying a threshold
	 * of MAX_VALUE will always return the nearest remembered object.
	 *
	 * Also note that it is likely that nearby objects are also the most recently
	 * observed
	 *
	 *
	 * @param sx        coordinate from which to check for objects
	 * @param sy        coordinate from which to check for objects
	 * @param threshold how recently we want to have seen the object
	 * @param type      the class of object we're looking for (Must inherit from
	 *                  TWObject, specifically tile or hole)
	 * @return
	 */
	private TWObject getNearbyObject(int sx, int sy, double threshold, Class<?> type) {

		// If we cannot find an object which we have seen recently, then we want
		// the one with maxTimestamp
		double maxTimestamp = 0;
		TWObject o = null;
		double time = 0;
		TWObject ret = null;
		int x, y;
		for (Int2D offset : spiral) {
			x = offset.x + sx;
			y = offset.y + sy;

			if (mySelf.getEnvironment().isInBounds(x, y) && objects[x][y] != null) {
				o = (TWObject) objects[x][y].getO();// get mem object
				if (type.isInstance(o)) {// if it's not the type we're looking for do nothing

					time = objects[x][y].getT();// get time of memory

					if (this.getSimulationTime() - time <= threshold) {
						// if we found one satisfying time, then return
						return o;
					} else if (time > maxTimestamp) {
						// otherwise record the timestamp and the item in case
						// it's the most recent one we see
						ret = o;
						maxTimestamp = time;
					}
				}
			}
		}

		// this will either be null or the object of Class type which we have
		// seen most recently but longer ago than now-threshold.
		return ret;
	}

	/**
	 * Is the cell blocked according to our memory?
	 *
	 * @param tx x position of cell
	 * @param ty y position of cell
	 * @return true if the cell is blocked in our memory
	 */
	public boolean isCellBlocked(int tx, int ty) {

		// no memory at all, so assume not blocked
		if (objects[tx][ty] == null) {
			return false;
		}

		TWEntity e = (TWEntity) objects[tx][ty].getO();
		// is it an obstacle?
		return (e instanceof TWObstacle);
	}

	public TWAgentPercept[][] getObjects() {
		return objects;
	}
}
