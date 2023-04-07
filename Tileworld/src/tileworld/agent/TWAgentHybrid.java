package tileworld.agent;

import java.util.*;

import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWDirection;
import tileworld.planners.DefaultTWPlanner;
import tileworld.exceptions.CellBlockedException;

/**
 * agent实现
 */
public class TWAgentHybrid extends TWAgent {
    /*
     * 原代码中的可调超参，供参考，不一定按这些来
     * 保留了原本的英文注释
     */
    /**
     * When comparing remaining fuel to distance to fuel station, how much buffer to
     * account for
     * Higher tolerance means agent leaves little buffer fuel to account for
     * obstacles appearing
     * Lower tolerance means agent leaves plenty of buffer fuel and may tend to
     * refuel more
     */
    private double fuelTolerance;

    /**
     * Hard fuel limit before needing to refuel
     */
    private double hardFuelLimit;

    /**
     * Modifies the heuristic used for prioritizing the list of possible goals.
     */
    private boolean TSPHeuristic;

    /**
     * Lifetime threshold used for determining whether a object at risk of decay
     * should be pursued,
     * taking into account both its estimated remaining lifetime and its distance
     * from agent.
     * Estimated remaining time left for a memorized object is based on its memory
     * time stamp.
     */
    private double objectLifetimeThreshold;

    /**
     * Maximum number of goals in queue to announce.
     * Announcing goals prevent goal collisions.
     * However, reserving too many goals can lead to sub-optimal division of goals
     * between agents.
     * Reserving too little can result in collision between an assisting agent's
     * goal and this agent's next immediate goal,
     * thus wasting the assisting agent's resources.
     */
    private int goalAnnounceCount;

    private boolean allowAssistance;

    /**
     * Furthest zone agent can move to assist, specified in terms of number of zones
     */
    private int maxAssistZoneDistance;

    // ==============================================================================
    // 成员变量
    // ==============================================================================
    /**
     * 行动模式
     */
    enum Mode {
        EXPLORE, COLLECT, FILL, REFUEL, ASSIST_COLLECT, ASSIST_FILL, REACT_COLLECT, REACT_FILL, WAIT
    }

    /**
     * agent大名
     * 没啥用，建议用id识别agent
     */
    private String name;

    @Override
    public String getName() {
        return name;
    }

    /**
     * agent识别符
     * 应该独一无二
     */
    private int agentID;
    /**
     * 计划对象
     * 计划对象中包含了最终行动所需的目标的位置
     * 并且会根据目标位置和当前位置生成路径path
     */
    private DefaultTWPlanner planner;
    /**
     * 行动模式
     */
    private Mode mode;
    /**
     * 区域编号列表
     * key代表agentID，value代表区域编号
     * 源代码中用数组索引易出错，改为用hashmap更好
     */
    private HashMap<Integer, Integer> myZoneID;
    /**
     * 包含了agent所属区域四个角坐标的数组
     * （左上-右上-右下-左下)
     */
    private Int2D[] bounds;
    /**
     * 锚点的坐标列表
     * 每个锚点都是按agent感受野划分的不交叉的小区域
     * 允许边界上的有交叉，详情看原ppt有讲
     */
    private Int2D[] anchors;
    /**
     * agent的记忆
     */
    private TWAgentWorkingMemory workingMemory;
    /**
     * agent记忆中获取的
     * 在自己负责范围内的tile
     * 按特定方式排序的优先队列
     * 越优先越紧急（大概）
     */
    PriorityQueue<TWEntity> tilesInZone;
    /**
     * 同上
     * 是hole的列表
     */
    PriorityQueue<TWEntity> holesInZone;
    /**
     * 可能的tile链表
     * 初筛后的结果，可以根据这个进一步确定最终加入planner的对象
     * 具体用啥数据类型也可以改
     */
    LinkedList<TWEntity> possibleTileGoals;
    /**
     * 同上
     * 可能的hole链表
     */
    LinkedList<TWEntity> possibleHoleGoals;
    /**
     * 最近的tile
     * 这个在记忆TWAgentWorkingMemory里也可以实现
     * 不过他们放在这里实现
     */
    TWEntity[] closestTile;
    /**
     * 同上
     * 最近的hole
     */
    TWEntity[] closestHole;

    // ==============================================================================
    // 构造函数
    // ==============================================================================
    /**
     * 创建agent实例
     * 
     * @param name      大名
     * @param xpos      初始x坐标
     * @param ypos      初始y坐标
     * @param env       所属环境
     * @param fuelLevel 初始油量
     */
    public TWAgentHybrid(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        // 大名
        this.name = name;
        // 标识符
        this.agentID = Character.getNumericValue(name.charAt(name.length() - 1)) - 1;
        // agent的计划
        this.planner = new DefaultTWPlanner(this);
        // agent的记忆
        this.memory = new TWAgentWorkingMemory(this, env.schedule);
        // agent所属区域的四角
        this.bounds = new Int2D[4];
        // 初始化其他参数
        this.tilesInZone = new PriorityQueue<TWEntity>();
        this.holesInZone = new PriorityQueue<TWEntity>();
        this.possibleTileGoals = new LinkedList<TWEntity>();
        this.possibleHoleGoals = new LinkedList<TWEntity>();
    }

    // ==============================================================================
    // 主要逻辑函数
    // ==============================================================================
    /**
     * 交流函数
     * 包含简单的记忆分析（判断可达性）
     * 并据此添加对象到可能列表和招标列表中
     * 发送信息
     * （也可以再拆成几个函数，比如判断部分移出去，让此函数只负责发信息）
     */
    @Override
    public void communicate() {
        // 获取记忆
        // 发送自己的记忆
        // 暂且规定 receiver 为 0 时向全体广播
        Message message = new Message(agentID, 0, MsgType.agentInfo,
                new Object[] { workingMemory.getAgentPercept(), new Int2D(x, y) });
        this.getEnvironment().receiveMessage(message);

        // 判断是否初始化完成（划分好区域了）
        if (bounds[0] == null) return;

        // 是的话就
        // 判断可达性--添加可能列表--添加招标列表--发送自己的goal和招标信息
        // 注意判断各种限制，比如最大可携带的tile数量
        tilesInZone = workingMemory.getNearbyObjectsWithinBounds(bounds, TWTile.class);
        holesInZone = workingMemory.getNearbyObjectsWithinBounds(bounds, TWHole.class);

        possibleTileGoals.clear();
        possibleHoleGoals.clear();

        LinkedList<TWEntity> goals = new LinkedList<TWEntity>();
        LinkedList<TWEntity> auctionTiles = new LinkedList<TWEntity>();
        LinkedList<TWEntity> auctionHoles = new LinkedList<TWEntity>();

        // 判断可达性--添加可能列表
        while (!tilesInZone.isEmpty()) {
            TWEntity tile = tilesInZone.poll();
            double distToTile = this.getDistanceTo(tile);
            if (workingMemory.getEstimatedRemainingLifetime(tile, this.objectLifetimeThreshold) <= distToTile) auctionTiles.add(tile);
            else possibleTileGoals.add(tile);
        }

        while (!holesInZone.isEmpty()) {
            TWEntity hole = holesInZone.poll();
            double distToHole = this.getDistanceTo(hole);
            if (workingMemory.getEstimatedRemainingLifetime(hole, this.objectLifetimeThreshold) <= distToHole) auctionHoles.add(hole);
            else possibleHoleGoals.add(hole);
        }

        closestTile = possibleTileGoals.toArray(new TWEntity[0]);
        closestHole = possibleHoleGoals.toArray(new TWEntity[0]);

        // 筛选目标以及招标
        int tileCount = this.carriedTiles.size();
        for (int i = 0; i < closestTile.length; i++) {
            if (tileCount >= 3 || i > goalAnnounceCount) auctionTiles.add(closestTile[i]);
            else {
                tileCount ++;
                goals.add(closestTile[i]);
            }
        }

        for (int i = 0; i < closestHole.length; i++) {
            if (tileCount <= 0 || i > goalAnnounceCount) auctionHoles.add(closestHole[i]);
            else {
                tileCount --;
                goals.add(closestHole[i]);
            }
        }

        // 发送自己的目标和招标信息
        if (goals.size() > 0) {
            TWEntity[] goalsArr = goals.toArray(new TWEntity[0]);
            Message goalMessage = new Message(agentID, 0, MsgType.goalInfo, goalsArr);
            this.getEnvironment().receiveMessage(goalMessage);
        }

        if (auctionTiles.size() > 0) {
            TWEntity[] auctionArr = auctionTiles.toArray(new TWEntity[0]);
            Message auctionTileMessage = new Message(agentID, 0, MsgType.contractInfo_tile, new Object[] { auctionArr, myZoneID.get(agentID) });
            this.getEnvironment().receiveMessage(auctionTileMessage);
        }

        if (auctionHoles.size() > 0) {
            TWEntity[] auctionArr = auctionHoles.toArray(new TWEntity[0]);
            Message auctionHoleMessage = new Message(agentID, 0, MsgType.contractInfo_hole, new Object[] { auctionArr, myZoneID.get(agentID) });
            this.getEnvironment().receiveMessage(auctionHoleMessage);
        }
    }

    /**
     * 区域划分函数
     * 负责程序开始时按agent数量将地图区域均匀划分，
     * 并将区域分给对应agent（按距离远近）
     * 
     * @param width  地图宽（x轴）
     * @param height 地图高（y轴）
     */
    public void assignZone(int width, int height) {
        // 具体逻辑可以参考文档里说的
        int agentCount = 0;
        ArrayList<Message> messages = this.getEnvironment().getMessages();
        Map<Integer, Int2D> agentPos = new HashMap<>();

        // 确认agent数量存储agent当前位置
        for (Message msg : messages) {
            if (msg.getReceiver() == 0 && msg.getMessageType() == MsgType.agentInfo) {
                agentPos.put(msg.getSender(), (Int2D) msg.getMessageContent()[1]);
                agentCount ++;
            }
        }

        // 对agent和zone按距离进行匹配，之后确定各区域边界
        Int2D zoneDim;
        int myZoneIdx;
        boolean[] agentAssigned = new boolean[agentCount];
        if (width <= height) {
            zoneDim = new Int2D(width, height / agentCount);

            for (Map.Entry<Integer, Int2D> e : agentPos.entrySet()) { // 对每个agent
                int minDist = Parameters.xDimension + Parameters.yDimension;
                int zoneIdx = 0;
                for (int j = 0; j < agentCount; j ++) { // 对每个区域
                    if (agentAssigned[j]) continue; // 已被分配
                    Int2D agtPos = e.getValue();
                    int distToZone = agtPos.x + Math.abs(agtPos.y - zoneDim.y * j);
                    if (distToZone < minDist) zoneIdx = j;
                }
                myZoneID.put(e.getKey(), zoneIdx);
                agentAssigned[zoneIdx] = true;
            }

            // 计算边界
            myZoneIdx = myZoneID.get(agentID);

            bounds[0] = new Int2D(0, zoneDim.y * myZoneIdx);
            bounds[1] = new Int2D(width, zoneDim.y * myZoneIdx);

            // 如果是最后一块则覆盖剩余所有区域
            if (myZoneIdx == agentCount - 1) {
                bounds[2] = new Int2D(width, height);
                bounds[3] = new Int2D(0, height);
            }
            else {
                bounds[2] = new Int2D(width, zoneDim.y * (myZoneIdx + 1));
                bounds[3] = new Int2D(0, zoneDim.y * (myZoneIdx + 1));
            }
        }
        else {
            zoneDim = new Int2D(width / agentCount, height);

            for (Map.Entry<Integer, Int2D> e : agentPos.entrySet()) { // 对每个agent
                int minDist = Parameters.xDimension + Parameters.yDimension;
                int zoneIdx = 0;
                for (int j = 0; j < agentCount; j ++) { // 对每个区域
                    if (agentAssigned[j]) continue; // 已被分配
                    Int2D agtPos = e.getValue();
                    int distToZone = agtPos.y + Math.abs(agtPos.x - zoneDim.x * j);
                    if (distToZone < minDist) zoneIdx = j;
                }
                myZoneID.put(e.getKey(), zoneIdx);
                agentAssigned[zoneIdx] = true;
            }

            myZoneIdx = myZoneID.get(agentID);
            bounds[0] = new Int2D(zoneDim.x * myZoneIdx, 0);

            if (myZoneIdx == agentCount - 1) {
                bounds[1] = new Int2D(width, 0);
                bounds[2] = new Int2D(width, height);
            }
            else {
                bounds[1] = new Int2D(zoneDim.x * (myZoneIdx + 1), 0);
                bounds[2] = new Int2D(zoneDim.x * (myZoneIdx + 1), height);
            }

            bounds[3] = new Int2D(zoneDim.x * myZoneIdx, height);
        }

        // 计算锚点
        zoneDim = new Int2D(bounds[1].x - bounds[0].x, bounds[3].y - bounds[0].y);
        int horizontalAnchors = (int) Math.ceil(zoneDim.x / (Parameters.defaultSensorRange * 2.0 + 1));
        int verticalAnchors = (int) Math.ceil(zoneDim.y / (Parameters.defaultSensorRange * 2.0 + 1));
        anchors = new Int2D[horizontalAnchors * verticalAnchors];

        for (int i = 0, j = 0; i < verticalAnchors; i ++) {
            Int2D[] tmpAnchors = new Int2D[horizontalAnchors];

            for (int k = 0; k < horizontalAnchors; k ++) {
                int anchorX, anchorY;

                if (i == verticalAnchors - 1) anchorY = bounds[2].y - Parameters.defaultSensorRange;
                else anchorY = bounds[0].y + Parameters.defaultSensorRange + (Parameters.defaultSensorRange * 2 + 1) * i;

                if (k == horizontalAnchors - 1) anchorX = bounds[2].x - Parameters.defaultSensorRange;
                else anchorX = bounds[0].x + Parameters.defaultSensorRange + (Parameters.defaultSensorRange * 2 + 1) * k;

                tmpAnchors[k] = new Int2D(anchorX, anchorY);
            }

            // 按照行进顺序把锚点依次放入
            if (i % 2 != 0) {
                for (int k = horizontalAnchors - 1; k >= 0; k --) {
                    anchors[j] = tmpAnchors[k];
                    j ++;
                }
            }
            else {
                for (int k = 0; k < horizontalAnchors; k ++) {
                    anchors[j] = tmpAnchors[k];
                    j ++;
                }
            }
        }
    }

    /**
     * 协作整合函数
     * 用于把他人共享的对象评估后整合进自己的队列中去
     * 
     * @param queue        自己的队列
     * @param contractObj  他人的可协助对象列表
     * @param contractZone 他人所在区域id
     */
    public void mergeContracts(PriorityQueue<TWEntity> queue, TWEntity[] contractObj, int contractZone) {
        // 比如通过判断可达性将物品放入自己列表中
        int myZone = myZoneID.get(agentID);
        if (myZone != contractZone && Math.abs(myZone - contractZone) <= maxAssistZoneDistance) {
            for (TWEntity twEntity : contractObj) {
                double distToObj = this.getDistanceTo(twEntity);
                if (!(workingMemory.getEstimatedRemainingLifetime(twEntity, this.objectLifetimeThreshold) <= distToObj)) queue.add(twEntity);
            }
        }
    }

    public double getTSPDistance(TWEntity o) {
        double oDist = getDistanceTo(o);
        // Modifies Manhattan distance by lifetime remaining, so between two equidistant objects, the one with a shorter lifetime is closer
        if (this.TSPHeuristic) {
            oDist *= workingMemory.getEstimatedRemainingLifetime(o, 1.0)/Parameters.lifeTime;
        }
        return oDist;
    }
    /**
     * 主要的思考函数
     */
    @Override
    protected TWThought think() {
        // 然后进行知识共享，发送message进行记忆合并
        // 分析招标协助，把可能的标加入可协助表，并广播
        // 接收广播，解决可能的冲突
        // 思考行动模式（可以按他们ppt里的大逻辑图来）
        // 重新思考行动计划（包括清除现有计划、检查当前位置、按照当前位置信息和行动模式来确定最终的goal，并使用planner生成路径，得到行动方向）
        // 
        if (bounds[0] == null) {
			assignZone(this.getEnvironment().getxDimension(), this.getEnvironment().getyDimension());
			tilesInZone = workingMemory.getNearbyObjectsWithinBounds(bounds, new TWTile().getClass());
			holesInZone = workingMemory.getNearbyObjectsWithinBounds(bounds, new TWHole().getClass());
			while (!tilesInZone.isEmpty()) {
				TWEntity tile = tilesInZone.poll();
				double distToTile = this.getDistanceTo(tile);
				if (!(workingMemory.getEstimatedRemainingLifetime(tile, this.objectLifetimeThreshold) <= distToTile)) {
					possibleTileGoals.add(tile);
				}
			}
			while (!holesInZone.isEmpty()) {
				TWEntity hole = holesInZone.poll();
				double distToHole = this.getDistanceTo(hole);
				if (!(workingMemory.getEstimatedRemainingLifetime(hole, this.objectLifetimeThreshold) <= distToHole)) {
					possibleHoleGoals.add(hole);
				}
			}
			closestTile = new TWEntity[possibleTileGoals.size()];
			closestHole = new TWEntity[possibleHoleGoals.size()];
			closestTile = possibleTileGoals.toArray(closestTile);
			closestHole = possibleHoleGoals.toArray(closestHole);
		}

		// Merge all shared maps before any further deliberation
		ArrayList<Message> messages = this.getEnvironment().getMessages();
		for (int i = 0; i < messages.size(); i++) {
			Message message = messages.get(i);
			if (message.getSender() != agentID &&
				message.getReceiver() == 0 &&
				message.getMessageType() == MsgType.agentInfo) {
				workingMemory.mergeMemory((TWAgentPercept[][]) message.getMessageContent()[0], (Int2D) message.getMessageContent()[1]);
			}
		}

		// Check environment for available contracts
		Comparator<TWEntity> distHeur = new Comparator<TWEntity>() {
			public int compare(TWEntity o1, TWEntity o2) {
				   return (int) (getTSPDistance(o1) - getTSPDistance(o2));
			}
		};
		PriorityQueue<TWEntity> assistableTiles = new PriorityQueue<TWEntity>(10, distHeur);
		PriorityQueue<TWEntity> assistableHoles = new PriorityQueue<TWEntity>(10, distHeur);
		for (int i = 0; i < messages.size(); i++) {
			Message message = (Message) messages.get(i);
			if (message.getSender() != agentID &&
				message.getReceiver() == 0) {
				if (message.getMessageType() == MsgType.contractInfo_tile) {
					mergeContracts(assistableTiles, (TWEntity[]) message.getMessageContent()[0], (int) message.getMessageContent()[1]);
				}
				else if (message.getMessageType() == MsgType.contractInfo_hole) {
					mergeContracts(assistableHoles, (TWEntity[]) message.getMessageContent()[0], (int) message.getMessageContent()[1]);
				}
			}
		}

		// Remove announced goals from goal and contract lists to prevent goal collisions
		for (int i = 0; i < messages.size(); i++) {
			Message message = (Message) messages.get(i);
			if (message.getSender() != agentID &&
				message.getReceiver() == 0 &&
				message.getMessageType() == MsgType.goalInfo) {
				TWEntity[] announcedGoals = (TWEntity[]) message.getMessageContent();
				for (int j = 0; j < announcedGoals.length; j++) {
					if (announcedGoals[j] instanceof TWTile) {
						assistableTiles.remove(announcedGoals[j]);
						possibleTileGoals.remove(announcedGoals[j]);
					}
					else if (announcedGoals[j] instanceof TWHole) {
						assistableHoles.remove(announcedGoals[j]);
						possibleHoleGoals.remove(announcedGoals[j]);
					}
				}
			}
		}

		// Default Mode
		mode = Mode.EXPLORE;

		// Refueling takes utmost priority if fuel station already found and low on fuel
		if (workingMemory.getFuelStation() != null && this.getDistanceTo(workingMemory.getFuelStation().x, workingMemory.getFuelStation().y) >= this.fuelLevel * fuelTolerance) {
			mode = Mode.REFUEL;
		}
		// If fuel station not yet found, exploration takes highest priority
		else if (workingMemory.getFuelStation() == null) {
			mode = Mode.EXPLORE;
		}
		else if (this.fuelLevel <= this.hardFuelLimit) {
			// Extremely rare scenario where fuel station is not found during first exploration phase
			// Agent waits until other agents finishes exploring their zones and hopefully finds the fuel station
			// In the event fuel station is in this agent's zone, other agents have to assist exploring
			// the remaining parts of this zone, hopefully with enough fuel to spare
			// (ASSIST_EXPLORE not programmed in yet, requires broadcasting remaining exploration map to environment)
			if (workingMemory.getFuelStation() == null) {
				mode = Mode.WAIT;
			}
			else {
				mode = Mode.REFUEL;
			}
		}
		// If no tile and tile nearby, collect tile else explore
		else if (!this.hasTile()) {
			if (closestTile.length > 0) {
				mode = Mode.COLLECT;
			}
			else if (allowAssistance && !assistableTiles.isEmpty()) {
				mode = Mode.ASSIST_COLLECT;
			}
		}
		// If agent has tile and there is a hole nearby, prioritize filling hole
		else if (closestHole.length > 0) {
			if (closestTile.length == 0 ||
				(getTSPDistance(closestHole[0]) <= getTSPDistance(closestTile[0])) ||
				this.carriedTiles.size() >= 3) {
				mode = Mode.FILL;
			}
			else {
				mode = Mode.COLLECT;
			}
		}
		// If not at maximum number of tiles and there is only tile(s) nearby, collect tile(s)
		else if (closestTile.length > 0 && this.carriedTiles.size() < 3) {
			mode = Mode.COLLECT;
		}
		// If no tiles and holes in own zone, but there are assistable holes in a neighboring zone,
		// assist agent in neighboring zone to fill holes
		else if (allowAssistance && !assistableHoles.isEmpty()) {
			mode = Mode.ASSIST_FILL;
		}

		// Deletes plan and reevaluate goals every step rather than checking for new/decayed tiles/holes/obstacles
		// and changing existing plan
		planner.getGoals().clear();
		planner.voidPlan();

		// Always checks if possible to pickup/fill/refuel even when prioritizing
		// exploration
		Object curLocObj = this.memory.getMemoryGrid().get(x, y);
		if (curLocObj instanceof TWHole &&
			this.getEnvironment().canPutdownTile((TWHole) curLocObj, this)) {
			mode = Mode.REACT_FILL;

			// Announce goal as there is possibility target is not in purview of own's zone and is encountered enroute to or from refueling
			// If so, there may be a possibility of goal collision
			TWEntity[] goalsArr = new TWEntity[] {this.workingMemory.getObjects()[x][y].getO()};
			Message goalMessage = new Message(agentID, 0, MsgType.goalInfo, goalsArr);
			this.getEnvironment().receiveMessage(goalMessage);

			planner.getGoals().add(new Int2D(this.x, this.y));
			return new TWThought(TWAction.PUTDOWN, null);
		}
		else if (curLocObj instanceof TWTile &&
				 this.carriedTiles.size() < 3 &&
				 this.getEnvironment().canPickupTile((TWTile) curLocObj, this))
		{
			mode = Mode.REACT_COLLECT;

			TWEntity[] goalsArr = new TWEntity[] {this.workingMemory.getObjects()[x][y].getO()};
			Message goalMessage = new Message(agentID, 0, MsgType.goalInfo, goalsArr);
			this.getEnvironment().receiveMessage(goalMessage);

			planner.getGoals().add(new Int2D(this.x, this.y));
			return new TWThought(TWAction.PICKUP, null);
		}
		// If stumble upon fuel station, refuel if below 75% fuel.
		// This is different from the fuel management mechanism using the fuelTolerance threshold.
		else if (curLocObj instanceof TWFuelStation &&
				 this.fuelLevel < (0.75 * Parameters.defaultFuelLevel))
		{
			planner.getGoals().add(new Int2D(this.x, this.y));
			return new TWThought(TWAction.REFUEL, null);
		}
		// Modes which require a TWDirection and plan to be generated
		else {
			// getMemory().getClosestObjectInSensorRange(Tile.class);
			if (mode == Mode.EXPLORE) {
				// Collect exploration scores for all anchors
				Int2D anchorGoal = anchors[0];
				Double max_score = Double.NEGATIVE_INFINITY;

				for (int i = 0; i < anchors.length; i++) {
					Double curExplorationScore = workingMemory.getAnchorExplorationScore(anchors[i]);
					Double distToAnchor = this.getDistanceTo(anchors[i].x, anchors[i].y);

					if (curExplorationScore > max_score ||
					   ((curExplorationScore.equals(max_score)) &&
						(distToAnchor < this.getDistanceTo(anchorGoal.x, anchorGoal.y)))
					   ) {
						max_score = curExplorationScore;

						// If blocked, source for alternative positions with highest exploration score
						if (workingMemory.isCellBlocked(anchors[i].x, anchors[i].y)) {
							ArrayList<Int2D> alternativeAnchors = new ArrayList<Int2D>();
							ArrayList<Double> alternativeScores = new ArrayList<Double>();
							for (int j = -1; j <= 1; j++) {
								for (int k = -1; k <= 1; k++) {
									if (anchors[i].x + j < this.getEnvironment().getxDimension() &&
										anchors[i].y + k < this.getEnvironment().getyDimension() &&
										anchors[i].x - j >= 0 &&
										anchors[i].y + k >= 0 &&
										!workingMemory.isCellBlocked(anchors[i].x + j, anchors[i].y + k)) {
										alternativeAnchors.add(new Int2D(anchors[i].x + j, anchors[i].y + k));
										alternativeScores.add(workingMemory.getAnchorExplorationScore(alternativeAnchors.get(alternativeAnchors.size() - 1)));
									}
								}
							}
							anchorGoal = alternativeAnchors.get(0);
							int max_alt = 0;
							for (int j = 0; j < alternativeAnchors.size(); j++) {
								if (alternativeScores.get(j) > alternativeScores.get(max_alt)) {
									anchorGoal = alternativeAnchors.get(j);
								}
							}
						}
						else {
							anchorGoal = anchors[i];
						}
					}
				}

				planner.getGoals().add(anchorGoal);
			}
			else if (mode == Mode.REFUEL) {
				planner.getGoals().add(workingMemory.getFuelStation());
			}
			else if (mode == Mode.COLLECT) {
				planner.getGoals().add(new Int2D(closestTile[0].getX(), closestTile[0].getY()));
			}
			else if (mode == Mode.FILL) {
				planner.getGoals().add(new Int2D(closestHole[0].getX(), closestHole[0].getY()));
			}
			else if (mode == Mode.ASSIST_COLLECT) {
				planner.getGoals().add(new Int2D(assistableTiles.peek().getX(), assistableTiles.peek().getY()));

				// Broadcast goal being assisted to indicate contract is no longer available
				TWEntity[] goalsArr = new TWEntity[] {assistableTiles.peek()};
				Message goalMessage = new Message(agentID, 0, MsgType.goalInfo, goalsArr);
				this.getEnvironment().receiveMessage(goalMessage);
			}
			else if (mode == Mode.ASSIST_FILL) {
				planner.getGoals().add(new Int2D(assistableHoles.peek().getX(), assistableHoles.peek().getY()));

				// Broadcast goal being assisted to indicate contract is no longer available
				TWEntity[] goalsArr = new TWEntity[] {assistableHoles.peek()};
				Message goalMessage = new Message(agentID, 0, MsgType.goalInfo, goalsArr);
				this.getEnvironment().receiveMessage(goalMessage);
			}
			else if (mode == Mode.WAIT) {
				return new TWThought(TWAction.MOVE, TWDirection.Z);
			}

			planner.generatePlan();
			if (!planner.hasPlan()) {
				return new TWThought(TWAction.MOVE, TWDirection.Z);
			}
			return new TWThought(TWAction.MOVE, planner.execute());
		}
    }

    /**
     * 按照think中的行动和方向操作agent移动
     * 可行的行动共4种：
     * MOVE
     * PICKUP
     * PUTDOWN
     * REFUEL
     */
    @Override
    protected void act(TWThought thought) {
        switch (thought.getAction()) {
            case MOVE:
                try {
                    move(thought.getDirection());
                } catch (CellBlockedException ex) {
                    // 移动位置上遇到了障碍物，无法移动，应该考虑rethinking
                    System.out.println("Cell is blocked. Current Position: " + Integer.toString(this.x) + ", " + Integer.toString(this.y));
                }
                break;
            case PICKUP:
                pickUpTile((TWTile) memory.getMemoryGrid().get(this.x, this.y));
				planner.getGoals().clear();
				break;
            case PUTDOWN:
				putTileInHole((TWHole) memory.getMemoryGrid().get(this.x, this.y));
				planner.getGoals().clear();
				break;
			case REFUEL:
				refuel();
				planner.getGoals().clear();
				break;
        }
    }
}
