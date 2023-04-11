package tileworld.agent;

import java.awt.*;
import java.util.*;
import java.util.List;

import sim.display.GUIState;
import sim.portrayal.Inspector;
import sim.portrayal.LocationWrapper;
import sim.portrayal.Portrayal;
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
    private double fuelTolerance=0.8;

    /**
     * Hard fuel limit before needing to refuel
     */
    private double hardFuelLimit=50;

    /**
     * Modifies the heuristic used for prioritizing the list of possible goals.
     */
    private boolean TSPHeuristic=false;

    /**
     * Lifetime threshold used for determining whether a object at risk of decay
     * should be pursued,
     * taking into account both its estimated remaining lifetime and its distance
     * from agent.
     * Estimated remaining time left for a memorized object is based on its memory
     * time stamp.
     */
    private double objectLifetimeThreshold=1.0;

    /**
     * Maximum number of goals in queue to announce.
     * Announcing goals prevent goal collisions.
     * However, reserving too many goals can lead to sub-optimal division of goals
     * between agents.
     * Reserving too little can result in collision between an assisting agent's
     * goal and this agent's next immediate goal,
     * thus wasting the assisting agent's resources.
     */
    private int goalAnnounceCount=1;

    private boolean allowAssistance=false;

    /**
     * Furthest zone agent can move to assist, specified in terms of number of zones
     */
    private int maxAssistZoneDistance=1;

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
//    private TWAgentWorkingMemory workingMemory;
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
        this.agentID = Character.getNumericValue(name.charAt(name.length() - 1)); // 这里不需要减一，因为agentID为0在communicate的时候代表全部Agent
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
        this.myZoneID = new HashMap<>();
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
                new Object[] { memory.getAgentPercept(), new Int2D(x, y) });
        this.getEnvironment().receiveMessage(message);

        // 判断是否初始化完成（划分好区域了）
        if (bounds[0] == null) return;

        // 是的话就
        // 判断可达性--添加可能列表--添加招标列表--发送自己的goal和招标信息
        // 注意判断各种限制，比如最大可携带的tile数量
        tilesInZone = memory.getNearbyObjectsWithinBounds(bounds, TWTile.class);
        holesInZone = memory.getNearbyObjectsWithinBounds(bounds, TWHole.class);

        possibleTileGoals.clear();
        possibleHoleGoals.clear();

        LinkedList<TWEntity> goals = new LinkedList<TWEntity>();
        LinkedList<TWEntity> auctionTiles = new LinkedList<TWEntity>();
        LinkedList<TWEntity> auctionHoles = new LinkedList<TWEntity>();

        // 判断可达性--添加可能列表
        while (!tilesInZone.isEmpty()) {
            TWEntity tile = tilesInZone.poll();
            double distToTile = this.getDistanceTo(tile);
            if (memory.getEstimatedRemainingLifetime(tile, this.objectLifetimeThreshold) <= distToTile) auctionTiles.add(tile);
            else possibleTileGoals.add(tile);
        }

        while (!holesInZone.isEmpty()) {
            TWEntity hole = holesInZone.poll();
            double distToHole = this.getDistanceTo(hole);
            if (memory.getEstimatedRemainingLifetime(hole, this.objectLifetimeThreshold) <= distToHole) auctionHoles.add(hole);
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
                if (!(memory.getEstimatedRemainingLifetime(twEntity, this.objectLifetimeThreshold) <= distToObj)) queue.add(twEntity);
            }
        }
    }

    /**
     * 计算TSP距离，用于比较两个对象之间的距离
     * @param o 要计算距离的目标实体
     * @return 返回TSP距离
     */
    public double getTSPDistance(TWEntity o) {
        double oDist = getDistanceTo(o);
        // Modifies Manhattan distance by lifetime remaining, so between two equidistant objects, the one with a shorter lifetime is closer
        if (this.TSPHeuristic) {
            oDist *= memory.getEstimatedRemainingLifetime(o, 1.0)/Parameters.lifeTime;
        }
        return oDist;
    }

    /**
     * 初始化分配区域,并初始化closestTile和closestHole
     */
    private void assignZoneAndFindEntities() {
        if (bounds[0] != null) {
            return;
        }
        assignZone(this.getEnvironment().getxDimension(), this.getEnvironment().getyDimension());
        tilesInZone = memory.getNearbyObjectsWithinBounds(bounds, new TWTile().getClass());
        holesInZone = memory.getNearbyObjectsWithinBounds(bounds, new TWHole().getClass());
        while (!tilesInZone.isEmpty()) {
            TWEntity tile = tilesInZone.poll();
            double distToTile = this.getDistanceTo(tile);
            if (!(memory.getEstimatedRemainingLifetime(tile, this.objectLifetimeThreshold) <= distToTile)) {
                possibleTileGoals.add(tile);
            }
        }
        while (!holesInZone.isEmpty()) {
            TWEntity hole = holesInZone.poll();
            double distToHole = this.getDistanceTo(hole);
            if (!(memory.getEstimatedRemainingLifetime(hole, this.objectLifetimeThreshold) <= distToHole)) {
                possibleHoleGoals.add(hole);
            }
        }
        closestTile = new TWEntity[possibleTileGoals.size()];
        closestHole = new TWEntity[possibleHoleGoals.size()];
        closestTile = possibleTileGoals.toArray(closestTile);
        closestHole = possibleHoleGoals.toArray(closestHole);
        System.out.printf("Agent [%d]: LeftUp(%d %d), rightDown(%d %d), \n",agentID, bounds[0].x, bounds[0].y, bounds[2].x, bounds[2].y);
    }

    /**
     * 获取可协助的对象
     * @return 返回一个列表，其中包含可协助收集的tile和可协助填充的hole
     */
    private List<PriorityQueue<TWEntity>> getAssistableObjects() {
        Comparator<TWEntity> distHeur = new Comparator<TWEntity>() {
            public int compare(TWEntity o1, TWEntity o2) {
                return (int) (getTSPDistance(o1) - getTSPDistance(o2));
            }
        };
        PriorityQueue<TWEntity> assistableTiles = new PriorityQueue<TWEntity>(10, distHeur);
        PriorityQueue<TWEntity> assistableHoles = new PriorityQueue<TWEntity>(10, distHeur);

        ArrayList<Message> messages = this.getEnvironment().getMessages();
        for (int i = 0; i < messages.size(); i++) {
            Message message = messages.get(i);
            if (message.getSender() != agentID &&
                    message.getReceiver() == 0) {
                if (message.getMessageType() == MsgType.contractInfo_tile) {
                    mergeContracts(assistableTiles, (TWEntity[]) message.getMessageContent()[0], (int) message.getMessageContent()[1]);
                } else if (message.getMessageType() == MsgType.contractInfo_hole) {
                    mergeContracts(assistableHoles, (TWEntity[]) message.getMessageContent()[0], (int) message.getMessageContent()[1]);
                }
            }
        }

        for (int i = 0; i < messages.size(); i++) {
            Message message = messages.get(i);
            if (message.getSender() != agentID &&
                    message.getReceiver() == 0 &&
                    message.getMessageType() == MsgType.goalInfo) {
                TWEntity[] announcedGoals = (TWEntity[]) message.getMessageContent();
                for (int j = 0; j < announcedGoals.length; j++) {
                    if (announcedGoals[j] instanceof TWTile) {
                        assistableTiles.remove(announcedGoals[j]);
                        possibleTileGoals.remove(announcedGoals[j]);
                    } else if (announcedGoals[j] instanceof TWHole) {
                        assistableHoles.remove(announcedGoals[j]);
                        possibleHoleGoals.remove(announcedGoals[j]);
                    }
                }
            }
        }

        List<PriorityQueue<TWEntity>> result = new ArrayList<>();
        result.add(assistableTiles);
        result.add(assistableHoles);

        return result;
    }

    /**
     * 获取锚点目标
     * @return 返回具有最高探索得分的锚点, 用于生成探索路径
     */
    private Int2D getAnchorGoal() {
        // 收集所有锚点的探索分数
        Int2D anchorGoal = anchors[0];
        Double max_score = Double.NEGATIVE_INFINITY;

        for (int i = 0; i < anchors.length; i++) {
            Double curExplorationScore = memory.getAnchorExplorationScore(anchors[i]);
            Double distToAnchor = this.getDistanceTo(anchors[i].x, anchors[i].y);
//            System.out.printf("Anchors: %d, %d CurExplorationScore: %f\n", anchors[i].x, anchors[i].y, curExplorationScore);
            if (curExplorationScore > max_score ||
                    ((curExplorationScore.equals(max_score)) &&
                            (distToAnchor < this.getDistanceTo(anchorGoal.x, anchorGoal.y)))
            ) {
                max_score = curExplorationScore;

                // 如果被阻挡，寻找具有最高探索得分的替代位置
                if (memory.isCellBlocked(anchors[i].x, anchors[i].y)) {
                    ArrayList<Int2D> alternativeAnchors = new ArrayList<Int2D>();
                    ArrayList<Double> alternativeScores = new ArrayList<Double>();
                    for (int j = -1; j <= 1; j++) {
                        for (int k = -1; k <= 1; k++) {
                            if (anchors[i].x + j < this.getEnvironment().getxDimension() &&
                                    anchors[i].y + k < this.getEnvironment().getyDimension() &&
                                    anchors[i].x - j >= 0 &&
                                    anchors[i].y + k >= 0 &&
                                    !memory.isCellBlocked(anchors[i].x + j, anchors[i].y + k)) {
                                alternativeAnchors.add(new Int2D(anchors[i].x + j, anchors[i].y + k));
                                alternativeScores.add(memory.getAnchorExplorationScore(alternativeAnchors.get(alternativeAnchors.size() - 1)));
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
        return anchorGoal;
    }

    /**
     * 根据Agent当前状态设置行动模式Mode
     * @param assistableTiles 可协助收集的瓦片
     * @param assistableHoles 可协助填充的洞
     */
    private void setMode(PriorityQueue<TWEntity> assistableTiles, PriorityQueue<TWEntity> assistableHoles) {
        // 默认模式
        mode = Mode.EXPLORE;

        // 如果已经找到加油站并且燃料不足，加油优先级最高
        if (memory.getFuelStation() != null && this.getDistanceTo(memory.getFuelStation().x, memory.getFuelStation().y) >= this.fuelLevel * fuelTolerance) {
            mode = Mode.REFUEL;
        }
        // 如果尚未找到加油站，探索优先级最高
        else if (memory.getFuelStation() == null) {
            mode = Mode.EXPLORE;
        }
//        // 下列代码感觉可以删除
        else if (this.fuelLevel <= this.hardFuelLimit) {
            // 极其罕见的情况，即在第一阶段探索过程中未发现加油站
            // 代理等待其他代理完成他们的区域探索，希望能找到加油站
            // 如果加油站在此代理的区域内，其他代理必须协助探索
            // 此区域剩余部分，希望有足够的燃料来支持
            // (ASSIST_EXPLORE 尚未编程，需要向环境广播剩余探索地图)
            if (memory.getFuelStation() == null) {
                mode = Mode.WAIT;
            }
            else {
                mode = Mode.REFUEL;
            }
        }
        // 如果没有tile并且附近有tile，收集tile，否则探索
        else if (!this.hasTile()) {
            if (closestTile.length > 0) {
                mode = Mode.COLLECT;
            }
            else if (allowAssistance && !assistableTiles.isEmpty()) {
                mode = Mode.ASSIST_COLLECT;
            }
        }
        // 如果有tile并且附近有一个hole，优先填充hole
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
        // 如果未达到最大数量的tile且附近只有tile，收集tile
        else if (closestTile.length > 0 && this.carriedTiles.size() < 3) {
            mode = Mode.COLLECT;
        }
        // 如果自己的区域没有瓦片和洞，但相邻区域有可协助的洞，
        // 协助相邻区域的代理填充洞
        else if (allowAssistance && !assistableHoles.isEmpty()) {
            mode = Mode.ASSIST_FILL;
        }
    }


    /**
     * 主要的思考函数
     * @return 返回TWThought对象, Agent的act方法接受这个对象并做出对应的行动
     */
    @Override
    protected TWThought think() {
        // [初始化] 初始化划分区域, 设置默认的closestTile和closestHole
        assignZoneAndFindEntities();

		// [知识共享] 获取环境中的所有Messages,并将类别是agentInfo(MAP)的MessageContent合并到自己的memory里面
		ArrayList<Message> messages = this.getEnvironment().getMessages();
		for (int i = 0; i < messages.size(); i++) {
			Message message = messages.get(i);
			if (message.getSender() != agentID &&
				message.getReceiver() == 0 &&
				message.getMessageType() == MsgType.agentInfo) {
				memory.mergeMemory((TWAgentPercept[][]) message.getMessageContent()[0], (Int2D) message.getMessageContent()[1]);
			}
		}

		// [获取可协同的目标] 从Messages中找到可协助的assitableTiles和assitableHoles, 并移除有conflict的object(也即是别的Agent的Goal)
        List<PriorityQueue<TWEntity>> assistableObjects = getAssistableObjects();
        PriorityQueue<TWEntity> assistableTiles = assistableObjects.get(0);
        PriorityQueue<TWEntity> assistableHoles = assistableObjects.get(1);

        // [设置当前行动模式] 设置Agent的行动模式
		setMode(assistableTiles, assistableHoles);

        // [设置单步行动计划] 根据行动模式mode设置细粒度的行动计划: 每一步删除计划并重新评估目标，而不是检查新的/衰减的tile
		planner.getGoals().clear();
		planner.voidPlan();

//        System.out.println(fuelLevel);

        // 即使在优先考虑探索的情况下，始终检查是否可以拾起/填充/加油
//		Object curLocObj = this.memory.getMemoryGrid().get(x, y);
        Object curLocObj = this.getEnvironment().getObjectGrid().get(x, y);
		if (curLocObj instanceof TWHole &&
			this.getEnvironment().canPutdownTile((TWHole) curLocObj, this)) {
			mode = Mode.REACT_FILL;
            // 宣布目标，因为可能目标不在自己区域的视野范围内，而是在前往或离开加油站的途中遇到的
            // 如果是这样，可能会出现目标冲突的情况
			TWEntity[] goalsArr = new TWEntity[] {this.memory.getObjects()[x][y].getO()};
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

			TWEntity[] goalsArr = new TWEntity[] {this.memory.getObjects()[x][y].getO()};
			Message goalMessage = new Message(agentID, 0, MsgType.goalInfo, goalsArr);
			this.getEnvironment().receiveMessage(goalMessage);

			planner.getGoals().add(new Int2D(this.x, this.y));
			return new TWThought(TWAction.PICKUP, null);
		}
        // 如果偶然碰到加油站，燃料不足75%时加油。
        // 这与使用fuelTolerance阈值的燃料管理机制不同
		else if (curLocObj instanceof TWFuelStation &&
				 this.fuelLevel < (0.75 * Parameters.defaultFuelLevel))
		{
			planner.getGoals().add(new Int2D(this.x, this.y));
			return new TWThought(TWAction.REFUEL, null);
		}
        // 需要生成TWDirection和计划的模式
		else {
			// getMemory().getClosestObjectInSensorRange(Tile.class);
			if (mode == Mode.EXPLORE) {
                // 收集所有锚点的探索分数
				Int2D anchorGoal = getAnchorGoal();
				planner.getGoals().add(anchorGoal);
//                System.out.println("Cur Loc: " + this.x + " " + this.y + " Goals: " + planner.getGoals());
			}
			else if (mode == Mode.REFUEL) {
				planner.getGoals().add(memory.getFuelStation());
			}
			else if (mode == Mode.COLLECT) {
				planner.getGoals().add(new Int2D(closestTile[0].getX(), closestTile[0].getY()));
			}
			else if (mode == Mode.FILL) {
				planner.getGoals().add(new Int2D(closestHole[0].getX(), closestHole[0].getY()));
			}
			else if (mode == Mode.ASSIST_COLLECT) {
				planner.getGoals().add(new Int2D(assistableTiles.peek().getX(), assistableTiles.peek().getY()));

                // 广播正在协助的目标，表明contract不再可用
				TWEntity[] goalsArr = new TWEntity[] {assistableTiles.peek()};
				Message goalMessage = new Message(agentID, 0, MsgType.goalInfo, goalsArr);
				this.getEnvironment().receiveMessage(goalMessage);
			}
			else if (mode == Mode.ASSIST_FILL) {
				planner.getGoals().add(new Int2D(assistableHoles.peek().getX(), assistableHoles.peek().getY()));

                // 广播正在协助的目标，表明contract不再可用
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
     * 可行的单步行动共4种：
     * MOVE
     * PICKUP
     * PUTDOWN
     * REFUEL
     */
    @Override
    protected void act(TWThought thought) {
        System.out.println("Agent" + agentID + " Mode:" + mode + " Location:" + x + "," + y + " Goal: " + planner.getCurrentGoal() + " FuelLevel: " + fuelLevel + " FuelStation: " + this.memory.getFuelStation() + "Current Carry: " + carriedTiles.size());
        System.out.println("Agent" + agentID + " closestHole.length: " + closestHole.length + " closestTile.length: " + closestTile.length);
        switch (thought.getAction()) {
            case MOVE:
//                System.out.println(thought.getAction());
//                System.out.println(thought.getDirection());
                try {
                    move(thought.getDirection());
                } catch (CellBlockedException ex) {
                    // 移动位置上遇到了障碍物，无法移动，应该考虑rethinking
                    System.out.println("Cell is blocked. Current Position: " + Integer.toString(this.x) + ", " + Integer.toString(this.y));
                }
                break;
            case PICKUP:
                pickUpTile((TWTile) getEnvironment().getObjectGrid().get(this.x, this.y));
				planner.getGoals().clear();
				break;
            case PUTDOWN:
				putTileInHole((TWHole) getEnvironment().getObjectGrid().get(this.x, this.y));
				planner.getGoals().clear();
				break;
			case REFUEL:
				refuel();
				planner.getGoals().clear();
				break;
        }
    }
}
