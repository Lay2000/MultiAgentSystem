package tileworld.agent;

import java.util.LinkedList;
import java.util.PriorityQueue;

import sim.util.Int2D;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.planners.DefaultTWPlanner;

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
     * 第agentID个元素代表其分配到的区域编号
     * 区域可按顺序编号，比如从左到右均匀划分地图5块，从左至右为0到4号
     */
    private Integer[] myZoneID;
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

        // 判断是否初始化完成（划分好区域了）
        // 是的话就
        // 判断可达性--添加可能列表--添加招标列表--发送自己的goal和招标信息
        // 注意判断各种限制，比如最大可携带的tile数量
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
    }

    /**
     * 主要的思考函数
     */
    @Override
    protected TWThought think() {
        assignZone(this.getEnvironment().getxDimension(), this.getEnvironment().getyDimension());
        // 然后进行知识共享，发送message进行记忆合并
        // 分析招标协助，把可能的标加入可协助表，并广播
        // 接收广播，解决可能的冲突
        // 思考行动模式（可以按他们ppt里的大逻辑图来）
        // 重新思考行动计划（包括清除现有计划、检查当前位置、按照当前位置信息和行动模式来确定最终的goal，并使用planner生成路径，得到行动方向）

        return null;
    }

    /**
     * 按照think中的行动和方向操作agent移动
     */
    @Override
    protected void act(TWThought thought) {
        //
    }

}
