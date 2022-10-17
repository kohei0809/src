package test.UTest;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import agents.HandAgent;
import agents.IPathPlanner;
import agents.ITargetDecider;
import agents.TargetPathAgent;
import agents.TargetPathAgent_Communication;
import agents.TargetPathAgent_CycleLearning;
import agents.TargetPathAgent_Onebyone;
import agents.TargetPathAgent_PDALearning;
import agents.TargetPathAgent_PlannedStopping;
import agents.TargetPathAgent_Return;
import agents.TargetPathAgent_ReturnAndWait;
import agents.TargetPathAgent_TimeChange;
import agents.TargetPathAgent_TimeChange_Communication;
import agents.TargetPathAgent_TimeChange_Learning;
import agents.TargetPathAgent_TimeChange_PlannedStopping;
import agents.TargetPathAgent_TimeChange_U;
import agents.TargetPathAgent_Wait;
import agents.common.RequirementEstimator;
import agents.common.RequirementEstimatorU;
import agents.path.ShortestGreedyPathPlanner;
import agents.path.ShortestRandomPathPlanner;
import agents.path.SubgoalPathPlanner;
import agents.target.AMTDSLearningTargetDecider;
import agents.target.AMTDSLearningTargetDecider_Energy_Clean;
import agents.target.AMTDSLearningTargetDecider_Security;
import agents.target.GreedyTargetDecider;
import agents.target.LongIntervalTargetDecider;
import agents.target.MyopiaSweepTargetDecider;
import agents.target.RandomTargetDecider;
import core.Coordinate;
import core.GridGraph;
import core.IAgentManager;
import core.IEnvironment;
import core.LitterSpawnPattern;
import core.LitterSpawnProbability;
import core.RobotSpec;
import core.agent.IAgent;
import core.environments.VirtualEnvironment;
import main.simulations.AgentType;
import main.simulations.BehaviorType;
import main.simulations.Evaluator;
import main.simulations.SimulationFactory;


//UTest用の、環境やエージェントの設定を行うクラス
public class UTestSimulationFactory extends SimulationFactory{
    static int[] seeds = new int[] { //ある程度大きい素数
        31,     53,     79,     337,    449,    557,    601,    751,    809,    907, 
        977,    701,    1087,   1129,   1259,   1381,   1453,   1549,   1663,   1721, 
        1823,   2069,   2939,   3463,   4271,   5059,   5779,   6569,   7499,   8287, 
        8999,   9337,   9901,   10007,  10459,  10949,  11443,  11933,  12829,  13309, 
        13781,  14029,  15013,  15451,  15913,  16433,  17431,  17923,  18371,  18947, 
        19949,  20399,  20939,  21419,  22381,  22877,  23369,  23873,  24923,  25447, 
        25943,  26437,  27509,  27997,  28547,  29017,  30103,  30643,  31151,  31657, 
        32653,  33179,  33637,  34213,  35251,  35809,  36319,  36821,  37871,  38449, 
        38971,  39499,  40591,  41143,  41627,  42139,  43117,  43721,  44257,  44789, 
        45341,  45943,  46489,  47057,  47581,  48661,  49177,  49711,  50221,  51361, 
        51853,  52391,  52967,  53527,  54049,  54581,  55163,  55717,  56239,  57241, 
        57791,  58321,  59399,  59971,  59998 
    };

    VirtualEnvironment environment;
    UTestAgentManager agentManager;
    Evaluator evaluator;
    GridGraph graph;
    LitterSpawnPattern spawnPattern;
    
    //イベント発生確率ごとのノードを格納するリスト
    private List<Integer> excludeNodes = new LinkedList<Integer>();
	private List<Integer> highNodes = new LinkedList<Integer>();
	private List<Integer> middleNodes = new LinkedList<Integer>();
	private List<Integer> lowNodes = new LinkedList<Integer>();
	private List<Integer> uniformNodes = new LinkedList<Integer>();
    
    //エージェントの行動に関する変数
	Map<Integer, RobotSpec> robots;
	int basePosition, robotCount = 1, mapScale = 50;
    double requirementLevel, correction;
	BehaviorType beType = BehaviorType.normal;
	AgentType agentType = AgentType.normal;
	boolean isAccumulate = true;

    //イベント発生確率
    final double spawnLow = 1e-6, spawnMiddle = 1e-4, spawnHigh = 1e-3, spawnNone = 0.0;
	final double spawnUniform = 5e-6;
    String patternName = "Uniform";

    //ランダムシード
	int environmentSeed = 0, agentSeed = 0, estimatorSeed = 0;
	int patherSeed = 1, patherNumber = 2;
	int targetterSeed = 2, targetterNumber = 4;

    public void setLocalNumbers(int eseed, int robotC, int s, boolean isA, String pattern, BehaviorType bType, AgentType aType){
        environmentSeed = eseed;
        robotCount = robotC;
        mapScale = s;
        isAccumulate = isA;
        patternName = pattern;
        beType = bType;
        agentType = aType;
    }

    public void setLocalNumbers(int eseed, int aseed, int pseed, int pnumber, int tseed, int tnumber, int robotC, int s, 
            boolean isA, String pattern, BehaviorType bType, AgentType aType, double req, double cor, int rseed){
        environmentSeed = eseed;
        agentSeed = aseed;
        patherSeed = pseed;
        patherNumber = pnumber;
        targetterSeed = tseed;
        targetterNumber = tnumber;

        robotCount = robotC;
        mapScale = s;

        isAccumulate = isA;
        patternName = pattern;
        beType = bType;
        agentType = aType;
        requirementLevel = req;
        correction = cor;

        estimatorSeed = rseed;
    }

    @Override
    public IEnvironment environment(){
        return this.environment;
    }

    public IAgentManager agentManager(){
        return this.agentManager;
    }

    public Evaluator evaluator(){
        return this.evaluator;
    }

    public GridGraph graph(){
        return this.graph;
    }

    public int scale(){
        return this.mapScale;
    }

    public List<Integer> excludeNodes(){
        return this.excludeNodes;
    }

    public List<Integer> highNodes(){
		return this.highNodes;
	}

	public List<Integer> middleNodes(){
		return this.middleNodes;
	}

	public List<Integer> lowNodes(){
		return this.lowNodes;
	}

	public List<Integer> uniformNodes(){
		return this.uniformNodes;
	}

    public void make(){
        createSpatialStructure();
        createLitterSpawnPattern();

        environment = new VirtualEnvironment(graph, spawnPattern, isAccumulate, seeds[environmentSeed]);
        agentManager = new UTestAgentManager(environment, beType, seeds[environmentSeed + 3], graph);

        createRobotBases();
        createRobots(robotCount);

        createAgent();
        environment.update();
        createEvaluator();
    }

    public void createSpatialStructure(){
        Coordinate max = new Coordinate(mapScale, mapScale);
        Coordinate min = new Coordinate(-mapScale, -mapScale);

        graph = new GridGraph(max, min);
    }

    public void createLitterSpawnPattern(){
        Random rand = new Random(seeds[environmentSeed + 1]);
        spawnPattern = new LitterSpawnPattern();

        for(int node : graph.getAllNode()){
            Coordinate c = graph.getCoordinate(node);
            double prob;

            if(patternName == "Test") {
               if(c.x == -1 && c.y == 1 || c.x == 0 && c.y == -2 || c.x == 1 && c.y == 2){
                   prob = 1.0;
               }
               else{
                   prob = 0.0;
               }
            }

            if(patternName == "Test2") {
                if(-2 <= c.x && c.x <= 2 && -2 <= c.y && c.y <= 2){
                    prob = spawnHigh + (rand.nextDouble() - 0.5) * spawnHigh * 0.01;
                    highNodes.add(node);
                }
                else if((-25 <= c.x && c.x <= -16 && c.y == 5) || (-10 <= c.x && c.x <= 10 && c.y == 5) || (16 <= c.x && c.y == 5)
                        || (-25 <= c.x && c.x <= -16 && c.y == -5) || (-10 <= c.x && c.x <= 10 && c.y == -5) || (16 <= c.x && c.y == -5)
                        || (c.x == 0 && 5 <= c.y && c.y <= 25) || (c.x == 0 && -25 <= c.y && c.y <= -5)){
                            prob = spawnNone;
                            excludeNodes.add(node);
                        }
                else{
                    prob = spawnLow + (rand.nextDouble() - 0.5) * spawnLow * 0.01;
                    lowNodes.add(node);
                }
             }
            
            else if(patternName == "Block") {
                if((-50 <= c.x && c.x <= -1 && 1<= c.y && c.y <= 50) || (1 <= c.x && c.x <= 50 && -50 <= c.y && c.y <= -1)){
                    prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                    middleNodes.add(node);
                }
                else if(1 <= c.x && c.x <= 50 && 1 <= c.y && c.y <= 50){
                    prob = spawnHigh + (rand.nextDouble() - 0.5) * spawnHigh * 0.01;
                	highNodes.add(node);
                }
                else{
                    prob = spawnLow + (rand.nextDouble() - 0.5) * spawnLow * 0.01;
					lowNodes.add(node);
                }
            }

            else if(patternName == "Complex"){
                //壁相当ノードの除外リスト
                if((-50 <= c.x && c.x <= -31 && c.y == 6) || (-50 <= c.x && c.x <= -31 && c.y == -6) || (-20 <= c.x && c.x <= 20 && c.y == 6) || 
                    (31 <= c.x && c.x <= 50 && c.y == 6) || (-20 <= c.x && c.x <= 20 && c.y == -6) || (31 <= c.x && c.x <= 50 && c.y == -6) || 
                    (-20 <= c.x && c.x <= 20 && c.y == 6) || (c.x == 0 && 7 <= c.y && c.y <= 50) || (c.x == 0 && -50 <= c.y && c.y <= -7)){
                    prob = spawnNone;
                    excludeNodes.add(node);
                }

                //左上の部屋
                else if((-50 <= c.x && c.x <= -1 && 46 <= c.y && c.y <= 50) || (-50 <= c.x && c.x <= -46 && 12 <= c.y && c.y <= 45) || 
                    (-5 <= c.x && c.x <= -1 && 12 <= c.y && c.y <= 45) || (-50 <= c.x && c.x <= -31 && 7 <= c.y && c.y <= 11) || 
                    (-20 <= c.x && c.x <= -1 && 7 <= c.y && c.y <= 11)){
                        prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                        middleNodes.add(node);
                }

                //右上の部屋
                else if(5 <= c.x && c.x <= 9 && 41 <= c.y && c.y <=45){
                    prob = spawnHigh + (rand.nextDouble() - 0.5) * spawnHigh * 0.01;
                	highNodes.add(node);
                }

                else if((15 <= c.x && c.x <= 19 && 17 <= c.y && c.y <= 21) || (30 <= c.x && c.x <= 34 && 31 <= c.y && c.y <= 35) || (41 <= c.x && c.x <= 45 && 10 <= c.y && c.y <= 14)){
                    prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                    middleNodes.add(node);
                } 

                //左下の部屋
                else if(-30 <= c.x && c.x <= -21 && -33 <= c.y && c.y <= -24){
                    prob = spawnHigh + (rand.nextDouble() - 0.5) * spawnHigh * 0.01;
                	highNodes.add(node);
                }

                else{
                    prob = spawnLow + (rand.nextDouble() - 0.5) * spawnLow * 0.01;
                	lowNodes.add(node);
                }
            }

            else if(patternName == "Office") {
				//壁相当ノードの除外リスト
                if ((c.x == -17 && 5 <= c.y) || (c.x == -17 && c.y <= -5) || (c.x == 17 && 5 <= c.y) || (c.x == 17 && c.y <= -5) || (-50 <= c.x && c.x <= -37 && c.y == -5) ||
                    (-31 <= c.x && c.x <= -3 && c.y == -5) || (3 <= c.x && c.x <= 31 && c.y == -5) || (37 <= c.x && c.x <= 50 && c.y == -5) || (-50 <= c.x && c.x <= -37 && c.y == 5) ||
                    (-31 <= c.x && c.x <= -3 && c.y == 5) || (3 <= c.x && c.x <= 31 && c.y == 5) || (37 <= c.x && c.x <= 50 && c.y == 5)) {
                	prob = spawnNone;
                	excludeNodes.add(node);
                }

                //左上の部屋
                else if ((c.x <= -47 && 6 <= c.y) || (-21 <= c.x && c.x <= -18 && 6 <= c.y) || (-46 <= c.x && c.x <= -22 && 47 <= c.y) ||
                    (-46 <= c.x && c.x <= -37 && 6 <= c.y && c.y <= 9) || (-31 <= c.x && c.x <= -18 && 6 <= c.y && c.y <= 9)) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                //上の部屋
                else if (-11 <= c.x && c.x <= -7 && 41 <= c.y && c.y <= 45) {
                	prob = spawnHigh + (rand.nextDouble() - 0.5) * spawnHigh * 0.01;
                	highNodes.add(node);
                }

                else if (3 <= c.x && c.x <= 7 && 25 <= c.y && c.y <= 29) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                //右上の部屋
                else if (19 <= c.x && c.x <= 23 && 15 <= c.y && c.y <= 19) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                else if (23 <= c.x && c.x <= 27 && 7 <= c.y && c.y <= 11) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                else if (24 <= c.x && c.x <= 28 && 33 <= c.y && c.y <= 37) {
                	prob = spawnHigh + (rand.nextDouble() - 0.5) * spawnHigh * 0.01;
                	highNodes.add(node);
                }

                else if (38 <= c.x && c.x <= 42 && 17 <= c.y && c.y <= 21) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                else if (40 <= c.x && c.x <= 44 && 41 <= c.y && c.y <= 45) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                //左下の部屋
                else if (-41 <= c.x && c.x <= -28 && -37 <= c.y && c.y <= -19) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                //下の部屋
                else if ((-16 <= c.x && c.x <= -13 && c.y <= -6) || (-12 <= c.x && c.x <= 16 && c.y <= -47) ||
                         (13 <= c.x && c.x <= 16 && c.y <= -6) || (-12 <= c.x && c.x <= -3 && -9 <= c.y && c.y <= -6) ||
                         (3 <= c.x && c.x <= 16 && -9 <= c.y && c.y <= -6)) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                else if (-8 <= c.x && c.x <= -4 && -20 <= c.y && c.y <= -16) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                else if (2 <= c.x && c.x <= 6 && -38 <= c.y && c.y <= -34) {
                	prob = spawnHigh + (rand.nextDouble() - 0.5) * spawnHigh * 0.01;
                	highNodes.add(node);
                }

                //その他
                else {
                	prob = spawnLow + (rand.nextDouble() - 0.5) * spawnLow * 0.01;
                	lowNodes.add(node);
                }
			}
            else{
                prob = spawnUniform * rand.nextDouble();
                uniformNodes.add(node);
            } 
            spawnPattern.addSpawnProbability(node, new LitterSpawnProbability(1, prob, 1));           
        }

        removeExcludeEdges();
    }

    private void removeExcludeEdges(){

        graph.removeObstacleNodes(excludeNodes);

        for(int node : excludeNodes) {
			List<Integer> tmp = new LinkedList<Integer>();

			for(int endpoint : graph.getChildNodes(node)) {
				tmp.add(endpoint);
			}

			for(int end : tmp) {
				graph.removeEdge(node, end);
				graph.removeEdge(end, node);
			}
		}
    }

    public void createRobotBases(){
        basePosition = graph.getNode(0, 0);
        environment.setRobotBase(1, basePosition);
        spawnPattern.addSpawnProbability(basePosition, new LitterSpawnProbability(1, 0.0, 0));
    }

    public void createRobots(int num){
        robots = new HashMap<Integer, RobotSpec>();
        RobotSpec spec = new RobotSpec(2700, 3);

        for(int i = 0; i < num; i++){
            int robotid = environment.createRobot(spec, graph.getNode(0, 0));
            robots.put(robotid, spec);
        }
    }

    protected void createAgent(){
        Random aRand = new Random(seeds[agentSeed]);
        Random pRand = new Random(seeds[patherSeed]);
        Random tRand = new Random(seeds[targetterSeed]);
        Random eRand = new Random(seeds[estimatorSeed]);

        for(int robot : robots.keySet()){
            IAgent agent;
            switch(agentType){
                case hand:
                    //手動
                    agent = new HandAgent(robot, getBehaviorList());
                    break;
                case normal:
                    //米田さん手法
                    agent = new TargetPathAgent(robot, spawnPattern);
                    break;
                case PDALearning:
                default:
                    //AMTDS/LD
                    agent = new TargetPathAgent_PDALearning(robot, spawnPattern, graph, excludeNodes);
                    break;
                case Communicating:
                    //PDALearning + Communication
                    agent = new TargetPathAgent_Communication(robot, spawnPattern, graph, excludeNodes);
                    break;
                case PlannedStopping:
                    //PlannedStop
                    agent = new TargetPathAgent_PlannedStopping(robot, spawnPattern, graph, excludeNodes);
                    break;
                case Homing:
                    //Homing
                    agent = new TargetPathAgent_Return(robot, spawnPattern, graph, aRand.nextInt());
                    break;
                case Pausing:
                    //Pausing
                    agent = new TargetPathAgent_Wait(robot, spawnPattern, graph, aRand.nextInt());
                    break;
                case HomingAndPausing:
                    //HomingAndPausing
                    agent = new TargetPathAgent_ReturnAndWait(robot, spawnPattern, graph, aRand.nextInt());
                    break;
                case TimeChange:
                    //TimeChange
                    agent = new TargetPathAgent_TimeChange(robot, spawnPattern, graph, aRand.nextInt(), excludeNodes);
                    break;
                case TimeChange_Learning:
                    //TimeChange_Learning
                    agent = new TargetPathAgent_TimeChange_Learning(robot, spawnPattern, graph, aRand.nextInt(), excludeNodes);
                    break;
                case TimeChange_Communication:
                    //TimeChange_Communication
                    agent = new TargetPathAgent_TimeChange_Communication(robot, spawnPattern, graph, aRand.nextInt(), excludeNodes);
                    break;
                case TimeChange_PlannedStopping:
                    //TimeChange_PlannedStopping
                    agent = new TargetPathAgent_TimeChange_PlannedStopping(robot, spawnPattern, graph, aRand.nextInt(), excludeNodes);
                    break;
                case Onebyone:
                    //Onebyone
                    agent = new TargetPathAgent_Onebyone(robot, spawnPattern, graph, aRand.nextInt(), excludeNodes);
                    break;
                case TimeChange_U:
                    //TimeChange_U
                    agent = new TargetPathAgent_TimeChange_U(robot, spawnPattern, graph, aRand.nextInt(), excludeNodes);
                    break;
                case CycleLearning:
                    //CycleLearning(U(s))
                    agent = new TargetPathAgent_CycleLearning(robot, spawnPattern, graph, aRand.nextInt(), excludeNodes);
                    break;
            }

            ITargetDecider targetter;
            IPathPlanner pather;

            // Path planner
            switch(patherNumber){
                case 0:
                    pather = new ShortestRandomPathPlanner(robot, basePosition, graph, pRand.nextInt(), excludeNodes);
                    break;
                case 1:
                    pather = new ShortestGreedyPathPlanner(robot, graph, spawnPattern, basePosition, isAccumulate, pRand.nextInt(), excludeNodes);
                    break;
                default:
                    pather = new SubgoalPathPlanner(robot, graph, spawnPattern, basePosition, isAccumulate, pRand.nextInt(), excludeNodes);
                    break;
            }

            //Target decider
            switch(targetterNumber){
                case 0:
                    targetter = new RandomTargetDecider(robot, graph, tRand.nextInt(), excludeNodes);
                    break;

                case 1:
                    targetter = new GreedyTargetDecider(robot, graph, spawnPattern, isAccumulate, tRand.nextInt(), excludeNodes);
                    break;
                case 2:
                    targetter = new LongIntervalTargetDecider(robot, graph, tRand.nextInt(), excludeNodes);
                    break;
                case 3:
                    targetter = new MyopiaSweepTargetDecider(robot, graph, spawnPattern, isAccumulate, tRand.nextInt(), excludeNodes);
                    break;
                case 4:
                default:
                    AMTDSLearningTargetDecider decider4 = new AMTDSLearningTargetDecider(robot, graph, spawnPattern, isAccumulate, tRand.nextInt());
                    decider4.addTargetDecider(new RandomTargetDecider(robot, graph, tRand.nextInt(), excludeNodes));
                    decider4.addTargetDecider(new GreedyTargetDecider(robot, graph, spawnPattern, isAccumulate, tRand.nextInt(), excludeNodes));
                    decider4.addTargetDecider(new LongIntervalTargetDecider(robot, graph, tRand.nextInt(), excludeNodes));
                    decider4.addTargetDecider(new MyopiaSweepTargetDecider(robot, graph, spawnPattern, isAccumulate, tRand.nextInt(), excludeNodes));
                    targetter = decider4;
                    break;
                case 5:
                    AMTDSLearningTargetDecider_Energy_Clean decider5 = new AMTDSLearningTargetDecider_Energy_Clean(robot, graph, spawnPattern, isAccumulate, tRand.nextInt());
                    decider5.addTargetDecider(new RandomTargetDecider(robot, graph, tRand.nextInt(), excludeNodes));
                    decider5.addTargetDecider(new GreedyTargetDecider(robot, graph, spawnPattern, isAccumulate, tRand.nextInt(), excludeNodes));
                    decider5.addTargetDecider(new LongIntervalTargetDecider(robot, graph, tRand.nextInt(), excludeNodes));
                    decider5.addTargetDecider(new MyopiaSweepTargetDecider(robot, graph, spawnPattern, isAccumulate, tRand.nextInt(), excludeNodes));
                    targetter = decider5;
                    break;    
                case 6:
                    AMTDSLearningTargetDecider_Security decider6 = new AMTDSLearningTargetDecider_Security(robot, graph, spawnPattern, isAccumulate, tRand.nextInt());
                    decider6.addTargetDecider(new RandomTargetDecider(robot, graph, tRand.nextInt(), excludeNodes));
                    decider6.addTargetDecider(new GreedyTargetDecider(robot, graph, spawnPattern, isAccumulate, tRand.nextInt(), excludeNodes));
                    decider6.addTargetDecider(new LongIntervalTargetDecider(robot, graph, tRand.nextInt(), excludeNodes));
                    decider6.addTargetDecider(new MyopiaSweepTargetDecider(robot, graph, spawnPattern, isAccumulate, tRand.nextInt(), excludeNodes));
                    targetter = decider6;
                    break;
            }

            RequirementEstimator estimator = new RequirementEstimator(graph, basePosition, correction);
            RequirementEstimatorU estimatorU = new RequirementEstimatorU(graph, basePosition, correction, eRand.nextInt());

            agent.setBaseNode(basePosition);
            agent.setPathPlanner(pather);
            agent.setTargetDecider(targetter);
            agent.setEnvironmentEstimator(estimator);
            agent.setEnvironmentEstimatorU(estimatorU);
			agent.setRequirement(requirementLevel);
            agent.setExpectation();
            agentManager.addAgent(agent);
            agentManager.setExcludingNodes(excludeNodes);
        }
    }

    void createEvaluator(){
        evaluator = new Evaluator(environment, agentManager);
    }
    
    List<Integer> getBehaviorList(){
        List<Integer> bList = new LinkedList<Integer>();
        /*int node;
        Coordinate coo;
        node = graph.getNode(-1, 0);
        coo = graph.getCoordinate(node);
        //System.out.println("add (" + coo.x + "," + coo.y + ") node=" + node);
        bList.add(node);
        node = graph.getNode(-1, 1);
        coo = graph.getCoordinate(node);
        //System.out.println("add (" + coo.x + "," + coo.y + ") node=" + node);
        bList.add(node);
        node = graph.getNode(-1, 2);
        coo = graph.getCoordinate(node);
        //System.out.println("add (" + coo.x + "," + coo.y + ") node=" + node);
        bList.add(node);
        node = graph.getNode(0, 2);
        coo = graph.getCoordinate(node);
        //System.out.println("add (" + coo.x + "," + coo.y + ") node=" + node);
        bList.add(node);
        node = graph.getNode(1, 2);
        coo = graph.getCoordinate(node);
        //System.out.println("add (" + coo.x + "," + coo.y + ") node=" + node);
        bList.add(node);
        node = graph.getNode(1, 1);
        coo = graph.getCoordinate(node);
        //System.out.println("add (" + coo.x + "," + coo.y + ") node=" + node);
        bList.add(node);
        node = graph.getNode(1, 0);
        coo = graph.getCoordinate(node);
        //System.out.println("add (" + coo.x + "," + coo.y + ") node=" + node);
        bList.add(node);
        node = graph.getNode(2, 0);
        coo = graph.getCoordinate(node);
        //System.out.println("add (" + coo.x + "," + coo.y + ") node=" + node);
        bList.add(node);
        node = graph.getNode(2, -1);
        coo = graph.getCoordinate(node);
        //System.out.println("add (" + coo.x + "," + coo.y + ") node=" + node);
        bList.add(node);
        node = graph.getNode(2, -2);
        coo = graph.getCoordinate(node);
        //System.out.println("add (" + coo.x + "," + coo.y + ") node=" + node);
        bList.add(node);
        node = graph.getNode(1, -2);
        coo = graph.getCoordinate(node);
        //System.out.println("add (" + coo.x + "," + coo.y + ") node=" + node);
        bList.add(node);
        node = graph.getNode(0, -2);
        coo = graph.getCoordinate(node);
        //System.out.println("add (" + coo.x + "," + coo.y + ") node=" + node);
        bList.add(node);
        node = graph.getNode(-1, -2);
        coo = graph.getCoordinate(node);
        //System.out.println("add (" + coo.x + "," + coo.y + ") node=" + node);
        bList.add(node);
        node = graph.getNode(-1, -1);
        coo = graph.getCoordinate(node);
        //System.out.println("add (" + coo.x + "," + coo.y + ") node=" + node);
        bList.add(node);
        node = graph.getNode(0, -1);
        coo = graph.getCoordinate(node);
        //System.out.println("add (" + coo.x + "," + coo.y + ") node=" + node);
        bList.add(node);
        node = graph.getNode(0, 0);
        coo = graph.getCoordinate(node);
        //System.out.println("add (" + coo.x + "," + coo.y + ") node=" + node);
        bList.add(node);*/

        return  bList;
    }
}   
