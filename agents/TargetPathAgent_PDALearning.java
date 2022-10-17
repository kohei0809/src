package agents;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import agents.common.LitterExistingExpectation;
import agents.common.RequirementEstimator;
import agents.common.RequirementEstimatorU;
import core.CommunicationDetails;
import core.Coordinate;
import core.GridGraph;
import core.IEnvironment;
import core.LitterSpawnPattern;
import core.LitterSpawnProbability;
import core.RobotData;
import core.agent.AgentActions;
import core.agent.IAgent;
import core.agent.ObservedData;

public class TargetPathAgent_PDALearning implements IAgent{
    //ごみの発生確率を学習する手法 AMTDS/LD

    ITargetDecider targetter;
    IPathPlanner pather;
    int baseNode, target, nextNode, requirement;
    double sleepProb = 0.0;
    boolean isChargeRequired = false;
    LitterExistingExpectation expectation;
    Random rand;
    int scale = 50;
    int interval = 3600;

    LitterSpawnPattern mySpawnPattern; //agent's map
    LitterSpawnPattern realSpawnPattern; //real map
    GridGraph graph;
    List<Integer> excludeNodes;
    List<Integer> nodes;
    List<Integer> searchNodes;
    
    public int[][] positionCount = new int[2*scale+1][2*scale+1];

    private int robotID;
    private AgentActions action;
    private int stopTime = -1;
    private int restartTime = -1;

    public TargetPathAgent_PDALearning(int id, LitterSpawnPattern pattern, GridGraph graph, List<Integer> excludeNodes){
        setRobotID(id);
        setAction(AgentActions.standby);
        this.excludeNodes = excludeNodes;
        this.graph = graph;
        nodes = new LinkedList<Integer>(this.graph.getAllNode());

        for(int node : excludeNodes){
            nodes.remove(nodes.indexOf(node));
        }
        searchNodes = new LinkedList<Integer>(nodes);

        realSpawnPattern = pattern;
        InitializeMySpawnPattern();

        expectation = new LitterExistingExpectation(mySpawnPattern, true);
    }

    @Override
    public void update(ObservedData data){
        update(data, false);
    }

    @Override
    public void update(ObservedData data, boolean flipCoin){
        RobotData robotData = data.getRobotDataCollection().getRobotData(robotID);
        int position = robotData.getPosition();
        int interval = expectation.getInterval(position, data.getTime());
        int vacuumedLitter = robotData.getVacuumedLitter();

        // Update agent's probability map
        mySpawnPattern.setProbability(position, vacuumedLitter, interval);

        // Available to know other's position
        expectation.update(data.getRobotDataCollection(), data.getTime());

        //State transition
        if(action == AgentActions.move){
            if(position == target){
                if(isChargeRequired || targetter.getIsChargeRequired()){
                    // start charging
                    action = AgentActions.charge;
                }
            }
        }
        else if(action == AgentActions.charge){
            if(robotData.getBatteryLevel() == robotData.getRobotSpec().getCapacity()){
                //finish charging
                action = AgentActions.move;
                isChargeRequired = false;
            }
        }
        else if(action == AgentActions.standby){
            action = AgentActions.move;
            target = position;
        }
        else if(action == AgentActions.sleep){
            action = AgentActions.move;
        }

        //Flip coin when litter amount requirement is met
        if(action == AgentActions.move && flipCoin == true){
            double n = (new Random()).nextDouble();
            if(n < sleepProb){
                action = AgentActions.sleep;
                return;
            }
        }

        //Target Decision & Path Gneration
        TargetPathAgentStatus status = new TargetPathAgentStatus(action, target, data);
        targetter.update(status);

        if(target != targetter.getNextTarget() && !isChargeRequired){
            //new target
            target = targetter.getNextTarget();

            status = new TargetPathAgentStatus(action, target, data);
            pather.update(status);

            if(pather.getCanArrive()){
                return;
            }

            //Enforce to charge if cannot arrive
            target = baseNode;
            isChargeRequired = true;
            if(position == baseNode){
                action = AgentActions.charge;
                return;
            }
            status = new TargetPathAgentStatus(action, target, data);
        }

        if(position == baseNode && targetter.getIsChargeRequired()){
            action = AgentActions.charge;
            return;
        }

        pather.update(status);

        Coordinate p = graph.getCoordinate(position);
        int px = p.x + scale;
        int py = p.y + scale;

        positionCount[py][px]++;
    }

    // no initial value
    public void InitializeMySpawnPattern(){
        mySpawnPattern = new LitterSpawnPattern();
        for(int node : graph.getAllNode()){
            mySpawnPattern.addSpawnProbability(node, new LitterSpawnProbability(1, 0.0, 1));
        }
    }

    // with initial value
    public void InitializeMySpawnPattern(LitterSpawnPattern initPattern){
        mySpawnPattern = new LitterSpawnPattern();
        for(int node : graph.getAllNode()){
            mySpawnPattern.addSpawnProbability(node, new LitterSpawnProbability(1, initPattern.getSpawnProbability(node).getProbability(), 1));
        }
    }

    @Override
    public void setRobotID(int id){
        robotID = id;
    }

    @Override
    public void setBaseNode(int base){
        baseNode = base;
    }

    @Override
    public void setPathPlanner(IPathPlanner p){
        pather = p;
    }

    @Override
    public void setTargetDecider(ITargetDecider t){
        targetter = t;
    }

    @Override
    public void setAction(AgentActions a){
        action = a;
    }

    @Override
    public void setExpectation(){
        targetter.setExpectation(expectation);
        pather.setExpectation(expectation);
    }

    @Override
    public int getRobotID(){
        return robotID;
    }

    @Override
    public AgentActions getAction(){
        return action;
    }

    @Override
    public int getNextNode(){
        return pather.getNextNode();
    }

    @Override
    public int getTargetter(){
        return targetter.getTargetDecisionStrategy();
    }

    @Override
    public LitterSpawnPattern getMySpawnPattern(){
        return expectation.getMySpawnPattern();
    }

    @Override
    public void setStopTime(int time){
        stopTime = time;
    }

    @Override
    public void setRestartTime(int time){
        restartTime = time;
    }

    @Override
    public int getStopTime(){
        return stopTime;
    }

    @Override
    public int getRestartTime(){
        return restartTime;
    }

    @Override
    public LitterExistingExpectation getExpectation(){
        return expectation;
    }

    @Override
    public void setExpectation(LitterExistingExpectation exp){
        expectation = exp;
    }

    @Override
    public void setEnvironment(IEnvironment environment){
        //ダミーメソッド
    }

    @Override
    public void setSleepProbability(double p){
        //ダミーメソッド
    }

    @Override
    public void setPreStop(){
        //ダミーメソッド
    }

    @Override
    public boolean getPreStop(){
        //ダミーメソッド
        return false;
    }

    @Override
    public void addStartIndex(int n){
        //ダミーメソッド
    }

    @Override
    public void resetPreStop(){
        //ダミーメソッド
    }

    @Override
    public CommunicationDetails getCommunicationDetails(){
        //ダミーメソッド
        return new CommunicationDetails(new Coordinate(0, 0), 0.0, new LitterSpawnPattern(), new LinkedList<Integer>());
    }

    @Override
    public void searchNumberDecrease(int decrease){
        //ダミーメソッド
    }

    @Override
    public int getPositionCount(int x, int y){
        return -1;
    }

    @Override
    public void countPosition(int position){
        //ダミーメソッド
    }

    @Override
    public void setRequirement(double requirementLevel){
        //ダミーメソッド
    }

    @Override
    public void setEnvironmentEstimator(RequirementEstimator estimator){
        //ダミーメソッド
    }

    @Override
    public boolean requirementReached(){
        return false;
    }

    @Override
	public int getSearchNodeNumber(){
		return -1;
	}

	@Override
    public Coordinate getCenterNode(){
		return new Coordinate(0, 0);
	}

    @Override
    public double getCorrection(){
        return -1;
    }

    @Override
	public int getSuccessCount(){
		//ダミーメソッド
        return -1;
	}
    
	@Override
    public int getFailureCount(){
		//ダミーメソッド
        return -1;
	}

    @Override
    public void resetSuccessCount(){
        //ダミーメソッド
    }

    @Override
	public void setEnvironmentEstimatorU(RequirementEstimatorU estimator){
		//ダミーメソッド
	}

    @Override
    public void restart(ObservedData data){
        //ダミーメソッド
    }

    @Override
    public int getCycleIndex(){
        return 0;
    }
}
