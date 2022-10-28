package agents;

import java.util.AbstractMap;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
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

// Learning of Dirt Accumulation Probability with Communication
public class TargetPathAgent_Communication implements IAgent{
    ITargetDecider targetter;
    IPathPlanner pather;
    int baseNode, target, nextNode;
    double sleepProb = 0.0;
    boolean isChargeRequired = false;
    LitterExistingExpectation expectation;
    Random rand;
    LitterSpawnPattern mySpawnPattern; //agent's map
    LitterSpawnPattern realSpawnPattern; //real map
    GridGraph graph;
    List<Integer> excludeNodes;
    List<Integer> nodes;
    List<Integer> searchNodes;
    int searchNodeNumber;
    int maxNodeNumber;
    Coordinate myCenterNode = new Coordinate(0, 0);
    double myCenterNodeWeight = 0.0;
    CommunicationDetails communicationDetail;
    int scale = 50;

    public int[][] positionCount = new int[2*scale+1][2*scale+1];

    private int robotID;
    private AgentActions action;
    private int stopTime = -1;
    private int restartTime = -1;

    public TargetPathAgent_Communication(int id, LitterSpawnPattern pattern, GridGraph graph, List<Integer> excludeNodes){
        setRobotID(id);
        action = AgentActions.standby;
        this.excludeNodes = excludeNodes;
        this.graph = graph;
        nodes = new LinkedList<Integer>(this.graph.getAllNode());
        
        for(int node : excludeNodes){
            nodes.remove(nodes.indexOf(node));
        }
        searchNodes = new LinkedList<Integer>(nodes);
        
        searchNodeNumber = nodes.size();
        maxNodeNumber = nodes.size();
        realSpawnPattern = pattern;
        InitializeMySpawnPattern();
        expectation = new LitterExistingExpectation(mySpawnPattern, true);
        communicationDetail = new CommunicationDetails(myCenterNode, myCenterNodeWeight, mySpawnPattern, searchNodes);
    }

    @Override
    public void setBaseNode(int base){
        baseNode = base;
    }

    @Override
    public void setTargetDecider(ITargetDecider t){
        targetter = t;
    }

    @Override
    public void setPathPlanner(IPathPlanner p){
        pather = p;
    }

    @Override
    public void setSleepProbability(double p){
        sleepProb = p;
    }

    @Override
    public void setExpectation(){
        targetter.setExpectation(expectation);
        pather.setExpectation(expectation);
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

    public LitterSpawnPattern getMySpawnPattern(){
        return mySpawnPattern;
    }

    @Override
    public void update(ObservedData data){
        RobotData robotData = data.getRobotDataCollection().getRobotData(robotID);
        int position = robotData.getPosition();
        int interval = expectation.getInterval(position, data.getTime());
        int vacuumedLitter = robotData.getVacuumedLitter();

        // Update agent's probability map
        mySpawnPattern.setProbability(position, vacuumedLitter, interval);

        // Available to know other's position
        expectation.update(data.getRobotDataCollection(), data.getTime());

        // State trasition
        if(action == AgentActions.move){
            if(position == target){
                if(isChargeRequired || targetter.getIsChargeRequired()){
                    //start charging
                    action = AgentActions.charge;
                    calculateSearchNode();
                    calculateCenterNode();
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

        // Target Decision & Path generation
        TargetPathAgentStatus status = new TargetPathAgentStatus(action, target, data);
        targetter.update(status);

        if(target != targetter.getNextTarget() && !isChargeRequired){
            // new target
            target = targetter.getNextTarget();
            status = new TargetPathAgentStatus(action, target, data);
            pather.update(status);

            if(pather.getCanArrive()){
                return;
            }

            // Enforce to charge if cannot arrive
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
    }

    private void calculateCenterNode(){
        double sumPDA = 0.0;
        for(int node : searchNodes){
            sumPDA += mySpawnPattern.getAllPatterns().get(node).getProbability();
        }
        communicationDetail.myExperienceWeight = sumPDA;
        double sumX = 0.0;
        double sumY = 0.0;

        for(int node : searchNodes){
            double n = sumPDA == 0 ? 0 : mySpawnPattern.getAllPatterns().get(node).getProbability() / sumPDA;
            sumX += n * graph.getCoordinate(node).x;
            sumY += n * graph.getCoordinate(node).y;
        }

        myCenterNode.x = (int) Math.round(sumX);
        myCenterNode.y = (int) Math.round(sumY);
        communicationDetail.myCenterNode = myCenterNode;
    }

    private void calculateSearchNode(){
        searchNodes.clear();
        List<Map.Entry<Integer, Double>> array = new LinkedList<Map.Entry<Integer, Double>>();
        int i = 0;
        for(int node : nodes){
            double imp = mySpawnPattern.getAllPatterns().get(node).getProbability();
            array.add(i, new AbstractMap.SimpleEntry<Integer, Double>(node, imp));
            i++;
        }

        //降順
        //同じ値の場合はランダムに並べる
        Collections.shuffle(array);

        array.sort(new Comparator<Map.Entry<Integer, Double>>() {
            @Override
            public int compare(Map.Entry<Integer, Double> o1, Map.Entry<Integer, Double> o2){
                return o2.getValue().compareTo(o1.getValue());
            }
        });

        int count = 0;
        for(Map.Entry<Integer, Double> node : array){
            searchNodes.add(node.getKey());
            count++;
            if(count == searchNodeNumber){
                break;
            }
        }
    }

    public void searchNumberDecrease(int decrease){
        searchNodeNumber -= decrease;
        if(searchNodeNumber > maxNodeNumber){
            searchNodeNumber = maxNodeNumber;
        }
    }

    @Override
    public void setRobotID(int id){
        robotID = id;
    }

    @Override
    public int getNextNode(){
        return pather.getNextNode();
    }

    @Override
    public int getRobotID(){
        return robotID;
    }

    @Override
    public void setAction(AgentActions action){
        this.action = action;
    }

    @Override
    public CommunicationDetails getCommunicationDetails(){
        return communicationDetail;
    }

    @Override
    public AgentActions getAction(){
        return action;
    }

    @Override
    public int getTargetter(){
        return targetter.getTargetDecisionStrategy();
    }

    @Override
    public void setStopTime(int time){
        stopTime = time;
    }

    @Override
    public int getStopTime(){
        return stopTime;
    }

    @Override
    public void setRestartTime(int time){
        restartTime = time;
    }

    @Override
    public int getRestartTime(){
        return restartTime;
    }

    @Override
    public void setEnvironment(IEnvironment environment){

    }

    @Override
    public void addStartIndex(int n){

    }

    @Override
    public void setPreStop(){

    }

    @Override
    public void resetPreStop(){

    }

    @Override
    public boolean getPreStop(){
        return false;
    }

    @Override
    public int getPositionCount(int x, int y){
        return -1;
    }

    @Override
    public void countPosition(int position){

    }

    @Override
    public void setRequirement(double requirementLevel){

    }

    @Override
    public void setEnvironmentEstimator(RequirementEstimator estimator){

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
	public int getSearchNodeNumber(){
		return searchNodeNumber;
	}

	@Override
    public Coordinate getCenterNode(){
		return myCenterNode;
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
