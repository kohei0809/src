package agents;

import java.util.LinkedList;

import agents.common.LitterExistingExpectation;
import agents.common.RequirementEstimator;
import agents.common.RequirementEstimatorU;
import core.CommunicationDetails;
import core.Coordinate;
import core.IEnvironment;
import core.LitterSpawnPattern;
import core.RobotData;
import core.agent.AgentActions;
import core.agent.IAgent;
import core.agent.ObservedData;

public class TargetPathAgent implements IAgent{
    //Agentが経路追従等をするプログラム(ごみ発生確率は知っている)米田さん手法

    ITargetDecider targetter;
    IPathPlanner pather;
    int baseNode, target, nextNode;
    double sleepProb = 0.0;
    boolean isChargeRequired = false;
    LitterExistingExpectation expectation;

    private int robotID;
    private AgentActions action;

    public TargetPathAgent(int id, LitterSpawnPattern pattern){
        setRobotID(id);
        setAction(AgentActions.standby);
        expectation = new LitterExistingExpectation(pattern, true);
    }

    @Override
    public void update(ObservedData data){
        RobotData robotData = data.getRobotDataCollection().getRobotData(robotID);
        int position = robotData.getPosition();

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
    public LitterExistingExpectation getExpectation(){
        return expectation;
    }

    @Override
    public void setExpectation(LitterExistingExpectation exp){
        expectation = exp;
    }

    @Override
    public void setEnvironment(IEnvironment environment){

    }

    @Override
    public int getPositionCount(int x, int y){
        return -1;
    }

    @Override
    public void countPosition(int position){

    }

    @Override
    public void setSleepProbability(double p){
        //ダミーメソッド
    }

    @Override
    public void setStopTime(int time){
        //ダミーメソッド
    }

    @Override
    public void setRestartTime(int time){
        //ダミーメソッド
    }

    @Override
    public void setPreStop(){
        //ダミーメソッド
    }

    @Override
    public int getStopTime(){
        //ダミーメソッド
        return -1;
    }

    @Override
    public int getRestartTime(){
        //ダミーメソッド
        return -1;
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
    public void setRequirement(double requirementLevel){

    }

    @Override
    public void setEnvironmentEstimator(RequirementEstimator estimator){

    }

    @Override
	public int getSearchNodeNumber(){
		//ダミーメソッド
		return -1;
	}

	@Override
    public Coordinate getCenterNode(){
		//ダミーメソッド
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

	@Override
	public double getRequirement() {
		return -1;
	}
}
