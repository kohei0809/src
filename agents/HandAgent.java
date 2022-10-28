package agents;

import java.util.LinkedList;
import java.util.List;

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

public class HandAgent implements IAgent{

    private int robotID;
    private List<Integer> behaviorList;
    private AgentActions action;

    int target;

    public HandAgent(int id, List<Integer> bList){
        robotID = id;
        behaviorList = bList;
		action = AgentActions.standby;
    }

	@Override
	public AgentActions getAction() {
		return action;
	}

    @Override
	public void setBaseNode(int base) {
	
    }

	@Override
	public void setTargetDecider(ITargetDecider t) {
	}

	@Override
	public void setPathPlanner(IPathPlanner p) {
	}

	@Override
	public void setSleepProbability(double p) {
	}

	@Override
	public void setExpectation() {
	}

	@Override
	public void update(ObservedData data) {
		RobotData robotdata = data.getRobotDataCollection().getRobotData(robotID);
		int position = robotdata.getPosition();

		// State transition
		if (action == AgentActions.standby) {
			action = AgentActions.move;
			target = position;
		}
	}

	@Override
	public void setRobotID(int id) {
		robotID = id;
	}

	@Override
	public int getNextNode() {
		int node = behaviorList.get(0);
		//int node = behaviorList.get(behaviorList.size() - 1);
        behaviorList.remove(0);
        return node;
	}

	@Override
	public int getRobotID() {
		return robotID;
	}

	@Override
	public void setAction(AgentActions action) {
		this.action = action;
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
	public LitterSpawnPattern getMySpawnPattern() {
		//ダミーメソッド
		return new LitterSpawnPattern();
	}

	@Override
	public CommunicationDetails getCommunicationDetails() {
		//ダミーメソッド
		return new CommunicationDetails(new Coordinate(0, 0), 0.0, new LitterSpawnPattern(), new LinkedList<Integer>());
	}

	@Override
	public void searchNumberDecrease(int decrease) {
		//ダミーメソッド
	}

	@Override
	public int getTargetter() {
		//ダミーメソッド
		return -1;
	}

	@Override
	public void setStopTime(int time) {
		//ダミーメソッド
	}

	@Override
	public int getStopTime() {
		//ダミーメソッド
		return -1;
	}

	@Override
	public void setRestartTime(int time) {
		//ダミーメソッド
	}

	@Override
	public int getRestartTime() {
		//ダミーメソッド
		return -1;
	}

	@Override
	public void addStartIndex(int n) {
		//ダミーメソッド
	}

	@Override
	public void setPreStop() {
		//ダミーメソッド
	}

	@Override
	public void resetPreStop() {
		//ダミーメソッド
	}

	@Override
	public boolean getPreStop() {
		//ダミーメソッド
		return false;
	}

	@Override
    public int getPositionCount(int x, int y){
		//ダミーメソッド
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
    public void setEnvironment(IEnvironment environment){
		//ダミーメソッド
    }

	@Override
    public LitterExistingExpectation getExpectation(){
		//ダミーメソッド
        return new LitterExistingExpectation(new LitterSpawnPattern(), true);
    }

	@Override
    public void setExpectation(LitterExistingExpectation exp){
		//ダミーメソッド
    }

	@Override
    public double getCorrection(){
        return -1;
    }

	@Override
	public int getSuccessCount(){
		return -1;
	}
    
	@Override
    public int getFailureCount(){
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
