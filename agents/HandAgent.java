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
		update(data, false);
	}

	@Override
	public void update(ObservedData data, boolean flipCoin) {
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
		//γγγΌγ‘γ½γγ
		return -1;
	}

	@Override
    public Coordinate getCenterNode(){
		//γγγΌγ‘γ½γγ
		return new Coordinate(0, 0);
	}

	@Override
	public LitterSpawnPattern getMySpawnPattern() {
		//γγγΌγ‘γ½γγ
		return new LitterSpawnPattern();
	}

	@Override
	public CommunicationDetails getCommunicationDetails() {
		//γγγΌγ‘γ½γγ
		return new CommunicationDetails(new Coordinate(0, 0), 0.0, new LitterSpawnPattern(), new LinkedList<Integer>());
	}

	@Override
	public void searchNumberDecrease(int decrease) {
		//γγγΌγ‘γ½γγ
	}

	@Override
	public int getTargetter() {
		//γγγΌγ‘γ½γγ
		return -1;
	}

	@Override
	public void setStopTime(int time) {
		//γγγΌγ‘γ½γγ
	}

	@Override
	public int getStopTime() {
		//γγγΌγ‘γ½γγ
		return -1;
	}

	@Override
	public void setRestartTime(int time) {
		//γγγΌγ‘γ½γγ
	}

	@Override
	public int getRestartTime() {
		//γγγΌγ‘γ½γγ
		return -1;
	}

	@Override
	public void addStartIndex(int n) {
		//γγγΌγ‘γ½γγ
	}

	@Override
	public void setPreStop() {
		//γγγΌγ‘γ½γγ
	}

	@Override
	public void resetPreStop() {
		//γγγΌγ‘γ½γγ
	}

	@Override
	public boolean getPreStop() {
		//γγγΌγ‘γ½γγ
		return false;
	}

	@Override
    public int getPositionCount(int x, int y){
		//γγγΌγ‘γ½γγ
        return -1;
    }

    @Override
    public void countPosition(int position){
		//γγγΌγ‘γ½γγ
    }

	@Override
    public void setRequirement(double requirementLevel){
		//γγγΌγ‘γ½γγ
    }	

    @Override
    public void setEnvironmentEstimator(RequirementEstimator estimator){
		//γγγΌγ‘γ½γγ
    }

	@Override
    public void setEnvironment(IEnvironment environment){
		//γγγΌγ‘γ½γγ
    }

    @Override
    public boolean requirementReached(){
        return false;
    }

	@Override
    public LitterExistingExpectation getExpectation(){
		//γγγΌγ‘γ½γγ
        return new LitterExistingExpectation(new LitterSpawnPattern(), true);
    }

	@Override
    public void setExpectation(LitterExistingExpectation exp){
		//γγγΌγ‘γ½γγ
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
        //γγγΌγ‘γ½γγ
    }

	@Override
	public void setEnvironmentEstimatorU(RequirementEstimatorU estimator){
		//γγγΌγ‘γ½γγ
	}

	@Override
    public void restart(ObservedData data){
        //γγγΌγ‘γ½γγ
    }

	@Override
    public int getCycleIndex(){
        return 0;
    }
}
