package core.agent;

import agents.IPathPlanner;
import agents.ITargetDecider;
import agents.common.LitterExistingExpectation;
import agents.common.RequirementEstimator;
import agents.common.RequirementEstimatorU;
import core.CommunicationDetails;
import core.Coordinate;
import core.IEnvironment;
import core.LitterSpawnPattern;

public interface IAgent {
    public int[][] positionCount = new int[50*2+1][50*2+1];

    public void setRobotID(int id);

    public void setBaseNode(int basePosition);

    public void setPathPlanner(IPathPlanner pather);

    public void setTargetDecider(ITargetDecider targetter);

    public void setSleepProbability(double prob);

    public void setAction(AgentActions action);

    public void setStopTime(int time);

    public void setRestartTime(int time);

    public void setPreStop();

    public void setExpectation();

    public void setRequirement(double requirementLevel);

    public void setEnvironmentEstimator(RequirementEstimator estimator);

    public int getRobotID();

    public int getStopTime();

    public int getRestartTime();

    public boolean getPreStop();

    public AgentActions getAction();

    public int getNextNode();

    public int getTargetter();

    public void addStartIndex(int n);

    public void resetPreStop();

    public LitterSpawnPattern getMySpawnPattern();

    public CommunicationDetails getCommunicationDetails();

    public void searchNumberDecrease(int decrease);

    //エージェントの状態をアップデート
    public void update(ObservedData data);

    public int getPositionCount(int x, int y);

    public void countPosition(int position);

    public LitterExistingExpectation getExpectation();

    public void setExpectation(LitterExistingExpectation exp);

    public void setEnvironment(IEnvironment environment);

    public int getSearchNodeNumber();

    public Coordinate getCenterNode();

    public double getCorrection();

    public int getSuccessCount();

    public double getRequirement();
    
    public int getFailureCount();

    public void resetSuccessCount();

    public void setEnvironmentEstimatorU(RequirementEstimatorU estimator);

    public void restart(ObservedData data);

    public int getCycleIndex();
}