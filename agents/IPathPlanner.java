package agents;

import agents.common.LitterExistingExpectation;

public interface IPathPlanner {
	int getNextNode();

	boolean getCanArrive();

	/**
	 * Update the status of target-path planner
	 * @param status status data of agent
	 */
	void update(TargetPathAgentStatus status);

	void setExpectation(LitterExistingExpectation expectation);

	int getTarget();

	void updateHoming(TargetPathAgentStatus status); 
}
