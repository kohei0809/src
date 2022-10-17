package agents;

import agents.common.LitterExistingExpectation;

public interface ITargetDecider {
    int getNextTarget();

	boolean getIsChargeRequired();

	int getTargetDecisionStrategy();

	String getString();

	/**
	 * @param status status data of agent
	 */
	void update(TargetPathAgentStatus status);

	void setExpectation(LitterExistingExpectation expectation);

	//追加
	void resetState();

	//追加
	void setPreStop();

	//追加
	void resetPreStop();
}
