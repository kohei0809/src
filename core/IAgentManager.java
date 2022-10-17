package core;

public interface IAgentManager {
    /**
	 * Update agents.
	 */
	void move();

	/**
	 * Let agents clean.
	 */
	void clean();

	/**
	 * Return number of agents performing energy-saving behaviors.
	 * e.g. homing, pausing
	 */
	//int getEcoAgentNumber();
}
