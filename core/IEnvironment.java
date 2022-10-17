package core;

public interface IEnvironment {
    /**
	 * Create robot in virtual environment.
	 * @param spec robot specification
	 * @param position initial position
	 */
	int createRobot(RobotSpec spec, int position);

	/**
	 * Create robot charging base in virtual environment.
	 * @param chargeValue charge value in charging base
	 * @param position position of charging base
	 */
	int setRobotBase(int chargeValue, int position);

	/**
	 * Move robot.
	 * @param id ID of the robot to move
	 * @param node destination node
	 */
	void moveRobot(int id, int node);

	/**
	 * Connect to its charging base.
	 */
	void connectRobotBase(int id);

	/**
	 * Disconnect to its charging base.
	 */
	void disconnectRobotBase(int id);

	/**
	 * Let the assigned robot do cleaning task.
	 */
	void clean(int id);

	/**
	 * Update the environment
	 */
	void update();

	/**
	 * Get environment time
	 */
	int getTime();

	/**
	 * Get environment spatial structure
	 */
	IGraph getSpatialStructure();

	/**
	 * Get robot data
	 */
	RobotData getRobotData(int id);

	/**
	 * Get data of all robot
	 */
	RobotDataCollection getRobotDataCollection();

	/**
	 * Get data of litter
	 */
	LitterDataCollection getLitterDataCollection();

	/**
	 * Get amount of litter
	 */
	int getLitterAmount();

	/**
	 * Get max amount of litter
	 */
	int getMaxLitterAmount();

	/**
	 * Get the spawn pattern of litter
	 */
	LitterSpawnPattern getLitterSpawnPattern();

	/**
	 * Get value of consumed energy
	 */
	int getTotalEnergyCost();

	/**
	 * Get amount of litter in position
	 */

	int getLitterAmount(int position);

	/**
	 * Get position of max amount of litter
	 */

	int getMaxTarget();

	/**
	 * Get size of max node list
	 */

	int getNodeListSize();
}
