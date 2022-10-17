package core;

public interface ISimulation {
    /**
	 * execution
	 * @param steps number of step
	 */
	void run(int steps);

	/**
	 * ?????
	 */
	void step();

	/**
	 * Reset simulation.
	 */
	void reset();
}
