package core;

import java.util.List;

public interface IGraph {
    // // Whether the graph is a directed graph.
	// boolean isDirected = false;
	// // Array of nodes
	// List<Integer> nodes = new LinkedList<Integer>();
	// // Total number of nodes.
	// int count = 0;

	/**
	 * Get a list of nodes.
	 */
	List<Integer> getAllNode();

	/**
	 * Add node to the graph.
	 */
	void addNode(int node);

	/*
	 * Remove node from the graph.
	 */
	boolean removeNode(int node);

	/**
	 * Check if the node exists.
	 */
	boolean containsNode(int node);

	/**
	 * Add edge to the graph.
	 * @param start starting node
	 * @param end ending node
	 */
	void addEdge(int start, int end);

	/**
	 * Remove edge from the graph.
	 * @param start starting node
	 * @param end ending node
	 */
	boolean removeEdge(int start, int end);

	/**
	 * Check if there is edge between the assigned nodes.
	 * @param start starting node
	 * @param end ending node
	 */
	boolean containsEdge(int start, int end);

	/**
	 * Get neighbor nodes.
	 */
	List<Integer> getNeighbors(int node);

	/**
	 * Get parent nodes.
	 */
	List<Integer> getParentNodes(int node);

	/**
	 * Get child nodes.
	 */
	List<Integer> getChildNodes(int node);

	/**
	 * Add edge to the graph.
	 */
	void addEdge(int start, int end, int weight);

	/**
	 * Get the weight of the node between the assigned nodes.
	 */
	int getWeight(int start, int end);

	/**
	 * Set weight.
	 */
	void setWeight(int start, int end, int weight);

	/**
	 * Get a list of accessible nodes. (excluding obstacles/walls)
	 */
	List<Integer> getAccessibleNode();

	/*
	 * Remove excluded nodes such as obstacles and walls, not from the graph, but
	 * save to another list.
	 */
	void removeObstacleNodes(List<Integer> excludedNodes);
}
