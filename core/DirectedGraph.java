package core;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class DirectedGraph implements IGraph {

	boolean isDirected;
	int count;

	List<Integer> nodes;
	List<Integer> accessibleNodes;
	Map<Integer, LinkedList<Integer>> parents;
	Map<Integer, LinkedList<Integer>> children;
	Map<Tuple<Integer, Integer>, Integer> weights;

	public DirectedGraph() {
		isDirected = true;
		nodes = new LinkedList<Integer>();
		parents = new HashMap<Integer, LinkedList<Integer>>();
		children = new HashMap<Integer, LinkedList<Integer>>();
		weights = new HashMap<Tuple<Integer, Integer>, Integer>();
		count = nodes.size();
	}

	public List<Integer> getAllNode() {
		return nodes;
	}

	public List<Integer> getAccessibleNode() {
		return accessibleNodes;
	}

	public void addNode(int node) {
		if (nodes.contains(node))
			throw new IllegalArgumentException("The node has already been in the graph.");
		else {
			nodes.add(node);
			parents.put(node, new LinkedList<Integer>());
			children.put(node, new LinkedList<Integer>());
		}
	}

	public boolean removeNode(int node) {
		if (nodes.contains(node)) {
			for (int parent : parents.get(node)) {
				Tuple<Integer, Integer> set = new Tuple<Integer, Integer>(parent, node);
				weights.remove(set);
				children.get(parent).remove(node);
			}
			for (int child : children.get(node)) {
				Tuple<Integer, Integer> set = new Tuple<Integer, Integer>(node, child);
				weights.remove(set);
				parents.get(child).remove(node);
			}

			nodes.remove(node);
			parents.remove(node);
			children.remove(node);

			return true;
		}

		else
			return false;
	}

	public void removeObstacleNodes(List<Integer> excludedNodes) {
		accessibleNodes = new LinkedList<Integer>(nodes);

		for (int node : excludedNodes)
			accessibleNodes.remove((Integer) node);
	}

	public boolean containsNode(int node) {
		if (nodes.contains(node))
			return true;
		else
			return false;
	}

	public void addEdge(int start, int end) {
		if (containsEdge(start, end) == true)
			throw new IllegalArgumentException("The edge has already been in the graph.");
		else {
			children.get(start).add(end);
			parents.get(end).add(start);
			setWeight(start, end, 0);
		}
	}

	public boolean removeEdge(int start, int end) {
		Tuple<Integer, Integer> set = new Tuple<Integer, Integer>(start, end);
		if (containsEdge(start, end) == false)
			throw new IllegalArgumentException("The edge is not in the graph.");
		else {
			children.get(start).remove((Integer) end);
			parents.get(end).remove((Integer) start);
			weights.remove(set);
			return true;
		}
	}

	public boolean containsEdge(int start, int end) {
		if (children.get(start).contains(end))
			return true;
		else
			return false;
	}

	public List<Integer> getNeighbors(int node) {
		if (containsNode(node) == false)
			throw new IllegalArgumentException("The node is not in the graph.");
		else {
			List<Integer> neighbors = new LinkedList<Integer>();
			for (int element : parents.get(node)) {
				neighbors.add(element);
			}
			for (int element : children.get(node)) {
				if (!neighbors.contains(element))
					neighbors.add(element);
			}
			return neighbors;
		}
	}

	public List<Integer> getParentNodes(int node) {
		if (containsNode(node) == false)
			throw new IllegalArgumentException("The node is not in the graph.");
		else
			return parents.get(node);
	}

	public List<Integer> getChildNodes(int node) {
		if (containsNode(node) == false)
			throw new IllegalArgumentException("The node is not in the graph.");
		else
			return children.get(node);
	}

	public void addEdge(int start, int end, int weight) {
		if (containsEdge(start, end) == false) {
			children.get(start).add(end);
			parents.get(end).add(start);
			setWeight(start, end, weight);
		} else
			throw new IllegalArgumentException("The edge has already been in the graph.");
	}

	public int getWeight(int start, int end) {
		if (containsEdge(start, end) == false)
			throw new IllegalArgumentException("The weight is not in the graph.");
		else
			return weights.get(new Tuple<Integer, Integer>(start, end));
	}

	public void setWeight(int start, int end, int weight) {
		if (containsEdge(start, end) == false)
			throw new IllegalArgumentException("The weight is not in the graph.");
		else {
			Tuple<Integer, Integer> set = new Tuple<Integer, Integer>(start, end);
			weights.put(set, weight);
		}
	}
}
