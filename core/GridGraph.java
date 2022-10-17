package core;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class GridGraph implements IPointMappedGraph {

	// GridGraph Members
	DirectedGraph graph;
	Map<Integer, Coordinate> coords;
	Coordinate scale, maxCoordinate, minCoordinate;

	boolean isDirected;
	List<Integer> nodes;
	List<Integer> accessibleNodes;
	int count;

	private GridGraph() {
		graph = new DirectedGraph();
		coords = new HashMap<Integer, Coordinate>();
		isDirected = graph.isDirected;
		nodes = graph.nodes;
		count = graph.count;
	}

	public GridGraph(Coordinate max, Coordinate min) {

		this();
		setMaxCoordinate(max);
		setMinCoordinate(min);
		scale = new Coordinate(max.x - min.x, max.y - min.y);

		// Connect each node
		int n = 0;
		for (int i = min.x; i <= max.x; i++) {
			for (int j = min.y; j <= max.y; j++) {
				graph.addNode(n);
				coords.put(n, new Coordinate(i, j));
				n++;
			}
		}
		for (int i = min.x; i <= max.x; i++) {
			for (int j = min.y; j <= max.y; j++) {
				if (i != max.x)
					addEdge(getNode(i, j), getNode(i + 1, j));
				if (i != min.x)
					addEdge(getNode(i, j), getNode(i - 1, j));
				if (j != max.y)
					addEdge(getNode(i, j), getNode(i, j + 1));
				if (j != min.y)
					addEdge(getNode(i, j), getNode(i, j - 1));
			}
		}
	}

	public Coordinate getCoordinate(int node) {
		return coords.get(node);
	}

	public int getNode(Coordinate coord) {
		int i = coord.x - minCoordinate.x;
		int j = coord.y - minCoordinate.y;
		return (scale.x + 1) * i + j;
	}

	public int getNode(int x, int y) {
		return getNode(new Coordinate(x, y));
	}

	public void setMaxCoordinate(Coordinate max) {
		maxCoordinate = max;
	}

	public void setMinCoordinate(Coordinate min) {
		minCoordinate = min;
	}

	public Coordinate getMaxCoordinate() {
		return maxCoordinate;
	}

	public Coordinate getMinCoordinate() {
		return minCoordinate;
	}

	// PointMappedGraph Implementation
	public Coordinate getPointMappedGraph(int node) {
		return coords.get(node);
	}

	public int getPointMappedGraph(Coordinate point) {
		int i = point.x - minCoordinate.x;
		int j = point.y - minCoordinate.y;
		return (scale.x + 1) * i + j;
	}

	// IGraph Implementation
	public List<Integer> getAllNode() {
		return nodes;
	}

	public void addNode(int node) {
		throw new IllegalStateException("This object doesn't support this method.");
	}

	public boolean removeNode(int node) {
		throw new IllegalStateException("This object doesn't support this method.");
	}

	public boolean containsNode(int node) {
		return graph.containsNode(node);
	}

	public void addEdge(int start, int end) {
		Coordinate startc = getCoordinate(start);
		Coordinate endc = getCoordinate(end);
		int x = startc.x - endc.x;
		int y = startc.y - endc.y;

		if ((x * x + y * y) != 1)
			throw new IllegalArgumentException("This edge is invalid" + startc + " -> " + endc);

		graph.addEdge(start, end, 1);
	}

	public boolean removeEdge(int start, int end) {
		return graph.removeEdge(start, end);
	}

	public boolean containsEdge(int start, int end) {
		return graph.containsEdge(start, end);
	}

	public List<Integer> getNeighbors(int node) {
		return graph.getNeighbors(node);
	}

	public List<Integer> getParentNodes(int node) {
		return graph.getParentNodes(node);
	}

	public List<Integer> getChildNodes(int node) {
		return graph.getChildNodes(node);
	}

	public void addEdge(int start, int end, int weight) {
		throw new IllegalStateException("This object doesn't support this method.");
	}

	public int getWeight(int start, int end) {
		if (containsEdge(start, end))
			return 1;
		else
			throw new IllegalArgumentException("The edge is not in the graph.");
	}

	public void setWeight(int start, int end, int weight) {
		throw new IllegalStateException("This objevt doesn't support this method.");
	}

	public List<Integer> getAccessibleNode() {
		return accessibleNodes;
	}

	public void removeObstacleNodes(List<Integer> excludedNodes) {
		accessibleNodes = new LinkedList<Integer>(nodes);

		for (int node : excludedNodes)
			accessibleNodes.remove((Integer) node);
	}

	// Object override
	public String toString() {
		return graph.toString();
	}

}