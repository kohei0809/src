package core.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import core.IGraph;

public class DijkstraAlgorithm {
	/**
	 * Calculate potential using the Dijkstra's Algorithm. Find the shortest paths
	 * between nodes in a graph.
	 */
	IGraph graph;
	Map<Integer, Integer> distance; // < node ID, length of path (to target) >
	Map<Integer, Integer> route; // < node ID, next node (with shortest path) >

	public DijkstraAlgorithm(IGraph graph) {
		this.graph = graph;
	}

	public PotentialCollection execute(int dest) {
		return execute(dest, dest);
	}

	/**
	 * @param dest: target
	 * @param src:  current position
	 */
	public PotentialCollection execute(int dest, int src) {
		if (!graph.containsNode(dest))
			throw new IllegalStateException("This node is not in the graph: " + dest);
		if (!graph.containsNode(src))
			throw new IllegalStateException("This node is not in the graph: " + src);

		distance = new HashMap<Integer, Integer>();
		for (int node : graph.getAllNode()) {
			distance.put(node, Integer.MAX_VALUE);
		}
		route = new HashMap<Integer, Integer>();
		route.put(dest, dest);

		List<Integer> calc = new LinkedList<Integer>(); // < node ID >
		calc.add(dest);
		distance.put(dest, 0);

		while (calc.size() != 0) {
			int node = calc.get(0);
			int potential = distance.get(node);
			if (node == src && node != dest)
				return getPotentialMap(); // goal!!

			for (int child : graph.getParentNodes(node)) {
				int cpotential = potential + graph.getWeight(node, child);
				if (distance.get(child) > cpotential) {
					if (distance.get(child) != Integer.MAX_VALUE)
						calc.remove(child);

					// 追加処理
					int index = calc.size() - 1;
					for (; distance.get(calc.get(index)) > cpotential; index--)
						;
					calc.add(index + 1, child);

					// 反映処理
					distance.put(child, cpotential);
					route.put(child, node);
				}
			}
			calc.remove(0);
		}

		return getPotentialMap();
	}

	public PotentialCollection getPotentialMap() {
		return new PotentialCollection(distance);
	}

	// Get the shortest path
	public ArrayList<Integer> getRoute(int src, int dest) {
		ArrayList<Integer> path = new ArrayList<Integer>();
		int next = src;

		while (next != dest) {
			next = route.get(next);
			path.add(next);
		}
		return path;
	}

	// Get the length of longest path from target
	public int getMaxRouteLength(int target) {
		PotentialCollection routeMap = execute(target);
		int max = 0;

		for (int node : graph.getAccessibleNode()) {
			int length = routeMap.getPotential(node);
			if (length > max)
				max = length;
		}
		return max;
	}
}
