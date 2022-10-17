package core;

public interface IPointMappedGraph extends IGraph{
    /**
	 * Get the point related to the given node.
	 */
	Coordinate getPointMappedGraph(int node);

	/**
	 * Get the node related to the given point.
	 */
	int getPointMappedGraph(Coordinate point);
}
