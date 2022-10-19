package agents.common;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import core.IGraph;
import core.util.DijkstraAlgorithm;
import core.util.LogManagerContext;
import core.util.LogWriter2;
import core.util.PotentialCollection;

// Estimate whether the total amount of litter reaches the requirement level.
public class RequirementEstimator {

	int refNodesNum, totalSize, minRange;
	int base, maxRoute;
	double estimation, requirement;
	boolean reached;
	IGraph map;
	double correction;
	boolean change;
	double learnRate;
	double beta = 0.1;
	
	LogWriter2 expLogger;

	public RequirementEstimator(IGraph m, int chargingBase, double corr) {
		refNodesNum = 100;
		map = m;
		totalSize = map.getAccessibleNode().size();
		minRange = 10;
		base = chargingBase;
		reached = false;
		learnRate = 1.0;

		maxRoute = new DijkstraAlgorithm(map).getMaxRouteLength(base);

		if(corr < 0){
			change = false;
			correction = 1.0;
		}
		else{
			correction = corr;
			change = true;
		}

		expLogger = LogManagerContext.getLogManager().createWriter2("expLogger");
	}

	/**
	 * Probability of Dirt Accumulation unknown (PDA Learning)
	 * 
	 * @param expectation probability * time interval
	 * @param visitedNode list of nodes agent has visited
	 */
	public void update(LitterExistingExpectation expectation, List<Integer> visitedNodes) {
		reached = false;
		List<Integer> nodes = visitedNodes;

		double sum = 0.0;
		int size = 0, sizeZero = 0;
		for (int node : nodes) {
			double exp = expectation.getExpectation(node);
			sum += exp;
			if(expectation.getProbability(node) != 0){
				size++;
			}
			else{
				sizeZero++;
			}
		}
		//estimation = sum / (size + (sizeZero/8));
		//estimation *= totalSize;
		double Size = (1-beta)*size + beta*(size+sizeZero);
		estimation = sum / Size;
		
		//estimation = sum / nodes.size();
		estimation *= totalSize;
		if ((estimation * learnRate / correction) < requirement){
			reached = true;
		}

		//expLogger.writeLine(nodes.size() + "," + totalSize + "," + "H&P");
	}

	public void update_future(LitterExistingExpectation expectation, int time, List<Integer> visitedNodes) {
		reached = false;
		
		List<Integer> nodes = visitedNodes;
		
		double sum = 0.0;
		int size = 0, sizeZero = 0;
		for (int node : nodes) {
			double exp = expectation.getExpectation(node, time);
			sum += exp;
			if(expectation.getProbability(node) != 0){
				size++;
			}
			else{
				sizeZero++;
			}
		}
		//estimation = sum / (size + (sizeZero/8));
		//estimation *= totalSize;
		//estimation = sum / (2*size);
		
		double Size = (1-beta)*size + beta*(size+sizeZero);
		estimation = sum / Size;
		//estimation = sum / nodes.size();
		estimation *= totalSize;
		//expLogger.writeLine(nodes.size() + "," + totalSize + "," + "future");
		
		//estimation = sum;
		
		if ((estimation * learnRate / correction) < requirement){
			reached = true
		}
	}

	/**
	 * PDA given
	 * 
	 * @param expectation probability * time interval
	 */
	public void update(LitterExistingExpectation expectation, int position) {
		reached = false;
		//List<Integer> nodes = pickRandomNodes();
		//List<Integer> nodes = pickNeighborNodes(position); // local observations
		List<Integer> nodes = map.getAllNode();

		double sum = 0.0;
		for (int node : nodes) {
			double exp = expectation.getExpectation(node);
			sum += exp;
		}
		//estimation = sum / nodes.size() * totalSize;
		//expLogger.writeLine(estimation + "");
		
		estimation = sum;
		
		if (estimation < requirement * correction){
			reached = true;
		}
	}

	public void update_future(LitterExistingExpectation expectation, int position, int time) {
		reached = false;
		//List<Integer> nodes = pickRandomNodes();
		//List<Integer> nodes = pickNeighborNodes(position); // local observations
		List<Integer> nodes = map.getAllNode();

		double sum = 0.0;
		for (int node : nodes) {
			double exp = expectation.getExpectation(node, time);
			sum += exp;
		}
		//estimation = sum / nodes.size() * totalSize;
		//expLogger.writeLine(estimation + "");
		
		estimation = sum;
		
		if (estimation < requirement * correction){
			reached = true;
		}
	}

	// When PDA unknown (P initial value = 0)
	private List<Integer> pickRandomNodes(List<Integer> visitedNode) {
		List<Integer> nodes = new LinkedList<Integer>();
		Random rand = new Random();
		// if (visitedNode.size() < range)
		// range = visitedNode.size();
		for (int i = 0; i < refNodesNum; i++) {
			nodes.add(rand.nextInt(visitedNode.size()));
		}

		return nodes;
	}

	// Larger reference area when being closer to charging base
	/*
	 * maxRoute: farthest node to charging bases
	 */
	private List<Integer> pickNeighborNodes(int position) {
		List<Integer> nodes = new LinkedList<Integer>();
		Random rand = new Random();
		PotentialCollection distanceMap = new DijkstraAlgorithm(map).execute(position); // route to current position

		int toBase = distanceMap.getPotential(base);
		int range = Integer.max(minRange, maxRoute - toBase);
		for (int i = 0; i < refNodesNum;) {
			int node = rand.nextInt(totalSize);
			if (distanceMap.getPotential(node) < range) {
				nodes.add(node);
				i++;
			}
		}

		return nodes;
	}

	// Pick from the whole environment.
	private List<Integer> pickRandomNodes() {
		List<Integer> nodes = new LinkedList<Integer>();
		Random rand = new Random();
		for (int i = 0; i < refNodesNum; i++) {
			nodes.add(rand.nextInt(totalSize));
		}

		return nodes;
	}

	public void setLearnRate(double value){
		learnRate = value;
	}

	public double getLearnRate(){
		return learnRate;
	}

	public void setRequirement(double req) {
		requirement = req;
	}

	public double getRequirement(){
		return requirement;
	}

	public boolean requirementReached() {
		return reached;
	}

	public double getEstimatedValue() {
		return estimation;
	}

	public double getCorrection(){
		return correction;
	}

	public void setCorrection(double corre){
		correction = corre;
	}

	public boolean getChange(){
		return change;
	}
}
