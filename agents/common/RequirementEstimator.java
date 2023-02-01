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

	//現在のイベント量の予測(p(v)未知)
	public void update(LitterExistingExpectation expectation, List<Integer> visitedNodes) {
		reached = false;
		List<Integer> nodes = visitedNodes;

		double sum = 0.0;
		for (int node : nodes) {
			double exp = expectation.getExpectation(node);
			sum += exp;
		}
		
		estimation = sum / nodes.size();
		estimation *= totalSize;
		if ((estimation / correction) * learnRate < requirement){
			reached = true;
		}

		//expLogger.writeLine(nodes.size() + "," + totalSize + "," + "H&P");
	}

	//未来のイベント量の予測(p(v)未知)
	public void update_future(LitterExistingExpectation expectation, int time, List<Integer> visitedNodes) {
		reached = false;
		
		List<Integer> nodes = visitedNodes;
		
		double sum = 0.0;
		for (int node : nodes) {
			double exp = expectation.getExpectation(node, time);
			sum += exp;
		}
		estimation = sum / nodes.size();
		estimation *= totalSize;
		//expLogger.writeLine(nodes.size() + "," + totalSize + "," + "future");
		
		if ((estimation / correction) * learnRate < requirement){
			reached = true;
		}
	}

	/**
	 * PDA given
	 * 
	 * @param expectation probability * time interval
	 */

	//現在のイベント量の予測(p(v)既知)
	public void update(LitterExistingExpectation expectation, int position) {
		reached = false;
		List<Integer> nodes = map.getAllNode();

		double sum = 0.0;
		for (int node : nodes) {
			double exp = expectation.getExpectation(node);
			sum += exp;
		}
		//expLogger.writeLine(estimation + "");
		
		estimation = sum;
		
		if (estimation < requirement * correction){
			reached = true;
		}
	}

	//未来のイベント量の予測(p(v)既知)
	public void update_future(LitterExistingExpectation expectation, int position, int time) {
		reached = false;
		List<Integer> nodes = map.getAllNode();

		double sum = 0.0;
		for (int node : nodes) {
			double exp = expectation.getExpectation(node, time);
			sum += exp;
		}
		//expLogger.writeLine(estimation + "");
		
		estimation = sum;
		
		if (estimation < requirement * correction){
			reached = true;
		}
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
