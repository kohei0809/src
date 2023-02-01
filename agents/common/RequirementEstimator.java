package agents.common;

import java.util.List;

import core.IGraph;

// Estimate whether the total amount of litter reaches the requirement level.
//品質要求を満たしているかを確認

public class RequirementEstimator {

	private int totalSize;
	private double estimation, requirement;
	private boolean reached;
	private IGraph map;
	private double correction;
	private boolean change;
	private double learnRate;

	public RequirementEstimator(IGraph m, int chargingBase, double corr) {
		map = m;
		totalSize = map.getAccessibleNode().size();
		//base = chargingBase;
		reached = false;
		learnRate = 1.0;

		if(corr < 0){
			change = false;
			correction = 1.0;
		}
		else{
			correction = corr;
			change = true;
		}
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
