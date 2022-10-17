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
public class RequirementEstimatorU {
	//U(s)のためのやつ

	int totalSize;
	int base;
	double estimation, requirement;
	boolean reached;
	IGraph map;
	double correction;
	boolean change;
	int max_node;
	double alpha = 2/3;
	PotentialCollection basePotential, targetPotential;
	List<Integer> maxNodeList;
	Random rand;

	//LogWriter2 expLogger;

	public RequirementEstimatorU(IGraph m, int chargingBase, double corr, int seed) {
		map = m;
		totalSize = map.getAccessibleNode().size();
		base = chargingBase;
		reached = false;
		max_node = 0;
		maxNodeList = new LinkedList<Integer>();
		rand = new Random(seed);

		if(corr < 0){
			change = false;
			correction = 1.0;
		}
		else{
			correction = corr;
			change = true;
		}

		//expLogger = LogManagerContext.getLogManager().createWriter2("expLogger");
	}

	/**
	 * Probability of Dirt Accumulation unknown (PDA Learning)
	 * 
	 * @param expectation probability * time interval
	 * @param visitedNode list of nodes agent has visited
	 */

	//変更前
	public void update(LitterExistingExpectation expectation, List<Integer> visitedNodes) {
		reached = false;
		List<Integer> nodes = visitedNodes;

		double sum = 0.0;
		for (int node : nodes) {
			double exp = expectation.getExpectation(node);
			sum += exp;
		}
		estimation = sum / nodes.size() * totalSize;
		if (estimation < requirement * correction){
			reached = true;
		}

		//expLogger.writeLine(nodes.size() + "," + totalSize + "," + "H&P");
	}

	//変更前
	public void update_future(LitterExistingExpectation expectation, int time, List<Integer> visitedNodes) {
		reached = false;
		
		List<Integer> nodes = visitedNodes;
		
		double sum = 0.0;
		for (int node : nodes) {
			double exp = expectation.getExpectation(node, time);
			sum += exp;
		}
		estimation = sum / nodes.size() * totalSize;
		//expLogger.writeLine(nodes.size() + "," + totalSize + "," + "future");
		
		estimation = sum;
		
		if (estimation < requirement * correction){
			reached = true;
		}
	}

	/**
	 * PDA given
	 * 
	 * @param expectation probability * time interval
	 */
	public void update(LitterExistingExpectation expectation, int position, int time) {
		reached = false;
		List<Integer> nodes = map.getAllNode();

		double max = 0.0;
		int count = 0;
		//maxNodeList.clear();
		for (int node : nodes) {
			double exp = expectation.getExpectation(node, time);
			//if(exp < requirement * correction * alpha){
			//	continue;
			//}
			//targetPotential = new DijkstraAlgorithm(map).execute(node, position);
			//exp = expectation.getExpectation(node, time + targetPotential.getPotential(position));
			if(max == exp){
				count++;
				//maxNodeList.add(node);
			}
			if(max < exp){
				max = exp;
				max_node = node;
				//maxNodeList.clear();
				//maxNodeList.add(node);
			}
		}
		//max_node = maxNodeList.get(rand.nextInt(maxNodeList.size()));
		//targetPotential = new DijkstraAlgorithm(map).execute(max_node, position);
		estimation = max;
		//expLogger.writeLine(estimation + "," + max_node + "," + time + ",now");
		//int dist = targetPotential.getPotential(max_node);
		//double exp = expectation.getExpectation(max_node, time, time+dist);
		//expLogger.writeLine(time + "," + estimation + "," + max_node + "," + position + "," + count + ",now");
		
		if (estimation < requirement * correction){
			reached = true;
		}
	}

	public void update_future(LitterExistingExpectation expectation, int position, int time, boolean flipCoin) {
		reached = false;
		List<Integer> nodes = map.getAllNode();

		double max = 0.0;
		if(flipCoin){
			maxNodeList.clear();
			for (int node : nodes) {
				double exp = expectation.getExpectation(node, time);
				//if(exp < requirement * correction * alpha){
				//	continue;
				//}
				//targetPotential = new DijkstraAlgorithm(map).execute(node, position);
				//exp = expectation.getExpectation(node, time+targetPotential.getPotential(position));
				if(max == exp){
					maxNodeList.add(node);
				}
				if(max < exp){
					max = exp;
					maxNodeList.clear();
					maxNodeList.add(node);
				}
			}
			max_node = maxNodeList.get(rand.nextInt(maxNodeList.size()));
		}
		else{
			max = expectation.getExpectation(max_node, time);
		}
		estimation = max;
		int dist = basePotential.getPotential(max_node);
		double exp = expectation.getExpectation(max_node, time+dist);
		//expLogger.writeLine(time + "," + estimation + "," + max_node + "," + dist + "," + exp + "," + position + "," + maxNodeList.size() + ",future");
		//expLogger.writeLine(time + "," + estimation + "," + max_node + "," + position + "," + maxNodeList.size() + ",future");
		estimation += exp;
		
		if (estimation < requirement*correction){
			reached = true;
		}
	}

	public double getMaxTargetExp(LitterExistingExpectation expectation, int time){
		List<Integer> nodes = map.getAllNode();
        double max = 0.0;
        for(int node : nodes){
            double exp = expectation.getExpectation(node);
            if(max < exp){
                max = exp;
                max_node = node;
            }
        }
        return max / correction;
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

	public int getMaxNode(){
		return max_node;
	}

	public void setPotentialMap(int base){
		basePotential = new DijkstraAlgorithm(map).execute(base);
	}
}
