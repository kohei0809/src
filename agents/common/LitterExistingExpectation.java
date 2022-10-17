package agents.common;

import java.util.HashMap;
import java.util.Map;

import core.LitterSpawnPattern;
import core.RobotData;
import core.RobotDataCollection;

public class LitterExistingExpectation {

	int time;
	LitterSpawnPattern pattern;
	boolean isAccumulate, incrementEnabled;
	Map<Integer, Integer> visitedTime; // < node, time>

	public LitterExistingExpectation(LitterSpawnPattern p, boolean isA) {
		time = 0;
		pattern = p;
		visitedTime = new HashMap<Integer, Integer>();
		for (int node : pattern.getAllKeys()){
			visitedTime.put(node, 0);
        }
		isAccumulate = isA;
		incrementEnabled = false;
	}

	// Available to know other's position.
	public void update(RobotDataCollection data, int time) {
		update(time);
		for (RobotData robot : data.getAllRobotData()){
			visitedTime.put(robot.getPosition(), time);
        }
	}

	// Not available to know other's position.
	public void update(int position, int time) {
		update(time);
		visitedTime.put(position, time);
	}

	// Update time
	public void update(int time) {
		if (this.time > time){
			throw new IllegalArgumentException("The set time is invalid.");
        }
        this.time = time;
	}

	public double getExpectation(int node) {
		return getExpectation(node, time);
	}

	public double getExpectation(int node, int time) {
		double expectation = 0.0;
		int interval = time - visitedTime.get(node);
		double prob = pattern.getAllPatterns().get(node).getProbability();
		if (interval > 0) {
			if (isAccumulate){
				expectation = prob * interval;
            }
			else{
				expectation = 1.0 - Math.pow(1.0 - prob, interval);
            }
        }
		return incrementEnabled ? expectation * pattern.getAllPatterns().get(node).getIncrement() : expectation;
	}

	//interval between stime and etime
	public double getExpectation(int node, int stime, int etime) {
		double expectation = 0.0;
		int interval = etime - stime;
		double prob = pattern.getAllPatterns().get(node).getProbability();
		if (interval > 0) {
			if (isAccumulate){
				expectation = prob * interval;
            }
			else{
				expectation = 1.0 - Math.pow(1.0 - prob, interval);
            }
        }
		return incrementEnabled ? expectation * pattern.getAllPatterns().get(node).getIncrement() : expectation;
	}

	public double getExpetationValue(int node, int interval){
		double prob = pattern.getAllPatterns().get(node).getProbability();
		double exp = prob * interval;
		return exp;
	}

	public int getInterval(int node, int time) {
		int interval = time - visitedTime.get(node);
		return interval;
	}

	public int getInterval(int node) {
		return getInterval(node, this.time);
	}

	public void setMySpawnPattern(LitterSpawnPattern myP) {
		pattern = myP;
	}

	public LitterSpawnPattern getMySpawnPattern() {
		return pattern;
	}

	public Map<Integer, Integer> getVisitedTime(){
		return visitedTime;
	}

	public void setVisitedTime(Map<Integer, Integer> map){
		visitedTime = map;
	}
	
	public double getProbability(int node){
		return pattern.getAllPatterns().get(node).getProbability();
	}
}
