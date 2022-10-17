package agents.target;

import java.util.AbstractMap;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import agents.ITargetDecider;
import agents.TargetPathAgentStatus;
import agents.common.LitterExistingExpectation;
import core.IPointMappedGraph;
import core.RobotData;
import core.agent.AgentActions;

public class LongIntervalTargetDecider implements ITargetDecider{
    //訪問間隔優先法

    int robotID;
	List<Integer> nodes;
	IPointMappedGraph map;
	double rate;
	Random rand;
	LitterExistingExpectation expectation;
	boolean firstSearch = true;

	private int nextTarget;

	public LongIntervalTargetDecider(int id, IPointMappedGraph m, int seed, List<Integer> excludeNodes) {
		robotID = id;
		map = m;
		nodes = new LinkedList<Integer>(map.getAllNode());

		for(int node : excludeNodes) {
			nodes.remove(nodes.indexOf(node));
		}

		rate = 0.01;
		rand = new Random(seed);

		nextTarget = nodes.get(rand.nextInt(nodes.size()));
	}

	public void update(TargetPathAgentStatus status) {
		if (status.getAction() != AgentActions.move)
			return;

		RobotData mydata = status.getObservedData().getRobotDataCollection().getRobotData(robotID);
		if (mydata.getPosition() == status.getTarget()) {
			List<Map.Entry<Integer, Integer>> array = new LinkedList<Map.Entry<Integer, Integer>>();
			int i = 0;

			for(int node : nodes) {
				int interval = expectation.getInterval(node);

				if(node != mydata.getPosition()) {
					array.add(i, new AbstractMap.SimpleEntry<Integer, Integer>(node, interval));
				}else {
					array.add(i, new AbstractMap.SimpleEntry<Integer, Integer>(node, -1));
				}

				i++;
			}

			//降順
			array.sort(new Comparator<Map.Entry<Integer, Integer>>() {
				@Override
				public int compare(Map.Entry<Integer, Integer> o1, Map.Entry<Integer, Integer> o2) {
					return o2.getValue().compareTo(o1.getValue());
				}
			});

			//選択肢となる数
	        int count = 5;

	        if(array.size() < 5) {
				count = array.size();
			}

	        if(array.size() > 5) {
	        	//Integer→intへの変換
	        	//これをしないと、==で比較してはいけない（その場合は、equalsメソッドで比較）
	        	int in_count_last_value = array.get(count-1).getValue();
	        	int out_count_first_value = array.get(count).getValue();

	        	//countに含まれるノードと同じ期待値がある場合には，そのノードも選択肢に含める
	        	if(in_count_last_value == out_count_first_value) {
//	        		count = BinarySearchDecending.binarySearchDecendingInteger(array, count);
	        		//同じ値の基準値
	        		int sameValue = array.get(count).getValue();

	        		int same = count;
	        		int small = array.size() - 1;
	        		int check = (same + small) / 2;

	        		do {
	        			if(array.get(check).getValue().intValue() == sameValue) {
	        				same = check;
	        			}else if(array.get(check).getValue().intValue() < sameValue) {
	        				small = check;
	        			}

	        			check = (same + small) / 2;

	        		} while(!(check == same || check == small));

	        		count = same + 1;
	        	}
            }

            int index = count < 1 ? 0 : rand.nextInt(count);
            nextTarget = array.get(index).getKey();
		}
	}

	public void setExpectation(LitterExistingExpectation exp) {
		expectation = exp;
	}

	public void setNextTarget(int node) {
		nextTarget = node;
	}

	public void setRandomSelectRate(double rate) {
		this.rate = rate;
	}

	public int getNextTarget() {
		return nextTarget;
	}

	public boolean getIsChargeRequired() {
		return false;
	}

	@Override
	public int getTargetDecisionStrategy() {
		return 3;
	}

	@Override
	public String getString() {
		return "LongInterval";
	}

	@Override
	public void resetState() {

	}

	@Override
	public void setPreStop() {

	}

	@Override
	public void resetPreStop() {

	}
}
