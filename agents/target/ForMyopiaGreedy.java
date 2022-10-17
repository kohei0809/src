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
import core.IGraph;
import core.LitterSpawnPattern;
import core.RobotData;
import core.agent.AgentActions;
import core.agent.ObservedData;

public class ForMyopiaGreedy implements ITargetDecider{
    //MyopiaSweepTargetDeciderで使う
    //ログ出力のためにMyopiaSweepTargetDecider用に、GreedyTargetDeciderと同じものを用意
	//ログ出力部分のみ変更(消した)

    Random rand;
    List<Integer> nodes;
    int robotID;
    boolean isAccumulate;
    double rate, expectationSum;
    LitterExistingExpectation expectation;
    boolean firstSearch = true;

    private int nextTarget;

    boolean preStop = false;

    public ForMyopiaGreedy(int id, IGraph map, LitterSpawnPattern pattern, boolean isA, int seed, List<Integer> excludeNodes){
        robotID = id;
        nodes = new LinkedList<Integer>(map.getAllNode());

        for(int node : excludeNodes){
            nodes.remove(nodes.indexOf(node));
        }

        isAccumulate = isA;
        rate = 0.05;
        rand = new Random(seed);
        
        //ごみの発生確率が既知の場合
        //expectation = new LitterExistingExpectation(pattern, isAccumulate);
        
        nextTarget = nodes.get(rand.nextInt(nodes.size()));
    }

    public void setAccessibleNodes(List<Integer> nodes){
        this.nodes = nodes;
    }

    public void update(TargetPathAgentStatus status){
        if(status.getAction() != AgentActions.move){
            return;
        }

        RobotData mydata = status.getObservedData().getRobotDataCollection().getRobotData(robotID);

        //Find new target
        if(mydata.getPosition() == status.getTarget()){
            List<Map.Entry<Integer, Double>> array = new LinkedList<Map.Entry<Integer, Double>>();
            //<node, expected value of dirt>
            int i = 0;
            double sum = 0.0;
            for(int node : nodes){
                double exp = expectation.getExpectation(node);
                int interval = expectation.getInterval(node);

                if(node != mydata.getPosition()){
                    array.add(i, new AbstractMap.SimpleEntry<Integer, Double>(node, exp * interval * discountExpectation(status.getObservedData(), node)));
                    sum += exp;
                }
                else{
                    array.add(i, new AbstractMap.SimpleEntry<Integer, Double>(node, -1.0));
                }
                i++;
            }
            expectationSum = sum;

            //降順
            array.sort(new Comparator<Map.Entry<Integer, Double>>() {
                @Override
                public int compare(Map.Entry<Integer, Double> o1, Map.Entry<Integer, Double> o2){
                    return o2.getValue().compareTo(o1.getValue());
                }
            });

            //選択肢となる数
            int count = 5;

            if(array.size() < 5){
                count = array.size();
            }

            if(array.size() > 5){
                //Double→doubleへの変換
	        	//これをしないと、==で比較してはいけない（その場合は、equalsメソッドで比較）
	        	double in_count_last_value = array.get(count-1).getValue();
	        	double out_count_first_value = array.get(count).getValue();

                //countに含まれるノードと同じ期待値がある場合には，そのノードも選択肢に含める
                if(in_count_last_value == out_count_first_value){
                    //同じ値の基準値
                    double sameValue = array.get(count).getValue();

                    int same = count;
                    int out = array.size() - 1;
                    int check = (same + out) / 2;

                    do{
                        if(array.get(check).getValue().doubleValue() == sameValue){
                            same = check;
                        }
                        else if(array.get(check).getValue().doubleValue() < sameValue){
                            out = check;
                        }

                        check = (same + out) / 2;
                    }while(!(check == same || check == out));

                    count = same + 1;
                }
            }

            int index = count < 1 ? 0 : rand.nextInt(count);
            nextTarget = array.get(index).getKey();
        }
    }

    public double discountExpectation(ObservedData data, int node){
        return 1.0;
    }

    public void setExpectation(LitterExistingExpectation exp){
        expectation = exp;
    }

    public void setNextTarget(int node){
        nextTarget = node;
    }

    public int getNextTarget(){
        return nextTarget;
    }

    public boolean getIsChargeRequired(){
        return false;
    }

    public void upperLevelRate(int value){
        rate = value;
    }

    public double upperLevelRate(){
        return rate;
    }

    protected LitterExistingExpectation LitterExpectation(){
        return expectation;
    }

    @Override
    public int getTargetDecisionStrategy(){
        return 2;
    }

    @Override
    public String getString(){
        return "Greedy";
    }

    @Override
    public void resetState(){

    }

    @Override
    public void setPreStop(){
        preStop = true;
    }

    @Override
    public void resetPreStop(){
        preStop = false;
    }
}
