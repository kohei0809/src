package agents.target;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.util.TreeSet;

import agents.ITargetDecider;
import agents.TargetPathAgentStatus;
import agents.common.LitterExistingExpectation;
import core.IGraph;
import core.LitterSpawnPattern;
import core.RobotData;
import core.agent.AgentActions;
import core.util.DijkstraAlgorithm;
import core.util.PotentialCollection;

public class MyopiaSweepTargetDecider implements ITargetDecider{
    //戦略型目標決定法
    
    int robotID, myopia, seepTimes, state, area;
    double threshold, alpha = 0.05, epsilon = 0.05;
    Random rand;
    IGraph map;
    List<Integer> possibleNodes;
    List<Integer> excludeNodes;
    ForMyopiaGreedy decider;

    int sweepCounter = Integer.MAX_VALUE;
    boolean preStop = false;

    public MyopiaSweepTargetDecider(int id, IGraph m, LitterSpawnPattern pattern, boolean isAccumulate, int seed, List<Integer> excludeNodes){
        robotID = id;
        map = m;
        decider = new ForMyopiaGreedy(robotID, map, pattern, isAccumulate, seed, excludeNodes);

        possibleNodes = new LinkedList<Integer>(map.getAllNode());

        for(int node : excludeNodes){
            possibleNodes.remove(possibleNodes.indexOf(node));
        }

        myopia = 15;
        this.excludeNodes = excludeNodes;
        rand = new Random(seed);
    }
    
    public void update(TargetPathAgentStatus status){
        RobotData mydata = status.getObservedData().getRobotDataCollection().getRobotData(robotID);
        if(mydata.getPosition() == status.getTarget() && status.getAction() == AgentActions.move){
            // Arrive! Choose new target from whole area
            if(state == 0){
                List<Integer> nodes = getRealNearbyNodes(mydata.getPosition()); //ポテンシャル基準，実距離
                //List<Integer> nodes = getNearbyNodes(mydata.getPosition()); //最短経路，壁無視
                decider.setAccessibleNodes(nodes);
                area = nodes.size();
                state = 1;
            }

            //Update threshold
            else if(state == 1){
                double value = decider.expectationSum;
                if(threshold != 0){
                    // for balancing local and non-local target selection
                    threshold = alpha * value / area + (1 - alpha) * threshold;
                }
                else{
                    // Recalculation
                    // average value of expected accumulated dirt
                    // value of cleaned dirt per tick if agent moves in neighbor
                    threshold = value / area;
                }

                state = 2;
            }

            //Inside neighbor
            else if(state == 2){
                double exp = decider.expectationSum / area;
                if(exp < threshold || rand.nextDouble() < epsilon){
                    // Threshold not reached. Go to original target.
                    decider.setAccessibleNodes(possibleNodes);
                    state = 0;
                }
            }
        }
        decider.update(status);
    }

    public List<Integer> getNearbyNodes(int start){
        List<Integer> nodes = new LinkedList<Integer>();
        Set<Integer> investigated = new TreeSet<Integer>();

        nodes.add(start);
        investigated.add(start);

        // myopia : distance from start
        for(int i = 0; i < myopia; i++){
            List<Integer> nexts = new LinkedList<Integer>();
            for(int node : nodes){
                List<Integer> children = map.getChildNodes(node);
                for(int child : children){
                    if(investigated.contains(child)){
                        continue;
                    }
                    if(!excludeNodes.contains(child)){
                        investigated.add(child);
                    }
                    nexts.add(child);
                }
            }
            nodes = nexts;
        }
        return new LinkedList<Integer>(investigated);
    }

    public List<Integer> getRealNearbyNodes(int start){
        PotentialCollection myopiaPotential = new DijkstraAlgorithm(map).execute(start);
        List<Integer> realInvestigated = new LinkedList<Integer>();

        for(Map.Entry<Integer, Integer> node : myopiaPotential.getAllPotentials().entrySet()){
            if(node.getValue() <= myopia){
                realInvestigated.add(node.getKey());
            }
        }
        return realInvestigated;
    }

    public void setExpectation(LitterExistingExpectation exp){
        decider.setExpectation(exp);
    }

    public int getNextTarget(){
        return decider.getNextTarget();
    }

    public boolean getIsChargeRequired(){
        return false;
    }

    @Override
    public int getTargetDecisionStrategy(){
        return 4;
    }

    @Override
    public String getString(){
        return "BNPS";
    }

    @Override
    public void resetState(){ //領域内清掃中に別戦略を取った時に，領域情報をリセットする
        state = 0;
    }

    @Override
    public void setPreStop(){
        preStop = true;
        decider.setPreStop();;
    }

    @Override
    public void resetPreStop(){
        preStop = false;
        decider.resetPreStop();
    }
}
