package agents.path;

import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.TreeMap;

import agents.IPathPlanner;
import agents.TargetPathAgentStatus;
import agents.common.LitterExistingExpectation;
import core.IGraph;
import core.LitterSpawnPattern;
import core.RobotData;
import core.agent.AgentActions;
import core.util.DijkstraAlgorithm;
//import core.util.LogManager2;
import core.util.PotentialCollection;

public class ShortestGreedyPathPlanner implements IPathPlanner{
    //Greedy法で経路生成

    private Random rand;
    private int robotID, target = -1, base, time;
    private IGraph map;
    private LitterExistingExpectation expectation;
    private PotentialCollection potentialMap, pathMap;
    private List<Integer> excludeNodes = new LinkedList<Integer>();

    private int nextNode;
    private boolean canArrive;

    //LogManager2 shortestGreedyLogger;

    public ShortestGreedyPathPlanner(int id, IGraph m, LitterSpawnPattern pattern, int baseNode, boolean isAccumulate, int seed, List<Integer> excludeNodes){
        robotID = id;
        map = m;
        base = baseNode;
        rand = new Random(seed);
        this.excludeNodes = excludeNodes;

        //path from charging base (distance to charging base)
        potentialMap = new DijkstraAlgorithm(map).execute(base);

        //ごみの発生確率が既知の場合
        expectation = new LitterExistingExpectation(pattern, isAccumulate);
    }

    public void update(TargetPathAgentStatus status){
        RobotData data = status.getObservedData().getRobotDataCollection().getRobotData(robotID);
        int position = data.getPosition();
        time = status.getObservedData().getTime();

        if(status.getAction() != AgentActions.move){
            return;
        }
        if(target != status.getTarget()){
            int battery = data.getBatteryLevel();
            int wattage = data.getRobotSpec().getWattage();
            int cycle = 0;
            int remainBattery = 0;

            if(status.getIsCycle() == true){
                cycle = status.getMyCycle();
                remainBattery = cycle * wattage - (data.getRobotSpec().getCapacity() - battery);
            }
            else{
                remainBattery = battery;
            }


            target = status.getTarget();
            //path from current position to target node (distance to ...)
            pathMap = new DijkstraAlgorithm(map).execute(target, position);

            //Cannot arrive when energy cost of
            //(target -> base) + (position -> target) > current battery level
            if((potentialMap.getPotential(target) + pathMap.getPotential(position)) * wattage > remainBattery){
                canArrive = false;
                return;
            }
            canArrive = true;
        }

        TreeMap<Double, Integer> candidates = new TreeMap<Double, Integer>(new Comparator<Double>() {
            @Override
            public int compare(Double o1, Double o2){
                //降順
                return o2.compareTo(o1);
            }
        });

        for(int node : map.getChildNodes(position)){
            //(child-node -> target) < (position -> target)
            if(pathMap.getPotential(node) < pathMap.getPotential(position)){
                candidates.put(expectation.getExpectation(node), node);
            }
        }

        if(candidates.size() == 0){
            if(position == target){
                //greedy-target-decider might choose itself as target
                setNextNode(position);
            }
            return;
        }

        setNextNode(candidates.firstEntry().getValue());
    }

    public void setExpectation(LitterExistingExpectation exp){
        expectation = exp;
    }

    public void setNextNode(int node){
        nextNode = node;
    }

    public void setCanArrive(boolean b){
        canArrive = b;
    }

    public int getNextNode(){
        return nextNode;
    }

    public boolean getCanArrive(){
        return canArrive;
    }

    public int getTarget(){
        return target;
    }

    public void updateHoming(TargetPathAgentStatus status){

    }
}
