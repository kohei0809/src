package agents.path;

import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

import agents.IPathPlanner;
import agents.TargetPathAgentStatus;
import agents.common.LitterExistingExpectation;
import core.IGraph;
import core.LitterSpawnPattern;
import core.RobotData;
import core.Tuple;
import core.agent.AgentActions;
import core.util.DijkstraAlgorithm;
import core.util.PotentialCollection;

public class SubgoalPathPlanner implements IPathPlanner{

    int robotID, target;
    IGraph map;

    int myopia;
    int time;
    double attraction, rover, threshold;
    LitterExistingExpectation expectation;
    PotentialCollection basePotential, targetPotential;
    //distance of returning to base

    IPathPlanner pathPlanner;
    List<Integer> subgoals;
    int subgoal, goalIndex;
    List<Integer> excludeNodes = new LinkedList<Integer>();

    private boolean canArrive;

    public SubgoalPathPlanner(int id, IGraph m, LitterSpawnPattern pattern, int baseNode, boolean isAccumulate, int seed, List<Integer> excludeNodes){
        robotID = id;
        map = m;
        this.excludeNodes = excludeNodes;
        basePotential = new DijkstraAlgorithm(map).execute(baseNode);
        pathPlanner = new ShortestGreedyPathPlanner(robotID, map, pattern, baseNode, isAccumulate, seed, excludeNodes);

        //ごみの発生確率が既知の場合
        expectation = new LitterExistingExpectation(pattern, isAccumulate);

        subgoal = baseNode;

        myopia = 10;
        attraction = 1.0;
        rover = 1.2;
        threshold = 0.0;
    }

    public void update(TargetPathAgentStatus status){
        RobotData mydata = status.getObservedData().getRobotDataCollection().getRobotData(robotID);
        int position = mydata.getPosition();
        time = status.getObservedData().getTime();
        //System.out.println(status.getIsCycle());

        if(target != status.getTarget() || subgoals == null){
            int battery = mydata.getBatteryLevel();
            int wattage = mydata.getRobotSpec().getWattage();
            int cycle = 0;
            int remainBattery = 0;

            if(status.getIsCycle() == true){
                cycle = status.getMyCycle();
                remainBattery = cycle * wattage - (mydata.getRobotSpec().getCapacity() - battery);
            }
            else{
                remainBattery = battery;
            }

            target = status.getTarget();
            targetPotential = new DijkstraAlgorithm(map).execute(target, position);

            //Determine if possible to reach target
            if((basePotential.getPotential(target) + targetPotential.getPotential(position)) * wattage > remainBattery){
                canArrive = false;
                return;
            }
            canArrive = true;

            //Create subgoal
            subgoals = new LinkedList<Integer>();
            Tuple<Integer, Integer> goal = new Tuple<Integer,Integer>(position, remainBattery - basePotential.getPotential(target) * wattage);
            do{
                goal = getSubgoal(goal.val1, goal.val2, wattage);
                subgoals.add(goal.val1);
            } while(goal.val1 != target);
            goalIndex = 0;
        }

        if(subgoal == position && status.getAction() == AgentActions.move){
            if(goalIndex >= subgoals.size()){
                goalIndex = subgoals.size() - 1;
            }
            subgoal = subgoals.get(goalIndex);
            goalIndex++;
        }

        if(status.getIsCycle() == true){
        	pathPlanner.update(new TargetPathAgentStatus(status.getAction(), subgoal, status.getObservedData(), status.getMyCycle()));
        }
        else{
        	pathPlanner.update(new TargetPathAgentStatus(status.getAction(), subgoal, status.getObservedData()));
        }
        
    }

    public void updateHoming(TargetPathAgentStatus status){
        //目的地までの最短を行く
        pathPlanner.update(status);
    } 

    Tuple<Integer, Integer> getSubgoal(int start, int battery, int wattage){
        int stDistance = targetPotential.getPotential(start);

        // Add to goals if close enough to target
        if(stDistance <= myopia){
            return new Tuple<Integer,Integer>(target, battery - stDistance * wattage);
        }

        // Investigate child nodes
        List<Integer> nodes = new LinkedList<Integer>();
        Set<Integer> investigated = new TreeSet<Integer>();
        // Value: <node, consumed energy>
        TreeMap<Double, Tuple<Integer, Integer>> candidate = new TreeMap<Double, Tuple<Integer, Integer>>(
                new Comparator<Double>() {
                   @Override
                   public int compare(Double o1, Double o2){
                       return o2 > o1 ? 1 : o2 == o1 ? 0 : -1;
                   } 
                });
        
        nodes.add(start);
        investigated.add(start);

        for(int i = 1; i <= myopia; i++){
            List<Integer> nexts = new LinkedList<Integer>();

            for(int node : nodes){
                List<Integer> children = map.getChildNodes(node);
                for(int child : children){
                    if(investigated.contains(child)){
                        continue;
                    }
                    investigated.add(child);

                    int childDistance = targetPotential.getPotential(child);
                    int distance = i + childDistance;
                    int level = battery - distance * wattage;
                    if(level >= 0 && distance <= rover * childDistance){
                        nexts.add(child);
                        if(expectation.getExpectation(child) >= threshold && childDistance < attraction * stDistance){
                            candidate.put(expectation.getExpectation(child), new Tuple<Integer,Integer>(child, battery - i * wattage));
                        }
                    }
                }
            }

            nodes = nexts;
        }

        if(candidate.size() == 0){
            return new Tuple<Integer,Integer>(target, battery - stDistance);
        }

        return candidate.firstEntry().getValue();
    }

    public void setExpectation(LitterExistingExpectation exp){
        expectation = exp;
        pathPlanner.setExpectation(exp);
    }

    public void setCanArrive(boolean b){
        canArrive = b;
    }

    public int getNextNode(){
        return pathPlanner.getNextNode();
    }

    public boolean getCanArrive(){
        return canArrive;
    }

    public int getTarget(){
        return target;
    }
}
