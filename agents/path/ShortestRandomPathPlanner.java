package agents.path;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import agents.IPathPlanner;
import agents.TargetPathAgentStatus;
import agents.common.LitterExistingExpectation;
import core.IGraph;
import core.RobotData;
import core.agent.AgentActions;
import core.util.DijkstraAlgorithm;
import core.util.PotentialCollection;

public class ShortestRandomPathPlanner implements IPathPlanner{
    //ランダム法で最短距離を計画するプログラム

    Random rand;
    int robotID, target, base;
    IGraph map;
    PotentialCollection potentialMap, pathMap;
    List<Integer> excludeNodes = new LinkedList<Integer>();

    private int nextNode;
    private boolean canArrive;

    public ShortestRandomPathPlanner(int id, int baseNode, IGraph spatialStructure, int seed, List<Integer> excludeNodes){
        robotID = id;
        base = baseNode;
        target = baseNode;
        map = spatialStructure;
        this.excludeNodes = excludeNodes;
        
        rand = new Random(seed);
        potentialMap = new DijkstraAlgorithm(map).execute(base);
    }

    @Override
    public void  update(TargetPathAgentStatus status){
        RobotData data = status.getObservedData().getRobotDataCollection().getRobotData(robotID);
        int position = data.getPosition();

        if(status.getAction() != AgentActions.move){
            return;
        }

        if(target != status.getTarget()){//when changing target
            target = status.getTarget();
            pathMap = new DijkstraAlgorithm(map).execute(target, position);
            int wattage = data.getRobotSpec().getWattage();

            //targetに行った後にbaseに戻る分の充電がない場合
            if((potentialMap.getPotential(target) + pathMap.getPotential(position)) * wattage > data.getBatteryLevel()){
                setCanArrive(false);
                return;
            }
            setCanArrive(true);
        }

        LinkedList<Integer> candidates = new LinkedList<Integer>();
        for(int node : map.getChildNodes(position)){
            if(pathMap.getPotential(node) < pathMap.getPotential(position)){
                candidates.add(node);
            }
        }
        setNextNode(candidates.get(rand.nextInt(candidates.size())));
    }

    public void setNextNode(int node){
        nextNode = node;
    }

    public void setCanArrive(boolean b){
        canArrive = b;
    }

    @Override
    public int getNextNode(){
        return nextNode;
    }

    @Override
    public boolean getCanArrive(){
        return canArrive;
    }

    public void setExpectation(LitterExistingExpectation exp){

    }

    @Override
    public int getTarget(){
        return target;
    }

    public void updateHoming(TargetPathAgentStatus status){

    } 
}
