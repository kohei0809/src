package agents.target;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import agents.ITargetDecider;
import agents.TargetPathAgentStatus;
import agents.common.LitterExistingExpectation;
import core.IGraph;
import core.RobotData;
import core.agent.AgentActions;

public class RandomTargetDecider implements ITargetDecider{
    //ランダム法で目標決定するプログラム

    Random rand;
    List<Integer> nodes;
    int robotID;

    private int nextTarget;

    public RandomTargetDecider(int id, IGraph map, int seed, List<Integer> excludeNodes){
        rand = new Random(seed);
        nodes = new LinkedList<Integer>(map.getAllNode());

        for(int node : excludeNodes){
            nodes.remove(nodes.indexOf(node));
        }

        robotID = id;
        setNextTarget(nodes.get(rand.nextInt(nodes.size())));
    }

    public void update(TargetPathAgentStatus status){
        if(status.getAction() != AgentActions.move){
            return;
        }

        RobotData myData = status.getObservedData().getRobotDataCollection().getRobotData(robotID);
        if(myData.getPosition() == status.getTarget()){
            int target = myData.getPosition();
            do{
                target = nodes.get(rand.nextInt(nodes.size()));
            } while(target == myData.getPosition());
            setNextTarget(target);
        }
    }

    public void setExpectation(LitterExistingExpectation exp){

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

    @Override
    public int getTargetDecisionStrategy(){
        return 1;
    }

    @Override
    public String getString(){
        return "Random";
    }

    public void resetState(){

    }

    public void setPreStop(){

    }

    public void resetPreStop(){
        
    }
}
