package agents.target;

import java.util.HashMap;
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

//Interval-Greedy
public class IntervalGreedyTargetDecider implements ITargetDecider{
    int robotID, learnTime;
    Random rand;
    boolean preStop = false;

    List<Integer> nodes;
    LitterExistingExpectation exception;

    List<ITargetDecider> deciders;
    Map<ITargetDecider, Integer> deciderNumber;
    ITargetDecider selectedDecider = null;

    public IntervalGreedyTargetDecider(int id, IGraph m, LitterSpawnPattern pattern, boolean isAccumulate, int seed){
        robotID = id;
        nodes = new LinkedList<Integer>();
        rand = new Random(seed);
        deciders = new LinkedList<ITargetDecider>();
        deciderNumber = new HashMap<ITargetDecider, Integer>();
        learnTime = 200000;
    }

    //最初にInterval, 次にGreedyを入れる
    public void addTargetDecider(ITargetDecider decider){
        deciders.add(decider);
        deciderNumber.put(decider, deciders.size());
    }

    public void update(TargetPathAgentStatus status){
        update(status, false);
    }

    public void update(TargetPathAgentStatus status, boolean isSkip){
        RobotData mydata = status.getObservedData().getRobotDataCollection().getRobotData(robotID);

        if(status.getAction() != AgentActions.move){
            return;
        }

        //Update target
        if(mydata.getPosition() == status.getTarget()){
            if(selectedDecider == null){// initialization
                selectedDecider = deciders.get(rand.nextInt(deciders.size()));
            }
            else{
                selectDecider(status);
            }
            selectedDecider.update(status);
        }
    }

    public void selectDecider(TargetPathAgentStatus status){
        int time = status.getObservedData().getTime();
        if(time <= learnTime){ //learnTimeまでInterval学習 
            selectedDecider = deciders.get(0);
        }
        else{
            selectedDecider = deciders.get(1);
        }
    }

    public void setExpectation(LitterExistingExpectation exp){
        exception = exp;
        for(ITargetDecider strategy : deciders){
            strategy.setExpectation(exp);
        }
    }

    public int getNextTarget(){
        return selectedDecider.getNextTarget();
    }

    public boolean getIsChargeRequired(){
        return selectedDecider.getIsChargeRequired();
    }

    @Override
    public int getTargetDecisionStrategy(){
        return deciderNumber.get(selectedDecider);
    }

    @Override
    public String getString(){
        return "IntervalGreedy";
    }

    @Override
    public void resetState(){

    }

    @Override
    public void setPreStop(){
        preStop = true;
        for(ITargetDecider strategy : deciders){
            strategy.setPreStop();
        }
    }

    @Override
    public void resetPreStop(){
        preStop = false;
        for(ITargetDecider strategy : deciders){
            strategy.resetPreStop();
        }
    } 

}
