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
import core.util.LogWriter;

public class AMTDSLearningTargetDecider_Security implements ITargetDecider{
    int robotID, time, preLitters = 0, travels = 0;
    double alpha, epsilon, collectedLitters = 0.0, collectExp = 0.0, consumedEnergy = 0.0;;
    Random rand;
    boolean preStop = false;

    List<Integer> nodes;
    LitterExistingExpectation expectation;

    List<ITargetDecider> deciders;
    Map<ITargetDecider, Double> Q;
    Map<ITargetDecider, Integer> deciderNumber;
    ITargetDecider selectedDecider = null;
    ITargetDecider preSelectedDecider = null;

    int maxValue = 0;

    LogWriter qLogger;

    public AMTDSLearningTargetDecider_Security(int id, IGraph m, LitterSpawnPattern pattern, boolean isAccumulate, int seed){
        robotID = id;
        nodes = new LinkedList<Integer>();
        alpha = 0.1;
        epsilon = 0.05;
        rand = new Random(seed);
        deciders = new LinkedList<ITargetDecider>();
        Q = new HashMap<ITargetDecider, Double>();
        deciderNumber = new HashMap<ITargetDecider, Integer>();

        //qLogger = LogManagerContext.getLogManager().createWriter("QValue" + robotID);
    }

    public void setParameter(double a, double e){
        alpha = a;
        epsilon = e;
    }

    public void addTargetDecider(ITargetDecider decider){
        deciders.add(decider);
        Q.put(decider, 1.0);
        deciderNumber.put(decider, deciders.size());
    }

    public void update(TargetPathAgentStatus status){
        update(status, false);
    }

    public void update(TargetPathAgentStatus status, boolean isSkip){
        time = status.getObservedData().getTime();
        RobotData mydata = status.getObservedData().getRobotDataCollection().getRobotData(robotID);
        collectExp += expectation.getExpectation(mydata.getPosition(), time);
		consumedEnergy += mydata.getConsumedEnergy();

        if(status.getAction() != AgentActions.move){
            return;
        }

        travels++;

        int litter = mydata.getAccumulatedLitter();
        if(litter > 0){
            //collectedLitters++;
            if(maxValue < litter){
                maxValue = litter;
            }
        }

        //Update target
        if(mydata.getPosition() == status.getTarget()){
            double reward = 0.0;

            if(selectedDecider == null){// initialization
                selectedDecider = deciders.get(rand.nextInt(deciders.size()));
                preSelectedDecider = selectedDecider;
            }
            else{
                //reward = collectedLitters / travels;
                //reward = collectExp / consumedEnergy / travels;
                
                //変更
                reward = (double) maxValue / travels;
                
                //最大量だけで報酬にする
                //reward = (double) maxValue;


                updateQ(reward);
                selectDecider();

                if(selectedDecider != preSelectedDecider){
                    //MyopiaSweepTargetDeciderのときに，state=0に戻す
                    preSelectedDecider.resetState();
                    preSelectedDecider = selectedDecider;
                }
            }

            travels = 0;
            collectedLitters = 0.0;
            collectExp = 0.0;
            maxValue = 0;
            selectedDecider.update(status);
        }
    }

    public void updateQ(double reward){
        double q = (1.0 - alpha) * Q.get(selectedDecider) + alpha * reward;
        Q.put(selectedDecider, q);
    }

    public void selectDecider(){
        if(rand.nextDouble() < epsilon){ // epsilon-greedy
            selectedDecider = deciders.get(rand.nextInt(deciders.size()));
        }
        else{
            double max = Double.MIN_VALUE;
            ITargetDecider maxDecider = null;
            for(ITargetDecider decider : deciders){
                if(Q.get(decider) > max){
                    max = Q.get(decider);
                    maxDecider = decider;
                }
            }
            selectedDecider = maxDecider;
        }
    }

    public void setExpectation(LitterExistingExpectation exp){
        expectation = exp;
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
        return "AMTDS-ESC";
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
