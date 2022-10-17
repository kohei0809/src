package agents;

import java.util.List;

import core.agent.AgentActions;
import core.agent.ObservedData;

public class TargetPathAgentStatus {
    //ロボットのAgentActionやtargetを管理するプログラム

    private AgentActions action;
    private int target;
    private ObservedData data;
    private List<Integer> searchNodes;
    private int myCycle;
    private boolean isCycle;

    public TargetPathAgentStatus(AgentActions a, int node, ObservedData d){
        setAction(a);
        setTarget(node);
        setObservedData(d);
    }

    public TargetPathAgentStatus(AgentActions a, int node, ObservedData d, int mycycle){
        setAction(a);
        setTarget(node);
        setObservedData(d);
        setMyCycle(mycycle);
        isCycle = true;
    }

    //ごみ発生確率学習バージョン
    public TargetPathAgentStatus(AgentActions a, int node, ObservedData d, List<Integer> sNodes){
        this(a, node, d);
        setSearchNodes(sNodes);
    }

    public void setAction(AgentActions a){
        action = a;
    }

    public void setTarget(int node){
        target = node;
    }

    public void setObservedData(ObservedData d){
        data = d;
    }

    public void setSearchNodes(List<Integer> sNodes){
        searchNodes = sNodes;
    }

    public void setMyCycle(int cycle){
        myCycle = cycle;
    }

    public AgentActions getAction(){
        return action;
    }

    public int getTarget(){
        return target;
    }

    public ObservedData getObservedData(){
        return data;
    }

    public List<Integer> getSearchNodes(){
        return searchNodes;
    }

    public boolean getIsCycle(){
        return isCycle;
    }

    public int getMyCycle(){
        return myCycle;
    }
}
