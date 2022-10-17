package core;

import java.util.List;

public class CommunicationDetails {
    //AMTSA-EDCの通信内容をまとめたプログラム

    public Coordinate myCenterNode;
    public double myExperienceWeight;
    public LitterSpawnPattern mySpawnPattern;
    public List<Integer> searchNodes;

    public CommunicationDetails(Coordinate myCenter, double weight, LitterSpawnPattern myPattern, List<Integer> searchNodes){
        myCenterNode = myCenter;
        myExperienceWeight = weight;
        mySpawnPattern = myPattern;
        this.searchNodes = searchNodes;
    }
}