package core.environments;

import java.util.Map;
import java.util.HashMap;
import java.util.LinkedList;

public class RobotBaseCollection {
    //充電基地を複数管理するプログラム

    Map<Integer, RobotBase> bases;
    
    public RobotBaseCollection(){
        bases = new HashMap<Integer, RobotBase>();
    }

    public void add(RobotBase robotBase){
        bases.put(robotBase.getPosition(), robotBase);
    }

    public void remove(RobotBase robotBase){
        bases.remove(robotBase.getPosition());
    }

    public RobotBase getRobotBase(int position){
        return bases.get(position);
    }

    public LinkedList<RobotBase> getAllRobotBase(){
        return new LinkedList<RobotBase>(bases.values());
    }
}
