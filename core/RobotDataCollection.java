package core;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import core.environments.Robot;

public class RobotDataCollection {
    Map<Integer, RobotData> robots;
    boolean isReadOnly;

    public RobotDataCollection(){
        robots = new HashMap<Integer, RobotData>();
        isReadOnly = false;
    }

    public RobotDataCollection(RobotDataCollection collection, boolean rdonly){
        isReadOnly = rdonly;
        robots = collection.robots;
    }

    public void add(RobotData robot){
        if(!isReadOnly){
            robots.put(robot.getID(), robot);
        }
        else{
            throw new IllegalStateException("This Collection is ReadOnly.");
        }
    }

    public void remove(Robot robot){
        if(!isReadOnly){
            robots.remove(robot.getID());
        }
        else{
            throw new IllegalStateException("This Collection is ReadOnly.");
        }
    }

    public RobotData getRobotData(int id){
        return robots.get(id);
    }

    public List<RobotData> getAllRobotData(){
        return new LinkedList<RobotData>(robots.values());
    }
}
