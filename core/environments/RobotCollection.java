package core.environments;

import java.util.Map;
import java.util.HashMap;
import java.util.LinkedList;

public class RobotCollection {
    //ロボットを追加したり削除したりするプログラム

    Map<Integer, Robot> robots;

    public RobotCollection(){
        robots = new HashMap<Integer, Robot>();
    }

    public void add(Robot robot){
        robots.put(robot.getID(), robot);
    }

    public void remove(Robot robot){
        robots.remove(robot.getID());
    }

    public Robot getRobot(int id){
        return robots.get(id);
    }

    public LinkedList<Robot> getAllRobot(){
        //System.out.println("n=" + robots.size());
        return new LinkedList<Robot>(robots.values());
    }
}
