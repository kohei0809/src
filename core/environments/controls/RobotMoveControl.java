package core.environments.controls;

import java.util.AbstractMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import core.environments.Field;
import core.environments.RobotCollection;

public class RobotMoveControl {
    //各ロボットの向かう先を格納し，moveさせるプログラム

    Field field;
    RobotCollection robots;
    List<Map.Entry<Integer, Integer>> dest; //<id, destination>

    public RobotMoveControl(Field f){
        field = f;
        robots = field.getRobots();
        dest = new LinkedList<Map.Entry<Integer, Integer>>();
    }

    public void move(int id, int destination){
        dest.add(new AbstractMap.SimpleEntry<Integer, Integer>(id, destination));
    }

    //destに溜まった，各ロボットの移動を行う
    public void update(){
        for(Map.Entry<Integer, Integer> pair : dest){
            robots.getRobot(pair.getKey()).move(pair.getValue());
            
        }
        dest.clear();
    }
}
