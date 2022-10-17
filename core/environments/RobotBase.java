package core.environments;

import java.util.List;
import java.util.LinkedList;

public class RobotBase {
    //充電基地についてのプログラム
    
    List<Robot> robots;
    BatteryCharger charger;

    private int id, position;
    static int ID = 0;

    public RobotBase(int chargeValue){
        robots = new LinkedList<Robot>();
        charger = new BatteryCharger(chargeValue);
        id = ID++;
    }

    public void connect(Robot robot){
        robots.add(robot);
        robot.connectBase();
        charger.connect(robot.getBattery());
    }

    public void disconnect(Robot robot){
        robots.remove(robot);
        robot.disconnectBase();
        charger.disconnect(robot.getBattery());
    }

    public void charge(){
        charger.charge();
    }

    public void setID(int i){
        id = i;
    }

    public void setPosition(int p){
        position = p;
    }

    public int getID(){
        return id;
    }

    public int getPosition(){
        return position;
    }
}
