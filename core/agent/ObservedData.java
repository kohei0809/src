package core.agent;

import core.RobotDataCollection;

public class ObservedData {
    private int time;
    private RobotDataCollection robotData;

    public ObservedData(int t, RobotDataCollection data){
        setTime(t);
        setRobotData(data);
    }

    public void setTime(int time){
        this.time = time;
    }

    public void setRobotData(RobotDataCollection data){
        robotData = data;
    }

    public int getTime(){
        return time;
    }

    public RobotDataCollection getRobotDataCollection(){
        return robotData;
    }
}
