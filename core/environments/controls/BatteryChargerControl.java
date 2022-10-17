package core.environments.controls;

import core.environments.Field;
import core.environments.Robot;
import core.environments.RobotBase;
import core.environments.RobotBaseCollection;
import core.environments.RobotCollection;

public class BatteryChargerControl {
    //RobotCollectionやRobotBaseCollectionを使用して充電するプログラム

    Field field;
    RobotCollection robots;
    RobotBaseCollection bases;
    //LogWriter2 connectLogger, disconnectLogger;

    public BatteryChargerControl(Field f){
        field = f;
        robots = field.getRobots();
        bases = field.getRobotBases();
        /*connectLogger = LogManagerContext.getLogManager().createWriter2("ConnectLogger");
        connectLogger.writeLine("time,id,battery");
        disconnectLogger = LogManagerContext.getLogManager().createWriter2("DisconnectLogger");
        disconnectLogger.writeLine("time,id,battery");*/
    }

    public void connect(int id){
        Robot robot = robots.getRobot(id);
        RobotBase robotBase = bases.getRobotBase(robot.getPosition());
        robotBase.connect(robot);
        //connectLogger.writeLine(field.getTime() + "," + robot.getID() + "," + robot.getBatteryLevel());
    }

    public void disconnect(int id){
        Robot robot = robots.getRobot(id);
        RobotBase robotBase = bases.getRobotBase(robot.getPosition());
        robotBase.disconnect(robot);
        //disconnectLogger.writeLine(field.getTime() + "," + robot.getID() + "," + robot.getBatteryLevel());
    }

    public void update(){
        for(RobotBase rb : bases.getAllRobotBase()){
            rb.charge();
        }
    }
}
