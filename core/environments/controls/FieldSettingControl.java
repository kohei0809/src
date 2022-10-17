package core.environments.controls;

import core.RobotSpec;
import core.environments.Field;
import core.environments.Litter;
import core.environments.Robot;
import core.environments.RobotBase;

public class FieldSettingControl {
    //フィールドにロボットや基地，ごみを作成するプログラム

    Field field;
    int robotID;

    public FieldSettingControl(Field f){
        field = f;
    }

    public int createRobot(RobotSpec spec, int position){
        Robot robot = new Robot(spec, robotID++);
        robot.setFieldStructure(field.getSpatialStructure());
        robot.setPosition(position);

        field.addRobot(robot);
        robot.activate();
        
        return robot.getID();
    }

    public int createRobotBase(int chargeValue, int position){
        RobotBase base = new RobotBase(chargeValue);
        base.setPosition(position);
        field.addRobotBase(base);
        return base.getID();
    }

    public void createLitter(String type, int position, boolean isAccumulate){
        field.addLitter(new Litter(type, position, isAccumulate));
    }

    public void update(){
        field.updateTime();
    }
}
