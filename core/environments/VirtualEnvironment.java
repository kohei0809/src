package core.environments;

import java.util.List;
import java.util.Random;

import core.IEnvironment;
import core.IGraph;
import core.LitterDataCollection;
import core.LitterSpawnPattern;
import core.RobotData;
import core.RobotDataCollection;
import core.RobotSpec;
import core.environments.controls.BatteryChargerControl;
import core.environments.controls.CleaningControl;
import core.environments.controls.FieldSettingControl;
import core.environments.controls.LitterSpawnControl;
import core.environments.controls.ObservationControl;
import core.environments.controls.RobotMoveControl;

public class VirtualEnvironment implements IEnvironment{
    Field field;
    int state = 0;
    Random rand;

    FieldSettingControl fieldSettingControl;
    BatteryChargerControl batteryChargerControl;
    CleaningControl cleaningControl;
    LitterSpawnControl litterSpawnControl;
    RobotMoveControl robotMoveControl;
    ObservationControl observer;
    List<Integer> nodeList;

    //state
    final int initializing = 0;
    final int litterSpawning = 1;
    final int robotMoving = 2;
    final int robotCleaning = 3;
    final int timeUpdating = 4;

    public VirtualEnvironment(IGraph spatialStructure, LitterSpawnPattern spawnPattern, boolean isLitterAccumulate, int seed){
        rand = new Random(seed);
        field = new Field(spatialStructure, spawnPattern);

        fieldSettingControl = new FieldSettingControl(field);
        batteryChargerControl = new BatteryChargerControl(field);
        cleaningControl = new CleaningControl(field, rand.nextInt());
        litterSpawnControl = new LitterSpawnControl(field, rand.nextInt());
        robotMoveControl = new RobotMoveControl(field);
        observer = new ObservationControl(field);

        for(int node : spatialStructure.getAllNode()){
            fieldSettingControl.createLitter("none", node, isLitterAccumulate);
        }
    }

    public int createRobot(RobotSpec spec, int position){
        return fieldSettingControl.createRobot(spec, position);
    }

    public int setRobotBase(int chargeValue, int position){
        return fieldSettingControl.createRobotBase(chargeValue, position);
    }

    public void moveRobot(int id, int node){
        robotMoveControl.move(id, node);
    }

    public void connectRobotBase(int id){
        batteryChargerControl.connect(id);
    }

    public void disconnectRobotBase(int id){
        batteryChargerControl.disconnect(id);
    }

    public void clean(int id){
        cleaningControl.clean(id);
    }

    public void update(){
        switch(state){
            case 0:
                state = 1;
                break;
            case 1: //Litter appears
                litterSpawnControl.update();
                state = 2;
                break;
            case 2: //move, charge
                batteryChargerControl.update();
                robotMoveControl.update();
                state = 3;
                break;
            case 3: //clean
                cleaningControl.update();
                state = 4;
                break;
            case 4: //Update time
                fieldSettingControl.update();
                state = 1;
                break;
        }
    }

    public int getTime(){
        return observer.getTime();
    }

    public IGraph getSpatialStructure(){
        return observer.getSpatialStructure();
    }

    public RobotData getRobotData(int id){
        return observer.getRobotData(id);
    }

    public RobotDataCollection getRobotDataCollection(){
        return observer.getRobotDataCollection();
    }

    public LitterDataCollection getLitterDataCollection(){
        return observer.getLitterDataCollection();
    }

    public int getLitterAmount(){
        return observer.getLitterQuantity();
    }

    public int getMaxLitterAmount(){
        return observer.getMaxLitterQuantity();
    }

    public LitterSpawnPattern getLitterSpawnPattern(){
        return observer.getLitterSpawnPattern();
    }

    public int getTotalEnergyCost() {
		return observer.getTotalEnergyCost();
	}

    public int getLitterAmount(int position){
        return observer.getLitterQuantity(position);
    }

    public int getMaxTarget(){
        nodeList = observer.getMaxLitterList();
        return nodeList.get(rand.nextInt(nodeList.size()));
        //return observer.getMaxNode();
    }

    public int getNodeListSize(){
        return nodeList.size();
        //return observer.getCount();
    }
}
