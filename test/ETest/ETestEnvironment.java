package test.ETest;

import java.util.Random;

import core.IEnvironment;
import core.IGraph;
import core.LitterDataCollection;
import core.LitterSpawnPattern;
import core.RobotData;
import core.RobotDataCollection;
import core.RobotSpec;
import core.environments.Field;
import core.environments.controls.FieldSettingControl;
import core.environments.controls.LitterSpawnControl;
import core.environments.controls.ObservationControl;

public class ETestEnvironment implements IEnvironment{
    //ETest用の環境

    Field field;
    int state = 0;
    Random rand;

    FieldSettingControl fieldSettingControl;
    LitterSpawnControl litterSpawnControl;
    ObservationControl observer;

    //state
    final int initializing = 0;
    final int litterSpawning = 1;
    final int timeUpdating = 2;

    public ETestEnvironment(IGraph spatialStructure, LitterSpawnPattern spawnPattern, boolean isLitterAccumulate, int seed){
        rand = new Random(seed);
        field = new Field(spatialStructure, spawnPattern);
        fieldSettingControl = new FieldSettingControl(field);
        litterSpawnControl = new LitterSpawnControl(field, seed);
        observer = new ObservationControl(field);

        for(int node : spatialStructure.getAllNode()){
            fieldSettingControl.createLitter("non",node, isLitterAccumulate);
        }
    }

    public void update(){
        switch(state){
            case 0:
                state = 1;
                break;
            case 1://Litter appears
                litterSpawnControl.update();
                state = 2;
                break;
            case 2://Update time
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

    public int createRobot(RobotSpec spec, int position){
        throw new IllegalStateException("This object doesn't support this method.");
    }

    public int setRobotBase(int chargeValue, int position){
        throw new IllegalStateException("This object doesn't support this method.");
    }

    public void moveRobot(int id, int node){
        throw new IllegalStateException("This object doesn't support this method.");
    }

    public void connectRobotBase(int id){
        throw new IllegalStateException("This object doesn't support this method.");
    }

	public void disconnectRobotBase(int id){
        throw new IllegalStateException("This object doesn't support this method.");
    }

	public void clean(int id){
        throw new IllegalStateException("This object doesn't support this method.");
    }

    public RobotData getRobotData(int id){
        throw new IllegalStateException("This object doesn't support this method.");
    }

	public RobotDataCollection getRobotDataCollection(){
        throw new IllegalStateException("This object doesn't support this method.");
    }

	public int getTotalEnergyCost(){
        return -1;
    }

    public int getLitterAmount(int position){
        return -1;
    }

    public int getMaxTarget(){
        return -1;
    }

    public int getNodeListSize(){
        return -1;
    }
}

