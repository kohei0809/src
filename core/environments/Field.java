package core.environments;

import core.IGraph;
import core.LitterSpawnPattern;

public class Field {
    //フィールドに関するプログラム

    private int time;
    private IGraph spatialStructure;
    private RobotCollection robots;
    private LitterCollection litter;
    private RobotBaseCollection robotBases;
    private LitterSpawnPattern litterSpawnPattern;

    public Field(IGraph structure, LitterSpawnPattern pattern){
        time = 0;
        spatialStructure = structure;
        litterSpawnPattern = pattern;
        robots = new RobotCollection();
        robotBases = new RobotBaseCollection();
        litter = new LitterCollection();
    }

    public void addRobot(Robot robot){
        robots.add(robot);
    }

    public void addLitter(Litter dirt){
        litter.add(dirt);
    }

    public void addRobotBase(RobotBase base){
        robotBases.add(base);
    }

    public void removeLitter(int position){
        litter.remove(litter.getLitter(position));
    }

    public void updateTime(){
        time++;
    }

    public void setTime(int t){
        time = t;
    }

    public void setSpatialStructure(IGraph g){
        spatialStructure = g;
    }

    public void setRobots(RobotCollection r){
        robots = r;
    }

    public void setLitter(LitterCollection l){
        litter = l;
    }

    public void setRobotBases(RobotBaseCollection b){
        robotBases = b;
    }

    public void setLitterSpawnPattern(LitterSpawnPattern p){
        litterSpawnPattern = p;
    }

    public int getTime(){
        return time;
    }

    public IGraph getSpatialStructure(){
        return spatialStructure;
    }

    public RobotCollection getRobots(){
        return robots;
    }

    public LitterCollection getLitter(){
        return litter;
    }

    public RobotBaseCollection getRobotBases(){
        return robotBases;
    }

    public LitterSpawnPattern getLitterSpawnPattern(){
        return litterSpawnPattern;
    }
}
