package core;

public class RobotData {
    //ロボットのデータを管理するプログラム

    private int id, batteryLevel, position, accumulatedLitter, litter, scale, consumedEnergy;
    private RobotSpec spec;

    public RobotData(int id, int b, int p, int a, int l, RobotSpec s, int e){
        setID(id);
        setBatteryLevel(b);
        setPosition(p);
        setAccumulatedLitter(a);
        setVacuumedLitter(l);
        setRobotSpec(s);
        setConsumedEnergy(e);
        setScale(scale);
    }

    public void setID(int i){
        id = i;
    }

    public void setBatteryLevel(int b){
        batteryLevel = b;
    }

    public void setPosition(int p){
        position = p;
    }

    public void setAccumulatedLitter(int a){
        accumulatedLitter = a;
    }

    public void setVacuumedLitter(int l){
        litter = l;
    }

    public void setRobotSpec(RobotSpec s){
        spec = s;
    }

    public void setScale(int s){
        scale = s;
    }

    public void setConsumedEnergy(int x) {
		consumedEnergy = x;
	}

    public int getID(){
        return id;
    }

    public int getBatteryLevel(){
        return batteryLevel;
    }

    public int getPosition(){
        return position;
    }

    public int getAccumulatedLitter(){
        return accumulatedLitter;
    }

    public int getVacuumedLitter(){
        return litter;
    }

    public RobotSpec getRobotSpec(){
        return spec;
    }

    public int getScale(){
        return scale;
    }

    public int getConsumedEnergy() {
		return consumedEnergy;
	}
}
