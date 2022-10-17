package core.environments;

import java.util.List;
import java.util.LinkedList;

public class BatteryCharger {
    //ロボットを充電する(何体でも可)

    public List<Battery> batteries;
    private int chargeRate;

    public BatteryCharger(int chargeRate){
        batteries = new LinkedList<Battery>();
        setChargeRate(chargeRate);
    }

    public void charge(){
        for(Battery battery : batteries){
            battery.charge(chargeRate);
        }
    }

    public void connect(Battery battery){
        batteries.add(battery);
    }

    public void disconnect(Battery battery){
        batteries.remove(battery);
    }

    public void setChargeRate(int rate){
        chargeRate = rate;
    }

    public int getChargeRate(){
        return chargeRate;
    }
}