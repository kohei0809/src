package core;

public class RobotSpec {
    //ロボットの電力関係についてのプログラム

    /**
	 * @param capacity battery capacity
	 * @param wattage consumed energy
	 */
    
    private int capacity, wattage;

    public RobotSpec(int c, int w){
        setCapacity(c);
        setWattage(w);
    }

    public void setCapacity(int c){
        capacity = c;
    }

    public void setWattage(int w){
        wattage = w;
    }

    public int getCapacity(){
        return capacity;
    }

    public int getWattage(){
        return wattage;
    }

}
