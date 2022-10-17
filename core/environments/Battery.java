package core.environments;

public class Battery {
    //電力をチャージ・消費するプログラム

    private int level, capacity;

    public Battery(int cap){
        setCapacity(cap);
        setLevel(cap);
    }

    public int charge(int value){
        setLevel(level + value);
        return level;
    }

    public int discharge(int value){
        setLevel(level - value);
        return level;
    }

    public void setLevel(int value){
        if(value > capacity){
            level = capacity;
        }
        else if(value < 0){
            level = 0;
        }
        else{
            level = value;
        }
    }

    public void setCapacity(int cap){
        capacity = cap;
    }

    public int getLevel(){
        return level;
    }

    public int getCapacity(){
        return capacity;
    } 
}