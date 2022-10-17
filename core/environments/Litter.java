package core.environments;

public class Litter {
    //ごみに関するプログラム

    private int quantity, position;
    private String type;
    private boolean isAccumulate;
    private int value;

    public Litter(String t, int p){
        setType(t);
        setPosition(p);
    }

    public Litter(String t, int p, boolean isA){
        setType(t);
        setPosition(p);
        setIsAccumulate(isA);
    }

    public void increase(int amount){
        if(isAccumulate){
            quantity += amount;
        }
        else{
            quantity = amount;
        }
        //System.out.println("spwan:" + position + " " + quantity + " " + amount);
    }

    public int clean(int amount){//掃除したごみの量を返す
        value = amount;
        if(amount > quantity){
            value = quantity;
        }
        quantity -= value;
        return value;
    }

    public int clean(){
        value = quantity;
        quantity = 0;
        return value;
    }

    public void setType(String t){
        type = t;
    }

    public void setQuantity(int q){
        if(q < 0){
            quantity = 0;
        }
        else{
            quantity = q;
        }
    }

    public void setIsAccumulate(boolean isA){
        isAccumulate = isA;
    }

    public void setPosition(int p){
        position = p;
    }

    public String getType(){
        return type;
    }

    public int getQuantity(){
        return quantity;
    }

    public boolean getIsAccumulate(){
        return isAccumulate;
    }

    public int getPosition(){
        return position;
    }

    public int getValue(){
        return value;
    }
}