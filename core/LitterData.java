package core;

public class LitterData {
    //ごみのposition, type, amountを管理するプログラム

    private int position, amount;
    private String type;

    public LitterData(int p, String t, int a){
        setPosition(p);
        setType(t);
        setAmount(a);
    }

    public void setPosition(int p){
        position = p;
    }

    public void setType(String t){
        type = t;
    }

    public void setAmount(int a){
        amount = a;
    }

    public int getPosition(){
        return position;
    }

    public String getType(){
        return type;
    }

    public int getAmount(){
        return amount;
    }
}
