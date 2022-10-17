package core;

public class Coordinate {
    //座標に関するプログラム

    public int x, y;

    public Coordinate(int x, int y){
        this.x = x;
        this.y = y;
    }

    public String ToString(){
        return x + "," + y;
    }

    public Coordinate minus(Coordinate a){
        return new Coordinate(x - a.x, y - a.y);
    }

    public double length(){
        return Math.sqrt(x * x + y * y);
    }
}
