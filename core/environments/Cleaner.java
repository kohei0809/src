package core.environments;

public class Cleaner {
    //取ったごみの量や現在のステップで取ったごみの量を格納するプログラム

    private int accumulatedLitter; // accumulate amount of vacuumed litter
	private int litter; // step amount of vacuumed litter


    public Cleaner(){

    }

    public void clean(Litter dirt) {
		litter = dirt.clean();
        //System.out.println("amount=" + litter);
		accumulatedLitter += litter;
	}   

    public int getLitter(){
        return litter;
    }

    public int getAccumulatedLitter(){
        return accumulatedLitter;
    }
}