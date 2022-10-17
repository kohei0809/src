package core;

public class LitterSpawnProbability {
    //ごみの発生確率を管理するプログラム

    double alpha, tmpalpha;
    double vaccumedLitter = 0.0, stepInterval = 0.0;

    private double probability; 
    private int interval;
    private int increment;

    public LitterSpawnProbability(int inte, double prob, int inc){
        //initialization: 1, 0.0, 1
        setInterval(inte);
        setProbability(prob);
        setIncrement(inc);
    }

    /*
	 * Calculate litter spawn probability from vacuumed litter.
	 *
	 * @param litter vacuumed litter amount
	 *
	 * @param interval time difference between current time and the most recent
	 * time when an agent visited the node
	 */

    public void updateProbability(double litter, int inte){
        vaccumedLitter = litter;
        stepInterval = inte;
        alpha = 0.1;
        tmpalpha = 0.00001;
        /*if(vaccumedLitter >= 1.0){
            vaccumedLitter = 1.0;
        }*/
        
        //
       /*alpha = tmpalpha * inte;
        if(alpha > 0.1){
        	alpha = 0.1;
        }*/
        //

        if(stepInterval != 0){
            probability = (1.0 - alpha) * probability + alpha * (vaccumedLitter / stepInterval);//probabilityの更新
        }
    }

    public void setInterval(int inte){
        interval = inte;
    }

    public void setProbability(double prob){
        probability = prob;
    }

    public void setIncrement(int inc){
        increment = inc;
    }

    public int getInterval(){
        return interval;
    }

    public double getProbability(){
        return probability;
    }

    public int getIncrement(){
        return increment;
    }
}
