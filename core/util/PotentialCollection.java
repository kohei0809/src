package core.util;

import java.util.Map;

public class PotentialCollection {
    //nodeごとのpotentialを管理するプログラム
    /**
	 * potential: minimum capacity of battery to return to its charging base
	 */

    Map<Integer, Integer> potentials; //<node ID, potential>
    public PotentialCollection(Map<Integer, Integer> potentials){
        this.potentials = potentials;
    }

    public int getPotential(int node){
        return potentials.get(node);
    }

    public Map<Integer, Integer> getAllPotentials(){
        return potentials;
    }
}
