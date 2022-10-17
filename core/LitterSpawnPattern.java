package core;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class LitterSpawnPattern {
    //ごみの発生確率をノードごとに管理するプログラム

    Map<Integer, LitterSpawnProbability> patterns;// < node, probability>

    public LitterSpawnPattern(){
        patterns = new HashMap<Integer, LitterSpawnProbability>();
    }

    public void addSpawnProbability(int node, LitterSpawnProbability prob){
        patterns.put(node, prob);
    }

    public void removeSpawnProbability(int node){
        patterns.remove(node);
    }

    public LitterSpawnProbability getSpawnProbability(int node){
        return patterns.get(node);
    }

    public void setProbability(int node, int vacuumedLitter, int stepInterval){
        patterns.get(node).updateProbability(vacuumedLitter, stepInterval);
    }

    public Map<Integer, LitterSpawnProbability> getAllPatterns(){
        return patterns;
    }

    public List<Integer> getAllKeys(){
        return new LinkedList<Integer>(patterns.keySet());
    }
}