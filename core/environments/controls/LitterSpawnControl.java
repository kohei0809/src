package core.environments.controls;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import core.LitterSpawnPattern;
import core.LitterSpawnProbability;
import core.environments.Field;
import core.environments.LitterCollection;

public class LitterSpawnControl {
    //ごみの出現をコントロールするプログラム

    Field field;
    LitterCollection litter;
    LitterSpawnPattern patterns;
    Random rand;

    public LitterSpawnControl(Field f, int seed){
        field = f;
        litter = field.getLitter();
        patterns = field.getLitterSpawnPattern();
        rand = new Random(seed);
    }

    public void update(){
        Map<Integer, LitterSpawnProbability> allPatterns =patterns.getAllPatterns();
        List<Integer> patternKey = new LinkedList<Integer>(allPatterns.keySet());
        for(int key : patternKey){
            LitterSpawnProbability prob = allPatterns.get(key);
            if(field.getTime() % prob.getInterval() == 0){
                double n = rand.nextDouble();
                if(n < prob.getProbability()){//ごみの発生確率を満たしたため，ごみ発生
                    litter.getLitter(key).increase(prob.getIncrement());//ごみの累積数を増やす
                }
            }
        }
    }
}
