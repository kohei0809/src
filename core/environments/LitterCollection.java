package core.environments;

import java.util.Map;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

public class LitterCollection {
    //ごみを場所も含めてまとめて管理する

    Map<Integer, Litter> litter;

    public LitterCollection(){
        litter = new HashMap<Integer, Litter>();        
    }

    public void add(Litter dirt){
        litter.put(dirt.getPosition(), dirt);
    }

    public void remove(Litter dirt){
        litter.remove(dirt.getPosition());
    }

    public Litter getLitter(int position){
        return litter.get(position);
    }

    public List<Litter> getAllLitter(){
        return new LinkedList<Litter>(litter.values());
    }
}