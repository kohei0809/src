package core;

import java.util.List;

import java.util.LinkedList;

public class LitterDataCollection {
    //ごみをリストにまとめて管理するプログラム

    List<LitterData> litter;

    public LitterDataCollection(){
        litter = new LinkedList<LitterData>();
    }

    public void addLitterData(LitterData dirt){
        litter.add(dirt);
    }

    public List<LitterData> getAllLitters(){
        return litter;
    }
}
