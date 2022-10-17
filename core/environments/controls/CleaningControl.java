package core.environments.controls;

import java.util.AbstractMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import core.environments.Cleaner;
import core.environments.Field;
import core.environments.Litter;
import core.environments.LitterCollection;
import core.environments.Robot;
import core.environments.RobotCollection;

public class CleaningControl {
    //各ロボットのいる位置の掃除を一気に行うプログラム

    Field field;
    LitterCollection litterCollection;
    RobotCollection robots;
    List<Map.Entry<Cleaner, Litter>> tasks;
    Random rand;

    public CleaningControl(Field f, int seed){
        field = f;
        litterCollection = field.getLitter();
        robots = field.getRobots();

        tasks = new LinkedList<Map.Entry<Cleaner, Litter>>();
        rand = new Random(seed);
    }

    public void clean(int id){
        Robot robot = robots.getRobot(id);
        int index = 0;

        if(tasks.size() > 0){
            index = rand.nextInt(tasks.size());
        }

        //tasksにidのrobotの掃除タスクを追加
        tasks.add(index, new AbstractMap.SimpleEntry<Cleaner, Litter>(robot.getCleaner(), litterCollection.getLitter(robot.getPosition())));
        //System.out.println(robot.getPosition() + ":litter=" + litterCollection.getLitter(robot.getPosition()).getQuantity());
        
    }

    //tasksに溜まっている掃除を行う
    public void update(){
        for(Map.Entry<Cleaner, Litter> task : tasks){
            Cleaner cleaner = task.getKey();
            Litter litter = task.getValue();
            cleaner.clean(litter);
        }
        tasks.clear();
    }
}
