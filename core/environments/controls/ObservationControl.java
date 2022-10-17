package core.environments.controls;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import core.IGraph;
import core.LitterData;
import core.LitterDataCollection;
import core.LitterSpawnPattern;
import core.RobotData;
import core.RobotDataCollection;
import core.environments.Field;
import core.environments.Litter;
import core.environments.Robot;

public class ObservationControl {
    Field field;
    private HashMap<Integer, Integer> robotEnergyCost; // < id, energy cost>
    List<Integer> maxNodeList;
    int maxNode, count;

    public ObservationControl(Field f){
        field = f;
        robotEnergyCost = new HashMap<Integer, Integer>();
		for (Robot robot : field.getRobots().getAllRobot()){
            //System.out.println("start:" + robot.getID());
			robotEnergyCost.put(robot.getID(), 0);
            //System.out.println("fin:" + robot.getID());
        }
        /*for(int i = 0; i < 20; i++){
            robotEnergyCost.put(i, 0);
        }*/

        maxNodeList = new LinkedList<Integer>();
    }

    public int getTime(){
        return field.getTime();
    }

    public IGraph getSpatialStructure(){
        return field.getSpatialStructure();
    }

    public RobotData getRobotData(int id) {
		Robot robot = field.getRobots().getRobot(id);
		return new RobotData(robot.getID(), robot.getBatteryLevel(), robot.getPosition(), robot.getAccumulatedLitter(), robot.getStepVacuumedLitter(), robot.getRobotSpec(), robotEnergyCost.get(id));
	}

    public RobotDataCollection getRobotDataCollection() {
		RobotDataCollection collection = new RobotDataCollection();
		for (Robot robot : field.getRobots().getAllRobot()) {
            //System.out.println(robot.getID());
            //System.out.println(robotEnergyCost.get(robot.getID()));
			RobotData data = new RobotData(robot.getID(), robot.getBatteryLevel(), robot.getPosition(), robot.getAccumulatedLitter(), robot.getStepVacuumedLitter(), robot.getRobotSpec(), robotEnergyCost.get(robot.getID()));
			collection.add(data);
		}
		return new RobotDataCollection(collection, true);
	}

    public LitterDataCollection getLitterDataCollection(){
        LitterDataCollection collection = new LitterDataCollection();
        for(Litter litter : field.getLitter().getAllLitter()){
            if(litter.getQuantity() != 0){
                LitterData data = new LitterData(litter.getPosition(), litter.getType(), litter.getQuantity());
                collection.addLitterData(data);
            }
        }
        return collection;
    }

    public int getLitterQuantity(){
        int amount = 0;
        for(Litter litter : field.getLitter().getAllLitter()){
            amount += litter.getQuantity();
        }
        return amount;
    }

    public int getMaxLitterQuantity(){
        int maxAmount = 0;
        count = 0;
        for(Litter litter : field.getLitter().getAllLitter()){
            int amount = litter.getQuantity();
            if(maxAmount == amount){
                maxNodeList.add(litter.getPosition());
                count++;
            }
            else if(maxAmount < amount){
                maxNodeList.clear();
                maxNodeList.add(litter.getPosition());
                maxAmount = amount;
                maxNode = litter.getPosition();

            }
        }
        return maxAmount;
    }

    public int getLitterQuantity(int position){
        for(Litter litter : field.getLitter().getAllLitter()){
            if(litter.getPosition() == position){
                //return litter.getValue();
                return litter.getQuantity();
            }
        }
        return -1;
    }

    public LitterSpawnPattern getLitterSpawnPattern(){
        return field.getLitterSpawnPattern();
    }

    public int getTotalEnergyCost() {
		int amount = 0;
		for (Robot robot : field.getRobots().getAllRobot()) {
			int energy = robot.getEnergyCost();
			robotEnergyCost.put(robot.getID(), energy);
			amount += energy;
		}
		return amount;
	}

	public void update() {
		// reset robot energy cost
		for (int i = 0; i < robotEnergyCost.size(); i++) {
			robotEnergyCost.put(i, 0);
		}
	}

    public List<Integer> getMaxLitterList(){
        return maxNodeList;
    }

    public int getMaxNode(){
        return maxNode;
    }

    public int getCount(){
        return count;
    }
}
