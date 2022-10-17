package agents;

import java.util.LinkedList;
import java.util.Random;

import agents.common.Importance;
import agents.common.LitterExistingExpectation;
import agents.common.RequirementEstimator;
import agents.common.RequirementEstimatorU;
import core.CommunicationDetails;
import core.Coordinate;
import core.GridGraph;
import core.IEnvironment;
import core.LitterSpawnPattern;
import core.RobotData;
import core.agent.AgentActions;
import core.agent.IAgent;
import core.agent.ObservedData;
import core.util.LogManagerContext;
import core.util.LogWriter2;

public class TargetPathAgent_Wait implements IAgent{
    //Agentが経路追従等をするプログラム(ごみ発生確率は知っている)Pausing

    ITargetDecider targetter;
    IPathPlanner pather;
    int baseNode, target, nextNode;
    double waitProb = 0.0;
    boolean isChargeRequired = false, hasUpdatedTargetter = false;
    GridGraph graph;
    Random rand;

    LitterExistingExpectation expectation;
    RequirementEstimator estimator;
    Importance importance;
    

    private int robotID, waitTime = 100, chargeCounter, chargeTime = 2700;
    private AgentActions action;
    private String dir;

    LogWriter2 chargeLogger;
    LogWriter2 waitLogger;
                    
    public TargetPathAgent_Wait(int id, LitterSpawnPattern pattern, GridGraph graph, int seed) {
		setRobotID(id);
		action = AgentActions.standby;
		expectation = new LitterExistingExpectation(pattern, true);
		importance = new Importance(graph);
        this.graph = graph;

		rand = new Random(seed);
        chargeCounter = 0;

        dir = LogManagerContext.getLogManager().makeDir("Agent" + robotID);
        //time,remainBattery
        waitLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Wait");
        //time, start, homingFlag, battery
        chargeLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Charge");

	}

    @Override
    public void update(ObservedData data){
        update(data, false);
    }

    @Override
    public void update(ObservedData data, boolean flipCoin){
        RobotData robotData = data.getRobotDataCollection().getRobotData(robotID);
        int position = robotData.getPosition();
        hasUpdatedTargetter = false;

        expectation.update(data.getRobotDataCollection(), data.getTime());
        importance.update(action, robotData.getVacuumedLitter());

        //State transition
        if(action == AgentActions.move){
            if(position == target){
                if(isChargeRequired){
                    // start charging
                    action = AgentActions.charge;
                    chargeCounter = 0;
                    chargeLogger.writeLine(data.getTime() + ",start," + "," + robotData.getBatteryLevel());
                }
            }
        }
        else if(action == AgentActions.charge){
            chargeCounter++;
            if(robotData.getBatteryLevel() == robotData.getRobotSpec().getCapacity()){
                //finish charging
                action = AgentActions.move;
                isChargeRequired = false;
                
                // When litter amount requirement is reached
                estimator.update(expectation, position);

                if(data.getTime() > 1000 && estimator.requirementReached() == true){
                    // Target Decision
                    TargetPathAgentStatus status = new TargetPathAgentStatus(action, target, data);
                    targetter.update(status);
                    hasUpdatedTargetter = true;
                    importance.update_future(position, targetter.getNextTarget(), expectation, data.getTime());
                    waitProb = 1.0 - importance.evaluate();
                    if(rand.nextDouble() < waitProb){
                        action = AgentActions.wait;
                        chargeLogger.writeLine(data.getTime() + ",wait,"+ "," + robotData.getBatteryLevel());
		    		    waitLogger.writeLine(data.getTime() + "," + robotData.getBatteryLevel() + "," + waitProb);
                        return;
                    }
                }

                chargeLogger.writeLine(data.getTime() + ",finish," + "," + robotData.getBatteryLevel());
            }
        }
        else if(action == AgentActions.wait){
            chargeCounter++;
            if(chargeCounter < waitTime + chargeTime){
                return;
            }
            if(chargeCounter == waitTime + chargeTime){
                action = AgentActions.move;
                chargeCounter = 0;
                chargeLogger.writeLine(data.getTime() + ",unwait," + "," + robotData.getBatteryLevel());
            }
        }
        else if(action == AgentActions.standby){
            action = AgentActions.move;
            target = position;
        }
        else if(action == AgentActions.sleep){
            action = AgentActions.move;
        }

        //Target Decision
        if(!hasUpdatedTargetter){
            TargetPathAgentStatus status = new TargetPathAgentStatus(action, target, data);
            targetter.update(status);
        }

        /*if(data.getTime() % 1000 == 0) {
        	LogWriter2 actionLogger;
            actionLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Action");
            Coordinate c = graph.getCoordinate(position);
            Coordinate tc = graph.getCoordinate(targetter.getNextTarget());
            Coordinate pc = graph.getCoordinate(pather.getTarget());
            //time, action, isChargeReturn, x, y, chargeCounter
    		actionLogger.writeLine(data.getTime() + "," + action + "," + "," + c.x + "," + c.y + "," + tc.x + "," + tc.y + "," + pc.x + "," + pc.y + "," + data.getRobotDataCollection().getRobotData(robotID).getBatteryLevel() + "," + chargeCounter);
        }*/

        //Path Gneration
        if(target != targetter.getNextTarget() && !isChargeRequired){
            //new target
            target = targetter.getNextTarget();

            TargetPathAgentStatus status = new TargetPathAgentStatus(action, target, data);
            pather.update(status);

            if(pather.getCanArrive()){
                return;
            }

            //Enforce to charge if cannot arrive
            target = baseNode;
            isChargeRequired = true;
            if(position == baseNode){
                action = AgentActions.charge;
                return;
            }
        }

        if(position == baseNode && isChargeRequired){
            action = AgentActions.charge;
            return;
        }

        TargetPathAgentStatus status = new TargetPathAgentStatus(action, target, data);
        pather.update(status);
    }

    @Override
    public void setRobotID(int id){
        robotID = id;
    }

    @Override
    public void setBaseNode(int base){
        baseNode = base;
    }

    @Override
    public void setPathPlanner(IPathPlanner p){
        pather = p;
    }

    @Override
    public void setTargetDecider(ITargetDecider t){
        targetter = t;
    }

    @Override
    public void setAction(AgentActions a){
        action = a;
    }

    @Override
    public void setExpectation(){
        targetter.setExpectation(expectation);
        pather.setExpectation(expectation);
    }

    @Override
    public int getRobotID(){
        return robotID;
    }

    @Override
    public AgentActions getAction(){
        return action;
    }

    @Override
    public int getNextNode(){
        return pather.getNextNode();
    }

    @Override
    public int getTargetter(){
        return targetter.getTargetDecisionStrategy();
    }

    @Override
    public LitterSpawnPattern getMySpawnPattern(){
        return expectation.getMySpawnPattern();
    }

    @Override
    public LitterExistingExpectation getExpectation(){
        return expectation;
    }

    @Override
    public void setExpectation(LitterExistingExpectation exp){
        expectation = exp;
    }

    @Override
    public void setEnvironment(IEnvironment environment){

    }

    @Override
	public void setRequirement(double req) {
		estimator.setRequirement(req);
	}

    @Override
	public void setEnvironmentEstimator(RequirementEstimator estimator) {
		this.estimator = estimator;
	}

    @Override
    public double getCorrection(){
        return estimator.getCorrection();
    }

    @Override
    public int getPositionCount(int x, int y){
        return -1;
    }

    @Override
    public void countPosition(int position){

    }

    @Override
    public boolean requirementReached(){
        return false;
    }

    @Override
    public void setSleepProbability(double p){
        //ダミーメソッド
    }

    @Override
    public void setStopTime(int time){
        //ダミーメソッド
    }

    @Override
    public void setRestartTime(int time){
        //ダミーメソッド
    }

    @Override
    public void setPreStop(){
        //ダミーメソッド
    }

    @Override
    public int getStopTime(){
        //ダミーメソッド
        return -1;
    }

    @Override
    public int getRestartTime(){
        //ダミーメソッド
        return -1;
    }

    @Override
    public boolean getPreStop(){
        //ダミーメソッド
        return false;
    }

    @Override
    public void addStartIndex(int n){
        //ダミーメソッド
    }

    @Override
    public void resetPreStop(){
        //ダミーメソッド
    }

    @Override
    public CommunicationDetails getCommunicationDetails(){
        //ダミーメソッド
        return new CommunicationDetails(new Coordinate(0, 0), 0.0, new LitterSpawnPattern(), new LinkedList<Integer>());
    }

    @Override
    public void searchNumberDecrease(int decrease){
        //ダミーメソッド
    }

    @Override
	public int getSearchNodeNumber(){
		//ダミーメソッド
		return -1;
	}

	@Override
    public Coordinate getCenterNode(){
		//ダミーメソッド
		return new Coordinate(0, 0);
	}

    @Override
	public int getSuccessCount(){
		//ダミーメソッド
        return -1;
	}

    @Override
    public int getCycleIndex(){
        return 0;
    }
    
	@Override
    public int getFailureCount(){
		//ダミーメソッド
        return -1;
	}

    @Override
    public void resetSuccessCount(){
        //ダミーメソッド
    }

    @Override
	public void setEnvironmentEstimatorU(RequirementEstimatorU estimator){
		//ダミーメソッド
	}

    @Override
    public void restart(ObservedData data){
        //ダミーメソッド
    }
}
