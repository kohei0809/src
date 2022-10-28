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

public class TargetPathAgent_ReturnAndWait implements IAgent{
    //Agentが経路追従等をするプログラム(ごみ発生確率は知っている)Return

    ITargetDecider targetter;
    IPathPlanner pather;
    int baseNode, target, nextNode;
    double returnProb = 0.0, waitProb = 0.0;
    boolean isChargeRequired = false, homingFlag = false, hasUpdatedTargetter = false;
    GridGraph graph;
    Random rand;

    LitterExistingExpectation expectation;
    RequirementEstimator estimator;
    Importance importance;

    private int robotID, counter, chargeCounter, checkInterval = 100, chargeTime = 2700, waitTime = 100, sumTime = 0;
    private AgentActions action;
    private String dir;

    LogWriter2 chargeLogger;
    LogWriter2 actionLogger;
    LogWriter2 returnLogger;
    LogWriter2 waitLogger;

    public TargetPathAgent_ReturnAndWait(int id, LitterSpawnPattern pattern, GridGraph graph, int seed) {
		setRobotID(id);
		action = AgentActions.standby;
		expectation = new LitterExistingExpectation(pattern, true);
		importance = new Importance(graph);
        this.graph = graph;

		rand = new Random(seed);
		counter = 0;
        chargeCounter = 0;

        dir = LogManagerContext.getLogManager().makeDir("Agent" + robotID);
        //time, start, homingFlag, battery
        chargeLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Charge");
        //time, action, isChargeReturn, x, y
        actionLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Action");
        //time,remainBattery, px, py, returnFlag
        returnLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Return");
        //time,remainBattery
        waitLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Wait");
	}

    @Override
    public void update(ObservedData data){
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
                    counter = 0;
                    chargeCounter = 0;
    		        chargeLogger.writeLine(data.getTime() + ",start," + homingFlag + "," + robotData.getBatteryLevel());
                }
            }
        }
        else if(action == AgentActions.charge){
            chargeCounter++;
            if(robotData.getBatteryLevel() == robotData.getRobotSpec().getCapacity()){
                //finish charging

                if(homingFlag){
                    //充電し始めてから2,700step経過していたらmoveになる
                    if(chargeCounter == chargeTime){
                        action = AgentActions.move;
                        isChargeRequired = false;
                        homingFlag = false;
                        chargeLogger.writeLine(data.getTime() + ",finish," + homingFlag + "," + robotData.getBatteryLevel());
                    }
                    else{
                        action = AgentActions.wait;
                        isChargeRequired = false;
                        sumTime = chargeTime;
                        chargeLogger.writeLine(data.getTime() + ",wait," + homingFlag + "," + robotData.getBatteryLevel() + "," + sumTime);
                    }
                }
                else{
                    action = AgentActions.move;
                    isChargeRequired = false;
                    chargeLogger.writeLine(data.getTime() + ",finish," + homingFlag + "," + robotData.getBatteryLevel());
                }

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
                        sumTime = chargeTime + waitTime;
                        chargeLogger.writeLine(data.getTime() + ",wait,"+ "," + robotData.getBatteryLevel() + "," + sumTime);
                        waitLogger.writeLine(data.getTime() + "," + robotData.getBatteryLevel() + "," + waitProb);
                        return;
                    }
                }
            }
        }
        else if(action == AgentActions.wait){
            chargeCounter++;
            if(chargeCounter == sumTime){
                action = AgentActions.move;
                homingFlag = false;
                chargeCounter = 0;
                chargeLogger.writeLine(data.getTime() + ",unwait," + homingFlag + "," + robotData.getBatteryLevel());
            }
            else{
                return;
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
        TargetPathAgentStatus status = new TargetPathAgentStatus(action, target, data);
        if(!hasUpdatedTargetter){
            targetter.update(status);
        }

        // Identify agent behavior (every 100 ticks)
		if (action == AgentActions.move && counter == checkInterval) {
			counter = 0;
			// When litter amount requirement is reached
			if (data.getTime() > 1000 && !isChargeRequired && robotData.getBatteryLevel() < robotData.getRobotSpec().getCapacity() / 3){
				estimator.update(expectation, position);
                if(estimator.requirementReached() == true){
                    importance.update_future(position, targetter.getNextTarget(), expectation, data.getTime());
				    returnProb = 1.0 - importance.evaluate();
				    if (rand.nextDouble() < returnProb) {
                        homingFlag = true;

                        Coordinate pc = graph.getCoordinate(pather.getTarget());
		    		    returnLogger.writeLine(data.getTime() + "," + robotData.getBatteryLevel() + "," + pc.x + "," + pc.y + "," + homingFlag);

                        if(position == baseNode){
                            action = AgentActions.charge;
                            return;
                        }  
                        return;                  
				    }
                }
			}
		}

        //Path Gneration
        if(homingFlag && action == AgentActions.move){
            target = baseNode;
			isChargeRequired = true;
			status = new TargetPathAgentStatus(action, target, data);
            //homingでは最短で帰る(バッテリ切れで止まることがあるから)謎
			pather.updateHoming(status);
			counter = 0; // stop counting -> no requirement estimation

			if (position == baseNode) {
				action = AgentActions.charge;
				return;
			}

        }
        else if(target != targetter.getNextTarget() && !isChargeRequired){
            //new target
            target = targetter.getNextTarget();

            status = new TargetPathAgentStatus(action, target, data);
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
            status = new TargetPathAgentStatus(action, target, data);
        }

        if(position == baseNode && isChargeRequired){
            action = AgentActions.charge;
            return;
        }

        if(!homingFlag){
            pather.update(status);
        }

        if(action == AgentActions.move && !isChargeRequired){
            counter++;
        }
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
	public void setRequirement(double req) {
		estimator.setRequirement(req);
	}

    @Override
	public void setEnvironmentEstimator(RequirementEstimator estimator) {
		this.estimator = estimator;
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

    @Override
    public int getCycleIndex(){
        return 0;
    }
}

