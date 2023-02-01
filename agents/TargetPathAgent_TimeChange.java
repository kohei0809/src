package agents;

import java.util.LinkedList;
import java.util.List;
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

public class TargetPathAgent_TimeChange implements IAgent{
    //AMTDS/ER

    final private int homingTime = 1000;
    final private int checkInterval = 100;
    final private double alpha = 0.1;
    final private int scale = 50;

    private ITargetDecider targetter;
    private IPathPlanner pather;
    private int baseNode, target;
    private double returnProb = 0.0, waitProb = 0.0;
    private boolean isChargeRequired = false, homingFlag = false, hasUpdatedTargetter = false;
    private GridGraph graph;
    private Random rand;
    private int[][] agentPosition = new int[2*scale+1][2*scale+1];
    List<Integer> excludeNodes;

    private LitterExistingExpectation expectation;
    private RequirementEstimator estimator;
    private Importance importance;

    private int robotID, counter, waitCounter, sumTime = 0;
    private AgentActions action;
    
    private String dir;
    private LogWriter2 chargeLogger;
    private LogWriter2 waitLogger;
    private LogWriter2 correctionLogger;

    public TargetPathAgent_TimeChange(int id, LitterSpawnPattern pattern, GridGraph graph, int seed, List<Integer> excludeNodes) {
		setRobotID(id);
		action = AgentActions.standby;
		expectation = new LitterExistingExpectation(pattern, true);
		importance = new Importance(graph);
        this.graph = graph;
        this.excludeNodes = excludeNodes;

		rand = new Random(seed);
		counter = 0;
        waitCounter = 0;

        for (int node : excludeNodes) {
            Coordinate c = graph.getCoordinate(node);
            int x = c.x + scale;
            int y = -c.y + scale;
            agentPosition[y][x] = -1;
        }

        dir = LogManagerContext.getLogManager().makeDir("Agent" + robotID);
        //time, start, homingFlag, battery
        chargeLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Charge");
        //time,remainBattery
        waitLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Wait");
        //time, correction, newcorrection, realValue, req
        correctionLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Correction");
    }

    @Override
    public void update(ObservedData data){
        RobotData robotData = data.getRobotDataCollection().getRobotData(robotID);
        int position = robotData.getPosition();
        int time = data.getTime();
        hasUpdatedTargetter = false;

        expectation.update(data.getRobotDataCollection(), time);
        importance.update(action, robotData.getVacuumedLitter());

        countPosition(position);

        if(time == 0){
            correctionLogger.writeLine(time + "," + estimator.getCorrection() + "," + estimator.getCorrection() + "," + -1 + "," + estimator.getRequirement());
            waitLogger.writeLine(time + "," + -1 + "," + -1 + "," + 0);
        }

        //State transition
        if(action == AgentActions.move){
            if(position == target){
                if(isChargeRequired){
                    // start charging
                    action = AgentActions.charge;
                    counter = 0;
    		        chargeLogger.writeLine(time + ",start," + homingFlag + "," + robotData.getBatteryLevel() + "," + 0);
                }
            }
        }
        else if(action == AgentActions.charge){
            if(robotData.getBatteryLevel() == robotData.getRobotSpec().getCapacity()){
                //finish charging
                //Homing後は必ずPausingを行う
                if(homingFlag){
                    action = AgentActions.wait;
                    isChargeRequired = false;
                    waitCounter = 0;
                }
                else{
                    action = AgentActions.move;
                    isChargeRequired = false;
                    chargeLogger.writeLine(time + ",finish," + homingFlag + "," + robotData.getBatteryLevel() + "," + 0);
                }

                // When litter amount requirement is reached
                estimator.update(expectation, position);
                if(time > homingTime && (estimator.requirementReached() == true || homingFlag)){
                    // Target Decision
                    TargetPathAgentStatus status = new TargetPathAgentStatus(action, target, data);
                    targetter.update(status);
                    hasUpdatedTargetter = true;
                    importance.update_future(position, targetter.getNextTarget(), expectation, time);
                    waitProb = 1.0 - importance.evaluate();
                    if(rand.nextDouble() < waitProb || homingFlag){
                        action = AgentActions.wait;
                        waitCounter = 0;
                        sumTime = calculatePausingTime(time, position);
                     
                        chargeLogger.writeLine(time + ",wait,"+ "," + robotData.getBatteryLevel() + "," + sumTime);
                        waitLogger.writeLine(time + "," + robotData.getBatteryLevel() + "," + waitProb + "," + sumTime);
                        return;
                    }
                }
            }
        }
        else if(action == AgentActions.wait){
            waitCounter++;
            if(waitCounter >= sumTime){
                action = AgentActions.move;
                homingFlag = false;
                waitCounter = 0;
                chargeLogger.writeLine(time + ",unwait," + homingFlag + "," + robotData.getBatteryLevel() + "," + 0);
                
                if(estimator.getChange()){
                    updateCorrection(data);
                } 
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
			if (time > homingTime && !isChargeRequired && robotData.getBatteryLevel() < robotData.getRobotSpec().getCapacity() / 3){
				estimator.update(expectation, position);
                if(estimator.requirementReached() == true){
                    importance.update_future(position, targetter.getNextTarget(), expectation, time);
				    returnProb = 1.0 - importance.evaluate();
				    if (rand.nextDouble() < returnProb) {
                        homingFlag = true;
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

    //K^iを更新する
    private void updateCorrection(ObservedData data){        
        List<Integer> nodes = graph.getAllNode();

		double sum = 0.0;
		for (int node : nodes) {
			double exp = expectation.getExpectation(node);
			sum += exp;
		}
		
		double realValue =sum;

        double correction = 0;
        if(realValue <= estimator.getRequirement()){
        	correction = (1.0 - alpha) * estimator.getCorrection() + alpha * (estimator.getRequirement() / realValue) * estimator.getCorrection();
        }
        else {
        	correction = estimator.getCorrection() - (realValue / estimator.getRequirement() - 1);
        }
        
        correctionLogger.writeLine(data.getTime() + "," + estimator.getCorrection() + "," + correction + "," + realValue + "," + estimator.getRequirement());

        estimator.setCorrection(correction);
    }

    //Pausingの長さを計算する
    private int calculatePausingTime(int time, int position){
    	int waitTime = 100;
        int checkTime = 10000;
        while(checkTime > 10){
        	while(true){
                estimator.update_future(expectation, position, time + waitTime);
                              
                if(estimator.requirementReached() == false){
                    waitTime = waitTime - checkTime;
                    if(waitTime < 0){
                    	waitTime = 0;
                    }
                    break;
                }
                else{
                }
                waitTime += checkTime;
            }
        	checkTime /= 10;
        }  
        return waitTime;
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
    public int getPositionCount(int x, int y){
        return agentPosition[x][y];
    }

    @Override
    public void countPosition(int position){
        Coordinate c = graph.getCoordinate(position);
        int x = c.x + scale;
        int y = -c.y + scale;
        agentPosition[y][x]++;
    }


    //////////////////////////////ダミーメソッド//////////////////////////////
    @Override
    public void setEnvironment(IEnvironment env){
    }

    @Override
    public void setSleepProbability(double p){
    }

    @Override
    public void setStopTime(int time){
    }

    @Override
    public void setRestartTime(int time){
    }

    @Override
    public void setPreStop(){
    }

    @Override
    public int getStopTime(){
        return -1;
    }

    @Override
    public int getRestartTime(){
        return -1;
    }

    @Override
    public boolean getPreStop(){
        return false;
    }

    @Override
    public void addStartIndex(int n){
    }

    @Override
    public void resetPreStop(){
    }

    @Override
    public CommunicationDetails getCommunicationDetails(){
        return new CommunicationDetails(new Coordinate(0, 0), 0.0, new LitterSpawnPattern(), new LinkedList<Integer>());
    }

    @Override
    public void searchNumberDecrease(int decrease){
    }

    @Override
	public void setRequirement(double req) {
		estimator.setRequirement(req);
	}

    @Override
    public double getCorrection(){
        return estimator.getCorrection();
    }

    @Override
	public void setEnvironmentEstimator(RequirementEstimator estimator) {
		this.estimator = estimator;
	}

    @Override
	public int getSearchNodeNumber(){
		return -1;
	}

	@Override
    public Coordinate getCenterNode(){
		return new Coordinate(0, 0);
	}

    @Override
	public int getSuccessCount(){
        return -1;
	}
    
	@Override
    public int getFailureCount(){
        return -1;
	}

    @Override
    public void resetSuccessCount(){
    }

    @Override
	public void setEnvironmentEstimatorU(RequirementEstimatorU estimator){
	}

    @Override
    public void restart(ObservedData data){
    }

    @Override
    public int getCycleIndex(){
        return 0;
    }

	@Override
	public double getRequirement() {
		return estimator.getRequirement();
	}
}


