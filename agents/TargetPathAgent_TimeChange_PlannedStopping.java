package agents;

import java.util.AbstractMap;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
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
import core.LitterSpawnProbability;
import core.RobotData;
import core.agent.AgentActions;
import core.agent.IAgent;
import core.agent.ObservedData;
import core.util.LogManagerContext;
import core.util.LogWriter2;

public class TargetPathAgent_TimeChange_PlannedStopping implements IAgent{
    ITargetDecider targetter;
    IPathPlanner pather;
    int baseNode, target, nextNode;
    double returnProb = 0.0, waitProb = 0.0;
    boolean isChargeRequired = false, homingFlag = false, hasUpdatedTargetter = false;
    GridGraph graph;
    Random rand;
    IEnvironment environment;
    double alpha = 0.1;
    int scale = 50;
    int[][] agentPosition = new int[2*scale+1][2*scale+1];
    List<Integer> excludeNodes;
    LitterSpawnPattern mySpawnPattern; //agent's map
    LitterSpawnPattern realSpawnPattern; //real map
    List<Integer> visitedNodes;

    LitterExistingExpectation expectation;
    RequirementEstimator estimator;
    Importance importance;

    List<Integer> nodes;
    List<Integer> searchNodes;
    int searchNodeNumber;
    int maxNodeNumber;
    Coordinate myCenterNode = new Coordinate(0, 0);
    double myCenterNodeWeight = 0.0;
    CommunicationDetails communicationDetail;

    private int robotID, counter, waitCounter, checkInterval = 100, sumTime = 0;
    private AgentActions action;
    private int stopTime =-1;
    private int restartTime = -1;
    private int searchNodeStartIndex = 0;
    private boolean preStop = false;
    private String dir;

    LogWriter2 chargeLogger;
    LogWriter2 actionLogger;
    LogWriter2 returnLogger;
    LogWriter2 waitLogger;
    LogWriter2 estimationLogger;
    LogWriter2 correctionLogger;

    public TargetPathAgent_TimeChange_PlannedStopping(int id, LitterSpawnPattern pattern, GridGraph graph, int seed, List<Integer> excludeNodes) {
		setRobotID(id);
		action = AgentActions.standby;
		importance = new Importance(graph);
        this.graph = graph;
        this.excludeNodes = excludeNodes;
        visitedNodes = new LinkedList<Integer>();

        realSpawnPattern = pattern;
        InitializeMySpawnPattern();

        expectation = new LitterExistingExpectation(mySpawnPattern, true);

		rand = new Random(seed);
		counter = 0;
        waitCounter = 0;

        nodes = new LinkedList<Integer>(this.graph.getAllNode());
        
        for(int node : excludeNodes){
            nodes.remove(nodes.indexOf(node));
        }
        searchNodes = new LinkedList<Integer>(nodes);
        searchNodeNumber = nodes.size();
        maxNodeNumber = nodes.size();
        communicationDetail = new CommunicationDetails(myCenterNode, myCenterNodeWeight, mySpawnPattern, searchNodes);

        for (int node : excludeNodes) {
            Coordinate c = graph.getCoordinate(node);
            int x = c.x + scale;
            int y = -c.y + scale;
            agentPosition[y][x] = -1;
        }

        dir = LogManagerContext.getLogManager().makeDir("Agent" + robotID);
        //time, start, homingFlag, battery
        chargeLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Charge");
        //time, action, isChargeReturn, x, y
        actionLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Action");
        //time,remainBattery, px, py, returnFlag
        returnLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Return");
        //time,remainBattery
        waitLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Wait");
        //time, estimation
        estimationLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Estimate");
        //time, correction, newcorrection, realValue, req
        correctionLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Correction");
    }

    @Override
    public void update(ObservedData data){
        RobotData robotData = data.getRobotDataCollection().getRobotData(robotID);
        int position = robotData.getPosition();
        int interval = expectation.getInterval(position, data.getTime());
        int vacuumedLitter = robotData.getVacuumedLitter();
        hasUpdatedTargetter = false;

        // Update agent's probability map
        mySpawnPattern.setProbability(position, vacuumedLitter, interval);

        // Available to know other's position
        expectation.update(data.getRobotDataCollection(), data.getTime());

        importance.update(action, vacuumedLitter);

        countPosition(position);

        visitPosition(position);

        if(data.getTime() == 0){
            correctionLogger.writeLine(data.getTime() + "," + estimator.getCorrection() + "," + -1 + "," + -1 + "," + estimator.getRequirement());
        }

        //State transition
        if(action == AgentActions.move){
            if(position == target){
                if(isChargeRequired){
                    // start charging
                    action = AgentActions.charge;
                    calculateSearchNode();
                    calculateCenterNode();
                    counter = 0;
    		        chargeLogger.writeLine(data.getTime() + ",start," + homingFlag + "," + robotData.getBatteryLevel() + "," + 0);
                }
            }
        }
        else if(action == AgentActions.charge){
            if(robotData.getBatteryLevel() == robotData.getRobotSpec().getCapacity()){
                //finish charging
                if(homingFlag){
                    action = AgentActions.wait;
                    isChargeRequired = false;
                    waitCounter = 0;
                }
                else{
                    action = AgentActions.move;
                    isChargeRequired = false;
                    chargeLogger.writeLine(data.getTime() + ",finish," + homingFlag + "," + robotData.getBatteryLevel() + "," + 0);
                }

                // When litter amount requirement is reached
                estimator.update(expectation, searchNodes);
                if(data.getTime() > 500000 && (estimator.requirementReached() == true || homingFlag)){
                    // Target Decision
                    TargetPathAgentStatus status = new TargetPathAgentStatus(action, target, data);
                    targetter.update(status);
                    hasUpdatedTargetter = true;
                    importance.update_future(position, targetter.getNextTarget(), expectation, data.getTime());
                    waitProb = 1.0 - importance.evaluate();
                    if(rand.nextDouble() < waitProb || homingFlag){
                        action = AgentActions.wait;
                        waitCounter = 0;
                        int waitTime = 100;

                        while(true){
                            estimator.update_future(expectation, data.getTime() + waitTime, searchNodes);
                            estimationLogger.writeLine(data.getTime() + "," + estimator.getEstimatedValue() + "," + waitTime + "," + estimator.requirementReached());                            
                        
                            if(estimator.requirementReached() == false){
                                sumTime = waitTime - 100;
                                break;
                            }
                            waitTime += 100;
                        }

                        chargeLogger.writeLine(data.getTime() + ",wait,"+ "," + robotData.getBatteryLevel() + "," + sumTime);
                        waitLogger.writeLine(data.getTime() + "," + robotData.getBatteryLevel() + "," + waitProb + "," + sumTime);
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
                chargeLogger.writeLine(data.getTime() + ",unwait," + homingFlag + "," + robotData.getBatteryLevel() + "," + 0);
                
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
			if (data.getTime() > 500000 && !isChargeRequired && robotData.getBatteryLevel() < robotData.getRobotSpec().getCapacity() / 3){
                estimator.update(expectation, searchNodes);
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

    public void updateCorrection(ObservedData data){
        double realValue = environment.getLitterAmount();

        //変更
        double correction = (1.0 - alpha) * estimator.getCorrection() + alpha * (estimator.getRequirement() / realValue) * estimator.getCorrection();//correctionの更新
        
        if(correction <= 0){
            correction = 0.1;
        }
        
        correctionLogger.writeLine(data.getTime() + "," + estimator.getCorrection() + "," + correction + "," + realValue + "," + estimator.getRequirement());

        estimator.setCorrection(correction);
    }

    private void calculateCenterNode(){
        double sumPDA = 0.0;
        for(int node : searchNodes){
            sumPDA += mySpawnPattern.getAllPatterns().get(node).getProbability();
        }
        communicationDetail.myExperienceWeight = sumPDA;
        double sumX = 0.0;
        double sumY = 0.0;

        for(int node : searchNodes){
            double n = sumPDA == 0 ? 0 : mySpawnPattern.getAllPatterns().get(node).getProbability() / sumPDA;
            sumX += n * graph.getCoordinate(node).x;
            sumY += n * graph.getCoordinate(node).y;
        }

        myCenterNode.x = (int) Math.round(sumX);
        myCenterNode.y = (int) Math.round(sumY);
        communicationDetail.myCenterNode = myCenterNode;
    }

    private void calculateSearchNode(){
        searchNodes.clear();
        List<Map.Entry<Integer, Double>> array = new LinkedList<Map.Entry<Integer, Double>>();
        int i = 0;
        for(int node : nodes){
            double imp = mySpawnPattern.getAllPatterns().get(node).getProbability();
            array.add(i, new AbstractMap.SimpleEntry<Integer, Double>(node, imp));
            i++;
        }

        //降順
        //同じ値の場合はランダムに並べる
        Collections.shuffle(array);

        array.sort(new Comparator<Map.Entry<Integer, Double>>() {
            @Override
            public int compare(Map.Entry<Integer, Double> o1, Map.Entry<Integer, Double> o2){
                return o2.getValue().compareTo(o1.getValue());
            }
        });

        if(preStop){
            if(searchNodeStartIndex > 0 && searchNodeStartIndex != searchNodeNumber){
                List<Map.Entry<Integer, Double>> array_copy = new LinkedList<Map.Entry<Integer, Double>>(array);
                List<Map.Entry<Integer, Double>> sub_array = array_copy.subList(0, searchNodeStartIndex);

                //先頭から決まった個数を削除する(計画停止までにコミュニケーションで渡せるもの)
                array.removeAll(sub_array);

                //末尾にsub_arrayを追加
                array.addAll(sub_array);
            }
        }

        int count = 0;
        for(Map.Entry<Integer, Double> node : array){
            searchNodes.add(node.getKey());
            count++;
            if(count == searchNodeNumber){
                break;
            }
        }
    }

    @Override
    public void searchNumberDecrease(int decrease){
        searchNodeNumber -= decrease;
        if(searchNodeNumber > maxNodeNumber){
            searchNodeNumber = maxNodeNumber;
        }
    }

    public void visitPosition(int position){
        if(!visitedNodes.contains(position)){
            visitedNodes.add(position);
        }
    }

    // no initial value
    public void InitializeMySpawnPattern(){
        mySpawnPattern = new LitterSpawnPattern();
        for(int node : graph.getAllNode()){
            mySpawnPattern.addSpawnProbability(node, new LitterSpawnProbability(1, 0.0, 1));
        }
    }

    // with initial value
    public void InitializeMySpawnPattern(LitterSpawnPattern initPattern){
        mySpawnPattern = new LitterSpawnPattern();
        for(int node : graph.getAllNode()){
            mySpawnPattern.addSpawnProbability(node, new LitterSpawnProbability(1, initPattern.getSpawnProbability(node).getProbability(), 1));
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
    public void setEnvironment(IEnvironment env){
        environment = env;
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

    @Override
    public CommunicationDetails getCommunicationDetails(){
        return communicationDetail;
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
	public int getSearchNodeNumber(){
		return searchNodeNumber;
	}

	@Override
    public Coordinate getCenterNode(){
		return myCenterNode;
	}

    @Override
    public void setStopTime(int time){
        stopTime = time;
    }

    @Override
    public void setRestartTime(int time){
        restartTime = time;
    }

    @Override
    public void setPreStop(){
        preStop = true;
        targetter.setPreStop();
    }

    @Override
    public int getStopTime(){
        return stopTime;
    }

    @Override
    public int getRestartTime(){
        return restartTime;
    }

    @Override
    public boolean getPreStop(){
        return preStop;
    }

    @Override
    public void addStartIndex(int n){
        searchNodeStartIndex += n;
        if(searchNodeStartIndex > searchNodeNumber){
            searchNodeStartIndex = searchNodeNumber;
        }
    }

    @Override
    public void setSleepProbability(double p){
        //ダミーメソッド
    }

    @Override
    public void resetPreStop(){
        //ダミーメソッド
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

	@Override
	public double getRequirement() {
		return -1;
	}
}
