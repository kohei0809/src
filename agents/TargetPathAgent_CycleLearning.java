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
import core.Tuple;
import core.agent.AgentActions;
import core.agent.IAgent;
import core.agent.ObservedData;
import core.util.DijkstraAlgorithm;
import core.util.LogManagerContext;
import core.util.LogWriter2;
import core.util.PotentialCollection;

//cycleを学習し、それをPausingに用いる(評価指標はU(s))
public class TargetPathAgent_CycleLearning implements IAgent{
    ITargetDecider targetter;
    IPathPlanner pather;
    int baseNode, target, nextNode;
    double returnProb = 0.0, waitProb = 0.0;
    boolean isChargeRequired = false, homingFlag = false, hasUpdatedTargetter = false, pausingFlag = false;
    GridGraph graph;
    Random rand;
    IEnvironment environment;
    double alpha = 0.1, beta = 0.3;
    int scale = 50;
    int[][] agentPosition = new int[2*scale+1][2*scale+1];
    List<Integer> excludeNodes;
    int maxNode = 0;
    double targetExp = 0.0;
    int overCount = 0;
    double maxCorrection = 0.0, sumCorrection = 0.0, prior = 1.0;
    int moveCount = 0;
    
    int collectedCycleLitter = 0;
    int startTime = 0;
    boolean isCycleLearning = false;
    Tuple<Integer, Double> ccAndMce = new Tuple<Integer, Double>(0, 0.0);
    int maxCycle = 900;
    int cycleCount;
    int myCycle; //連続稼働時間
    int cycleIndex;
    int[] cycleTime = new int[3];
    double[] cycleValues = {0.0, 0.0, 0.0};
    double cycleLearnRatio = 0.1;
    double cycleLearnEpsilon = 0.05;
    boolean endLearnTime = false;
    List<Integer> maxList;

    LitterExistingExpectation expectation;
    RequirementEstimatorU estimator;
    Importance importance;
    PotentialCollection basePotential;

    private int robotID, counter, waitCounter, sumTime = 0, checkInterval = 100, cycleLearnTerm = 100000, cycleLearnEndTime = 500000;
    private AgentActions action, preaction;
    private String dir;

    LogWriter2 chargeLogger;
    LogWriter2 returnLogger;
    LogWriter2 waitLogger;
    LogWriter2 estimationLogger;
    LogWriter2 correctionLogger;
    LogWriter2 expLogger;
    LogWriter2 pausingLogger;
    LogWriter2 waitingLogger;
    LogWriter2 targetLogger;
    LogWriter2 maxTargetLogger;
    LogWriter2 cycleLogger;
    LogWriter2 maxLogger;
    LogWriter2 actionLogger;
    LogWriter2 checkLogger;
    LogWriter2 cycleLearnLogger;

    public TargetPathAgent_CycleLearning(int id, LitterSpawnPattern pattern, GridGraph graph, int seed, List<Integer> excludeNodes) {
		setRobotID(id);
		action = AgentActions.standby;
		expectation = new LitterExistingExpectation(pattern, true);
		importance = new Importance(graph);
        this.graph = graph;
        this.excludeNodes = excludeNodes;

		rand = new Random(seed);
		counter = 0;
        waitCounter = 0;

        cycleTime[0] = 150;
        cycleTime[1] = 300;
        cycleTime[2] = 900;
        cycleIndex = rand.nextInt(3);
        myCycle = cycleTime[cycleIndex];
        cycleCount = maxCycle / myCycle;
        maxList = new LinkedList<Integer>();

        for (int node : excludeNodes) {
            Coordinate c = graph.getCoordinate(node);
            int x = c.x + scale;
            int y = -c.y + scale;
            agentPosition[y][x] = -1;
        }

        dir = LogManagerContext.getLogManager().makeDir("Agent" + robotID);
        //time, start, homingFlag, battery
        chargeLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Charge");
        //time,remainBattery, px, py, returnFlag
        returnLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Return");
        //time,remainBattery
        waitLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Wait");
        //time, estimation
        estimationLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Estimate");
        //time, correction, newcorrection, realValue, req
        correctionLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Correction");
        //time, 
        //expLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Exp");
        //time, maxNode, position, action
        pausingLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Pausing");
        //time, maxNode, waittime, action
        waitingLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Waiting");
        //time, maxNode, newmaxNode, exp;
        targetLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Target");
        //time, maxNode, litter, size of list;
        maxTargetLogger = LogManagerContext.getLogManager().createWriter2(dir + "/MaxTarget");
        //time, myCycle, ccAndMce.val1, ccAndMce.val2, cycleCount, maxCycle, cycleValue[0], cycleValue[1], cycleValue[2]); 
        cycleLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Cycle");
        //time, maxValue, maxValue(real);
        maxLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Max");
        actionLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Action");
        checkLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Check");
        cycleLearnLogger = LogManagerContext.getLogManager().createWriter2(dir + "/CycleLearn");
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
        int time = data.getTime();
        int litter = robotData.getVacuumedLitter();
        int wattage = robotData.getRobotSpec().getWattage();
        preaction = action;

        collectedCycleLitter += litter;

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
                    moveCount = 0;
    		        chargeLogger.writeLine(time + ",start," + homingFlag + "," + robotData.getBatteryLevel() + "," + 0);
                    
    		        ccAndMce = new Tuple<Integer,Double>(ccAndMce.val1 + 1, ccAndMce.val2 + ((double)collectedCycleLitter / (double)(time - startTime)));
                    if(endLearnTime == true){

                    }
    		        else if(isCycleLearning == true){
    		        	if(cycleLearnEndTime <= time){
                            endLearnTime = true;
                            cycleIndex = -1;
                            double maxValue = -1.0;
                            for(int i = 0; i < cycleValues.length; i++){
                                if(cycleValues[i] >= maxValue){
                                    if(cycleValues[i] == maxValue && rand.nextDouble() < 0.5){
                                        continue;
                                    }
                                    else{
                                        cycleIndex = i;
                                        maxValue = cycleValues[i];
                                    }
                                }
                            }
                            myCycle = cycleTime[cycleIndex];
                            cycleCount = maxCycle / myCycle;
                            cycleLearnLogger.writeLine(time + "," + ccAndMce.val1 + "," + cycleCount + "," + endLearnTime + "change");

                            cycleLogger.write(time + "," + myCycle + "," + ccAndMce.val1 + "," + ccAndMce.val2 + ","  + cycleCount + "," + maxCycle + "," + robotData.getBatteryLevel() + ",end," + endLearnTime);
                            for(int i = 0; i < cycleValues.length; i++){
                                cycleLogger.write(cycleValues[i] + ",");
                            }
                            cycleLogger.writeLine("");
                        }
                        else if(ccAndMce.val1 == cycleCount){
                            cycleLearning();
                            cycleLearnLogger.writeLine(time + "," + ccAndMce.val1 + "," + cycleCount + "," + endLearnTime + "Learn");
                        }
                    }
                    else{
                        if(time > cycleLearnTerm){
                            setInitialValueToCycle();
                            isCycleLearning = true;
                            cycleIndex = rand.nextInt(3);
                            myCycle = cycleTime[cycleIndex];
                            cycleCount = maxCycle / myCycle;
                            cycleLearnLogger.writeLine(time + "," + ccAndMce.val1 + "," + cycleCount + "," + endLearnTime + "change");
                        }
                        else{
                            cycleIndex = rand.nextInt(3);
                            myCycle = cycleTime[cycleIndex];
                            cycleLearnLogger.writeLine(time + "," + ccAndMce.val1 + "," + cycleCount + "," + endLearnTime + "random");
                        }
                    }
    		        
                    cycleLogger.write(time + "," + myCycle + "," + ccAndMce.val1 + "," + ccAndMce.val2 + ","  + cycleCount + "," + maxCycle + "," + robotData.getBatteryLevel() + ",charge,");
                    for(int i = 0; i < cycleValues.length; i++){
                        cycleLogger.write(cycleValues[i] + ",");
                    }
                    cycleLogger.writeLine("");
                }
                if(pausingFlag){
                    pausingFlag = false;
                    pausingLogger.writeLine(time + "," + estimator.getMaxNode() + "," + position + "," + "arrive");
                }
                collectedCycleLitter = 0;
            }
            moveCount++;
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
                    chargeLogger.writeLine(time + ",finish," + homingFlag + "," + robotData.getBatteryLevel() + "," + 0 + "," + endLearnTime);
                    startTime = time;
                }

                // When litter amount requirement is reached
                estimator.update(expectation, position, data.getTime());
                if((cycleLearnEndTime <= time) && (estimator.requirementReached() == true || homingFlag)){
                    // Target Decision
                    TargetPathAgentStatus status = new TargetPathAgentStatus(action, target, data, myCycle);
                    targetter.update(status);
                    hasUpdatedTargetter = true;
                    importance.update_future(position, targetter.getNextTarget(), expectation, time);
                    waitProb = 1.0 - importance.evaluate();
                    if(rand.nextDouble() < waitProb || homingFlag){
                        action = AgentActions.wait;
                        waitCounter = 0;
                        //expLogger.writeLine(time + "," + estimator.getMaxNode() + "," + estimator.getEstimatedValue() + "," + waitProb + "," + estimator.getRequirement() + "," + estimator.getCorrection());
                        //waitTimeの計算
                        sumTime = calculatePausingTime(time, position, robotData);
                        return;
                    }
                }
                cycleLogger.writeLine(time + "," + myCycle + "," + ccAndMce.val1 + "," + ccAndMce.val2 + ","  + cycleCount + "," + maxCycle + "," + robotData.getBatteryLevel() + ",go," + action);
            }
        }
        else if(action == AgentActions.wait){
            waitCounter++;
            pausingFlag = true;
            //maxNodeに他のエージェントが訪問したら、再計算
            // if(expectation.getInterval(maxNode) == 0){
            //     sumTime = calculatePausingTime(time, position, robotData);
            //     waitCounter = 0;
            //     waitingLogger.writeLine(time + "," + estimator.getMaxNode() + "," + sumTime  + "," + "change");
            // }
            if(time % 100 == 0){
                waitingLogger.writeLine(time + "," + estimator.getMaxNode() + "," + sumTime  + "," + "normal");
            }
            if(waitCounter >= sumTime){
                action = AgentActions.move;
                homingFlag = false;
                waitCounter = 0;
                chargeLogger.writeLine(time + ",unwait," + homingFlag + "," + robotData.getBatteryLevel() + "," + 0);
                pausingLogger.writeLine(time + "," + estimator.getMaxNode() + "," + position + "," + "go");
                startTime = time;

                //targetの再計算
                double targetexp = estimator.getMaxTargetExp(expectation, time);
                target = estimator.getMaxNode();
                TargetPathAgentStatus status = new TargetPathAgentStatus(action, target, data, myCycle);
                targetter.update(status);
                hasUpdatedTargetter = true;
                
                if(estimator.getChange()){
                    updateCorrection(data, position);
                }
                cycleLogger.writeLine(time + "," + myCycle + "," + ccAndMce.val1 + "," + ccAndMce.val2 + ","  + cycleCount + "," + maxCycle + "," + robotData.getBatteryLevel() + ",unwait");
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
        
        if(preaction != action){
        	actionLogger.writeLine(time + "," + action + "," + preaction + "," + endLearnTime);
        }

        //Target Decision
        //Pausing後にtargetに向かっている途中で他のエージェントに訪問されたら、targetを変更
        if(pausingFlag == true && expectation.getInterval(target) == 0){
          	targetLogger.write(time + "," + target + ",");
            targetExp = estimator.getMaxTargetExp(expectation, time);
            target = estimator.getMaxNode();
            targetLogger.writeLine(target + "," + targetExp);
        }

        TargetPathAgentStatus status = new TargetPathAgentStatus(action, target, data, myCycle);
        if(!hasUpdatedTargetter){
            targetter.update(status);
        }
        
        // if(!hasUpdatedTargetter || !pausingFlag){
        //     targetter.update(status);
        // }

        // Identify agent behavior (every 100 ticks)
		if (action == AgentActions.move && counter == checkInterval) {
			counter = 0;
			// When litter amount requirement is reached
			//if (time > cycleLearnTerm && !isChargeRequired && robotData.getBatteryLevel() < robotData.getRobotSpec().getCapacity() / 3){
            int remainBattery = myCycle * wattage - (robotData.getRobotSpec().getCapacity() - robotData.getBatteryLevel());
            if ((cycleLearnEndTime <= time) && !isChargeRequired && remainBattery < myCycle/3){
				estimator.update(expectation, position, time);
                if(estimator.requirementReached() == true){
                    importance.update_future(position, targetter.getNextTarget(), expectation, time);
				    returnProb = 1.0 - importance.evaluate();
				    if (rand.nextDouble() < returnProb) {
                        homingFlag = true;

                        Coordinate pc = graph.getCoordinate(pather.getTarget());
		    		    returnLogger.writeLine(time + "," + robotData.getBatteryLevel() + "," + pc.x + "," + pc.y + "," + homingFlag + "," + remainBattery);

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
			status = new TargetPathAgentStatus(action, target, data, myCycle);
            //homingでは最短で帰る(バッテリ切れで止まることがあるから)謎
			pather.updateHoming(status);
			counter = 0; // stop counting -> no requirement estimation

			if (position == baseNode) {
				action = AgentActions.charge;
				return;
			}

        }

        else if(pausingFlag && action == AgentActions.move){//pausing後にmaxnodeに向かっている途中
            target = estimator.getMaxNode();
            status = new TargetPathAgentStatus(action, target, data, myCycle);
            //pausing後は最短で最大の目的地に向かう
            pather.updateHoming(status);    
        }

        else if(target != targetter.getNextTarget() && !isChargeRequired){
            //new target
            target = targetter.getNextTarget();

            status = new TargetPathAgentStatus(action, target, data, myCycle);
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
            status = new TargetPathAgentStatus(action, target, data, myCycle);
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

    public void updateCorrection(ObservedData data, int position){
        double realValue2 = environment.getMaxLitterAmount();
        double realValue = 0;
            
        List<Integer> nodes = graph.getAllNode();
        List<Integer> maxNodes = new LinkedList<Integer>();
        //double sum = 0.0;
        for (int node : nodes) {
            double exp = expectation.getExpectation(node);
            if(realValue < exp){
                realValue = exp;
                maxNodes.clear();
                maxNodes.add(node);
            }
            if(realValue == exp){
                maxNodes.add(node);
            }
        }
            
        target = maxNodes.get(rand.nextInt(maxNodes.size()));
        int target2 = environment.getMaxTarget();
        int size = maxNodes.size();
        int size2 = environment.getNodeListSize();
        //PotentialCollection targetPotential = new DijkstraAlgorithm(graph).execute(target, position);
        double exp = expectation.getExpectation(target, data.getTime()+basePotential.getPotential(target));
        maxLogger.writeLine(data.getTime() + "," + realValue + "," + (realValue+exp) + "," + realValue2 + "," + size + "," + size2);
        maxTargetLogger.writeLine(data.getTime() + "," + target + "," + exp + "," + size);
        //maxTargetLogger.writeLine(data.getTime() + "," + target + "," + size);
        realValue += exp;
        double correction = 0;
        int checkCount = 1;
        if(realValue == 0){
            correction = estimator.getCorrection();
        }
            
        else if(realValue <= estimator.getRequirement()){
            prior = estimator.getCorrection();
            correction = (1.0 - alpha) * estimator.getCorrection() + alpha * (estimator.getRequirement() / realValue) * estimator.getCorrection();
        }
        else {
            correction = estimator.getCorrection() - (realValue / estimator.getRequirement() - 1);            
            overCount++;

            if(overCount == checkCount){
                sumCorrection += prior;
                maxCorrection = sumCorrection / (double)overCount;
            }
            else{
                if(maxCorrection == prior){
                    maxCorrection *= 0.9;
                }
                else{
                    //maxCorrection = (1.0 - beta) * maxCorrection + beta * prior;
                    maxCorrection = prior;
                }
            }
        }
    
        if(correction <= 0.0){
            correction = 0.1;
        }
        if(overCount >= checkCount && correction > maxCorrection){
            correction = maxCorrection;
        }
            
        correctionLogger.writeLine(data.getTime() + "," + estimator.getCorrection() + "," + correction + "," + realValue + "," + estimator.getRequirement() + "," + sumCorrection + "," + maxCorrection + "," + prior);
        estimator.setCorrection(correction);
    }

    public int calculatePausingTime(int time, int position, RobotData robotData){
    	int waitTime = myCycle;
        int count = 0;
        //int checkTime = 10000;
        int checkTime = myCycle * 100;
        while(checkTime > myCycle/10){
        	while(true){
                if(count == 0){
                    estimator.update_future(expectation, position, time + waitTime, true);
                    count++;
                }
                else{
                    estimator.update_future(expectation, position, time + waitTime, false);
                }
                maxNode = estimator.getMaxNode();                       
                if(estimator.requirementReached() == false){
                    waitTime = waitTime - checkTime;
                    if(waitTime < 0){
                    	waitTime = 0;
                    }
                    checkLogger.writeLine(time + "," + waitTime + "," + checkTime + "," + false);
                    break;
                }
                else{
                	checkLogger.writeLine(time + "," + waitTime + "," + checkTime + "," + true);
                }
                waitTime += checkTime;
            }
        	checkTime /= 10;
        }
        
        estimationLogger.writeLine(time + "," + maxNode + "," + estimator.getEstimatedValue() + "," + waitTime + "," + estimator.requirementReached() + "," + estimator.getCorrection());                            
        chargeLogger.writeLine(time + ",wait,"+ "," + robotData.getBatteryLevel() + "," + waitTime);
        waitLogger.writeLine(time + "," + robotData.getBatteryLevel() + "," + waitProb + "," + waitTime);
        return waitTime;
    }

    public void setInitialValueToCycle(){
        double averageCollectedEvent = ccAndMce.val2 / ccAndMce.val1;

        for(int i = 0; i < cycleValues.length; i++){
            cycleValues[i] = averageCollectedEvent;
        }

        ccAndMce = new Tuple<Integer, Double>(0, 0.0);
    }

    public void cycleLearning(){
        double averageCollectedEvent = ccAndMce.val2 / ccAndMce.val1;
        cycleValues[cycleIndex] = (1 - cycleLearnRatio) * cycleValues[cycleIndex] + cycleLearnRatio * averageCollectedEvent;
        if(rand.nextDouble() < cycleLearnEpsilon){
            cycleIndex = rand.nextInt(3);
        }
        else{
            cycleIndex = -1;
            double maxValue = -1.0;
            maxList.clear();
            for(int i = 0; i < cycleValues.length; i++){
                if(cycleValues[i] >= maxValue){
                    if(cycleValues[i] == maxValue){
                        maxList.add(i);
                    }
                    else{
                        cycleIndex = i;
                        maxValue = cycleValues[i];
                        maxList.clear();
                        maxList.add(i);
                    }
                }
            }
            if(maxList.size() > 1){
                cycleIndex = maxList.get(rand.nextInt(maxList.size()));
            }
        }

        myCycle = cycleTime[cycleIndex];
        
        cycleCount = maxCycle / myCycle;
        ccAndMce = new Tuple<Integer,Double>(0, 0.0);
    }

    @Override
    public void restart(ObservedData data){
        int time = data.getTime();
        RobotData robotData = data.getRobotDataCollection().getRobotData(robotID);
        int position = robotData.getPosition();

        action = AgentActions.move;
        homingFlag = false;
        waitCounter = 0;
    
        chargeLogger.writeLine(time + ",restart," + homingFlag + "," + robotData.getBatteryLevel() + "," + 0);
        pausingLogger.writeLine(time + "," + estimator.getMaxNode() + "," + position + "," + "restart");

        target = environment.getMaxTarget();
    }

    @Override
    public void setRobotID(int id){
        robotID = id;
    }

    @Override
    public void setBaseNode(int base){
        baseNode = base;
        basePotential = new DijkstraAlgorithm(graph).execute(baseNode);
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
    public int getCycleIndex(){
        return cycleIndex;
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
    public double getCorrection(){
        return estimator.getCorrection();
    }

    @Override
	public void setEnvironmentEstimator(RequirementEstimator estimator) {

	}

    @Override
    public void setEnvironmentEstimatorU(RequirementEstimatorU estimator){
        this.estimator = estimator;
        estimator.setPotentialMap(baseNode);
    }

    @Override
    public boolean requirementReached(){
        return false;
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
}


