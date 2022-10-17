package test.TTest;

import java.util.AbstractMap;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import core.CommunicationDetails;
import core.Coordinate;
import core.GridGraph;
import core.IAgentManager;
import core.IEnvironment;
import core.LitterSpawnPattern;
import core.RobotData;
import core.RobotDataCollection;
import core.agent.AgentActions;
import core.agent.IAgent;
import core.agent.ObservedData;
import core.util.DijkstraAlgorithm;
import core.util.LogManagerContext;
import core.util.LogWriter2;
import core.util.PotentialCollection;
import main.simulations.BehaviorType;

public class TTestAgentManager implements IAgentManager{
    IEnvironment environment;
    BehaviorType behaviorType; // controlled by manager or autonomous
    List<Map.Entry<IAgent, Integer>> agents; //<agent, robotID>
    Map<Integer, AgentActions> agentActions; //previous actions before update
    List<Integer> excludeNodes;
    Random rand;
    GridGraph graph;
    int[][] communicationMemory = new int[20][20];
    final int communicationInterval = 10800; // 同ロボットとの通信間隔(step) 10800-12h
    final int communicationRange = 5; // 通信半径(ノード=m)
    int[] potentialCenterNodeMemory ={-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    PotentialCollection[] agentsPotential = new PotentialCollection[20];
    
    int[][] agentPosition;
    
    int stop_index = 0;
    int max_stop_number = 1;

    boolean isStop = true;
    int stop_time;
    int stop_robots_number;
    int restart_time;

    int[] communicationTimesMemory = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //復帰からの他のエージェントとの実際の交渉回数
    int[] restart_time_before = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //以前の停止時間

    List<Integer> agent_number_list;

    //ログ出力用
    int[] accumulatedLitters = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    int[] preAccumulatedLitters = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    LogWriter2 accumulatedLitter; //各エージェントの各ステップでのごみ回収量
    LogWriter2 strategyLogger; //各エージェントがどの戦略をとっているか
    LogWriter2 difValueLogger;
    LogWriter2 negoLoggerBT;
    LogWriter2 negoLoggerTO;
    LogWriter2 negoLoggerCT;
    LogWriter2 searchNode;
    LogWriter2 stopAgents;
    LogWriter2 restartAgents;
    LogWriter2 giveAmountAveLogger;
    LogWriter2 remainValueLogger;
    

    public TTestAgentManager(IEnvironment environment, BehaviorType behavior, int seed, GridGraph graph){
        this.environment = environment;
        behaviorType = behavior;
        agents = new LinkedList<Map.Entry<IAgent, Integer>>();
        agentActions = new HashMap<Integer, AgentActions>();
        rand = new Random(seed);
        this.graph = graph;

        String[] lavels = new String[21];
        for(int i = 0; i < 21; i++){
            if(i > 0){
                lavels[i] = Integer.toString(i - 1);
            }
            else if(i == 0){
                lavels[i] = "";
            }
        }
        
        agent_number_list = new LinkedList<Integer>();
        for(int i = 0; i < 20; i++) {
        	agent_number_list.add(i);
        }
        Collections.shuffle(agent_number_list);

        //マルチスレッド用
        accumulatedLitter = LogManagerContext.getLogManager().createWriter2("AccumulatedLitter");
        accumulatedLitter.writeLineBlock(lavels);
        strategyLogger = LogManagerContext.getLogManager().createWriter2("StrategyDistribution");
        strategyLogger.writeLine("time,R,PGS,LI,BNPS\n");
        difValueLogger = LogManagerContext.getLogManager().createWriter2("DifValueLogger");
        negoLoggerBT = LogManagerContext.getLogManager().createWriter2("NegoLoggerBT");	//Negotiation For Balancing Tasks
        negoLoggerTO = LogManagerContext.getLogManager().createWriter2("NegoLoggerTO"); //negotiation For Trade-Off Responsibility
        negoLoggerCT = LogManagerContext.getLogManager().createWriter2("NegoLoggerCT"); //Negotiation for Committing Tasks
        stopAgents = LogManagerContext.getLogManager().createWriter2("StopAgents");
        restartAgents = LogManagerContext.getLogManager().createWriter2("RestartAgents");
        giveAmountAveLogger = LogManagerContext.getLogManager().createWriter2("GiveAmountAveLogger");
        remainValueLogger = LogManagerContext.getLogManager().createWriter2("RemainValueLogger");
    }

    public void addAgent(IAgent agent){
        agents.add(new AbstractMap.SimpleEntry<IAgent, Integer>(agent, agent.getRobotID()));
        agentActions.put(agent.getRobotID(), AgentActions.standby);
        agent.setEnvironment(environment);
    }

    public void move(){
        switch(behaviorType){
            case normal:
            default:
                normal();
                break;
            case communicable:
                communicable();
                //エージェントの場所と担当ノードの図示
                //printAgentInformation(100000);
                //エージェントの担当ノードの数と中心の記録
                //printSearchNodeNumber(3600);
                //エージェントのマップ全体の重要度を記録
                //printAgentImportance(3600);
                break;
            case plannedStoppable:
                plannedStoppable();
                //エージェントの場所と担当ノードの図示
                printAgentInformation(100000);
                //エージェントの担当ノードの数と中心の記録
                printSearchNodeNumber(3600);
                //エージェントのマップ全体の重要度を記録
                printAgentImportance(3600);
                break;
            case randomStoppable:
                randomStoppable();
                break;
            case descendingStoppable:
                descendingStoppable();
                break;
        }

        printAgentImportance(100000);
        printAccumulatedLitter(10000);
        printAgentEvaluation(10000);
        //printAgentPositionCount(1000000);
    }

    //Communicationを一切しない
    public void normal(){
        RobotDataCollection robots = environment.getRobotDataCollection();
        int time = environment.getTime();

        ObservedData data = new ObservedData(time, robots);

        for(Map.Entry<IAgent, Integer> pair : agents){
            IAgent agent = pair.getKey();
            int id = pair.getValue();

            //エージェントが停止していなければ
            if(agentActions.get(id) != AgentActions.stop){
                agent.update(data);
                //このステップから充電を止めてmoveになるとき
                if(agent.getAction() == AgentActions.move){
                    if(agentActions.get(id) == AgentActions.charge || agentActions.get(id) == AgentActions.wait){
                        environment.disconnectRobotBase(id);
                    }
                    environment.moveRobot(id, agent.getNextNode());
                }
                //このステップっから充電開始のとき
                else if(agent.getAction() == AgentActions.charge){
                    if(agentActions.get(id) != AgentActions.charge){
                        environment.connectRobotBase(id);
                    }
                }
                agentActions.put(id, agent.getAction());
            }
        }
    }

    public void communicable(){
        RobotDataCollection robots = environment.getRobotDataCollection();
        int time = environment.getTime();

        //エージェントの停止の設定
        boolean isStop = false;
        stop_time = 3000000;
        stop_robots_number = 5;
        restart_time = Integer.MAX_VALUE;

        if(isStop) {
        	//エージェントの停止
            AgentStop(time, stop_time, restart_time, stop_robots_number);

            //降順にエージェントの停止
            //descendingAgentStop(time, stop_time, restart_time, stop_robots_number);
        }
        
        ObservedData data = new ObservedData(time, robots);

        for(Map.Entry<IAgent, Integer> pair : agents){
            IAgent agent = pair.getKey();
            int id = pair.getValue();

            //エージェントが停止していなければ
            if(agentActions.get(id) != AgentActions.stop){
                agent.update(data);                
                if(agent.getAction() == AgentActions.move){
                    communication(id, data);
                    
                    if(agentActions.get(id) == AgentActions.charge  || agentActions.get(id) == AgentActions.wait){
                        environment.disconnectRobotBase(id);
                    }
                    environment.moveRobot(id, agent.getNextNode());
                }
                else if(agent.getAction() == AgentActions.charge){
                    if(agentActions.get(id) != AgentActions.charge){
                        environment.connectRobotBase(id);
                    }
                }
                agentActions.put(id, agent.getAction());
            }
        }
    }

    public void plannedStoppable(){
        RobotDataCollection robots = environment.getRobotDataCollection();
        int time = environment.getTime();

        //エージェントの停止の設定
        stop_time = 3000000;
        stop_robots_number = 5;
        restart_time = Integer.MAX_VALUE;
        int reservation_time = stop_time - 5000;
        
        // エージェントの停止予約
		if (time == reservation_time){
            AgentStopReservation(stop_time, stop_robots_number, restart_time);
        }

        // エージェントの計画停止と再開
        AgentStopForPlannedStop(time);
        
        ObservedData data = new ObservedData(time, robots);

        for(Map.Entry<IAgent, Integer> pair : agents){
            IAgent agent = pair.getKey();
            int id = pair.getValue();

            //エージェントが停止していなければ
            if(agentActions.get(id) != AgentActions.stop){
                agent.update(data);                
                if(agent.getAction() == AgentActions.move){
                    communicationForPlannedStop(id, data, time);
                    
                    if(agentActions.get(id) == AgentActions.charge){
                        environment.disconnectRobotBase(id);
                    }
                    environment.moveRobot(id, agent.getNextNode());
                }
                else if(agent.getAction() == AgentActions.charge){
                    if(agentActions.get(id) != AgentActions.charge){
                        environment.connectRobotBase(id);
                    }
                }
                agentActions.put(id, agent.getAction());
            }
        }
    }

    //エージェント間のコミュニケーション
    private void communication(int id, ObservedData data){
        Coordinate myCoordinate = graph.getCoordinate(data.getRobotDataCollection().getRobotData(id).getPosition());
        CommunicationDetails me = agents.get(id).getKey().getCommunicationDetails();
        int myCenterNodeNumber = graph.getNode(me.myCenterNode);
        LitterSpawnPattern mySpawnPattern = me.mySpawnPattern;
        double myExperienceWeight = me.myExperienceWeight;
        List<Integer> searchN = me.searchNodes;

        for(RobotData robot : data.getRobotDataCollection().getAllRobotData()){
            int partnerID = robot.getID();

            //通信条件判定
            if(partnerID != id && agentActions.get(partnerID) != AgentActions.stop && (data.getTime() - communicationMemory[id][partnerID]) > communicationInterval){
                Coordinate coo = graph.getCoordinate(robot.getPosition());
                int distance = Math.abs(myCoordinate.x - coo.x) + Math.abs(myCoordinate.y - coo.y);

                //通信範囲判定
                if(distance <= communicationRange){
                    CommunicationDetails partner = agents.get(partnerID).getKey().getCommunicationDetails();
                    int partnerCenterNodeNumber = graph.getNode(partner.myCenterNode);
                    LitterSpawnPattern partnerSpawnPattern = partner.mySpawnPattern;
                    double partnerExperienceWeight = partner.myExperienceWeight;

                    //重心が変更されていた場合，重心からのポテンシャルマップを取得する
                    if(myCenterNodeNumber != potentialCenterNodeMemory[id]){
                        agentsPotential[id] = new DijkstraAlgorithm(graph).execute(myCenterNodeNumber);
                        potentialCenterNodeMemory[id] = myCenterNodeNumber;
                    }
                    if(partnerCenterNodeNumber != potentialCenterNodeMemory[partnerID]){
                        agentsPotential[partnerID] = new DijkstraAlgorithm(graph).execute(partnerCenterNodeNumber);
                        potentialCenterNodeMemory[partnerID] = partnerCenterNodeNumber;
                    }

                    //責任の総和の比較
                    int counter = searchN.size() - 1;
                    double difValue;

                    if(partnerExperienceWeight != 0){//0乗算の例外処理
                        double diffRatio = myExperienceWeight / partnerExperienceWeight;
                        difValueLogger.writeLine(data.getTime() + "," + id + "," + partnerID + "," + diffRatio);
                        difValue = (diffRatio > 10.0) ? 10.0 : diffRatio;
                    }
                    else{
                        difValue = 10.0;
                    }

                    //責任を公平にする交渉
                    if(difValue > 1.05){
                        if(difValue > 10.0){
                            difValue = 10.0;
                        }
                        int decrease = (int)Math.floor(difValue * 10);
                        if(decrease > counter - 1){
                            decrease = counter - 1;
                        }
                        int l = 0;
                        List<Integer> removeList = new LinkedList<Integer>();

                        while(l < decrease && counter > 1){
                            int target = searchN.get(counter);
                            if(agentsPotential[id].getPotential(target) > agentsPotential[partnerID].getPotential(target)){
                                double givingHalf = mySpawnPattern.getAllPatterns().get(target).getProbability() * 0.5;
                                partnerSpawnPattern.getAllPatterns().get(target).setProbability(partnerSpawnPattern.getAllPatterns().get(target).getProbability() + givingHalf);
                                mySpawnPattern.getAllPatterns().get(target).setProbability(givingHalf);
                                removeList.add(target);
                                l++;
                            }
                            counter--;
                        }
                        searchN.removeAll(removeList);
                        agents.get(id).getKey().searchNumberDecrease(l);
                        agents.get(partnerID).getKey().searchNumberDecrease(-l);

                        negoLoggerBT.writeLine(data.getTime() + "," + id + "," + partnerID + "," + l);
                    }

                    //責任ノードを改善する交渉
                    else if(difValue > 0.95){
                        int changeCount = 0;
                        int searchNCount = 0;
                        List<Integer> removeList2 = new LinkedList<Integer>();

                        while(changeCount < 5 && searchNCount < counter - 1){
                            int changeTarget = searchN.get(searchNCount);

                            if(agentsPotential[id].getPotential(changeTarget) > agentsPotential[partnerID].getPotential(changeTarget)){
                                double givingHalf = mySpawnPattern.getAllPatterns().get(changeTarget).getProbability() * 0.5;
                                partnerSpawnPattern.getAllPatterns().get(changeTarget).setProbability(partnerSpawnPattern.getAllPatterns().get(changeTarget).getProbability() + givingHalf);
                                mySpawnPattern.getAllPatterns().get(changeTarget).setProbability(givingHalf);
                                removeList2.add(changeTarget);
                                changeCount++;
                            }
                            searchNCount++;
                        }
                        searchN.removeAll(removeList2);
                        agents.get(id).getKey().searchNumberDecrease(changeCount);
                        agents.get(partnerID).getKey().searchNumberDecrease(-changeCount);

                        negoLoggerTO.writeLine(data.getTime() + "," + id + "," + partnerID + "," + changeCount);
                    }
                    communicationMemory[id][partnerID] = data.getTime();
                }
            }
        }
    }

    private void communicationForPlannedStop(int id, ObservedData data, int time){
        Coordinate myCoordinate = graph.getCoordinate(data.getRobotDataCollection().getRobotData(id).getPosition());
        IAgent myAgent = agents.get(id).getKey();
        CommunicationDetails me = agents.get(id).getKey().getCommunicationDetails();
        int myCenterNodeNumber = graph.getNode(me.myCenterNode);
        LitterSpawnPattern mySpawnPattern = me.mySpawnPattern;
        double myExperienceWeight = me.myExperienceWeight;
        List<Integer> searchN = me.searchNodes;

        for(RobotData robot : data.getRobotDataCollection().getAllRobotData()){
            int partnerID = robot.getID();

            //通信条件判定
            if(partnerID != id && agentActions.get(partnerID) != AgentActions.stop && (data.getTime() - communicationMemory[id][partnerID]) > communicationInterval){
                Coordinate coo = graph.getCoordinate(robot.getPosition());
                int distance = Math.abs(myCoordinate.x - coo.x) + Math.abs(myCoordinate.y - coo.y);

                //通信範囲判定
                if(distance <= communicationRange){
                    IAgent partnerAgent = agents.get(partnerID).getKey();
                    CommunicationDetails partner = agents.get(partnerID).getKey().getCommunicationDetails();
                    int partnerCenterNodeNumber = graph.getNode(partner.myCenterNode);
                    LitterSpawnPattern partnerSpawnPattern = partner.mySpawnPattern;
                    double partnerExperienceWeight = partner.myExperienceWeight;

                    //重心が変更されていた場合，重心からのポテンシャルマップを取得する
                    if(myCenterNodeNumber != potentialCenterNodeMemory[id]){
                        agentsPotential[id] = new DijkstraAlgorithm(graph).execute(myCenterNodeNumber);
                        potentialCenterNodeMemory[id] = myCenterNodeNumber;
                    }
                    if(partnerCenterNodeNumber != potentialCenterNodeMemory[partnerID]){
                        agentsPotential[partnerID] = new DijkstraAlgorithm(graph).execute(partnerCenterNodeNumber);
                        potentialCenterNodeMemory[partnerID] = partnerCenterNodeNumber;
                    }

                    //責任の総和の比較
                    int counter = searchN.size() - 1;
                    double difValue;

                    if(partnerExperienceWeight != 0){//0乗算の例外処理
                        double diffRatio = myExperienceWeight / partnerExperienceWeight;
                        difValueLogger.writeLine(data.getTime() + "," + id + "," + partnerID + "," + diffRatio);
                        difValue = (diffRatio > 10.0) ? 10.0 : diffRatio;
                    }
                    else{
                        difValue = 10.0;
                    }

                    //停止時間より前の時
                    if(time < myAgent.getStopTime()){
                        //自分が計画停止をするエージェントで，相手が計画停止をしないエージェントの場合
                        if(myAgent.getPreStop() && !partnerAgent.getPreStop()){
                            int remain_time = myAgent.getStopTime() - time;//計画停止までの残り時間
                            int changeCount = 0;
                            int searchNCount = 0;
                            List<Integer>removeList3 = new LinkedList<Integer>();

                            //繰り上げ
                            double remain_neg_number = (double)communicationTimesMemory[id] 
                                    / (double) (time - restart_time_before[id]) * (double) remain_time 
                                    * ((double) stop_robots_number/ 19);
                            remainValueLogger.writeLine(data.getTime() + "," + id + "," + partnerID + ","
									+ communicationTimesMemory[id] + "," + restart_time_before[id] + "," +
									remain_time + "," + stop_robots_number + "," + stop_index);

                            double giveamount_ave = (double) searchN.size() / remain_neg_number;
                            giveAmountAveLogger.writeLine(data.getTime() + "," + id + "," + partnerID + "," + giveamount_ave);
                            int decrease = (int) Math.floor(giveamount_ave);
                            
                            //上限なし
                            /*if(decrease > 500){
                                decrease = 500;
                            }*/

                            while(changeCount < decrease && searchNCount < counter){
                                int target = searchN.get(searchNCount);
                                double givingValue = mySpawnPattern.getAllPatterns().get(target).getProbability() * 0.5;
                                partnerSpawnPattern.getAllPatterns().get(target).setProbability(partnerSpawnPattern.getAllPatterns().get(target).getProbability() + givingValue);
                                mySpawnPattern.getAllPatterns().get(target).setProbability(mySpawnPattern.getAllPatterns().get(target).getProbability() - givingValue);
                                removeList3.add(target);
                                changeCount++;
                                searchNCount++;
                            }

                            searchN.removeAll(removeList3);
                            agents.get(id).getKey().searchNumberDecrease(changeCount);
                            agents.get(partnerID).getKey().searchNumberDecrease(-changeCount);

                            negoLoggerCT.writeLine(data.getTime() + "," + id + "," + partnerID + "," + changeCount);
                        }

                        //自分が計画停止をしないエージェントで，相手が計画停止をするエージェントの場合
                        else if(!myAgent.getPreStop() && partnerAgent.getPreStop()){
                            //何もしない
                        }
                        else{
                            //責任を公平にする交渉
                            if(difValue > 1.05){
                                if(difValue > 10.0){
                                    difValue = 10.0;
                                }
                                int decrease = (int)Math.floor(difValue * 10);
                                if(decrease > counter - 1){
                                    decrease = counter - 1;
                                }
                                int l = 0;
                                List<Integer> removeList = new LinkedList<Integer>();

                                while(l < decrease && counter > 1){
                                    int target = searchN.get(counter);
                                    if(agentsPotential[id].getPotential(target) > agentsPotential[partnerID].getPotential(target)){
                                        double givingHalf = mySpawnPattern.getAllPatterns().get(target).getProbability() * 0.5;
                                        partnerSpawnPattern.getAllPatterns().get(target).setProbability(partnerSpawnPattern.getAllPatterns().get(target).getProbability() + givingHalf);
                                        mySpawnPattern.getAllPatterns().get(target).setProbability(givingHalf);
                                        removeList.add(target);
                                        l++;
                                    }
                                    counter--;
                                }
                                searchN.removeAll(removeList);
                                agents.get(id).getKey().searchNumberDecrease(l);
                                agents.get(partnerID).getKey().searchNumberDecrease(-l);

                                negoLoggerBT.writeLine(data.getTime() + "," + id + "," + partnerID + "," + l);
                            }

                            //責任ノードを改善する交渉
                            else if(difValue > 0.95){
                                int changeCount = 0;
                                int searchNCount = 0;
                                List<Integer> removeList2 = new LinkedList<Integer>();

                                while(changeCount < 5 && searchNCount < counter - 1){
                                    int changeTarget = searchN.get(searchNCount);

                                    if(agentsPotential[id].getPotential(changeTarget) > agentsPotential[partnerID].getPotential(changeTarget)){
                                        double givingHalf = mySpawnPattern.getAllPatterns().get(changeTarget).getProbability() * 0.5;
                                        partnerSpawnPattern.getAllPatterns().get(changeTarget).setProbability(partnerSpawnPattern.getAllPatterns().get(changeTarget).getProbability() + givingHalf);
                                        mySpawnPattern.getAllPatterns().get(changeTarget).setProbability(givingHalf);
                                        removeList2.add(changeTarget);
                                        changeCount++;
                                    }
                                    searchNCount++;
                                }
                                searchN.removeAll(removeList2);
                                agents.get(id).getKey().searchNumberDecrease(changeCount);
                                agents.get(partnerID).getKey().searchNumberDecrease(-changeCount);

                                negoLoggerTO.writeLine(data.getTime() + "," + id + "," + partnerID + "," + changeCount);
                            }
                        }
                    }

                    else{
                        //責任を公平にする交渉
                        if(difValue > 1.05){
                            if(difValue > 10.0){
                                difValue = 10.0;
                            }
                            int decrease = (int)Math.floor(difValue * 10);
                            if(decrease > counter - 1){
                                decrease = counter - 1;
                            }
                            int l = 0;
                            List<Integer> removeList = new LinkedList<Integer>();

                            while(l < decrease && counter > 1){
                                int target = searchN.get(counter);
                                if(agentsPotential[id].getPotential(target) > agentsPotential[partnerID].getPotential(target)){
                                    double givingHalf = mySpawnPattern.getAllPatterns().get(target).getProbability() * 0.5;
                                    partnerSpawnPattern.getAllPatterns().get(target).setProbability(partnerSpawnPattern.getAllPatterns().get(target).getProbability() + givingHalf);
                                    mySpawnPattern.getAllPatterns().get(target).setProbability(givingHalf);
                                    removeList.add(target);
                                    l++;
                                }
                                counter--;
                            }
                            searchN.removeAll(removeList);
                            agents.get(id).getKey().searchNumberDecrease(l);
                            agents.get(partnerID).getKey().searchNumberDecrease(-l);

                            negoLoggerBT.writeLine(data.getTime() + "," + id + "," + partnerID + "," + l);
                        }

                        //責任ノードを改善する交渉
                        else if(difValue > 0.95){
                            int changeCount = 0;
                            int searchNCount = 0;
                            List<Integer> removeList2 = new LinkedList<Integer>();

                            while(changeCount < 5 && searchNCount < counter - 1){
                                int changeTarget = searchN.get(searchNCount);

                                if(agentsPotential[id].getPotential(changeTarget) > agentsPotential[partnerID].getPotential(changeTarget)){
                                    double givingHalf = mySpawnPattern.getAllPatterns().get(changeTarget).getProbability() * 0.5;
                                    partnerSpawnPattern.getAllPatterns().get(changeTarget).setProbability(partnerSpawnPattern.getAllPatterns().get(changeTarget).getProbability() + givingHalf);
                                    mySpawnPattern.getAllPatterns().get(changeTarget).setProbability(givingHalf);
                                    removeList2.add(changeTarget);
                                    changeCount++;
                                }
                                searchNCount++;
                            }
                            searchN.removeAll(removeList2);
                            agents.get(id).getKey().searchNumberDecrease(changeCount);
                            agents.get(partnerID).getKey().searchNumberDecrease(-changeCount);

                            negoLoggerTO.writeLine(data.getTime() + "," + id + "," + partnerID + "," + changeCount);
                        }
                    }
                    
                    communicationMemory[id][partnerID] = data.getTime();
                    communicationTimesMemory[id]++;
                }
            }
        }
    }

    //エージェントの停止と再開予約(Kの降順)
    private void AgentStopReservation(int stop_time, int stop_robots_number, int restart_time){
    	System.out.println("Agents will stop at " + stop_time + "ticks!");

        List<Integer> rnd = new LinkedList<Integer>();
        List<Map.Entry<Integer, Double>> array = new LinkedList<Map.Entry<Integer, Double>>();
        //<robotID, correction factor>
        int n = 0;
        for(Map.Entry<IAgent, Integer> pair : agents){
            IAgent agent = pair.getKey();
            int agentID = agent.getRobotID();

            rnd.add(agentID);
            array.add(n, new AbstractMap.SimpleEntry<Integer, Double>(agentID, agent.getCorrection()));
            n++;
        }

        //降順
        array.sort(new Comparator<Map.Entry<Integer, Double>>() {
            @Override
            public int compare(Map.Entry<Integer, Double> o1, Map.Entry<Integer, Double> o2){
                return o2.getValue().compareTo(o1.getValue());
            }
        });

        for(int i=0; i<stop_robots_number; i++) {
            double value = array.get(i).getValue();
            int delete_id = array.get(i).getKey();

            for(Map.Entry<IAgent, Integer> pair : agents) {
    			int id = pair.getValue();
    			if(id == delete_id){
    				IAgent stopAgent = pair.getKey();
    				stopAgent.setStopTime(stop_time);
    				stopAgent.setRestartTime(restart_time);
                    stopAgent.setPreStop();
    				
    				System.out.println("Agent" + delete_id + " will stop at " + stopAgent.getStopTime() + "ticks!");
    				System.out.println("Agent" + delete_id + " will restart at " + stopAgent.getRestartTime() + "ticks!");
    			}
    		}
            
            rnd.remove(rnd.indexOf(delete_id));
            //ログ記録
            stopAgents.writeLine(delete_id + "," + agentActions.get(delete_id) + "," + value);
        }
    	
    	for(int i = 0; i < stop_robots_number; i++) {
    		int delete_id = agent_number_list.get(0);
    		for(Map.Entry<IAgent, Integer> pair : agents) {
    			int id = pair.getValue();
    			if(id == delete_id){
    				IAgent stopAgent = pair.getKey();
    				stopAgent.setStopTime(stop_time);
    				stopAgent.setRestartTime(restart_time);
                    stopAgent.setPreStop();
    				
    				System.out.println("Agent" + delete_id + " will stop at " + stopAgent.getStopTime() + "ticks!");
    				System.out.println("Agent" + delete_id + " will restart at " + stopAgent.getRestartTime() + "ticks!");
    			}
    		}
    	}
    	stop_index++;
    }
    
    //エージェントの途中停止と再開
    private void AgentStopForPlannedStop(int time){
    	//予約時間になったらエージェントを停止
    	for(Map.Entry<IAgent, Integer> pair : agents){
    		IAgent agent = pair.getKey();
    		int agent_id = pair.getValue();
    		
    		if(agent.getStopTime() > 0 && time == agent.getStopTime()) {
    			agentActions.put(agent_id, AgentActions.stop);
    			agent.resetPreStop();
    			stopAgents.writeLine(time + "," + agent_id + "," + agentActions.get(agent_id));
    			System.out.println("Agent" + agent_id + " is stopped at " + time + "ticks!");
    		}
    	}
    	
    	//予約時間になったらエージェントを再開
    	for(Map.Entry<IAgent, Integer> pair : agents) {
    		IAgent agent = pair.getKey();
    		int agent_id = pair.getValue();
    		
    		if(agent.getRestartTime() > 0 && time == agent.getRestartTime() && agentActions.get(agent_id) == AgentActions.stop){
    			agentActions.put(agent_id, agent.getAction());
    			restartAgents.writeLine(time + "," + agent_id + "," + agentActions.get(agent_id));
    			System.out.println("Agent" + agent_id + " is restarted at " + time + "ticks!");

                for(int i = 0; i < 20; i++){
                    restart_time_before[i] = time;
                    communicationTimesMemory[i] = 0;
                }
    		}
    	}
    }
    
    public void randomStoppable(){
        RobotDataCollection robots = environment.getRobotDataCollection();
        int time = environment.getTime();
        if(stop_index >= max_stop_number){
            stop_index = max_stop_number - 1;
        }
        
        //エージェントの停止の設定
        int stop_time = 3000000;
        int stop_robots_number = 5;
        int restart_time = Integer.MAX_VALUE;

        //エージェントの停止
        AgentStop(time, stop_time, restart_time, stop_robots_number);

        ObservedData data = new ObservedData(time, robots);

        for(Map.Entry<IAgent, Integer> pair : agents){
            IAgent agent = pair.getKey();
            int id = pair.getValue();

            //エージェントが停止していなければ
            if(agentActions.get(id) != AgentActions.stop){
                agent.update(data);
                //このステップから充電を止めてmoveになるとき
                if(agent.getAction() == AgentActions.move){
                    if(agentActions.get(id) == AgentActions.charge || agentActions.get(id) == AgentActions.wait){
                        environment.disconnectRobotBase(id);
                    }
                    environment.moveRobot(id, agent.getNextNode());
                }
                //このステップっから充電開始のとき
                else if(agent.getAction() == AgentActions.charge){
                    if(agentActions.get(id) != AgentActions.charge){
                        environment.connectRobotBase(id);
                    }
                }
                agentActions.put(id, agent.getAction());
            }            
        }
    }
    
    public void descendingStoppable(){
        RobotDataCollection robots = environment.getRobotDataCollection();
        int time = environment.getTime();
        if(stop_index >= max_stop_number){
            stop_index = max_stop_number - 1;
        }
        
        //エージェントの停止の設定
        int stop_time = 3000000;
        int stop_robots_number = 5;
        int restart_time = Integer.MAX_VALUE;

        //降順にエージェントの停止
        descendingAgentStop(time, stop_time, restart_time, stop_robots_number);

        ObservedData data = new ObservedData(time, robots);

        for(Map.Entry<IAgent, Integer> pair : agents){
            IAgent agent = pair.getKey();
            int id = pair.getValue();

            //エージェントが停止していなければ
            if(agentActions.get(id) != AgentActions.stop){
                agent.update(data);
                //このステップから充電を止めてmoveになるとき
                if(agent.getAction() == AgentActions.move){
                    if(agentActions.get(id) == AgentActions.charge || agentActions.get(id) == AgentActions.wait){
                        environment.disconnectRobotBase(id);
                    }
                    environment.moveRobot(id, agent.getNextNode());
                }
                //このステップっから充電開始のとき
                else if(agent.getAction() == AgentActions.charge){
                    if(agentActions.get(id) != AgentActions.charge){
                        environment.connectRobotBase(id);
                    }
                }
                agentActions.put(id, agent.getAction());
            }            
        }
    }
    
    public void clean(){
        for(Map.Entry<IAgent, Integer> robot : agents){
            int id = robot.getKey().getRobotID();
            if(agentActions.get(id) != AgentActions.stop){
                environment.clean(id);
            }
        }
    }

    public void setExcludingNodes(List<Integer> exclude){
        excludeNodes = exclude;
    }
    
  //エージェントの途中停止と再開
  	private void AgentStop(int time, int stop_time, int restart_time, int stop_robots_number) {
  		if(time == stop_time) {
  			System.out.println("Agents stop at " + stop_time + "ticks!");

  			List<Integer> rnd = new LinkedList<Integer>();
  			for(int i=0; i<20; i++)
  				rnd.add(i);

  			for(int i=0; i<stop_robots_number; i++) {
  				int value = rand.nextInt(rnd.size());
  				int delete_id = rnd.get(value);

  				agentActions.put(delete_id, AgentActions.stop);

  				rnd.remove(rnd.indexOf(delete_id));
  				//ログ記録
  				stopAgents.writeLine(delete_id + "," + agentActions.get(delete_id));
  				System.out.println("Agent" + delete_id + " is stopped.");
  			}
  		}

  		if(time == restart_time) {
  			System.out.println("Agents restart at " + restart_time + "ticks!");

  			for(Map.Entry<Integer, AgentActions> entry : agentActions.entrySet()) {
  				int agentId = entry.getKey();
  				AgentActions agentAction = entry.getValue();

  				if(agentAction == AgentActions.stop) {
  					for (Map.Entry<IAgent, Integer> pair : agents) {
  						if(pair.getValue().intValue() == agentId) {
  							IAgent restartAgent = pair.getKey();
  							AgentActions nextAction = restartAgent.getAction();
  							agentActions.put(agentId, nextAction);
  							//ログ記録
  							restartAgents.writeLine(agentId + "," + agentActions.get(agentId));
  							System.out.println("Agent" + agentId + " is restarted.");
  						}
  					}
  				}
  			}

  		}
  	}
  	
  //降降順にエージェントの途中停止と再開
  	private void descendingAgentStop(int time, int stop_time, int restart_time, int stop_robots_number) {
        if(time == stop_time) {
            System.out.println("Agents stop at " + stop_time + "ticks!");

            List<Integer> rnd = new LinkedList<Integer>();
            List<Map.Entry<Integer, Double>> array = new LinkedList<Map.Entry<Integer, Double>>();
            //<robotID, correction factor>
            int n = 0;
            for(Map.Entry<IAgent, Integer> pair : agents){
                IAgent agent = pair.getKey();
                int agentID = agent.getRobotID();

                rnd.add(agentID);
                array.add(n, new AbstractMap.SimpleEntry<Integer, Double>(agentID, agent.getCorrection()));
                n++;
            }

            //降順
            array.sort(new Comparator<Map.Entry<Integer, Double>>() {
                @Override
                public int compare(Map.Entry<Integer, Double> o1, Map.Entry<Integer, Double> o2){
                    return o2.getValue().compareTo(o1.getValue());
                }
            });

            for(int i=0; i<stop_robots_number; i++) {
                double value = array.get(i).getValue();
                int delete_id = array.get(i).getKey();

                agentActions.put(delete_id, AgentActions.stop);

                rnd.remove(rnd.indexOf(delete_id));
                //ログ記録
                stopAgents.writeLine(delete_id + "," + agentActions.get(delete_id) + "," + value);
                System.out.println("Agent" + delete_id + " is stopped.");
            }
        }

        if(time == restart_time) {
            System.out.println("Agents restart at " + restart_time + "ticks!");

            for(Map.Entry<Integer, AgentActions> entry : agentActions.entrySet()) {
                int agentId = entry.getKey();
                AgentActions agentAction = entry.getValue();

                if(agentAction == AgentActions.stop) {
                    for (Map.Entry<IAgent, Integer> pair : agents) {
                        if(pair.getValue().intValue() == agentId) {
                            IAgent restartAgent = pair.getKey();
                            AgentActions nextAction = restartAgent.getAction();
                            agentActions.put(agentId, nextAction);
                            //ログ記録
                            restartAgents.writeLine(agentId + "," + agentActions.get(agentId));
                            System.out.println("Agent" + agentId + " is restarted.");
                        }
                    }
                }
            }

        }
    }

    //各エージェントが回収したごみの量を記録
    private void printAccumulatedLitter(int interval){
        RobotDataCollection robots = environment.getRobotDataCollection();
        int time = environment.getTime();
        ObservedData data = new ObservedData(time, robots);

        if(time % interval == 0){
            //書き込み内容
            String[] logs = new String[agents.size() + 1];
            logs[0] = Integer.toString(time);

            for(Map.Entry<IAgent, Integer> pair : agents){
                int id = pair.getValue();
                int litter = data.getRobotDataCollection().getAllRobotData().get(id).getAccumulatedLitter();

                accumulatedLitters[id] = litter - preAccumulatedLitters[id];
                preAccumulatedLitters[id] = litter;
                logs[id + 1] = Integer.toString(accumulatedLitters[id]);
            }

            accumulatedLitter.writeLineBlock(logs);
        }
    }

    //各エージェントのマップ全体の重要度を記録
    private void printAgentImportance(int interval){
        int time = environment.getTime();
        int scale = ((int) Math.sqrt(graph.getAllNode().size()) - 1) / 2;

        if(time % interval == 0){
            for(Map.Entry<IAgent, Integer> pair : agents){
                IAgent agent = pair.getKey();
                int id = pair.getValue();

                String dir = LogManagerContext.getLogManager().makeDir("Agent" + id + "/Importance");
                LogWriter2 agentImportanceLogger = LogManagerContext.getLogManager().createWriter2(dir + "/Importance_" + time);

                //各エージェントの重要度を表示
                LitterSpawnPattern mySpawnPattern = agent.getMySpawnPattern();

                //ファイルへ書き込み
                String[][] imps = new String[2*scale+1][2*scale+1];
                int k = 0;
                for(int j = 2*scale; j >= 0; j--){
                    for(int i = 0; i <= 2*scale; i++){
                        int x = i - scale;
                        int y = j - scale;
                        int node_number = graph.getNode(x, y);

                        //ここでnullpointerexception対処療法
                        double importance;
                        if(mySpawnPattern.getSpawnProbability(node_number) != null){
                            importance = mySpawnPattern.getSpawnProbability(node_number).getProbability();
                        }
                        else{
                            importance = 0.0;
                        }

                        imps[k][i] = Double.toString(importance);
                    }
                    k++;
                }
                agentImportanceLogger.writeLineMatrix(imps);
            }
        }
    }

    //各エージェントの担当ノードを記録
    private void printAgentInformation(int interval){
        int time = environment.getTime();

        if(time % interval == 0){
            int scale = ((int) Math.sqrt(graph.getAllNode().size()) - 1) / 2;
            int[][] value = new int[2*scale+1][2*scale+1];

            for(int id = 0; id < agents.size(); id++){
                //マルチスレッド用
                String dir = LogManagerContext.getLogManager().makeDir("Agent" + id + "/Information");
                LogWriter2 graphAgentExportCsv = LogManagerContext.getLogManager().createWriter2(dir + "/graph _" + time);

                //初期化(白)
                for(int i = 0; i <= 2*scale; i++){
                    for(int j = 0; j <= 2*scale; j++){
                        value[i][j] = 1;
                    }
                }

                //グラフ情報の書き込み
				//0:充電基地 緑 1:それ以外 白 2:担当ノード オレンジ 3:自分の位置 赤 4:障害物 黒 5:発生確率一律 ピンク

                Coordinate c;
                int x, y;

                //除外ノード(黒)
                for (int node : excludeNodes) {
					c = graph.getCoordinate(node);

					x = c.x + scale;
					y = c.y + scale;

					value[x][y] = 4;
				}

				//担当ノード(オレンジ)
				for (int node : agents.get(id).getKey().getCommunicationDetails().searchNodes) {
					c = graph.getCoordinate(node);

					x = c.x + scale;
					y = c.y + scale;

					value[x][y] = 2;
				}

				//充電基地(緑)
				c = new Coordinate(0, 0);
				x = c.x + scale;
				y = c.y + scale;
				value[x][y] = 0;

				//自分の位置(赤)
				c = graph.getCoordinate(environment.getRobotData(id).getPosition());
				x = c.x + scale;
				y = c.y + scale;
				value[x][y] = 3;

				//ファイルへ書き込み
				String[][] values = new String[2*scale+1][2*scale+1];
				int k = 0;
				for(int j=2*scale; j>=0; j--) {
					for(int i=0; i<=2*scale; i++) {
						values[k][i] = Integer.toString(value[i][j]);
					}
					k++;
				}
				graphAgentExportCsv.writeLineMatrix(values);
			}
		}
    }

    //各エージェントの担当ノード数を記録
    private void printSearchNodeNumber(int interval){
        RobotDataCollection robots = environment.getRobotDataCollection();
        int time = environment.getTime();
        ObservedData data = new ObservedData(time, robots);

        if(time % interval == 0){
            //書き込み内容
            for(Map.Entry<IAgent, Integer> pair : agents){
                int id = pair.getValue();
                IAgent agent = pair.getKey();

                int searchNodeNumber = agent.getSearchNodeNumber();
                Coordinate myCenterNode = agent.getCenterNode();

                String dir = LogManagerContext.getLogManager().makeDir("Agent" + id);
                //time, searchNodeNumber, myCenterNode.x, myCenterNode.y
                searchNode = LogManagerContext.getLogManager().createWriter2(dir + "/SearchNode");
                searchNode.writeLine(data.getTime() + "," + searchNodeNumber + "," + myCenterNode.x + "," + myCenterNode.y);
            }
        }
    }

    private void printAgentEvaluation(int interval){
        int time = environment.getTime();

        if(time % interval == 0){
            int r = 0, pgs = 0, li = 0, bnps = 0;

            for(Map.Entry<IAgent, Integer> pair : agents){
                IAgent agent = pair.getKey();

                if(agentActions.get(agent.getRobotID()) != AgentActions.stop){
                    switch(agent.getTargetter()){
                        case 1: //Random
                            r++;
                            break;
                        case 2: //Greedy
                            pgs++;
                            break;
                        case 3: //LongInterval
                            li++;
                            break;
                        case 4: //BNPS
                            bnps++;
                            break;
                        default:
                            break;
                    }
                }
            }

            strategyLogger.writeLine(time + "," + r + "," + pgs + "," + li + "," + bnps);
        }
    }

    private void printAgentPositionCount(int interval){
        int time = environment.getTime();
        int scale = ((int) Math.sqrt(graph.getAllNode().size()) - 1) / 2;

        if(time % interval == 0){
            for(Map.Entry<IAgent, Integer> pair : agents){
                IAgent agent = pair.getKey();
                int id = pair.getValue();
                //マルチスレッド用
                String dir = LogManagerContext.getLogManager().makeDir("Agent" + id + "/PositionCount");
                LogWriter2 positionLogger = LogManagerContext.getLogManager().createWriter2(dir + "/PositionCount_" + time);
                
                String[][] pcount = new String[2*scale+1][2*scale+1];
                for(int i = 0; i < 2*scale+1; i++){
                    for(int j = 0; j < 2*scale+1; j++){
                        pcount[i][j] = Integer.toString(agent.getPositionCount(i, j));
                    }
                }

                positionLogger.writeLineMatrix(pcount);
            }
        }
    }
}
