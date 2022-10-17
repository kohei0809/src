package test.CTest;

import java.util.AbstractMap;
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

public class CTestAgentManager implements IAgentManager{
    //計画停止無し

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

    //ログ出力用
    int[] accumulatedLitters = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    int[] preAccumulatedLitters = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    LogWriter2 accumulatedLitter; //各エージェントの各ステップでのごみ回収量
    LogWriter2 strategyLogger; //各エージェントがどの戦略をとっているか
    LogWriter2 difValueLogger;
    LogWriter2 negoLoggerBT;
    LogWriter2 negoLoggerTO;

    public CTestAgentManager(IEnvironment environment, BehaviorType behavior, int seed, GridGraph graph){
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

        //マルチスレッド用
        accumulatedLitter = LogManagerContext.getLogManager().createWriter2("AccumulatedLitter");
        accumulatedLitter.writeLineBlock(lavels);
        strategyLogger = LogManagerContext.getLogManager().createWriter2("StrategyDistribution");
        strategyLogger.writeLine("time,R,PGS,LI,BNPS\n");
        difValueLogger = LogManagerContext.getLogManager().createWriter2("DifValueLogger");
        negoLoggerBT = LogManagerContext.getLogManager().createWriter2("NegoLoggerBT");	//Negotiation For Balancing Tasks
        negoLoggerTO = LogManagerContext.getLogManager().createWriter2("NegoLoggerTO"); //negotiation For Trade-Off Responsibility
    }

    public void addAgent(IAgent agent){
        agents.add(new AbstractMap.SimpleEntry<IAgent, Integer>(agent, agent.getRobotID()));
        agentActions.put(agent.getRobotID(), AgentActions.standby);
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
                //printAgentInformation(3600);
                break;
        }

        //printAgentImportance(10000);
        printAccumulatedLitter(3600);
        printAgentEvaluation(3600);
        //printAgentPositionCount(3600);
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
                    if(agentActions.get(id) == AgentActions.charge){
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

        /*
        //エージェントの計画停止関係を書く
        */

        ObservedData data = new ObservedData(time, robots);

        for(Map.Entry<IAgent, Integer> pair : agents){
            IAgent agent = pair.getKey();
            int id = pair.getValue();

            //エージェントが停止していなければ
            if(agentActions.get(id) != AgentActions.stop){
                agent.update(data);                
                if(agent.getAction() == AgentActions.move){
                    communication(id, data);
                    
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

        String logdir = LogManagerContext.getLogManager().makeDir("AgentImportance");

        if(time % interval == 0){
            for(Map.Entry<IAgent, Integer> pair : agents){
                IAgent agent = pair.getKey();
                int id = pair.getValue();

                String dir = LogManagerContext.getLogManager().makeDir(logdir + "/" + "Agent" + id);
                LogWriter2 agentImportanceLogger = LogManagerContext.getLogManager().createWriter2(dir + "/" + "Agent" + id + "_Importance_" + time);

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

    private void printAgentInformation(int interval){
        int time = environment.getTime();

        if(time % interval == 0){
            int scale = ((int) Math.sqrt(graph.getAllNode().size()) - 1) / 2;
            int[][] value = new int[2*scale+1][2*scale+1];

            for(int id = 0; id < agents.size(); id++){
                //マルチスレッド用
                String dir = LogManagerContext.getLogManager().makeDir("AgentGraph");
                LogWriter2 graphAgentExportCsv = LogManagerContext.getLogManager().createWriter2(dir + "/" + "graph _" + time + "_Agent" + id);

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

            strategyLogger.writeLine(time + "," + r + "," + pgs + "," + li + "," + bnps + "\n");
        }
    }

    private void printAgentPositionCount(int interval){
        int time = environment.getTime();
        //マルチスレッド用
		String logdir = LogManagerContext.getLogManager().makeDir("AgentPositionCount");
        int scale = ((int) Math.sqrt(graph.getAllNode().size()) - 1) / 2;

        if(time % interval == 0){
            for(Map.Entry<IAgent, Integer> pair : agents){
                IAgent agent = pair.getKey();
                int id = pair.getValue();
                //マルチスレッド用
				String dir = LogManagerContext.getLogManager().makeDir(logdir + "/" + "Agent" + id);
                LogWriter2 positionLogger; //各エージェントがどのノードにどれくらい行ったのか
                positionLogger = LogManagerContext.getLogManager().createWriter2(dir + "/" + "Agent" + id + "_PositionCount_" + time);
                
                System.out.println(agentPosition.length);
                String[][] pcount = new String[2*scale+1][2*scale+1];
                for(int i = 0; i < 2*scale+1; i++){
                    for(int j = 0; j < 2*scale+1; j++){
                        pcount[j][i] = Integer.toString(agent.positionCount[j][i]);
                    }
                }

                positionLogger.writeLineMatrix(pcount);
            }
        }
    }
}
