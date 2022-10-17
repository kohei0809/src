package test.RTest;

import java.util.AbstractMap;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import core.GridGraph;
import core.IAgentManager;
import core.IEnvironment;
import core.LitterSpawnPattern;
import core.RobotDataCollection;
import core.agent.AgentActions;
import core.agent.IAgent;
import core.agent.ObservedData;
import core.util.LogManagerContext;
import core.util.LogWriter2;
import core.util.PotentialCollection;
import main.simulations.BehaviorType;

public class RTestAgentManager implements IAgentManager{
    //Communication無し
    //計画停止無し

    IEnvironment environment;
    BehaviorType behaviorType; // controlled by manager or autonomous
    List<Map.Entry<IAgent, Integer>> agents; //<agent, robotID>
    Map<Integer, AgentActions> agentActions; //previous actions before update
    List<Integer> excludeNodes;
    Random rand;
    GridGraph graph;
    int[] potentialCenterNodeMemory ={-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    PotentialCollection[] agentPotential = new PotentialCollection[20];
    
    int[][] agentPosition;

    //ログ出力用
    int[] accumulatedLitters = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    int[] preAccumulatedLitters = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    LogWriter2 accumulatedLitter; //各エージェントの各ステップでのごみ回収量
    LogWriter2 strategyLogger; //各エージェントがどの戦略をとっているか

    public RTestAgentManager(IEnvironment environment, BehaviorType behavior, int seed, GridGraph graph){
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
        }

        //printAgentImportance(10000);
        //printAccumulatedLitter(3600);
        //printAgentEvaluation(3600);
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
                        pcount[j][i] = Integer.toString(agentPosition[j][i]);
                    }
                }

                positionLogger.writeLineMatrix(pcount);
            }
        }
    }
}
