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
import core.util.LogManager;
import core.util.LogManagerContext;
import core.util.LogWriter2;
import core.util.PotentialCollection;
import main.simulations.BehaviorType;

public class RTestAgentManager2 implements IAgentManager{
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

    //ログ出力用
    int[] accumulatedLitters = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    int[] preAccumulatedLitters = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    LogWriter2 accumulatedLitter; //各エージェントの各ステップでのごみ回収量

    public RTestAgentManager2(IEnvironment environment, BehaviorType bType, int seed, GridGraph graph){
        this.environment = environment;
        behaviorType = bType;
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

        accumulatedLitter = LogManager.createWriter2("AccumulatedLitter");
        accumulatedLitter.writeLineBlock(lavels);
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
            case hand:
                hand();
                break;
        }

        //printAgentImportance(10000);
        printAccumulatedLitter(1);
    }

    //手動
    public void hand(){
        RobotDataCollection robots = environment.getRobotDataCollection();
        int time = environment.getTime();

        ObservedData data = new ObservedData(time, robots);

        for(Map.Entry<IAgent, Integer> pair : agents){
            IAgent agent = pair.getKey();
            int id = pair.getValue();

            //エージェントが停止していなければ
            if(agentActions.get(id) != AgentActions.stop){
                agent.update(data);
                if(agent.getAction() == AgentActions.move){
                    environment.moveRobot(id, agent.getNextNode());
                }
                agentActions.put(id, agent.getAction());
            }
        }
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
}

