package test.PTest;

import java.util.concurrent.CountDownLatch;

import core.util.LogManager2;
import core.util.LogManagerContext;
import main.simulations.AgentType;
import main.simulations.BehaviorType;

public class PTestProcess implements Runnable{
    private LogManager2 logManager;
    private CountDownLatch startLatch;
    private int runs;
    private AgentType agentType;
    private String env;
    private int t;
    private String targetter;
    private String date;
    private int robots;
    private int counter;
    private int s;
    private int p;
    private int scale;
    private BehaviorType behaviorType;

    public PTestProcess(LogManager2 logManager, CountDownLatch startLatch, int runs, AgentType agentType, String env, int t, String targetter,
            String date, int robots, int counter, int s, int p, int scale, BehaviorType behaviorType){
        this.logManager = logManager;
        this.startLatch = startLatch;
        this.runs = runs;
        this.agentType = agentType;
        this.env = env;
        this.t = t;
        this.targetter = targetter;
        this.date = date;
        this.robots = robots;
        this.counter = counter;
        this.s = s;
        this.p = p;
        this.scale = scale;
        this.behaviorType = behaviorType;
    }
    @Override
    public void run(){
        try{
            LogManagerContext.setLogManager(logManager);
            //スタートの合図を待つ
            startLatch.await();

            LogManager2 logManager = LogManagerContext.getLogManager();
            logManager.setLogDirectory("log/PTest/" + agentType + "/" + env + "/" + t + "-" + targetter + "/" 
                    + date + " robots-number=" + robots + "_" + counter);
            PTestSimulationFactory factory = new PTestSimulationFactory();
            factory.setLocalNumbers(s, s+1, p, s+2, t, robots, scale, true, env, behaviorType, agentType);

            PTestSimulation simulation = new PTestSimulation(factory);
            simulation.reset();
            simulation.run(runs);
        } catch(InterruptedException e){
            e.printStackTrace();
        }
    }
}
