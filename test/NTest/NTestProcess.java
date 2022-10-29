package test.NTest;

import java.util.concurrent.CountDownLatch;

import core.util.LogManager2;
import core.util.LogManagerContext;
import main.simulations.AgentType;
import main.simulations.BehaviorType;

public class NTestProcess implements Runnable{
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
    private double requirement;
    private BehaviorType behaviorType;
    private double correction;

    public NTestProcess(LogManager2 logManager, CountDownLatch startLatch, int runs, AgentType agentType, String env, int t, String targetter,
            String date, int robots, int counter, int s, int p, int scale, BehaviorType behaviorType, double requirement, double correction){
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
        this.requirement = requirement;
        this.correction = correction;
    }
    @Override
    public void run(){
        try{
            LogManagerContext.setLogManager(logManager);
            //スタートの合図を待つ
            startLatch.await();

            LogManager2 logManager = LogManagerContext.getLogManager();
            logManager.setLogDirectory("log/NTest/" + agentType + "/" + env + "/" + t + "-" + targetter + "/" 
                    + date + " robots-number=" + robots + "_" + counter);
            NTestSimulationFactory factory = new NTestSimulationFactory();
            factory.setLocalNumbers(s, s+3, s+1, p, s+2, t, robots, scale, true, env, behaviorType, agentType, requirement, correction);

            NTestSimulation simulation = new NTestSimulation(factory);
            simulation.reset();
            simulation.run(runs);
        } catch(InterruptedException e){
            e.printStackTrace();
        }
    }
}
