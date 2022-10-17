package test.ETest;

import java.util.concurrent.CountDownLatch;

import core.util.LogManager2;
import core.util.LogManagerContext;
import tools.ETestGraphCsvExport;

public class ETestProcess implements Runnable{
    //ETest用のprocess

    private LogManager2 logManager;
    private CountDownLatch startLatch;
    private int runs;
    private String env;
    private String date;
    private int counter;
    private int s;
    private int scale;

    public ETestProcess(LogManager2 logManager, CountDownLatch startLatch, int runs, String env, String date, 
            int counter, int s, int scale){
        this.logManager = logManager;
        this.startLatch = startLatch;
        this.runs = runs;
        this.env = env;
        this.date = date;
        this.counter = counter;
        this.s = s;
        this.scale = scale;
    }

    @Override
    public void run(){
        //TODO 自動生成されたメソッド・スタブ

        try{
            LogManagerContext.setLogManager(logManager);
            //スタートの合図を待つ
            startLatch.await();

            LogManager2 logManager = LogManagerContext.getLogManager();
            logManager.setLogDirectory("log/" + "ETest" + "/" + env + "/" + date + "_" + counter);
            ETestSimulationFactory factory = new ETestSimulationFactory();
            factory.setLocalNumbers(s, scale, true, env);
            
            ETestSimulation simulation = new ETestSimulation(factory);
            simulation.reset();
            //環境表示テスト
            if(counter == 1){
                ETestGraphCsvExport.exportCsv(factory, env);
            }

            simulation.run(runs);
        } catch(InterruptedException e){
            e.printStackTrace();
        }
    }
}
