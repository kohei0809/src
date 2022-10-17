package main.simulations;

import core.IAgentManager;
import core.IEnvironment;
import core.util.LogManagerContext;
import core.util.LogWriter2;

public class Evaluator {
    //ごみの量などを評価するプログラム

    IEnvironment environment;
    IAgentManager agentManager;
    int evaluationSum100 = 0, evaluationCost100 = 0;
    int evaluationSum3600 = 0, evaluationCost3600 = 0;
    int maxLitterAmount = 0;

    LogWriter2 logger100, logger3600;

    public Evaluator(IEnvironment en, IAgentManager manager){
        environment = en;
        agentManager = manager;

        //シングルスレッド用
		//logger100 = LogManager.createWriter("evaluation-s100");
//		logger3600 = LogManager.createWriter2(dir + "/../" + "evaluation-s3600");

//		//マルチスレッド用
//		//logger100 = LogManagerContext.getLogManager().createWriter("evaluation-s100");
		logger3600 = LogManagerContext.getLogManager().createWriter2("evaluation-s3600");
    }

    public Evaluator(IEnvironment en){
        environment = en;

        //シングルスレッド用
		//logger100 = LogManager.createWriter("evaluation-s100");
		//logger3600 = LogManager.createWriter("evaluation-s3600");

//		//マルチスレッド用
//		//logger100 = LogManagerContext.getLogManager().createWriter("evaluation-s100");
		//logger3600 = LogManagerContext.getLogManager().createWriter("evaluation-s3600");
    }

    public int getEvaluation(){
        int time = environment.getTime();
        int litterAmount = environment.getLitterAmount();
        int currentmaxLitterAmount = environment.getMaxLitterAmount();
        int energyCost = environment.getTotalEnergyCost();
		//int ecoCount = agentManager.getEcoAgentNumber();

        // Cumulative Existence Amount of Trash
        evaluationSum3600 += litterAmount;
        evaluationCost3600 += energyCost;

        // Max Amount of Trash
        if(maxLitterAmount < currentmaxLitterAmount){
            maxLitterAmount = currentmaxLitterAmount;
        }

        //OutPut
        if(time % 3600 == 0){
            //logger3600.writeLine("Thread.currentThread().getId()=" + Thread.currentThread().getId() + "\n");
            logger3600.writeLine(time + "," + evaluationSum3600 + "," + maxLitterAmount + "," + evaluationCost3600);
            evaluationSum3600 = 0;
            maxLitterAmount = 0;
            evaluationCost3600 = 0;
        }
        return litterAmount;
    }
}
