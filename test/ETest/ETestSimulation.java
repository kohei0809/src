package test.ETest;

import core.Coordinate;
import core.GridGraph;
import core.IEnvironment;
import core.LitterData;
import core.LitterDataCollection;
import core.util.LogManagerContext;
import core.util.LogWriter2;
import main.simulations.Evaluator;
import main.simulations.SimulationFactory;

public class ETestSimulation {
    //ETest用のSimulation

    SimulationFactory factory;
    IEnvironment environment;
    Evaluator evaluator;

    //LogWriter logWriter;
    private int evaluationValue;

    public ETestSimulation(SimulationFactory f){
        factory = f;
    }

    public void run(int steps){
        for(int i = 0; i < steps; i++){
            //進行状況記録
            if((i % 100000) == 0){
                System.out.println(i + "steps");
            } 
            step();
        }
    }

    public void step(){
        //printlitteramount(Log Export)
        printLitterAmount(100000);

        //Litter appears
        environment.update();

        //Update time
        environment.update();

        evaluationValue = evaluator.getEvaluation();
    }

    public void reset(){
        SimulationFactory fac = factory;
        fac.make();
        environment = fac.environment();
        evaluator = fac.evaluator();

        evaluationValue = evaluator.getEvaluation();
    }

    public void setEvaluationValue(int v){
        evaluationValue = v;
    }

    public int getEvaluationValue(){
        return evaluationValue;
    }

    private void printLitterAmount(int interval){
        int time = environment.getTime();
        //表示間隔
        //int interval = 10000;
        /*if(time >= 950000 && time <= 1050000){
            interval = 100;
        }*/

        //intervalごとにごみの残量を表示
        if(time % interval == 0){
            LitterDataCollection litterDataCollection = environment.getLitterDataCollection();
            GridGraph graph = factory.graph();
            int scale = ((int) Math.sqrt(graph.getAllNode().size() - 1)) / 2;
            int[][] value = new int [2*scale+1][2*scale+1];

            /*シングルスレッド用
			//String dir = LogManager.makeDir("LitterAmount");
			LogWriter2 litterLogger = LogManager.createWriter2(dir + "/" + "LitterAmount_" + time);*/

			//マルチスレッド用
			String dir = LogManagerContext.getLogManager().makeDir("LitterAmount");
			LogWriter2 litterLogger = LogManagerContext.getLogManager().createWriter2(dir + "/" + "LitterAmount_" + time);
			//litterLogger.writeLine("Thread.currentThread().getId()=" + Thread.currentThread().getId());

            //初期化
            for(int i=0; i <= 2+scale; i++){
                for(int j = 0; j <= 2*scale; j++){
                    value[i][j] = 0;
                }
            }

            for(LitterData data : litterDataCollection.getAllLitters()){
                int position = data.getPosition();
                Coordinate coo = graph.getCoordinate(position);

                int x = coo.x + scale;
                int y = coo.y + scale;

                value[x][y] = data.getAmount();
            }

            //ファイルへ書き込み
            String[][] amounts = new String[2*scale+1][2*scale+1];
            int k = 0;
            for(int j = 2*scale; j >= 0; j--){
                //String[] amounts = new String[2*scale+1];

                for(int i = 0; i <= 2*scale; i++){
                    //amounts[i] = Integer.toString(value[i][j]);
                    amounts[k][i] = Integer.toString(value[i][j]);
                }

                k++;
//				litterLogger.writeLineBlock(amounts);
            }
            litterLogger.writeLineMatrix(amounts);

            /*for(int j=2*scale; j>=0; j--) {
            	for(int i=0; i<=2*scale; i++) {
            		litterLogger.write(Integer.toString(value[i][j]));
            
            		//カンマで区切る
            		if(i != 2*scale) {
                		litterLogger.write(",");
                	}
                }
                //改行
                litterLogger.writeLine("");
            }*/
        }
    }
}
