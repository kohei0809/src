package test.OTest;

import core.Coordinate;
import core.GridGraph;
import core.IAgentManager;
import core.IEnvironment;
import core.LitterData;
import core.LitterDataCollection;
import core.util.LogManagerContext;
import core.util.LogWriter2;
import main.simulations.Evaluator;
import main.simulations.SimulationFactory;

public class OTestSimulation {

    SimulationFactory factory;
    IEnvironment environment;
    IAgentManager agentManager;
    Evaluator evaluator;

    private int evaluationValue;

    public OTestSimulation(SimulationFactory f){
        factory = f;
    }

    public void run(int steps){
        for(int i = 0; i < steps; i++){
            //進行状況記録
            if(i % 10000 == 0){
                System.out.println(i + "steps");
            }
            if(i == 0){
                //print litter amount(Log Export)
		        //printLitterAmount(3600); //default: 10000
            }
            step();
        }
    }

    public void step() {
        
		// Litter appears
		environment.update();

		// move, charge
		agentManager.move();
		environment.update();

		// clean
		agentManager.clean();
		environment.update();

		// Update time
		environment.update();

		evaluationValue = evaluator.getEvaluation();

        //print litter amount(Log Export)
		//printLitterAmount(3600); //default: 10000
	}

    public void reset(){
        SimulationFactory fac = factory;
        fac.make();
        environment = fac.environment();
        agentManager = fac.agentManager();
        evaluator = fac.evaluator();
        evaluationValue = evaluator.getEvaluation();
    }

    public void setEvaluationValue(int v){
        evaluationValue = v;
    }

    public int getEvaluationValue(){
        return evaluationValue;
    }

    public IEnvironment environment(){
        return environment;
    }

    private void printLitterAmount(int interval){
        int time = environment.getTime();

        //intervalごとにごみの残量を表示
        if(time % interval == 0){
            LitterDataCollection litterDataCollection = environment.getLitterDataCollection();
            GridGraph graph = factory.graph();
            int scale = ((int) Math.sqrt(graph.getAllNode().size()) - 1) / 2;
            int[][] value = new int[2*scale+1][2*scale+1];

            //シングルスレッド用
//			String dir = LogManager.makeDir("LitterAmount");
//			LogWriter2 litterLogger = LogManager.createWriter2(dir + "/" + "LitterAmount_" + time);

            //マルチスレッド用
            String dir = LogManagerContext.getLogManager().makeDir("LitterAmount");
            LogWriter2 litterLogger = LogManagerContext.getLogManager().createWriter2(dir + "/" + "LitterAmount_" + time);

            //初期化
			for(int i=0; i<= 2*scale; i++) {
				for(int j=0; j<= 2*scale; j++) {
					value[i][j] = 0;
				}
			}

			for(LitterData data : litterDataCollection.getAllLitters()) {
				int position = data.getPosition();
				Coordinate coo = graph.getCoordinate(position);

				int x = coo.x + scale;
				int y = coo.y + scale;

				value[x][y] = data.getAmount();
			}

			//ファイルへ書き込み
			String[][] amounts = new String[2*scale+1][2*scale+1];
			int k = 0;
			for(int j=2*scale; j>=0; j--) {
				for(int i=0; i<=2*scale; i++) {
					amounts[k][i] = Integer.toString(value[i][j]);
					//System.out.print(amounts[k][i] + " ");
				}
				//System.out.println();
				k++;
			}
			litterLogger.writeLineMatrix(amounts);
        }
    }
}
