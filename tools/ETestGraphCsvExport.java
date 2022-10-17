package tools;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;
import java.util.Map;

import core.Coordinate;
import core.GridGraph;
import core.IEnvironment;
import core.agent.IAgent;
import core.util.LogManager;
import core.util.LogWriter2;
import test.ETest.ETestSimulationFactory;

public class ETestGraphCsvExport {
	//グラフ情報をcsvファイルに出力するクラス

	public static void exportCsv(ETestSimulationFactory factory, String env) {
		GridGraph graph = factory.graph();
		int scale = factory.scale();
		int[][] value = new int[2*scale+1][2*scale+1];

		try {
			//出力ファイルの作成
			FileWriter fw = new FileWriter("./env/graph_" + env + ".csv", false);
			PrintWriter pw = new PrintWriter(new BufferedWriter(fw));

			//初期化
			for(int i=0; i<= 2*scale; i++) {
				for(int j=0; j<= 2*scale; j++) {
					value[i][j] = 0;
				}
			}

			//グラフ情報の書き込み
			//0:発生確率0 緑 1:発生確率低 白 2:発生確率中 オレンジ 3:発生確率高 赤 4:障害物 黒 5:発生確率一律 ピンク

			//除外ノード
			for (int node : factory.excludeNodes()) {
				Coordinate c = graph.getCoordinate(node);

				int x = c.x + scale;
				int y = c.y + scale;

				value[x][y] = 4;
			}

			//高確率ノード
			for (int node : factory.highNodes()) {
				Coordinate c = graph.getCoordinate(node);

				int x = c.x + scale;
				int y = c.y + scale;

				value[x][y] = 3;
			}

			//中確率ノード
			for (int node : factory.middleNodes()) {
				Coordinate c = graph.getCoordinate(node);

				int x = c.x + scale;
				int y = c.y + scale;

				value[x][y] = 2;
			}

			//低確率ノード
			for (int node : factory.lowNodes()) {
				Coordinate c = graph.getCoordinate(node);

				int x = c.x + scale;
				int y = c.y + scale;

				value[x][y] = 1;
			}

			//確率一律ノード
			for (int node : factory.uniformNodes()) {
				Coordinate c = graph.getCoordinate(node);

				int x = c.x + scale;
				int y = c.y + scale;

				value[x][y] = 5;
			}

			//ファイルへ書き込み
			for(int j=2*scale; j>=0; j--) {
				for(int i=0; i<=2*scale; i++) {
					pw.print(value[i][j]);

					//カンマで区切る
					if(i != 2*scale) {
						pw.print(",");
					}
				}
				//改行
				pw.println();
			}
			//ファイルを閉じる
			pw.close();

		} catch (IOException e) {
			e.printStackTrace();
		}

		System.out.println("Graph csv exporting completed.");
	}

	public static void exportCsv_Agent(GridGraph graph, List<Integer> excludeNodes, List<Map.Entry<IAgent, Integer>> agents, IEnvironment environment) {
//		GridGraph graph = factory.graph();
		int time = environment.getTime();
		int scale = ((int) Math.sqrt(graph.getAllNode().size()) - 1) / 2;
		int[][] value = new int[2*scale+1][2*scale+1];

		for(int id=0; id<agents.size(); id++) {
			String dir = LogManager.makeDir("AgentGraph");
			LogWriter2 graphAgentExportCsv = LogManager.createWriter2(dir + "/" + "graph_" + time + "_Agent" + id);

			//初期化(白)
			for(int i=0; i<= 2*scale; i++) {
				for(int j=0; j<= 2*scale; j++) {
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

			//ファイルへ書き込み(変更)
			for(int j=2*scale; j>=0; j--) {
				String[] values = new String[2*scale+1];

				for(int i=0; i<=2*scale; i++) {
					values[i] = Integer.toString(value[i][j]);
				}

				graphAgentExportCsv.writeLineBlock(values);
			}

//			for(int j=2*scale; j>=0; j--) {
//				for(int i=0; i<=2*scale; i++) {
//					graphAgentExportCsv.write(Integer.toString(value[i][j]));
//
//					//カンマで区切る
//					if(i != 2*scale) {
//						graphAgentExportCsv.write(",");
//					}
//				}
//				//改行
//				graphAgentExportCsv.writeLine("");
//			}

//		System.out.println("Graph csv exporting completed.");
		}
	}
}

