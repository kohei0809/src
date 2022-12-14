package core;

import java.util.LinkedList;
import java.util.List;

import core.agent.AgentActions;
//import core.util.HardMarginSVM;
import core.util.LogManagerContext;
import core.util.LogWriter2;

public class Clustering {

    private final int max_iter = 300;
    private final double threshold = 0.7;
    
    private List<Integer> cluster; //各クラスター
    private double[][] centroids; //各クラスターの重心
    //private HardMarginSVM svm;
    private double[][] X; //[x, y]
    private double[] ave, std;
    private int robotNum = 30;
    
    private int[] prePausingUnit = new int[robotNum];
    private double[] stopThreshold = new double[robotNum];
    private int[] prePausingTime = new int[robotNum];
    //private boolean[] changeAgent = new boolean[robotNum];
    private double alpha = 0.1;
    private int count = 0;

    private LogWriter2 clusteringLogger;
    private LogWriter2 silLogger;
    private LogWriter2 standardizationLogger;
    private LogWriter2 stopThresholdLogger;
    private LogWriter2 clusterThresholdLogger;

    public Clustering(int seed){
        //svm = new HardMarginSVM(seed);
        ave = new double[2];
        std = new double[2];

        clusteringLogger = LogManagerContext.getLogManager().createWriter2("clusteringLogger");
        silLogger = LogManagerContext.getLogManager().createWriter2("silLogger");
        standardizationLogger = LogManagerContext.getLogManager().createWriter2("standardizationLogger");
        //stopThresholdLogger = LogManagerContext.getLogManager().createWriter2("stopThresholdLogger");
        clusterThresholdLogger = LogManagerContext.getLogManager().createWriter2("clusterThreshold");
    }

    //停止できるか
    public boolean isStop(List<Double> x, List<Integer> y, int[] pausingStepUnit, int time, int interval, AgentActions[] AoA){        
        //総待機時間がintervalを超えているか
        int sum = 0;
    	/*
    	for(int i = 0; i < y.size(); i++){
        	sum += y.get(i);
        }
    	clusteringLogger.writeLine(time + "," + sum);
    	if(sum < interval){
    		return false;
    	}
    	*/

        //全エージェントが待機時間を閾値よりも変化させていないか
        int diff = 0;
        //boolean flag = true;
        int j = 0;
        for(int i = 0; i < robotNum; i++){
        	if(AoA[i] == AgentActions.stop){
        		continue;
        	}
        	
            diff = Math.abs(y.get(j) - prePausingTime[i]);
            prePausingTime[i] = y.get(j);
            //diff = pausingStepUnit[i] - prePausingUnit[i];
            if(stopThreshold[i]*10 < diff){
            //if(stopThreshold[i] > diff){
                clusterThresholdLogger.writeLine(time + "," + i + "," + stopThreshold[i] + "," + stopThreshold[i]*10 + "," + diff + "," + prePausingTime[i] + "," + false);
                //changeAgent[i] = false;
                //flag = false;
                //return false;
            }
            else{
            	//changeAgent[i] = true;
            	//trueのエージェントの待機時間だけを足す
            	clusterThresholdLogger.writeLine(time + "," + i + "," + stopThreshold[i] + "," + stopThreshold[i]*10 + "," + diff + "," + prePausingTime[i] + "," + true);
            	sum += y.get(j);
            }    
            j++;
        }
        
        /*
        if(flag == false){
        	return false;
        }
        */
        
        //総待機時間がintervalを超えているか
        clusteringLogger.writeLine(time + "," + sum);
    	if(sum < interval){
    		return false;
    	}

    	//標準化
        standardization(x, y);

        //クラスタリング
        kmeans2(X[0], X[1]);
    
        //clusterを-1, 1に変換
        for(int i = 0; i < cluster.size(); i++){
            int res = 2 * cluster.get(i) - 1;
            cluster.set(i, res);
        }

        //シルエット係数による比較
        double sil = calSil(time);
        silLogger.write(time + "," + sil + ",");

        if(sil < threshold){
            silLogger.writeLine("false");
            return false;
        }
        else{
            silLogger.writeLine("true");
            return true;
            
            //各クラスターの重心の標準化を戻す
            //reStandardization();
            
            //前の停止の際のサポートベクターマシンよりもEnergy-Save groupの重心が上にあるか判定
            /*if(svm.predict(centroids[1], time)){
                //SVM実行
                int[] clusterArray = new int[cluster.size()];
                
                for(int i = 0; i < cluster.size(); i++) {
                	clusterArray[i] = cluster.get(i);
                }
                
                svm.fit(X, clusterArray, ave, std, time);
                return true;
            }
            else{
                return false;
            }*/
            
            //ES-Groupの総待機時間の合計がインターバルを超えていたら停止
            //sumTime = 0;
            //for(int i = 0; i < cluster.size(); i++){
            	/*if(cluster.get(i) == 1){
            		double restandardX = (X[1][i] * std[1]) + ave[1];
            		sumTime += restandardX;
            	}*/
            	
            	//double restandardX = (X[1][i] * std[1]) + ave[1];
        		//sumTime += restandardX;
            //}
            //clusteringLogger.writeLine(time + "," + sumTime);
            //if(sumTime >= interval){
            	//return true;
            //}
            //else{
            	//return false;
            //}
        }

    }

    //エージェントが待機時間を変更中かの閾値を更新する
    public void updateThreshold(int time, int[] pausingStepUnit, int interval){
        if(time == interval){
            for(int i = 0; i < pausingStepUnit.length; i++){
                prePausingUnit[i] = pausingStepUnit[i];
                stopThreshold[i] = pausingStepUnit[i];
            }
            //count++;
            return;
        }
        else{
            int diff = 0;
            for(int i = 0; i < pausingStepUnit.length; i++){
                diff = Math.abs(pausingStepUnit[i] - prePausingUnit[i]);
                
                //(エージェントiが待機時間を大きく減少中なら閾値を少し変える)
                
                //変化量の平均を求める
                /*
                double sum = (stopThreshold[i] * count) + diff;
                stopThreshold[i] = sum / (count+1);
				*/

                double threshold = stopThreshold[i];
                
                if(threshold < diff){
                    alpha = 0.05;
                }
                else{
                    alpha = 0.1;
                }
                
                prePausingUnit[i] = pausingStepUnit[i];

                stopThreshold[i] = (1 - alpha) * threshold + alpha * diff;
                
                String dir = LogManagerContext.getLogManager().makeDir("Agent" + i); 
                stopThresholdLogger = LogManagerContext.getLogManager().createWriter2(dir + "/stopThreshold");
                stopThresholdLogger.writeLine(time + "," + threshold + ","  + diff + "," + stopThreshold[i] + "," +alpha);
                //stopThresholdLogger.writeLine(time + "," + diff + "," + sum + "," +stopThreshold[i] + "," + count);
            }
            //count++;
        }
    }

    //k-means法によるクラスタリング(クラスタ数2)
    private void kmeans2(double[] x, double[] y){
        int k = 2; //クラスタ数
        int size = x.length;
        double[] distance = new double[k];
        cluster = new LinkedList<Integer>();
        centroids = new double[2][2];

        //Kの最小値と最大値を各クラスターの重心とする 
        centroids[0][0] = Double.MAX_VALUE;
        centroids[1][0] = -1.0;
        for(int i = 0; i < size; i++){
            double correction = x[i];
            if(correction < centroids[0][0]){
                centroids[0][0] = correction;
                centroids[0][1] = y[i];
            }
            if(correction > centroids[1][0]){
                centroids[1][0] = correction;
                centroids[1][1] = y[i];
            }
        }
        
        clusteringLogger.writeLine(centroids[0][0] + "," + centroids[0][1] + "," + centroids[1][0] + "," + centroids[1][1]);

        for(int epoch = 0; epoch < max_iter; epoch++){
            for(int i = 0; i < size; i++){
                for(int j = 0; j < k; j++){
                    //各クラスタの重心との距離を計算
                    distance[j] = ((centroids[j][0] - x[i]) * (centroids[j][0] - x[i])) + ((centroids[j][1] - y[i]) * (centroids[j][1] - y[i]));
                }
                //clusteringLogger.writeLine(i + "," + distance[0] + "," + distance[1]);
                if(distance[0] <= distance[1]){
                    cluster.add(0);
                }
                else{
                    cluster.add(1);
                }
            }

            double[][] newCentroids = {{0.0, 0.0}, {0.0, 0.0}};
            for(int i = 0; i < k; i++){
                int csize = 0;
                for(int j = 0; j < size; j++){
                    if(cluster.get(j) == i){
                        csize++;
                        newCentroids[i][0] += x[j];
                        newCentroids[i][1] += y[j];
                    }
                }

                newCentroids[i][0] /= csize;
                newCentroids[i][1] /= csize;
                
                clusteringLogger.writeLine(csize + "," + newCentroids[i][0] + "," + newCentroids[i][1]);
            }

            //終了条件
            if((newCentroids[0][0] == centroids[0][0]) && (newCentroids[0][1] == centroids[0][1])){
                break;
            }

            for(int i = 0; i < 2; i++){
                for(int j = 0; j < 2; j++){
                    centroids[i][j] = newCentroids[i][j];
                }
            }
            cluster.clear();
        }
    }

    //シルエット係数の計算
    private double calSil(int time){
        double ans = 0.0;
        int[] clus = {0, 0};
        int size = cluster.size();
        double[] sil = new double[size]; //各エージェントのシルエット係数
        double[] a = new double[size]; //各エージェントの凝集度
        double[] b = new double[size]; //各エージェントの乖離度

        for(int i = 0; i < size; i++){
            sil[i] = 0.0;
            a[i] = 0.0;
            b[i] = 0.0;
        }

        //各クラスターの数を計算
        for(int i = 0; i < size; i++){
            if(cluster.get(i) == -1){
                clus[0]++;
            }
            else{
                clus[1]++;
            }
        }
        clusteringLogger.writeLine(time + "," + clus[0] + "," + clus[1]);

        for(int i = 0; i < size; i++){
            int myclus = cluster.get(i);
            int oppclus = (1-myclus) / 2;
            
            int my_index = (myclus+1) / 2;
            int opp_index = (oppclus+1) / 2;

            //aとbの計算
            for(int j = 0; j < size; j++){
                if(i == j){
                    continue;
                }

                double dis = (X[0][i] - X[0][j]) * (X[0][i] - X[0][j]) + (X[1][i] - X[1][j]) * (X[1][i] - X[1][j]);
                dis = Math.sqrt(dis);
                
                //aの計算
                if(cluster.get(j) == myclus){
                    a[i] += dis;
                    //silLogger.writeLine(i + "," + j + "," + dis + "," + a[i] + ",a");
                }
                //bの計算
                else{
                    b[i] += dis;
                    //silLogger.writeLine(i + "," + j + "," + dis + "," + b[i] + ",b");
                }
            }

            if(clus[my_index] == 1){
                a[i] = 0.0;
            }
            else{
                a[i] /= (clus[my_index] - 1);
            }

            b[i] /= clus[opp_index];
        
            //シルエット係数の計算
            sil[i] = (b[i] - a[i]) / max(a[i], b[i]);
            ans += sil[i];
        }

        //平均シルエット係数の計算
        ans /= size;

        return ans;
    }

    //標準化
    private void standardization(List<Double> x, List<Integer> y){   	
    	int size = x.size();
        X = new double[2][size];

        ave[0] = 0.0;
        ave[1] = 0.0;
        std[0] = 0.0;
        std[1] = 0.0;

        //aveの計算
        for(int i = 0; i < size; i++){
            ave[0] += x.get(i);
            ave[1] += y.get(i);
        }

        ave[0] /= size;
        ave[1] /= size;

        //stdの計算
        for(int i = 0; i < size; i++){
            std[0] += (x.get(i) - ave[0]) * (x.get(i) - ave[0]);
            std[1] += (y.get(i) - ave[1]) * (y.get(i) - ave[1]);
        }
        
        std[0] /= size;
        std[1] /= size;

        std[0] = Math.sqrt(std[0]);
        std[1] = Math.sqrt(std[1]);
        
        if(std[1] == 0){
        	std[1] = 0;
        }
        for(int i = 0; i < size; i++){
            X[0][i] = (x.get(i) - ave[0]) / std[0];
            X[1][i] = (y.get(i) - ave[1]) / std[1];
        }
    
        standardizationLogger.writeLine(ave[0] + "," + ave[1] + "," + std[0] + "," + std[1]);
        return;
    }

    //各クラスターの重心の標準化を戻す
    private void reStandardization(){
        for(int i = 0; i < 2; i++){
            centroids[0][i] = (centroids[0][i] * std[0]) + ave[0];
            centroids[1][i] = (centroids[1][i] * std[1]) + ave[1];
        }
    }

    private double max(double a, double b){
        if(a  > b){
            return a;
        }
        else{
            return b;
        }
    }
}

