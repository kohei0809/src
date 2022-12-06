package core.util;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;

public class HardMarginSVM {
    //サポートベクターマシン(ハードマージン)についてのクラス
    //配列操作のメソッドも

    final private double eta = 0.001;
    final private int epoch = 1000;

    private Random rand;
    private boolean isTrained;
    
    //SVM関係
    private double[] w;
    private double b;
    private double[] alpha; //未定乗数
    private List<Integer> indexes_sv; //サポートベクトルのindex

    private LogWriter2 svmLogger;
    private LogWriter2 predictLogger;
    private LogWriter2 reStandardizationLogger;

    public HardMarginSVM(int seed){
        rand = new Random(seed);
        isTrained = false;

        svmLogger = LogManagerContext.getLogManager().createWriter2("svmLogger");
        predictLogger = LogManagerContext.getLogManager().createWriter2("predictLogger");
        reStandardizationLogger = LogManagerContext.getLogManager().createWriter2("reStandardizationLogger");
    }

    public void fit(double[][] X, int[] y, double[] ave, double[] std, int time){
        int num_samples = X.length;
        int num_features = X[0].length;

        //パラメータベクトルを0で初期化
        w = new double[num_features];
        for(int i = 0; i < num_features; i++){
            w[i] = 0.0;
        }
        b = 0;

        //正規乱数を用いてalpah(未定乗数)を初期化
        alpha = new double[num_samples];
        for(int i = 0; i < num_samples; i++){
            alpha[i] = rand.nextGaussian();
        }

        //最急降下法を用いて双対問題を解く
        for(int i = 0; i < epoch; i++){
            cycle(X, y);
        }

        //サポートベクトルのindexを取得
        getSVIndex(num_samples);

        //wを計算
        calW(X, y);

        //bを計算
        calB(X, y);

        //学習完了のフラグを立てる
        isTrained = true;

        //標準化を戻す
        reStandardization(ave, std, X, y, time);
        svmLogger.writeLine(time + "," + w[0] + "," + w[1] + "," + b + "," + "y=" + (-w[0]/w[1]) + "x + " + (-b/w[1]));
    }

    //Xがサポートベクターマシンよりも上にあるか判断
    public boolean predict(double[] X, int time){
        if(!isTrained){
            System.out.println("///////////////////////// Not Trained !!!! /////////////////////////");
            return true;
        }
        double resMul = matrixMul(X, w);
        double resAdd = resMul + b;

        predictLogger.writeLine(time + "," + w[0] + "," + w[1] + "," + b + "," + X[0] + "," + X[1] + "," + resAdd);

        if(resAdd < 0){
            return false;
        }
        else{
            return true;
        }
    }

    //勾配降下法の1サイクル
    private void cycle(double[][] X, int[] y){
        double[] ones = new double[y.length];

        for(int i = 0; i < ones.length; i++){
            ones[i] = 1.0;
        }

        int[][] squY = matrixSquareMul(y);
        double[][] squX = matrixSquareMul(X);

        double[][] H = matrixMul(squY, squX);

        //勾配ベクトルを計算
        double[] resMul = matrixMul(H, alpha);
        double[] grad = matrixSub(ones, resMul);

        //alpha(未定乗数)の更新
        double[] resMul2 = matrixMul(grad, eta);
        alpha = matrixAdd(alpha, resMul2);

        //alpha(未定乗数)の各成分はゼロ以上である必要があるので、負の成分をゼロにする
        matrixNegToZero(alpha);
    }

    //行列の負の要素を0にする
    private void matrixNegToZero(double[] matrix){
        for(int i = 0; i < matrix.length; i++){
            if(matrix[i] < 0.0){
                matrix[i] = 0.0;
            }
        }
        return;
    }

    //サポートベクトルのindexを取得
    private void getSVIndex(int num){
        indexes_sv = new LinkedList<Integer>();

        for(int i = 0; i < num; i++){
            if(alpha[i] != 0.0){
                indexes_sv.add(i);
            }
        }
        return;
    }

    //wを計算
    private void calW(double[][] X, int[] y){
        for(int i : indexes_sv){
            double resMul = alpha[i] * y[i];
            double[] resMul2 = matrixMul(X[i], resMul);
            w = matrixAdd(w, resMul2);
        }
    }

    //bを計算
    private void calB(double[][] X, int[] y){
        for(int i : indexes_sv){
            double resMul = matrixMul(w, X[i]);
            b += y[i] - resMul;
        }

        b /= indexes_sv.size();
    }

    //wとbの標準化を元に戻す
    private void reStandardization(double[] ave, double[] std, double[][] X, int[] y, int time){
        double[] z = new double[2];
        int size = X[0].length;
        double[] center = new double[2];

        for(int i = 0; i < size; i++){
            X[0][i] = (X[0][i] * std[0]) + ave[0];
            X[1][i] = (X[1][i] * std[1] + ave[1]);
        }

        for(int i = 0; i < 2; i++){
            z[i] = (w[i] * std[1-i]) + ave[1-i];
        }
        
        for(int i = 0; i < 2; i++){
            w[i] = z[i];
        }

        double min = Double.MAX_VALUE;
        int index_i = -1, index_j = -1;
        for(int i = 0; i < size; i++){
            if(y[i] == -1){
                for(int j = 0; j < size; j++){
                    if(y[j] == 1){
                        double dis = ((X[0][i] - X[0][j]) * (X[0][i] - X[0][j])) + ((X[1][i] - X[1][j]) * (X[1][i] - X[1][j]));
                        if(dis < min){
                            min = dis;
                            index_i = i;
                            index_j = j;
                        }
                    }
                }
            }
        }
        reStandardizationLogger.writeLine(time + "," + X[0][index_i] + "," + X[1][index_i] + "," + X[0][index_j] + "," + X[1][index_j] + "," + min);
        
        for(int i = 0; i < 2; i++){
            center[i] = (X[i][index_i] + X[i][index_j]) / 2;
        }

        b = -z[0]*center[0] - z[1]*center[1];
    }



    ////////////////////////行列計算メソッド/////////////////////////

    //////////////////////////行列の転置//////////////////////////////
    //y(b, a) = x(a, b).T
    private double[][] matrixTransport(double[][] matrix){
        int y = matrix.length;
        int x = matrix[0].length;

        double[][] newMatrix = new double[x][y];

        for(int i = 0; i < y; i++){
            for(int j = 0; j < x; j++){
                newMatrix[j][i] = matrix[i][j];
            }
        }

        return newMatrix;
    }


    ////////////////////////////行列の2乗////////////////////////////////////

    //res(n, n) = x(n, m) @ x(n, m).T
    private double[][] matrixSquareMul(double[][] a){
        int h = a.length;   
        int n = a[0].length; 
        double[][] c = new double[h][h];

        double[][] b = matrixTransport(a);
        for(int i = 0; i < h; i++){
            for(int j = 0; j < h; j++){
                c[i][j] = 0;
                for(int k = 0; k < n; k++){
                    c[i][j] += a[i][k]*b[k][j];
                }
            }
        }
        return c;
    }

    //res(n, n) = x(n) @ x(n).T
    private int[][] matrixSquareMul(int[] a){
        int h = a.length;   
        int[][] c = new int[h][h];

        for(int i = 0; i < h; i++){
            for(int j = 0; j < h; j++){
                c[i][j] = a[i]*a[j];
            }
        }
        return c;
    }



    ////////////////////////////////////////行列の掛け算////////////////////////////////////////

    //c(h, w) = a(h, n) @ b(n, w)
    private double[][] matrixMul(int[][] a, double[][] b){
        int h = a.length;
        int w = b[0].length;   
        int n = b.length; 
        double[][] c = new double[h][w];

        for(int i = 0; i < h; i++){
            for(int j = 0; j < w; j++){
                for(int k = 0; k < n; k++){
                    c[i][j] += a[i][k]*b[k][j];
                }
            }
        }

        return c;
    }

    //c(h) = a(h, n) @ b(n)
    private double[] matrixMul(double[][] a, double[] b){
        int h = a.length;   
        int n = b.length; 
        double[] c = new double[h];

        for(int i = 0; i < h; i++){
            for(int j = 0; j < n; j++){
                c[i] += a[i][j]*b[j];
            }
        }

        return c;
    }

    //c = a(h) @ b(h)
    private double matrixMul(double[] a, double[] b){
        int h = a.length;
        double c = 0;

        for(int i = 0; i < h; i++){
            c += a[i]*b[i];
        }

        return c;
    }

    //c(h) = a(h) * b
    private double[] matrixMul(double[] a, double b){
        int h = a.length;
        double[] c = new double[h];

        for(int i = 0; i < h; i++){
            c[i] = a[i] * b;
        }

        return c;
    }


    ////////////////////////////////////行列の引き算////////////////////////////////////////
    //c(h) = a(h) - b(h)
    private double[] matrixSub(double[] a, double[] b){
        int h = a.length;
        double[] c = new double[h];

        for(int i = 0; i < h; i++){
            c[i] = a[i] - b[i];
        }

        return c;
    }

    /////////////////////////////////////行列の足し算//////////////////////////////////////
    public static double[] matrixAdd(double[] a, double[] b){
        int h = a.length;
        double[] c = new double[h];

        for(int i = 0; i < h; i++){
            c[i] = a[i] + b[i];
        }

        return c;
    }

}
