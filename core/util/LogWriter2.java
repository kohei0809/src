package core.util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class LogWriter2 {
    //ログを書くプログラム

    private File file;

    public LogWriter2(String filePath){
        file = new File(filePath);
    }

    public boolean isDisposed(){
        return false;
    }

    //改行なし
    public void write(String log){
        //ファイルが存在しない場合，作成
        if(!file.exists()){
            try{
                file.createNewFile();
            } catch(IOException e){
                e.printStackTrace();
            }
        }

        try{//ファイルへの出力
            //追記モード
            FileWriter fw = new FileWriter(file, true);
            PrintWriter pw = new PrintWriter(new BufferedWriter(fw));

            pw.print(log);
            pw.close();
        } catch(IOException e){
            e.printStackTrace();
        }
    }

    //改行あり
    public void writeLine(String log){
        //ファイルが存在しない場合，作成
        if(!file.exists()){
            try{
                file.createNewFile();
            } catch(IOException e){
                e.printStackTrace();
            }
        }

        //ファイルへの出力
        try{
            //追記モード
            FileWriter fw = new FileWriter(file, true);
            PrintWriter pw = new PrintWriter(new BufferedWriter(fw));

            pw.println(log);
            pw.close();
        } catch(IOException e){
            e.printStackTrace();
        }
    }

    //まとめて書き込み(1行分)
    public void writeLineBlock(String[] logs){
        //ファイルが存在しない場合，作成
        if(!file.exists()){
            try{
                file.createNewFile();
            } catch(IOException e){
                e.printStackTrace();
            }
        }

        //ファイルへの出力
        try{
            //追記モード
            FileWriter fw = new FileWriter(file, true);
            PrintWriter pw = new PrintWriter(new BufferedWriter(fw));

            String[] lgs = new String[logs.length];
            System.arraycopy(logs, 0, lgs, 0, lgs.length);

            int i = 0;
            for(String log : lgs){
                pw.print(log);
                if(i != lgs.length-1){
                    pw.print(",");
                }
                i++;
            }
            pw.println();
            pw.close();
        } catch(IOException e){
            e.printStackTrace();
        }
    }

    //まとめて書き込み(2次元配列)
    public void writeLineMatrix(String[][] logs){
        //ファイルが存在しない場合，作成
        if(!file.exists()){
            try{
                file.createNewFile();
            } catch(IOException e){
                e.printStackTrace();
            }
        }

        //ファイルへの出力
        try{
            //追記モード
            FileWriter fw = new FileWriter(file, true);
            PrintWriter pw = new PrintWriter(new BufferedWriter(fw));

            String[][] lgs = new String[logs.length][];
            for(int i = 0; i < logs.length; i++){
                lgs[i] = new String[logs[i].length];
                System.arraycopy(logs[i], 0, lgs[i], 0, logs[i].length);
            }

            for(String[] log : lgs){
                int i = 0;
                for(String l : log){
                    pw.print(l);
                    if(i != log.length-1){
                        pw.print(",");
                    }
                    i++;
                }
                pw.println();
            }
            pw.close();
        } catch(IOException e){
            e.printStackTrace();
        }
    }
}
