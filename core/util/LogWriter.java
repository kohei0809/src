package core.util;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;;

public class LogWriter {
    //ログを書くプログラム(マルチスレッド仕様?)

    Object locker = new Object();
    Object filelock = new Object();
    Object savelock = new Object();
    boolean disposed = false;

    String filePath;
    String[] buffer;
    String[] saveBuffer;
    int counter, capacity;

    public LogWriter(String path, int c){
        buffer = new String[c];
        counter = 0;
        capacity = c;
        filePath = path;
    }

    public boolean isDisposed(){
        synchronized(locker){
            return disposed;
        }
    }

    public void writeLine(String log){
        if(buffer.length <= counter){
            saveBuffer = buffer;
            buffer = new String[capacity];
            counter = 0;

            Runnable runnable = new Runnable() {
                public void run(){
                    saveToFile();
                }
            };
            Thread thread = new Thread(runnable);
            thread.start();
        }

        buffer[counter] = log;
        counter++;
    }

    public void writeLineWithLock(String log){
        synchronized(locker){
            writeLine(log);
        }
    }

    public void save(){
        saveBuffer = new String[counter];
        for(int i = 0; i < counter; i++){
            saveBuffer[i] = buffer[i];
        }
        saveToFile();
    }

    public void saveToFile(){
        synchronized(locker){
            String[] array = saveBuffer;
            if(saveBuffer == null){
                return;
            }
            saveBuffer =  null;

            FileOutputStream writer = null;
            try{
                File file = new File(filePath);
                //ファイルが存在しない場合，作成
                if(!file.exists()){
                    file.createNewFile();
                }
                writer = new FileOutputStream(filePath, true);
                for(String line : array){
                    writer.write(line.getBytes());
                }
                writer.flush();
            } catch(IOException e){
                e.printStackTrace();
            } finally{
                try{
                    if(writer != null){
                        writer.close();
                    }
                } catch(IOException e){
                    e.printStackTrace();
                }
            }
        }
    }
}
