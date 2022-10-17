package core.util;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

public class LogManager {
    static String logDirectory;
    static int defaultCapacity = 10;
    static Map<String, LogWriter> writers = new HashMap<String, LogWriter>();
    static Map<String, LogWriter2> writers2 = new HashMap<String, LogWriter2>();
    
    public static void setLogDirectory(String path){
        File dir = new File("./" + path);
        if(!dir.exists()){
            dir.mkdirs();
        }

        System.out.println(path);

        logDirectory = "./" + path + "/";
        writers.clear();
        writers2.clear();
    }

    public static String makeDir(String path){
        File dir = new File(logDirectory + path);
        if(!dir.exists()){
            dir.mkdirs();
        }

        return path;
    }

    //ログ取得が低いもの
    public static LogWriter createWriter(String key){
        if(writers.containsKey(key)){
            if(writers.get(key).isDisposed() == false){
                return writers.get(key);
            }
            writers.remove(key);
        }
        LogWriter writer = new LogWriter(logDirectory + key + ".csv", defaultCapacity);
        writers.put(key, writer);
        return writer;
    }

    //ログ取得頻度が高いもの
    public static LogWriter2 createWriter2(String key){
        if(writers2.containsKey(key)){
            if(writers2.get(key).isDisposed() == false){
                return writers2.get(key);
            }
            writers2.remove(key);
        }
        LogWriter2 writer = new LogWriter2(logDirectory + key +".csv");
        writers2.put(key, writer);
        return writer;
    }
}
