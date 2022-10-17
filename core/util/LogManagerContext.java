package core.util;

public class LogManagerContext {
    
    private static ThreadLocal<LogManager2> logManagerThreadLocal = new ThreadLocal<LogManager2>();
    
    public static LogManager2 getLogManager(){
        return logManagerThreadLocal.get();
    }

    public static void setLogManager(LogManager2 logManager){
        logManagerThreadLocal.set(logManager);
    }

    public static void removeLogManager(){
        logManagerThreadLocal.remove();
    }
}
