package main;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.CountDownLatch;

import core.util.LogManager;
import core.util.LogManager2;
import main.simulations.AgentType;
import main.simulations.BehaviorType;
import test.CTest.CTestProcess;
import test.ETest.ETestProcess;
import test.ETest.ETestSimulation;
import test.ETest.ETestSimulationFactory;
import test.LTest.LTestProcess;
import test.MTest.MTestProcess;
import test.OTest.OTestProcess;
import test.PTest.PTestProcess;
import test.RTest.RTestProcess;
import test.RTest.RTestSimulation;
import test.RTest.RTestSimulationFactory;
import test.STest.STestProcess;
import test.TTest.TTestProcess;
import test.UTest.UTestProcess;
import tools.ETestGraphCsvExport;

public class App {
    boolean isAccumulate = true;
    static int runs = 5010000;
    static int scale = 50;
    static int min_robot_num = 20;
    static int max_robot_num = 20;

    /**
	 * patherNumber -> 0:Random / 1:Greedy / default:Subgoal
	 *
	 * targetterNumber -> 0:Random / 1:Greedy(PGS) / 2: Interval
	 * 3:MyopiaSweep(BNPS) / 4:AMTDS/LD(including 0~3) /
	 * 5:AMTDS/ESC / 6:Security
	 */

    static String[] environment = {"Uniform", "Block", "Complex", "Office", "Test", "Test2", "Test3"};
    static String[] targetter = {"Random", "Greedy", "Interval", "BNPS", "AMTDS-LD", "AMTDS-ESC", "Security"};
    
    static void eTestSimulate(int s, int counter) {
		int env = 1;
		String date = "";
		System.out.println(counter);

        //開始日時
		if(counter == 1){
			date = LocalDateTime.now().format(DateTimeFormatter.ofPattern("MM-dd HH-mm-ss"));
        }
		LogManager.setLogDirectory("log/" + environment[env] + "/" + date + counter);
		ETestSimulationFactory factory = new ETestSimulationFactory();
		factory.setLocalNumbers(s, scale, true, environment[env]);

        ETestSimulation simulation = new ETestSimulation(factory);
		simulation.reset();
		//環境表示テスト
		if(counter == 1){
    		ETestGraphCsvExport.exportCsv(factory, environment[env]);
        }
		simulation.run(runs);
	}

    static void eTestSimulationSingleThread(){
        int maxseed = 11;
        int counter = 0;

        for(int s = 10; s < maxseed; s++){
            counter++;
            eTestSimulate(s, counter);
        }
    }

    static void eTestSimulationMultiThread(){
        int counter = 0, env = 3;
        int maxseed = 20;
        String date = "";
        CountDownLatch startLatch = new CountDownLatch(1);
        List<Thread> threads = new ArrayList<Thread>();

        for(int s = 10; s < maxseed; s++){
            counter++;
            System.out.println(counter);
            //開始日時
            if(counter == 1){
                date = LocalDateTime.now().format(DateTimeFormatter.ofPattern("MM-dd HH-mm-ss"));
            }
            LogManager2 logManager = new LogManager2();

            Thread thread = new Thread(new ETestProcess(logManager, startLatch, runs, environment[env], date, counter, s, scale));
            thread.start();
            threads.add(thread);
        }

        //同時にスレッドを実行する
        startLatch.countDown();
        try{
            //スレッドが終了するのを待つ。
            for(Thread thread : threads){
                thread.join();
            }
        } catch(InterruptedException e){
            e.printStackTrace();
        }
    }

    static void rTestSimulationMultiThread(AgentType agentType, BehaviorType behaviorType){
        int counter = 0, env = 3;
        int t_from = 5, p = 2, t;
        int robots = 20;
        
        int maxseed = 15;
        String date = "";

        for(t = t_from; t < t_from+1; t++){
            CountDownLatch startLatch = new CountDownLatch(1);
            List<Thread> threads = new ArrayList<Thread>();

            for(int s = 10; s < maxseed; s++){
                counter++;
                System.out.println(counter);
                //開始時刻
                if(counter == 1){
                    date = LocalDateTime.now().format(DateTimeFormatter.ofPattern("MM-dd HH-mm-ss"));
                }
                LogManager2 logManager = new LogManager2();

                Thread thread = new Thread(new RTestProcess(logManager, startLatch, runs, agentType, environment[env], t, targetter[t],
                        date, robots, counter, s, p, scale, behaviorType));
                thread.start();
                threads.add(thread);
            }

            //同時にスレッドを実行する
            startLatch.countDown();
            try{
                //スレッドが終了するのを待つ
                for(Thread thread : threads){
                    thread.join();
                }
            } catch(InterruptedException e){
                e.printStackTrace();
            }
        }
    }

    static void rTestSimulation2(int s){
        scale = s;
        int env = 4;
        int robots = 1;
        runs = 16;

        String date = "";
		//開始日時
		date = LocalDateTime.now().format(DateTimeFormatter.ofPattern("MM-dd HH-mm-ss"));

		LogManager.setLogDirectory("log/test/" + environment[env] + "/" + date);
					RTestSimulationFactory factory = new RTestSimulationFactory();
					factory.setLocalNumbers(s, robots, scale, true, environment[env], BehaviorType.hand, AgentType.hand);

					RTestSimulation simulation = new RTestSimulation(factory);
					simulation.reset();
					simulation.run(runs);
    }

    static void cTestSimulationMultiThread(AgentType agentType, BehaviorType behaviorType){
        int counter = 0, env = 3;
        int t = 4, p = 2;
        int robots = 20;
        
        int maxseed = 20;
        String date = "";

        for(t = 4; t < 5; t++){
            CountDownLatch startLatch = new CountDownLatch(1);
            List<Thread> threads = new ArrayList<Thread>();

            for(int s = 10; s < maxseed; s++){
                counter++;
                System.out.println(counter);
                //開始時刻
                if(counter == 1){
                    date = LocalDateTime.now().format(DateTimeFormatter.ofPattern("MM-dd HH-mm-ss"));
                }
                LogManager2 logManager = new LogManager2();

                Thread thread = new Thread(new CTestProcess(logManager, startLatch, runs, agentType, environment[env], t, targetter[t],
                        date, robots, counter, s, p, scale, behaviorType));
                thread.start();
                threads.add(thread);
            }

            //同時にスレッドを実行する
            startLatch.countDown();
            try{
                //スレッドが終了するのを待つ
                for(Thread thread : threads){
                    thread.join();
                }
            } catch(InterruptedException e){
                e.printStackTrace();
            }
        }
    }
    
    static void sTestSimulationMultiThread(AgentType agentType, BehaviorType behaviorType){
        int counter = 0, env = 3;
        int t = 4, p = 2;
        int robots = 20;
        
        int maxseed = 20;
        String date = "";

        for(t = 4; t < 5; t++){
            CountDownLatch startLatch = new CountDownLatch(1);
            List<Thread> threads = new ArrayList<Thread>();

            for(int s = 10; s < maxseed; s++){
                counter++;
                System.out.println(counter);
                //開始時刻
                if(counter == 1){
                    date = LocalDateTime.now().format(DateTimeFormatter.ofPattern("MM-dd HH-mm-ss"));
                }
                LogManager2 logManager = new LogManager2();

                Thread thread = new Thread(new STestProcess(logManager, startLatch, runs, agentType, environment[env], t, targetter[t],
                        date, robots, counter, s, p, scale, behaviorType));
                thread.start();
                threads.add(thread);
            }

            //同時にスレッドを実行する
            startLatch.countDown();
            try{
                //スレッドが終了するのを待つ
                for(Thread thread : threads){
                    thread.join();
                }
            } catch(InterruptedException e){
                e.printStackTrace();
            }
        }
    }

    static void pTestSimulationMultiThread(AgentType agentType, BehaviorType behaviorType){
        int counter = 0, env = 3;
        int t_from = 4, p = 2, t;
        int robots = 20;
        
        int maxseed = 20;
        String date = "";

        for(t = t_from; t < t_from+1; t++){
            CountDownLatch startLatch = new CountDownLatch(1);
            List<Thread> threads = new ArrayList<Thread>();

            for(int s = 10; s < maxseed; s++){
                counter++;
                System.out.println(counter);
                //開始時刻
                if(counter == 1){
                    date = LocalDateTime.now().format(DateTimeFormatter.ofPattern("MM-dd HH-mm-ss"));
                }
                LogManager2 logManager = new LogManager2();

                Thread thread = new Thread(new PTestProcess(logManager, startLatch, runs, agentType, environment[env], t, targetter[t],
                        date, robots, counter, s, p, scale, behaviorType));
                thread.start();
                threads.add(thread);
            }

            //同時にスレッドを実行する
            startLatch.countDown();
            try{
                //スレッドが終了するのを待つ
                for(Thread thread : threads){
                    thread.join();
                }
            } catch(InterruptedException e){
                e.printStackTrace();
            }
        }
    }

    static void lTestSimulationMultiThread(AgentType agentType, BehaviorType behaviorType, double req){
        int counter = 0, env = 2;
        int t_from = 5, p = 2, t;
        int robots = 20;
        
        int maxseed = 20;
        String date = "";

        for(t = t_from; t < t_from+1; t++){
            CountDownLatch startLatch = new CountDownLatch(1);
            List<Thread> threads = new ArrayList<Thread>();

            for(int s = 10; s < maxseed; s++){
                counter++;
                System.out.println(counter);
                //開始時刻
                if(counter == 1){
                    date = LocalDateTime.now().format(DateTimeFormatter.ofPattern("MM-dd HH-mm-ss"));
                }
                LogManager2 logManager = new LogManager2();

                Thread thread = new Thread(new LTestProcess(logManager, startLatch, runs, agentType, environment[env], t, targetter[t],
                        date, robots, counter, s, p, scale, behaviorType, req));
                thread.start();
                threads.add(thread);
            }

            //同時にスレッドを実行する
            startLatch.countDown();
            try{
                //スレッドが終了するのを待つ
                for(Thread thread : threads){
                    thread.join();
                }
            } catch(InterruptedException e){
                e.printStackTrace();
            }
        }
    }
    
    static void tTestSimulationMultiThread(AgentType agentType, BehaviorType behaviorType, double req, double correction, int sseed, int eseed){
        int counter = 0, env = 3;
        int t_from = 5, p = 2, t;
        int robots = 20;
        
        int maxseed = eseed;
        String date = "";

        for(t = t_from; t < t_from+1; t++){
            CountDownLatch startLatch = new CountDownLatch(1);
            List<Thread> threads = new ArrayList<Thread>();

            for(int s = sseed; s < maxseed; s++){
                counter++;
                System.out.println(counter);
                //開始時刻
                if(counter == 1){
                    date = LocalDateTime.now().format(DateTimeFormatter.ofPattern("MM-dd HH-mm-ss"));
                }
                LogManager2 logManager = new LogManager2();

                Thread thread = new Thread(new TTestProcess(logManager, startLatch, runs, agentType, environment[env], t, targetter[t],
                        date, robots, counter, s, p, scale, behaviorType, req, correction));
                thread.start();
                threads.add(thread);
            }

            //同時にスレッドを実行する
            startLatch.countDown();
            try{
                //スレッドが終了するのを待つ
                for(Thread thread : threads){
                    thread.join();
                }
            } catch(InterruptedException e){
                e.printStackTrace();
            }
        }
    }
    
    static void oTestSimulationMultiThread(AgentType agentType, BehaviorType behaviorType, double req, double correction, int sseed, int eseed){
        int counter = 0, env = 3;
        int t_from = 5, p = 2, t;
        int robots = 20;
        
        int maxseed = eseed;
        String date = "";

        for(t = t_from; t < t_from+1; t++){
            CountDownLatch startLatch = new CountDownLatch(1);
            List<Thread> threads = new ArrayList<Thread>();

            for(int s = sseed; s < maxseed; s++){
                counter++;
                System.out.println(counter);
                //開始時刻
                if(counter == 1){
                    date = LocalDateTime.now().format(DateTimeFormatter.ofPattern("MM-dd HH-mm-ss"));
                }
                LogManager2 logManager = new LogManager2();

                Thread thread = new Thread(new OTestProcess(logManager, startLatch, runs, agentType, environment[env], t, targetter[t],
                        date, robots, counter, s, p, scale, behaviorType, req, correction));
                thread.start();
                threads.add(thread);
            }

            //同時にスレッドを実行する
            startLatch.countDown();
            try{
                //スレッドが終了するのを待つ
                for(Thread thread : threads){
                    thread.join();
                }
            } catch(InterruptedException e){
                e.printStackTrace();
            }
        }
    }

    static void mTestSimulationMultiThread(AgentType agentType, BehaviorType behaviorType, double req, double correction, int sseed, int eseed){
        int counter = 0, env = 3;
        int t_from = 5, p = 2, t;
        int robots = 20;
        
        int maxseed = eseed;
        String date = "";

        for(t = t_from; t < t_from+1; t++){
            CountDownLatch startLatch = new CountDownLatch(1);
            List<Thread> threads = new ArrayList<Thread>();

            for(int s = sseed; s < maxseed; s++){
                counter++;
                System.out.println(counter);
                //開始時刻
                if(counter == 1){
                    date = LocalDateTime.now().format(DateTimeFormatter.ofPattern("MM-dd HH-mm-ss"));
                }
                LogManager2 logManager = new LogManager2();

                Thread thread = new Thread(new MTestProcess(logManager, startLatch, runs, agentType, environment[env], t, targetter[t],
                        date, robots, counter, s, p, scale, behaviorType, req, correction));
                thread.start();
                threads.add(thread);
            }

            //同時にスレッドを実行する
            startLatch.countDown();
            try{
                //スレッドが終了するのを待つ
                for(Thread thread : threads){
                    thread.join();
                }
            } catch(InterruptedException e){
                e.printStackTrace();
            }
        }
    }

    static void uTestSimulationMultiThread(AgentType agentType, BehaviorType behaviorType, double req, double correction, int sseed, int eseed){
        int counter = 0, env = 3;
        int t_from = 5, p = 2, t;
        int robots = 20;
        List<Integer> list = new LinkedList<Integer>();
        list.add(13);
        list.add(14);
        list.add(17);
        
        int maxseed = eseed;
        String date = "";

        for(t = t_from; t < t_from+1; t++){
            CountDownLatch startLatch = new CountDownLatch(1);
            List<Thread> threads = new ArrayList<Thread>();

            for(int s = sseed; s < maxseed; s++){
            	if(list.contains(s)){
            		maxseed++;
            		continue;
            	}
                counter++;
                System.out.println(counter);
                //開始時刻
                if(counter == 1){
                    date = LocalDateTime.now().format(DateTimeFormatter.ofPattern("MM-dd HH-mm-ss"));
                }
                LogManager2 logManager = new LogManager2();

                Thread thread = new Thread(new UTestProcess(logManager, startLatch, runs, agentType, environment[env], t, targetter[t],
                        date, robots, counter, s, p, scale, behaviorType, req, correction));
                thread.start();
                threads.add(thread);
            }

            //同時にスレッドを実行する
            startLatch.countDown();
            try{
                //スレッドが終了するのを待つ
                for(Thread thread : threads){
                    thread.join();
                }
            } catch(InterruptedException e){
                e.printStackTrace();
            }
        }
    }

    public static void main(String[] args){
    	//eTestSimulationMultiThread();
        //eTestSimulationSingleThread();
        //rTestSimulationMultiThread(AgentType.PDALearning, BehaviorType.normal);
        //rTestSimulation2(2);
        //cTestSimulationMultiThread(AgentType.Communicating, BehaviorType.communicable);
    	//sTestSimulationMultiThread(AgentType.Communicating, BehaviorType.communicable);
    	//sTestSimulationMultiThread(AgentType.PDALearning, BehaviorType.normal);
        //pTestSimulationMultiThread(AgentType.PlannedStopping, BehaviorType.plannedStoppable);
        tTestSimulationMultiThread(AgentType.TimeChange_Learning, BehaviorType.normal, 1000, 0.5, 10, 15);
        //tTestSimulationMultiThread(AgentType.TimeChange_Communication, BehaviorType.normal, 600, 0.5, 10, 15);
    	//oTestSimulationMultiThread(AgentType.Onebyone, BehaviorType.onebyoneStoppable, 600, 1.0, 10, 35);
    	//oTestSimulationMultiThread(AgentType.Onebyone, BehaviorType.onebyoneStoppable, 600, 1.0, 35, 60);
    	//mTestSimulationMultiThread(AgentType.Onebyone, BehaviorType.multipleStoppable, 600, 1.0, 10, 35);
    	//mTestSimulationMultiThread(AgentType.Onebyone, BehaviorType.multipleStoppable, 600, 1.0, 35, 60);
    	//uTestSimulationMultiThread(AgentType.TimeChange_U, BehaviorType.normal, 15, 1.0, 10, 20);
    	
        //////////////////////////////////////////確認事項//////////////////////////////////////////////
        ////SubGoalPathPlanner, GreedyTargetDecide, ForMyopiaGreedy, ShortestGreedyPathPlanner, は既知の時はコメントアウトを外し////
        ////未知の場合はコメントアウトしているか確認                                                 /////   
        ///////////////////////////////////////////////////////////////////////////////////////////////
    	
        //終了
        System.out.println();
        System.out.println("DONE");
    }
}
