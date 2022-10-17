package test.ETest;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import core.Coordinate;
import core.GridGraph;
import core.IAgentManager;
import core.IEnvironment;
import core.LitterSpawnPattern;
import core.LitterSpawnProbability;
import main.simulations.Evaluator;
import main.simulations.SimulationFactory;

public class ETestSimulationFactory extends SimulationFactory{
    //ETest用のSimulationFactory

    static int[] seeds = new int[] { 31, 53, 79, 337, 601, 751, 907, 809, 977, 701, 1823, 2069, 2939, 3463, 4271, 5059,
        5779, 6569, 7499, 8287, 8999, 9337, 9901, 10007, 10459, 10949, 11443, 11933, 12829, 13309, 13781, 14029, 15013,
        15451, 15913, 16433, 17431, 17923, 18371, 18947, 19949, 20399, 20939, 21419, 22381, 22877, 23369, 23873, 24923,
        25447, 25943, 26437, 27509, 27997, 28547, 29017, 30103, 30643, 31151, 31657, 32653, 33179, 33637, 34213, 35251,
        35809, 36319, 36821, 37871, 38449, 38971, 39499, 40591, 41143, 41627, 42139, 43117, 43721, 44257, 44789, 45341,
        45943, 46489, 47057, 47581, 48661, 49177, 49711, 50221, 51361, 51853, 52391, 52967, 53527, 54049, 54581, 55163,
        55717, 56239, 57241, 57791, 58321, 59399, 59971, 59998 };

    ETestEnvironment environment;
    Evaluator evaluator;
    GridGraph graph;
    LitterSpawnPattern spawnPattern;

    private List<Integer> excludeNodes;
    private List<Integer> highNodes;
    private List<Integer> middleNodes;
    private List<Integer> lowNodes;
    private List<Integer> uniformNodes;

    int scale = 50;
    int steps = 3000000;

    boolean isAccumulate = true;

    String patternName = "Uniform";

    final double spawnLow = 1e-6, spawnMiddle = 1e-4, spawnHigh = 1e-3, spawnNone = 0.0;
    final double spawnUniform = 5e-6;

    int environmentSeed = 0;

    public ETestSimulationFactory(){
        excludeNodes = new LinkedList<Integer>();
        highNodes = new LinkedList<Integer>();
        middleNodes = new LinkedList<Integer>();
        lowNodes = new LinkedList<Integer>();
        uniformNodes = new LinkedList<Integer>();
    }

    public void setLocalNumbers(int eseed, int s, boolean isA, String pattern){
        environmentSeed = eseed;
        scale = s;
        isAccumulate = isA;
        patternName = pattern;
    }

    @Override
    public IEnvironment environment(){
        return this.environment;
    }

    public IAgentManager agentManager(){
        throw new IllegalStateException("This object doesn't support this method.");
    }

    public Evaluator evaluator(){
        return this.evaluator;
    }

    public GridGraph graph(){
        return this.graph;
    }

    public int scale(){
        return this.scale;
    }

    public List<Integer> excludeNodes(){
        return this.excludeNodes;
    }

    public List<Integer> highNodes(){
        return this.highNodes;
    }

    public List<Integer> middleNodes(){
        return this.middleNodes;
    }

    public List<Integer> lowNodes(){
        return this.lowNodes;
    }

    public List<Integer> uniformNodes(){
        return this.uniformNodes;
    }

    public void make(){
        createSpatialStructure();
        createLitterSpawnPattern();

        environment = new ETestEnvironment(graph, spawnPattern, isAccumulate, seeds[environmentSeed]);

        environment.update();
        createEvaluator();

        //ETest用期待値出力
        printExpectedValue();
    }

    public void createSpatialStructure(){
        Coordinate max = new Coordinate(scale, scale);
        Coordinate min = new Coordinate(-scale, -scale);

        graph = new GridGraph(max, min);
    }

    public void createLitterSpawnPattern(){
        Random rand = new Random(seeds[environmentSeed + 1]);
        spawnPattern = new LitterSpawnPattern();

        for(int node : graph.getAllNode()){
            Coordinate c = graph.getCoordinate(node);
            double prob;

            if(patternName == "Block") {
                if((-50 <= c.x && c.x <= -1 && 1<= c.y && c.y <= 50) || (1 <= c.x && c.x <= 50 && -50 <= c.y && c.y <= -1)){
                    prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                    middleNodes.add(node);
                }
                else if(1 <= c.x && c.x <= 50 && 1 <= c.y && c.y <= 50){
                    prob = spawnHigh + (rand.nextDouble() - 0.5) * spawnHigh * 0.01;
                	highNodes.add(node);
                }
                else{
                    prob = spawnLow + (rand.nextDouble() - 0.5) * spawnLow * 0.01;
					lowNodes.add(node);
                }
            }

            else if(patternName == "Complex"){
                //壁相当ノードの除外リスト
                if((-50 <= c.x && c.x <= -31 && c.y == 6) || (-50 <= c.x && c.x <= -31 && c.y == -6) || (-20 <= c.x && c.x <= 20 && c.y == 6) || 
                    (31 <= c.x && c.x <= 50 && c.y == 6) || (-20 <= c.x && c.x <= 20 && c.y == -6) || (31 <= c.x && c.x <= 50 && c.y == -6) || 
                    (-20 <= c.x && c.x <= 20 && c.y == 6) || (c.x == 0 && 7 <= c.y && c.y <= 50) || (c.x == 0 && -50 <= c.y && c.y <= -7)){
                    prob = spawnNone;
                    excludeNodes.add(node);
                }

                //左上の部屋
                else if((-50 <= c.x && c.x <= -1 && 46 <= c.y && c.y <= 50) || (-50 <= c.x && c.x <= -46 && 12 <= c.y && c.y <= 45) || 
                    (-5 <= c.x && c.x <= -1 && 12 <= c.y && c.y <= 45) || (-50 <= c.x && c.x <= -31 && 7 <= c.y && c.y <= 11) || 
                    (-20 <= c.x && c.x <= -1 && 7 <= c.y && c.y <= 11)){
                        prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                        middleNodes.add(node);
                }

                //右上の部屋
                else if(5 <= c.x && c.x <= 9 && 41 <= c.y && c.y <=45){
                    prob = spawnHigh + (rand.nextDouble() - 0.5) * spawnHigh * 0.01;
                	highNodes.add(node);
                }

                else if((15 <= c.x && c.x <= 19 && 17 <= c.y && c.y <= 21) || (30 <= c.x && c.x <= 34 && 31 <= c.y && c.y <= 35) || (41 <= c.x && c.x <= 45 && 10 <= c.y && c.y <= 14)){
                    prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                    middleNodes.add(node);
                } 

                //左下の部屋
                else if(-30 <= c.x && c.x <= -21 && -33 <= c.y && c.y <= -24){
                    prob = spawnHigh + (rand.nextDouble() - 0.5) * spawnHigh * 0.01;
                	highNodes.add(node);
                }

                else{
                    prob = spawnLow + (rand.nextDouble() - 0.5) * spawnLow * 0.01;
                	lowNodes.add(node);
                }
            }

            else if(patternName == "Office") {
				//壁相当ノードの除外リスト
                if ((c.x == -17 && 5 <= c.y) || (c.x == -17 && c.y <= -5) || (c.x == 17 && 5 <= c.y) || (c.x == 17 && c.y <= -5) || (-50 <= c.x && c.x <= -37 && c.y == -5) ||
                    (-31 <= c.x && c.x <= -3 && c.y == -5) || (3 <= c.x && c.x <= 31 && c.y == -5) || (37 <= c.x && c.x <= 50 && c.y == -5) || (-50 <= c.x && c.x <= -37 && c.y == 5) ||
                    (-31 <= c.x && c.x <= -3 && c.y == 5) || (3 <= c.x && c.x <= 31 && c.y == 5) || (37 <= c.x && c.x <= 50 && c.y == 5)) {
                	prob = spawnNone;
                	excludeNodes.add(node);
                }

                //左上の部屋
                else if ((c.x <= -47 && 6 <= c.y) || (-21 <= c.x && c.x <= -18 && 6 <= c.y) || (-46 <= c.x && c.x <= -22 && 47 <= c.y) ||
                    (-46 <= c.x && c.x <= -37 && 6 <= c.y && c.y <= 9) || (-31 <= c.x && c.x <= -18 && 6 <= c.y && c.y <= 9)) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                //上の部屋
                else if (-11 <= c.x && c.x <= -7 && 41 <= c.y && c.y <= 45) {
                	prob = spawnHigh + (rand.nextDouble() - 0.5) * spawnHigh * 0.01;
                	highNodes.add(node);
                }

                else if (3 <= c.x && c.x <= 7 && 25 <= c.y && c.y <= 29) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                //右上の部屋
                else if (19 <= c.x && c.x <= 23 && 15 <= c.y && c.y <= 19) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                else if (23 <= c.x && c.x <= 27 && 7 <= c.y && c.y <= 11) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                else if (24 <= c.x && c.x <= 28 && 33 <= c.y && c.y <= 37) {
                	prob = spawnHigh + (rand.nextDouble() - 0.5) * spawnHigh * 0.01;
                	highNodes.add(node);
                }

                else if (38 <= c.x && c.x <= 42 && 17 <= c.y && c.y <= 21) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                else if (40 <= c.x && c.x <= 44 && 41 <= c.y && c.y <= 45) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                //左下の部屋
                else if (-41 <= c.x && c.x <= -28 && -37 <= c.y && c.y <= -19) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                //下の部屋
                else if ((-16 <= c.x && c.x <= -13 && c.y <= -6) || (-12 <= c.x && c.x <= 16 && c.y <= -47) ||
                         (13 <= c.x && c.x <= 16 && c.y <= -6) || (-12 <= c.x && c.x <= -3 && -9 <= c.y && c.y <= -6) ||
                         (3 <= c.x && c.x <= 16 && -9 <= c.y && c.y <= -6)) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                else if (-8 <= c.x && c.x <= -4 && -20 <= c.y && c.y <= -16) {
                	prob = spawnMiddle + (rand.nextDouble() - 0.5) * spawnMiddle * 0.01;
                	middleNodes.add(node);
                }

                else if (2 <= c.x && c.x <= 6 && -38 <= c.y && c.y <= -34) {
                	prob = spawnHigh + (rand.nextDouble() - 0.5) * spawnHigh * 0.01;
                	highNodes.add(node);
                }

                //その他
                else {
                	prob = spawnLow + (rand.nextDouble() - 0.5) * spawnLow * 0.01;
                	lowNodes.add(node);
                }
			}
            else{
                prob = spawnUniform * rand.nextDouble();
                uniformNodes.add(node);
            } 
            spawnPattern.addSpawnProbability(node, new LitterSpawnProbability(1, prob, 1));           
        }
        removeExcludeEdges();
    }

    private void removeExcludeEdges(){
        for(int node : excludeNodes){
            List<Integer> tmp = new LinkedList<Integer>();
            for(int endPoint : graph.getChildNodes(node)){
                tmp.add(endPoint);
            }
            for(int end : tmp){
                graph.removeEdge(node, end);
                graph.removeEdge(end, node);
            }
        }
    }

    public void createEvaluator(){
        evaluator = new Evaluator(environment);
    }

    public void printExpectedValue(){
        double value = 0.0;
        for(LitterSpawnProbability prob : spawnPattern.getAllPatterns().values()){
            value += prob.getProbability();
        }
        value *= steps;
        System.out.println("ExpectValue : " + value);
    }
}
