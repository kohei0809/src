package main.simulations;

import core.GridGraph;
import core.IAgentManager;
import core.IEnvironment;


public abstract class SimulationFactory {
    public abstract IEnvironment environment();

    public abstract IAgentManager agentManager();

    public abstract Evaluator evaluator();

    //Create elements for simulation
    public abstract void make();

    public abstract GridGraph graph();
}
