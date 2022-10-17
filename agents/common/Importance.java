package agents.common;

import java.util.ArrayList;
import java.util.LinkedList;

import core.GridGraph;
import core.agent.AgentActions;
import core.util.DijkstraAlgorithm;

// Contribution Degree of Agent: for performance self-monitoring
public class Importance {

	private LinkedList<Integer> record;
	private DijkstraAlgorithm dijk;
	private int Ls = 20, Ll = 50, Lf = 10;
	private double futureExp;

	public Importance(GridGraph graph) {
		record = new LinkedList<Integer>();
		dijk = new DijkstraAlgorithm(graph);
	}

	public void update(AgentActions action, int vacuumedLitter) {
		if (action == AgentActions.move) {
			record.offerFirst(vacuumedLitter);
			if (record.size() > Ll)
				record.pollLast();
		}
	}

	public void update_future(int position, int target, LitterExistingExpectation expectation, int time) {
		futureExp = 0.0;
		if (Lf == 0)
			return;

		dijk.execute(target, position);
		ArrayList<Integer> route = dijk.getRoute(position, target);

		Lf = route.size() < Lf ? route.size() : Lf;
		for (int i = 0; i < Lf; i++) {
			futureExp += expectation.getExpectation(route.get(i), time + i + 1);
		}
		futureExp /= Lf;
	}

	public double evaluate() {
		// calculate short-term and long-term performance
		double shortTerm = 0.0, longTerm = 0.0;
		int ls = Ls, ll = Ll;
		int size = record.size();
		
		if(ls >= size){
			ls = size;
		}
		if(ll >= size){
			ll = size;
		}
		
		for (int i = 0; i < ls; i++){
			shortTerm += record.get(i);
		}
			
		for (int i = 0; i < ll; i++){
			longTerm += record.get(i);
		}
		shortTerm /= ls;
		longTerm /= ll;

		if ((shortTerm + futureExp) >= longTerm)
			return 1.0;
		else if (longTerm == 0)
			return 0.0;
		else
			return (shortTerm + futureExp) / longTerm;
	}
}

