package se.oru.inst_aware_planner_pkg.inst_aware_planner.framework;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import se.oru.inst_aware_planner_pkg.inst_aware_planner.utils.Triple;

// Domain holds set of agents, their behaviors and objects in the environment
// It also holds relations between those sets = affordances 
// Each set can have elements with arbitrary types
public class InstDomain<T_A, T_B, T_O> {

	public class Agent {
		private T_A ag;

		public Agent(T_A agent) {
			this.ag = agent;
		}

		public T_A getAg() {
			return ag;
		}
	}
	public class Behavior {
		private T_B beh;

		public Behavior(T_B beh) {
			this.beh = beh;
		}

		public T_B getBeh() {
			return beh;
		}
	}

	public class Obj {
		private T_O obj;

		public Obj(T_O obj) {
			this.obj = obj;
		}

		public T_O getObj() {
			return obj;
		}
	}
	
	
	// SETS -- implemented as MAP for convenience
	protected Map<String, Agent> agentSet;
	protected Map<String, Behavior> behaviorSet;
	protected Map<String, Obj> objectSet;

	
	protected List<Triple<Agent, Behavior, ?>> affordances;
	
	public InstDomain() {
		super();

		this.agentSet = new HashMap<String, Agent>();
		this.behaviorSet = new HashMap<String, Behavior>();
		this.objectSet = new HashMap<String, Obj>();
		
		// Affordances
		this.affordances = new ArrayList<Triple<Agent, Behavior, ?>>();
	}
	
	public Map<String, Agent> getAgentSet() {
		return agentSet;
	}
	
	public Map<String, Behavior> getBehaviorSet() {
		return behaviorSet;
	}
	
	public Map<String, Obj> getObjectSet() {
		return objectSet;
	}

	public List<Triple<Agent, Behavior, ?>> getAffordances() {
		return affordances;
	}
	


}
