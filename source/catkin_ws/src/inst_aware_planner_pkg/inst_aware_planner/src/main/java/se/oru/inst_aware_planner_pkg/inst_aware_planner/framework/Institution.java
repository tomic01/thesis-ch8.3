package se.oru.inst_aware_planner_pkg.inst_aware_planner.framework;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import se.oru.inst_aware_planner_pkg.inst_aware_planner.utils.Pair;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.utils.Triple;

// Institution holds set of roles, norms, acts, artifacts and cardinality relations 
// It should be translated into SAP domain (not institution domain). 
public class Institution<T_ROLE, T_ACT, T_ART> {

	public class Act {
		private T_ACT act;

		public Act(T_ACT act) {
			this.act = act;
		}

		public T_ACT getAct() {
			return act;
		}
		
		@Override
		public String toString() {
			if (act instanceof String) {
				return (String) act;
			}
			
			// Default
			return toString();
		}
	}

	public class Art {
		private T_ART art;

		public Art(T_ART art) {
			this.art = art;
		}

		public T_ART getArt() {
			return art;
		}
		
		@Override
		public String toString() {
			if (art instanceof String) {
				return (String) art;
			}
			
			return toString();
		}

	}
	
	public class Role {
		private T_ROLE role;

		public Role(T_ROLE role) {
			this.role = role;
		}

		public T_ROLE getRole() {
			return role;
		}

		@Override
		public String toString() {
			if (role instanceof String) {
				return (String) role;
			}
			
			return toString();
		}
		
		
	}
	
//	public class Norm {
//		private String qualifier;
//		private List<Triple<Role,Act,?>> triples;
//		
//		public Norm(String qualifier, List<Triple<Role,Act,?>> triples) {
//			this.qualifier = qualifier;
//			this.triples = triples;
//		}
//		
//		@SuppressWarnings("unused")
//		private Norm() {}
//		
//	}

	// Name
	protected String instName;

	// Sets
	protected Map<String, Role> roleSet;
	protected Map<String, Act> actsSet;
	protected Map<String, Art> artsSet;

	// ***** NORMS ******//
	
	// Obligation Norms (OBN)
	protected List<Pair<String, Triple<Role,Act,?>>> normsOBN;
	// Modal Norms (MOD)
	protected List<Pair<String, List<Triple<Role,Act,?>>>> normsMOD;
	// NOTE: Not type safe, must be a better way...
	
	// Cardinality as a type of norm
	protected Map<Role, Pair<Integer, Integer>> roleCardinality;

	public Institution() {
		super();

		this.roleSet = new HashMap<String, Role>();
		this.roleCardinality = new HashMap<Role, Pair<Integer, Integer>>();
		this.actsSet = new HashMap<String, Act>();
		this.artsSet = new HashMap<String, Art>();
		
		this.normsOBN = new ArrayList<Pair<String, Triple<Role,Act,?>>>();
		this.normsMOD = new ArrayList<Pair<String, List<Triple<Role,Act,?>>>>();
		
	}
	
	public void addNormOBN(String qualifier, Triple<Role, Act, ?> triple) {
		Pair<String, Triple<Role,Act,?>> pair = new Pair<String, Triple<Role,Act,?>>(qualifier, triple);
		this.normsOBN.add(pair);
	}
	
	public void addNormMOD(String qualifier, List<Triple<Role,Act,?>> triples) {
		Pair<String, List<Triple<Role,Act,?>>> pair = new Pair<String, List<Triple<Role,Act,?>>>(qualifier, triples);
		this.normsMOD.add(pair);
	}
	

	/***** GETTERS ******/

	public String getInstName() {
		return instName;
	}
	
	public Map<String, Role> getRoleSet() {
		return roleSet;
	}

	public Map<String, Act> getActsSet() {
		return actsSet;
	}

	public Map<String, Art> getArtsSet() {
		return artsSet;
	}
	
	public List<Pair<String, Triple<Role,Act,?>>> getNormsOBN(){
		return normsOBN;
	}
	
	public List<Pair<String, List<Triple<Role,Act,?>>>> getNormsMOD() {
		return normsMOD;
	}

	public Integer getRoleMaxCard(Role role) {
		Pair<Integer, Integer> cardPair = roleCardinality.get(role);
		return cardPair.getSecond();
	}

	public Integer getRoleMinCard(Role role) {
		Pair<Integer, Integer> cardPair = roleCardinality.get(role);
		return cardPair.getFirst();
	}

	/***** SETTERS ******/
	
	public void setInstName(String instName) {
		this.instName = instName;
	}
	
	public void setRoleCardinality(Role role, Integer minCardinality, Integer maxCardinality) {
		if (minCardinality <= maxCardinality) {
			Pair<Integer, Integer> cardinality = new Pair<Integer, Integer>(minCardinality, maxCardinality);
			roleCardinality.put(role, cardinality);
		} else
			throw new Error("MinCardinality is higher than the max cardinality");
	}

}
