package se.oru.inst_aware_planner_pkg.inst_aware_planner.framework;

import java.util.List;

import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.InstDomain.Agent;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.InstDomain.Behavior;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.InstDomain.Obj;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Institution.Act;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Institution.Art;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Institution.Role;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.utils.Pair;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.utils.Relation;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.utils.Triple;

@SuppressWarnings("rawtypes")
public class Grounding {

	public Relation<Role, Agent> Ga;
	public Relation<Act, Behavior> Gb;
	public Relation<Art, Obj> Go;

	private Institution institution = null;
	private InstDomain instDomain = null;

	@SuppressWarnings("unused")
	private Grounding() {
	}

	public Grounding(Institution institution, InstDomain instDomain) {
		super();
		this.institution = institution;
		this.instDomain = instDomain;

		Ga = new Relation<Role, Agent>();
		Gb = new Relation<Act, Behavior>();
		Go = new Relation<Art, Obj>();
	}

	// Get the existing role and agent from the institution by the name and add
	protected void addRelationGa(String roleToGet, String agentToGet) {
		Role role = (Role) institution.getRoleSet().get(roleToGet);
		Agent ag = (Agent) instDomain.getAgentSet().get(agentToGet);

		if (role == null) {
			throw new Error("There is no Role with the name: " + roleToGet);
		} else if (ag == null) {
			throw new Error("There is no Agent with the name: " + agentToGet);
		}

		Ga.addRelation(role, ag);
	}

	protected void addRelationGb(String actToGet, String behToGet) {
		Act act = (Act) this.institution.getActsSet().get(actToGet);
		Behavior beh = (Behavior) this.instDomain.getBehaviorSet().get(behToGet);

		if (act == null) {
			throw new Error("There is no Role with the name: " + actToGet);
		} else if (beh == null) {
			throw new Error("There is no Agent with the name: " + behToGet);
		}

		Gb.addRelation(act, beh);
	}

	protected void addRelationGo(String artToGet, String objToGet) {
		Art art = (Art) this.institution.getArtsSet().get(artToGet);
		Obj obj = (Obj) this.instDomain.getObjectSet().get(objToGet);

		if (art == null) {
			throw new Error("There is no Role with the name" + artToGet);
		} else if (obj == null) {
			throw new Error("There is no Agent with the name" + objToGet);
		}

		Go.addRelation(art, obj);
	}

	@SuppressWarnings("unchecked")
	private boolean checkCardinality(Role role) {
		Integer roleCard = Ga.countKeyCardinality(role);
		Integer minCard = institution.getRoleMinCard(role);
		Integer maxCard = institution.getRoleMaxCard(role);

		if (roleCard < minCard || roleCard > maxCard) {
			System.out.println("Cardinality fails! \nRole: " + role.getRole().toString() + "\nRole Card: "
					+ roleCard.toString() + "\nMin Card: " + minCard.toString() + "\nMax Card: " + maxCard.toString());
			return false;
		}

		return true;
	}

	@SuppressWarnings("unchecked")
	public boolean isAdmissibleGrounding() {

		List<Pair<String, Triple<Role, Act, ?>>> normsOBN = this.institution.getNormsOBN();

		for (Pair<String, Triple<Role, Act, ?>> norm : normsOBN) {
			
			System.out.println("Checking norm: " + norm.getFirst() + norm.getSecond().toString());
			
			Role normRole = norm.getSecond().getFirst();
			if (!checkCardinality(normRole))
				return false;
			
			Act act = norm.getSecond().getSecond();
			Object artOrRole = norm.getSecond().getThird();
			
			List<Agent> groundedAgents = Ga.getAllSeconds(normRole);
			for (Agent ag : groundedAgents) {
				if (!capable(ag, act, artOrRole)) {
					return false;
				}
			}

			System.out.println("--");
		}
		
		System.out.println();
		return true;
	}

	private boolean capable(Agent ag, Act act, Object artOrRole) {
		
		for (Pair<Act, Behavior> actBehavior : Gb.getRelation()) {
			Behavior beh = actBehavior.getSecond();
			if (artOrRole instanceof Art) {
				for (Pair<Art, Obj> artObj : Go.getRelation()) {
					Obj obj = artObj.getSecond();
					if (instDomain.affordances.contains( new Triple<Agent,Behavior,Obj>(ag,beh,obj))) {
						return true;
					}
				}
			} else if (artOrRole instanceof Role) {
				for (Pair<Role, Agent> roleAg : Ga.getRelation()) {
					Agent ag2 = roleAg.getSecond();
					if (instDomain.affordances.contains(new Triple<Agent,Behavior,Agent>(ag,beh,ag2))) {
						return true;
					}
					
				}
			}

		}
		
		return false;
	}

	public List<Agent> getAgentsFromGa(Role rol) {
		return Ga.getAllSeconds(rol);
	}

	public List<Role> getRolesFromGa(Agent ag) {
		return Ga.getAllFirsts(ag);
	}

	public List<Behavior> getBehFromGb(Act act) {
		return Gb.getAllSeconds(act);
	}

	public List<Act> getActFromGb(Behavior beh) {
		return Gb.getAllFirsts(beh);
	}

	public List<Obj> getObjFromGo(Art art) {
		return Go.getAllSeconds(art);
	}

	public List<Art> getArtFromGo(Obj obj) {
		return Go.getAllFirsts(obj);
	}

}