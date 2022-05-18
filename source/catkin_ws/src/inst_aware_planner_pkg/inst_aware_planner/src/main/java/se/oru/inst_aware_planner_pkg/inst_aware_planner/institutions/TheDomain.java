package se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions;

import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.InstDomain;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.utils.Triple;

public class TheDomain extends InstDomain<String, String, String> {
	
	public TheDomain() {
		super();
		
		// Agents
		Agent aMbot11 = new Agent("mbot11");
		Agent aMbot03 = new Agent("mbot03");
		Agent aPepper01 = new Agent("pepper01");
		Agent aHuman01 = new Agent("human01");
		Agent aHuman02 = new Agent("human02");
		Agent aHuman03 = new Agent("human03");
		Agent aHuman04 = new Agent("humanPer");
		
		this.agentSet.put("mbot11", aMbot11);
		this.agentSet.put("mbot03", aMbot03);
		this.agentSet.put("pepper01", aPepper01);
		this.agentSet.put("human01", aHuman01);
		this.agentSet.put("human02", aHuman02);
		this.agentSet.put("human03", aHuman03);
		this.agentSet.put("humanPer", aHuman04);
		
		// Behaviors
		Behavior bMoveOnTrajectory = new Behavior("moveOnTrajectory");
		Behavior bDescribe = new Behavior("describe");
		Behavior bWayPointNavigation = new Behavior("wayPointNavigation");
		Behavior bMoveInFormation = new Behavior("moveInFormation");
		
		this.behaviorSet.put("moveOnTrajectory", bMoveOnTrajectory);
		this.behaviorSet.put("describe", bDescribe);
		this.behaviorSet.put("wayPointNavigation", bWayPointNavigation);
		this.behaviorSet.put("moveInFormation", bMoveInFormation);
		
		// Objects (parameters)
		Obj oPositionsLab = new Obj("labStart,vr,tracker,labExit"); // array of waypoints 
		Obj oPositionsLab2 = new Obj("labStart,girafe,tracker,labExit"); // array of waypoints
		Obj oLab = new Obj("lab");
		Obj oPeisHome = new Obj("peisHome");
		Obj oMap = new Obj("map");
		
		this.objectSet.put("labStart,vr,tracker,labExit", oPositionsLab);
		this.objectSet.put("labStart,girafe,tracker,labExit", oPositionsLab2);
		this.objectSet.put("lab", oLab);
		this.objectSet.put("peisHome", oPeisHome);
		this.objectSet.put("map", oMap);
		
		// Affordances
		this.affordances.add(new Triple<>(aMbot11, bMoveOnTrajectory, oMap));
		this.affordances.add(new Triple<>(aMbot03, bMoveInFormation, aMbot11));
		this.affordances.add(new Triple<>(aPepper01, bDescribe, oLab));
		this.affordances.add(new Triple<>(aPepper01, bDescribe, oPeisHome));
		this.affordances.add(new Triple<>(aMbot11, bWayPointNavigation, oPositionsLab)); //
		this.affordances.add(new Triple<>(aMbot11, bWayPointNavigation, oPositionsLab2)); //
		this.affordances.add(new Triple<>(aHuman04, bDescribe, oLab));
		this.affordances.add(new Triple<>(aHuman01, bMoveInFormation, aMbot11));
		
		

	}

}
