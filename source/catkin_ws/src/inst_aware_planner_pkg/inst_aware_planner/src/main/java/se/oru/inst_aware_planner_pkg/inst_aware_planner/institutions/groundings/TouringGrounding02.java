package se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.groundings;

import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Grounding;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Institution;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.TheDomain;

public class TouringGrounding02 extends Grounding {

	@SuppressWarnings("rawtypes")
	public TouringGrounding02(Institution instTouring, TheDomain instDomain) {
		super(instTouring, instDomain);

		// Ga
		addRelationGa("Tutor", "pepper01");
		addRelationGa("Helper", "mbot11");
		
		// Gb
		addRelationGb("Tutoring", "describe");
		addRelationGb("ShowAround", "wayPointNavigation");
		
		// Go
		addRelationGo("Place", "lab");
		addRelationGo("ObjectsOfInterest", "labStart,girafe,tracker,labExit");

	}

}
