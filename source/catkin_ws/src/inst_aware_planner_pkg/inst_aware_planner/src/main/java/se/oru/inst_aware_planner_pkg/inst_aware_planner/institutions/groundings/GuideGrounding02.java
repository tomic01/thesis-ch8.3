package se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.groundings;

import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Grounding;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Institution;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.GuidingInstitution;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.TheDomain;

public class GuideGrounding02 extends Grounding {

	public GuideGrounding02(Institution instGuide, TheDomain instDomain) {
		super(instGuide, instDomain);

		// Ga
		addRelationGa("Guide", "mbot11");
		addRelationGa("Helper", "human01");
		
		// Gb
		addRelationGb("DoGuidance", "moveOnTrajectory");
		addRelationGb("DoGuidanceHelp", "moveInFormation");
		
		// Go
		addRelationGo("Place", "map");

	}

}
