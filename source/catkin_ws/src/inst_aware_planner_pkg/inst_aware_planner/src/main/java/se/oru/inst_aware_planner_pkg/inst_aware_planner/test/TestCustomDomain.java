package se.oru.inst_aware_planner_pkg.inst_aware_planner.test;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.meta.simplePlanner.SimplePlanner;
import org.metacsp.multi.activity.ActivityNetworkSolver;

import se.oru.inst_aware_planner_pkg.inst_aware_planner.csp.InstitutionAwareDomain;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.GuidingInstitution;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.TheDomain;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.TouringInstitution;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.groundings.GuideGrounding01;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.groundings.TouringGrounding01;

//import se.oru.socially_aware_planner_pkg.im.action.DynamicDomainDomainGoals;
//import se.oru.socially_aware_planner_pkg.im.action.DynamicDomainInstitution;
//import se.oru.socially_aware_planner_pkg.im.action.DynamicDomainDomain;

public class TestCustomDomain {

	public static void main(String[] args) {

		// Create the institution
		TouringInstitution instTouring = new TouringInstitution();
		GuidingInstitution instGuide = new GuidingInstitution();
		// Create the domain
		TheDomain theDomain = new TheDomain();
		// Create the grounding
		TouringGrounding01 groundTour01 = new TouringGrounding01(instTouring, theDomain);
		GuideGrounding01 groundGuide01 = new GuideGrounding01(instGuide, theDomain);

		SimplePlanner planner = new SimplePlanner(1000, 100000, 0);
		
		new InstitutionAwareDomain(planner, instTouring, theDomain, groundTour01);
		//new InstitutionAwareDomain(planner, instGuide, theDomain, groundGuide01,"./");

		final ActivityNetworkSolver activitySolver = (ActivityNetworkSolver) planner.getConstraintSolvers()[0];
		ConstraintNetwork.draw(activitySolver.getConstraintNetwork());

		return;

	}

}
