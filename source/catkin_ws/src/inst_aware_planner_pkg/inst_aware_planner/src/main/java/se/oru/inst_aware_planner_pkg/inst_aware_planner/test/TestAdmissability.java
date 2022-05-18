package se.oru.inst_aware_planner_pkg.inst_aware_planner.test;

import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.TheDomain;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.TouringInstitution;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.groundings.TouringGrounding01;

public class TestAdmissability {

	public static void main(String[] args) {
		// Create the institution
		TouringInstitution touringInst = new TouringInstitution();

		// Create the domain
		TheDomain theDomain = new TheDomain();

		// Create the grounding
		TouringGrounding01 theGrounding01 = new TouringGrounding01(touringInst, theDomain);

		// // Create another grounding
		// TheGameGroundingTurtlebot1 theGrounding02 = new
		// TheGameGroundingTurtlebot1(theGame, theDomain);
		//
		
		// Check if the grounding is admissible
		if (theGrounding01.isAdmissibleGrounding()) {
			System.out.println("\nAdmissible!!!");
		} else {
			System.out.println("\nNOT Admissible!!!");
		}

		// if (theGrounding02.isAdmissibleGrounding()) {
		// System.out.println("\nAdmissible 02!!!");
		// } else {
		// System.out.println("\nNOT Admissible 02!!!");
		// }

	}

}
