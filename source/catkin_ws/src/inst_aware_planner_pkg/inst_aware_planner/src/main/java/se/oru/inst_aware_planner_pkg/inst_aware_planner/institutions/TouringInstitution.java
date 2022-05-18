package se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions;

import java.util.ArrayList;
import java.util.List;

import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Institution;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Institution.Act;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Institution.Role;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.utils.Triple;

public class TouringInstitution extends Institution<String, String, String> {

	public TouringInstitution() {
		super();
		
		// Institution Name
		this.setInstName("Touring");

		// Create Roles
		Role Tutor = new Role("Tutor");
		Role Helper = new Role("Helper");
		
		this.roleSet.put("Tutor", Tutor);
		this.setRoleCardinality(Tutor, 1, 1);
		this.roleSet.put("Helper", Helper);
		this.setRoleCardinality(Helper, 1, 1);
		
		// Create Acts
		Act Tutoring = new Act("Tutoring");
		Act ShowAround = new Act("ShowAround");
		
		this.actsSet.put("Tutoring", Tutoring);
		this.actsSet.put("ShowAround", ShowAround);
		
		// Create Arts
		Art Place = new Art("Place");
		Art ObjectsOfInterest = new Art("ObjectsOfInterest");
		
		this.artsSet.put("Place", Place);
		this.artsSet.put("ObjectsOfInterest", ObjectsOfInterest);
		
		// Relations in the institution
		Triple<Role,Act,Art> tTutoring = new Triple<>(Tutor, Tutoring, Place);
		Triple<Role,Act,Art> tShowAround = new Triple<>(Helper, ShowAround, ObjectsOfInterest);
		
		// OBLIGATION NORMS //
		
		this.addNormOBN("must", tTutoring); // must(Tutor, Tutoring, Place)
		this.addNormOBN("must", tShowAround); // must(Helper, ShowAround, ObjectsOfInterest)
		
		
		// MODAL NORMS //
		
		// use((Tutor, Tutoring, Place))
		List<Triple<Role,Act,?>> lUsePlace = new ArrayList<>(); 
		lUsePlace.add(tTutoring);
		this.addNormMOD("use", lUsePlace); 
		
		// use((Helper, ShowAround, ObjectsOfInterest))
		List<Triple<Role,Act,?>> lUseWaypoints = new ArrayList<>();
		lUseWaypoints.add(tShowAround);
		this.addNormMOD("use", lUseWaypoints);
		
		// starts( (Tutor, Tutoring, Place), (Helper, ShowAround, ObjectsOfInterest) )
		List<Triple<Role,Act,?>> lAfter = new ArrayList<>();
		lAfter.add(tTutoring);
		lAfter.add(tShowAround);
		this.addNormMOD("equals", lAfter);
	
	}

};
