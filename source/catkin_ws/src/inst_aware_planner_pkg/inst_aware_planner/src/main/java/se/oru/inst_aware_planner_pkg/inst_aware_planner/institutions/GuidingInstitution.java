package se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions;

import java.util.ArrayList;
import java.util.List;

import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Institution;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.utils.Triple;

public class GuidingInstitution extends Institution<String, String, String> {

	public GuidingInstitution() {
		super();

		// Institution Name
		this.setInstName("Guiding");

		// Create Roles
		Role Guide = new Role("Guide");
		Role Helper = new Role("Helper");
		Role Humans = new Role("Humans");
		Role NonCoopHuman = new Role("NonCoopHuman");

		this.roleSet.put("Guide", Guide);
		this.setRoleCardinality(Guide, 1, 1);
		this.roleSet.put("Helper", Helper);
		this.setRoleCardinality(Helper, 1, 1);

		// Create Acts
		Act DoGuidance = new Act("DoGuidance");
		Act DoGuidanceHelp = new Act("DoGuidanceHelp");

		this.actsSet.put("DoGuidance", DoGuidance);
		this.actsSet.put("DoGuidanceHelp", DoGuidanceHelp);

		// Create Arts
		Art Place = new Art("Place");
		this.artsSet.put("Place", Place);

		// Relations in the institution
		Triple<Role, Act, Art> tGuidance = new Triple<>(Guide, DoGuidance, Place);
		Triple<Role, Act, Role> tGHelp = new Triple<>(Helper, DoGuidanceHelp, Guide);

		// OBLIGATION NORMS //
		this.addNormOBN("must", tGuidance);
		this.addNormOBN("must", tGHelp);

		// MODAL NORMS //
		List<Triple<Role, Act, ?>> lUsePlace = new ArrayList<>();
		lUsePlace.add(tGuidance);
		this.addNormMOD("use", lUsePlace);

		List<Triple<Role, Act, ?>> lUseGuide = new ArrayList<>();
		lUseGuide.add(tGHelp);
		this.addNormMOD("use", lUseGuide);

		List<Triple<Role, Act, ?>> lStarts = new ArrayList<>();
		lStarts.add(tGuidance);
		lStarts.add(tGHelp);
		this.addNormMOD("equals", lStarts);
	}

}
