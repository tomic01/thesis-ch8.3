package se.oru.inst_aware_planner_pkg.inst_aware_planner.csp;

import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Vector;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.VariableOrderingH;
import org.metacsp.framework.meta.MetaConstraintSolver;
import org.metacsp.meta.simplePlanner.ProactivePlanningDomain;
import org.metacsp.meta.simplePlanner.SimpleDomain;
import org.metacsp.meta.simplePlanner.SimpleOperator;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;

import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Grounding;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.InstDomain;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.InstDomain.Agent;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.InstDomain.Behavior;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.InstDomain.Obj;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Institution;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Institution.Act;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Institution.Art;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Institution.Role;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.utils.MapUtil;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.utils.Pair;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.utils.Triple;

public class InstitutionAwareDomain {

	// Planner's domain
	SimpleDomain dom;
	@SuppressWarnings("rawtypes")
	private Institution inst;
	@SuppressWarnings("rawtypes")
	private InstDomain instDomain;
	private Grounding grounding;

	// Maps the domain triples (from norms) to their indices
	@SuppressWarnings("rawtypes")
	private Map<Triple<Agent, Behavior, ?>, Integer> indicesTriplesMap = new HashMap<>();
	private int req_num = 1; // 0 is reserved for the HEAD (in the planner's domain language)

	// Keep information about the constraints
	Vector<AdditionalConstraint> constrVector = new Vector<AdditionalConstraint>();

	static String strInstitution = "Institution";

	private List<String> sensors;
	private List<String> contexts;
	private List<String> actuators;

	MetaConstraintSolver solver;

	public InstitutionAwareDomain(MetaConstraintSolver solver, Institution inst, InstDomain domain,
			Grounding grounding) {
		super();

		this.solver = solver;

		// Create the institution
		this.inst = inst;
		// Create the domain
		this.instDomain = domain;
		// Create the grounding
		this.grounding = grounding;

		// CHECK ADMISSIBLITY
		checkAdmissible();

		// CREATE VARIABLES (Sensros, Context, Actuators)
		this.sensors = createSensorsVar();
		this.contexts = createContextVar();
		this.actuators = createActuators();

		// CREATE VARIABLES AND CONSTRAINTS FROM I,D,G
		createDomainTriplesFromNorms();
		createConstraintInfoFromNorms();

	}

	public SimpleDomain getDomain() {
		return dom;
	}

	// Creates a domain file
	public void createFileDomain(String packPath) {
		try {
			domainFileCreator(strInstitution, packPath);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	// Creates internal domain
	public SimpleDomain createInternalDomain() {

		// Create empty domain
		createEmptyDomain();

		// Create Variables
		createVariablesInDomain(sensors, contexts, actuators);

		SimpleOperator instOp = CreateDomOperator();

		dom.addOperator(instOp);
		solver.addMetaConstraint(dom);
		return dom;

		// NOTE: add other operators here
	}

	private SimpleOperator CreateDomOperator() {

		// Create requirement list
		String head = strInstitution + "::" + inst.getInstName();
		String[] requirementStrings = createRequirementStringArray(head);

		AllenIntervalConstraint[] consFromHeadtoReq = new AllenIntervalConstraint[requirementStrings.length];

		SimpleOperator instOp = new SimpleOperator(head, consFromHeadtoReq, requirementStrings, new int[0]);

		for (AdditionalConstraint ac : constrVector) {
			ac.addAdditionalConstraint(instOp);
		}

		return instOp;
	}

	private List<String> createSensorsVar() {
		return Arrays.asList("command");
	}

	private List<String> createContextVar() {
		return Arrays.asList(strInstitution);
	}

	@SuppressWarnings("rawtypes")
	private List<String> createActuators() {
		// Actuators
		List<String> addedAgents = new ArrayList<String>();
		List<Pair<Role, Agent>> relationsGa = grounding.Ga.getRelation();

		// Go through grounding and add agents (as actuators in planning domain)
		for (Pair<Role, Agent> gaPair : relationsGa) {
			Agent ag = gaPair.getSecond();
			if (!addedAgents.contains(ag)) {
				addedAgents.add((String) ag.getAg());
			}
		}

		return addedAgents;
	}

	@SuppressWarnings("rawtypes")
	private void createVariablesInDomain(List<String> sensors, List<String> contextVars, List<String> actuators) {

		// Sensors
		for (String sensor : sensors) {
			dom.addSensor(sensor);
		}

		// Context for the institution
		for (String contextVar : contextVars) {
			dom.addContextVar(contextVar);
		}

		// Actuators
		for (String actuator : actuators) {
			dom.addActuator(actuator);
		}
	}

	@SuppressWarnings("rawtypes")
	private String[] createRequirementStringArray(String head) {

		List<String> requirments = new ArrayList<>();
		addRequirementStrings(requirments);
		// Additional: Add sensor
		requirments.add("command::" + inst.getInstName());

		return requirments.toArray(new String[0]);

	}

	@SuppressWarnings("rawtypes")
	private void addRequirementStrings(List<String> requirments) throws Error {

		// Sort requirements so that values are 1,2,3,4...
		indicesTriplesMap = MapUtil.sortByValue(indicesTriplesMap);

		List<Triple<Agent, Behavior, ?>> listTriples = new ArrayList<>(indicesTriplesMap.keySet());
		for (Triple<Agent, Behavior, ?> triple : listTriples) {
			String agStr = (String) triple.getFirst().getAg();
			String behStr = (String) triple.getSecond().getBeh();
			String objRolStr;

			if (triple.getThird() instanceof Obj) {
				Obj obj = (Obj) triple.getThird();
				objRolStr = (String) obj.getObj();
			} else if (triple.getThird() instanceof Agent) {
				Agent ag = (Agent) triple.getThird();
				objRolStr = (String) ag.getAg();
			} else
				throw new Error("Type error. Thirt element in a truple must be eather Object or Role");

			String req = agStr + "::" + behStr + "(" + objRolStr + ")";
			requirments.add(req);
		}

		return;
	}

	private void checkAdmissible() throws Error {
		// Check if the grounding is admissible
		if (grounding.isAdmissibleGrounding()) {
			System.out.println("\nThe grounding is Admissible!!!");
		} else {
			System.out.println("\nThe grounding is NOT Admissible!!! Stopping...");
			throw new Error("The institution is not admissible");
		}
	}

	@SuppressWarnings("rawtypes")
	private void createDomainTriplesFromNorms() {

		// Go through the institution obligation norms and crate domain triples
		// e.g.(ag1,beh2,obj1)
		List<Triple<Agent, Behavior, ?>> listStatements = new ArrayList<>();
		@SuppressWarnings("unchecked")
		List<Pair<String, Triple<Role, Act, ?>>> normsOBN = inst.getNormsOBN();
		for (Pair<String, Triple<Role, Act, ?>> norm : normsOBN) {
			Triple<Role, Act, ?> stm = norm.getSecond();
			addDomainTriplesFromStm(listStatements, stm);
		}

		for (Triple<Agent, Behavior, ?> domTriple : listStatements) {
			indicesTriplesMap.put(domTriple, req_num);
			req_num++;
		}

		checkUsabilityNorms();

		return;
	}

	@SuppressWarnings("rawtypes")
	private void checkUsabilityNorms() {

		// All 'use' norms have to be already contained in the triples
		@SuppressWarnings("unchecked")
		List<Pair<String, List<Triple<Role, Act, ?>>>> normsMOD = inst.getNormsMOD();
		for (Pair<String, List<Triple<Role, Act, ?>>> norm : normsMOD) {
			String normModality = norm.getFirst();

			if (normModality == "use") {

				List<Triple<Role, Act, ?>> normTriples = norm.getSecond();

				// "use" norm supports only one triple
				if (normTriples.size() != 1) {
					throw new Error("Number of statements is not supported for the 'use' norm");
				}

				// Get the statement from this norm
				Triple<Role, Act, ?> stm = normTriples.get(0);
				List<Triple<Agent, Behavior, ?>> domainTriples = new ArrayList<>();
				addDomainTriplesFromStm(domainTriples, stm);

				// Check if domain statements are already contained in the indicesMap
				for (Triple<Agent, Behavior, ?> domTriple : domainTriples) {
					if (!indicesTriplesMap.containsKey(domTriple)) {
						throw new Error("'Use' norm for statement (" + stm.toString()
								+ " does not exists in the planning domain");
					}
				}
			}
		}
	}

	@SuppressWarnings("rawtypes")
	private void createConstraintInfoFromNorms() {

		// Go through MOD norms (ones that can be implemented as constraints in the
		// planning domain)
		@SuppressWarnings("unchecked")
		List<Pair<String, List<Triple<Role, Act, ?>>>> normsMOD = inst.getNormsMOD();
		for (Pair<String, List<Triple<Role, Act, ?>>> norm : normsMOD) {

			String normModality = norm.getFirst();
			if (normModality == "starts") {

				List<Triple<Role, Act, ?>> normTriples = norm.getSecond();

				if (normTriples.size() != 2) {
					throw new Error("The norm 'starts' has to have two statements");
				}

				Triple<Role, Act, ?> stm1 = normTriples.get(0);
				Triple<Role, Act, ?> stm2 = normTriples.get(1);

				List<Triple<Agent, Behavior, ?>> listDomTriples1 = new ArrayList<>();
				List<Triple<Agent, Behavior, ?>> listDomTriples2 = new ArrayList<>();

				addDomainTriplesFromStm(listDomTriples1, stm1);
				addDomainTriplesFromStm(listDomTriples2, stm2);

				for (Triple<Agent, Behavior, ?> domTriple1 : listDomTriples1) {
					if (indicesTriplesMap.containsKey(domTriple1)) {
						int tripleIndexFirst = indicesTriplesMap.get(domTriple1);
						// We have a first index, let's find the next one...
						for (Triple<Agent, Behavior, ?> domTriple2 : listDomTriples2) {
							if (indicesTriplesMap.containsKey(domTriple2)) {
								int tripleIndexSecond = indicesTriplesMap.get(domTriple2);
								// Now create a constraint:
								AllenIntervalConstraint cContains = new AllenIntervalConstraint(
										AllenIntervalConstraint.Type.Starts);
								AdditionalConstraint con = new AdditionalConstraint(cContains, tripleIndexFirst,
										tripleIndexSecond);
								constrVector.addElement(con);

							} else
								throw new Error("You are trying to refer to a domain triple that does not exists...");
						}
					} else
						throw new Error("You are trying to refer to a domain triple that does not exists...");
				}

			}
			if (normModality == "equals") {

				List<Triple<Role, Act, ?>> normTriples = norm.getSecond();

				if (normTriples.size() != 2) {
					throw new Error("The norm 'equals' has to have two statements");
				}

				Triple<Role, Act, ?> stm1 = normTriples.get(0);
				Triple<Role, Act, ?> stm2 = normTriples.get(1);

				List<Triple<Agent, Behavior, ?>> listDomTriples1 = new ArrayList<>();
				List<Triple<Agent, Behavior, ?>> listDomTriples2 = new ArrayList<>();

				addDomainTriplesFromStm(listDomTriples1, stm1);
				addDomainTriplesFromStm(listDomTriples2, stm2);

				for (Triple<Agent, Behavior, ?> domTriple1 : listDomTriples1) {
					if (indicesTriplesMap.containsKey(domTriple1)) {
						int tripleIndexFirst = indicesTriplesMap.get(domTriple1);
						// We have a first index, let's find the next one...
						for (Triple<Agent, Behavior, ?> domTriple2 : listDomTriples2) {
							if (indicesTriplesMap.containsKey(domTriple2)) {
								int tripleIndexSecond = indicesTriplesMap.get(domTriple2);
								// Now create a constraint:
								AllenIntervalConstraint cContains = new AllenIntervalConstraint(
										AllenIntervalConstraint.Type.Equals);
								AdditionalConstraint con = new AdditionalConstraint(cContains, tripleIndexFirst,
										tripleIndexSecond);
								constrVector.addElement(con);

							} else
								throw new Error("You are trying to refer to a domain triple that does not exists...");
						}
					} else
						throw new Error("You are trying to refer to a domain triple that does not exists...");
				}

			}
		}
	}

	@SuppressWarnings("rawtypes")
	private void addDomainTriplesFromStm(List<Triple<Agent, Behavior, ?>> listStatements,
			Triple<Role, Act, ?> statement) {

		Role role = statement.getFirst();
		Act act = statement.getSecond();

		List<Agent> agents = grounding.getAgentsFromGa(role);
		List<Behavior> behaviors = grounding.getBehFromGb(act);

		// Create requirements
		for (Agent ag : agents) {
			for (Behavior beh : behaviors) {

				if (statement.getThird() instanceof Art) {
					Art art = (Art) statement.getThird();
					List<Obj> objects = grounding.getObjFromGo(art);
					for (Obj obj : objects) {
						Triple<Agent, Behavior, Obj> triple = new Triple<>(ag, beh, obj);
						if (checkAffordance(triple)) {
							listStatements.add(triple);
						}
					}
				} else if (statement.getThird() instanceof Role) {
					Role role2 = (Role) statement.getThird();
					List<Agent> agents2 = grounding.getAgentsFromGa(role2);
					for (Agent ag2 : agents2) {
						Triple<Agent, Behavior, Agent> triple = new Triple<>(ag, beh, ag2);
						if (checkAffordance(triple)) {
							listStatements.add(triple);
						}
					}
				} else {
					throw new Error("Third parameter in the triple must be either art or role");
				}
			}
		}

		return;
	}

	@SuppressWarnings("rawtypes")
	private boolean checkAffordance(Triple<Agent, Behavior, ?> triple) {
		return instDomain.getAffordances().contains(triple);
	}

	private void createEmptyDomain() {
		int[] resourceCaps = new int[0];
		String[] resourceNames = new String[0];
		String domainName = strInstitution;
		this.dom = new ProactivePlanningDomain(resourceCaps, resourceNames, domainName, null);

		ValueOrderingH valOH = new ValueOrderingH() {
			@Override
			public int compare(ConstraintNetwork arg0, ConstraintNetwork arg1) {
				// Return unifications first
				if (arg0.getAnnotation() != null && arg1.getAnnotation() != null) {
					if (arg0.getAnnotation() instanceof Integer && arg1.getAnnotation() instanceof Integer) {
						int annotation1 = ((Integer) arg0.getAnnotation()).intValue();
						int annotation2 = ((Integer) arg1.getAnnotation()).intValue();
						return annotation2 - annotation1;
					}
				}

				// Return unifications first
				return arg0.getVariables().length - arg1.getVariables().length;
			}
		};

		// No variable ordering
		VariableOrderingH varOH = new VariableOrderingH() {
			@Override
			public int compare(ConstraintNetwork o1, ConstraintNetwork o2) {
				return 0;
			}

			@Override
			public void collectData(ConstraintNetwork[] allMetaVariables) {
			}
		};

		dom.setValOH(valOH);
		dom.setVarOH(varOH);

	}

	class AdditionalConstraint {
		AllenIntervalConstraint con;
		int from, to;

		public AdditionalConstraint(AllenIntervalConstraint con, int from, int to) {
			this.con = con;
			this.from = from;
			this.to = to;
		}

		public void addAdditionalConstraint(SimpleOperator op) {
			op.addConstraint(con, from, to);
		}
	}

	// A method for creating planning domain (.ddl) file from the institution,
	// institution domain and grounding
	private void domainFileCreator(String domainName, String packPath) throws IOException {

		List<String> lines = new ArrayList<>();

		String autoGenerationMsg = "### Auto-generated planning domain from the Institution, Institution Domain and Grounding ###\n";
		lines.add(autoGenerationMsg);

		// Create lines
		String planningDomainName = "(Domain " + domainName + ")";
		lines.add(planningDomainName);

		// Add sensors
		for (String sensor : sensors) {
			String ddlSensor = "(Sensor " + sensor + ")";
			lines.add(ddlSensor);
		}

		// Add context (only one for this implementation)
		for (String context : contexts) {
			String ddlContext = "(ContextVariable " + context + ")";
			lines.add(ddlContext);
		}

		// Add actuators
		for (String actuator : actuators) {
			String ddlActuator = "(Actuator " + actuator + ")";
			lines.add(ddlActuator);
		}

		// Start of simple operator
		lines.add("(SimpleOperator");

		String ddlHead = "(Head " + strInstitution + "::" + inst.getInstName() + ")";
		lines.add(ddlHead);

		// Requirements
		List<String> requirments = new ArrayList<>();
		addRequirementStrings(requirments);

		// Add command requirement also
		requirments.add("command::" + inst.getInstName());

		String strReq = "req";
		Integer count = 1;
		for (String req : requirments) {
			String ddlReq = "(RequiredState " + strReq + count.toString() + " " + req + ")";
			count++;
			lines.add(ddlReq);
		}

		// Constraints
		for (AdditionalConstraint ac : constrVector) {
			String ddlConstr = "(Constraint " + ac.con.getTypes()[0] + "(" + strReq + Integer.toString(ac.from) + ","
					+ strReq + Integer.toString(ac.to) + "))";
			lines.add(ddlConstr);
		}

		// End of simple operator
		lines.add(")");

		String fileName = inst.getInstName() + ".ddl";
		System.out.println("Package path: " + packPath);
		Path file = Paths.get(packPath + "/domains/" + fileName);
		Files.write(file, lines, Charset.forName("UTF-8"));
		// Append option
		// Files.write(file, lines, Charset.forName("UTF-8"),
		// StandardOpenOption.APPEND);

	}

}
