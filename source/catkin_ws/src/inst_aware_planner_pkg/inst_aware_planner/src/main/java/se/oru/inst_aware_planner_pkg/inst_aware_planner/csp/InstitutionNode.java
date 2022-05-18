/*
 * Copyright (C) 2016 S.Tomic
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package se.oru.inst_aware_planner_pkg.inst_aware_planner.csp;

import java.util.Collections;
import java.util.LinkedList;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.metacsp.meta.simplePlanner.ProactivePlanningDomain;
import org.metacsp.meta.simplePlanner.SimpleDomain;
import org.metacsp.meta.simplePlanner.SimplePlanner;
import org.metacsp.meta.simplePlanner.SimplePlannerInferenceCallback;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.sensing.ConstraintNetworkAnimator;
import org.metacsp.sensing.Sensor;
import org.metacsp.time.Bounds;
import org.metacsp.utility.logging.MetaCSPLogging;
import org.metacsp.utility.timelinePlotting.TimelinePublisher;
import org.metacsp.utility.timelinePlotting.TimelineVisualizer;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;

import se.oru.inst_aware_planner_pkg.inst_aware_planner.csp.dispatch.DispatchManager;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Grounding;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Institution;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.GuidingInstitution;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.TheDomain;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.TouringInstitution;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.groundings.GuideGrounding01;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.groundings.GuideGrounding02;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.groundings.TouringGrounding01;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.institutions.groundings.TouringGrounding02;

/**
 * A simple {@link Publisher} {@link NodeMain}.
 */
public class InstitutionNode extends AbstractNodeMain {

	// Log
	public static Logger logger;

	public static ConstraintNetworkAnimator animator;
	public static DispatchManager dispatchMngr;
	public static Sensor sensorCommand;
	public static Sensor sensorRestart;

	String instName = System.getenv("INST");
	String ground_i1 = System.getenv("GROUNDING_I1");
	String ground_i2 = System.getenv("GROUNDING_I2");

	public static void displayTimelineAnimator(ActivityNetworkSolver activitySolver, SimpleDomain domain) {

		LinkedList<String> sensorsActuatorsList = new LinkedList<String>();

		// Time
		sensorsActuatorsList.add("Time");

		// Sensors
		Collections.addAll(sensorsActuatorsList, domain.getSensors());

		// Context Variables
		Collections.addAll(sensorsActuatorsList, domain.getContextVars());

		// Actuators
		Collections.addAll(sensorsActuatorsList, domain.getActuators());

		String[] sensorsActuatorsArray = sensorsActuatorsList.toArray(new String[sensorsActuatorsList.size()]);
		TimelinePublisher tp = new TimelinePublisher(activitySolver, new Bounds(0, 60000), true, sensorsActuatorsArray);
		TimelineVisualizer tv = new TimelineVisualizer(tp);
		tv.startAutomaticUpdate(1000);

	}

	// Private eRobotActions robotActions;
	private String prefix = "/planning";

	@Override
	public GraphName getDefaultNodeName() {
		System.out.println("Institution name extension: " + instName);
		return GraphName.of(prefix + "/" + this.getClass().getSimpleName() + instName);
	}

	public void inputSensorTraces(String sapPackagePath) {
		// String relativePath = sapPackagePath + "/sensorTraces/experiments/";
		sensorCommand = new Sensor("command", InstitutionNode.animator);
		// sensorCommand.registerSensorTrace(relativePath + "command.st");
		sensorCommand.postSensorValue("None", animator.getTimeNow());
	}

	@SuppressWarnings("rawtypes")
	@Override
	public void onStart(final ConnectedNode connectedNode) {

		// Use ROS parameters to get the name of the planning domain
		ParameterTree params = connectedNode.getParameterTree();
		String sapPackagePath = params.getString("/inst_aware_planner_" + instName + "/iwp_pkg_path");

		Boolean testingMode = false;

		// Initialize the planner
		long origin = 0l;
		final SimplePlanner planner = new SimplePlanner(origin, origin + 31000000000l, 0);
		// 32054400 seconds in a year with 53 weeks
		// 40000000 seconds for the planning horizon

		// Logging
		logger = MetaCSPLogging.getLogger(connectedNode.getClass());
		MetaCSPLogging.setLevel(planner.getClass(), Level.OFF);
		MetaCSPLogging.setLevel(connectedNode.getClass(), Level.FINE);
		logger.info("Logger is set");

		// INSTITUTIONS //

		// Create the institution
		Institution instTouring = new TouringInstitution();
		Institution instGuide = new GuidingInstitution();
		// Create the domain
		TheDomain theDomain = new TheDomain();
		// Create the grounding
		GuideGrounding01 groundGuide01 = new GuideGrounding01(instGuide, theDomain);
		GuideGrounding02 groundGuide02 = new GuideGrounding02(instGuide, theDomain);
		TouringGrounding01 groundTour01 = new TouringGrounding01(instTouring, theDomain);
		TouringGrounding02 groundTour02 = new TouringGrounding02(instTouring, theDomain);
		
		// Decide grounding to be used for each of institutions
		Grounding groundingToUseI1 = null;
		Grounding groundingToUseI2 = null;
		if (ground_i1.equals("default")) {
			groundingToUseI1 = groundGuide01;
		}
		else if(ground_i1.equals("human_follower")) {
			groundingToUseI1 = groundGuide02;
		}
		else {
			throw new Error("Unknow grounding in use for Guide institution");
		}
		
		if (ground_i2.equals("default")) {
			groundingToUseI2 = groundTour01;
		} else if (ground_i2.equals("girafe")) {
			groundingToUseI2 = groundTour02;
		} else {
			throw new Error("Unknow grounding in use for Tutoring institution");
		}

		// Create domain files
		InstitutionAwareDomain iwDomGuide = new InstitutionAwareDomain(planner, instGuide, theDomain, groundingToUseI1);
		InstitutionAwareDomain iwDomTour = new InstitutionAwareDomain(planner, instTouring, theDomain, groundingToUseI2);
		
		iwDomTour.createFileDomain(sapPackagePath);
		iwDomGuide.createFileDomain(sapPackagePath);

		// PLANNER //

		// Set planner domain
		String domainFilePath = sapPackagePath + "/domains/" + instName + ".ddl";
		SimpleDomain plannerDom = SimpleDomain.parseDomain(planner, domainFilePath, ProactivePlanningDomain.class);

		// Planner's Callback
		SimplePlannerInferenceCallback cb = new SimplePlannerInferenceCallback(planner);
		final ActivityNetworkSolver activitySolver = (ActivityNetworkSolver) planner.getConstraintSolvers()[0];

		// Active component that animates and updates the constraint network
		animator = new ConstraintNetworkAnimator(activitySolver, 1000, cb);

		// ConstraintNetwork.draw(activitySolver.getConstraintNetwork());

		// DISPATCHING
		InstitutionNode.displayTimelineAnimator(activitySolver, plannerDom);
		dispatchMngr = new DispatchManager(activitySolver, connectedNode, plannerDom.getActuators(), testingMode);

		if (instName.equals("Guiding")) {
			dispatchMngr.dispatcherSetup(instGuide, theDomain, groundGuide01);
		} else if (instName.equals("Touring")) {
			dispatchMngr.dispatcherSetup(instTouring, theDomain, groundTour01);
		}

		// Listen for commands for activating the institutions
		dispatchMngr.listenForInstitutionStart();

	}

}
