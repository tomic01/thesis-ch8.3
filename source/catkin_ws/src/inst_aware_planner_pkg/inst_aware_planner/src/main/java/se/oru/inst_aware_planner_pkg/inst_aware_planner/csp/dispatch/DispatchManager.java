package se.oru.inst_aware_planner_pkg.inst_aware_planner.csp.dispatch;

import java.util.List;
import java.util.Map.Entry;

import org.metacsp.dispatching.DispatchingFunction;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.sensing.Sensor;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import monarch_msgs.KeyValuePair;
import monarch_msgs.KeyValuePairArray;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.csp.InstitutionNode;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Grounding;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.InstDomain;
import se.oru.inst_aware_planner_pkg.inst_aware_planner.framework.Institution;

// import org.ros.node.NodeConfiguration;

public class DispatchManager extends DispatchAbstract {

	// On this topic the planner will listen if behavior is finished
	private static final String BehaviorResultTopicName = new String("common_behavior_feedback");

	private static final String TopicPrefix = new String("sar/");
	private static final String ActivationTopicName = new String("behavior_selection");
	private static final String ActivationTopicName_Tut = new String("behavior_selection_tut");
	private static final String StartInstitutionTopicNameInter = new String("inst_to_start_internal");
	private static final String StartInstitutionTopicName = new String("inst_to_start");

	// Behaviors that can be dispatched (TODO: automatically from theDomain)
	private static String strMoveOnTrajectory = "moveOnTrajectory";
	private static String strDescribe = "describe";
	private static String strWayPointNavigation = "wayPointNavigation";
	private static String strMoveInFormation = "moveInFormation";

	// Booleans to control status of the behaviors
	private static Boolean boolMoveOnTrajectoryActivation = false;
	private static Boolean boolDescribeActivation = false;
	private static Boolean boolWayPointNavigationActivation = false;
	private static Boolean boolMoveInFormationActivation = false;

	private static Boolean boolNextInstitutionPublished = false;

	// Planner components
	private final ActivityNetworkSolver activitySolver;
	private final ConnectedNode connectedNode;
	private final String[] actuators;

	// Institution part
	Institution inst;
	InstDomain theDomain;
	Grounding grounding;

	// Simulation or testing
	Boolean testingMode = false;

	public DispatchManager(final ActivityNetworkSolver actSolver, final ConnectedNode connectedNode,
			final String[] actuators, Boolean testingMode) {
		this.activitySolver = actSolver;
		this.connectedNode = connectedNode;
		this.actuators = actuators;
		this.testingMode = testingMode;
	}

	public void dispatcherSetup(Institution inst, InstDomain theDomain, Grounding grounding) {

		this.inst = inst;
		this.theDomain = theDomain;
		this.grounding = grounding;

		// Get through all actuators (robots)
		for (final String actuator : this.actuators) {
			dispatcherSetup(actuator);
		}

		if (testingMode) {
			monitorResultTest();
		} else {
			monitorBehResults();
		}

	}

	// protected void monitorBehResults() {
	// Subscriber<inst_msgs.BehResult> subscriberBehaviorResult =
	// connectedNode.newSubscriber(BehaviorResultTopicName,
	// inst_msgs.BehResult._TYPE);
	// subscriberBehaviorResult.addMessageListener(new MessageListener<BehResult>()
	// {
	// @Override
	// public void onNewMessage(BehResult behResult) {
	// InstitutionNode.logger.info("Result Feedback received: \n");
	// InstitutionNode.logger.info("Agent: " + behResult.getAgentName() + "\n");
	// InstitutionNode.logger.info("Behav: " + behResult.getBehName() + "\n");
	// InstitutionNode.logger.info("Succe: " + behResult.getSuccess() + "\n");
	//
	// String uniqueName = behResult.getAgentName() + behResult.getBehName();
	//
	// InstitutionNode.logger.info("Stoping unique name: " + uniqueName);
	// DispatchUtils.stopExecutingActivity(uniqueName);
	// }
	// });
	//
	// }

	protected void monitorBehResults() {
		String fullTopicName = TopicPrefix + BehaviorResultTopicName;
		Subscriber<KeyValuePairArray> subscriberBehaviorResult = connectedNode.newSubscriber(fullTopicName,
				KeyValuePairArray._TYPE);
		subscriberBehaviorResult.addMessageListener(new MessageListener<KeyValuePairArray>() {
			@Override
			public void onNewMessage(KeyValuePairArray resultArray) {
				InstitutionNode.logger.info("Result Feedback received: \n");

				String agentName = null;
				String behName = null;
				String status = null;
				List<KeyValuePair> resultPairs = resultArray.getArray();
				for (KeyValuePair pair : resultPairs) {
					if (pair.getKey().equals("agent")) {
						agentName = pair.getValue();
					} else if (pair.getKey().equals("behavior")) {
						behName = pair.getValue();
					} else if (pair.getKey().equals("status")) {
						status = pair.getValue();
					}
				}

				if (agentName != null && behName != null && status != null) {

					InstitutionNode.logger.info("Agent: " + agentName + "\n");
					InstitutionNode.logger.info("Behav: " + behName + "\n");
					InstitutionNode.logger.info("Status: " + status + "\n");

					String uniqueName = agentName.toLowerCase() + behName.toLowerCase();

					InstitutionNode.logger.info("Stoping unique name: " + uniqueName);
					DispatchUtils.stopExecutingActivity(uniqueName);

					// *** HACK ***
					// For durative behaviors (behaviors that does not finish on their own)
					// HARDCODED (eventually this should depend on some additional information
					// If in guidng institution mbot11::moveOnTrajectory finishes, then also finish
					// activity, mbot03:moveInFormation
					// TODO: IF THE GROUNINDG CHANGES THIS IS FAILING!!!
					if (agentName.equals("mbot11") && behName.equals("moveontrajectory")) {
						String uniqueName2 = "mbot03" + "moveinformation";
						
						// TODO: Remove this sleep if you want this to finish at the same time
						/*try {
							
							Thread.sleep(1000);
						} catch (InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}*/
						DispatchUtils.stopExecutingActivity(uniqueName2);

						// ******** HACK ********* PUBLISH THE NAME OF THE NEXT INSTITUTION
						publishNextInstitution("Touring");

					}
				} else {
					throw new Error("There is a problem with behavior resutls message");
				}
			}

			private void publishNextInstitution(final String nextInstitutionName) {
				String topicFullName = TopicPrefix + StartInstitutionTopicNameInter;
				InstitutionNode.logger.info("Publishing Next Institution on topic: " + topicFullName);

				final Publisher<std_msgs.String> publisher = connectedNode.newPublisher(topicFullName,
						std_msgs.String._TYPE);

				boolNextInstitutionPublished = true;

				connectedNode.executeCancellableLoop(new CancellableLoop() {
					@SuppressWarnings("unused")
					private int sequenceNumber;

					@Override
					protected void setup() {
						sequenceNumber = 0;
					}

					@Override
					protected void loop() throws InterruptedException {

						// Publish only once
						if (boolNextInstitutionPublished) {

							Thread.sleep(1000); // wait for publisher initialization

							std_msgs.String nextInstitutionMsg = publisher.newMessage();

							nextInstitutionMsg.setData(nextInstitutionName);

							publisher.publish(nextInstitutionMsg);
							boolNextInstitutionPublished = false;

							sequenceNumber++;
							Thread.sleep(1000);
						}

					}
				});
			}
		});

	}

	// Automatically finishes behaviors after some delay
	@SuppressWarnings("unused")
	private void monitorResultTest() {
		while (true) {
			// Sleep for X seconds
			try {
				Thread.sleep(5000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			// Now get through each behavior, find one and stop it
			if (!DispatchAbstract.ID_Activity_Map.isEmpty()) {
				for (Entry<String, SymbolicVariableActivity> entry : DispatchAbstract.ID_Activity_Map.entrySet()) {
					String behaviorID = entry.getKey();
					SymbolicVariableActivity act = entry.getValue();
					DispatchUtils.stopExecutingActivity(behaviorID);
					break;
				}
			}
		}
	}

	private void dispatcherSetup(final String agentName) {
		// Dispatching function. Called when the behavior is dispatched.
		final DispatchingFunction dispatchBehavior = new DispatchingFunction(agentName) {
			@Override
			public void dispatch(SymbolicVariableActivity activity) {

				// Get actuator symbol from dispatched activity
				String agentName = activity.getComponent();
				String behaviorName = null;
				String[] params = null;
				String fullBehaviorName = activity.getSymbolicVariable().getSymbols()[0];
				Integer brackIndex = fullBehaviorName.lastIndexOf("(");
				if (brackIndex != -1) { // some value returned
					behaviorName = fullBehaviorName.substring(0, brackIndex);
					String allParams = fullBehaviorName.substring(brackIndex + 1, fullBehaviorName.length() - 1);
					params = allParams.split(",");

				} else {
					behaviorName = fullBehaviorName;
				}

				String uniqueName = agentName.toLowerCase() + behaviorName.toLowerCase();
				ID_Activity_Map.put(uniqueName, activity);

				// Print Info
				InstitutionNode.logger.info("\n**********Dispatching********** \nAgent: " + agentName + ";\nBehavior: "
						+ behaviorName + "\nParams: ");
				if (params != null) {
					for (String param : params) {
						InstitutionNode.logger.info(param + ", ");
					}
				}

				// Publish dispatching functions

				// Different topic are published for a different dispatched behaviors
				if (!testingMode) {
					if (behaviorName.equals(strMoveOnTrajectory)) {
						boolMoveOnTrajectoryActivation = true;
						// Publish MoveOnTrajectory
						pubMoveOnTrajectory(agentName, behaviorName, params, ActivationTopicName);
					} else if (behaviorName.equals(strDescribe)) {
						boolDescribeActivation = true;
						// Publish Describe
						pubDescribe(agentName, behaviorName, params, ActivationTopicName_Tut);
					} else if (behaviorName.equals(strWayPointNavigation)) {
						boolWayPointNavigationActivation = true;
						// Publish WayPointNavigation
						pubWayPointNavigation(agentName, behaviorName, params, ActivationTopicName_Tut);
					} else if (behaviorName.equals(strMoveInFormation)) {
						boolMoveInFormationActivation = true;
						// Publish MoveInFormation
						// Do not publish to humans
						if (!agentName.contains("human")) {
							pubMoveInFormation(agentName, behaviorName, params, ActivationTopicName);
						}
						
					}
				}
			}

			@Override
			public boolean skip(SymbolicVariableActivity act) {
				return false;
			}
		};

		InstitutionNode.animator.addDispatchingFunctions(activitySolver, dispatchBehavior);
	}

	// TODO: Add to a different class, or re-factory it a lot (same code used many
	// times)
	private void pubMoveOnTrajectory(String agentName, String behName, final String[] params, String topicName) {
		String topicFullName = TopicPrefix + agentName + "/" + topicName;
		InstitutionNode.logger.info("Topic name: " + topicFullName);

		final Publisher<KeyValuePairArray> publisher = connectedNode.newPublisher(topicFullName,
				KeyValuePairArray._TYPE);

		connectedNode.executeCancellableLoop(new CancellableLoop() {
			@SuppressWarnings("unused")
			private int sequenceNumber;

			@Override
			protected void setup() {
				sequenceNumber = 0;
			}

			@Override
			protected void loop() throws InterruptedException {

				// Publish only once
				if (boolMoveOnTrajectoryActivation) {

					Thread.sleep(1000); // wait for publisher initialization

					KeyValuePairArray msgKeyValueArray = publisher.newMessage();
					List<KeyValuePair> keyValueArray = msgKeyValueArray.getArray();

					// behavior:value
					KeyValuePair pairBeh = connectedNode.getTopicMessageFactory().newFromType(KeyValuePair._TYPE);
					pairBeh.setKey("behavior");
					pairBeh.setValue(strMoveOnTrajectory.toLowerCase());
					keyValueArray.add(pairBeh);

					// { (object1:value), (object2:value),.... }
					int count = 1;
					for (String param : params) {
						KeyValuePair pairObj = connectedNode.getTopicMessageFactory().newFromType(KeyValuePair._TYPE);
						pairObj.setKey("object" + Integer.toString(count));
						count++;
						pairObj.setValue(param.toLowerCase());
						keyValueArray.add(pairObj);
					}

					// ADDITIONAL
					// Role:value
					KeyValuePair pairRole = connectedNode.getTopicMessageFactory().newFromType(KeyValuePair._TYPE);
					pairRole.setKey("role");
					pairRole.setValue("leader");
					keyValueArray.add(pairRole);

					publisher.publish(msgKeyValueArray);
					boolMoveOnTrajectoryActivation = false;

					sequenceNumber++;
					Thread.sleep(1000);
				}

			}
		});
	}

	private void pubDescribe(String agentName, String behName, final String[] params, String topicName) {
		String topicFullName = TopicPrefix + agentName + "/" + topicName;
		InstitutionNode.logger.info("Topic name: " + topicFullName);

		final Publisher<KeyValuePairArray> publisher = connectedNode.newPublisher(topicFullName,
				KeyValuePairArray._TYPE);

		connectedNode.executeCancellableLoop(new CancellableLoop() {

			private int sequenceNumber;

			@Override
			protected void setup() {
				sequenceNumber = 0;
			}

			@Override
			protected void loop() throws InterruptedException {
				if (boolDescribeActivation) {
					Thread.sleep(1000);

					KeyValuePairArray msgKeyValueArray = publisher.newMessage();
					List<KeyValuePair> keyValueArray = msgKeyValueArray.getArray();

					// behavior:value
					KeyValuePair pairBeh = connectedNode.getTopicMessageFactory().newFromType(KeyValuePair._TYPE);
					pairBeh.setKey("behavior");
					pairBeh.setValue(strDescribe.toLowerCase());
					keyValueArray.add(pairBeh);

					// { (object1:value), (object2:value),.... }
					int count = 1;
					for (String param : params) {
						KeyValuePair pairObj = connectedNode.getTopicMessageFactory().newFromType(KeyValuePair._TYPE);
						pairObj.setKey("object" + Integer.toString(count));
						count++;
						pairObj.setValue(param.toLowerCase());
						keyValueArray.add(pairObj);
					}

					// ADDITIONAL
					// Role:value
					KeyValuePair pairRole = connectedNode.getTopicMessageFactory().newFromType(KeyValuePair._TYPE);
					pairRole.setKey("role");
					pairRole.setValue("tutor");
					keyValueArray.add(pairRole);

					publisher.publish(msgKeyValueArray);
					boolDescribeActivation = false;

					sequenceNumber++;
					Thread.sleep(1000);
				}
			}

		});
	}

	private void pubWayPointNavigation(String agentName, String behName, final String[] params, String topicName) {
		String topicFullName = TopicPrefix + agentName + "/" + topicName;
		InstitutionNode.logger.info("Topic name: " + topicFullName);

		final Publisher<KeyValuePairArray> publisher = connectedNode.newPublisher(topicFullName,
				KeyValuePairArray._TYPE);

		connectedNode.executeCancellableLoop(new CancellableLoop() {
			@SuppressWarnings("unused")
			private int sequenceNumber;

			@Override
			protected void setup() {
				sequenceNumber = 0;
			}

			@Override
			protected void loop() throws InterruptedException {

				// Publish only once
				if (boolWayPointNavigationActivation) {

					Thread.sleep(1000);

					KeyValuePairArray msgKeyValueArray = publisher.newMessage();
					List<KeyValuePair> keyValueArray = msgKeyValueArray.getArray();

					// behavior:value
					KeyValuePair pairBeh = connectedNode.getTopicMessageFactory().newFromType(KeyValuePair._TYPE);
					pairBeh.setKey("behavior");
					pairBeh.setValue(strWayPointNavigation.toLowerCase());
					keyValueArray.add(pairBeh);

					// { (object1:value), (object2:value),.... }
					int count = 1;
					for (String param : params) {
						KeyValuePair pairObj = connectedNode.getTopicMessageFactory().newFromType(KeyValuePair._TYPE);
						pairObj.setKey("object" + Integer.toString(count));
						count++;
						pairObj.setValue(param.toLowerCase());
						keyValueArray.add(pairObj);
					}

					// ADDITIONAL
					// Role:value
					KeyValuePair pairRole = connectedNode.getTopicMessageFactory().newFromType(KeyValuePair._TYPE);
					pairRole.setKey("role");
					pairRole.setValue("tutorhelper");
					keyValueArray.add(pairRole);

					publisher.publish(msgKeyValueArray);
					boolWayPointNavigationActivation = false;

					sequenceNumber++;
					Thread.sleep(1000);
				}

			}
		});
	}

	private void pubMoveInFormation(String agentName, String behName, final String[] params, String topicName) {
		String topicFullName = TopicPrefix + agentName + "/" + topicName;
		InstitutionNode.logger.info("Topic name: " + topicFullName);

		final Publisher<KeyValuePairArray> publisher = connectedNode.newPublisher(topicFullName,
				KeyValuePairArray._TYPE);

		connectedNode.executeCancellableLoop(new CancellableLoop() {
			@SuppressWarnings("unused")
			private int sequenceNumber;

			@Override
			protected void setup() {
				sequenceNumber = 0;
			}

			@Override
			protected void loop() throws InterruptedException {

				// Publish only once
				if (boolMoveInFormationActivation) {

					Thread.sleep(1000);

					KeyValuePairArray msgKeyValueArray = publisher.newMessage();
					List<KeyValuePair> keyValueArray = msgKeyValueArray.getArray();

					// behavior:value
					KeyValuePair pairBeh = connectedNode.getTopicMessageFactory().newFromType(KeyValuePair._TYPE);
					pairBeh.setKey("behavior");
					pairBeh.setValue(strMoveInFormation.toLowerCase());
					keyValueArray.add(pairBeh);

					// { (object1:value), (object2:value),.... }
					int count = 1;
					for (String param : params) {
						KeyValuePair pairObj = connectedNode.getTopicMessageFactory().newFromType(KeyValuePair._TYPE);
						pairObj.setKey("object" + Integer.toString(count));
						count++;
						pairObj.setValue(param.toLowerCase());
						keyValueArray.add(pairObj);
					}

					// ADDITIONAL
					// Role:value
					KeyValuePair pairRole = connectedNode.getTopicMessageFactory().newFromType(KeyValuePair._TYPE);
					pairRole.setKey("role");
					pairRole.setValue("follower");
					keyValueArray.add(pairRole);

					publisher.publish(msgKeyValueArray);
					boolMoveInFormationActivation = false;

					sequenceNumber++;
					Thread.sleep(1000);
				}

			}
		});
	}

	public void listenForInstitutionStart() {
		
		// LISTENS TO INTERNAL INSTITUTION ACTIVATION MESSAGE
		String fullTopicName = TopicPrefix + StartInstitutionTopicNameInter;
		Subscriber<std_msgs.String> subInstStartIntr = connectedNode.newSubscriber(fullTopicName,
				std_msgs.String._TYPE);
		
		subInstStartIntr.addMessageListener(new MessageListener<std_msgs.String>() {
			
			@Override
			public void onNewMessage(std_msgs.String instToActivate) {
				
				String inst = instToActivate.getData();
				InstitutionNode.logger.info("Institution to Actiate (internal) message recieved: " + inst + "\n");
				
				// Sensors (Trigger for the institution context variable)
				InstitutionNode.sensorCommand = new Sensor("command", InstitutionNode.animator);
				InstitutionNode.sensorCommand.postSensorValue(inst, InstitutionNode.animator.getTimeNow());
			}
		});
		
		// LISTENS TO EXTERNAL INSTITUTION ACTIVATION MESSAGE
		fullTopicName = TopicPrefix + StartInstitutionTopicName;
		Subscriber<std_msgs.String> subInstStart = connectedNode.newSubscriber(fullTopicName,
				std_msgs.String._TYPE);
		
		subInstStart.addMessageListener(new MessageListener<std_msgs.String>() {

			@Override
			public void onNewMessage(std_msgs.String instToActivate) {
				
				String inst = instToActivate.getData();
				InstitutionNode.logger.info("Institution to Actiate message (external) recieved: " + inst + "\n");
				
				// Sensors (Trigger for the institution context variable)
				InstitutionNode.sensorCommand = new Sensor("command", InstitutionNode.animator);
				InstitutionNode.sensorCommand.postSensorValue(inst, InstitutionNode.animator.getTimeNow());
				
			}
			
		});
		
	}
	

}
