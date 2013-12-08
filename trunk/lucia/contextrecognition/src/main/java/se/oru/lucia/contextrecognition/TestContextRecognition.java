package se.oru.lucia.contextrecognition;

import java.util.Vector;
import java.util.logging.Level;

import go_turtle.actions;

import org.metacsp.dispatching.DispatchingFunction;
import org.metacsp.meta.simplePlanner.ProactivePlanningDomain;
import org.metacsp.meta.simplePlanner.SimplePlanner;
import org.metacsp.multi.activity.Activity;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.sensing.ConstraintNetworkAnimator;
import org.metacsp.sensing.Sensor;
import org.metacsp.time.Bounds;
import org.metacsp.utility.logging.MetaCSPLogging;
import org.metacsp.utility.timelinePlotting.TimelinePublisher;
import org.metacsp.utility.timelinePlotting.TimelineVisualizer;
import org.ros.exception.ParameterNotFoundException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class TestContextRecognition extends AbstractNodeMain {

	private ConnectedNode cn;
	private String prefix = "/contextrecognition";
    
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(prefix + "/" + this.getClass().getSimpleName());
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		cn = connectedNode;		
		String domainFile = null;
		ParameterTree params = cn.getParameterTree();
		try { domainFile = params.getString("/lucia/domain"); }
		catch(ParameterNotFoundException e) {
			System.out.println("Please specify a domain file.");
			System.exit(0);
		}

		SimplePlanner planner = new SimplePlanner((long)(cn.getCurrentTime().toSeconds()*1000),(long)(cn.getCurrentTime().toSeconds()*1000)+(long)1000000,0);
		//MetaCSPLogging.setLevel(planner.getClass(), Level.FINE);
		
		//Planning
		ProactivePlanningDomain.parseDomain(planner, domainFile, ProactivePlanningDomain.class);
		ConstraintNetworkAnimator animator = new ConstraintNetworkAnimator(planner, 1000);
				
		//Sensing
		final Sensor sensorLocation = new Sensor("Location", animator);
		final Sensor sensorStove = new Sensor("Stove", animator);
		final Sensor sensorDoor = new Sensor("Door", animator);
		final Sensor sensorFridge = new Sensor("Fridge", animator);
		final Sensor sensorChair = new Sensor("Chair", animator);

		//Execution: dispatches all activities on state variable Robot to topic
    	final Publisher<go_turtle.actions> planPublisher = cn.newPublisher("/move_action", go_turtle.actions._TYPE);
		final Vector<Activity> currentActs = new Vector<Activity>();
		final DispatchingFunction df = new DispatchingFunction("Robot") {
			@Override
			public void dispatch(Activity act) {
				go_turtle.actions msg = planPublisher.newMessage();
				if (act.getSymbolicVariable().getSymbols()[0].equals("MoveTo()")) {
					msg.setAction(0);
					msg.setX(0);
					msg.setY(0);
					msg.setTimeout(100);
				}
				else if (act.getSymbolicVariable().getSymbols()[0].equals("SayWarning()") || act.getSymbolicVariable().getSymbols()[0].equals("SayBonAppetit()")) {
					msg.setAction(3);
					msg.setTimeout(100);
				}
				planPublisher.publish(msg);
				System.out.println(">>>>>>>>>>>>> DISPATCHED " + act);
				currentActs.add(act);
			}
		};
		animator.addDispatchingFunctions(df);

		//Sensor topic subscriptions: posts sensor values to sensors
    	Subscriber<std_msgs.String> subscriberLocation = cn.newSubscriber("/rfidlocalization/location", std_msgs.String._TYPE);
    	subscriberLocation.addMessageListener(new MessageListener<std_msgs.String>() {
    		String previousValue = "";
			@Override
			public void onNewMessage(std_msgs.String arg0) {
				if (!arg0.getData().equals(previousValue)) {
					previousValue = arg0.getData();
					sensorLocation.postSensorValue(arg0.getData()+"()", (long)(cn.getCurrentTime().toSeconds()*1000));
				}
			}
		},100);
    	
    	Subscriber<std_msgs.Bool> subscriberDiningChair = cn.newSubscriber("/PeisSensor/diningChair", std_msgs.Bool._TYPE);
    	subscriberDiningChair.addMessageListener(new MessageListener<std_msgs.Bool>() {
    		Boolean previousValue = null;
			@Override
			public void onNewMessage(std_msgs.Bool arg0) {
				if (previousValue == null || arg0.getData() != previousValue.booleanValue()) {
					previousValue = arg0.getData();
					if (arg0.getData()) sensorChair.postSensorValue("Occupied()", (long)(cn.getCurrentTime().toSeconds()*1000));
					else sensorChair.postSensorValue("Empty()", (long)(cn.getCurrentTime().toSeconds()*1000));
				}
			}
		},100);

    	Subscriber<std_msgs.Bool> subscriberPh2EntranceDoor = cn.newSubscriber("/PeisSensor/ph2EntranceDoor", std_msgs.Bool._TYPE);
    	subscriberPh2EntranceDoor.addMessageListener(new MessageListener<std_msgs.Bool>() {
    		Boolean previousValue = null;
			@Override
			public void onNewMessage(std_msgs.Bool arg0) {
				if (previousValue == null || arg0.getData() != previousValue.booleanValue()) {
					previousValue = arg0.getData();
					if (arg0.getData()) sensorDoor.postSensorValue("Closed()", (long)(cn.getCurrentTime().toSeconds()*1000));
					else sensorDoor.postSensorValue("Open()", (long)(cn.getCurrentTime().toSeconds()*1000));
				}
			}
		},100);

    	Subscriber<std_msgs.Bool> subscriberFridgeDoor = cn.newSubscriber("/PeisSensor/fridgeDoor", std_msgs.Bool._TYPE);
    	subscriberFridgeDoor.addMessageListener(new MessageListener<std_msgs.Bool>() {
    		Boolean previousValue = null;
			@Override
			public void onNewMessage(std_msgs.Bool arg0) {
				if (previousValue == null || arg0.getData() != previousValue.booleanValue()) {
					previousValue = arg0.getData();
					if (arg0.getData()) sensorFridge.postSensorValue("Closed()", (long)(cn.getCurrentTime().toSeconds()*1000));
					else sensorFridge.postSensorValue("Open()", (long)(cn.getCurrentTime().toSeconds()*1000));
				}
			}
		},100);

    	Subscriber<std_msgs.Bool> subscriberStove = cn.newSubscriber("/PeisSensor/stove", std_msgs.Bool._TYPE);
    	subscriberStove.addMessageListener(new MessageListener<std_msgs.Bool>() {
    		Boolean previousValue = null;
			@Override
			public void onNewMessage(std_msgs.Bool arg0) {
				if (previousValue == null || arg0.getData() != previousValue.booleanValue()) {
					previousValue = arg0.getData();
					if (arg0.getData()) sensorStove.postSensorValue("On()", (long)(cn.getCurrentTime().toSeconds()*1000));
					else sensorStove.postSensorValue("Off()", (long)(cn.getCurrentTime().toSeconds()*1000));
				}
			}
		},100);
    	
    	
    	//Robot feedback: signals that activities should finish
    	Subscriber<std_msgs.String> subscriberActionFeedback = cn.newSubscriber("/action_feedback", std_msgs.String._TYPE);
    	subscriberActionFeedback.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String arg0) {
				if (currentActs.size() != 0 && arg0.getData().equals("SUCCEEDED")) {
					df.finish(currentActs.get(0));
					currentActs.remove(0);
				}
			}
		},100);

    	//Timeline visualization
		TimelinePublisher tp = new TimelinePublisher((ActivityNetworkSolver)planner.getConstraintSolvers()[0], new Bounds(0,60000), true, "Time", "Location", "Stove", "Chair", "Fridge", "Door", "Human", "Alarm", "Robot");
		TimelineVisualizer tv = new TimelineVisualizer(tp);
		tv.startAutomaticUpdate(1000);

	}


}