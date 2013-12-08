package se.oru.lucia.rfidlocalization;

import geometry_msgs.Pose;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

public class RFIDLocalizer extends AbstractNodeMain {

	private ConnectedNode cn;
	private String prefix = "/rfidlocalization";
    
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(prefix + "/" + this.getClass().getSimpleName());
	}
	
	public static void main(String[] args) {
    	System.out.println("HELLO, THIS IS A TEST!");    	
    }
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		
		cn = connectedNode;
		
    	Subscriber<Pose> subscriberTest = cn.newSubscriber(prefix+"/testingSub", Pose._TYPE);
    	subscriberTest.addMessageListener(new MessageListener<Pose>() {
			@Override
			public void onNewMessage(Pose arg0) {
				System.out.println("POSE  " + arg0);
			}
		});

	}


}