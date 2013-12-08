package se.oru.lucia.rfidlocalization;

import geometry_msgs.Point;
import geometry_msgs.Pose;

import java.util.Arrays;

import org.ros.internal.message.RawMessage;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import org.ros.concurrent.CancellableLoop;

public class PeisRFIDMonitor extends AbstractNodeMain {

	private final PeisRFIDLocalizer tm = null;

	private ConnectedNode cn;
	private String prefix = "/rfidlocalization";

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(prefix + "/" + this.getClass().getSimpleName());
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {

		cn = connectedNode;
		String mapFile = "maps/map.txt.SAVE.4.SAVE";		
		final PeisRFIDLocalizer tm = new PeisRFIDLocalizer("PeisRFIDLocalizer",mapFile);

		final Publisher<std_msgs.String> locationPublisher = connectedNode.newPublisher(prefix + "/location", std_msgs.String._TYPE);
		final Publisher<std_msgs.String> rfidPublisher = connectedNode.newPublisher(prefix + "/RFIDtags", std_msgs.String._TYPE);
		final Publisher<geometry_msgs.Point> positionPublisher = connectedNode.newPublisher(prefix + "/position", geometry_msgs.Point._TYPE);

		connectedNode.executeCancellableLoop(new CancellableLoop() {
			@Override
			protected void loop() throws InterruptedException {
				Thread.sleep(500);
				tm.updateLocation();
				std_msgs.String tags = rfidPublisher.newMessage();
				tags.setData(tm.getRFIDsInRange());
				rfidPublisher.publish(tags);
				std_msgs.String str = locationPublisher.newMessage();
				str.setData(tm.getComputedLocation());
				locationPublisher.publish(str);
				geometry_msgs.Point point  = positionPublisher.newMessage();
				point.setX(tm.getComputedX());
				point.setY(tm.getComputedY());
				point.setZ(0);
				positionPublisher.publish(point);
			}
		});
	}

}
