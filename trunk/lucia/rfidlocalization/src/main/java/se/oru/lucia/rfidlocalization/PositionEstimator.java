package se.oru.lucia.rfidlocalization;

import java.util.Vector;

public class PositionEstimator {
	
	private Robot theRobot = null;
	private PeisRFIDLocalizer tm = null;
	private LocalizationMapCanvas canvas = null;
	private float previousX = 0.0f;
	private float previousY = 0.0f;
	private float previousBearing = 0.0f;
		
	public PositionEstimator(Robot r, PeisRFIDLocalizer l, LocalizationMapCanvas mc) {
		theRobot = r;
		tm = l;
		canvas = mc;
	}
	
	public void computeLocation() {
		boolean stop = false;
		int count = 0;
		float X = 0.0f;
		float Y = 0.0f;
		String tags = tm.getRFIDsInRange();
		if (!(tags == null || tags.contains("()") || tags.indexOf("(") < 0)) {
			while (!stop) {
				if (tags.indexOf("(") < 0) stop = true;
				else {
					tags = tags.substring(tags.indexOf("(")+1);
					String oneTag = tags.substring(0, tags.indexOf(")"));
					Tag theTag = tm.getFromID(oneTag.trim());
					count++;
					X += theTag.getX();
					Y += theTag.getY();
				}
			}
			float avgX = X/count;
			float avgY = Y/count;

			double a = Math.sqrt(Math.pow(Math.abs(avgX-previousX),2) + Math.pow(Math.abs(avgY-previousY),2));
			double b = Math.abs(avgY-previousY);
			float newBearing = -1.0f;
			if (avgX-previousX >= 0 && avgY-previousY >= 0) {
				newBearing = (float)Math.toDegrees(Math.asin(b/a));
			}
			else if (avgX-previousX <= 0 && avgY-previousY >= 0) {
				newBearing = (float)Math.toDegrees(Math.PI-Math.asin(b/a));
			}
			else if (avgX-previousX <= 0 && avgY-previousY <= 0) {
				newBearing = (float)Math.toDegrees(Math.PI+Math.asin(b/a));
			}
			else if (avgX-previousX >= 0 && avgY-previousY <= 0) {
				newBearing = (float)Math.toDegrees(2*Math.PI-Math.asin(b/a));
			}
			previousX = avgX;
			previousY = avgY;
			
//			System.out.println("Estimated bearing: (deg) = " + newBearing);
//			System.out.println("Estimated position: (x,y) = (" + avgX + "," + avgY + ")");

			previousX = avgX;
			previousY = avgY;
			if (!new Float(newBearing).equals(Float.NaN)) {
				previousBearing = newBearing;
			}
			
			theRobot.setBearing(previousBearing);				
			theRobot.setX((float)previousX-theRobot.getRadius());
			theRobot.setY((float)previousY-theRobot.getRadius());
			
			if (canvas != null) canvas.repaint();
		}

	}
	
//	public void run() {
//		while (true) {
//			try {
//				this.sleep(100);
//			} catch (InterruptedException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
//			boolean stop = false;
//			int count = 0;
//			float X = 0.0f;
//			float Y = 0.0f;
//			String tags = tm.getRFIDsInRange();
//			if (!(tags == null || tags.contains("()") || tags.indexOf("(") < 0)) {
//				while (!stop) {
//					if (tags.indexOf("(") < 0) stop = true;
//					else {
//						tags = tags.substring(tags.indexOf("(")+1);
//						String oneTag = tags.substring(0, tags.indexOf(")"));
//						Tag theTag = tm.getFromID(oneTag.trim());
//						count++;
//						X += theTag.getX();
//						Y += theTag.getY();
//					}
//				}
//				float avgX = X/count;
//				float avgY = Y/count;
//
//				double a = Math.sqrt(Math.pow(Math.abs(avgX-previousX),2) + Math.pow(Math.abs(avgY-previousY),2));
//				double b = Math.abs(avgY-previousY);
//				float newBearing = -1.0f;
//				if (avgX-previousX >= 0 && avgY-previousY >= 0) {
//					newBearing = (float)Math.toDegrees(Math.asin(b/a));
//				}
//				else if (avgX-previousX <= 0 && avgY-previousY >= 0) {
//					newBearing = (float)Math.toDegrees(Math.PI-Math.asin(b/a));
//				}
//				else if (avgX-previousX <= 0 && avgY-previousY <= 0) {
//					newBearing = (float)Math.toDegrees(Math.PI+Math.asin(b/a));
//				}
//				else if (avgX-previousX >= 0 && avgY-previousY <= 0) {
//					newBearing = (float)Math.toDegrees(2*Math.PI-Math.asin(b/a));
//				}
//				previousX = avgX;
//				previousY = avgY;
//				
////				System.out.println("Estimated bearing: (deg) = " + newBearing);
////				System.out.println("Estimated position: (x,y) = (" + avgX + "," + avgY + ")");
//
//				previousX = avgX;
//				previousY = avgY;
//				if (!new Float(newBearing).equals(Float.NaN)) {
//					previousBearing = newBearing;
//				}
//				
//				theRobot.setBearing(previousBearing);				
//				theRobot.setX((float)previousX-theRobot.getRadius());
//				theRobot.setY((float)previousY-theRobot.getRadius());
//				
//				if (canvas != null) canvas.repaint();
//								
//			}
//		}
//	}

}
