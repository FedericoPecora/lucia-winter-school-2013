package se.oru.lucia.rfidlocalization;

import java.awt.BasicStroke;
import java.awt.Canvas;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.Collections;
import java.util.HashMap;
import java.util.Random;
import java.util.Vector;

import javax.imageio.ImageIO;
import javax.swing.JLabel;
import javax.swing.JPanel;

public class SimulationMapCanvas extends JPanel implements MouseListener, MouseMotionListener {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private Robot theRobot = null;
	private MyRobot robot = null;
	
	private float previousX;
	private float previousY;
	
	private long dragcounter = 0; 
	private Random rand = new Random();
	
	private long framenum = 0;
	
	protected class MyDot extends Ellipse2D.Double {

		private int myNumber = -1;
		private Tag myTag = null;
		private SimulationMapCanvas parentCanvas = null;
		private Random rand = new Random();
		
		public MyDot(Tag theTag, SimulationMapCanvas c) {
			super((double) theTag.getX(), (double) theTag.getY(), (double) diameter, (double)diameter);
			myTag = theTag;
			parentCanvas = c;
			myNumber = theTag.getReadorder();
		}
		
		@Override
		public double getHeight() {
			return super.getHeight();
		}

		@Override
		public double getWidth() {
			return super.getWidth();
		}

		@Override
		public double getX() {
			return super.getX();
		}

		@Override
		public double getY() {
			return super.getY();
		}

		@Override
		public boolean isEmpty() {
			return super.isEmpty();
		}

		@Override
		public void setFrame(double arg0, double arg1, double arg2, double arg3) {
			super.setFrame(arg0, arg1, arg2, arg3);
			
		}

		@Override
		public Rectangle2D getBounds2D() {
			return super.getBounds2D();
		}
	}
	
	protected class MyRobot extends Ellipse2D.Double {

		private float robotRadius = 0.0f;
		private SimulationMapCanvas parentCanvas = null;
		
		public MyRobot(float x, float y, SimulationMapCanvas c, float radius) {
			super((double) x, (double) y, (double) radius*2, (double) radius*2);
			robotRadius = radius;
			parentCanvas = c;
			robot = this;
		}
		
		public float getRadius() {
			return robotRadius;
		}
		
		@Override
		public double getHeight() {
			return super.getHeight();
		}

		@Override
		public double getWidth() {
			return super.getWidth();
		}

		@Override
		public double getX() {
			return super.getX();
		}

		@Override
		public double getY() {
			return super.getY();
		}

		@Override
		public boolean isEmpty() {
			return super.isEmpty();
		}

		@Override
		public void setFrame(double arg0, double arg1, double arg2, double arg3) {
			super.setFrame(arg0, arg1, arg2, arg3);
			
		}

		@Override
		public Rectangle2D getBounds2D() {
			return super.getBounds2D();
		}
	}
	
	public int diameter;
	private Vector<Tag> mapping = null;
	private PeisRFIDLocalizer tm;
	
	public SimulationMapCanvas(Vector<Tag> map, PeisRFIDLocalizer tm, Robot tr) {
		diameter = 10;
		mapping = map;
		setBackground (Color.yellow);
		this.addMouseListener(this);
		this.addMouseMotionListener(this);
		this.tm = tm;
		Collections.sort(mapping);
		theRobot = tr;
		previousX = theRobot.getX();
		previousY = theRobot.getY();
	}
	

	public Dimension getPreferredSize() {
		Tag last = mapping.lastElement();
		return new Dimension((int)last.getX()+4*diameter, (int)last.getY()+4*diameter);
	}

	  
	public void setDiameter (int value) {
		diameter = value;
		repaint ();
	}

	public void paint (Graphics g) {
		super.paint(g);
		
		Graphics2D g2;
		MyDot circle;
		
		g2 = (Graphics2D) g;
		
		float centerX = theRobot.getX();
		float centerY = theRobot.getY();
		float bearing = theRobot.getBearing();
		float radius = theRobot.getRadius();
		g2.setColor (Color.black);
		g2.setStroke(new BasicStroke(4.0f));
		MyRobot robot = new MyRobot(centerX,centerY,this,radius);
		g2.draw(robot);
		Line2D lin = new Line2D.Double((double)centerX+radius, (double)centerY+radius, centerX+radius+Math.cos(Math.toRadians(bearing))*radius, centerY+radius+Math.sin(Math.toRadians(bearing))*radius);
		g2.draw(lin);

		//draw the read radius
		g2.setColor (Color.gray);
		g2.setStroke(new BasicStroke(2.0f));
		Ellipse2D.Double readRadius = new Ellipse2D.Double(centerX-(theRobot.getReadRadius()-theRobot.getRadius()), centerY-(theRobot.getReadRadius()-theRobot.getRadius()), theRobot.getReadRadius()*2, theRobot.getReadRadius()*2);
		g2.draw(readRadius);
		
		Vector<Tag> range = new Vector<Tag>();
		int inRadius = 0;
		int read = 0;
		for (Tag t : mapping) {
			if (t.getID().equals("x")) g2.setColor (Color.gray);
			else if (t.getID().equals("*")) g2.setColor (Color.red);
			else g2.setColor (Color.green);
			circle = new MyDot(t,this);
			if (readRadius.contains(circle.getCenterX(), circle.getCenterY())) {
				inRadius++;
				double probReadX = (new Gaussian(t.getInfluence(), circle.getCenterX())).getY(readRadius.getCenterX());
				double probReadY = (new Gaussian(t.getInfluence(), circle.getCenterY())).getY(readRadius.getCenterY());
				float extract = ((float)rand.nextInt(100))/100.0f;
				//System.out.println("extract: " + extract + ", probReadX: " + probReadX + ", probReadY:" + probReadY + ", sigmaSq: " + t.getInfluence());
				if (extract <= probReadX && extract <= probReadY) {
					read++;
					g2.setColor (Color.white);
					range.add(t);					
				}
			}
			g2.fill(circle);
		}
		tm.setRFIDsInRange(range);
		//System.out.println("Proportion read = " + (float)read/(float)inRadius);
	}
	
	private void dumpToDisk() {
		try
        {
            BufferedImage image = new BufferedImage(getWidth(), getHeight(), BufferedImage.TYPE_INT_RGB);
            Graphics2D graphics2D = image.createGraphics();
            paint(graphics2D);
            String framenumS = "";
            if (framenum < 10) framenumS = "0000";
            else if (framenum < 100) framenumS = "000";
            else if (framenum < 1000) framenumS = "00";
            else if (framenum < 10000) framenumS = "0";
            else if (framenum < 100000) framenumS = "";
            framenumS += new Long(framenum++).toString();
            ImageIO.write(image,"jpeg", new File("screenshot-" + framenumS));
        }
        catch(Exception exception) { exception.printStackTrace(); }
	}
	
	public boolean readProbablility(Tag t, double x, double y) {
		
		return true;
	}
	
	@Override
	public void mouseClicked(MouseEvent e) {
		
	}

	@Override
	public void mouseEntered(MouseEvent e) {
		// TODO Auto-generated method stub	
	}

	@Override
	public void mouseExited(MouseEvent e) {
		// TODO Auto-generated method stub	
	}

	@Override
	public void mousePressed(MouseEvent e) {

	}

	@Override
	public void mouseReleased(MouseEvent e) {
		dragcounter = 0;
	}

	@Override
	public void mouseDragged(MouseEvent arg0) {
		
		dragcounter++;
		
		float x = arg0.getX();
		float y = arg0.getY();
		
		if (robot.contains((double)x, (double)y)) {
			double a = Math.sqrt(Math.pow(Math.abs(x-previousX),2) + Math.pow(Math.abs(y-previousY),2));
			double b = Math.abs(y-previousY);
			if (dragcounter%7 == 0) {
				float newBearing = -1.0f;
				if (x-previousX >= 0 && y-previousY >= 0) {
					newBearing = (float)Math.toDegrees(Math.asin(b/a));
				}
				else if (x-previousX <= 0 && y-previousY >= 0) {
					newBearing = (float)Math.toDegrees(Math.PI-Math.asin(b/a));
				}
				else if (x-previousX <= 0 && y-previousY <= 0) {
					newBearing = (float)Math.toDegrees(Math.PI+Math.asin(b/a));
				}
				else if (x-previousX >= 0 && y-previousY <= 0) {
					newBearing = (float)Math.toDegrees(2*Math.PI-Math.asin(b/a));
				}
				theRobot.setBearing(newBearing);
				//System.out.println("IN HERE!!! " + newBearing);
				previousX = x;
				previousY = y;
			}
			theRobot.setX((float)x-theRobot.getRadius());
			theRobot.setY((float)y-theRobot.getRadius());
		}
		this.repaint();
		//dumpToDisk();
	}


	@Override
	public void mouseMoved(MouseEvent arg0) {
		// TODO Auto-generated method stub
		
	}
	
}