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
import java.util.Collections;
import java.util.HashMap;
import java.util.Random;
import java.util.Vector;

import javax.swing.JLabel;
import javax.swing.JPanel;

public class LocalizationMapCanvas extends JPanel {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private Robot theRobot = null;
	private MyRobot robot = null;
	
	private float previousX;
	private float previousY;
	
	protected class MyDot extends Ellipse2D.Double {

		private int myNumber = -1;
		private Tag myTag = null;
		private LocalizationMapCanvas parentCanvas = null;
		
		public MyDot(Tag theTag, LocalizationMapCanvas c) {
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
		private LocalizationMapCanvas parentCanvas = null;
		
		public MyRobot(float x, float y, LocalizationMapCanvas c, float radius) {
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
	
	public LocalizationMapCanvas(Vector<Tag> map, PeisRFIDLocalizer tm, Robot tr) {
		diameter = 10;
		mapping = map;
		setBackground (Color.yellow);
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
		/*
		g2.setColor (Color.gray);
		g2.setStroke(new BasicStroke(2.0f));
		Ellipse2D.Double readRadius = new Ellipse2D.Double(centerX-(theRobot.getReadRadius()-theRobot.getRadius()), centerY-(theRobot.getReadRadius()-theRobot.getRadius()), theRobot.getReadRadius()*2, theRobot.getReadRadius()*2);
		g2.draw(readRadius);
		*/
		
		for (Tag t : mapping) {
			if (t.getID().equals("x")) g2.setColor (Color.gray);
			else if (t.getID().equals("*")) g2.setColor (Color.red);
			else g2.setColor (Color.green);
			circle = new MyDot(t,this);
			/*
			if (readRadius.contains(circle.getCenterX(), circle.getCenterY())) {
					g2.setColor (Color.white);
			}
			*/
			g2.fill(circle);
		}
	}
	
}