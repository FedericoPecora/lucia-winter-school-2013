
package se.oru.lucia.rfidlocalization;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.GridLayout;
import java.awt.Toolkit;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.Vector;

import javax.swing.*;

public class PeisRFIDLocalizer extends JFrame {
	
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	private float side = 20.0f;
	private float height = (float) Math.sqrt((double)3.0)/2.0f*side;
	public String mapFile = null;
	//private String mapFile = "./maps/map.txt.SAVE.4";
	
	private String inRange = "";
	
	private Robot simRobot;
	private Robot locRobot;
	
	public String tagTupleName = "tags";
	public int tagPeisId = 999;
	public boolean createPeisMap = false;
	
	public int substituteNumber = -1;

	private SimulationMapCanvas mcSim = null;
	private LocalizationMapCanvas mcLoc = null;
	private int currentUnread = 1;
	
	public Vector<Tag> mapping;
	
	private String computedLocation = "NOPOS";
	private float computedX = -1;
	private float computedY = -1;
	private float computedBearing = -1;
	
	private PositionEstimator pe;

	public String getComputedLocation() { return computedLocation; }
	public float getComputedX() { return computedX; }
	public float getComputedY() { return computedY; }
	public float getComputedBearing() { return computedBearing; }
	
	private class Bounds {
		private float xlb;
		private float xub;
		private float ylb;
		private float yub;
		public Bounds(float xl, float xu, float yl, float yu) {
			xlb = xl;
			xub = xu;
			ylb = yl;
			yub = yu;
		}
		public boolean contains(float x, float y) {
			return (x >= xlb &&  x <= xub && y >= ylb && y <= yub);
		}
	}
	
	public PeisRFIDLocalizer(String frameTitle, String mf) {
		super(frameTitle);
		mapFile = mf;
		mapping = new Vector<Tag>();
		
		simRobot = new Robot(20.0f,25.0f,"Simulation");
		simRobot.setX(110.0f);
		simRobot.setY(200.0f);
		simRobot.setBearing(45.0f);
		
		locRobot = new Robot(20.0f,60.0f,"Localization");
		
		this.readFile();
		this.makeGUI(simRobot,locRobot);
		
		pe = new PositionEstimator(locRobot, this, mcLoc);
	}
	
	public void updateLocation() {
		pe.computeLocation();
		//Kitchen: y > 700, 0 < x < 266
		//DiningRoom: y > 700, x > 266
		//LivingRoom: 320 < y < 700, 0 < x 600
		//BedRoom: 0 < y < 320, 0 < x < 400
		//Entrance: 100 < y < 320, 400 < x 600
		//NOPOS: 0 < y < 100, 400 < x 600
		String[] locations = {"Kitchen", "DiningRoom", "LivingRoom", "BedRoom", "Entrance", "NOPOS"};
		Bounds[] bounds = {
				new Bounds(0.0f, 266.0f, 700.0f, 900.0f),
				new Bounds(266.0f, 600.0f, 700.0f, 900.0f),
				new Bounds(0.0f, 600.0f, 320.0f, 700.0f),
				new Bounds(0.0f, 400.0f, 0.0f, 320.0f),
				new Bounds(400.0f, 600.0f, 100.0f, 320.0f),
				new Bounds(400.0f, 600.0f, 0.0f, 100.0f),				
			};
		for (int i = 0; i < locations.length; i++) {
			if (bounds[i].contains(locRobot.getX(), locRobot.getY())) {
				computedLocation = locations[i];
				break;
			}
		}
		computedX = locRobot.getX();
		computedY = locRobot.getY();
		computedBearing = locRobot.getBearing();
		System.out.println("Estimated bearing: (deg) = " + computedBearing);
		System.out.println("Estimated position: (x,y) = (" + computedX + "," + computedY + ")");

	}
	
	public int getSubstituteNumber() {
		return substituteNumber;
	}

	public void setSubstituteNumber(int num) {
		substituteNumber = num;
	}

	public void resetSubstituteNumber() {
		substituteNumber = -1;
	}
	
	public void backtrack() {
		currentUnread--;
	}


	public Tag getNextTag() {
		//return getFromReadNumber(currentUnread++);
		return getLowestUnread();
	}
	
	public Tag getFromID(String tagID) {
		for (Tag t : mapping) {
			//System.out.println("Comparing " + t.getID() + " and " + tagID);
			if (t.getID().equals(tagID)) {
				//System.out.println("Matched " + t.getID() + " and " + tagID);
				return t;
			}
		}
		return null;
	}

	public Tag getFromReadNumber(int rn) {
		for (Tag t : mapping) {
			if (t.getReadorder() == rn) {
				return t;
			}
		}
		return null;
	}

	public Tag getLowestUnread() {
		Collections.sort(mapping);
		for (Tag t : mapping)
			if (t.getID().equals("*")) return t;
		return null;
	}

	public void printMapping() {
		Collections.sort(mapping);
		for (Tag t : mapping)
			System.out.println(t);
	}

	public void printMapping(String tagID) {
		for (Tag t : mapping) {
			if (t.getID().equals(tagID)) {
				System.out.println(t);
				break;
			}
		}
	}

	public void printMapping(int order) {
		for (Tag t : mapping) {
			if (t.getReadorder() == order) {
				System.out.println(t);
				break;
			}
		}
	}
	
	public String getRFIDsInRange() {
		//System.out.println(inRange.length());
		return inRange;
	}
	
	public void setRFIDsInRangeReal(String tags) {
		inRange = tags;
	}
	
	public void setRFIDsInRange(Vector<Tag> range) {
		inRange = "";
		for (Tag t : range)
			inRange += ("(" + t.getID() + ")");
	}
	
	public void readFile() {
		try {
		    BufferedReader in = new BufferedReader(new FileReader(mapFile));
		    String str;
		    int gridCounterX = -1;
		    int gridCounterY = -1;
		    float d = 0.0f;
		    float h = height;
		    int tagnumber = 0;
		    while ((str = in.readLine()) != null) {
		    	gridCounterY++;
		    	boolean done = false;
		    	boolean first = true;
		    	while (!done) {
		    		try {
			    		char oneChar = str.charAt(0);
			    		if (oneChar == ' ') {
			    			if (first) first = false;
			    			d += side;
			    			str = str.substring(1);
			    		}
			    		else if (oneChar == '*' || oneChar == 'x') {
				    		gridCounterX++;
			    			if (first) {
			    				first = false;
			    				d -= 10;
			    				d += side;
			    			}
			    			tagnumber++;
			    			Tag c = new Tag(d,h,tagnumber,oneChar+"",gridCounterX,gridCounterY);
			    			mapping.add(c);
			    			str = str.substring(1);
			    		}
			    		//else we have a previously read tag!!
			    		else {
				    		gridCounterX++;
			    			if (first) {
			    				first = false;
			    				d -= 10;
			    				d += side;
			    			}
			    			String tempStr = str.substring(0,str.indexOf(' '));
			    			str = str.substring(str.indexOf(' '));
			    			tagnumber++;
			    			Tag c = new Tag(d,h,tagnumber,tempStr,gridCounterX,gridCounterY);
			    			mapping.add(c);
			    			currentUnread = tagnumber+1;
			    		}
		    		}
		    		catch(Exception e) {done = true;}
		    	}
		    	gridCounterX = -1;
		    	d = 0.0f;
		        h += height;
		    }
		    in.close();
		} catch (IOException e) {
			System.err.println( "Error reading from input file: " + this.mapFile );
			System.exit(0);
		}
		catch (NullPointerException e) {
			System.err.println( "Error reading from input file: " + this.mapFile );
			System.exit(0);
		}
		System.out.println("Read " + mapping.size() + " tags");
	}
	
	private void makeGUI(Robot sRobot, Robot lRobot) {
		Container c = this.getContentPane();
		Toolkit tk = Toolkit.getDefaultToolkit();  
	    this.setLocation(0,0);
	    
	    JLabel sName = null;
	    sName = new JLabel(sRobot.getName());
	    JLabel lName = new JLabel(lRobot.getName());
	    	    
	    mcLoc = new LocalizationMapCanvas(mapping,this,lRobot);
	    JScrollPane jspLoc = new JScrollPane(mcLoc);

    	mcSim = new SimulationMapCanvas(mapping,this,sRobot);
    	JScrollPane jspSim = new JScrollPane(mcSim);

	    GroupLayout layout = new GroupLayout(c);
	    c.setLayout(layout);
	    layout.setAutoCreateGaps(true);
	    layout.setAutoCreateContainerGaps(true);
	    layout.setHorizontalGroup(
	    		   layout.createSequentialGroup()
	    		      .addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)
	    		           .addComponent(sName)
	    		           .addComponent(jspSim))
	    		      .addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)
	    		           .addComponent(lName)
	    		           .addComponent(jspLoc))
	    		);
	    layout.setVerticalGroup(
	    		   layout.createSequentialGroup()
	    		      .addGroup(layout.createParallelGroup(GroupLayout.Alignment.BASELINE)
	    		           .addComponent(sName)
	    		           .addComponent(lName))
	    		      .addGroup(layout.createParallelGroup(GroupLayout.Alignment.BASELINE)
	    		           .addComponent(jspSim)
	    		           .addComponent(jspLoc))
	    		);
	    //int xSize = ((int) tk.getScreenSize().getWidth());  
		//int ySize = ((int) tk.getScreenSize().getHeight());
	    int xSize = mcSim.getPreferredSize().width + mcSim.getPreferredSize().width;
		int ySize = mcLoc.getPreferredSize().height + mcLoc.getPreferredSize().height;    
		this.setSize(xSize,ySize);
	    this.setVisible(true);
	    this.pack();
	}
	
	public void update(Graphics g) {
		super.update(g);
		this.updateGraphics();
	}
	
	public void updateGraphics() {
		this.mcSim.repaint();
		this.mcLoc.repaint();
	}
	
	public void makePeisMap() {
		try {
	        FileWriter rawOut = new FileWriter( mapFile + ".PEISMAP.txt");
	        PrintWriter out = new PrintWriter( rawOut );
	        for (Tag t : mapping) {
				out.println(t.getID() + "\t" + "(" + t.getX()/100 + " " + t.getY()/100 + ")\t" + "(" + t.getGridX() + "," + t.getGridY() + ")");	        	
	        }
	        out.close();
	    }
	    catch ( IOException error ) {
	        System.err.println( "Error writing to output file: " + error );
	    }
	}
	
	public Robot getSimRobot() {
		return simRobot;
	}

	public Robot getLocRobot() {
		return locRobot;
	}

}
