/*******************************************************************************
 * Copyright (c) 2010-2013 Federico Pecora <federico.pecora@oru.se>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************/


import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.multi.activity.Activity;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.time.Bounds;
import org.metacsp.utility.timelinePlotting.TimelinePublisher;
import org.metacsp.utility.timelinePlotting.TimelineVisualizer;

public class TestTimelines {
	
	public static void main(String[] args) {
		
		ActivityNetworkSolver solver = new ActivityNetworkSolver(0,100);
		Activity tokenA = (Activity)solver.createVariable("State variable 1");
		tokenA.setSymbolicDomain("Token A");
		Activity tokenB = (Activity)solver.createVariable("State variable 2");
		tokenB.setSymbolicDomain("Token B");
		Activity tokenC = (Activity)solver.createVariable("State variable 3");
		tokenC.setSymbolicDomain("Token C");

		AllenIntervalConstraint dur1 = new AllenIntervalConstraint( AllenIntervalConstraint.Type.Duration, new Bounds(10, 20));
		dur1.setFrom(tokenA);
		dur1.setTo(tokenA);

		AllenIntervalConstraint dur2 = new AllenIntervalConstraint( AllenIntervalConstraint.Type.Duration, new Bounds(10, 20));
		dur2.setFrom(tokenB);
		dur2.setTo(tokenB);

		AllenIntervalConstraint dur3 = new AllenIntervalConstraint( AllenIntervalConstraint.Type.Duration, new Bounds(10, 20));
		dur3.setFrom(tokenC);
		dur3.setTo(tokenC);

		AllenIntervalConstraint tokenADuringTokenB = new AllenIntervalConstraint( AllenIntervalConstraint.Type.During);
		tokenADuringTokenB.setFrom(tokenA);
		tokenADuringTokenB.setTo(tokenB);

		AllenIntervalConstraint tokenAEqualsTokenC = new AllenIntervalConstraint( AllenIntervalConstraint.Type.Equals);
		tokenAEqualsTokenC.setFrom(tokenA);
		tokenAEqualsTokenC.setTo(tokenC);

		solver.addConstraints(dur1,dur2,dur3,tokenADuringTokenB,tokenAEqualsTokenC);
		
    	//Timeline visualization
		TimelinePublisher tp = new TimelinePublisher(solver, new Bounds(0,100), false, "State variable 1", "State variable 2", "State variable 3");
		TimelineVisualizer tv = new TimelineVisualizer(tp);
		tp.setTemporalResolution(1);
		tp.publish(true, false);
		
		ConstraintNetwork.draw(solver.getConstraintSolvers()[0].getConstraintNetwork());
		
		AllenIntervalConstraint con3 = null;
		
		for (int timeCounter = 0; timeCounter < 200; timeCounter++) {
			try { Thread.sleep(500); }
			catch (InterruptedException e) { e.printStackTrace(); }
			if (con3 != null) solver.removeConstraint(con3);
			con3 = new AllenIntervalConstraint( AllenIntervalConstraint.Type.Release, new Bounds(solver.getOrigin()+timeCounter, solver.getOrigin()+timeCounter));
			con3.setFrom(tokenB);
			con3.setTo(tokenB);
			solver.addConstraint(con3);
			tp.publish(false,true);
		}		
	}
	

}
