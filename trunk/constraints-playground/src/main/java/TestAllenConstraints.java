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
import org.metacsp.multi.allenInterval.AllenIntervalNetworkSolver;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;

public class TestAllenConstraints {
	
	public static void main(String[] args) {
		ActivityNetworkSolver intervalsAndSymbolsSolver = new ActivityNetworkSolver(0,500);
		AllenIntervalNetworkSolver intervalsSolver = (AllenIntervalNetworkSolver)intervalsAndSymbolsSolver.getConstraintSolvers()[0];
		APSPSolver stpSolver = (APSPSolver)intervalsSolver.getConstraintSolvers()[0];
		
		//create high-level variables (tokens)
		Activity token1 = (Activity)intervalsAndSymbolsSolver.createVariable("State variable 1");
		token1.setSymbolicDomain("Symbol A");
		Activity token2 = (Activity)intervalsAndSymbolsSolver.createVariable("State variable 1");
		token2.setSymbolicDomain("Symbol B");
		
		ConstraintNetwork.draw(intervalsAndSymbolsSolver.getConstraintNetwork(), "Symbols and Intervals");
		ConstraintNetwork.draw(intervalsSolver.getConstraintNetwork(), "Intervals");
		ConstraintNetwork.draw(stpSolver.getConstraintNetwork(), "Simple Temporal Problem");

		AllenIntervalConstraint con1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before, new Bounds(10, 20));
		con1.setFrom(token1);
		con1.setTo(token2);
		
		intervalsAndSymbolsSolver.addConstraint(con1);

	}

}
