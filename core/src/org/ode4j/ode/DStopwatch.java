package org.ode4j.ode;

public class DStopwatch {

	private double _time;			/* total clock count */
	private long _cc;   /* clock count since last `start' */
	
	//****************************************************************************
	// stop watches

	public void reset () {
		_time = 0;
		_cc = 0;
	}


	public void start () {
		_cc = System.nanoTime();
	}


	public void stop () {
		_time += System.nanoTime() - _cc;
	}


	public double getTime () {
		return _time * 10e-9;
	}
}
