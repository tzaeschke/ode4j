package org.cpp4j.java;

public final class RefDouble {
	public double d;

	public RefDouble() {
		this.d = 0;
	}
	
	public RefDouble(double d) {
		this.d = d;
	}
	
	public final void set(double d) {
		this.d = d;
	}

	public final double get() {
		return d;
	}

	public final float getF() {
		return (float) d;
	}

	public void sub(double e) {
		d -= e;
	}
}
