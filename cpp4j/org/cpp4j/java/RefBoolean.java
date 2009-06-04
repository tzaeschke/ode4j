package org.cpp4j.java;

public final class RefBoolean {
	public boolean b;

	public RefBoolean(boolean b) {
		this.b = b;
	}
	
	public final void set(boolean b) {
		this.b = b;
	}

	public final boolean get() {
		return b;
	}
}
