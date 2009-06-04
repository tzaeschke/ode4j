package org.cpp4j.java;

public final class RefInt {
	public int i = 0;

	public RefInt() {
		// Nothing
	}

	public RefInt(int v) {
		i = v;
	}

	public final void set(int v) {
		i = v;
	}

	public final int get() {
		return i;
	}

	public final void inc() {
		i++;
	}

	public void add(int j) {
		i += j;
	}
}
