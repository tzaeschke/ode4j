package org.cpp4j.java;

public final class RefString {
	public String b;

	public RefString(String b) {
		this.b = b;
	}
	
	public final void set(String b) {
		this.b = b;
	}

	public final String get() {
		return b;
	}
}
