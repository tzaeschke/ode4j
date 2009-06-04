package org.cpp4j.java;

final public class Ref<T> {
	public T r;
	public Ref() {
		r = null;
	}
	
	final public T get() {
		return r;
	}
	
	final public void set(T r) {
		this.r = r;
	}
}
