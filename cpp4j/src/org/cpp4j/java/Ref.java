package org.cpp4j.java;


final public class Ref<T> {
	public T r;
	public Ref() {
		r = null;
	}
	
	public Ref(T obj) {
		r = obj;
	}

	public final T get() {
		return r;
	}
	
	public final void set(T r) {
		this.r = r;
	}
}
