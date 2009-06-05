package org.ode4j.ode.internal;

import org.cpp4j.java.Ref;


// base class for bodies and joints

public abstract class DObject extends DBase {
	
	public DxWorld world;		// world this object is in
//	  dObject *next;		// next object of this type in list
//	  dObject **tome;		// pointer to previous object's next ptr
//	private Ref<dObject> _next;		// next object of this type in list
	private DObject _next;		// next object of this type in list
	//private dObject _prev;		// pointer to previous object (TZ)
//	private dObject tome;		// pointer to previous object's next ptr
	private Ref<DObject> _tome;		// pointer to previous object's next ptr
	public int tag;			// used by dynamics algorithms
	//void userdata;		// user settable data
	protected Object userdata;		// user settable data
	protected DObject(DxWorld w) { //From ODE.java
		world = w;
		_next = null;
		_tome = null;
		userdata = null;
		tag = 0;
	}
	
	// add an object `obj' to the list who's head pointer is pointed 
	//to by `first'.

	//static void addObjectToList (dObject *obj, dObject **first)
	public static <T extends DObject>void addObjectToList (T obj, 
			Ref<T> first)
	{
//		System.err.println("ADDING OBJ: " + obj.getClass().getName());
		//  obj.next = *first;
		//  obj.tome = first;
		//  if (first != null) first.tome = &obj.next;
		//  first = obj;
		obj._next = first.get();
		obj._tome = (Ref<DObject>) first;
		if (first.get() != null) first.get()._tome.set(obj._next);
		first.set(obj);
	}


	// remove the object from the linked list

//	public static <T extends dObject>void removeObjectFromList (T obj)
	public <T extends DObject>void removeObjectFromList ()
	{
		//System.err.println("REMOVING OBJ: " + getClass().getName() + " / " + toString());
		//	  if (obj.next) obj.next.tome = obj.tome;
		//	  *(obj.tome) = obj.next;
		//	  // safeguard
		//	  obj.next = 0;
		//	  obj.tome = 0;
		if (_next != null) _next._tome = _tome;
		_tome.set(_next);
		// safeguard
		_next = null;
		_tome = null;
	}
	
	public DxWorld getWorld() {
		return world;
	}
	
	protected DObject getNext() {
		return _next;
	}
	
	protected DObject getTome() {
		return _tome.get();
	}
}
