/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zaeschke     *
 * All rights reserved.  Email: ode4j@gmx.de   Web: www.ode4j.org        *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.         *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT, ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT for more   *
 * details.                                                              *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode.internal;

import org.ode4j.ode.internal.cpp4j.java.Ref;


/**
 * Base class for bodies and joints.
 */
public abstract class DObject extends DBase {
   
    public DxWorld world;        // world this object is in
//      dObject *next;        // next object of this type in list
//      dObject **tome;        // pointer to previous object's next ptr
    private final Ref<DObject> _next;        // next object of this type in list
    private Ref<DObject> _tome;        // pointer to previous object's next ptr
    public int tag;            // used by dynamics algorithms
    //void userdata;        // user settable data
    protected Object userdata;        // user settable data
    protected DObject(DxWorld w) { //From ODE.java
        world = w;
        _next = new Ref<DObject>();
        _tome = null;
        userdata = null;
        tag = 0;
    }
   
    /**
     * Add an object `obj' to the list who's head pointer is pointed
     * to by `first'.
     * This is equivalent to obj.addObjectToList(first).
     * @param <T> Type of the object/list
     * @param obj Object that should be added to a List.
     * @param first List to which the object should be added.
     * @see DObject#addObjectToList(Ref)
     */
    //static void addObjectToList (dObject *obj, dObject **first)
    public static <T extends DObject> void addObjectToList (T obj, Ref<T> first) {
        obj.addObjectToList(first);
    }

    /**
     * Add an object `obj' to the list who's head pointer is pointed
     * to by `first'.
     * TODO This is quite weird, having the list function on the obj rather than
     * on the list...
     * @param <T> Type of the object/list
     * @param first List to which the object should be added.
     */
    @SuppressWarnings("unchecked")
    protected <T extends DObject> void addObjectToList (Ref<T> first)
    {
        //  obj.next = *first;
        //  obj.tome = first;
        //  if (first != null) first.tome = &obj.next;
        //  first = obj;
        _next.set(first.get());
        _tome = (Ref<DObject>) first;
        if (first.get() != null) {
            first.get().setTome(_next);
        }
        first.set((T) this);
    }

    protected void setTome(Ref<DObject> obj) {
        _tome = obj;
    }

    DObject getTome() {
        return _tome.get();
    }

    /**
     * Remove the object from the linked list.
     */
//    public static <T extends dObject>void removeObjectFromList (T obj)
    public void removeObjectFromList ()
    {
        //      if (obj.next) obj.next.tome = obj.tome;
        //      *(obj.tome) = obj.next;
        //      // safeguard
        //      obj.next = 0;
        //      obj.tome = 0;
        if (_next.get() != null) _next.get()._tome = _tome;
        _tome.set(_next.get());
        // safeguard
        _next.set(null);
        _tome = null;
    }
   
    public DxWorld getWorld() {
        return world;
    }
   
    public DObject getNext() {
        return _next.get();

    }
}
