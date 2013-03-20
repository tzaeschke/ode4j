/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2013 Tilmann Zaeschke     *
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

import java.util.ArrayList;


/**
 * this comes from the `reuse' library. copy any changes back to the source.
 *
 * Variable sized array template. The array is always stored in a contiguous
 * chunk. The array can be resized. A size increase will cause more memory
 * to be allocated, and may result in relocation of the array memory.
 * A size decrease has no effect on the memory allocation.
 *
 * Array elements with constructors or destructors are not supported!
 * But if you must have such elements, here's what to know/do:
 *   - Bitwise copy is used when copying whole arrays.
 *   - When copying individual items (via push(), insert() etc) the `='
 *     (equals) operator is used. Thus you should define this operator to do
 *     a bitwise copy. You should probably also define the copy constructor.
 *     
 *     
 * (TZ) @deprecated Remove if possible
 */
class DArray<T> {//extends dArrayBase<T> {
	private ArrayList<T> _data = new ArrayList<T>();
	//template <class T> class dArray : public dArrayBase {
	//public:
//	public void equals (final dArray<T> x) {
//		setSize (x.size());
//		memcpy (_data,x._data,x._size * sizeof(T));
//	}
//
//	public dArray () { constructor(); }
//	public dArray (final dArray<T> x) { constructor(); equals (x); }
//	//~dArray () { _freeAll(sizeof(T)); }
//	protected void finalize() throws Throwable {
//		_freeAll(sizeof(T));
//	}
//	public void setSize (int newsize) { _setSize (newsize,sizeof(T)); }
	public void setSize(int i) {
		if (i == 0) {
			_data.clear();
			return;
		}
		if (i > size()) {
			return;  //Will be allocated dynamically
		}
		//Reduction of size not implemented
		while (size() > i) {
			remove(size()-1);
		}
		//TODO
		//throw new IllegalArgumentException("i=" + i + "; size=" + size());
	}
	
//	public T data() { return (T) _data; }
//	public T operator[] (int i) { return ((T)_data)[i]; }
	public T get(int i) {
		return _data.get(i);
	}
	
//	public void operator = (final dArray<T> x) { equals (x); }
//
//	public void push (final T item) {
//		if (_size < _anum) _size++; else _setSize (_size+1,sizeof(T));
//		memcpy ((((T)_data)[_size-1]), item, sizeof(T));
//	}
	public void push(T item) {
		_data.add(item);
	}
//
//	public void swap (dArray<T> x) {
//		int tmp1;
//		T tmp2;
//		tmp1=_size; _size=x._size; x._size=tmp1;
//		tmp1=_anum; _anum=x._anum; x._anum=tmp1;
//		tmp2=_data; _data=x._data; x._data=tmp2;
//	}
//
//	// insert the item at the position `i'. if i<0 then add the item to the
//	// start, if i >= size then add the item to the end of the array.
//	public void insert (int i, final T item) {
//		if (_size < _anum) _size++; else _setSize (_size+1,sizeof(T));
//		if (i >= (_size-1)) i = _size-1;	// add to end
//		else {
//			if (i < 0) i=0;			// add to start
//			int n = _size-1-i;
//			if (n>0) memmove (((T)_data) + i+1, ((T)_data) + i, n*sizeof(T));
//		}
//		((T)_data)[i] = item;
//	}
//
//	public void remove (int i) {
//		if (i >= 0 && i < _size) {	// passing this test guarantees size>0
//			int n = _size-1-i;
//			if (n>0) memmove (((T)_data) + i, ((T)_data) + i+1, n*sizeof(T));
//			_size--;
//		}
//	}
	public void remove(int i) {
		_data.remove(i);
	}


	public int size() {
		return _data.size();
	}
	
//	static int roundUpToPowerOfTwo (int x)
//	{
//		int i = 1;
//		while (i < x) i <<= 1;
//		return i;
//	}

	public void set(int index, T element) {
		if (index == size()) {
			_data.add(element);
			return;
		}
		_data.set(index, element);
	}
}