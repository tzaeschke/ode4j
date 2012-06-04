/***
 * libccd
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 * Java-port: Copyright (c) 2007-2012 Tilmann Zäschke <ode4j@gmx.de>  
 *
 *
 *  This file is part of libccd.
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see accompanying file BDS-LICENSE for details or see
 *  <http://www.opensource.org/licenses/bsd-license.php>.
 *
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */
package org.ode4j.ode.internal.libccd;

import java.util.Iterator;

public class CCDList {

	static final class ccd_list_t<T> implements Iterable<T> {
	    ccd_list_t<T> next;
	    ccd_list_t<T> prev;
	    final T o; 
	    public ccd_list_t(T obj) {
	    	this.o = obj;
		}
		@Override
		public Iterator<T> iterator() {
			return new Iterator<T>() {
				private final ccd_iter_t<T> iter = new ccd_iter_t<T>(ccd_list_t.this);
				@Override
				public boolean hasNext() {
					return iter.hasNext();
				}

				@Override
				public T next() {
					return iter.next();
				}

				@Override
				public void remove() {
					// TODO Auto-generated method stub
					
				}
				
			};
		}
	};

	//TZ
	static final class ccd_iter_t<T> {
		private ccd_list_t<T> next;
		ccd_iter_t(ccd_list_t<T> start) {
			next = start;
		}
		boolean hasNext() {
			return next != null;
		}
		T next() {
			T ret = next.o;
			next = next.next;
			return ret;
		}
	}
	
	static final <T> ccd_iter_t<T> ccd_iter(ccd_list_t<T> list) {
		return new ccd_iter_t<T>(list);
	}
	

//	/**
//	 * Get the struct for this entry.
//	 * @ptr:	the &ccd_list_t pointer.
//	 * @type:	the type of the struct this is embedded in.
//	 * @member:	the name of the list_struct within the struct.
//	 */
//	static final void ccdListEntry(ptr, type, member) {
//	    ccd_container_of(ptr, type, member);
//	}
//
//	/**
//	 * Iterates over list.
//	 */
//	static final void ccdListForEach(ccd_list_t list, ccd_list_t item) {
//	        for (item = (list).next; 
//	             _ccd_prefetch((item).next), item != (list); 
//	             item = (item).next);
//	}
//
//	/**
//	 * Iterates over list safe against remove of list entry
//	 */
//	static final void ccdListForEachSafe(ccd_list_t list, ccd_list_t item, tmp) {
//		    for (item = (list).next, tmp = (item).next; 
//	             item != (list); 
//			     item = tmp, tmp = (item).next);
//	}
//
//	/**
//	 * Iterates over list of given type.
//	 * @pos:	the type * to use as a loop cursor.
//	 * @head:	the head for your list.
//	 * @member:	the name of the list_struct within the struct.
//	 */
//	static final void ccdListForEachEntry(ccd_list_t head, pos, postype, member) {
//		for (pos = ccdListEntry((head).next, postype, member);	
//		     _ccd_prefetch(pos.member.next), pos.member != (head); 	
//		     pos = ccdListEntry(pos.member.next, postype, member));
//	}
//
//	/**
//	 * Iterates over list of given type safe against removal of list entry
//	 * @pos:	the type * to use as a loop cursor.
//	 * @n:		another type * to use as temporary storage
//	 * @head:	the head for your list.
//	 * @member:	the name of the list_struct within the struct.
//	 */
//	static final void ccdListForEachEntrySafe(ccd_list_t head, pos, postype, n, ntype, member) {
//	    for (pos = ccdListEntry((head).next, postype, member),             
//			 n = ccdListEntry(pos.member.next, postype, member);	
//		     pos.member != (head); 					
//		     pos = n, n = ccdListEntry(n.member.next, ntype, member));
//	}







	///
	/// INLINES:
	///

   	/**
   	 * Initialize list.
   	 */
	@SuppressWarnings({ "rawtypes", "unchecked" })
	static final void ccdListInit(ccd_list_t l)
	{
	    l.next = l;
	    l.prev = l;
	}

//	static final ccd_list_t ccdListNext(ccd_list_t l)
//	{
//	    return l.next;
//	}
//
//	static final ccd_list_t ccdListPrev(ccd_list_t l)
//	{
//	    return l.prev;
//	}

	/**
	 * Returns true if list is empty.
	 */
	@SuppressWarnings("rawtypes")
	static final boolean ccdListEmpty(final ccd_list_t head)
	{
	    //return head.next == head;
		return head.next == null;
	}

	/**
	 * Appends item to end of the list l.
	 */
	@SuppressWarnings({ "rawtypes", "unchecked" })
	static final void ccdListAppend(ccd_list_t l, ccd_list_t newccd)
	{
	    newccd.prev = l.prev;
	    newccd.next = l;
	    l.prev.next = newccd;
	    l.prev = newccd;
	}

	/**
	 * Removes item from list.
	 */
	@SuppressWarnings({ "rawtypes", "unchecked" })
	static final void ccdListDel(ccd_list_t item)
	{
	    item.next.prev = item.prev;
	    item.prev.next = item.next;
	    item.next = null;//item;
	    item.prev = null;//item;
	}
}
