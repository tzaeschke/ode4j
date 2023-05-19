/***
 * libccd
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 * Java-port: Copyright (c) 2009-2014 Tilmann Zaeschke<ode4j@gmx.de>  
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


/**
 *
 * LibCCD list class.
 */
public class CCDList {

	static final class ccd_list_t<T> implements Iterable<T> {
	    private ccd_list_t<T> next = this;
	    private ccd_list_t<T> prev = this;
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
					throw new UnsupportedOperationException();
				}
				
			};
		}
	};

	//TZ
	private static final class ccd_iter_t<T> {
//	    for (pos = ccdListEntry((head).next, postype, member), n = ccdListEntry(pos.member.next, postype, member);	
//	     pos.member != (head); 					
//	     pos = n, n = ccdListEntry(n.member.next, ntype, member));
		private ccd_list_t<T> pos;
		private ccd_list_t<T> n;
		private final ccd_list_t<T> head;
		ccd_iter_t(ccd_list_t<T> head) {
			this.head = head;
			this.pos = head.next;
			this.n = pos.next;
		}
		boolean hasNext() {
			return pos != head;
		}
		T next() {
			T prevPos = pos.o;
			pos = n;
			n = n.next;
			return prevPos;
		}
	}
	
//	static final <T> ccd_iter_t<T> ccd_iter(ccd_list_t<T> list) {
//		return new ccd_iter_t<T>(list);
//	}
	

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
	static final <T> boolean ccdListEmpty(final ccd_list_t<T> head)
	{
	    return head.next == head;
	}

	/**
	 * Appends item to end of the list l.
	 */
	static final <T> void ccdListAppend(ccd_list_t<T> l, ccd_list_t<T> newccd)
	{
	    newccd.prev = l.prev;
	    newccd.next = l;
	    l.prev.next = newccd;
	    l.prev = newccd;
	}

	/**
	 * Removes item from list.
	 */
	static final <T> void ccdListDel(ccd_list_t<T> item)
	{
	    item.next.prev = item.prev;
	    item.prev.next = item.next;
	    item.next = item;
	    item.prev = item;
	}

	private CCDList() {}
}
