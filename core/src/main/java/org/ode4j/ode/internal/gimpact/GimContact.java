/**
 * ----------------------------------------------------------------------------
 * This source file is part of the ODE4J library (ported to
 * Java from the GIMPACT Library).
 * 
 * For the latest info on ODE4J, see http://www.ode4j.org/
 * For the latest info on GIMPACT, see http://gimpact.sourceforge.net/
 * 
 * Copyright of GIMPACT (c) 2006 Francisco Leon. C.C. 80087371.
 * email: projectileman@yahoo.com
 * Copyright of ODE4J (c) 2009-2014 Tilmann Zäschke.
 * email: ode4j.gmx.de
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of EITHER:
 *   (1) The GNU Lesser General Public License as published by the Free
 *       Software Foundation; either version 2.1 of the License, or (at
 *       your option) any later version. The text of the GNU Lesser
 *       General Public License is included with this library in the
 *       file GIMPACT-LICENSE-LGPL.TXT and LICENSE.TXT.
 *   (2) The BSD-style license that is included with this library in
 *       the file GIMPACT-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 * GIMPACT-LICENSE-LGPL.TXT, GIMPACT-LICENSE-BSD.TXT, LICENSE.TXT and 
 * ODE4J-LICENSE-BSD.TXT for more details.
 * 
 * ----------------------------------------------------------------------------
 */
package org.ode4j.ode.internal.gimpact;

import static org.ode4j.ode.internal.gimpact.GimGeometry.VEC_ACCUM;
import static org.ode4j.ode.internal.gimpact.GimGeometry.VEC_COPY;
import static org.ode4j.ode.internal.gimpact.GimGeometry.VEC_DOT;
import static org.ode4j.ode.internal.gimpact.GimGeometry.VEC_NORMALIZE;
import static org.ode4j.ode.internal.gimpact.GimGeometry.VEC_SCALE;
import static org.ode4j.ode.internal.gimpact.GimGeometry.VEC_SUM;
import static org.ode4j.ode.internal.gimpact.GimMath.GIM_SQRT;

import org.ode4j.ode.internal.cpp4j.java.ObjArray;
import org.ode4j.ode.internal.gimpact.GimGeometry.vec3f;
import org.ode4j.ode.internal.gimpact.GimGeometry.vec4f;
import org.ode4j.ode.internal.gimpact.GimRadixSort.GIM_RSORT_TOKEN;

/**
 * Structure for collision results.
 * 
 * Functions for managing and sorting contacts resulting from a collision query.
 * <ul>
 * <li> Contact lists must be create by calling \ref GIM_CREATE_CONTACT_LIST
 * <li> After querys, contact lists must be destroy by calling \ref GIM_DYNARRAY_DESTROY
 * <li> Contacts can be merge for avoid duplicate results by calling \ref gim_merge_contacts
 * </ul>
 *
 * Ported to Java by Tilmann Zaeschke
 * @author Francisco Leon
 */
public class GimContact {

	/** Structure for collision results. */
//	struct GIM_CONTACT
//	{
//	    vec3f m_point;
//	    vec3f m_normal;
//	    GREAL m_depth;//Positive value indicates interpenetration
//	    void * m_handle1;
//	    void * m_handle2;
//	    GUINT32 m_feature1;//Face number
//	    GUINT32 m_feature2;//Face number
	final vec3f m_point = new vec3f();
	final vec3f m_normal = new vec3f();
	float m_depth;//Positive value indicates interpenetration
	Object m_handle1;
	Object m_handle2;
	int m_feature1;//Face number
	int m_feature2;//Face number
//	};
	//typedef struct _GIM_CONTACT GIM_CONTACT;

	private GimContact() {}

	public vec3f getPoint() {
		return m_point;
	}

	public vec3f getNormal() {
		return m_normal;
	}
	
	public float getDepth() {
		return m_depth;
	}
	
	public int getFeature1() {
		return m_feature1;
	}
	
	public int getFeature2() {
		return m_feature2;
	}
	
	
	//#define CONTACT_DIFF_EPSILON 0.00001f
	private static final float CONTACT_DIFF_EPSILON = 0.00001f;

	//#define GIM_CALC_KEY_CONTACT(pos,hash)\
	private static int GIM_CALC_KEY_CONTACT(vec3f pos)
	{
//		GINT32 _coords[] = {(GINT32)(pos[0]*1000.0f+1.0f),(GINT32)(pos[1]*1333.0f),(GINT32)(pos[2]*2133.0f+3.0f)};
//		GUINT32 _hash=0;
//		GUINT32 *_uitmp = (GUINT32 *)(&_coords[0]);
//		_hash = *_uitmp;
//		_uitmp++;
//		_hash += (*_uitmp)<<4;
//		_uitmp++;
//		_hash += (*_uitmp)<<8;
		int _coords1 = (int)(pos.f[0]*1000.0f+1.0f);
		int _coords2 = (int)(pos.f[1]*1333.0f);
		int _coords3 = (int)(pos.f[2]*2133.0f+3.0f);
		int _hash=0;
		//GUINT32 *_uitmp = (GUINT32 *)(&_coords[0]);
		_hash = _coords1;//*_uitmp;
		//_uitmp++;
		_hash += _coords2 << 4;//(*_uitmp)<<4;
		//_uitmp++;
		_hash += _coords3 << 8;//(*_uitmp)<<8;
		return _hash;//hash = _hash;
	}

	///Creates a contact list for queries
	//#define GIM_CREATE_CONTACT_LIST(contact_array) GIM_DYNARRAY_CREATE(GIM_CONTACT,contact_array,100)
	public static GimDynArray<GimContact> GIM_CREATE_CONTACT_LIST() { 
		return GimDynArray.GIM_DYNARRAY_CREATE(GimContact.class, 100);//GIM_CONTACT,contact_array,100);
	}

	//#define GIM_PUSH_CONTACT(contact_array, point, normal, deep,handle1, handle2, feat1, feat2)\
	static void GIM_PUSH_CONTACT(GimDynArray<GimContact> contact_array, vec3f point, 
			vec3f normal, final float deep, GimTrimesh handle1, GimTrimesh handle2, 
			final int feat1, final int feat2)
	{
//	    GIM_DYNARRAY_PUSH_EMPTY(GIM_CONTACT,contact_array);
//	    GimContact _last = GIM_DYNARRAY_POINTER_LAST(GIM_CONTACT,contact_array);
		GimContact _last = new GimContact();
	    VEC_COPY(_last.m_point,point);
	    VEC_COPY(_last.m_normal,normal);
	    _last.m_depth = deep;
	    _last.m_handle1 = handle1;
	    _last.m_handle2 = handle2;
	    _last.m_feature1 = feat1;
	    _last.m_feature2 = feat2;
		contact_array.GIM_DYNARRAY_PUSH_ITEM_TZ(_last);
	}

	static void GIM_PUSH_CONTACT(GimDynArray<GimContact> contact_array, vec3f point, 
			vec4f normal, final float deep, GimTrimesh handle1, GimTrimesh handle2, 
			final int feat1, final int feat2)
	{
//	    GIM_DYNARRAY_PUSH_EMPTY(GIM_CONTACT,contact_array);
//	    GimContact _last = GIM_DYNARRAY_POINTER_LAST(GIM_CONTACT,contact_array);
		GimContact _last = new GimContact();
	    VEC_COPY(_last.m_point,point);
	    VEC_COPY(_last.m_normal,normal);
	    _last.m_depth = deep;
	    _last.m_handle1 = handle1;
	    _last.m_handle2 = handle2;
	    _last.m_feature1 = feat1;
	    _last.m_feature2 = feat2;
		contact_array.GIM_DYNARRAY_PUSH_ITEM_TZ(_last);
	}

	///Receive pointer to contacts
	//#define GIM_COPY_CONTACTS(dest_contact, source_contact)\
	static void GIM_COPY_CONTACTS(GimContact dest_contact, GimContact source_contact)
	{
	    VEC_COPY(dest_contact.m_point,source_contact.m_point);
	    VEC_COPY(dest_contact.m_normal,source_contact.m_normal);
	    dest_contact.m_depth = source_contact.m_depth;
	    dest_contact.m_handle1 = source_contact.m_handle1;
	    dest_contact.m_handle2 = source_contact.m_handle2;
	    dest_contact.m_feature1 = source_contact.m_feature1;
	    dest_contact.m_feature2 = source_contact.m_feature2;
	}


	//! @}
	//#endif // GIM_CONTACT_H_INCLUDED

	


	//! Merges duplicate contacts with minimum depth criterion
//	void gim_merge_contacts(GDYNAMIC_ARRAY * source_contacts,
//						GDYNAMIC_ARRAY * dest_contacts)
	static void gim_merge_contacts(final GimDynArray<GimContact> source_contacts,
			final GimDynArray<GimContact> dest_contacts)
	{
	    dest_contacts.m_size = 0;

		int source_count = source_contacts.size();
		ObjArray<GimContact> psource_contacts = source_contacts.GIM_DYNARRAY_POINTER_V();
		//create keys
		//GIM_RSORT_TOKEN * keycontacts = (GIM_RSORT_TOKEN * )gim_alloc(sizeof(GIM_RSORT_TOKEN)*source_count);
		GIM_RSORT_TOKEN[] keycontacts = new GIM_RSORT_TOKEN[source_count];
		for (int i = 0; i < source_count; i++) keycontacts[i] = new GIM_RSORT_TOKEN();

	    int i;
		for(i=0;i<source_count;i++)
		{
			keycontacts[i].m_value = i;
			keycontacts[i].m_key = GIM_CALC_KEY_CONTACT(psource_contacts.at(i).m_point);
		}

		//sort keys
		//GIM_QUICK_SORT_ARRAY(GIM_RSORT_TOKEN , keycontacts, source_count, RSORT_TOKEN_COMPARATOR,GIM_DEF_EXCHANGE_MACRO);
		GimRadixSort.GIM_QUICK_SORT_ARRAY(keycontacts, source_count, 
				GimRadixSort.RSORT_TOKEN_COMPARATOR, GimRadixSort.GIM_DEF_EXCHANGE_MACRO);
		//Arrays.sort(keycontacts, COMPARATOT_TZ); //TZ //TODO remove

		// Merge contacts
		GimContact pcontact = null;
		GimContact scontact = null;
		long key,last_key=0;

		for(i=0;i<source_contacts.size();i++)
		{
		    key = keycontacts[i].m_key;
			scontact = psource_contacts.at( keycontacts[i].m_value );

			if(i>0 && last_key ==  key)
			{
				//merge contact
				if(pcontact.m_depth > scontact.m_depth + CONTACT_DIFF_EPSILON)
				{
				    GIM_COPY_CONTACTS(pcontact, scontact);
				}
			}
			else
			{//add new contact
				//dest_contacts.GIM_DYNARRAY_PUSH_EMPTY();
//				dest_contacts.GIM_DYNARRAY_PUSH_ITEM_TZ(new GimContact());
//	            pcontact = dest_contacts.GIM_DYNARRAY_POINTER_LAST();
	            pcontact = new GimContact();
	            GIM_COPY_CONTACTS(pcontact, scontact);
	            dest_contacts.GIM_DYNARRAY_PUSH_ITEM_TZ(pcontact);
			}
			last_key = key;
		}
		//TZ GimBufferArray.gim_free(keycontacts,0);
	}
	
	
//    private static final ComparatorTZ COMPARATOT_TZ = new ComparatorTZ();
//    private static final class ComparatorTZ implements Comparator<GIM_RSORT_TOKEN> {
////		private static int RSORT_TOKEN_COMPARATOR(GIM_RSORT_TOKEN x, GIM_RSORT_TOKEN y) { return x.m_key - y.m_key; }
////		interface GimRSortTokenComparator {
////			int run(GIM_RSORT_TOKEN x, GIM_RSORT_TOKEN y);
////		}
////		static final GimRSortTokenComparator RSORT_TOKEN_COMPARATOR = new GimRSortTokenComparator() {
////			@Override public int run(GIM_RSORT_TOKEN x, GIM_RSORT_TOKEN y) {
////				return RSORT_TOKEN_COMPARATOR(x, y);
////			}
////		};
//
//		@Override
//		public int compare(GIM_RSORT_TOKEN o1, GIM_RSORT_TOKEN o2) {
//			return (int) -(o1.m_key - o2.m_key);
//		}
//	};
	

	//! Merges to an unique contact
//	void gim_merge_contacts_unique(GDYNAMIC_ARRAY * source_contacts,
//						GDYNAMIC_ARRAY * dest_contacts)
	void gim_merge_contacts_unique(GimDynArray<GimContact> source_contacts,
			GimDynArray<GimContact> dest_contacts)
	{
	    dest_contacts.m_size = 0;
	    //Traverse the source contacts
		int source_count = source_contacts.size();
		if(source_count==0) return;

		ObjArray<GimContact> psource_contacts	= source_contacts.GIM_DYNARRAY_POINTER_V();

		//add the unique contact
		GimContact pcontact = null;
		dest_contacts.GIM_DYNARRAY_PUSH_EMPTY();
	    pcontact = dest_contacts.GIM_DYNARRAY_POINTER_LAST();
	    //set the first contact
	    GIM_COPY_CONTACTS(pcontact, psource_contacts.at0());

	    if(source_count==1) return;
	    //scale the first contact
	    VEC_SCALE(pcontact.m_normal,pcontact.m_depth,pcontact.m_normal);

	    psource_contacts.inc();//++;

		//Average the contacts
	    int i;
		for(i=1;i<source_count;i++)
		{
		    VEC_SUM(pcontact.m_point,pcontact.m_point,psource_contacts.at0().m_point);
		    VEC_ACCUM(pcontact.m_normal,psource_contacts.at0().m_depth,psource_contacts.at0().m_normal);
		    psource_contacts.inc();//++;
		}

		float divide_average = 1.0f/(source_count);

		VEC_SCALE(pcontact.m_point,divide_average,pcontact.m_point);

		pcontact.m_depth = VEC_DOT(pcontact.m_normal,pcontact.m_normal)*divide_average;
		pcontact.m_depth = GIM_SQRT(pcontact.m_depth);

		VEC_NORMALIZE(pcontact.m_normal);

		/*GREAL normal_len;
	    VEC_INV_LENGTH(pcontact->m_normal,normal_len);
		VEC_SCALE(pcontact->m_normal,normal_len,pcontact->m_normal);

	    //Deep = LEN(normal)/SQRT(source_count)
	    GIM_SQRT(divide_average,divide_average);
		pcontact->m_depth = divide_average/normal_len;
		*/
	}
}
