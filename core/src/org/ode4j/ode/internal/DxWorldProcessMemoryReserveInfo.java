/*
 * Created on Apr 14, 2012
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Generation - Code and Comments
 */
package org.ode4j.ode.internal;

public class DxWorldProcessMemoryReserveInfo {
 //TODO   public dBase
    DxWorldProcessMemoryReserveInfo(double fReserveFactor, int uiReserveMinimum)
    {
        Assign(fReserveFactor, uiReserveMinimum);
    }

    void Assign(double fReserveFactor, int uiReserveMinimum)
    {
        m_fReserveFactor = fReserveFactor;
        m_uiReserveMinimum = uiReserveMinimum;
    }

    double m_fReserveFactor; // Use float as precision does not matter here
    int m_uiReserveMinimum;

    void DESTRUCTOR() {
        // nothing
    }
      
}
