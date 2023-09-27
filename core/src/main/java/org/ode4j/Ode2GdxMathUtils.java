package org.ode4j;

import com.badlogic.gdx.math.Quaternion;

import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternionC;

public class Ode2GdxMathUtils {

    private static float m00;
    private static float m01;
    private static float m02;
    private static float m10;
    private static float m11;
    private static float m12;
    private static float m20;
    private static float m21;
    private static float m22;

    private static float qw, qx, qy, qz;
    private static float tr;
    private static float S;

    private static Quaternion q = new Quaternion();

    //          0, 1, 2, 3
    // ODE      w, x, y ,z
    // libGDX   x, y, z, w
    public static Quaternion getGdxQuaternion(DQuaternionC odeQ){
        q.set((float)odeQ.get1(), (float)odeQ.get2(), (float)odeQ.get3(), (float) odeQ.get0());
        return q;
    }

    // From https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    public static Quaternion getGdxQuaternion(DMatrix3C odeMat3) {

        m00 = (float) odeMat3.get00();
        m01 = (float) odeMat3.get01();
        m02 = (float) odeMat3.get02();
        m10 = (float) odeMat3.get10();
        m11 = (float) odeMat3.get11();
        m12 = (float) odeMat3.get12();
        m20 = (float) odeMat3.get20();
        m21 = (float) odeMat3.get21();
        m22 = (float) odeMat3.get22();

        tr = m00 + m11 + m22;

        if (tr > 0) {
            S = (float)Math.sqrt(tr + 1.0) * 2; // S=4*qw
            qw = 0.25f * S;
            qx = (m21 - m12) / S;
            qy = (m02 - m20) / S;
            qz = (m10 - m01) / S;
        } else if ((m00 > m11) & (m00 > m22)) {
            S = (float)Math.sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx
            qw = (m21 - m12) / S;
            qx = 0.25f * S;
            qy = (m01 + m10) / S;
            qz = (m02 + m20) / S;
        } else if (m11 > m22) {
            S = (float)Math.sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
            qw = (m02 - m20) / S;
            qx = (m01 + m10) / S;
            qy = 0.25f * S;
            qz = (m12 + m21) / S;
        } else {
            S = (float)Math.sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
            qw = (m10 - m01) / S;
            qx = (m02 + m20) / S;
            qy = (m12 + m21) / S;
            qz = 0.25f * S;
        }
        q.set(qx, qy, qz, qw);
        return q;
    }
}
