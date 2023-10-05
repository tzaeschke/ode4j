package org.ode4j.demo;

import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCapsule;
import static org.ode4j.drawstuff.DrawStuff.dsDrawConvex;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCylinder;
import static org.ode4j.drawstuff.DrawStuff.dsDrawSphere;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetColorAlpha;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.DMisc.dRandReal;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeConstants.dContactBounce;
import static org.ode4j.ode.OdeConstants.dContactRolling;
import static org.ode4j.ode.OdeHelper.areConnectedExcluding;

import java.util.ArrayList;
import java.util.List;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DCapsule;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DContactJoint;
import org.ode4j.ode.DConvex;
import org.ode4j.ode.DCylinder;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;

public class DemoBuoyancy extends dsFunctions {

    private static final double STEP_SIZE = 0.02;
    private static final int NUM = 200;         // max number of objects
    private static final double DENSITY = (0.2);        // density of all objects
    private static final int GPB = 3;           // maximum number of geometries per body
    private static final int MAX_CONTACTS = 40;     // maximum number of contact points per body

    // dynamics and collision objects

    private static class MyObject {
        DBody body;         // the body
        DGeom[] geom = new DGeom[GPB];      // geometries representing this body
    };

    private static int num=0;       // number of objects in simulation
    private static int nextobj=0;       // next object to recycle if num==NUM
    private static DWorld world;
    private static DSpace space;
    private static MyObject[] obj = new MyObject[NUM];
    private static DJointGroup contactgroup;
    private static boolean random_pos = true;   // drop objects from random position?

    private DNearCallback nearCallback = new DNearCallback() {
        @Override
        public void call(Object data, DGeom o1, DGeom o2) {
            nearCallback( data, o1, o2);
        }
    };

    private void nearCallback (Object data, DGeom o1, DGeom o2) {
        DBody b1 = o1.getBody();
        DBody b2 = o2.getBody();
        if (b1!=null && b2!=null && areConnectedExcluding (b1,b2,DContactJoint.class)) return;

        DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);   // up to MAX_CONTACTS contacts per box-box
        for (int i=0; i<MAX_CONTACTS; i++) {
            DContact contact = contacts.get(i);
            contact.surface.mode = dContactBounce | dContactRolling;
            contact.surface.mu = 250;
            contact.surface.rho = 0.2;
            contact.surface.bounce = 0.2;
        }
        int numc = OdeHelper.collide (o1,o2,MAX_CONTACTS,contacts.getGeomBuffer() );
        if (numc != 0) {
            for (int i=0; i<numc; i++) {
                DContact contact = contacts.get(i);
                DJoint c = OdeHelper.createContactJoint (world,contactgroup,contact );
                c.attach (b1,b2);
            }
        }
    }


    private static float[] xyz = {-3.5f, 0f, 6f};
    private static float[] hpr = {0f, -20.0000f,0.0000f};

    // start simulation - set viewpoint

    @Override
    public void start() {
        dsSetViewpoint (xyz,hpr);
        System.out.println ("To drop another object, press:");
        System.out.println ("   b for box.");
        System.out.println ("   s for sphere.");
        System.out.println ("   c for capsule.");
        System.out.println ("   y for cylinder.");
        System.out.println ("   x for a composite object.");
        System.out.println ("To toggle dropping from random position/orientation, press r.");
    }

    // called when a key pressed

    @Override
    public void command (char cmd) {
        int i,j,k;
        double[] sides= new double[3];
        DMass m = OdeHelper.createMass();
        boolean setBody = false;

        cmd = Character.toLowerCase (cmd);
        if (cmd == 'b' || cmd == 's' || cmd == 'c' || cmd == 'x' || cmd == 'y') {
            if (num < NUM) {
                i = num;
                num++;
            }
            else {
                i = nextobj;
                nextobj++;
                if (nextobj >= num) nextobj = 0;

                // destroy the body and geoms for slot i
                obj[i].body.destroy ();
                for (k=0; k < GPB; k++) {
                    if (obj[i].geom[k]!=null) obj[i].geom[k].destroy ();
                }
                obj[i] = new MyObject();
            }

            obj[i].body = OdeHelper.createBody (world);
            for (k=0; k<3; k++) sides[k] = dRandReal()*0.4+0.3;

            DMatrix3 R = new DMatrix3();
            if (random_pos) {
                obj[i].body.setPosition (
                        dRandReal()*4-2,dRandReal()*4-2,dRandReal()+6);
                dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
                        dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
            }
            else {
                double maxheight = 0;
                for (k=0; k<num; k++) {
                    DVector3C pos = obj[k].body.getPosition ();
                    if (pos.get2() > maxheight) maxheight = pos.get2();
                }
                obj[i].body.setPosition (0,0,maxheight+1);
                dRFromAxisAndAngle (R,0,0,1,dRandReal()*10.0-5.0);
            }
            obj[i].body.setRotation (R);
            obj[i].body.setData (i);

            if (cmd == 'b') {
                m.setBox (DENSITY,sides[0],sides[1],sides[2]);
                obj[i].geom[0] = OdeHelper.createBox (space,sides[0],sides[1],sides[2]);
            }
            else if (cmd == 'c') {
                sides[0] *= 0.5;
                m.setCapsule (DENSITY,3,sides[0],sides[1]);
                obj[i].geom[0] = OdeHelper.createCapsule (space,sides[0],sides[1]);
            }
            else if (cmd == 'y') {
                sides[1] *= 0.5;
                m.setCylinder(DENSITY,3,sides[0],sides[1]);
                obj[i].geom[0] = OdeHelper.createCylinder (space,sides[0],sides[1]);
            }
            else if (cmd == 's') {
                sides[0] *= 0.5;
                m.setSphere (DENSITY,sides[0]);
                obj[i].geom[0] = OdeHelper.createSphere (space,sides[0]);
            }
            else if (cmd == 'x') {
                setBody = true;

                // start accumulating masses for the encapsulated geometries
                DMass m2 = OdeHelper.createMass();
                m.setZero ();

                DVector3[] dpos = DVector3.newArray(GPB);   // delta-positions for encapsulated geometries
                DMatrix3[] drot = DMatrix3.newArray(GPB); 

                // set random delta positions
                for (j=0; j<GPB; j++) {
                    for (k=0; k<3; k++) {
                        dpos[j].set(k, dRandReal()*0.3-0.15 );
                    }
                }

                for (k=0; k<GPB; k++) {
                    if (k==0) {
                        double radius = dRandReal()*0.25+0.05;
                        obj[i].geom[k] = OdeHelper.createSphere (space,radius);
                        m2.setSphere (DENSITY,radius);
                    }
                    else if (k==1) {
                        obj[i].geom[k] = OdeHelper.createBox (space,sides[0],sides[1],sides[2]);
                        m2.setBox (DENSITY,sides[0],sides[1],sides[2]);
                    }
                    else {
                        double radius = dRandReal()*0.1+0.05;
                        double length = dRandReal()*1.0+0.1;
                        obj[i].geom[k] = OdeHelper.createCapsule (space,radius,length);
                        m2.setCapsule (DENSITY,3,radius,length);
                    }

                    dRFromAxisAndAngle (drot[k],dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
                            dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);

                    m2.rotate (drot[k]);

                    m2.translate (dpos[k]);

                    // add to the total mass
                    m.add (m2);
                }

                // move all encapsulated objects so that the center of mass is (0,0,0)
                DVector3 negC = new DVector3(m.getC()).scale(-1);
                for (k=0; k<GPB; k++) {
                    obj[i].geom[k].setBody(obj[i].body);
                    obj[i].geom[k].setOffsetPosition(dpos[k].reAdd(negC));
                    obj[i].geom[k].setOffsetRotation(drot[k]);
                }
                m.translate(negC);
                obj[i].body.setMass(m);
            }

            if (!setBody) {
                for (k=0; k < GPB; k++) {
                    if (obj[i].geom[k]!=null) {
                        obj[i].geom[k].setBody (obj[i].body);
                    }
                }
    
                obj[i].body.setMass (m);
            }
        }

    }

    private void drawGeom (DGeom g, DVector3C pos, DMatrix3C R)
    {
        if (g==null) return;
        if (pos==null) pos = g.getPosition ();
        if (R==null) R = g.getRotation ();

        if (g instanceof DBox) {
            DVector3C sides = ((DBox)g).getLengths();
            dsDrawBox (pos,R,sides);
        } else if (g instanceof DSphere) {
            dsDrawSphere( pos,R, ((DSphere)g).getRadius() );
        } else if (g instanceof DCapsule) {
            DCapsule c = (DCapsule) g;
            dsDrawCapsule( pos, R, c.getLength(), c.getRadius() );
        } else if (g instanceof DCylinder) {
            DCylinder c = (DCylinder) g;
            dsDrawCylinder (pos, R, c.getLength(), c.getRadius());
        } else if (g instanceof DConvex) {
            //dVector3 sides={0.50,0.50,0.50};
            dsDrawConvex(pos,R,ConvexCubeGeom.planes,
                    ConvexCubeGeom.planecount,
                    ConvexCubeGeom.points,
                    ConvexCubeGeom.pointcount,
                    ConvexCubeGeom.polygons);
        }
    }


    // simulation loop

    @Override
    public void step (boolean pause)
    {
        dsSetColor (0,0,2);
        space.collide (0,nearCallback);
        if (!pause) world.quickStep(STEP_SIZE);
        //if (!pause) dWorldStepFast (world,0.05, 1);

        // remove all contact joints
        contactgroup.empty ();
        handleBuoyancy();

        dsSetColor (1,1,0);
        dsSetTexture (DS_TEXTURE_NUMBER.DS_SKY);
        for (int i=0; i<num; i++) {
            for (int j=0; j < GPB; j++) {
                if (! obj[i].body.isEnabled ()) {
                    dsSetColor (1,0,0);
                } else {
                    dsSetColor (1,1,0);
                }
                drawGeom (obj[i].geom[j], null, null);
            }
        }
        dsSetColorAlpha (0.6f,0.8,0.9f,0.5f);
        dsDrawBox(new DVector3(0, 0, 0), new DMatrix3().setIdentity(), new DVector3(10, 10, 10));
    }

    public static void main(String[] args) {
        new DemoBuoyancy().demo(args);
    }

    private void demo (String[] args) {
        OdeHelper.initODE2(0);
        world = OdeHelper.createWorld();

        space = OdeHelper.createSimpleSpace();
        contactgroup = OdeHelper.createJointGroup();
        world.setGravity (0,0,-9.81);
        world.setContactMaxCorrectingVel(2.5);
        world.setMaxAngularSpeed(1);
        world.setAngularDamping(0.6 * STEP_SIZE);
        world.setAngularDampingThreshold(0);
        world.setLinearDampingThreshold(0);
        world.setLinearDamping(0.6 * STEP_SIZE);

        for (int i = 0; i < obj.length; i++) obj[i] = new MyObject();
        OdeHelper.createPlane( space, 0, 0, 1, 0 );

        dsSimulationLoop (args,640,480,this);
        contactgroup.destroy ();
        space.destroy ();
        world.destroy ();
        OdeHelper.closeODE();
    }


    @Override
    public void stop() {
    }

    private final static double WATER_LEVEL = 5; 
    private final static double WATER_DENSITY = 0.27; 

    public void handleBuoyancy() {
        List<DGeom> floatingGeoms = new ArrayList<>();
        for (int i=0; i<num; i++) {
            for (int k=0; k < GPB; k++) {
                if (obj[i].geom[k]!=null) {
                    floatingGeoms.add(obj[i].geom[k]);
                }
            }
        }
        for (DGeom geom : floatingGeoms) {
            if (geom instanceof DBox) {
                Buoy[] buoys = generateBuoys(3, (DBox) geom);
                processBuoys(geom, buoys, getVolume((DBox) geom), getArea((DBox) geom));
            } else if (geom instanceof DSphere) {
                processBuoys(geom, generateBuoys((DSphere) geom), getVolume((DSphere) geom), getArea((DSphere) geom));
            } else if (geom instanceof DCapsule) {
                processBuoys(geom, generateBuoys(2, (DCapsule) geom), getVolume((DCapsule) geom), getArea((DCapsule) geom));
            } else if (geom instanceof DCylinder) {
                processBuoys(geom, generateBuoys(5, (DCylinder) geom), getVolume((DCylinder) geom), getArea((DCylinder) geom));
            }
        }
        floatingGeoms.clear();
    }
    
    private void processBuoys(DGeom geom, Buoy[] buoys, double totalVolume, double totalArea) {
        double density = WATER_DENSITY;
        DBody body = geom.getBody();
        double buoyVolume = 0;
        double buoyArea = 0;
        for (Buoy b : buoys) {
            double radius = b.radius;
            double radiusSqr = radius * radius;
            buoyVolume += b.weight * 4 * Math.PI / 3 * radius * radiusSqr;
            buoyArea += b.weight * Math.PI * radiusSqr;
        }
        double volumeRatio = totalVolume / buoyVolume;
        double areaRatio = 0.5 * totalArea / buoyArea;
        DVector3 position = new DVector3();
        for (Buoy b : buoys) {
            geom.getRelPointPos(b.x, b.y, b.z, position);
            double waterLevel = getWaterLevel(position);
            double radius = b.radius;
            double y = position.get2();
            if (y - radius < waterLevel) {
                double h = Math.max(0, Math.min(waterLevel + radius - y, 2 * radius));
                double base = Math.sqrt(h * (2 * radius - h));
                double volume = Math.PI * h / 6.0 * (3 * base * base + h * h);

                DVector3 buoyancyForce = new DVector3();
                world.getGravity(buoyancyForce);
                buoyancyForce.scale(-density * volumeRatio * volume * b.weight);

                double area = Math.PI * radius * h * areaRatio * b.weight;
                DVector3 dragForce = getDragForce(density, body, area);
                DVector3 dragTorque = getDragTorque(density, body, area);
                body.addForceAtPos(dragForce.get0() + buoyancyForce.get0(), buoyancyForce.get1() + dragForce.get1(),
                        buoyancyForce.get2() + dragForce.get2(), position.get0(), position.get1(), position.get2());
                body.addTorque(dragTorque);
            }
        }
    }

    private double getWaterLevel(DVector3 position) {
        return WATER_LEVEL; // Waves can be simulated here
    }

    private Buoy[] generateBuoys(DSphere geom) {
        double radius = geom.getRadius();
        return new Buoy[] { new Buoy(0, 0, 0, (float) radius, 1) };
    }

    private Buoy[] generateBuoys(int bn, DBox geom) {
        DVector3C lengths = geom.getLengths();
        double lx = lengths.get0();
        double ly = lengths.get1();
        double lz = lengths.get2();
        double d = 0.30;
        double radius = Math.min(Math.min(lx, ly), lz) * 0.25;
        double weight = 1;
        double radius2 = Math.min(Math.min(lx, ly), lz) * 0.25;
        double weight2 = 0.1;
        Buoy[] buoys = new Buoy[14];
        double d2 = 0.25;
        buoys[0] = new Buoy(-d * lx, 0, 0, radius, weight);
        buoys[1] = new Buoy(d * lx, 0, 0, radius, weight);
        buoys[2] = new Buoy(0, -d * ly, 0, radius, weight);
        buoys[3] = new Buoy(0, d * ly, 0, radius, weight);
        buoys[4] = new Buoy(0, 0, -d * lz, radius, weight);
        buoys[5] = new Buoy(0, 0, d * lz, radius, weight);

        buoys[6] = new Buoy(-lx * d2, -ly * d2, -lz * d2, radius2, weight2);
        buoys[7] = new Buoy(-lx * d2, -ly * d2, lz * d2, radius2, weight2);
        buoys[8] = new Buoy(-lx * d2, ly * d2, -lz * d2, radius2, weight2);
        buoys[9] = new Buoy(-lx * d2, ly * d2, lz * d2, radius2, weight2);
        buoys[10] = new Buoy(lx * d2, -ly * d2, -lz * d2, radius2, weight2);
        buoys[11] = new Buoy(lx * d2, -ly * d2, lz * d2, radius2, weight2);
        buoys[12] = new Buoy(lx * d2, ly * d2, -lz * d2, radius2, weight2);
        buoys[13] = new Buoy(lx * d2, ly * d2, lz * d2, radius2, weight2);

        return buoys;
    }

    private Buoy[] generateBuoys(int bn, DCapsule geom) {
        double length = geom.getLength();
        double radius = geom.getRadius();
        Buoy[] buoys = new Buoy[bn];
        int i = 0;
        for (int bz = 0; bz < bn; bz++) {
            float pz = (float) (((float) bz / (bn - 1) - 0.5) * length);
            buoys[i++] = new Buoy(0, 0, pz, (float) radius, 1);
        }
        return buoys;
    }

    private Buoy[] generateBuoys(int bn, DCylinder geom) {
        double length = geom.getLength();
        double radius = geom.getRadius();
        double bRadius = Math.min(0.25 * length, 0.5 * radius);
        int segments = 2;
        Buoy[] buoys = new Buoy[bn * segments];
        int i = 0;
        for (int bz = 0; bz < segments; bz++) {
            float pz = (float) (((float) bz / (segments - 1) - 0.5) * (length - bRadius));
            for (int bc = 0; bc < bn; bc++) {
                double a = 2 * Math.PI * bc / bn;
                float px = (float) ((radius - bRadius) * Math.cos(a));
                float py = (float) ((radius - bRadius) * Math.sin(a));
                buoys[i++] = new Buoy(px, py, pz, (float) bRadius, 1);
            }
        }
        return buoys;
    }
    
    private double getVolume(DSphere geom) {
        double radius = geom.getRadius();
        return 4 * Math.PI / 3 * radius * radius * radius;
    }

    private double getVolume(DBox geom) {
        DVector3C lengths = geom.getLengths();
        return lengths.get0() * lengths.get1() * lengths.get2();
    }

    private double getVolume(DCapsule geom) {
        double radius = geom.getRadius();
        return Math.PI * radius * radius * geom.getLength() + 4 * Math.PI / 3 * radius * radius * radius;
    }

    private double getVolume(DCylinder geom) {
        double radius = geom.getRadius();
        return Math.PI * radius * radius * geom.getLength();
    }

    private double getArea(DSphere geom) {
        double radius = geom.getRadius();
        return 4 * Math.PI * radius * radius;
    }

    private double getArea(DBox geom) {
        DVector3C lengths = geom.getLengths();
        return lengths.get0() * lengths.get1() + lengths.get0() * lengths.get2() + lengths.get2() * lengths.get1();
    }

    private double getArea(DCapsule geom) {
        double radius = geom.getRadius();
        return 4 * Math.PI * radius * radius + 2 * Math.PI * radius * geom.getLength();
    }

    private double getArea(DCylinder geom) {
        double radius = geom.getRadius();
        return 2 * Math.PI * radius * radius + 2 * Math.PI * radius * geom.getLength();
    }

    private DVector3 getDragForce(double density, DBody odeBody, double area) {
        DVector3 dragForce = new DVector3(odeBody.getLinearVel());
        double lvel = dragForce.length();
        dragForce.safeNormalize();
        dragForce.scale(-0.5 * density * area * lvel * lvel * 0.5);
        return dragForce;
    }

    private DVector3 getDragTorque(double density, DBody odeBody, double area) {
        DVector3 dragTorque = new DVector3(odeBody.getAngularVel());
        double avel = dragTorque.length();
        dragTorque.safeNormalize();
        dragTorque.scale(-0.5 * density * area * avel * avel * 0.5);
        return dragTorque;
    }
    
    private class Buoy {

        public double x;
        public double y;
        public double z;
        public double radius;
        public double weight;

        public Buoy(double x, double y, double z, double radius, double weight) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.radius = radius;
            this.weight = weight;
        }
    }

}
