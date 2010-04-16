/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2010 Tilmann Zäschke      *
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
 *       the file ODE4J-LICENSE-BSD.TXT.                                 *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and ODE4J-LICENSE-BSD.TXT for more details.               *
 *                                                                       *
 *************************************************************************/
package org.ode4j.tests;


import java.io.File;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.net.URL;
import java.security.CodeSource;
import java.security.ProtectionDomain;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.logging.Logger;
import java.util.regex.Pattern;

import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.internal.DxGeom;
import org.ode4j.ode.internal.DxHashSpace;
import org.ode4j.ode.internal.Misc;
import org.ode4j.ode.internal.OdeInit;
import org.ode4j.ode.internal.Timer;

import junit.framework.Test;
import junit.framework.TestCase;
import junit.framework.TestSuite;

/**
 * Test harness to check for multi-thread tolerance in IA numeric and it's
 * sub-packages (SPR 3998).
 * <p>
 * If this test harness does not throw any errors, then it is at least possible 
 * to write multi-threaded code using IA numeric. 
 * However, it does not indicate thread-safety of any tested classes.
 * The command-line version of this test prints out a list of 
 * state-full (=mutable) classes. Instances of classes from this list should
 * not be shared between Threads. All other classes (not printed) are
 * state-less (=immutable) and therefore thread-safe.
 * <p>
 * As a rule of thumb, any object that can be created with 'new' (like Int3d 
 * and MatrixMultiply) should be used in one Thread only. All others (like
 * ArcSin.PROCEDURE) can be used in parallel in multiple Threads.
 * To clarify this, for example Int3d can be process in parallel, as long as the
 * two Threads work on different Int3d arrays.
 *
 * @author Tilmann Zaeschke
 */
public class JavaMultiThreadTest extends TestCase {

    //The following two designate the package to test. 
    private static final String PACKAGE_TO_TEST = "org.ode4j.ode";
    //TODO also test "org.ode4j.ode" !!
    private static final Class<?> REFERENCE_CLASS = OdeHelper.class;
    
    private static boolean _verbose = false;
    
    private int _nFieldStaticFinalPrimitive = 0;
    private int _nClassHasState = 0;
    private Set<Class<?>> _stateLessPrims = new HashSet<Class<?>>();
    private Set<Field> _ignoreFields = new HashSet<Field>();
    private Set<String> _ignoreFieldNames = new HashSet<String>();
    private Set<Class<?>> _ignoreClasses = new HashSet<Class<?>>();

    private LinkedHashSet<Class<?>> _circularStack = 
        new LinkedHashSet<Class<?>>();

    private Set<String> _classNames = new HashSet<String>();
    private Map<Class<?>, ClassInfo> _classes = 
        new HashMap<Class<?>, ClassInfo>();

    private int _nWarnings = 0;
    private int _nErrors = 0;
//    private static final String NUM = "herschel.ia.numeric";

    /**
     * @param name
     */
    public JavaMultiThreadTest(String name) {
        super(name);
        //State-less types are types that are immutable, and that can therefore
        //be referenced from static non-final fields without problems.
        _stateLessPrims.add(Byte.class);
        _stateLessPrims.add(Boolean.class);
        _stateLessPrims.add(Double.class);
        _stateLessPrims.add(Float.class);
        _stateLessPrims.add(Integer.class);
        _stateLessPrims.add(Long.class);
        _stateLessPrims.add(Short.class);
        _stateLessPrims.add(String.class);
        _stateLessPrims.add(Class.class);
        _stateLessPrims.add(Pattern.class);
        _stateLessPrims.add(Logger.class);
        _stateLessPrims.add(Throwable.class);

        //Special cases that should be ignored. This can happen if a field
        //is effectively not modified, but can not be made final due to Java's 
        //syntax. Unfortunately, this test harness supports only static 
        //analysis based on Java reflection. Examples for special cases:
        //- Multidimensional arrays, which are actually arrays of arrays. Only
        //  the reference to the outermost array can be made 'final'.
        //- Collections, e.g. 
        //  "static final Map xyz = Collections.unmodifyableMap(aMap)";
        //  The map 'xyz' is obviously immutable, but that this tool can not 
        //  realize this only via reflection.
        try {
            //This is an unmodifiableMap
//TODO            _ignoreFields.add(DataFormatter.class.getDeclaredField("_systemFormats"));
            //This array is never modified
//TODO            _ignoreFields.add(DataFormatter.class.getDeclaredField("_systemLayout"));
            //This array is never modified
//TODO            _ignoreFields.add(AbstractArrayData.class.getDeclaredField("RESIZER"));
            //This array is never modified
//TODO            _ignoreFields.add(
//                    AbstractGaussianQuadrature.class.getDeclaredField("_cof"));
            _ignoreFields.add(Misc.class.getDeclaredField("seed"));
            _ignoreFields.add(DxHashSpace.class.getDeclaredField("prime"));
            _ignoreFields.add(OdeInit.class.getDeclaredField("g_bODEInitialized"));
//            //This array is empty and never modified
//            _ignoreFieldNames.add("_empty_array");
            _ignoreFields.add(DxGeom.class.getDeclaredField("colliders_initialized")); //TODO fix this properly
            _ignoreFields.add(DxGeom.class.getDeclaredField("colliders")); //TODO fix this properly
//            //This array is empty and never modified
//            _ignoreFieldNames.add("_empty_re");
//            //This array is empty and never modified
//            _ignoreFieldNames.add("_empty_im");
            //These arrays are never modified
            _ignoreClasses.add(Timer.class);
            _ignoreClasses.add(Throwable.class);
            _ignoreClasses.add(TestCase.class);
//TODO            _ignoreClasses.add(ConfiguredTestCase.class);
        } catch (SecurityException e) {
            throw new RuntimeException(e);
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
//        } catch (ClassNotFoundException e) {
//            throw new RuntimeException(e);
        }
    }

    private class ClassInfo {
        List<Field> _fields = new LinkedList<Field>();
        List<Class<?>> _subClasses = new LinkedList<Class<?>>();
        Class<?> _cls;
        boolean _hasBeenChecked = false;
        ClassInfo _superInfo = null;
        boolean _hasState = false;
        boolean _isStaticallyReferenced = false;

        public ClassInfo(Class<?> cls) {
            _cls = cls;
            if (_ignoreClasses.contains(cls)) {
                _hasBeenChecked = true;
                return;
            }
            findFields(cls);
        }

        private void findFields(Class<?> cls) {
            //Add local fields
            for (Field f: cls.getDeclaredFields()) {
                _fields.add(f);
            }

            //handle super classes
            Class<?> clsSuper = cls.getSuperclass();
            if (clsSuper != null) {
                if (!_classes.containsKey(clsSuper)) {
                    _classes.put(clsSuper, new ClassInfo(clsSuper));
                }

                _superInfo = _classes.get(clsSuper);
                _fields.addAll(_superInfo._fields);
                _superInfo._subClasses.add(_cls);
            }
        }

        public void getStatesFromSuper() {
            if (_superInfo == null) {
                return;
            }
            if (!_hasState) {
                _hasState = _superInfo._hasState;
            }
        }
    }

    /**
     * This test does some check to identify problems with IA numerics in multi-
     * threaded applications. The goal of SPR 3998 is to make IA numeric thread-
     * tolerant, opposed to thread-safe.
     * <p>
     * Thread-tolerance in IA numeric is defined as being able to let multiple
     * Threads performs numeric operations in parallel on separate data.
     * To be more precise, that means:
     * - Two Threads cannot share the same arrays (Int3d, ...), for anything 
     *   else than read operations. Parallel updates will not work
     * - For high level operations like fitting or matrix operations, each 
     *   Thread has to create it's own instance of the Fitter, Matrix, ...
     *   
     * Technically, this is achieved not sharing any mutable data between 
     * Threads. Local variable inside methods or as method arguments are 
     * allowed, but class fields can not be allowed to be mutable, because
     * they make the instance itself mutable.
     * To be more precise, objects are mutable if they:
     * - have 'static' fields that are not 'final'
     * - have 'static final' fields that refer to other mutable objects.
     * <p>  
     * This test harness performs only static analysis, so it can't see whether
     * a non-final field would be effectively be final. Therefore, 'exceptions'
     * can be defined above to avoid checking of certain classes. For example
     * arrays can not be made 'final' because their can always be changed,
     * however the _ignoreXYZ Sets above allow defining exceptions.
     * <p>
     * The test harness reports ERRORs for classes that have Fields that are
     * - 'static final' but the type itself is mutable
     * - 'static' but not 'final'
     * The test harness reports WARNINGs for classes that are mutable. These 
     * classes should not be shared by multiple Threads.
     */
    public void testMultiThreadTolerance() {
        // Get the location of this class
        ProtectionDomain pDomain = REFERENCE_CLASS.getProtectionDomain();
        CodeSource cSource = pDomain.getCodeSource();
        URL loc = cSource.getLocation();  // file:/c:/almanac14/examples/

        String c = File.separator;
        String pkgStr = PACKAGE_TO_TEST.replaceAll("\\.", c);
        println("Loading from: " + loc.getPath() + pkgStr);
        File fIaNum = new File(loc.getPath() + pkgStr);

        println("[Finding class names]");
        findClasses(fIaNum, _classNames);

        println("[Loading classes]");
        loadClasses(_classNames, _classes);

        println("[Checking fields]");
        for (Class<?> cls: _classes.keySet()) {
            checkClassFields(cls);
        }
        int nStateful = 0;
        println("The following classes have a state:");
        println("-----------------------------------");
        for (ClassInfo clsInfo: _classes.values()) {
            if (clsInfo._hasState && 
                    clsInfo._cls.getName().startsWith(PACKAGE_TO_TEST)) {
                println(clsInfo._cls.getName());
                nStateful++;
            }
        }

        println("[Summary]");
        println("Class names found: " + _classNames.size());
        println("Classes checked:   " + _classes.size());
        println("Stateful classes:  " + _nClassHasState);
        println("SFP:               " + _nFieldStaticFinalPrimitive);
        println("Stateful after filter: " + nStateful);
        println("");
        println("Errors:   " + _nErrors);
        println("Warnings: " + _nWarnings);
        println("");
        println("[Done]");
    }

    private void checkClassFields(Class<?> cls) {
        //Exclude classes that have already been checked
        ClassInfo info = _classes.get(cls);
        if (info._hasBeenChecked) {
            return;
        }

        //Check super classes first
        ClassInfo superInfo = info._superInfo;
        if (superInfo != null && !superInfo._hasBeenChecked) {
            checkClassFields(superInfo._cls);
        }

        LinkedList<Field> staticSelfReferences = new LinkedList<Field>();

        for (Field f: info._fields) {
            if (_ignoreFields.contains(f)) continue;
            if (_ignoreFieldNames.contains(f.getName())) continue;

            Class<?> type = f.getType();
            int mod = f.getModifiers();
            boolean isArray = false;
            while (type.isArray()) {
                isArray = true;
                type = type.getComponentType();
            }
            boolean isFinal = Modifier.isFinal(mod) && (!isArray);
            boolean isStatic = Modifier.isStatic(mod);

            if (isFinal) {  //static or non-static
                if (_stateLessPrims.contains(type)) {
                    _nFieldStaticFinalPrimitive++;
                    //Static final primitives are all right
                    continue;
                }
            } else if (isFinal && isStatic) {
                if (!_classes.containsKey(type)) {
                    println("WARNING: Unknown Type in " + cls + 
                            ":   " + type + "." + f);
                    _nWarnings++;
                } else {
                    ClassInfo typeInfo = _classes.get(type);
                    if (!typeInfo._hasBeenChecked) {
                        //Check self references later
                        if (type.equals(cls)) {
                            staticSelfReferences.add(f);
                            continue;
                        }

                        //Hopefully we don't get circular here
                        if (_circularStack.contains(type)) {
                            String msg = ("ERROR: Circular problem in " + cls + 
                                    ":   " + type + "." + f + "            " +
                                    _circularStack);
                            throw new RuntimeException(msg);
                        }
                        _circularStack.add(type);
                        checkClassFields(type);
                        _circularStack.remove(type);
                    }
                    if (typeInfo._hasState) {
                        println("ERROR: Stateful static final " +
                                "field in " + cls + ":   " + type + "." + f);
                        _nErrors++;
                    }
                }
            } else if (isStatic & !f.isSynthetic()) {
            	//ignore synthetic fields, e.g. statically imported ENUMs
                //static but not final
                _nErrors++;
                info._hasState = true;
                fail("static non-final field in " + cls + ":   " + type + "." + f);
            } else { //not final/non-static
                info._hasState = true;
            }
        }

        for (Field f: staticSelfReferences) {
            int mod = f.getModifiers();
            if (Modifier.isStatic(mod)) {
                if (info._hasState) {
                    println("ERROR: Stateful static final " +
                            "field in " + cls + ":   " + f);
                }
            }
        }

        info._hasBeenChecked = true;
        if (info._hasState) _nClassHasState++;
        info.getStatesFromSuper();
    }

    private void findClasses(File path, Set<String> classNames) {
        if (path.getName().equals("test")) {
            return;
        }
        for (File fSub: path.listFiles()) {
            if (fSub.isDirectory()) {
                //println("Entering sub directory: " + fSub.toString());
                findClasses(fSub, classNames);
            }
            if (fSub.getName().endsWith(".class") && 
                    !fSub.getName().endsWith("$py.class")) {
                String name = fSub.getPath().replaceAll("\\\\", ".");
                name = fSub.getPath().replaceAll("/", ".");
                int in = name.indexOf(PACKAGE_TO_TEST + ".");
                name = name.substring(in, name.length()-6);
                //println("File: " + name);
                classNames.add(name);
            }
        }
    }

    private void loadClasses(Set<String> classNames, 
            Map<Class<?>, ClassInfo> classes) {
        for (String name: classNames) {
            try {
                Class<?> cls = Class.forName(name);
                if (!classes.containsKey(cls)) {
                    classes.put(cls, new ClassInfo(cls));
                }
            } catch (ClassNotFoundException e) {
                throw new RuntimeException(e);
            }
        }

    }
    
    private static void println(String msg) {
        if (_verbose) {
            System.out.println(msg);
        }
    }
    
    /**
     * @return A new test suite.
     */
    public static Test suite() {
        return new TestSuite(JavaMultiThreadTest.class);
      }

    public static void fail(String msg) {
        println("ERROR: " + msg);
        //TestCase.fail(msg);
        //TODO
        _verbose = true;
    }
    
    /**
     * Executing this tool will list all warnings and errors, and will produce
     * a list of all mutable classes. Instances of mutable classes may not
     * be safe to be shared amongst multiple threads.
     * @param args
     */
    public static void main(String[] args) {
        _verbose = true;
        new JavaMultiThreadTest("TestMe").testMultiThreadTolerance();
    }
}
