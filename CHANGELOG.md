# Changelog

## Considered for upcoming 0.6.0

- Make Java 9 the default.
- remove deprecations for 0.6.0
- Remove ccp4j
- Look at DContactGeom pos3()/normal3()
- Bug: Dropping first box on Heightfield is wrong.
  -> Cylinder is also often iffy, but that is the same in C++ -> e.g. 7. cylinder (java) or 11th (c++)!
  Remove (deprecate) and replace with PARAM, or at least document what is going on!
- Cleanup GimMath / GimGeometry  (e.g. remove RefFloat)
  -> Remove GIM_BUFFER_ARRAY_POINTER() -> avoid IntArr and ObjArray allocation + usage!!!!!
  -> e.g. in gim_trimesh_update_aabbset()
- Move to Google Java Code Style
- Document: Gimpact works best with (i.e. does not scale above ?) room size <= MAX_AABB_SIZE=1638.0
  -> Why 1638? Can we improve this?
- Create separate distribution (maven jar) with modules? -> Check how module-projects have trouble with including
  non-module projects...
  -> https://www.baeldung.com/java-9-modularity
  -> https://stackoverflow.com/questions/40490520/what-do-i-need-to-build-jdk-9-project-with-non-modular-dependencies-using-maven
- Fix maven warnings:
  - [WARNING] bootstrap class path not set in conjunction with -source 8
    WHen using JDK 9 or later

--> See TODO.txt

## 0.5.3 - 2024-04-28

- Fix INTERNAL ERROR in `FastLSolve.solveL1Straight()` caused by bug in `DLCP.solve1()`.
  [#133](https://github.com/tzaeschke/ode4j/issues/133)
- Fix formatting of debug/info/error message.
  [#135](https://github.com/tzaeschke/ode4j/issues/135)
- QOL: allow calling destroy() multiple times.
  [#139](https://github.com/tzaeschke/ode4j/pull/139)

## 0.5.2 - 2023-10-07
- Fix DVector3.cross() returning always 0. [#128](https://github.com/tzaeschke/ode4j/issues/128)
- Fix JUnit test lint. [#130](https://github.com/tzaeschke/ode4j/pull/130)
- Fix Demo window size to minimum 640x480. [#131](https://github.com/tzaeschke/ode4j/pull/131)
- Add javadoc to indicate that angular velocity is given in radians. [#132](https://github.com/tzaeschke/ode4j/pull/132)

## 0.5.1 - 2023-09-17
- Support for HiDPI screens / Apple Silicon/Retina. Contribution by valb3r,
  [#126]https://github.com/tzaeschke/ode4j/issues/126

## 0.5.0 - 2023-05-27
- Improved Android compatibility
  [#124](https://github.com/tzaeschke/ode4j/pull/124)
- Improved Android compatibility
  [#123](https://github.com/tzaeschke/ode4j/pull/123)
- Avoid garbage collection of DContact. This is now implemented without pooling but with reusing
  DContact instances in place.[#35](https://github.com/tzaeschke/ode4j/issues/35)
- Fixed Java module warning; added `-Werror`; new default `OdeConfig.dDEBUG = false`
  [#122](https://github.com/tzaeschke/ode4j/pull/122)
- BREAKING CHANGE: `DSpace.getGeoms()` now returns `DGeom` instead of `DxGeom`.
  [#121](https://github.com/tzaeschke/ode4j/pull/121)
- DGeom.isSpace() [#120](https://github.com/tzaeschke/ode4j/pull/120)
- CHANGELOG.txt -> .md and added overview to README.md [#119](https://github.com/tzaeschke/ode4j/pull/119)
- Cumulative fix:
    * Fixed missing call to Trimesh callbacks. [#76](https://github.com/tzaeschke/ode4j/issues/76)
    * Deprecated DTriArrayCallback. It was never supported and is considered for removal in ODE.
    * DBody.getGeomIterator()
    * DBody.getFirstGeom() is no longer deprecated
    * Simpler construction methods for DConvex
    * Some javadoc for DConvex
    * DContactJoint.getContact()
    * Updated Trimesh/Capsule collision (CollideTrimeshCCylinder)
      [#77](https://github.com/tzaeschke/ode4j/pull/77)
- Added DQuaternion.copy() and dot() [#117](https://github.com/tzaeschke/ode4j/pull/117)
- Added factory methods and missing API for DRagdoll, DConstrainedBallJoint and DTrimeshHeightfield.
  [#24](https://github.com/tzaeschke/ode4j/issues/24)
- Removed linter warnings when compiling with Java 17. [#116](https://github.com/tzaeschke/ode4j/pull/116)
- DVector3C.cross(). [#115](https://github.com/tzaeschke/ode4j/pull/115)
- DVector3.toDoubleArray(), fromDoubleArray(), fromFloatArray(). [#114](https://github.com/tzaeschke/ode4j/pull/114)
- Turned into tests: DemoI, DemoSpace, DemoSpaceStress. [#110](https://github.com/tzaeschke/ode4j/pull/110)
- Remove or deprecate `Cloneable` and `clone()`. [#109](https://github.com/tzaeschke/ode4j/pull/109)
- DVector3/DQuaternion/...: Better support for call chaining, added isEq(x, y, z, eps) and deprecated clone() and
  DQuaternion.*Euler*().
  [#108](https://github.com/tzaeschke/ode4j/pull/108)
- DVector3/DQuaternion/...: Deprecated hashCode(). [#107](https://github.com/tzaeschke/ode4j/pull/107)
- Removed unnecessary ObjArray from heightfield. [#106](https://github.com/tzaeschke/ode4j/pull/106)
- Added linter and fixed some lint. [#105](https://github.com/tzaeschke/ode4j/pull/105)
- Cleaned up Junit 3 style tests. [#104](https://github.com/tzaeschke/ode4j/pull/104)
- Moved tests into core/core-cpp. [#103](https://github.com/tzaeschke/ode4j/pull/103)
- Cleaned up unit test output. [#102](https://github.com/tzaeschke/ode4j/pull/102)
- Fix maven warnings of type "Use Import/Export Package directive -split-package [...]"
  [#101](https://github.com/tzaeschke/ode4j/pull/101)
- Port updates until 0.16.3. [#100](https://github.com/tzaeschke/ode4j/pull/100)
- Port updates until 0.16.2.
  This includes some libCCD updates missing fro the 0.15.1 update.
  This excludes improved solution finders with anything from *Cooperative* and ThreadedEquationSolverLDLT*.
  [#97](https://github.com/tzaeschke/ode4j/pull/97)
- Added GutHub Actions CI builds for Java 8 and 9. [#95](https://github.com/tzaeschke/ode4j/pull/95)
- Added default logger for demos and tests. [#94](https://github.com/tzaeschke/ode4j/pull/94)
- Added Android API compliance checker, now for API level 24 (the lowest that ode4j passed without changes)
  [#93](https://github.com/tzaeschke/ode4j/pull/93)
- Updated maven dependencies [#92](https://github.com/tzaeschke/ode4j/pull/92)
- Test that internal assertions did not throw Exceptions. This was fixed as part of #86.
  [#74](https://github.com/tzaeschke/ode4j/issues/74)
- Fixes to LWJGL 3.0 [#91](https://github.com/tzaeschke/ode4j/pull/91)
- Moved to LWJGL 3.0 [#89](https://github.com/tzaeschke/ode4j/issues/89)
- Move to Java [#87](https://github.com/tzaeschke/ode4j/issues/87)
  (Cleanup, part of this was done with the move to 0.15.1)
- Update to ODE 0.15.1 [#86](https://github.com/tzaeschke/ode4j/pull/86)
    - This includes increased stability, e.g. the DemoCards won't collapse anymore when 30 levels high.
    - **** WARNING **** : Still missing: trimesh collider updates.
- Changed CI build requirements:
    - Removed openjdk7: Not required. source compatibility is still set to 7, but should
      be safe to change to 8 now (Android Nougat is mostly 8 compatible), this covers > 80% users.
    - Changed oraclejdk build to 11

## 0.4.2

- Fix regression: missing internal version id
- renamed DQuaternion inverse() to eqInverse()

## 0.4.1

- API convenience changes [#79](https://github.com/tzaeschke/ode4j/pull/79)
    - DBody.addLinearVelocity()
    - DVector3C.reAdd()
    - DQuaternion.ZERO and DQuaternion.IDENTITY
    - Additional DQuaternionC API methods: isEq(), length(), lengthSquared(), ...
    - DQuaternion.toEuler() and fromEuler()
    - DQUaternion.invert() inverts a quaternion
    - DVector3.eqToRadians() and eqToDegrees() convert angles in a DVector3.
- Fixed OSGI bundle info to use Java 1.7 instead of 7.0. [#80](https://github.com/tzaeschke/ode4j/pull/80)
- Removed deprecated `<prerequisites>` tag in `.pom`. [#82](https://github.com/tzaeschke/ode4j/pull/82)
- Fixed some javadoc problems. [#83](https://github.com/tzaeschke/ode4j/pull/83)

## 0.4.0

- Fixed javadoc to compile without warnings
- Java 9 compatibility automatically detected and used with Maven profiles
- Changed to Java 9 (output still Java 7) and modularization (io7m)
- Changed to Java 7 and updated some maven dependencies
- Implemented/migrated multi-threading for the stepper (Pjotr)
- SAP-Space optimization: Avoid collision detection for immobile bodies (Pjotr)
- New BVH tree for better scalability with many bodies, ported from the Turbulenz Engine (Pjotr)
- Some fixes, including new random reordering in quickstep, see PR #67, #68 (Pjotr)

## 0.3.1

- Announcement: This will be the last release build with Java 6!
- Convex vs trimesh collision improvement for concave trimeshes, PR #51 (Piotr)
- Quickstep improvements, also fixes issue with DemoFeedback, PR #50 (Piotr)
- Added more stable cylinder stacking, PR #47 (Piotr)
- Deprecated DxSpace.getGeom(i) to fix compilation failure (TZ)
- Ported libccd updates, PR #45 (Piotr)
- Added new SAP space, PR #43 (Piotr)
- Fixes on SAP space, PR #42 (Piotr)
- Fixes on removing geoms from space, PR #41 (Piotr)
- Added support for convex-trimesh collision (Piotr)
- Fixed scaling of tacc in quickstep (issue #38, Eduard Nicodei)
- Fixed Common.__ASSERT methods to act correctly if parameter is '0' or 'false', see issue #32
- Removed unnecessary Common.__ASSERT calls if NPE would trivially be thrown by code below, see #32
- Fixed mode checking in dJointSetTransmissionAxis2(), see issue #33
- Test and fix for issue #31  (NPE in ragdoll)
- Added support for heightfiels with holes (DxTrimeshHeightField)
- Added support for the slf4j loggin API (TZ)
- Added Ragdoll support and ConstrainedBallJoint (Piotr)
- Added sorting of contact by penetration depth (Piotr), see issue #22
- Added TrimeshHeightfield provided by Piotr
- Fixed issue #19 (thanks to Piotr)

- Fixed issue #17
- Fixed NPE in DemoBoxStack-'x'
- Added newArray(size) methods to DVector3 and DMatrix3

## 0.3.0

Migration to 0.13.1

New:

- 3 new joints: DoubleBall, DoubleHinge, Transmission
- New demos: DoubleBall, DoubleHinge, Gyro2, RFriction, Transmission
- Multithreading (well...)

Done:

- main code
- tests
- demos
- Fixed bug in dMatrix3Inv() --> Calculation was incorrect
- Fixed bug in DObject --> destroying joints might have crashed
- Fixed bug: Calling destroy() on a space with a Heightfield cause hang.
- Removed GeomTransform and other deprecated stuff
- Java 1.6 again
- Now passes strict -Xlint:all / -Werror builds

Demos:

- DemoBoxstack has MT disabled.

TODO

- DemoPlane2 explodes...

Missing:

- Multithreading does not work
- Check further:
- DxWorldProcessMemArena.FreeMemArena --> Commented out
- DxWorldProcessContext.FreeArenasList --> Commented out
- ThreadingUtils.java 154-156
- check matrix.h
- Skipped: odeou.*, odetls.*

## 0.2.9

- Fixed possible infinite loop when calling dxQuickStepParameters.clone()
- fixed problem in low level check in Cstdio.fprintf()
- Deprecated equals() methods for DVector3, DVector6 and DQuaternion
- fixed NaN Check in Common.java
- Migration to Java 7!!!!
- Set version to 0.2.9
- Cleaned up javadoc (@remarks/note/warning to <p>REMARKS:/...) and removed @brief/@ingroup/@defgroup
- Updated license dates to 2014.
- CLeaned up plenty of compiler warnings

## 0.2.8

- Fixed issue #9: Exception when updating large trimeshes

## 0.2.7

- Merged 'cpp' package into 'core' package
- Issue #3: porting typo in space collider
