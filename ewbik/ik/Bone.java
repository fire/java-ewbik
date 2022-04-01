/*

Copyright (c) 2015 Eron Gjoni

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and 
associated documentation files (the "Software"), to deal in the Software without restriction, including 
without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION 
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 

 */

package ik;

import ewbik.ik.*;
import ewbik.ik.IKExceptions.NullParentForBoneException;
import ewbik.math.*;
import ewbik.processing.singlePrecision.Kusudama;
import processing.Skeleton3D;
import processing.core.PConstants;
import processing.core.PGraphics;
import processing.core.PMatrix;
import processing.core.PVector;

import java.util.ArrayList;

public class Bone implements ewbik.asj.Saveable, Comparable<Bone> {
    public static boolean drawKusudamas = false;
    public Skeleton3D parentArmature;
    public Kusudama constraints;
    public int ancestorCount = 0;
    protected String tag;
    protected Quaternion lastRotation;
    protected ewbik.processing.sceneGraph.Node3D previousOrientation;
    protected ewbik.processing.sceneGraph.Node3D localNode3D;
    protected ewbik.processing.sceneGraph.Node3D majorRotationNode3D;
    protected float boneHeight;
    protected Bone parent;
    protected ArrayList<Bone> children = new ArrayList<>();
    protected ArrayList<Bone> freeChildren = new ArrayList<>();
    protected ArrayList<Bone> effectoredChildren = new ArrayList<>();
    protected IKPin pin = null;
    protected boolean orientationLock = false;
    protected float stiffnessScalar = 0f;

    // default constructor required for file loader to work.
    public Bone() {
        this.lastRotation = new Quaternion();
    }

    /**
     * @param par             the parent bone for this bone
     * @param tipHeading      the orienational heading of this bone (global vs
     *                        relative coords specified in coordinateType)
     * @param rollHeading     axial rotation heading of the bone (it's z-axis)
     * @param inputTag        some user specified name for the bone, if desired
     * @param inputBoneHeight bone length
     * @param coordinateType
     * @throws NullParentForBoneException
     */
    public Bone(Bone par, // parent bone
                PVector tipHeading, // the orienational heading of this bone (global vs relative coords specified in
                // coordinateType)
                PVector rollHeading, // axial rotation heading of the bone (it's z-axis)
                String inputTag, // some user specified name for the bone, if desired
                float inputBoneHeight, // bone length
                frameType coordinateType) throws NullParentForBoneException {
        Vector3 tipHeading1 = ewbik.processing.sceneGraph.Node3D.toVec3f(tipHeading);
        Vector3 rollHeading1 = ewbik.processing.sceneGraph.Node3D.toVec3f(rollHeading);

        this.lastRotation = new Quaternion();
        if (par != null) {
            if (inputTag == null || inputTag == "") {
                this.tag = Integer.toString(System.identityHashCode(this));
            } else
                this.tag = inputTag;
            this.boneHeight = inputBoneHeight;

            Ray3 tipHeadingRay = new Ray3(((Bone) par).getTip_(), tipHeading1);
            Ray3 rollHeadingRay = new Ray3(((Bone) par).getTip_(), rollHeading1);
            Vector3 tempTip = tipHeading1.copy();
            tempTip.set(0, 0, 0);
            Vector3 tempRoll = rollHeading1.copy();
            tempRoll.set(0, 0, 0);
            Vector3 tempX = tempRoll.copy();
            tempX.set(0, 0, 0);

            if (coordinateType == Bone.frameType.GLOBAL) {
                tempTip = tipHeadingRay.heading();
                tempRoll = rollHeadingRay.heading();
            } else if (coordinateType == Bone.frameType.RELATIVE) {
                tempTip = ((Bone) par).localAxes().getGlobalOf(tipHeadingRay.heading());
                tempRoll = ((Bone) par).localAxes().getGlobalOf(rollHeadingRay.heading());
            } else {

                System.out.println("WOAH WOAH WOAH");
            }

            tempX = tempTip.crossCopy(tempRoll);
            // TODO: this commented out one is correct. Using old version test chirality
            // code.
            tempRoll = tempX.crossCopy(tempTip);
            // tempRoll = tempTip.crossCopy(tempX);

            tempX.normalize();
            tempTip.normalize();
            tempRoll.normalize();

            this.parent = par;
            this.parentArmature = this.parent.parentArmature;
            Bone.this.parentArmature.addToBoneList(this);

            Bone.this.generateAxes(Bone.this.parent.getTip_(), tempX, tempTip, tempRoll);
            // this.localAxes.orthoNormalize(true);
            Bone.this.localNode3D.setParent(Bone.this.parent.localNode3D);

            Bone.this.previousOrientation = Bone.this.localNode3D.attachedCopy(true);

            Bone.this.majorRotationNode3D = Bone.this.parent.localAxes().getGlobalCopy();
            Bone.this.majorRotationNode3D.translateTo(Bone.this.parent.getTip_());
            Bone.this.majorRotationNode3D.setParent(Bone.this.parent.localNode3D);

            this.parent.addFreeChild(this);
            this.parent.addChild(this);
            this.updateAncestorCount();
            // this.updateSegmentedArmature();
        } else {
            throw ewbik.ik.IKExceptions.NullParentForBoneException();
        }

    }

    /**
     * @param armature       the parent armature for this bone
     * @param tipHeading     the orienational heading of this bone (global vs
     *                       relative coords specified in coordinateType)
     * @param rollHeading    axial rotation heading of the bone (it's z-axis)
     * @param inputTag       some user specified name for the bone, if desired
     * @param boneHeight     bone length
     * @param coordinateType
     * @throws NullParentForBoneException
     */
    public Bone(
            Skeleton3D parArma,
            Vector3 tipHeading,
            Vector3 rollHeading,
            String inputTag,
            float inputBoneHeight,
            frameType coordinateType)
            throws NullParentForBoneException {

        this.lastRotation = new Quaternion();
        if (parArma != null) {
            if (inputTag == null || inputTag == "") {
                this.tag = Integer.toString(System.identityHashCode(this));
            } else
                this.tag = inputTag;

            Ray3 tipHeadingRay;
            tipHeadingRay = new Ray3(parArma.localNode3D.origin_(), tipHeading);
            tipHeadingRay.getRayScaledTo(inputBoneHeight);
            Ray3 rollHeadingRay = new Ray3(parArma.localNode3D.origin_(), rollHeading);
            Vector3 tempTip = tipHeading.copy();
            Vector3 tempRoll = rollHeading.copy();
            Vector3 tempX = tempTip.copy();

            if (coordinateType == frameType.GLOBAL) {
                tempTip = tipHeadingRay.heading();
                tempRoll = rollHeadingRay.heading();
            } else if (coordinateType == frameType.RELATIVE) {
                tempTip = parArma.localNode3D.getGlobalOf(tipHeadingRay.heading());
                tempRoll = parArma.localNode3D.getGlobalOf(rollHeadingRay.heading());
            } else {
                System.out.println("WOAH WOAH WOAH");
            }

            tempX = tempTip.crossCopy(tempRoll);
            // TODO: this commented out one is correct. Using old version test chirality
            // code.
            tempRoll = tempX.crossCopy(tempTip);
            // tempRoll = tempTip.crossCopy(tempX);

            tempX.normalize();
            tempTip.normalize();
            tempRoll.normalize();

            this.parentArmature = parArma;
            parentArmature.addToBoneList(this);

            generateAxes(parentArmature.localNode3D.origin_(), tempX, tempTip, tempRoll);
            localNode3D.setParent(parentArmature.localNode3D);
            previousOrientation = localNode3D.attachedCopy(true);

            majorRotationNode3D = parentArmature.localAxes().getGlobalCopy();
            majorRotationNode3D.setParent(parentArmature.localAxes());

            this.boneHeight = inputBoneHeight;
            this.updateAncestorCount();
        } else {
            throw new NullParentForBoneException();
        }

    }

    /**
     * Creates a new bone of specified length emerging from the parentBone.
     * The new bone extends in the same exact direction as the parentBone.
     * You can then manipulate its orientation using something like
     * rotAboutFrameX(), rotAboutFrameY(), or rotAboutFrameZ().
     * You can also change its frame of rotation using setFrameOfRotation(Axes
     * rotationFrame);
     *
     * @param par             the parent bone to which this bone is attached.
     * @param inputTag        some user specified name for the bone, if desired
     * @param inputBoneHeight bone length
     */


    /**
     * @param par             the parent bone to which this bone is attached.
     * @param xAngle          how much the bone should be pitched relative to its
     *                        parent bone
     * @param yAngle          how much the bone should be rolled relative to its
     *                        parent bone
     * @param zAngle          how much the bone should be yawed relative to its
     *                        parent bone
     * @param inputTag        some user specified name for the bone, if desired
     * @param inputBoneHeight bone length
     */
    public Bone(Bone par, // parent bone
                float xAngle, // how much the bone should be pitched relative to its parent bone
                float yAngle, // how much the bone should be rolled relative to its parent bone
                float zAngle, // how much the bone should be yawed relative to its parent bone
                String inputTag, // some user specified name for the bone, if desired
                float inputBoneHeight // bone length
    ) {

        this.lastRotation = new Quaternion();
        if (par != null) {
            if (inputTag == null || inputTag == "") {
                this.tag = Integer.toString(System.identityHashCode(this));
            } else {
                this.tag = inputTag;
            }
            this.boneHeight = inputBoneHeight;

            ewbik.processing.sceneGraph.Node3D tempNode3D = par.localAxes().getGlobalCopy();
            Quaternion newRot = new Quaternion(RotationOrder.XZY, xAngle, yAngle, zAngle);
            tempNode3D.rotateBy(newRot);

            this.parent = par;
            this.parentArmature = this.parent.parentArmature;
            Bone.this.parentArmature.addToBoneList(this);

            Bone.this.generateAxes(Bone.this.parent.getTip_(), tempNode3D.x_().heading(), tempNode3D.y_().heading(), tempNode3D.z_().heading());
            // this.localAxes.orthoNormalize(true);
            Bone.this.localNode3D.setParent(Bone.this.parent.localNode3D);
            Bone.this.previousOrientation = Bone.this.localNode3D.attachedCopy(true);

            Bone.this.majorRotationNode3D = Bone.this.parent.localAxes().getGlobalCopy();
            Bone.this.majorRotationNode3D.translateTo(Bone.this.parent.getTip_());
            Bone.this.majorRotationNode3D.setParent(Bone.this.parent.localNode3D);

            this.parent.addFreeChild(this);
            this.parent.addChild(this);
            this.updateAncestorCount();
        } else {
            throw ewbik.ik.IKExceptions.NullParentForBoneException();
        }

    }

    public Bone(Bone par, // parent bone
                String inputTag, // some user specified name for the bone, if desired
                float inputBoneHeight // bone length
    ) {
        this.lastRotation = new Quaternion();
        if (par != null) {
            if (inputTag == null || inputTag == "") {
                this.tag = Integer.toString(System.identityHashCode(this));
            } else {
                this.tag = inputTag;
            }
            this.boneHeight = inputBoneHeight;

            ewbik.processing.sceneGraph.Node3D tempNode3D = par.localAxes().getGlobalCopy();
            Quaternion newRot = new Quaternion();
            tempNode3D.rotateBy(newRot);

            this.parent = par;
            this.parentArmature = this.parent.parentArmature;
            Bone.this.parentArmature.addToBoneList(this);

            Bone.this.generateAxes(Bone.this.parent.getTip_(), tempNode3D.x_().heading(), tempNode3D.y_().heading(), tempNode3D.z_().heading());
            // this.localAxes.orthoNormalize(true);
            Bone.this.localNode3D.setParent(Bone.this.parent.localNode3D);
            Bone.this.previousOrientation = Bone.this.localNode3D.attachedCopy(true);

            Bone.this.majorRotationNode3D = Bone.this.parent.localAxes().getGlobalCopy();
            Bone.this.majorRotationNode3D.translateTo(Bone.this.parent.getTip_());
            Bone.this.majorRotationNode3D.setParent(Bone.this.parent.localNode3D);

            this.parent.addFreeChild(this);
            this.parent.addChild(this);
            this.updateAncestorCount();
        } else {
            throw ewbik.ik.IKExceptions.NullParentForBoneException();
        }
    }

    public static void setDrawKusudamas(boolean draw) {
        drawKusudamas = draw;
    }

    protected void generateAxes(Vector3 origin, Vector3 x, Vector3 y, Vector3 z) {
        this.localNode3D = new ewbik.processing.sceneGraph.Node3D(origin, x, y, z);
    }

    public PVector getBase() {
        ewbik.math.Vector3 base = (ewbik.math.Vector3) getBase_();
        return new PVector(base.x, base.y, base.z);
    }

    public PVector getTip() {
        ewbik.math.Vector3 tip = (ewbik.math.Vector3) getTip_();
        return new PVector(tip.x, tip.y, tip.z);
    }

    protected IKPin createAndReturnPinOnAxes(ewbik.processing.sceneGraph.Node3D on) {
        return new IKPin(
                (ewbik.processing.sceneGraph.Node3D) on,
                true,
                this);
    }

    public void enablePin(PVector pin) {
        enablePin_(new ewbik.math.Vector3(pin.x, pin.y, pin.z));
    }

    public void setPin(PVector pin) {
        setPin_(new ewbik.math.Vector3(pin.x, pin.y, pin.z));
    }

    /**
     * @return In the case of this out-of-the-box class, getPin() returns a IKVector
     * indicating
     * the spatial target of the pin.
     */
    public PVector getPinLocation() {
        if (pin == null)
            return null;
        else {
            ewbik.math.Vector3 loc = (ewbik.math.Vector3) pin.getLocation_();
            return new PVector(loc.x, loc.y, loc.z);
        }
    }

    public void drawMeAndChildren(PGraphics pg, int boneCol, float pinSize) {

        if (this.constraints != null && drawKusudamas) {
            pg.pushMatrix();
            ((ewbik.processing.singlePrecision.Kusudama) constraints).drawMe(pg, boneCol, pinSize);
            pg.popMatrix();
        }

        PMatrix localMat = localAxes().getLocalPMatrix();
        pg.applyMatrix(localMat);

        pg.beginShape(PConstants.TRIANGLE_FAN);
        pg.fill(pg.color(0, 255 - boneCol, boneCol));
        // pg.stroke(lineColor);
        float circumference = (float) (boneHeight / 8f);
        pg.noStroke();
        pg.vertex(0, (float) boneHeight, 0);
        pg.vertex(circumference, circumference, 0);
        pg.vertex(0, circumference, circumference);
        pg.vertex(-circumference, circumference, 0);
        pg.vertex(0, circumference, -circumference);
        pg.vertex(circumference, circumference, 0);
        pg.endShape();
        pg.beginShape(PConstants.TRIANGLE_FAN);
        pg.vertex(0, 0, 0);
        pg.vertex(0, circumference, -circumference);
        pg.vertex(circumference, circumference, 0);
        pg.vertex(0, circumference, circumference);
        pg.vertex(-circumference, circumference, 0);
        pg.vertex(0, circumference, -circumference);
        pg.endShape();
        pg.emissive(0, 0, 0);

        for (Bone b : getChildren()) {
            pg.pushMatrix();
            b.drawMeAndChildren(pg, boneCol + 10, pinSize);
            pg.popMatrix();
        }

        pg.strokeWeight(4f);
        if (this.isPinned()) {
            ((ewbik.processing.sceneGraph.Node3D) this.getIKPin().getAxes()).drawMe(pg, pinSize);
        }

        if (this.isPinned()) {
            pg.strokeWeight(2f);
            localAxes().drawMe(pg, pinSize);
        }
    }

    public IKPin getIKPin() {
        return (IKPin) this.pin;
    }

    /**
     * Get the Axes associated with this bone.
     */
    public ewbik.processing.sceneGraph.Node3D localAxes() {
        return (ewbik.processing.sceneGraph.Node3D) this.localNode3D;
    }

    /**
     * Get the Axes relative to which this bone's rotations are defined. (If the
     * bone has constraints, this will be the
     * constraint Axes)
     *
     * @return
     */
    @SuppressWarnings("unchecked")
    public ewbik.processing.sceneGraph.Node3D getMajorRotationAxes() {
        return (ewbik.processing.sceneGraph.Node3D) this.majorRotationNode3D;
    }

    @SuppressWarnings("unchecked")
    public ArrayList<Bone> getChildren() {
        return (ArrayList<Bone>) (ArrayList<? extends Bone>) children;
    }

    public void setChildren(ArrayList<? extends Bone> children) {
        this.children = (ArrayList<Bone>) children;
    }

    private void updateAncestorCount() {
        int countedAncestors = 0;
        Bone currentBone = this.parent;
        while (currentBone != null) {
            countedAncestors++;
            currentBone = currentBone.parent;
        }
        setAncestorCount(countedAncestors);
    }

    /**
     * updates the ancestor count for this bone, and
     * sets the ancestor count of all child bones
     * to this bone's ancestor count +1;
     *
     * @param count
     */
    private void setAncestorCount(int count) {
        this.ancestorCount = count;
        for (Bone b : this.children) {
            b.setAncestorCount(this.ancestorCount + 1);
        }
    }

    public Bone getParent() {
        return this.parent;
    }

    public void attachToParent(Bone inputParent) {
        inputParent.addChild(this);
        this.parent = inputParent;
        this.updateAncestorCount();
    }

    public void solveIKFromHere() {
        this.parentArmature.IKSolver(this);
    }

    public void snapToConstraints() {
        if (constraints != null) {
            constraints.snapToLimits();
        }
    }

    /**
     * Called whenever this bone's orientation has changed due to an Inverse
     * Kinematics computation.
     * This function is called only once per IK solve, not per iteration. Meaning
     * one call to Solve IK **SHOULD NOT**
     * results in more than one call to each affected bone.
     */
    public void IKUpdateNotification() {

    }

    /**
     * same as snapToConstraints, but operates on user
     * supplied axes meant to correspond to the bone's
     * localAxes and majorRotationAxes
     * <p>
     * you are unlikely to need to use this, and at the moment
     * it presumes KusudamaExample constraints
     */
    public void setAxesToSnapped(ewbik.processing.sceneGraph.Node3D toSet, ewbik.processing.sceneGraph.Node3D limitingNode3D, float cosHalfAngleDampen) {
        if (constraints != null && Kusudama.class.isAssignableFrom(constraints.getClass())) {
            ((Kusudama) constraints).setAxesToSnapped(toSet, limitingNode3D, cosHalfAngleDampen);
        }
    }

    public void setAxesToReturnfulled(ewbik.processing.sceneGraph.Node3D toSet, ewbik.processing.sceneGraph.Node3D limitingNode3D, float cosHalfAngleDampen,
                                      float angleDampen) {
        if (constraints != null && Kusudama.class.isAssignableFrom(constraints.getClass())) {
            ((Kusudama) constraints).setAxesToReturnfulled(toSet, limitingNode3D, cosHalfAngleDampen,
                    angleDampen);
        }
    }

    public void setPin_(Vector3 pin) {
        if (this.pin == null) {
            this.enablePin_(pin);
        } else {
            this.pin.translateTo_(pin);
        }
    }

    /**
     * @param newConstraint a constraint Object to add to this bone
     * @return the constraintObject that was just added
     */
    public Kusudama addConstraint(Kusudama newConstraint) {
        constraints = newConstraint;
        return constraints;
    }

    /**
     * @return this bone's constraint object.
     */
    public Kusudama getConstraint() {
        return constraints;
    }

    /**
     * @return an array where each element indicated how much this bone is rotated
     * on the X,Y,Z (in that order)
     * axes relative to its constraint Axes.
     * If there are no constraint Axes, the result is relative to the parent
     * bone.
     * If the bone has no parent, this method throws an exception.
     * @throws NullParentForBoneException
     */
    public float[] getXYZAngle() throws NullParentForBoneException {
        if (this.parent != null) {
            this.localAxes().markDirty();
            this.localAxes().updateGlobal();
            ewbik.processing.sceneGraph.Node3D result = this.localAxes().getGlobalCopy();
            this.getMajorRotationAxes().updateGlobal();
            this.getMajorRotationAxes().globalMBasis.setToLocalOf(result.globalMBasis, result.globalMBasis);
            return result.globalMBasis.rotation.getAngles(RotationOrder.XYZ);
        } else {
            return null;
        }
    }

    /**
     * @return An Apache Commons Rotation object representing the rotation of this
     * bone relative to its
     * reference frame. The Rotation object is more versatile and robust
     * than an array of angles.
     * And allows you to treat rotation in a wide variety of conventions.
     */
    public Quaternion getRotation() {
        return (new Quaternion(this.majorRotationNode3D.x_().heading(), this.majorRotationNode3D.y_().heading(),
                this.localAxes().x_().heading(), this.localAxes().y_().heading()));
    }

    /**
     * @return An Apache Commons Rotation object representing the rotation which
     * transforms this
     * BoneExample from its previous orientation to its current orientation.
     */

    public Quaternion getRotationFromPrevious() {
        return lastRotation;
    }

    /**
     * @return the reference frame representing this bone's previous orientation
     * relative to
     * its parent.
     */
    public ewbik.processing.sceneGraph.Node3D getPreviousOrientation() {
        return previousOrientation;
    }

    /**
     * Rotate the bone about its frame of reference by a custom Apache Commons
     * Rotation object
     *
     * @param rot
     */
    public void rotateBy(Quaternion rot) {
        this.previousOrientation.alignLocalsTo(localNode3D);
        this.localNode3D.rotateBy(rot);

        this.lastRotation.set(rot);
    }

    /**
     * rotates the bone about the major X axis of rotation,
     * obeying constraints by default
     *
     * @param amt number of degrees to rotate by
     */
    public void rotAboutFrameX(float amt) {
        rotAboutFrameX(amt, true);
    }

    /**
     * rotates the bone about the major Y axis of rotation,
     * obeying constraints by default
     *
     * @param amt number of degrees to rotate by
     */
    public void rotAboutFrameY(float amt) {
        rotAboutFrameY(amt, true);
    }

    /**
     * rotates the bone about the major Z axis of rotation,
     * obeying constraints by default
     *
     * @param amt number of degrees to rotate by
     */
    public void rotAboutFrameZ(float amt) {
        rotAboutFrameZ(amt, true);
    }

    public void rotateAxially(float amt) {
        rotateAxially(amt, true);
    }

    /**
     * rotates the bone about the major X axis of rotation
     *
     * @param amt            number of degrees to rotate by
     * @param obeyConstrants whether or not this functions should obey constraints
     *                       when rotating the bone
     */
    public void rotAboutFrameX(float amt, boolean obeyConstraints) {
        // localAxes().alignLocalsTo(previousOrientation);
        previousOrientation.alignLocalsTo(localNode3D);

        Quaternion xRot = new Quaternion(majorRotationNode3D.x_().heading(), amt);
        localNode3D.rotateBy(xRot);

        lastRotation.set(xRot);

        if (obeyConstraints)
            this.snapToConstraints();
    }

    /**
     * rotates the bone about the major Y axis of rotation
     *
     * @param amt            number of degrees to rotate by
     * @param obeyConstrants whether or not this functions should obey constraints
     *                       when rotating the bone
     */
    public void rotAboutFrameY(float amt, boolean obeyConstraints) {
        previousOrientation.alignLocalsTo(localNode3D);

        Quaternion yRot = new Quaternion(majorRotationNode3D.y_().heading(), amt);
        localNode3D.rotateBy(yRot);

        lastRotation.set(yRot);

        if (obeyConstraints)
            this.snapToConstraints();
    }

    /**
     * rotates the bone about the major Z axis of rotation
     *
     * @param amt            number of degrees to rotate by
     * @param obeyConstrants whether or not this functions should obey constraints
     *                       when rotating the bone
     */
    public void rotAboutFrameZ(float amt, boolean obeyConstraints) {
        previousOrientation.alignLocalsTo(localNode3D);

        Quaternion zRot = new Quaternion(majorRotationNode3D.z_().heading(), amt);
        localNode3D.rotateBy(zRot);

        lastRotation.set(zRot);

        if (obeyConstraints)
            this.snapToConstraints();
    }

    /**
     * rotates the bone about its own length. (Like a drill, or The Queen waving).
     *
     * @param number  of degrees to rotate by
     * @param whether or not this functions should obey constraints when rotating
     *                the bone
     */
    public void rotateAxially(float amt, boolean obeyConstraints) {
        localNode3D.rotateAboutY(amt, true);

        if (obeyConstraints)
            this.snapToConstraints();
    }

    /**
     * @param rotationFrameCoordinates the Axes around which rotAboutFrameX,
     *                                 rotAboutFrameY, and rotAboutFrameZ will
     *                                 rotate,
     *                                 and against which getXZYAngle() will be
     *                                 computed.
     *                                 The input is expected to be in RELATIVE
     *                                 coordinates.
     *                                 so specifying these axes as having an <br>
     *                                 x component heading of (1,0,0) and <br>
     *                                 y component heading of (0,1,0) and<br>
     *                                 z component heading of (0,0,1) <br>
     *                                 <br>
     *                                 is equivalent to specifying the
     *                                 frameOfRotation of this bone as being
     *                                 perfectly aligned
     *                                 with the localAxes of its parent bone.<br>
     *                                 <br>
     *                                 <p>
     *                                 The physical intuition of this is maybe
     *                                 something like
     *                                 "at what angle did I place the servos on this
     *                                 joint"<br>
     *                                 <br>
     *                                 <p>
     *                                 It doesn't necessarily determine where the
     *                                 bone can rotate, but it does determine *how*
     *                                 the bone would rotate to get there.<br>
     *                                 <br>
     *                                 <p>
     *                                 This is also used to set constraints. For
     *                                 example, euler constraints are computed
     *                                 against these axes
     *                                 and the limitCones and axial twist limits of
     *                                 KusudamaExample are specified relative to
     *                                 these Axes.<br>
     *                                 <br>
     *                                 <p>
     *                                 Changing these axes is essentially the
     *                                 equivalent of rotating the joint on which
     *                                 this bone rests,
     *                                 while keeping the bone in globally in
     *                                 place.<br>
     *                                 <br>
     *                                 <p>
     *                                 You don't need to change this unless you
     *                                 start wishing you could change this.
     */
    public void setFrameofRotation(ewbik.processing.sceneGraph.Node3D rotationFrameCoordinates) {
        majorRotationNode3D.alignLocalsTo(rotationFrameCoordinates);
        if (parent != null) {
            majorRotationNode3D.translateTo(parent.getTip_());
        }
    }

    /**
     * Disables the pin for this bone so that it no longer interests the IK Solver.
     * However, all information abut the pin is maintained, so the pin can be turned
     * on again with enablePin().
     */
    public void disablePin() {
        pin.disable();
        if (this.effectoredChildren.size() == 0) {
            notifyAncestorsOfUnpin();
        }
        this.updateSegmentedArmature();
    }

    /**
     * Entirely removes the pin from this bone. Any child pins attached to it are
     * reparented to this
     * pin's parent.
     */
    public void removePin() {
        pin.disable();
        if (this.effectoredChildren.size() == 0) {
            notifyAncestorsOfUnpin();
        }
        pin.removalNotification();
        this.updateSegmentedArmature();
    }

    /**
     * Enables an existing pin for this BoneExample. Or creates a pin for this bone
     * at the bone's tip.
     */
    public void enablePin() {
        if (pin == null) {
            ewbik.processing.sceneGraph.Node3D pinNode3D = this.localAxes().getGlobalCopy();
            pinNode3D.setParent(this.parentArmature.localAxes().getParentAxes());
            pin = createAndReturnPinOnAxes(pinNode3D);
        }
        pin.enable();
        freeChildren.clear();
        for (Bone child : getChildren()) {
            if (child.pin != null && !child.pin.isEnabled()) {
                addFreeChild(child);
                // System.out.println("childAdd");
            }
        }
        // System.out.println("notifying ancestors");
        notifyAncestorsOfPin(false);
        // System.out.println("updating segment armature");
        this.updateSegmentedArmature();
        // System.out.println("segment armature updated");
    }

    /**
     * Creates a pin for this bone
     *
     * @param pinTo the position of the pin in the coordinateFrame of the
     *              parentArmature.
     */

    public void enablePin_(Vector3 pinTo) {
        if (pin == null) {
            ewbik.processing.sceneGraph.Node3D pinNode3D = this.localAxes().getGlobalCopy();
            pinNode3D.setParent(this.parentArmature.localAxes().getParentAxes());
            pinNode3D.translateTo(pinTo);
            pin = createAndReturnPinOnAxes(pinNode3D);
        } else
            pin.translateTo_(pinTo);
        pin.enable();
        freeChildren.clear();
        for (Bone child : getChildren()) {
            if (!child.pin.isEnabled()) {
                addFreeChild(child);
            }
        }
        notifyAncestorsOfPin();
        // this.updateSegmentedArmature();
    }

    /**
     * @return true if the bone has a pin enabled, false otherwise.
     */
    public boolean isPinned() {
        if (pin == null || !pin.isEnabled()) {
            return false;
        } else {
            return true;
        }

    }

    /**
     * Creates / enables a pin if there no pin is active, disables the pin if it is
     * active.
     */
    public void togglePin() {
        if (this.pin == null)
            this.enablePin();
        this.pin.toggle();
        updateSegmentedArmature();
    }

    public ArrayList<Bone> returnChildrenWithPinnedDescendants() {
        ArrayList<Bone> childrenWithPinned = new ArrayList<Bone>();
        for (Bone c : getChildren()) {
            if (c.hasPinnedDescendant())
                childrenWithPinned.add(c);
        }
        return childrenWithPinned;
    }

    public ArrayList<? extends Bone> getMostImmediatelyPinnedDescendants() {
        ArrayList<Bone> mostImmediatePinnedDescendants = new ArrayList<Bone>();
        this.addSelfIfPinned(mostImmediatePinnedDescendants);
        return mostImmediatePinnedDescendants;
    }

    public ewbik.processing.sceneGraph.Node3D getPinnedAxes() {
        if (this.pin == null)
            return null;
        return this.pin.getAxes();
    }

    public void addSelfIfPinned(ArrayList<Bone> pinnedBones2) {
        if (this.isPinned()) {
            pinnedBones2.add(this);
        } else {
            for (Bone child : getChildren()) {
                child.addSelfIfPinned(pinnedBones2);
            }
        }
    }

    public void notifyAncestorsOfPin(boolean updateSegments) {
        if (this.parent != null) {
            parent.addToEffectored(this);
        }
        if (updateSegments)
            parentArmature.updateArmatureSegments();
    }

    public void notifyAncestorsOfPin() {
        notifyAncestorsOfPin(true);
    }

    public void notifyAncestorsOfUnpin() {
        if (this.parent != null) {
            parent.removeFromEffectored(this);
        }
        parentArmature.updateArmatureSegments();
    }

    public void addToEffectored(Bone Bone) {
        int freeIndex = freeChildren.indexOf(Bone);
        if (freeIndex != -1)
            freeChildren.remove(freeIndex);

        if (effectoredChildren.contains(Bone)) {
        } else {
            effectoredChildren.add(Bone);
        }
        if (this.parent != null) {
            parent.addToEffectored(this);
        }
    }

    public void removeFromEffectored(Bone Bone) {
        int effectoredIndex = effectoredChildren.indexOf(Bone);
        if (effectoredIndex != -1)
            effectoredChildren.remove(effectoredIndex);

        if (freeChildren.contains(Bone)) {
        } else {
            addFreeChild(Bone);
        }
        if (this.parent != null && this.effectoredChildren.size() == 0 && this.pin != null && this.pin.isEnabled()) {
            parent.removeFromEffectored(this);
        }
    }

    public Bone getPinnedRootBone() {
        Bone rootBone = this;
        while (rootBone.parent != null && !rootBone.parent.pin.isEnabled()) {
            rootBone = rootBone.parent;
        }
        return rootBone;
    }

    public void updateSegmentedArmature() {
        this.parentArmature.updateArmatureSegments();
    }

    public String getTag() {
        return this.tag;
    }

    public void setTag(String newTag) {
        parentArmature.updateBoneTag(this, this.tag, newTag);
        this.tag = newTag;
    }

    public Vector3 getBase_() {
        return localNode3D.origin_().copy();
    }

    public Vector3 getTip_() {
        return localNode3D.y_().getScaledTo(boneHeight);
    }

    /**
     * removes this BoneExample and any of its children from the armature.
     */
    public void deleteBone() {
        ArrayList<Bone> bones = new ArrayList<>();
        Bone root = parentArmature.getRootBone();
        root.hasChild(bones, this);
        for (Bone p : bones) {
            System.out.println("removing from" + p);
            p.removeFromEffectored(this);
            for (Bone ab : this.effectoredChildren) {
                p.removeFromEffectored(ab);
            }
            p.getChildren().remove(this);
            p.freeChildren.remove(this);
        }
        this.parentArmature.removeFromBoneList(this);
    }

    /* adds this bone to the arrayList if inputBone is among its children */
    private void hasChild(ArrayList<Bone> list, Bone query) {
        if (getChildren().contains(query))
            list.add(this);
        for (Bone c : getChildren()) {
            c.hasChild(list, query);
        }
    }

    public float getBoneHeight() {
        return this.boneHeight;
    }

    public void setBoneHeight(float inBoneHeight) {
        this.boneHeight = inBoneHeight;
        for (Bone child : this.getChildren()) {
            child.localAxes().translateTo(this.getTip_());
            child.majorRotationNode3D.translateTo(this.getTip_());
        }
    }

    public boolean hasPinnedDescendant() {
        if (this.isPinned())
            return true;
        else {
            boolean result = false;
            for (Bone c : getChildren()) {
                if (c.hasPinnedDescendant()) {
                    result = true;
                    break;
                }
            }
            return result;
        }

    }

    public boolean getIKOrientationLock() {
        return this.orientationLock;
    }

    /**
     * if set to true, the IK system will not rotate this bone
     * as it solves the IK chain.
     *
     * @param val
     */
    public void setIKOrientationLock(boolean val) {
        this.orientationLock = val;
    }

    public void addChild(Bone bone) {
        if (this.getChildren().indexOf(bone) == -1) {
            ((ArrayList<Bone>) getChildren()).add(bone);
        }
        // parentArmature.updateArmatureSegments();
    }

    public void addFreeChild(Bone bone) {
        if (this.freeChildren.indexOf(bone) == -1) {
            freeChildren.add(bone);
        }
        parentArmature.updateArmatureSegments();
    }

    public void addEffectoredChild(Bone bone) {
        if (this.effectoredChildren.indexOf(bone) == -1) {
            this.effectoredChildren.add(bone);
        }

    }

    public void addDescendantsToArmature() {
        for (Bone b : getChildren()) {
            parentArmature.addToBoneList(b);
            b.addDescendantsToArmature();
        }
    }

    /**
     * The stiffness of a bone determines how much the IK solver should
     * prefer to avoid rotating it if it can. A value of 0 means the solver will
     * rotate this bone as much as the overall dampening parameter will
     * allow it to per iteration. A value of 0.5 means the solver will
     * rotate it half as much as the dampening parameter will allow,
     * and a value of 1 effectively means the solver is not allowed
     * to rotate this bone at all.
     *
     * @return a value between 1 and 0.
     */
    public float getStiffness() {
        return stiffnessScalar;
    }

    /**
     * The stiffness of a bone determines how much the IK solver should
     * prefer to avoid rotating it if it can. A value of 0 means the solver will
     * rotate this bone as much as the overall dampening parameter will
     * allow it to per iteration. A value of 0.5 means the solver will
     * rotate it half as much as the dampening parameter will allow,
     * and a value of 1 effectively means the solver is not allowed
     * to rotate this bone at all.
     */
    public void setStiffness(float stiffness) {
        stiffnessScalar = stiffness;
        if (parentArmature != null) {
            ewbik.ik.ShadowSkeleton3D s = parentArmature.boneSegmentMap.get(this);
            if (s != null) {
                ewbik.ik.ShadowSkeleton3D.WorkingBone wb = s.simulatedBones.get(this);
                if (wb != null) {
                    wb.updateCosDampening();
                }
            }
        }
    }

    @Override
    public void loadFromJSONObject(ewbik.asj.data.JSONObject j, ewbik.asj.LoadManager l) {
        this.localNode3D = (ewbik.processing.sceneGraph.Node3D) l.getObjectFromClassMaps(ewbik.processing.sceneGraph.Node3D.class, j.getString("localAxes"));
        this.majorRotationNode3D = (ewbik.processing.sceneGraph.Node3D) l.getObjectFromClassMaps(ewbik.processing.sceneGraph.Node3D.class,
                j.getString("majorRotationAxes"));
        this.parentArmature = (Skeleton3D) l.getObjectFromClassMaps(Skeleton3D.class,
                j.getString("parentArmature"));
        l.arrayListFromJSONArray(j.getJSONArray("children"), this.children, this.getClass());
        this.boneHeight = j.getFloat("boneHeight");
        if (j.hasKey("stiffness"))
            this.setStiffness(j.getFloat("stiffness"));
        if (j.hasKey("constraints"))
            this.constraints = (ewbik.processing.singlePrecision.Kusudama) l.getObjectFromClassMaps(ewbik.processing.singlePrecision.Kusudama.class, j.getString("constraints"));
        if (j.hasKey("IKPin"))
            this.pin = (IKPin) l.getObjectFromClassMaps(IKPin.class, j.getString("IKPin"));
        this.tag = j.getString("tag");
    }

    @Override
    public ewbik.asj.data.JSONObject getSaveJSON(ewbik.asj.SaveManager saveManager) {
        ewbik.asj.data.JSONObject thisBone = new ewbik.asj.data.JSONObject();
        thisBone.setString("identityHash", this.getIdentityHash());
        thisBone.setString("localAxes", this.localNode3D.getIdentityHash());
        thisBone.setString("majorRotationAxes", majorRotationNode3D.getIdentityHash());
        thisBone.setString("parentArmature", ((ewbik.asj.Saveable) parentArmature).getIdentityHash());
        ewbik.asj.data.JSONArray children = saveManager.arrayListToJSONArray(getChildren());
        thisBone.setJSONArray("children", children);
        if (constraints != null) {
            thisBone.setString("constraints", constraints.getIdentityHash());
        }
        if (pin != null)
            thisBone.setString("IKPin", pin.getIdentityHash());

        thisBone.setFloat("boneHeight", this.getBoneHeight());
        thisBone.setFloat("stiffness", this.getStiffness());
        thisBone.setString("tag", this.getTag());

        return thisBone;
    }

    @Override
    public void makeSaveable(ewbik.asj.SaveManager saveManager) {
        saveManager.addToSaveState(this);
        if (this.getIKPin() != null) {
            this.getIKPin().makeSaveable(saveManager);
        }
        for (Bone b : this.children) {
            b.makeSaveable(saveManager);
        }
        if (this.getConstraint() != null) {
            this.getConstraint().makeSaveable(saveManager);
        }
    }

    @Override
    public void notifyOfSaveIntent(ewbik.asj.SaveManager saveManager) {
        // TODO Auto-generated method stub

    }

    @Override
    public void notifyOfSaveCompletion(ewbik.asj.SaveManager saveManager) {
        // TODO Auto-generated method stub

    }

    @Override
    public void notifyOfLoadCompletion() {
        this.setBoneHeight(boneHeight);
        for (Bone b : this.children) {
            b.attachToParent(this);
        }
        this.parentArmature.addToBoneList(this);
    }

    @Override
    public boolean isLoading() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void setLoading(boolean loading) {
        // TODO Auto-generated method stub

    }

    @Override
    public int compareTo(Bone i) {
        return this.ancestorCount - i.ancestorCount;
    }

    public String toString() {
        return this.getTag();
    }

    public enum frameType {
        GLOBAL, RELATIVE
    }
}
