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

package EWBIK;

import java.util.ArrayList;

public class IKBone3D implements Comparable<IKBone3D> {
    public static boolean draw_kusudamas = false;
    public IKSkeleton parent_armature;
    public IKKusudama constraints;
    public int ancestor_count = 0;
    protected String tag;
    protected IKQuaternion last_rotation;
    protected IKNode3D previous_orientation;
    protected IKNode3D local_node_3d;
    protected IKNode3D major_rotation_node_3d;
    protected float bone_height;
    protected IKBone3D parent;
    protected ArrayList<IKBone3D> children = new ArrayList<>();
    protected ArrayList<IKBone3D> free_children = new ArrayList<>();
    protected ArrayList<IKBone3D> effected_children = new ArrayList<>();
    protected IKPin3D pin = null;
    protected boolean orientation_lock = false;
    protected float stiffness_scalar = 0f;

    public IKBone3D() {
        this.last_rotation = new IKQuaternion();
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
    public IKBone3D(IKBone3D par, // parent bone
                    IKVector3 tipHeading, // the orienational heading of this bone (global vs relative coords specified in
                    // coordinateType)
                    IKVector3 rollHeading, // axial rotation heading of the bone (it's z-axis)
                    String inputTag, // some user specified name for the bone, if desired
                    float inputBoneHeight, // bone length
                    frameType coordinateType) {
        IKVector3 tipHeading1 = tipHeading;
        IKVector3 rollHeading1 = rollHeading;

        this.last_rotation = new IKQuaternion();
        if (par != null) {
            if (inputTag == null || inputTag == "") {
                this.tag = Integer.toString(System.identityHashCode(this));
            } else
                this.tag = inputTag;
            this.bone_height = inputBoneHeight;

            IKRay3D tipHeadingRay = new IKRay3D(par.getTip_(), tipHeading1);
            IKRay3D rollHeadingRay = new IKRay3D(par.getTip_(), rollHeading1);
            IKVector3 tempTip = tipHeading1.copy();
            tempTip.set(0, 0, 0);
            IKVector3 tempRoll = rollHeading1.copy();
            tempRoll.set(0, 0, 0);
            IKVector3 tempX = tempRoll.copy();
            tempX.set(0, 0, 0);

            if (coordinateType == IKBone3D.frameType.GLOBAL) {
                tempTip = tipHeadingRay.heading();
                tempRoll = rollHeadingRay.heading();
            } else if (coordinateType == IKBone3D.frameType.RELATIVE) {
                tempTip = par.localAxes().getGlobalOf(tipHeadingRay.heading());
                tempRoll = par.localAxes().getGlobalOf(rollHeadingRay.heading());
            } else {
                System.out.println("WOAH WOAH WOAH");
            }

            tempX = tempTip.crossCopy(tempRoll);
            tempRoll = tempX.crossCopy(tempTip);

            tempX.normalize();
            tempTip.normalize();
            tempRoll.normalize();

            this.parent = par;
            this.parent_armature = this.parent.parent_armature;
            IKBone3D.this.parent_armature.addToBoneList(this);

            IKBone3D.this.generateAxes(IKBone3D.this.parent.getTip_(), tempX, tempTip, tempRoll);
            IKBone3D.this.local_node_3d.setParent(IKBone3D.this.parent.local_node_3d);

            IKBone3D.this.previous_orientation = IKBone3D.this.local_node_3d.attachedCopy();

            IKBone3D.this.major_rotation_node_3d = IKBone3D.this.parent.localAxes().getGlobalCopy();
            IKBone3D.this.major_rotation_node_3d.translateTo(IKBone3D.this.parent.getTip_());
            IKBone3D.this.major_rotation_node_3d.setParent(IKBone3D.this.parent.local_node_3d);

            this.parent.addFreeChild(this);
            this.parent.addChild(this);
            this.updateAncestorCount();
        } else {
           this.parent_armature = null;
           this.parent = null;
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
    public IKBone3D(
            IKSkeleton parArma,
            IKVector3 tipHeading,
            IKVector3 rollHeading,
            String inputTag,
            float inputBoneHeight,
            frameType coordinateType) {

        this.last_rotation = new IKQuaternion();
        if (parArma != null) {
            if (inputTag == null || inputTag == "") {
                this.tag = Integer.toString(System.identityHashCode(this));
            } else
                this.tag = inputTag;

            IKRay3D tipHeadingRay;
            tipHeadingRay = new IKRay3D(parArma.localNode3D.calculatePosition(), tipHeading);
            tipHeadingRay.getRayScaledTo(inputBoneHeight);
            IKRay3D rollHeadingRay = new IKRay3D(parArma.localNode3D.calculatePosition(), rollHeading);
            IKVector3 tempTip = tipHeading.copy();
            IKVector3 tempRoll = rollHeading.copy();
            IKVector3 tempX = tempTip.copy();

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
            tempRoll = tempX.crossCopy(tempTip);

            tempX.normalize();
            tempTip.normalize();
            tempRoll.normalize();

            this.parent_armature = parArma;
            parent_armature.addToBoneList(this);

            generateAxes(parent_armature.localNode3D.calculatePosition(), tempX, tempTip, tempRoll);
            local_node_3d.setParent(parent_armature.localNode3D);
            previous_orientation = local_node_3d.attachedCopy();

            major_rotation_node_3d = parent_armature.localAxes().getGlobalCopy();
            major_rotation_node_3d.setParent(parent_armature.localAxes());

            this.bone_height = inputBoneHeight;
            this.updateAncestorCount();
        } else {
            parent_armature = null;
            parent = null;
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
    public IKBone3D(IKBone3D par, // parent bone
                    float xAngle, // how much the bone should be pitched relative to its parent bone
                    float yAngle, // how much the bone should be rolled relative to its parent bone
                    float zAngle, // how much the bone should be yawed relative to its parent bone
                    String inputTag, // some user specified name for the bone, if desired
                    float inputBoneHeight // bone length
    ) {

        this.last_rotation = new IKQuaternion();
        if (par != null) {
            if (inputTag == null || inputTag == "") {
                this.tag = Integer.toString(System.identityHashCode(this));
            } else {
                this.tag = inputTag;
            }
            this.bone_height = inputBoneHeight;

            IKNode3D tempNode3D = par.localAxes().getGlobalCopy();
            IKQuaternion newRot = new IKQuaternion(IKRotationOrder.XZY, xAngle, yAngle, zAngle);
            tempNode3D.rotateBy(newRot);

            this.parent = par;
            this.parent_armature = this.parent.parent_armature;
            IKBone3D.this.parent_armature.addToBoneList(this);

            IKBone3D.this.generateAxes(IKBone3D.this.parent.getTip_(), tempNode3D.calculateX().heading(),
                    tempNode3D.calculateY().heading(),
                    tempNode3D.calculateZ().heading());
            IKBone3D.this.local_node_3d.setParent(IKBone3D.this.parent.local_node_3d);
            IKBone3D.this.previous_orientation = IKBone3D.this.local_node_3d.attachedCopy();

            IKBone3D.this.major_rotation_node_3d = IKBone3D.this.parent.localAxes().getGlobalCopy();
            IKBone3D.this.major_rotation_node_3d.translateTo(IKBone3D.this.parent.getTip_());
            IKBone3D.this.major_rotation_node_3d.setParent(IKBone3D.this.parent.local_node_3d);

            this.parent.addFreeChild(this);
            this.parent.addChild(this);
            this.updateAncestorCount();
        }
    }

    public IKBone3D(IKBone3D par, // parent bone
                    String inputTag, // some user specified name for the bone, if desired
                    float inputBoneHeight // bone length
    ) {
        this.last_rotation = new IKQuaternion();
        if (par != null) {
            if (inputTag == null || inputTag == "") {
                this.tag = Integer.toString(System.identityHashCode(this));
            } else {
                this.tag = inputTag;
            }
            this.bone_height = inputBoneHeight;

            IKNode3D tempNode3D = par.localAxes().getGlobalCopy();
            IKQuaternion newRot = new IKQuaternion();
            tempNode3D.rotateBy(newRot);

            this.parent = par;
            this.parent_armature = this.parent.parent_armature;
            IKBone3D.this.parent_armature.addToBoneList(this);

            IKBone3D.this.generateAxes(IKBone3D.this.parent.getTip_(), tempNode3D.calculateX().heading(),
                    tempNode3D.calculateY().heading(),
                    tempNode3D.calculateZ().heading());
            IKBone3D.this.local_node_3d.setParent(IKBone3D.this.parent.local_node_3d);
            IKBone3D.this.previous_orientation = IKBone3D.this.local_node_3d.attachedCopy();

            IKBone3D.this.major_rotation_node_3d = IKBone3D.this.parent.localAxes().getGlobalCopy();
            IKBone3D.this.major_rotation_node_3d.translateTo(IKBone3D.this.parent.getTip_());
            IKBone3D.this.major_rotation_node_3d.setParent(IKBone3D.this.parent.local_node_3d);

            this.parent.addFreeChild(this);
            this.parent.addChild(this);
            this.updateAncestorCount();
        }
    }

    public static void setDraw_kusudamas(boolean draw) {
        draw_kusudamas = draw;
    }

    protected void generateAxes(IKVector3 origin, IKVector3 x, IKVector3 y, IKVector3 z) {
        this.local_node_3d = new IKNode3D(origin, x, y, z);
    }

    public IKVector3 getBase() {
        return getBase_();
    }

    public IKVector3 getTip() {
        return  getTip_();
    }

    protected IKPin3D createAndReturnPinOnAxes(IKNode3D on) {
        return new IKPin3D(
                on,
                true,
                this);
    }

    public void enablePin(IKVector3 pin) {
        enablePin_(pin);
    }

    public void setPin(IKVector3 pin) {
        setPin_(pin);
    }

    /**
     * @return In the case of this out-of-the-box class, getPin() returns a IKVector
     *         indicating
     *         the spatial target of the pin.
     */
    public IKVector3 getPinLocation() {
        if (pin == null)
            return null;
        else {
            IKVector3 loc = pin.getLocation_();
            return loc;
        }
    }

    /*
    public void drawMeAndChildren(PGraphics pg, int boneCol, float pinSize) {

        if (this.constraints != null && drawKusudamas) {
            pg.pushMatrix();
            constraints.drawMe(pg, boneCol, pinSize);
            pg.popMatrix();
        }

        PMatrix localMat = localAxes().getLocalPMatrix();
        pg.applyMatrix(localMat);

        pg.beginShape(PConstants.TRIANGLE_FAN);
        pg.fill(pg.color(0, 255 - boneCol, boneCol));
        float circumference = boneHeight / 8f;
        pg.noStroke();
        pg.vertex(0, boneHeight, 0);
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
            this.getIKPin().getAxes().drawMe(pg, pinSize);
        }

        if (this.isPinned()) {
            pg.strokeWeight(2f);
            localAxes().drawMe(pg, pinSize);
        }
    }
    */

    public IKPin3D getIKPin() {
        return this.pin;
    }

    /**
     * Get the Axes associated with this bone.
     */
    public IKNode3D localAxes() {
        return this.local_node_3d;
    }

    /**
     * Get the Axes relative to which this bone's rotations are defined. (If the
     * bone has constraints, this will be the
     * constraint Axes)
     *
     * @return
     */
    @SuppressWarnings("unchecked")
    public IKNode3D getMajorRotationAxes() {
        return this.major_rotation_node_3d;
    }

    @SuppressWarnings("unchecked")
    public ArrayList<IKBone3D> getChildren() {
        return children;
    }

    public void setChildren(ArrayList<IKBone3D> children) {
        this.children = (ArrayList<IKBone3D>) children;
    }

    private void updateAncestorCount() {
        int countedAncestors = 0;
        IKBone3D currentBone = this.parent;
        while (currentBone != null) {
            countedAncestors++;
            currentBone = currentBone.parent;
        }
        setAncestor_count(countedAncestors);
    }

    /**
     * updates the ancestor count for this bone, and
     * sets the ancestor count of all child bones
     * to this bone's ancestor count +1;
     *
     * @param count
     */
    private void setAncestor_count(int count) {
        this.ancestor_count = count;
        for (IKBone3D b : this.children) {
            b.setAncestor_count(this.ancestor_count + 1);
        }
    }

    public IKBone3D getParent() {
        return this.parent;
    }

    public void attachToParent(IKBone3D inputParent) {
        inputParent.addChild(this);
        this.parent = inputParent;
        this.updateAncestorCount();
    }

    public void solveIKFromHere() {
        this.parent_armature.IKSolver(this);
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
    public void setAxesToSnapped(IKNode3D toSet,
                                 IKNode3D limitingNode3D, float cosHalfAngleDampen) {
        if (constraints != null && IKKusudama.class.isAssignableFrom(constraints.getClass())) {
            constraints.setAxesToSnapped(toSet, limitingNode3D, cosHalfAngleDampen);
        }
    }

    public void setAxesToReturnfulled(IKNode3D toSet,
                                      IKNode3D limitingNode3D, float cosHalfAngleDampen,
                                      float angleDampen) {
        if (constraints != null && IKKusudama.class.isAssignableFrom(constraints.getClass())) {
            constraints.setAxesToReturnfulled(toSet, limitingNode3D, cosHalfAngleDampen,
                    angleDampen);
        }
    }

    public void setPin_(IKVector3 pin) {
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
    public IKKusudama addConstraint(IKKusudama newConstraint) {
        constraints = newConstraint;
        return constraints;
    }

    /**
     * @return this bone's constraint object.
     */
    public IKKusudama getConstraint() {
        return constraints;
    }

    /**
     * @return an array where each element indicated how much this bone is rotated
     *         on the X,Y,Z (in that order)
     *         axes relative to its constraint Axes.
     *         If there are no constraint Axes, the result is relative to the parent
     *         bone.
     *         If the bone has no parent, this method throws an exception.
     * @throws NullParentForBoneException
     */
    public float[] getXYZAngle() {
        if (this.parent != null) {
            this.localAxes().markDirty();
            this.localAxes().updateGlobal();
            IKNode3D result = this.localAxes().getGlobalCopy();
            this.getMajorRotationAxes().updateGlobal();
            this.getMajorRotationAxes().globalMBasis.setToLocalOf(result.globalMBasis, result.globalMBasis);
            return result.globalMBasis.rotation.getAngles(IKRotationOrder.XYZ);
        } else {
            IKNode3D empty = new IKNode3D();
            return empty.globalMBasis.rotation.getAngles(IKRotationOrder.XYZ);
        }
    }

    /**
     * @return An Apache Commons Rotation object representing the rotation of this
     *         bone relative to its
     *         reference frame. The Rotation object is more versatile and robust
     *         than an array of angles.
     *         And allows you to treat rotation in a wide variety of conventions.
     */
    public IKQuaternion getRotation() {
        return (new IKQuaternion(this.major_rotation_node_3d.calculateX().heading(),
                this.major_rotation_node_3d.calculateY().heading(),
                this.localAxes().calculateX().heading(), this.localAxes().calculateY().heading()));
    }

    /**
     * @return An Apache Commons Rotation object representing the rotation which
     *         transforms this
     *         BoneExample from its previous orientation to its current orientation.
     */

    public IKQuaternion getRotationFromPrevious() {
        return last_rotation;
    }

    /**
     * @return the reference frame representing this bone's previous orientation
     *         relative to
     *         its parent.
     */
    public IKNode3D getPrevious_orientation() {
        return previous_orientation;
    }

    /**
     * Rotate the bone about its frame of reference by a custom Apache Commons
     * Rotation object
     *
     * @param rot
     */
    public void rotateBy(IKQuaternion rot) {
        this.previous_orientation.alignLocalsTo(local_node_3d);
        this.local_node_3d.rotateBy(rot);

        this.last_rotation.set(rot);
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
        previous_orientation.alignLocalsTo(local_node_3d);

        IKQuaternion xRot = new IKQuaternion(major_rotation_node_3d.calculateX().heading(), amt);
        local_node_3d.rotateBy(xRot);

        last_rotation.set(xRot);

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
        previous_orientation.alignLocalsTo(local_node_3d);

        IKQuaternion yRot = new IKQuaternion(major_rotation_node_3d.calculateY().heading(), amt);
        local_node_3d.rotateBy(yRot);

        last_rotation.set(yRot);

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
        previous_orientation.alignLocalsTo(local_node_3d);

        IKQuaternion zRot = new IKQuaternion(major_rotation_node_3d.calculateZ().heading(), amt);
        local_node_3d.rotateBy(zRot);

        last_rotation.set(zRot);

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
        local_node_3d.rotateAboutY(amt, true);

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
     *                                 and the limit cones and axial twist limits of
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
    public void setFrameofRotation(IKNode3D rotationFrameCoordinates) {
        major_rotation_node_3d.alignLocalsTo(rotationFrameCoordinates);
        if (parent != null) {
            major_rotation_node_3d.translateTo(parent.getTip_());
        }
    }

    /**
     * Disables the pin for this bone so that it no longer interests the IK Solver.
     * However, all information abut the pin is maintained, so the pin can be turned
     * on again with enablePin().
     */
    public void disablePin() {
        pin.disable();
        if (this.effected_children.size() == 0) {
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
        if (this.effected_children.size() == 0) {
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
            IKNode3D pinNode3D = this.localAxes().getGlobalCopy();
            pinNode3D.setParent(this.parent_armature.localAxes().getParentAxes());
            pin = createAndReturnPinOnAxes(pinNode3D);
        }
        pin.enable();
        free_children.clear();
        for (IKBone3D child : getChildren()) {
            if (child.pin != null && !child.pin.isEnabled()) {
                addFreeChild(child);
            }
        }
        notifyAncestorsOfPin(false);
        this.updateSegmentedArmature();
    }

    /**
     * Creates a pin for this bone
     *
     * @param pinTo the position of the pin in the coordinateFrame of the
     *              parentArmature.
     */

    public void enablePin_(IKVector3 pinTo) {
        if (pin == null) {
            IKNode3D pinNode3D = this.localAxes().getGlobalCopy();
            pinNode3D.setParent(this.parent_armature.localAxes().getParentAxes());
            pinNode3D.translateTo(pinTo);
            pin = createAndReturnPinOnAxes(pinNode3D);
        } else
            pin.translateTo_(pinTo);
        pin.enable();
        free_children.clear();
        for (IKBone3D child : getChildren()) {
            if (!child.pin.isEnabled()) {
                addFreeChild(child);
            }
        }
        notifyAncestorsOfPin();
    }

    /**
     * @return true if the bone has a pin enabled, false otherwise.
     */
    public boolean isPinned() {
        return pin != null && pin.isEnabled();

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

    public ArrayList<IKBone3D> returnChildrenWithPinnedDescendants() {
        ArrayList<IKBone3D> childrenWithPinned = new ArrayList<IKBone3D>();
        for (IKBone3D c : getChildren()) {
            if (c.hasPinnedDescendant())
                childrenWithPinned.add(c);
        }
        return childrenWithPinned;
    }

    public ArrayList<IKBone3D> getMostImmediatelyPinnedDescendants() {
        ArrayList<IKBone3D> mostImmediatePinnedDescendants = new ArrayList<IKBone3D>();
        this.addSelfIfPinned(mostImmediatePinnedDescendants);
        return mostImmediatePinnedDescendants;
    }

    public IKNode3D getPinnedAxes() {
        if (this.pin == null)
            return null;
        return this.pin.getAxes();
    }

    public void addSelfIfPinned(ArrayList<IKBone3D> pinnedBones2) {
        if (this.isPinned()) {
            pinnedBones2.add(this);
        } else {
            for (IKBone3D child : getChildren()) {
                child.addSelfIfPinned(pinnedBones2);
            }
        }
    }

    public void notifyAncestorsOfPin(boolean updateSegments) {
        if (this.parent != null) {
            parent.addToEffectored(this);
        }
        if (updateSegments)
            parent_armature.updateBonechains();
    }

    public void notifyAncestorsOfPin() {
        notifyAncestorsOfPin(true);
    }

    public void notifyAncestorsOfUnpin() {
        if (this.parent != null) {
            parent.removeFromEffectored(this);
        }
        parent_armature.updateBonechains();
    }

    public void addToEffectored(IKBone3D Bone) {
        int freeIndex = free_children.indexOf(Bone);
        if (freeIndex != -1)
            free_children.remove(freeIndex);

        if (effected_children.contains(Bone)) {
        } else {
            effected_children.add(Bone);
        }
        if (this.parent != null) {
            parent.addToEffectored(this);
        }
    }

    public void removeFromEffectored(IKBone3D Bone) {
        int effectoredIndex = effected_children.indexOf(Bone);
        if (effectoredIndex != -1)
            effected_children.remove(effectoredIndex);

        if (free_children.contains(Bone)) {
        } else {
            addFreeChild(Bone);
        }
        if (this.parent != null && this.effected_children.size() == 0 && this.pin != null && this.pin.isEnabled()) {
            parent.removeFromEffectored(this);
        }
    }

    public IKBone3D getPinnedRootBone() {
        IKBone3D rootBone = this;
        while (rootBone.parent != null && !rootBone.parent.pin.isEnabled()) {
            rootBone = rootBone.parent;
        }
        return rootBone;
    }

    public void updateSegmentedArmature() {
        this.parent_armature.updateBonechains();
    }

    public String getTag() {
        return this.tag;
    }

    public void setTag(String newTag) {
        parent_armature.setBoneName(this, this.tag, newTag);
        this.tag = newTag;
    }

    public IKVector3 getBase_() {
        return local_node_3d.calculatePosition().copy();
    }

    public IKVector3 getTip_() {
        return local_node_3d.calculateY().getScaledTo(bone_height);
    }

    /**
     * removes this BoneExample and any of its children from the armature.
     */
    public void deleteBone() {
        ArrayList<IKBone3D> bones = new ArrayList<>();
        IKBone3D root = parent_armature.getRootBone();
        root.hasChild(bones, this);
        for (IKBone3D p : bones) {
            System.out.println("removing from" + p);
            p.removeFromEffectored(this);
            for (IKBone3D ab : this.effected_children) {
                p.removeFromEffectored(ab);
            }
            p.getChildren().remove(this);
            p.free_children.remove(this);
        }
        this.parent_armature.removeFromBoneList(this);
    }

    /* adds this bone to the arrayList if inputBone is among its children */
    private void hasChild(ArrayList<IKBone3D> list, IKBone3D query) {
        if (getChildren().contains(query))
            list.add(this);
        for (IKBone3D c : getChildren()) {
            c.hasChild(list, query);
        }
    }

    public float getBone_height() {
        return this.bone_height;
    }

    public void setBone_height(float inBoneHeight) {
        this.bone_height = inBoneHeight;
        for (IKBone3D child : this.getChildren()) {
            child.localAxes().translateTo(this.getTip_());
            child.major_rotation_node_3d.translateTo(this.getTip_());
        }
    }

    public boolean hasPinnedDescendant() {
        if (this.isPinned())
            return true;
        else {
            boolean result = false;
            for (IKBone3D c : getChildren()) {
                if (c.hasPinnedDescendant()) {
                    result = true;
                    break;
                }
            }
            return result;
        }

    }

    public boolean getIKOrientationLock() {
        return this.orientation_lock;
    }

    /**
     * if set to true, the IK system will not rotate this bone
     * as it solves the IK chain.
     *
     * @param val
     */
    public void setIKOrientationLock(boolean val) {
        this.orientation_lock = val;
    }

    public void addChild(IKBone3D bone) {
        if (this.getChildren().indexOf(bone) == -1) {
            getChildren().add(bone);
        }
    }

    public void addFreeChild(IKBone3D bone) {
        if (this.free_children.indexOf(bone) == -1) {
            free_children.add(bone);
        }
        parent_armature.updateBonechains();
    }

    public void addEffectoredChild(IKBone3D bone) {
        if (this.effected_children.indexOf(bone) == -1) {
            this.effected_children.add(bone);
        }

    }

    public void addDescendantsToArmature() {
        for (IKBone3D b : getChildren()) {
            parent_armature.addToBoneList(b);
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
        return stiffness_scalar;
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
        stiffness_scalar = stiffness;
        if (parent_armature != null) {
            IKShadowNode s = parent_armature.boneSegmentMap.get(this);
            if (s != null) {
                IKShadowNode.ShadowBone wb = s.simulatedBones.get(this);
                if (wb != null) {
                    wb.updateCosDampening();
                }
            }
        }
    }

    @Override
    public int compareTo(IKBone3D i) {
        return this.ancestor_count - i.ancestor_count;
    }

    public String toString() {
        return this.getTag();
    }

    public enum frameType {
        GLOBAL, RELATIVE
    }
}
