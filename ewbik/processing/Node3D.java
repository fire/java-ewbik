package processing;/*
                   
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

import ewbik.math.*;
import processing.core.PGraphics;
import processing.core.PMatrix;
import processing.core.PMatrix3D;
import processing.core.PVector;

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.function.Consumer;

public class Node3D implements ewbik.asj.Saveable {
    public static final int NORMAL = 0;
    public static final int FORWARD = 2;
    public static final int RIGHT = 1;
    public static final int LEFT = -1;
    public static final int X = 0;
    public static final int Y = 1;
    public static final int Z = 2;
    public boolean debug = false;
    public Transform3D localMBasis;
    public Transform3D globalMBasis;
    public boolean dirty = true;
    public LinkedList<DependencyReference<Node3D>> dependentsRegistry = new LinkedList<DependencyReference<Node3D>>();
    protected Vector3 workingVector;
    protected boolean areGlobal = true;
    float[][] outMatLocal = new float[4][4];
    float[][] outMatGlobal = new float[4][4];
    Vector3 tempOrigin;
    private DependencyReference<Node3D> parent = null;

    public Node3D(ewbik.math.Transform3D b, Node3D parent) {
        this.globalMBasis = ((Transform3D) b).copy();
        createTempVars(((Transform3D) b).getOrigin());
        if (this.getParentAxes() != null)
            Node3D.this.setParent(parent);
        else {
            this.areGlobal = true;
            this.localMBasis = ((Transform3D) b).copy();
        }

        this.updateGlobal();
    }

    /**
     * @param origin the center of this axes basis. The basis vector
     *               parameters will be automatically ADDED to the
     *               origin in order to create this basis vector.
     * @param inX    the direction of the X basis vector in global
     *               coordinates, given as an offset from this base's
     *               origin in global coordinates.
     * @param inY    the direction of the Y basis vector in global
     *               coordinates, given as an offset from this base's
     *               origin in global coordinates.
     * @param inZ    the direction of the Z basis vector in global
     *               coordinates, given as an offset from this base's
     *               origin in global coordinates.
     */
    public Node3D(PVector origin,
            PVector inX,
            PVector inY,
            PVector inZ,
            Node3D parent) {

        Vector3 origin1 = toVec3f(origin);
        Vector3 inX1 = toVec3f(inX);
        Vector3 inY1 = toVec3f(inY);
        Vector3 inZ1 = toVec3f(inZ);
        if (parent == null)
            this.areGlobal = true;
        createTempVars(origin1);

        Node3D.this.areGlobal = true;

        Node3D.this.localMBasis = new Transform3D(origin1, inX1, inY1, inZ1);
        Node3D.this.globalMBasis = new Transform3D(origin1, inX1, inY1, inZ1);

        Vector3 o = origin1.copy();
        o.set(0, 0, 0);
        Vector3 i = o.copy();
        i.set(1, 1, 1);

        if (parent != null) {
            this.setParent(parent);
        } else {
            this.areGlobal = true;
        }
        this.markDirty();
        this.updateGlobal();
    }

    public Node3D(Vector3 origin,
            Vector3 x,
            Vector3 y,
            Vector3 z) {
        this(origin, x, y, z, true, null);
    }

    public Node3D() {
        Vector3 origin = new Vector3(0, 0, 0);
        Vector3 inX = new Vector3(1, 0, 0);
        Vector3 inY = new Vector3(0, 1, 0);
        Vector3 inZ = new Vector3(0, 0, 1);
        if (null == null)
            this.areGlobal = true;
        createTempVars(origin);

        Node3D.this.areGlobal = true;

        Node3D.this.localMBasis = new Transform3D(origin, inX, inY, inZ);
        Node3D.this.globalMBasis = new Transform3D(origin, inX, inY, inZ);

        Vector3 o = origin.copy();
        o.set(0, 0, 0);
        Vector3 i = o.copy();
        i.set(1, 1, 1);

        if (null != null) {
            this.setParent(null);
        } else {
            this.areGlobal = true;
        }
        this.markDirty();
        this.updateGlobal();
    }

    public Node3D(Vector3 origin, Vector3 x, Vector3 y, Vector3 z, boolean forceOrthoNormality,
            Node3D parent) {
        if (parent == null)
            this.areGlobal = true;
        createTempVars(origin);

        Node3D.this.areGlobal = true;

        Node3D.this.localMBasis = new Transform3D(origin, x, y, z);
        Node3D.this.globalMBasis = new Transform3D(origin, x, y, z);

        Vector3 o = origin.copy();
        o.set(0, 0, 0);
        Vector3 i = o.copy();
        i.set(1, 1, 1);

        if (parent != null) {
            this.setParent(parent);
        } else {
            this.areGlobal = true;
        }
        this.markDirty();
        this.updateGlobal();
    }

    public static PVector toPVector(Vector3 sv) {
        return new PVector(sv.x, sv.y, sv.z);
    }

    public static void toDVector(Vector3 sv, PVector storeIn) {
        storeIn.x = sv.x;
        storeIn.y = sv.y;
        storeIn.z = sv.z;
    }

    public static Vector3 toVec3f(PVector ev) {
        return new Vector3(ev.x, ev.y, ev.z);
    }

    public static void drawRay(PGraphics p, Ray3D r) {
        p.line(r.p1().x, r.p1().y, r.p1().z, r.p2().x, r.p2().y, r.p2().z);
    }

    public static void drawPoint(PGraphics p, Vector3 pt) {
        p.point(pt.x, pt.y, pt.z);
    }

    public PVector origin() {
        return toPVector(this.calculatePosition());
    }

    /**
     * Make a GlobalCopy of these Axes.
     *
     * @return
     */
    public Node3D getGlobalCopy() {
        this.updateGlobal();
        return new Node3D(getGlobalMBasis(), this.getParentAxes());
    }

    public PVector getGlobalOf(PVector local_input) {
        return toPVector(
                getGlobalOf(
                        toVec3f(local_input)));
    }

    public PVector setToGlobalOf(PVector local_input) {
        return toPVector(
                setToGlobalOf(
                        toVec3f(local_input)));
    }

    public void setToGlobalOf(PVector local_input, PVector global_output) {
        toDVector(
                setToGlobalOf(
                        toVec3f(local_input)),
                global_output);
    }

    public void translateByLocal(PVector translate) {
        translateByLocal(
                toVec3f(translate));
    }

    public void translateByGlobal(PVector translate) {
        translateByGlobal(
                toVec3f(translate));
    }

    public void translateTo(PVector translate, boolean slip) {
        translateTo(
                toVec3f(translate),
                false);

    }

    public void translateTo(PVector translate) {
        translateTo(
                toVec3f(translate));
    }

    public void rotateAboutX(float radians) {
        rotateAboutX(radians, true);
    }

    public void rotateAboutY(float radians) {
        rotateAboutY(radians, true);
    }

    public void rotateAboutZ(float radians) {
        rotateAboutZ(radians, true);
    }

    public PVector calculatePositionPVector() {
        return toPVector(calculatePosition());
    }

    public PVector getLocalOf(PVector global_input) {
        return getLocalOf(global_input);
    }

    public PVector setToLocalOf(PVector global_input) {
        toDVector(
                setToLocalOf(
                        toVec3f(global_input)),
                global_input);
        return global_input;
    }

    public void setToLocalOf(PVector global_input, PVector local_output) {
        Vector3 tempVec = new Vector3();
        setToLocalOf(
                toVec3f(global_input),
                tempVec);
        toDVector(
                tempVec,
                local_output);
    }

    private void updateMatrix(Transform3D b, float[][] outputMatrix) {
        b.refreshPrecomputed();

        Vector3 x = b.getXHeading();
        Vector3 y = b.getYHeading();
        Vector3 z = b.getZHeading();

        Vector3 origin = b.getOrigin();

        outputMatrix[0][0] = x.x;
        outputMatrix[0][1] = x.y;
        outputMatrix[0][2] = x.z;

        outputMatrix[1][0] = y.x;
        outputMatrix[1][1] = y.y;
        outputMatrix[1][2] = y.z;

        outputMatrix[2][0] = z.x;
        outputMatrix[2][1] = z.y;
        outputMatrix[2][2] = z.z;

        outputMatrix[3][3] = 1;

        outputMatrix[3][0] = origin.x;
        outputMatrix[3][1] = origin.y;
        outputMatrix[3][2] = origin.z;

    }

    public PMatrix getLocalPMatrix() {
        updateMatrix(getLocalMBasis(), outMatLocal);
        float[][] m = outMatLocal;
        PMatrix result = new PMatrix3D(
                m[0][0], m[1][0], m[2][0], m[3][0],
                m[0][1], m[1][1], m[2][1], m[3][1],
                m[0][2], m[1][2], m[2][2], m[3][2],
                m[0][3], m[1][3], m[2][3], m[3][3]);
        return result;
    }

    public PMatrix getGlobalPMatrix() {
        this.updateGlobal();
        updateMatrix(getGlobalMBasis(), outMatGlobal);
        float[][] m = outMatGlobal;
        PMatrix result = new PMatrix3D(
                m[0][0], m[1][0], m[2][0], m[3][0],
                m[0][1], m[1][1], m[2][1], m[3][1],
                m[0][2], m[1][2], m[2][2], m[3][2],
                m[0][3], m[1][3], m[2][3], m[3][3]);
        return result;
    }

    public void drawMe(PGraphics pg, float size) {
        pg.noStroke();
        updateGlobal();
        pg.pushMatrix();
        pg.setMatrix(getGlobalPMatrix());
        pg.fill(2, 58, 0);
        pg.pushMatrix();
        pg.translate(size / 2f, 0, 0);
        pg.box(size, size / 10f, size / 10f);
        pg.popMatrix();
        drawRay(pg, calculateX().getRayScaledTo(size));
        pg.fill(94, 0, 0);
        pg.pushMatrix();
        pg.translate(0, size / 2f, 0);
        pg.box(size / 10f, size, size / 10f);
        pg.popMatrix();
        pg.fill(0, 26, 150);
        pg.pushMatrix();
        pg.translate(0, 0, size / 2f);
        pg.box(size / 10f, size / 10f, size);
        pg.popMatrix();
        pg.popMatrix();
    }

    /**
     * return a ray / segment representing this Axes global x basis position and
     * direction and magnitude
     *
     * @return a ray / segment representing this Axes global x basis position and
     *         direction and magnitude
     */
    public Ray3D calculateX() {
        this.updateGlobal();
        return this.getGlobalMBasis().getXRay();
    }

    /**
     * return a ray / segment representing this Axes global y basis position and
     * direction and magnitude
     *
     * @return a ray / segment representing this Axes global y basis position and
     *         direction and magnitude
     */
    public Ray3D calculateY() {
        this.updateGlobal();
        return this.getGlobalMBasis().getYRay();
    }

    /**
     * return a ray / segment representing this Axes global z basis position and
     * direction and magnitude
     *
     * @return a ray / segment representing this Axes global z basis position and
     *         direction and magnitude
     */
    public Ray3D calculateZ() {
        this.updateGlobal();
        return this.getGlobalMBasis().getZRay();
    }

    public boolean equals(Node3D ax) {
        this.updateGlobal();
        ax.updateGlobal();

        boolean composedRotationsAreEquivalent = getGlobalMBasis().rotation.equals(ax.globalMBasis.rotation);
        boolean originsAreEquivalent = getGlobalMBasis().getOrigin().equals(ax.calculatePosition());

        return composedRotationsAreEquivalent && originsAreEquivalent;
    }

    public Node3D relativeTo(Node3D in) {
        return null;
    }

    public Node3D getLocalOf(Node3D input) {
        return null;
    }

    /**
     * Creates an exact copy of this Axes object. Attached to the same parent as
     * this Axes object
     *
     * @param slipAware
     * @return
     */
    public Node3D attachedCopy() {
        this.updateGlobal();
        Node3D copy = new Node3D(getGlobalMBasis(),
                this.getParentAxes());
        copy.getLocalMBasis().adoptValues(this.localMBasis);
        copy.markDirty();
        return copy;
    }

    public Transform3D getLocalOf(Transform3D input) {
        Transform3D newBasis = new Transform3D((Transform3D) input);
        getGlobalMBasis().setToLocalOf(input, newBasis);
        return (Transform3D) newBasis;
    }

    public void createTempVars(Vector3 type) {
        workingVector = type.copy();
        tempOrigin = type.copy();
    }

    public Node3D getParentAxes() {
        if (this.parent == null)
            return null;
        else
            return this.parent.get();
    }

    public void updateGlobal() {
        if (this.dirty) {
            if (this.areGlobal) {
                globalMBasis.adoptValues(this.localMBasis);
            } else {
                getParentAxes().updateGlobal();
                getParentAxes().getGlobalMBasis().setToGlobalOf(this.localMBasis, this.globalMBasis);
            }
        }
        dirty = false;
    }

    public void debugCall() {
    }

    public Vector3 calculatePosition() {
        this.updateGlobal();
        tempOrigin.set(this.getGlobalMBasis().getOrigin());
        return tempOrigin;
    }

    /**
     * Sets the parentAxes for this axis globally.
     * in other words, globalX, globalY, and globalZ remain unchanged, but lx, ly,
     * and lz
     * change.
     *
     * @param par the new parent Axes
     **/
    public void setParent(Node3D par) {
        setParent(par, null);
    }

    /**
     * Sets the parentAxes for this axis globally.
     * in other words, globalX, globalY, and globalZ remain unchanged, but lx, ly,
     * and lz
     * change.
     *
     * @param intendedParent the new parent Axes
     * @param requestedBy    the object making thisRequest, will be passed on to
     *                       parentChangeWarning
     *                       for any AxisDependancy objects registered with this
     *                       Axes (can be null if not important)
     **/
    public void setParent(Node3D intendedParent, Object requestedBy) {
        this.updateGlobal();
        Node3D oldParent = this.getParentAxes();
        /*
         * for(DependencyReference<AxisDependency> ad : this.dependentsRegistry) {
         * ad.get().parentChangeWarning(this, oldParent, intendedParent, requestedBy);
         * }
         */
        forEachDependent(
                (ad) -> ad.get().parentChangeWarning(this, oldParent, intendedParent, requestedBy));

        if (intendedParent != null && intendedParent != this) {
            intendedParent.updateGlobal();
            intendedParent.getGlobalMBasis().setToLocalOf(globalMBasis, localMBasis);

            if (oldParent != null)
                oldParent.disown(this);
            this.parent = new DependencyReference<Node3D>(intendedParent);

            this.getParentAxes().registerDependent(this);
            this.areGlobal = false;
        } else {
            if (oldParent != null)
                oldParent.disown(this);
            this.parent = new DependencyReference<Node3D>(null);
            this.areGlobal = true;
        }
        this.markDirty();
        this.updateGlobal();

        forEachDependent(
                (ad) -> ad.get().parentChangeCompletionNotice(this, oldParent, intendedParent, requestedBy));
        /*
         * for(DependencyReference<AxisDependency> ad : this.dependentsRegistry) {
         * ad.get().parentChangeCompletionNotice(this, oldParent, intendedParent,
         * requestedBy);
         * }
         */
    }

    /**
     * runs the given runnable on each dependent axis,
     * taking advantage of the call to remove entirely any
     * weakreferences to elements that have been cleaned up by the garbage
     * collector.
     *
     * @param r
     */
    public void forEachDependent(Consumer<DependencyReference<Node3D>> action) {
        Iterator<DependencyReference<Node3D>> i = dependentsRegistry.iterator();
        while (i.hasNext()) {
            DependencyReference<Node3D> dr = i.next();
            if (dr.get() != null) {
                action.accept(dr);
            } else {
                i.remove();
            }
        }
    }

    /**
     * Sets the parentAxes for this axis locally.
     * in other words, lx,ly,lz remain unchanged, but globalX, globalY, and globalZ
     * change.
     * <p>
     * if setting this parent would result in a dependency loop, then the input Axes
     * parent is set to this Axes' parent, prior to this axes setting the input axes
     * as its parent.
     **/
    public void setRelativeToParent(Node3D par) {
        if (this.getParentAxes() != null)
            this.getParentAxes().disown(this);
        this.parent = new DependencyReference<Node3D>(par);
        this.areGlobal = false;
        this.getParentAxes().registerDependent(this);
        this.markDirty();
    }

    public boolean needsUpdate() {
        return this.dirty;
    }

    /**
     * Given a vector in this axes local coordinates, returns the vector's position
     * in global coordinates.
     *
     * @param in
     * @return
     */
    public Vector3 getGlobalOf(Vector3 in) {
        Vector3 result = (Vector3) in.copy();
        setToGlobalOf(in, result);
        return result;
    }

    /**
     * Given a vector in this axes local coordinates, modifies the vector's values
     * to represent its position global coordinates.
     *
     * @param in
     * @return a reference to this the @param in object.
     */
    public Vector3 setToGlobalOf(Vector3 in) {
        this.updateGlobal();
        getGlobalMBasis().setToGlobalOf(in, in);
        return in;
    }

    /**
     * Given an input vector in this axes local coordinates, modifies the output
     * vector's values to represent the input's position in global coordinates.
     *
     * @param in
     */
    public void setToGlobalOf(Vector3 input, Vector3 output) {
        this.updateGlobal();
        getGlobalMBasis().setToGlobalOf(input, output);
    }

    /**
     * Given an input sgRay in this axes local coordinates, modifies the output
     * Rays's values to represent the input's in global coordinates.
     *
     * @param in
     */
    public void setToGlobalOf(Ray3D input, Ray3D output) {
        this.updateGlobal();
        this.setToGlobalOf(input.p1(), output.p1());
        this.setToGlobalOf(input.p2(), output.p2());
    }

    public Ray3D getGlobalOf(Ray3D in) {
        return new Ray3D(this.getGlobalOf(in.p1()), this.getGlobalOf(in.p2()));
    }

    public Vector3 getLocalOf(Vector3 in) {
        this.updateGlobal();
        return getGlobalMBasis().getLocalOf(in);
    }

    /**
     * Given a vector in global coordinates, modifies the vector's values to
     * represent its position in theseAxes local coordinates.
     *
     * @param in
     * @return a reference to the @param in object.
     */

    public Vector3 setToLocalOf(Vector3 in) {
        this.updateGlobal();
        Vector3 result = (Vector3) in.copy();
        this.getGlobalMBasis().setToLocalOf(in, result);
        in.set(result);
        return result;
    }

    /**
     * Given a vector in global coordinates, modifies the vector's values to
     * represent its position in theseAxes local coordinates.
     *
     * @param in
     */

    public void setToLocalOf(Vector3 in, Vector3 out) {
        this.updateGlobal();
        this.getGlobalMBasis().setToLocalOf(in, out);
    }

    /**
     * Given a sgRay in global coordinates, modifies the sgRay's values to represent
     * its position in theseAxes local coordinates.
     *
     * @param in
     */

    public void setToLocalOf(Ray3D in, Ray3D out) {
        this.setToLocalOf(in.p1(), out.p1());
        this.setToLocalOf(in.p2(), out.p2());
    }

    public void setToLocalOf(Transform3D input, Transform3D output) {
        this.updateGlobal();
        this.getGlobalMBasis().setToLocalOf(input, output);
    }

    public Ray3D getLocalOf(Ray3D in) {
        return new Ray3D(this.getLocalOf(in.p1()), this.getLocalOf(in.p2()));
    }

    public void translateByLocal(Vector3 translate) {
        this.updateGlobal();
        getLocalMBasis().translateBy(translate);
        this.markDirty();

    }

    public void translateByGlobal(Vector3 translate) {
        if (this.getParentAxes() != null) {
            this.updateGlobal();
            this.translateTo(translate.addCopy(this.calculatePosition()));
        } else {
            getLocalMBasis().translateBy(translate);
        }

        this.markDirty();
    }

    public void translateTo(Vector3 translate, boolean slip) {
        this.updateGlobal();
        if (slip) {
            Node3D tempNode3D = this.getGlobalCopy();
            tempNode3D.translateTo(translate);
            this.slipTo(tempNode3D);
        } else {
            this.translateTo(translate);
        }
    }

    public void translateTo(Vector3 translate) {
        if (this.getParentAxes() != null) {
            this.updateGlobal();
            getLocalMBasis().translateTo(getParentAxes().getGlobalMBasis().getLocalOf(translate));
            this.markDirty();
        } else {
            this.updateGlobal();
            getLocalMBasis().translateTo(translate);
            this.markDirty();
        }

    }

    public void rotateAboutX(float angle, boolean orthonormalized) {
        this.updateGlobal();
        Quaternion xRot = new Quaternion(getGlobalMBasis().getXHeading(), angle);
        this.rotateBy(xRot);
        this.markDirty();
    }

    public void rotateAboutY(float angle, boolean orthonormalized) {
        this.updateGlobal();
        Quaternion yRot = new Quaternion(getGlobalMBasis().getYHeading(), angle);
        this.rotateBy(yRot);
        this.markDirty();
    }

    public void rotateAboutZ(float angle, boolean orthonormalized) {
        this.updateGlobal();
        Quaternion zRot = new Quaternion(getGlobalMBasis().getZHeading(), angle);
        this.rotateBy(zRot);
        this.markDirty();
    }

    public void rotateBy(ewbik.math.Quaternion apply) {
        this.updateGlobal();
        if (parent != null) {
            Quaternion newRot = this.getParentAxes().getGlobalMBasis().getLocalOfRotation(apply);
            this.getLocalMBasis().rotateBy(newRot);
        } else {
            this.getLocalMBasis().rotateBy(apply);
        }
        this.markDirty();
    }

    /**
     * rotates the bases around their origin in Local coordinates
     *
     * @param rotation
     */
    public void rotateByLocal(Quaternion apply) {
        this.updateGlobal();
        if (parent != null) {
            this.getLocalMBasis().rotateBy(apply);
        }
        this.markDirty();
    }

    /**
     * sets these axes to have the same orientation and location relative to their
     * parent
     * axes as the input's axes do to the input's parent axes.
     * <p>
     * If the axes on which this function is called are orthonormal,
     * this function normalizes and orthogonalizes them regardless of whether the
     * targetAxes are orthonormal.
     *
     * @param targetNode3D the Axes to make this Axis identical to
     */
    public void alignLocalsTo(Node3D targetNode3D) {
        this.getLocalMBasis().adoptValues(targetNode3D.localMBasis);
        this.markDirty();
    }

    /**
     * sets the bases to the Identity basis and Identity rotation relative to its
     * parent, and translates
     * its origin to the parent's origin.
     * <p>
     * be careful calling this method, as it destroys any shear / scale information.
     */
    public void alignToParent() {
        this.getLocalMBasis().setIdentity();
        this.markDirty();
    }

    /**
     * rotates and translates the axes back to its parent, but maintains
     * its shear, translate and scale attributes.
     */
    public void rotateToParent() {
        this.getLocalMBasis().rotateTo(new Quaternion());
        this.markDirty();
    }

    /**
     * sets these axes to have the same global orientation as the input Axes.
     * these Axes lx, ly, and lz headings will differ from the target ages,
     * but its gx, gy, and gz headings should be identical unless this
     * axis is orthonormalized and the target axes are not.
     *
     * @param targetNode3D
     */
    public void alignGlobalsTo(Node3D targetNode3D) {
        targetNode3D.updateGlobal();
        this.updateGlobal();
        if (this.getParentAxes() != null) {
            getParentAxes().getGlobalMBasis().setToLocalOf(targetNode3D.globalMBasis, localMBasis);
        } else {
            this.getLocalMBasis().adoptValues(targetNode3D.globalMBasis);
        }
        this.markDirty();
        this.updateGlobal();
    }

    public void alignOrientationTo(Node3D targetNode3D) {
        targetNode3D.updateGlobal();
        this.updateGlobal();
        if (this.getParentAxes() != null) {
            this.getGlobalMBasis().rotateTo(targetNode3D.getGlobalMBasis().rotation);
            getParentAxes().getGlobalMBasis().setToLocalOf(this.globalMBasis, this.localMBasis);
        } else {
            this.getLocalMBasis().rotateTo(targetNode3D.getGlobalMBasis().rotation);
        }
        this.markDirty();
    }

    /**
     * updates the axes object such that its global orientation
     * matches the given Quaternion object.
     *
     * @param rotation
     */
    public void setGlobalOrientationTo(Quaternion rotation) {
        this.updateGlobal();
        if (this.getParentAxes() != null) {
            this.getGlobalMBasis().rotateTo(rotation);
            getParentAxes().getGlobalMBasis().setToLocalOf(this.globalMBasis, this.localMBasis);
        } else {
            this.getLocalMBasis().rotateTo(rotation);
        }
        this.markDirty();
    }

    public void registerDependent(Node3D newDependent) {
        // Make sure we don't hit a dependency loop
        if (Node3D.class.isAssignableFrom(newDependent.getClass())) {
            if (newDependent.isAncestorOf(this)) {
                this.transferToParent(newDependent.getParentAxes());
            }
        }
        if (dependentsRegistry.indexOf(newDependent) == -1) {
            dependentsRegistry.add(new DependencyReference<Node3D>(newDependent));
        }
    }

    public boolean isAncestorOf(Node3D potentialDescendent) {
        boolean result = false;
        Node3D cursor = potentialDescendent.getParentAxes();
        while (cursor != null) {
            if (cursor == this) {
                result = true;
                break;
            } else {
                cursor = cursor.getParentAxes();
            }
        }
        return result;
    }

    /**
     * unregisters this Axes from its current parent and
     * registers it to a new parent without changing its global position or
     * orientation
     * when doing so.
     *
     * @param newParent
     */

    public void transferToParent(Node3D newParent) {
        this.emancipate();
        this.setParent(newParent);
    }

    /**
     * unregisters this Axes from its parent,
     * but keeps its global position the same.
     */
    public void emancipate() {
        if (this.getParentAxes() != null) {
            this.updateGlobal();
            Node3D oldParent = this.getParentAxes();
            for (DependencyReference<Node3D> ad : this.dependentsRegistry) {
                ad.get().parentChangeWarning(this, this.getParentAxes(), null, null);
            }
            this.getLocalMBasis().adoptValues(this.globalMBasis);
            this.getParentAxes().disown(this);
            this.parent = new DependencyReference<Node3D>(null);
            this.areGlobal = true;
            this.markDirty();
            this.updateGlobal();
            for (DependencyReference<Node3D> ad : this.dependentsRegistry) {
                ad.get().parentChangeCompletionNotice(this, oldParent, null, null);
            }
        }
    }

    public void disown(Node3D child) {
        dependentsRegistry.remove(child);
    }

    public Transform3D getGlobalMBasis() {
        this.updateGlobal();
        return globalMBasis;
    }

    public Transform3D getLocalMBasis() {
        return localMBasis;
    }

    @Override
    public ewbik.asj.data.JSONObject getSaveJSON(ewbik.asj.SaveManager saveManager) {
        this.updateGlobal();
        ewbik.asj.data.JSONObject thisAxes = new ewbik.asj.data.JSONObject();
        ewbik.asj.data.JSONObject shearScale = new ewbik.asj.data.JSONObject();
        Vector3 xShear = new Vector3();
        Vector3 yShear = new Vector3();
        Vector3 zShear = new Vector3();

        this.getLocalMBasis().setToShearXBase(xShear);
        this.getLocalMBasis().setToShearYBase(yShear);
        this.getLocalMBasis().setToShearZBase(zShear);

        shearScale.setJSONArray("x", xShear.toJSONArray());
        shearScale.setJSONArray("y", yShear.toJSONArray());
        shearScale.setJSONArray("z", zShear.toJSONArray());

        thisAxes.setJSONArray("translation", (new Vector3(getLocalMBasis().translate)).toJSONArray());
        thisAxes.setJSONArray("rotation", getLocalMBasis().rotation.toJsonArray());
        thisAxes.setJSONObject("bases", shearScale);

        String parentHash = "-1";
        if (getParentAxes() != null)
            parentHash = getParentAxes().getIdentityHash();
        thisAxes.setString("parent", parentHash);
        thisAxes.setString("identityHash", this.getIdentityHash());
        return thisAxes;
    }

    public void axisSlipWarning(Node3D globalPriorToSlipping,
            Node3D globalAfterSlipping, Node3D actualAxis,
            ArrayList<Object> dontWarn) {
        this.updateGlobal();
        if (this.getParentAxes() != null) {
            Node3D globalVals = globalPriorToSlipping;
            this.getLocalMBasis().adoptValues(globalMBasis);
            this.markDirty();
        }
    }

    public void axisSlipWarning(Node3D globalPriorToSlipping,
            Node3D globalAfterSlipping, Node3D actualAxis) {

    }

    public void axisSlipCompletionNotice(Node3D globalPriorToSlipping,
            Node3D globalAfterSlipping, Node3D thisAxis) {

    }

    public void slipTo(Node3D newAxisGlobal) {
        this.updateGlobal();
        Node3D originalGlobal = this.getGlobalCopy();
        notifyDependentsOfSlip(newAxisGlobal);
        Node3D newVals = newAxisGlobal.freeCopy();

        if (this.getParentAxes() != null) {
            newVals = getParentAxes().getLocalOf(newVals);
        }
        this.getLocalMBasis().adoptValues(newVals.globalMBasis);
        this.dirty = true;
        this.updateGlobal();

        notifyDependentsOfSlipCompletion(originalGlobal);
    }

    public Node3D freeCopy() {
        Node3D freeCopy = new Node3D(this.getLocalMBasis(),
                null);
        freeCopy.getLocalMBasis().adoptValues(this.localMBasis);
        freeCopy.markDirty();
        freeCopy.updateGlobal();
        return freeCopy;
    }

    /**
     * You probably shouldn't touch this unless you're implementing i/o or
     * undo/redo.
     *
     * @return
     */
    protected DependencyReference<Node3D> getWeakRefToParent() {
        return this.parent;
    }

    /**
     * You probably shouldn't touch this unless you're implementing i/o or
     * undo/redo.
     *
     * @return
     */
    protected void setWeakRefToParent(DependencyReference<Node3D> parentRef) {
        this.parent = parentRef;
    }

    public void slipTo(Node3D newAxisGlobal, ArrayList<Object> dontWarn) {
        this.updateGlobal();
        Node3D originalGlobal = this.getGlobalCopy();
        notifyDependentsOfSlip(newAxisGlobal, dontWarn);
        Node3D newVals = newAxisGlobal.getGlobalCopy();

        if (this.getParentAxes() != null) {
            newVals = getParentAxes().getLocalOf(newAxisGlobal);
        }
        this.alignGlobalsTo(newAxisGlobal);
        this.markDirty();
        this.updateGlobal();

        notifyDependentsOfSlipCompletion(originalGlobal, dontWarn);
    }

    public void notifyDependentsOfSlip(Node3D newAxisGlobal, ArrayList<Object> dontWarn) {
        for (int i = 0; i < dependentsRegistry.size(); i++) {
            if (!dontWarn.contains(dependentsRegistry.get(i))) {
                Node3D dependant = dependentsRegistry.get(i).get();

                // First we check if the dependent extends Axes
                // so we know whether or not to pass the dontWarn list
                if (this.getClass().isAssignableFrom(dependant.getClass())) {
                    dependant.axisSlipWarning(this.getGlobalCopy(),
                            newAxisGlobal, this, dontWarn);
                } else {
                    dependant.axisSlipWarning(this.getGlobalCopy(), newAxisGlobal, this);
                }
            } else {
                System.out.println("skipping: " + dependentsRegistry.get(i));
            }
        }
    }

    public void notifyDependentsOfSlipCompletion(Node3D globalAxisPriorToSlipping,
            ArrayList<Object> dontWarn) {
        for (int i = 0; i < dependentsRegistry.size(); i++) {
            if (!dontWarn.contains(dependentsRegistry.get(i)))
                dependentsRegistry.get(i).get().axisSlipCompletionNotice(globalAxisPriorToSlipping,
                        this.getGlobalCopy(), this);
            else
                System.out.println("skipping: " + dependentsRegistry.get(i));
        }
    }

    public void notifyDependentsOfSlip(Node3D newAxisGlobal) {
        for (int i = 0; i < dependentsRegistry.size(); i++) {
            dependentsRegistry.get(i).get().axisSlipWarning(this.getGlobalCopy(), newAxisGlobal, this);
        }
    }

    public void notifyDependentsOfSlipCompletion(Node3D globalAxisPriorToSlipping) {
        for (int i = 0; i < dependentsRegistry.size(); i++) {
            dependentsRegistry.get(i).get().axisSlipCompletionNotice(globalAxisPriorToSlipping, this.getGlobalCopy(),
                    this);
        }
    }

    public void markDirty() {

        if (!this.dirty) {
            this.dirty = true;
            this.markDependentsDirty();
        }

    }

    public void markDependentsDirty() {
        forEachDependent((a) -> a.get().markDirty());
    }

    public String toString() {
        String global = "Global: " + getGlobalMBasis().toString();
        String local = "Local: " + getLocalMBasis().toString();
        return global + "\n" + local;
    }

    @Override
    public void notifyOfSaveIntent(ewbik.asj.SaveManager saveManager) {
    }

    @Override
    public void notifyOfSaveCompletion(ewbik.asj.SaveManager saveManager) {
    }

    @Override
    public boolean isLoading() {
        return false;
    }

    @Override
    public void setLoading(boolean loading) {
    }

    @Override
    public void makeSaveable(ewbik.asj.SaveManager saveManager) {
        saveManager.addToSaveState(this);
        forEachDependent(
                (ad) -> {
                    if (ewbik.asj.Saveable.class.isAssignableFrom(ad.get().getClass()))
                        ((ewbik.asj.Saveable) ad.get()).makeSaveable(saveManager);
                });
    }

    public void parentChangeWarning(Node3D warningBy,
            Node3D oldParent, Node3D intendedParent,
            Object requestedBy) {
    }

    public void parentChangeCompletionNotice(Node3D warningBy,
            Node3D oldParent, Node3D intendedParent,
            Object requestedBy) {
    }

    public void loadFromJSONObject(ewbik.asj.data.JSONObject j, ewbik.asj.LoadManager l) {
        Vector3 origin = new Vector3(j.getJSONArray("translation"));
        Quaternion rotation = new Quaternion(j.getJSONArray("rotation"));
        this.getLocalMBasis().translate = origin;
        this.getLocalMBasis().rotation = rotation;
        this.getLocalMBasis().refreshPrecomputed();
        Node3D par;
        try {
            par = l.getObjectFor(Node3D.class, j, "parent");
            if (par != null)
                this.setRelativeToParent(par);
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    /**
     * custom Weakreference extension for garbage collection
     */
    public class DependencyReference<E> extends WeakReference<E> {
        public DependencyReference(E referent) {
            super(referent);
        }

        @Override
        public boolean equals(Object o) {
            if (o == this)
                return true;
            return o == this.get();
        }
    }
}
