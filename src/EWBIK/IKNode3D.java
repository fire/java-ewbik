package EWBIK;/*
                   
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

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.function.Consumer;

public class IKNode3D {
    public static final int NORMAL = 0;
    public static final int FORWARD = 2;
    public static final int RIGHT = 1;
    public static final int LEFT = -1;
    public static final int X = 0;
    public static final int Y = 1;
    public static final int Z = 2;
    public boolean debug = false;
    public IKTransform localMBasis;
    public IKTransform globalMBasis;
    public boolean dirty = true;
    public LinkedList<DependencyReference<IKNode3D>> dependentsRegistry = new LinkedList<DependencyReference<IKNode3D>>();
    protected IKVector3 workingVector;
    protected boolean areGlobal = true;
    float[][] outMatLocal = new float[4][4];
    float[][] outMatGlobal = new float[4][4];
    IKVector3 tempOrigin;
    private DependencyReference<IKNode3D> parent = null;

    public IKNode3D(IKTransform b, IKNode3D parent) {
        this.globalMBasis = ((IKTransform) b).copy();
        createTempVars(((IKTransform) b).getOrigin());
        if (this.getParentAxes() != null)
            IKNode3D.this.setParent(parent);
        else {
            this.areGlobal = true;
            this.localMBasis = ((IKTransform) b).copy();
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
    public IKNode3D(IKVector3 origin,
                    IKVector3 inX,
                    IKVector3 inY,
                    IKVector3 inZ,
                    IKNode3D parent) {

        IKVector3 origin1 = origin;
        IKVector3 inX1 = inX;
        IKVector3 inY1 = inY;
        IKVector3 inZ1 = inZ;
        if (parent == null)
            this.areGlobal = true;
        createTempVars(origin1);

        IKNode3D.this.areGlobal = true;

        IKNode3D.this.localMBasis = new IKTransform(origin1, inX1, inY1, inZ1);
        IKNode3D.this.globalMBasis = new IKTransform(origin1, inX1, inY1, inZ1);

        IKVector3 o = origin1.copy();
        o.set(0, 0, 0);
        IKVector3 i = o.copy();
        i.set(1, 1, 1);

        if (parent != null) {
            this.setParent(parent);
        } else {
            this.areGlobal = true;
        }
        this.markDirty();
        this.updateGlobal();
    }

    public IKNode3D(IKVector3 origin,
                    IKVector3 x,
                    IKVector3 y,
                    IKVector3 z) {
        this(origin, x, y, z, true, null);
    }

    public IKNode3D() {
        IKVector3 origin = new IKVector3(0, 0, 0);
        IKVector3 inX = new IKVector3(1, 0, 0);
        IKVector3 inY = new IKVector3(0, 1, 0);
        IKVector3 inZ = new IKVector3(0, 0, 1);
        if (null == null)
            this.areGlobal = true;
        createTempVars(origin);

        IKNode3D.this.areGlobal = true;

        IKNode3D.this.localMBasis = new IKTransform(origin, inX, inY, inZ);
        IKNode3D.this.globalMBasis = new IKTransform(origin, inX, inY, inZ);

        IKVector3 o = origin.copy();
        o.set(0, 0, 0);
        IKVector3 i = o.copy();
        i.set(1, 1, 1);

        if (null != null) {
            this.setParent(null);
        } else {
            this.areGlobal = true;
        }
        this.markDirty();
        this.updateGlobal();
    }

    public IKNode3D(IKVector3 origin, IKVector3 x, IKVector3 y, IKVector3 z, boolean forceOrthoNormality,
                    IKNode3D parent) {
        if (parent == null)
            this.areGlobal = true;
        createTempVars(origin);

        IKNode3D.this.areGlobal = true;

        IKNode3D.this.localMBasis = new IKTransform(origin, x, y, z);
        IKNode3D.this.globalMBasis = new IKTransform(origin, x, y, z);

        IKVector3 o = origin.copy();
        o.set(0, 0, 0);
        IKVector3 i = o.copy();
        i.set(1, 1, 1);

        if (parent != null) {
            this.setParent(parent);
        } else {
            this.areGlobal = true;
        }
        this.markDirty();
        this.updateGlobal();
    }
    public IKVector3 origin() {
        return this.calculatePosition();
    }

    /**
     * Make a GlobalCopy of these Axes.
     *
     * @return
     */
    public IKNode3D getGlobalCopy() {
        this.updateGlobal();
        return new IKNode3D(getGlobalMBasis(), this.getParentAxes());
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

    public IKVector3 calculatePositionPVector() {
        return calculatePosition();
    }

    private void updateMatrix(IKTransform b, float[][] outputMatrix) {
        b.refreshPrecomputed();

        IKVector3 x = b.getXHeading();
        IKVector3 y = b.getYHeading();
        IKVector3 z = b.getZHeading();

        IKVector3 origin = b.getOrigin();

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

    /**
     * return a ray / segment representing this Axes global x basis position and
     * direction and magnitude
     *
     * @return a ray / segment representing this Axes global x basis position and
     *         direction and magnitude
     */
    public IKRay3D calculateX() {
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
    public IKRay3D calculateY() {
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
    public IKRay3D calculateZ() {
        this.updateGlobal();
        return this.getGlobalMBasis().getZRay();
    }

    public boolean equals(IKNode3D ax) {
        this.updateGlobal();
        ax.updateGlobal();

        boolean composedRotationsAreEquivalent = getGlobalMBasis().rotation.equals(ax.globalMBasis.rotation);
        boolean originsAreEquivalent = getGlobalMBasis().getOrigin().equals(ax.calculatePosition());

        return composedRotationsAreEquivalent && originsAreEquivalent;
    }

    public IKNode3D relativeTo(IKNode3D in) {
        return null;
    }

    public IKNode3D getLocalOf(IKNode3D input) {
        return null;
    }

    /**
     * Creates an exact copy of this Axes object. Attached to the same parent as
     * this Axes object
     *
     * @param slipAware
     * @return
     */
    public IKNode3D attachedCopy() {
        this.updateGlobal();
        IKNode3D copy = new IKNode3D(getGlobalMBasis(),
                this.getParentAxes());
        copy.getLocalMBasis().adoptValues(this.localMBasis);
        copy.markDirty();
        return copy;
    }

    public IKTransform getLocalOf(IKTransform input) {
        IKTransform newBasis = new IKTransform((IKTransform) input);
        getGlobalMBasis().setToLocalOf(input, newBasis);
        return (IKTransform) newBasis;
    }

    public void createTempVars(IKVector3 type) {
        workingVector = type.copy();
        tempOrigin = type.copy();
    }

    public IKNode3D getParentAxes() {
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

    public IKVector3 calculatePosition() {
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
    public void setParent(IKNode3D par) {
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
    public void setParent(IKNode3D intendedParent, Object requestedBy) {
        this.updateGlobal();
        IKNode3D oldParent = this.getParentAxes();
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
            this.parent = new DependencyReference<IKNode3D>(intendedParent);

            this.getParentAxes().registerDependent(this);
            this.areGlobal = false;
        } else {
            if (oldParent != null)
                oldParent.disown(this);
            this.parent = new DependencyReference<IKNode3D>(null);
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
    public void forEachDependent(Consumer<DependencyReference<IKNode3D>> action) {
        Iterator<DependencyReference<IKNode3D>> i = dependentsRegistry.iterator();
        while (i.hasNext()) {
            DependencyReference<IKNode3D> dr = i.next();
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
    public void setRelativeToParent(IKNode3D par) {
        if (this.getParentAxes() != null)
            this.getParentAxes().disown(this);
        this.parent = new DependencyReference<IKNode3D>(par);
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
    public IKVector3 getGlobalOf(IKVector3 in) {
        IKVector3 result = (IKVector3) in.copy();
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
    public IKVector3 setToGlobalOf(IKVector3 in) {
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
    public void setToGlobalOf(IKVector3 input, IKVector3 output) {
        this.updateGlobal();
        getGlobalMBasis().setToGlobalOf(input, output);
    }

    /**
     * Given an input sgRay in this axes local coordinates, modifies the output
     * Rays's values to represent the input's in global coordinates.
     *
     * @param in
     */
    public void setToGlobalOf(IKRay3D input, IKRay3D output) {
        this.updateGlobal();
        this.setToGlobalOf(input.p1(), output.p1());
        this.setToGlobalOf(input.p2(), output.p2());
    }

    public IKRay3D getGlobalOf(IKRay3D in) {
        return new IKRay3D(this.getGlobalOf(in.p1()), this.getGlobalOf(in.p2()));
    }

    public IKVector3 getLocalOf(IKVector3 in) {
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

    public IKVector3 setToLocalOf(IKVector3 in) {
        this.updateGlobal();
        IKVector3 result = (IKVector3) in.copy();
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

    public void setToLocalOf(IKVector3 in, IKVector3 out) {
        this.updateGlobal();
        this.getGlobalMBasis().setToLocalOf(in, out);
    }

    /**
     * Given a sgRay in global coordinates, modifies the sgRay's values to represent
     * its position in theseAxes local coordinates.
     *
     * @param in
     */

    public void setToLocalOf(IKRay3D in, IKRay3D out) {
        this.setToLocalOf(in.p1(), out.p1());
        this.setToLocalOf(in.p2(), out.p2());
    }

    public void setToLocalOf(IKTransform input, IKTransform output) {
        this.updateGlobal();
        this.getGlobalMBasis().setToLocalOf(input, output);
    }

    public IKRay3D getLocalOf(IKRay3D in) {
        return new IKRay3D(this.getLocalOf(in.p1()), this.getLocalOf(in.p2()));
    }

    public void translateByLocal(IKVector3 translate) {
        this.updateGlobal();
        getLocalMBasis().translateBy(translate);
        this.markDirty();

    }

    public void translateByGlobal(IKVector3 translate) {
        if (this.getParentAxes() != null) {
            this.updateGlobal();
            this.translateTo(translate.addCopy(this.calculatePosition()));
        } else {
            getLocalMBasis().translateBy(translate);
        }

        this.markDirty();
    }

    public void translateTo(IKVector3 translate, boolean slip) {
        this.updateGlobal();
        if (slip) {
            IKNode3D tempNode3D = this.getGlobalCopy();
            tempNode3D.translateTo(translate);
            this.slipTo(tempNode3D);
        } else {
            this.translateTo(translate);
        }
    }

    public void translateTo(IKVector3 translate) {
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
        IKQuaternion xRot = new IKQuaternion(getGlobalMBasis().getXHeading(), angle);
        this.rotateBy(xRot);
        this.markDirty();
    }

    public void rotateAboutY(float angle, boolean orthonormalized) {
        this.updateGlobal();
        IKQuaternion yRot = new IKQuaternion(getGlobalMBasis().getYHeading(), angle);
        this.rotateBy(yRot);
        this.markDirty();
    }

    public void rotateAboutZ(float angle, boolean orthonormalized) {
        this.updateGlobal();
        IKQuaternion zRot = new IKQuaternion(getGlobalMBasis().getZHeading(), angle);
        this.rotateBy(zRot);
        this.markDirty();
    }

    public void rotateBy(IKQuaternion apply) {
        this.updateGlobal();
        if (parent != null) {
            IKQuaternion newRot = this.getParentAxes().getGlobalMBasis().getLocalOfRotation(apply);
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
    public void rotateByLocal(IKQuaternion apply) {
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
    public void alignLocalsTo(IKNode3D targetNode3D) {
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
        this.getLocalMBasis().rotateTo(new IKQuaternion());
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
    public void alignGlobalsTo(IKNode3D targetNode3D) {
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

    public void alignOrientationTo(IKNode3D targetNode3D) {
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
    public void setGlobalOrientationTo(IKQuaternion rotation) {
        this.updateGlobal();
        if (this.getParentAxes() != null) {
            this.getGlobalMBasis().rotateTo(rotation);
            getParentAxes().getGlobalMBasis().setToLocalOf(this.globalMBasis, this.localMBasis);
        } else {
            this.getLocalMBasis().rotateTo(rotation);
        }
        this.markDirty();
    }

    public void registerDependent(IKNode3D newDependent) {
        // Make sure we don't hit a dependency loop
        if (IKNode3D.class.isAssignableFrom(newDependent.getClass())) {
            if (newDependent.isAncestorOf(this)) {
                this.transferToParent(newDependent.getParentAxes());
            }
        }
        if (dependentsRegistry.indexOf(newDependent) == -1) {
            dependentsRegistry.add(new DependencyReference<IKNode3D>(newDependent));
        }
    }

    public boolean isAncestorOf(IKNode3D potentialDescendent) {
        boolean result = false;
        IKNode3D cursor = potentialDescendent.getParentAxes();
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

    public void transferToParent(IKNode3D newParent) {
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
            IKNode3D oldParent = this.getParentAxes();
            for (DependencyReference<IKNode3D> ad : this.dependentsRegistry) {
                ad.get().parentChangeWarning(this, this.getParentAxes(), null, null);
            }
            this.getLocalMBasis().adoptValues(this.globalMBasis);
            this.getParentAxes().disown(this);
            this.parent = new DependencyReference<IKNode3D>(null);
            this.areGlobal = true;
            this.markDirty();
            this.updateGlobal();
            for (DependencyReference<IKNode3D> ad : this.dependentsRegistry) {
                ad.get().parentChangeCompletionNotice(this, oldParent, null, null);
            }
        }
    }

    public void disown(IKNode3D child) {
        dependentsRegistry.remove(child);
    }

    public IKTransform getGlobalMBasis() {
        this.updateGlobal();
        return globalMBasis;
    }

    public IKTransform getLocalMBasis() {
        return localMBasis;
    }

    public void axisSlipWarning(IKNode3D globalPriorToSlipping,
                                IKNode3D globalAfterSlipping, IKNode3D actualAxis,
                                ArrayList<Object> dontWarn) {
        this.updateGlobal();
        if (this.getParentAxes() != null) {
            IKNode3D globalVals = globalPriorToSlipping;
            this.getLocalMBasis().adoptValues(globalMBasis);
            this.markDirty();
        }
    }

    public void axisSlipWarning(IKNode3D globalPriorToSlipping,
                                IKNode3D globalAfterSlipping, IKNode3D actualAxis) {

    }

    public void axisSlipCompletionNotice(IKNode3D globalPriorToSlipping,
                                         IKNode3D globalAfterSlipping, IKNode3D thisAxis) {

    }

    public void slipTo(IKNode3D newAxisGlobal) {
        this.updateGlobal();
        IKNode3D originalGlobal = this.getGlobalCopy();
        notifyDependentsOfSlip(newAxisGlobal);
        IKNode3D newVals = newAxisGlobal.freeCopy();

        if (this.getParentAxes() != null) {
            newVals = getParentAxes().getLocalOf(newVals);
        }
        this.getLocalMBasis().adoptValues(newVals.globalMBasis);
        this.dirty = true;
        this.updateGlobal();

        notifyDependentsOfSlipCompletion(originalGlobal);
    }

    public IKNode3D freeCopy() {
        IKNode3D freeCopy = new IKNode3D(this.getLocalMBasis(),
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
    protected DependencyReference<IKNode3D> getWeakRefToParent() {
        return this.parent;
    }

    /**
     * You probably shouldn't touch this unless you're implementing i/o or
     * undo/redo.
     *
     * @return
     */
    protected void setWeakRefToParent(DependencyReference<IKNode3D> parentRef) {
        this.parent = parentRef;
    }

    public void slipTo(IKNode3D newAxisGlobal, ArrayList<Object> dontWarn) {
        this.updateGlobal();
        IKNode3D originalGlobal = this.getGlobalCopy();
        notifyDependentsOfSlip(newAxisGlobal, dontWarn);
        IKNode3D newVals = newAxisGlobal.getGlobalCopy();

        if (this.getParentAxes() != null) {
            newVals = getParentAxes().getLocalOf(newAxisGlobal);
        }
        this.alignGlobalsTo(newAxisGlobal);
        this.markDirty();
        this.updateGlobal();

        notifyDependentsOfSlipCompletion(originalGlobal, dontWarn);
    }

    public void notifyDependentsOfSlip(IKNode3D newAxisGlobal, ArrayList<Object> dontWarn) {
        for (int i = 0; i < dependentsRegistry.size(); i++) {
            if (!dontWarn.contains(dependentsRegistry.get(i))) {
                IKNode3D dependant = dependentsRegistry.get(i).get();

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

    public void notifyDependentsOfSlipCompletion(IKNode3D globalAxisPriorToSlipping,
                                                 ArrayList<Object> dontWarn) {
        for (int i = 0; i < dependentsRegistry.size(); i++) {
            if (!dontWarn.contains(dependentsRegistry.get(i)))
                dependentsRegistry.get(i).get().axisSlipCompletionNotice(globalAxisPriorToSlipping,
                        this.getGlobalCopy(), this);
            else
                System.out.println("skipping: " + dependentsRegistry.get(i));
        }
    }

    public void notifyDependentsOfSlip(IKNode3D newAxisGlobal) {
        for (int i = 0; i < dependentsRegistry.size(); i++) {
            dependentsRegistry.get(i).get().axisSlipWarning(this.getGlobalCopy(), newAxisGlobal, this);
        }
    }

    public void notifyDependentsOfSlipCompletion(IKNode3D globalAxisPriorToSlipping) {
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

    public void parentChangeWarning(IKNode3D warningBy,
                                    IKNode3D oldParent, IKNode3D intendedParent,
                                    Object requestedBy) {
    }

    public void parentChangeCompletionNotice(IKNode3D warningBy,
                                             IKNode3D oldParent, IKNode3D intendedParent,
                                             Object requestedBy) {
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
