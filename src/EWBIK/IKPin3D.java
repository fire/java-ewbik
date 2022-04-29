package EWBIK;

import java.util.ArrayList;

public class IKPin3D {

    public static final short XDir = 1;
    public static final short YDir = 2;
    public static final short ZDir = 4;
    public IKBone3D forBone;
    protected boolean isEnabled;
    protected IKNode3D node3D;
    protected IKPin3D parentPin;
    protected ArrayList<IKPin3D> childPins = new ArrayList<>();
    protected float xPriority = 1f;
    protected float yPriority = 1f;
    protected float zPriority = 1f;
    float pinWeight = 1;
    byte modeCode = 7;
    int subTargetCount = 4;
    float depthFalloff = 0f;

    public IKPin3D() {
    }

    public IKPin3D(IKNode3D inNode3D, boolean enabled, IKBone3D bone) {
        this.isEnabled = enabled;
        this.node3D = inNode3D;
        this.forBone = bone;
        setTargetPriorities(IKPin3D.this.xPriority, IKPin3D.this.yPriority, IKPin3D.this.zPriority);
    }

    public IKPin3D(IKNode3D inNode3D, IKBone3D bone) {
        this.node3D = inNode3D;
        this.forBone = bone;
        this.isEnabled = false;
        setTargetPriorities(IKPin3D.this.xPriority, IKPin3D.this.yPriority, IKPin3D.this.zPriority);
    }

    public IKVector3 getLocation() {
        return getLocation_();
    }

    public void translateTo(IKVector3 v) {
        translateTo_(v);
    }

    public void translateBy(IKVector3 v) {
        translateBy_(v);
    }

    /**
     * rotate this pin about its X axis
     **/
    public void rotateAboutX(float radians) {
        node3D.rotateAboutX(radians, true);
    }

    /**
     * rotate this pin about its X axis
     **/
    public void rotateAboutY(float radians) {
        node3D.rotateAboutY(radians, true);
    }

    /**
     * rotate this pin about its X axis
     **/
    public void rotateAboutZ(float radians) {
        node3D.rotateAboutZ(radians, true);
    }

    public IKNode3D getAxes() {
        return node3D;
    }

    public IKBone3D forBone() {

        return this.forBone;
    }

    public boolean isEnabled() {
        return isEnabled;
    }

    public void toggle() {
        if (this.isEnabled())
            disable();
        else
            this.enable();
    }

    public void enable() {
        this.isEnabled = true;
    }

    public void disable() {
        this.isEnabled = false;
    }

    public float getDepthFalloff() {
        return depthFalloff;
    }

    /**
     * Pins can be ultimate targets, or intermediary targets.
     * By default, each pin is treated as an ultimate target, meaning
     * any bones which are ancestors to that pin's end-effector
     * are not aware of any pins wich are target of bones descending from that end
     * effector.
     * <p>
     * Changing this value makes ancestor bones aware, and also determines how much
     * less
     * they care with each level down.
     * <p>
     * Presuming all descendants of this pin have a falloff of 1, then:
     * A pin falloff of 0 on this pin means only this pin is reported to ancestors.
     * A pin falloff of 1 on this pin means ancestors care about all descendant pins
     * equally (after accounting for their pinWeight),
     * regardless of how many levels down they are.
     * A pin falloff of 0.5 means each descendant pin is cared about half as much as
     * its ancestor.
     * <p>
     * With each level, the pin falloff of a descendant is taken account for each
     * level.
     * Meaning, if this pin has a falloff of 1, and its descendent has a falloff of
     * 0.5
     * then this pin will be reported with full weight,
     * it descendant will be reported with full weight,
     * the descendant of that pin will be reported with half weight.
     * the desecendant of that one's descendant will be reported with quarter
     * weight.
     *
     * @param depth
     */
    public void setDepthFalloff(float depth) {
        this.depthFalloff = depth;
        this.forBone.parent_armature.rootwardlyUpdateFalloffCacheFrom(forBone);
    }

    /**
     * Sets the priority of the orientation bases which effectors reaching for this
     * target will and won't align with.
     * If all are set to 0, then the target is treated as a simple position target.
     * It's usually better to set at least on of these three values to 0, as giving
     * a nonzero value to all three is most often redundant.
     * <p>
     * This values this function sets are only considered by the orientation aware
     * solver.
     *
     * @param position
     * @param xPriority set to a positive value (recommended between 0 and 1) if you
     *                  want the bone's x basis to point in the same direction as
     *                  this target's x basis (by this library's convention the x
     *                  basis corresponds to a limb's twist)
     * @param yPriority set to a positive value (recommended between 0 and 1) if you
     *                  want the bone's y basis to point in the same direction as
     *                  this target's y basis (by this library's convention the y
     *                  basis corresponds to a limb's direction)
     * @param zPriority set to a positive value (recommended between 0 and 1) if you
     *                  want the bone's z basis to point in the same direction as
     *                  this target's z basis (by this library's convention the z
     *                  basis corresponds to a limb's twist)
     */
    public void setTargetPriorities(float xPriority, float yPriority, float zPriority) {
        boolean xDir = xPriority > 0;
        boolean yDir = yPriority > 0;
        boolean zDir = zPriority > 0;
        modeCode = 0;
        if (xDir)
            modeCode += XDir;
        if (yDir)
            modeCode += YDir;
        if (zDir)
            modeCode += ZDir;

        subTargetCount = 1;
        if ((modeCode & 1) != 0)
            subTargetCount++;
        if ((modeCode & 2) != 0)
            subTargetCount++;
        if ((modeCode & 4) != 0)
            subTargetCount++;

        this.xPriority = xPriority;
        this.yPriority = yPriority;
        this.zPriority = zPriority;
        this.forBone.parent_armature.rootwardlyUpdateFalloffCacheFrom(forBone);
    }

    /**
     * @return the number of bases an effector to this target will attempt to align
     *         on.
     */
    public int getSubtargetCount() {
        return subTargetCount;
    }

    public byte getModeCode() {
        return modeCode;
    }

    /**
     * @return the priority of this pin's x axis;
     */
    public float getXPriority() {
        return this.xPriority;
    }

    /**
     * @return the priority of this pin's y axis;
     */
    public float getYPriority() {
        return this.yPriority;
    }

    /**
     * @return the priority of this pin's z axis;
     */
    public float getZPriority() {
        return this.zPriority;
    }

    /**
     * translates and rotates the pin to match the position
     * and orientation of the input Axes. The orientation
     * is only relevant for orientation aware solvers.
     *
     * @param inNode3D
     */
    public void alignToAxes(IKNode3D inNode3D) {
        this.node3D.alignGlobalsTo(inNode3D);
    }

    /**
     * translates the pin to the location specified in global coordinates
     *
     * @param location
     */
    public void translateTo_(IKVector3 location) {
        this.node3D.translateTo(location);
    }

    /**
     * translates the pin to the location specified in Armature coordinates
     * (in other words, relative to whatever coordinate frame the armature itself is
     * specified in)
     *
     * @param location
     */
    public void translateToArmatureLocal_(IKVector3 location) {
        IKNode3D armNode3D = this.forBone().parent_armature.localAxes().getParentAxes();
        if (armNode3D == null) {
            this.node3D.translateTo(location);
        } else {
            this.node3D.translateTo(armNode3D.getLocalOf(location));
        }
    }

    /**
     * translates the pin to the location specified in local coordinates
     * (relative to any other Axes objects the pin may be parented to)
     *
     * @param location
     */
    public void translateBy_(IKVector3 location) {
        this.node3D.translateByLocal(location);
    }

    /**
     * @return the pin locationin global coordinates
     */
    public IKVector3 getLocation_() {
        return node3D.calculatePosition();
    }

    /**
     * called when this pin is being removed entirely from the Armature. (as opposed
     * to just being disabled)
     */
    public void removalNotification() {
        for (IKPin3D cp : childPins) {
            cp.setParentPin(getParentPin());
        }
    }

    public void solveIKForThisAndChildren() {

        try {
            for (IKPin3D childPin : childPins) {
                childPin.solveIKForThisAndChildren();
            }
            this.forBone.solveIKFromHere();
        } catch (Exception e) {
            e.printStackTrace(System.out);
        }
    }

    public void removeChildPin(IKPin3D child) {
        childPins.remove(child);
    }

    public void addChildPin(IKPin3D newChild) {
        if (newChild.isAncestorOf(this)) {
            this.setParentPin(newChild.getParentPin());
        }
        if (!childPins.contains(newChild))
            childPins.add(newChild);
    }

    public IKPin3D getParentPin() {
        return this.parentPin;
    }

    public void setParentPin(IKPin3D parent) {
        if (this.parentPin != null) {
            this.parentPin.removeChildPin(this);
        }
        // set the parent to the global axes if the user
        // tries to set the pin to be its own parent

        if (parent == this || parent == null) {
            this.node3D.setParent(null);
        } else if (parent != null) {
            this.node3D.setParent(parent.node3D);
            parent.addChildPin(this);
            this.parentPin = parent;
        }
    }

    public boolean isAncestorOf(IKPin3D potentialDescendent) {
        boolean result = false;
        IKPin3D cursor = potentialDescendent.getParentPin();
        while (cursor != null) {
            if (cursor == this) {
                result = true;
                break;
            } else {
                cursor = cursor.parentPin;
            }
        }
        return result;
    }

    public float getPinWeight() {
        return pinWeight;
    }

    /**
     * Currently only works with tranquil solver.
     *
     * @param weight any positive number representing how much the IK solver
     *               should prefer to satisfy this pin over competing pins. For
     *               example, setting
     *               one pin's weight to 90 and a competing pins weight to 10 will
     *               mean the IK solver
     *               will prefer to satisfy the pin with a weight of 90 by as much
     *               as 9 times over satisfying
     *               the pin with a weight of 10.
     */
    public void setPinWeight(float weight) {
        this.pinWeight = weight;
        this.forBone.parent_armature.rootwardlyUpdateFalloffCacheFrom(forBone);
    }
}
