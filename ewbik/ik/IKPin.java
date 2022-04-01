package ik;

import processing.core.PVector;

import java.util.ArrayList;

public class IKPin implements ewbik.asj.Saveable {

    public static final short XDir = 1;
    public static final short YDir = 2;
    public static final short ZDir = 4;
    public Bone forBone;
    protected boolean isEnabled;
    protected ewbik.processing.sceneGraph.Transform3D transform3D;
    protected IKPin parentPin;
    protected ArrayList<IKPin> childPins = new ArrayList<>();
    protected float xPriority = 1f;
    protected float yPriority = 1f;
    protected float zPriority = 1f;
    float pinWeight = 1;
    byte modeCode = 7;
    int subTargetCount = 4;
    float depthFalloff = 0f;

    // default constructor required for file loading to work
    public IKPin() {
    }

    public IKPin(ewbik.processing.sceneGraph.Transform3D inTransform3D, boolean enabled, Bone bone) {
        this.isEnabled = enabled;
        this.transform3D = (ewbik.processing.sceneGraph.Transform3D) inTransform3D;
        this.forBone = bone;
        setTargetPriorities(IKPin.this.xPriority, IKPin.this.yPriority, IKPin.this.zPriority);
    }

    public IKPin(ewbik.processing.sceneGraph.Transform3D inTransform3D, Bone bone) {
        this.transform3D = (ewbik.processing.sceneGraph.Transform3D) inTransform3D;
        this.forBone = bone;
        this.isEnabled = false;
        setTargetPriorities(IKPin.this.xPriority, IKPin.this.yPriority, IKPin.this.zPriority);
    }

    public PVector getLocation() {
        return ewbik.processing.sceneGraph.Transform3D.toPVector(getLocation_());
    }

    public void translateTo(PVector v) {
        translateTo_(ewbik.processing.sceneGraph.Transform3D.toVector3(v));
    }

    public void translateBy(PVector v) {
        translateBy_(ewbik.processing.sceneGraph.Transform3D.toVector3(v));
    }

    /**
     * rotate this pin about its X axis
     **/
    public void rotateAboutX(float radians) {
        transform3D.rotateAboutX(radians, true);
    }

    /**
     * rotate this pin about its X axis
     **/
    public void rotateAboutY(float radians) {
        transform3D.rotateAboutY(radians, true);
    }

    /**
     * rotate this pin about its X axis
     **/
    public void rotateAboutZ(float radians) {
        transform3D.rotateAboutZ(radians, true);
    }

    public ewbik.processing.sceneGraph.Transform3D getAxes() {
        return (ewbik.processing.sceneGraph.Transform3D) transform3D;
    }

    public Bone forBone() {

        return (Bone) this.forBone;
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
        this.forBone.parentArmature.rootwardlyUpdateFalloffCacheFrom(forBone);
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
        boolean xDir = xPriority > 0 ? true : false;
        boolean yDir = yPriority > 0 ? true : false;
        boolean zDir = zPriority > 0 ? true : false;
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
        this.forBone.parentArmature.rootwardlyUpdateFalloffCacheFrom(forBone);
    }

    /**
     * @return the number of bases an effector to this target will attempt to align
     * on.
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
     * @param inTransform3D
     */
    public void alignToAxes(ewbik.processing.sceneGraph.Transform3D inTransform3D) {
        this.transform3D.alignGlobalsTo(inTransform3D);
    }

    /**
     * translates the pin to the location specified in global coordinates
     *
     * @param location
     */
    public void translateTo_(ewbik.math.Vec3f<?> location) {
        this.transform3D.translateTo(location);
    }

    /**
     * translates the pin to the location specified in Armature coordinates
     * (in other words, relative to whatever coordinate frame the armature itself is
     * specified in)
     *
     * @param location
     */
    public void translateToArmatureLocal_(ewbik.math.Vec3f<?> location) {
        ewbik.processing.sceneGraph.Transform3D armTransform3D = this.forBone().parentArmature.localAxes().getParentAxes();
        if (armTransform3D == null) {
            this.transform3D.translateTo(location);
        } else {
            this.transform3D.translateTo(armTransform3D.getLocalOf(location));
        }
    }

    /**
     * translates the pin to the location specified in local coordinates
     * (relative to any other Axes objects the pin may be parented to)
     *
     * @param location
     */
    public void translateBy_(ewbik.math.Vec3f<?> location) {
        this.transform3D.translateByLocal(location);
    }

    /**
     * @return the pin locationin global coordinates
     */
    public ewbik.math.Vec3f<?> getLocation_() {
        return transform3D.origin_();
    }

    /**
     * called when this pin is being removed entirely from the Armature. (as opposed
     * to just being disabled)
     */
    public void removalNotification() {
        for (IKPin cp : childPins) {
            cp.setParentPin(getParentPin());
        }
    }

    public void solveIKForThisAndChildren() {

        try {
            for (IKPin childPin : childPins) {
                childPin.solveIKForThisAndChildren();
            }
            this.forBone.solveIKFromHere();
        } catch (Exception e) {
            e.printStackTrace(System.out);
        }
    }

    public void removeChildPin(IKPin child) {
        childPins.remove(child);
    }

    public void addChildPin(IKPin newChild) {
        if (newChild.isAncestorOf(this)) {
            this.setParentPin(newChild.getParentPin());
        }
        if (!childPins.contains(newChild))
            childPins.add(newChild);
    }

    public IKPin getParentPin() {
        return this.parentPin;
    }

    public void setParentPin(IKPin parent) {
        if (this.parentPin != null) {
            this.parentPin.removeChildPin(this);
        }
        // set the parent to the global axes if the user
        // tries to set the pin to be its own parent

        if (parent == this || parent == null) {
            this.transform3D.setParent(null);
        } else if (parent != null) {
            this.transform3D.setParent(parent.transform3D);
            parent.addChildPin(this);
            this.parentPin = parent;
        }
    }

    public boolean isAncestorOf(IKPin potentialDescendent) {
        boolean result = false;
        IKPin cursor = potentialDescendent.getParentPin();
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
        this.forBone.parentArmature.rootwardlyUpdateFalloffCacheFrom(forBone);
    }

    @Override
    public void makeSaveable(ewbik.asj.SaveManager saveManager) {
        saveManager.addToSaveState(this);
        getAxes().makeSaveable(saveManager);
    }

    @Override
    public ewbik.asj.data.JSONObject getSaveJSON(ewbik.asj.SaveManager saveManager) {
        ewbik.asj.data.JSONObject saveJSON = new ewbik.asj.data.JSONObject();
        saveJSON.setString("identityHash", this.getIdentityHash());
        saveJSON.setString("axes", getAxes().getIdentityHash());
        saveJSON.setString("forBone", forBone.getIdentityHash());
        saveJSON.setBoolean("isEnabled", this.isEnabled());
        saveJSON.setFloat("pinWeight", this.pinWeight);
        ewbik.asj.data.JSONObject priorities = new ewbik.asj.data.JSONObject();
        priorities.setFloat("x", xPriority);
        priorities.setFloat("y", yPriority);
        priorities.setFloat("z", zPriority);
        saveJSON.setFloat("depthFalloff", depthFalloff);
        saveJSON.setJSONObject("priorities", priorities);
        return saveJSON;
    }

    public void loadFromJSONObject(ewbik.asj.data.JSONObject j, ewbik.asj.LoadManager l) {
        this.transform3D = (ewbik.processing.sceneGraph.Transform3D) l.getObjectFromClassMaps(ewbik.processing.sceneGraph.Transform3D.class, j.getString("axes"));
        this.isEnabled = j.getBoolean("isEnabled");
        this.pinWeight = j.getFloat("pinWeight");
        this.forBone = (Bone) l.getObjectFromClassMaps(Bone.class, j.getString("forBone"));
        if (j.hasKey("priorities")) {
            ewbik.asj.data.JSONObject priorities = j.getJSONObject("priorities");
            xPriority = priorities.getFloat("x");
            yPriority = priorities.getFloat("y");
            zPriority = priorities.getFloat("z");
        }
        if (j.hasKey("depthFalloff")) {
            this.depthFalloff = j.getFloat("depthFalloff");
        }
    }

    @Override
    public void notifyOfLoadCompletion() {
        this.setTargetPriorities(xPriority, yPriority, zPriority);
        this.setDepthFalloff(depthFalloff);
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
    public boolean isLoading() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void setLoading(boolean loading) {
        // TODO Auto-generated method stub

    }
}
