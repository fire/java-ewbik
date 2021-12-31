package ewbik.processing.singlePrecision;

import ewbik.ik.AbstractIKPin;
import ewbik.processing.sceneGraph.Axes;
import processing.core.PVector;

public class IKPin extends AbstractIKPin {

    // default constructor required for file loading to work
    public IKPin() {
    }

    public IKPin(Axes inAxes, boolean enabled, ewbik.processing.singlePrecision.Bone bone) {
        super(inAxes, enabled, bone);
    }

    public IKPin(Axes inAxes, ewbik.processing.singlePrecision.Bone bone) {
        super(inAxes, bone);
    }

    public PVector getLocation() {
        return Axes.toPVector(super.getLocation_());
    }

    public void translateTo(PVector v) {
        super.translateTo_(Axes.toVector3(v));
    }

    public void translateBy(PVector v) {
        super.translateBy_(Axes.toVector3(v));
    }

    /**
     * rotate this pin about its X axis
     **/
    public void rotateAboutX(float radians) {
        axes.rotateAboutX(radians, true);
    }

    /**
     * rotate this pin about its X axis
     **/
    public void rotateAboutY(float radians) {
        axes.rotateAboutY(radians, true);
    }

    /**
     * rotate this pin about its X axis
     **/
    public void rotateAboutZ(float radians) {
        axes.rotateAboutZ(radians, true);
    }

    @Override
    public Axes getAxes() {
        return (Axes) axes;
    }

    @Override
    public Bone forBone() {
        return (Bone) super.forBone();
    }
}
