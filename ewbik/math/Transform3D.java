package ewbik.math;

import ewbik.asj.LoadManager;
import ewbik.asj.data.JSONObject;

public class Transform3D extends AbstractAxes {


    public Transform3D(AbstractBasis globalBasis, AbstractAxes parent) {
        super(globalBasis, parent);
    }

    public Transform3D(Vec3f<?> origin, Vec3f<?> inX, Vec3f<?> inY, Vec3f<?> inZ,
                       AbstractAxes parent) {
        super(origin, inX, inY, inZ, parent, true);
        createTempVars(origin);

        areGlobal = true;

        localMBasis = new Basis(origin, inX, inY, inZ);
        globalMBasis = new Basis(origin, inX, inY, inZ);

        Vec3f<?> o = origin.copy();
        o.set(0, 0, 0);
        Vec3f<?> i = o.copy();
        i.set(1, 1, 1);

        if (parent != null) {
            this.setParent(parent);
        } else {
            this.areGlobal = true;
        }
        this.markDirty();
        this.updateGlobal();
    }

    public Ray3 x_() {
        this.updateGlobal();
        return this.getGlobalMBasis().getXRay();
    }


    public Ray3 y_() {
        this.updateGlobal();
        return this.getGlobalMBasis().getYRay();
    }

    public Ray3 z_() {
        this.updateGlobal();
        return this.getGlobalMBasis().getZRay();
    }

    @Override
    public <A extends AbstractAxes> boolean equals(A ax) {
        this.updateGlobal();
        ax.updateGlobal();

        boolean composedRotationsAreEquivalent = getGlobalMBasis().rotation.equals(ax.globalMBasis.rotation);
        boolean originsAreEquivalent = getGlobalMBasis().getOrigin().equals(ax.origin_());

        return composedRotationsAreEquivalent && originsAreEquivalent;
    }

    @Override
    public ewbik.math.Transform3D getGlobalCopy() {
        return new ewbik.math.Transform3D(getGlobalMBasis(), this.getParentAxes());
    }


    @Override
    public AbstractAxes relativeTo(AbstractAxes in) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public AbstractAxes getLocalOf(AbstractAxes input) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public AbstractAxes freeCopy() {
        AbstractAxes freeCopy =
                new ewbik.math.Transform3D(this.getLocalMBasis(),
                        null);
        freeCopy.getLocalMBasis().adoptValues(this.localMBasis);
        freeCopy.markDirty();
        freeCopy.updateGlobal();
        return freeCopy;
    }

    @Override
    public void loadFromJSONObject(JSONObject j, LoadManager l) {
        super.loadFromJSONObject(j, l);
    }

    /**
     * Creates an exact copy of this Axes object. Attached to the same parent as this Axes object
     *
     * @param slipAware
     * @return
     */
    @Override
    public ewbik.math.Transform3D attachedCopy(boolean slipAware) {
        this.updateGlobal();
        ewbik.math.Transform3D copy = new ewbik.math.Transform3D(getGlobalMBasis(),
                this.getParentAxes());
        if (!slipAware) copy.setSlipType(IGNORE);
        copy.getLocalMBasis().adoptValues(this.localMBasis);
        copy.markDirty();
        return copy;
    }

    @Override
    public <B extends AbstractBasis> B getLocalOf(B input) {
        Basis newBasis = new Basis((Basis) input);
        getGlobalMBasis().setToLocalOf(input, newBasis);
        return (B) newBasis;
    }

}
