package ewbik.math;

public class Basis extends AbstractBasis {

    public Basis(ewbik.math.Basis basis) {
        super(basis);
    }

    public Basis(Vec3f<?> origin) {
        super(origin);
    }

    public <V extends Vec3f<?>> Basis(V origin, V inX, V inY, V inZ) {
        super(origin, inX, inY, inZ);
    }

    @Override
    public AbstractBasis copy() {
        return new ewbik.math.Basis(this);
    }

}
