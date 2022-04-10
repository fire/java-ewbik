package ewbik.math;

public class Transform3D {

    public static final int LEFT = -1;
    public static final int RIGHT = 1;
    public static final int NONE = -1;
    public static final int X = 0;
    public static final int Y = 1;
    public static final int Z = 2;
    // The chirality is always right.
    public Quaternion rotation = new Quaternion();
    public Quaternion inverseRotation = new Quaternion();
    /**
     * a vector representing the translation of this basis relative to its parent.
     */
    public Vector3 translate;
    protected Vector3 xBase = new Vector3(1, 0, 0);
    protected Vector3 yBase = new Vector3(0, 1, 0);
    protected Vector3 zBase = new Vector3(0, 0, 1);
    protected Ray3D xRay = new Ray3D(new Vector3(0, 0, 0), new Vector3(1, 0, 0));
    protected Ray3D yRay = new Ray3D(new Vector3(0, 0, 0), new Vector3(0, 1, 0));
    protected Ray3D zRay = new Ray3D(new Vector3(0, 0, 0), new Vector3(0, 0, 1));

    /**
     * Initialize this basis at the origin. The basis will be righthanded by
     * default.
     *
     * @param origin
     */
    public Transform3D(Vector3 origin) {
        translate = origin.copy();
        xBase = origin.copy();
        yBase = origin.copy();
        zBase = origin.copy();
        xBase.set(1, 0, 0);
        yBase.set(0, 1, 0);
        zBase.set(0, 0, 1);
        Vector3 zero = origin.copy();
        zero.set(0, 0, 0);
        xRay = new Ray3D(zero.copy(), xBase.copy());
        yRay = new Ray3D(zero.copy(), yBase.copy());
        zRay = new Ray3D(zero.copy(), zBase.copy());
        refreshPrecomputed();
    }

    public <T extends ewbik.math.Transform3D> Transform3D(T input) {
        translate = input.translate.copy();
        xBase = translate.copy();
        yBase = translate.copy();
        zBase = translate.copy();
        xBase.set(1, 0, 0);
        yBase.set(0, 1, 0);
        zBase.set(0, 0, 1);
        Vector3 zero = translate.copy();
        zero.set(0, 0, 0);
        xRay = new Ray3D(zero.copy(), xBase.copy());
        yRay = new Ray3D(zero.copy(), yBase.copy());
        zRay = new Ray3D(zero.copy(), zBase.copy());
        this.adoptValues(input);

    }

    /**
     * Initialize this basis at the origin.
     * The basis will be backed by a rotation object which presumes right handed
     * chirality.
     * Therefore, the rotation object will align so its local XY plane aligns with
     * this basis' XY plane
     * Afterwards, it will check chirality, and if the basis isn't righthanded, this
     * class will assume the
     * z-axis is the one that's been flipped.
     * <p>
     * If you want to manually specify which axis has been flipped
     * (so that the rotation object aligns with respect to the plane formed
     * by the other two basis vectors) then use the constructor dedicated for that
     * purpose
     *
     * @param origin
     * @param x      basis vector direction
     * @param y      basis vector direction
     * @param z      basis vector direction
     */
    public <V extends Vector3> Transform3D(V origin, V x, V y, V z) {
        this.translate = origin.copy();
        xRay = new Ray3D(origin.copy(), origin.copy());
        yRay = new Ray3D(origin.copy(), origin.copy());
        zRay = new Ray3D(origin.copy(), origin.copy());
        this.set(x.copy(), y.copy(), z.copy());
    }

    /**
     * Initialize this basis at the origin defined by the base of the @param x Ray.
     * <p>
     * The basis will be backed by a rotation object which presumes right-handed
     * chirality.
     * Therefore, the rotation object will align so its local XY plane aligns with
     * this basis' XY plane
     * Afterwards, it will check chirality, and if the basis isn't right-handed,
     * this
     * class will assume the
     * z-axis is the one that's been flipped.
     * <p>
     * If you want to manually specify which axis has been flipped
     * (so that the rotation object aligns with respect to the plane formed
     * by the other two basis vectors) then use the constructor dedicated for that
     * purpose
     *
     * @param x basis Ray
     * @param y basis Ray
     * @param z basis Ray
     */
    public <R extends Ray3D> Transform3D(R x, R y, R z) {
        this.translate = x.p1().copy();
        xRay = x.copy();
        yRay = y.copy();
        zRay = z.copy();
        Vector3 xDirNew = x.heading().copy();
        Vector3 yDirNew = y.heading().copy();
        Vector3 zDirNew = z.heading().copy();
        xDirNew.normalize();
        yDirNew.normalize();
        zDirNew.normalize();
        set(xDirNew, yDirNew, zDirNew);

    }

    public ewbik.math.Transform3D copy() {
        return new ewbik.math.Transform3D(this);
    }

    private <V extends Vector3> void set(V x, V y, V z) {
        xBase = translate.copy();
        yBase = translate.copy();
        zBase = translate.copy();
        xBase.set(1, 0, 0);
        yBase.set(0, 1, 0);
        zBase.set(0, 0, 1);
        Vector3 zero = translate.copy();
        zero.set(0, 0, 0);
        xRay.setP1(zero.copy());
        xRay.setP2(xBase.copy());
        yRay.setP1(zero.copy());
        yRay.setP2(yBase.copy());
        zRay.setP1(zero.copy());
        zRay.setP2(zBase.copy());
        this.rotation = createPrioritizedRotation(x, y, z);
        this.refreshPrecomputed();
    }

    /**
     * takes on the same values (not references) as the input basis.
     *
     * @param in
     */
    public <T extends ewbik.math.Transform3D> void adoptValues(T in) {
        this.translate.set(in.translate);
        this.rotation.set(in.rotation);
        xBase = translate.copy();
        yBase = translate.copy();
        zBase = translate.copy();
        xBase.set(1, 0, 0);
        yBase.set(0, 1, 0);
        zBase.set(0, 0, 1);
        xRay = in.xRay.copy();
        yRay = in.yRay.copy();
        zRay = in.zRay.copy();
        this.refreshPrecomputed();
    }

    public void setIdentity() {
        this.translate.set(0, 0, 0);
        xBase.set(1, 0, 0);
        yBase.set(0, 1, 0);
        zBase.set(0, 0, 1);
        this.xRay.p1.set(this.translate);
        this.xRay.p2.set(xBase);
        this.yRay.p1.set(this.translate);
        this.yRay.p2.set(yBase);
        this.zRay.p1.set(this.translate);
        this.zRay.p2.set(zBase);
        this.rotation = new Quaternion();
        refreshPrecomputed();
    }

    private Quaternion createPrioritizedRotation(Vector3 xHeading, Vector3 yHeading, Vector3 zHeading) {
        Vector3 x_identity = xBase.copy();
        Vector3 y_identity = yBase.copy();
        Vector3 z_identity = zBase.copy();
        Vector3 origin = new Vector3(0, 0, 0);

        Vector3[] from = { origin, x_identity, y_identity, z_identity };
        Vector3[] to = { origin.copy(), xHeading, yHeading, zHeading };
        QCP alignHeads = new QCP(ewbik.math.MathUtils.FLOAT_ROUNDING_ERROR,
                ewbik.math.MathUtils.FLOAT_ROUNDING_ERROR);
        alignHeads.setMaxIterations(50);
        Quaternion rotation = alignHeads.weightedSuperpose(from, to, null, false);
        return rotation;
    }

    public Quaternion getLocalOfRotation(Quaternion inRot) {
        Quaternion resultNew = inverseRotation.applyTo(inRot).applyTo(rotation);
        return resultNew;
    }

    public void setToLocalOf(Transform3D global_input, Transform3D local_output) {
        local_output.translate = this.getLocalOf(global_input.translate);
        inverseRotation.applyTo(global_input.rotation, local_output.rotation);

        local_output.refreshPrecomputed();
    }

    public void refreshPrecomputed() {
        this.rotation.setToReversion(inverseRotation);
        this.updateRays();
    }

    public Vector3 getLocalOf(Vector3 v) {
        Vector3 result = v;
        setToLocalOf(v, result);
        return result;
    }

    public void setToLocalOf(Vector3 input, Vector3 output) {
        output = input;
        output.sub(this.translate);
        inverseRotation.applyTo(output, output);
    }

    public void rotateTo(Quaternion newRotation) {
        this.rotation.set(newRotation);
        this.refreshPrecomputed();
    }

    public void rotateBy(Quaternion addRotation) {
        addRotation.applyTo(this.rotation, this.rotation);
        this.refreshPrecomputed();
    }

    /**
     * the default Transform3D implementation is orthonormal,
     * so by default this function will just set @param vec to (1,0,0),
     * but extending (affine) classes can override this to represent the direction
     * and magnitude of the x axis prior to rotation.
     */
    public <V extends Vector3> void setToShearXBase(V vec) {
        vec.set(xBase);
    }

    /**
     * the default Transform3D implementation is orthonormal,
     * so by default this function will just set @param vec to (0,1,0),
     * but extending (affine) classes can override this to represent the direction
     * and magnitude of the y axis prior to rotation.
     */
    public <V extends Vector3> void setToShearYBase(V vec) {
        vec.set(yBase);
    }

    /**
     * the default Transform3D implementation is orthonormal,
     * so by default this function will just set @param vec to (0,0,1),
     * but extending (affine) classes can override this to represent the direction
     * and magnitude of the z axis prior to rotation.
     */
    public <V extends Vector3> void setToShearZBase(V vec) {
        vec.set(zBase);
    }

    /**
     * sets globalOutput such that the result of
     * this.getLocalOf(globalOutput) == localInput.
     *
     * @param localInput
     * @param globalOutput
     */
    public <T extends ewbik.math.Transform3D> void setToGlobalOf(T localInput, T globalOutput) {
        this.rotation.applyTo(localInput.rotation, globalOutput.rotation);
        this.setToGlobalOf(localInput.translate, globalOutput.translate);
        globalOutput.refreshPrecomputed();
    }

    public <V extends Vector3> void setToGlobalOf(V input, V output) {
        try {
            rotation.applyTo(input, output);
            output.add(this.translate);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public <V extends Vector3> void translateBy(V transBy) {
        this.translate.x += transBy.x;
        this.translate.y += transBy.y;
        this.translate.z += transBy.z;
        updateRays();
    }

    public <V extends Vector3> void translateTo(V newOrigin) {
        this.translate.x = newOrigin.x;
        this.translate.y = newOrigin.y;
        this.translate.z = newOrigin.z;
        updateRays();
    }

    public Ray3D getXRay() {
        return xRay;
    }

    public Ray3D getYRay() {
        return yRay;
    }

    public Ray3D getZRay() {
        return zRay;
    }

    public Vector3 getXHeading() {
        return this.xRay.heading();
    }

    public Vector3 getYHeading() {
        return this.yRay.heading();
    }

    public Vector3 getZHeading() {
        return this.zRay.heading();
    }

    public Vector3 getOrigin() {
        return translate;
    }

    /**
     * @return a precomputed inverse of the rotation represented by this basis
     *         object.
     */
    public Quaternion getInverseRotation() {
        return this.inverseRotation;
    }

    private void updateRays() {
        xRay.setP1(this.translate);
        xRay.p2.set(xBase);
        yRay.setP1(this.translate);
        yRay.p2.set(yBase);
        zRay.setP1(this.translate);
        zRay.p2.set(zBase);

        rotation.applyTo(xRay.p2, xRay.p2);
        rotation.applyTo(yRay.p2, yRay.p2);
        rotation.applyTo(zRay.p2, zRay.p2);

        xRay.p2.add(this.translate);
        yRay.p2.add(this.translate);
        zRay.p2.add(this.translate);
    }

    public String toString() {
        Vector3 xh = xRay.heading().toVec3f();

        Vector3 yh = yRay.heading().toVec3f();

        Vector3 zh = zRay.heading().toVec3f();

        float xMag = xh.mag();
        float yMag = yh.mag();
        float zMag = zh.mag();
        String chirality = "RIGHT";
        String result = "-----------\n"
                + chirality + " handed \n"
                + "origin: " + this.translate.toVec3f() + "\n"
                + "rot Axis: " + this.rotation.getAxis().toVec3f() + ", "
                + "Angle: " + MathUtils.toDegrees(this.rotation.getAngle()) + "\n"
                + "xHead: " + xh + ", mag: " + xMag + "\n"
                + "yHead: " + yh + ", mag: " + yMag + "\n"
                + "zHead: " + zh + ", mag: " + zMag + "\n";

        return result;
    }
}
