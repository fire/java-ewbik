package EWBIK;

public class IKTransform {

    // The chirality is always right.
    public IKQuaternion rotation = new IKQuaternion();
    public IKQuaternion inverseRotation = new IKQuaternion();
    /**
     * a vector representing the translation of this basis relative to its parent.
     */
    public IKVector3 translate;
    protected IKVector3 xBase = new IKVector3(1, 0, 0);
    protected IKVector3 yBase = new IKVector3(0, 1, 0);
    protected IKVector3 zBase = new IKVector3(0, 0, 1);
    protected IKRay3D xRay;
    protected IKRay3D yRay;
    protected IKRay3D zRay;

    public IKTransform(IKTransform input) {
        translate = input.translate.copy();
        xBase = translate.copy();
        yBase = translate.copy();
        zBase = translate.copy();
        xBase.set(1, 0, 0);
        yBase.set(0, 1, 0);
        zBase.set(0, 0, 1);
        IKVector3 zero = translate.copy();
        zero.set(0, 0, 0);
        xRay = new IKRay3D(zero.copy(), xBase.copy());
        yRay = new IKRay3D(zero.copy(), yBase.copy());
        zRay = new IKRay3D(zero.copy(), zBase.copy());
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
    public IKTransform(IKVector3 origin, IKVector3 x, IKVector3 y, IKVector3 z) {
        this.translate = origin.copy();
        xRay = new IKRay3D(origin.copy(), origin.copy());
        yRay = new IKRay3D(origin.copy(), origin.copy());
        zRay = new IKRay3D(origin.copy(), origin.copy());
        this.set(x.copy(), y.copy(), z.copy());
    }

    public IKTransform copy() {
        return new IKTransform(this);
    }

    private void set(IKVector3 x, IKVector3 y, IKVector3 z) {
        xBase = translate.copy();
        yBase = translate.copy();
        zBase = translate.copy();
        xBase.set(1, 0, 0);
        yBase.set(0, 1, 0);
        zBase.set(0, 0, 1);
        IKVector3 zero = translate.copy();
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
    public void adoptValues(IKTransform in) {
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

    private IKQuaternion createPrioritizedRotation(IKVector3 xHeading, IKVector3 yHeading, IKVector3 zHeading) {
        IKVector3 x_identity = xBase.copy();
        IKVector3 y_identity = yBase.copy();
        IKVector3 z_identity = zBase.copy();
        IKVector3 origin = new IKVector3(0, 0, 0);

        IKVector3[] from = {origin, x_identity, y_identity, z_identity};
        IKVector3[] to = {origin.copy(), xHeading, yHeading, zHeading};
        IKQuaternionBasedCharacteristicPolynomial alignHeads = new IKQuaternionBasedCharacteristicPolynomial(IKMathUtils.FLOAT_ROUNDING_ERROR,
                IKMathUtils.FLOAT_ROUNDING_ERROR);
        alignHeads.setMaxIterations(50);
        IKQuaternion rotation = alignHeads.weightedSuperpose(from, to, null, false);
        return rotation;
    }

    public IKQuaternion getLocalOfRotation(IKQuaternion inRot) {
        IKQuaternion resultNew = inverseRotation.applyTo(inRot).applyTo(rotation);
        return resultNew;
    }

    public void setToLocalOf(IKTransform global_input, IKTransform local_output) {
        local_output.translate = this.getLocalOf(global_input.translate);
        inverseRotation.applyTo(global_input.rotation, local_output.rotation);

        local_output.refreshPrecomputed();
    }

    public void refreshPrecomputed() {
        this.rotation.setToReversion(inverseRotation);
        this.updateRays();
    }

    public IKVector3 getLocalOf(IKVector3 v) {
        IKVector3 result = (IKVector3) v.copy();
        setToLocalOf(v, result);
        return result;
    }

    public void setToLocalOf(IKVector3 input, IKVector3 output) {
        output.set(input);
        output.sub(this.translate);
        inverseRotation.applyTo(output, output);
    }

    public void rotateTo(IKQuaternion newRotation) {
        this.rotation.set(newRotation);
        this.refreshPrecomputed();
    }

    public void rotateBy(IKQuaternion addRotation) {
        addRotation.applyTo(this.rotation, this.rotation);
        this.refreshPrecomputed();
    }

    /**
     * sets globalOutput such that the result of
     * this.getLocalOf(globalOutput) == localInput.
     *
     * @param localInput
     * @param globalOutput
     */
    public void setToGlobalOf(IKTransform localInput, IKTransform globalOutput) {
        this.rotation.applyTo(localInput.rotation, globalOutput.rotation);
        this.setToGlobalOf(localInput.translate, globalOutput.translate);
        globalOutput.refreshPrecomputed();
    }

    public void setToGlobalOf(IKVector3 input, IKVector3 output) {
        try {
            rotation.applyTo(input, output);
            output.add(this.translate);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void translateBy(IKVector3 transBy) {
        this.translate.x += transBy.x;
        this.translate.y += transBy.y;
        this.translate.z += transBy.z;
        updateRays();
    }

    public void translateTo(IKVector3 newOrigin) {
        this.translate.x = newOrigin.x;
        this.translate.y = newOrigin.y;
        this.translate.z = newOrigin.z;
        updateRays();
    }

    public IKRay3D getXRay() {
        return xRay;
    }

    public IKRay3D getYRay() {
        return yRay;
    }

    public IKRay3D getZRay() {
        return zRay;
    }

    public IKVector3 getXHeading() {
        return this.xRay.heading();
    }

    public IKVector3 getYHeading() {
        return this.yRay.heading();
    }

    public IKVector3 getZHeading() {
        return this.zRay.heading();
    }

    public IKVector3 getOrigin() {
        return translate;
    }

    /**
     * @return a precomputed inverse of the rotation represented by this basis
     * object.
     */
    public IKQuaternion getInverseRotation() {
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
        IKVector3 xh = xRay.heading().toVec3f();

        IKVector3 yh = yRay.heading().toVec3f();

        IKVector3 zh = zRay.heading().toVec3f();

        float xMag = xh.mag();
        float yMag = yh.mag();
        float zMag = zh.mag();
        String chirality = "RIGHT";
        String result = "-----------\n"
                + chirality + " handed \n"
                + "origin: " + this.translate.toVec3f() + "\n"
                + "rot Axis: " + this.rotation.getAxis().toVec3f() + ", "
                + "Angle: " + IKMathUtils.toDegrees(this.rotation.getAngle()) + "\n"
                + "xHead: " + xh + ", mag: " + xMag + "\n"
                + "yHead: " + yh + ", mag: " + yMag + "\n"
                + "zHead: " + zh + ", mag: " + zMag + "\n";

        return result;
    }
}
