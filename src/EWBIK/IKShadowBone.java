package EWBIK;

/**
 * Holds working information for the given bone.
 *
 * @author Eron Gjoni
 */
public class IKShadowBone {
    IKBone3D forBone;
    IKNode3D simLocalNode3D;
    IKNode3D simConstraintNode3D;
    float cosHalfDampen = 0f;
    float[] cosHalfReturnfullnessDampened;
    float[] halfReturnfullnessDampened;
    boolean springy = false;

    public IKShadowBone(IKBone3D toSimulate) {
        forBone = toSimulate;
        simLocalNode3D = forBone.localAxes().getGlobalCopy();
        simConstraintNode3D = forBone.getMajorRotationAxes().getGlobalCopy();
        float predampening = 1f - forBone.getStiffness();
        float defaultDampening = forBone.parent_armature.getDampening();
        float dampening = forBone.getParent() == null ? IKMathUtils.PI : predampening * defaultDampening;
        cosHalfDampen = IKMathUtils.cos(dampening / 2f);
        IKKusudama k = forBone.getConstraint();
        if (k != null && k.getPainfullness() != 0f) {
            springy = true;
            populateReturnDampeningIterationArray(k);
        } else {
            springy = false;
        }
    }

    public void updateCosDampening() {
        float predampening = 1f - forBone.getStiffness();
        float defaultDampening = forBone.parent_armature.getDampening();
        float dampening = forBone.getParent() == null ? IKMathUtils.PI : predampening * defaultDampening;
        cosHalfDampen = IKMathUtils.cos(dampening / 2f);
        IKKusudama k = forBone.getConstraint();
        if (k != null && k.getPainfullness() != 0f) {
            springy = true;
            populateReturnDampeningIterationArray(k);
        } else {
            springy = false;
        }
    }

    public void populateReturnDampeningIterationArray(IKKusudama k) {
        float predampening = 1f - forBone.getStiffness();
        float defaultDampening = forBone.parent_armature.getDampening();
        float dampening = forBone.getParent() == null ? IKMathUtils.PI : predampening * defaultDampening;
        float iterations = forBone.parent_armature.getDefaultIterations();
        float returnfullness = k.getPainfullness();
        float falloff = 0.2f;
        halfReturnfullnessDampened = new float[(int) iterations];
        cosHalfReturnfullnessDampened = new float[(int) iterations];
        float iterationspow = IKMathUtils.pow(iterations, falloff * iterations * returnfullness);
        for (float i = 0; i < iterations; i++) {
            float iterationScalar = ((iterationspow) - IKMathUtils.pow(i, falloff * iterations * returnfullness))
                    / (iterationspow);
            float iterationReturnClamp = iterationScalar * returnfullness * dampening;
            float cosIterationReturnClamp = IKMathUtils.cos(iterationReturnClamp / 2f);
            halfReturnfullnessDampened[(int) i] = iterationReturnClamp;
            cosHalfReturnfullnessDampened[(int) i] = cosIterationReturnClamp;
        }
    }
}
