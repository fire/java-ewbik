/*

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

package processing;

import ewbik.asj.LoadManager;
import ewbik.asj.SaveManager;
import ewbik.asj.Saveable;
import ewbik.math.*;
import ewbik.math.MathUtils;
import ik.Bone;
import processing.core.PApplet;
import processing.core.PGraphics;
import processing.core.PMatrix;
import processing.core.PVector;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Note, this class is a concrete implementation of the abstract class
 * Skeleton3D. Please refer to the {@link Skeleton3D
 * Skeleton3D docs.}
 */
public class Skeleton3D implements Saveable {

    public ewbik.processing.sceneGraph.Node3D localNode3D;
    public HashMap<Bone, ewbik.ik.ShadowSkeleton3D> boneSegmentMap = new HashMap<Bone, ewbik.ik.ShadowSkeleton3D>();
    public ewbik.ik.ShadowSkeleton3D shadowSkeleton3D;
    public float IKSolverStability = 0f;
    public int defaultStabilizingPassCount = 1;
    protected ewbik.processing.sceneGraph.Node3D tempWorkingNode3D;
    protected ArrayList<Bone> bones = new ArrayList<Bone>();
    protected HashMap<String, Bone> tagBoneMap = new HashMap<String, Bone>();
    protected Bone rootBone;
    protected String tag;
    protected int IKIterations = 15;
    protected float dampening = MathUtils.toRadians(5f);
    PerformanceStats performance = new PerformanceStats();
    ewbik.processing.sceneGraph.Node3D fauxParent;
    boolean debug = true;
    Bone lastDebugBone = null;
    // debug code -- use to set a minimum distance an effector must move
    // in order to trigger a chain iteration
    float debugMag = 5f;
    ewbik.math.Vector3 lastTargetPos = new ewbik.math.Vector3();
    boolean monitorPerformance = false;
    private boolean abilityBiasing = false;

    // default constructor required for file loading to work
    public Skeleton3D() {
    }

    public Skeleton3D(String name) {

        this.localNode3D = (ewbik.processing.sceneGraph.Node3D) new ewbik.processing.sceneGraph.Node3D(
                new PVector(0, 0, 0), new PVector(1, 0, 0), new PVector(0, 1, 0), new PVector(0, 0, 1), null);
        this.tempWorkingNode3D = Skeleton3D.this.localNode3D.getGlobalCopy();
        this.tag = name;
        Skeleton3D.this.createRootBone(Skeleton3D.this.localNode3D.y_().heading(), Skeleton3D.this.localNode3D.z_().heading(), Skeleton3D.this.tag + " : rootBone", 1f,
                Bone.frameType.GLOBAL);
    }

    protected void initializeRootBone(
            Skeleton3D armature,
            ewbik.math.Vector3 tipHeading,
            ewbik.math.Vector3 rollHeading,
            String inputTag,
            float boneHeight,
            Bone.frameType coordinateType) {
        this.rootBone = new Bone(armature.getRootBone(),
                new PVector(tipHeading.x, tipHeading.y, tipHeading.z),
                new PVector(rollHeading.x, rollHeading.y, rollHeading.z),
                inputTag,
                boneHeight,
                coordinateType);
    }

    public void drawMe(PApplet p, int color, float pinSize) {
        drawMe(p.g, color, pinSize);
    }

    public void drawMe(PGraphics pg, int color, float pinSize) {
        PMatrix localMat = localAxes().getGlobalPMatrix();
        pg.applyMatrix(localMat);
        pg.pushMatrix();
        getRootBone().drawMeAndChildren(pg, color, pinSize);
        pg.popMatrix();
    }

    /**
     * @return the rootBone of this armature.
     */
    public Bone getRootBone() {
        return (Bone) rootBone;
    }

    /**
     * @param tag the tag of the bone object you wish to retrieve
     * @return the bone object corresponding to this tag
     */
    public Bone getBoneTagged(String tag) {
        return (Bone) tagBoneMap.get(tag);
    }

    /**
     * @return a reference to the Axes serving as this Armature's coordinate system.
     */
    public ewbik.processing.sceneGraph.Node3D localAxes() {
        return (ewbik.processing.sceneGraph.Node3D) this.localNode3D;
    }

    /**
     * Set the inputBone as this Armature's Root Bone.
     *
     * @param inputBone
     * @return
     */
    public Bone createRootBone(Bone inputBone) {
        this.rootBone = inputBone;
        this.shadowSkeleton3D = new ewbik.ik.ShadowSkeleton3D(rootBone);
        fauxParent = rootBone.localAxes().getGlobalCopy();

        return rootBone;
    }

    private <V extends ewbik.math.Vector3> Bone createRootBone(V tipHeading, V rollHeading, String inputTag,
                                                               float boneHeight, Bone.frameType coordinateType) {
        initializeRootBone(this, tipHeading, rollHeading, inputTag, boneHeight, coordinateType);
        this.shadowSkeleton3D = new ewbik.ik.ShadowSkeleton3D(rootBone);
        fauxParent = rootBone.localAxes().getGlobalCopy();

        return rootBone;
    }

    /**
     * The default maximum number of radians a bone is allowed to rotate per solver
     * iteration.
     * The lower this value, the more natural the pose results. However, this will
     * the number of iterations
     * the solver requires to converge.
     * <p>
     * !!THIS IS AN EXPENSIVE OPERATION.
     * This updates the entire armature's cache of precomputed quadrance angles.
     * The cache makes things faster in general, but if you need to dynamically
     * change the dampening during a call to IKSolver, use
     * the IKSolver(bone, dampening, iterations, stabilizationPasses) function,
     * which clamps rotations on the fly.
     *
     * @param damp
     */
    public void setDefaultDampening(float damp) {
        this.dampening = MathUtils.min(MathUtils.PI * 3f,
                MathUtils.max(MathUtils.abs(Float.MIN_VALUE), MathUtils.abs(damp)));
        updateArmatureSegments();
    }

    /**
     * (warning, this function is untested)
     *
     * @return all bones belonging to this armature.
     */
    public ArrayList<? extends Bone> getBoneList() {
        this.bones.clear();
        rootBone.addDescendantsToArmature();
        return bones;
    }

    /**
     * The armature maintains an internal hashmap of bone name's and their
     * corresponding
     * bone objects. This method should be called by any bone object if ever its
     * name is changed.
     *
     * @param bone
     * @param previousTag
     * @param newTag
     */
    public void updateBoneTag(Bone bone, String previousTag, String newTag) {
        tagBoneMap.remove(previousTag);
        tagBoneMap.put(newTag, bone);
    }

    /**
     * this method should be called by any newly created bone object if the armature
     * is
     * to know it exists.
     *
     * @param bone
     */
    public void addToBoneList(Bone Bone) {
        if (!bones.contains(Bone)) {
            bones.add(Bone);
            tagBoneMap.put(Bone.getTag(), Bone);
        }
    }

    /**
     * this method should be called by any newly deleted bone object if the armature
     * is
     * to know it no longer exists
     */
    public void removeFromBoneList(Bone Bone) {
        if (bones.contains(Bone)) {
            bones.remove(Bone);
            tagBoneMap.remove(Bone);
            this.updateArmatureSegments();
        }
    }

    /**
     * @return the user specified tag String for this armature.
     */
    public String getTag() {
        return this.tag;
    }

    /**
     * @param A user specified tag string for this armature.
     */
    public void setTag(String newTag) {
        this.tag = newTag;
    }

    /**
     * this method should be called whenever a bone
     * in this armature has been pinned or unpinned.
     * <p>
     * for the most part, the abstract classes call this when necessary.
     * But if you are extending classes more than you would reasonably expect
     * this library to reasonably expect and getting weird results, you might try
     * calling
     * this method after making any substantial structural changes to the armature.
     */
    public void updateArmatureSegments() {
        shadowSkeleton3D.updateSegmentedArmature();
        boneSegmentMap.clear();
        recursivelyUpdateBoneSegmentMapFrom(shadowSkeleton3D);
        ewbik.ik.ShadowSkeleton3D.recursivelyCreateHeadingArraysFor(shadowSkeleton3D);
    }

    private void recursivelyUpdateBoneSegmentMapFrom(ewbik.ik.ShadowSkeleton3D startFrom) {
        for (Bone b : startFrom.segmentBoneList) {
            boneSegmentMap.put(b, startFrom);
        }
        for (ewbik.ik.ShadowSkeleton3D c : startFrom.childSegments) {
            recursivelyUpdateBoneSegmentMapFrom(c);
        }
    }

    /**
     * If you have created some sort of save / load system
     * for your armatures which might make it difficult to notify the armature
     * when a pin has been enabled on a bone, you can call this function after
     * all bones and pins have been instantiated and associated with one another
     * to index all of the pins on the armature.
     */
    public void refreshArmaturePins() {
        Bone rootBone = this.getRootBone();
        ArrayList<Bone> pinnedBones = new ArrayList<>();
        rootBone.addSelfIfPinned(pinnedBones);

        for (Bone b : pinnedBones) {
            b.notifyAncestorsOfPin(false);
            updateArmatureSegments();
        }
    }

    /**
     * automatically solves the IK system of this armature from the
     * given bone using the armature's default IK parameters.
     * <p>
     * You can specify these using the setDefaultIterations() setDefaultIKType() and
     * setDefaultDampening() methods.
     * The library comes with some defaults already set, so you can more or less use
     * this method out of the box if
     * you're just testing things out.
     *
     * @param bone
     */
    public void IKSolver(Bone bone) {
        IKSolver(bone, -1, -1, -1);
    }

    /**
     * automatically solves the IK system of this armature from the
     * given bone using the given parameters.
     *
     * @param bone
     * @param dampening         dampening angle in radians. Set this to -1 if you
     *                          want to use the armature's default.
     * @param iterations        number of iterations to run. Set this to -1 if you
     *                          want to use the armature's default.
     * @param stabilizingPasses number of stabilization passes to run. Set this to
     *                          -1 if you want to use the armature's default.
     */
    public void IKSolver(Bone bone, float dampening, int iterations, int stabilizingPasses) {
        performance.startPerformanceMonitor();
        iteratedImprovedSolver(bone, dampening, iterations, stabilizingPasses);// (bone, dampening, iterations);
        performance.solveFinished(iterations == -1 ? this.IKIterations : iterations);
    }

    /**
     * The solver tends to be quite stable whenever a pose is reachable (or
     * unreachable but without excessive contortion).
     * However, in cases of extreme unreachability (due to excessive contortion on
     * orientation constraints), the solution might fail to stabilize, resulting in
     * an undulating
     * motion.
     * <p>
     * Setting this parameter to "1" will prevent such undulations, with a
     * negligible cost to performance. Setting this parameter to a value higher than
     * 1 will offer minor
     * benefits in pose quality in situations that would otherwise be prone to
     * instability, however, it will do so at a significant performance cost.
     * <p>
     * You're encourage to experiment with this parameter as per your use case, but
     * you may find the following guiding principles helpful:
     * <ul>
     * <li>
     * If your armature doesn't have any constraints, then leave this parameter set
     * to 0.
     * </li>
     * <li>
     * If your armature doesn't make use of orientation aware pins (x,y,and,z
     * direction pin priorities are set to 0) the leave this parameter set to 0.
     * </li>
     * <li>
     * If your armature makes use of orientation aware pins and orientation
     * constraints, then set this parameter to 1
     * </li>
     * <li>
     * If your armature makes use of orientation aware pins and orientation
     * constraints, but speed is of the highest possible priority, then set this
     * parameter to 0
     * </li>
     * </ul>
     *
     * @param passCount
     */
    public void setDefaultStabilizingPassCount(int passCount) {
        defaultStabilizingPassCount = passCount;
    }

    private void recursivelyNotifyBonesOfCompletedIKSolution(ewbik.ik.ShadowSkeleton3D startFrom) {
        for (Bone b : startFrom.segmentBoneList) {
            b.IKUpdateNotification();
        }
        for (ewbik.ik.ShadowSkeleton3D s : startFrom.childSegments) {
            recursivelyNotifyBonesOfCompletedIKSolution(s);
        }
    }

    /**
     * @param startFrom
     * @param dampening
     * @param iterations
     */

    public void iteratedImprovedSolver(Bone startFrom, float dampening, int iterations,
                                       int stabilizationPasses) {
        ewbik.ik.ShadowSkeleton3D armature = boneSegmentMap.get(startFrom);

        if (armature != null) {
            ewbik.ik.ShadowSkeleton3D pinnedRootChain = armature.getPinnedRootChainFromHere();
            armature = pinnedRootChain == null ? armature.getAncestorSegmentContaining(rootBone) : pinnedRootChain;
            if (armature != null && armature.pinnedDescendants.size() > 0) {
                armature.alignSimulationAxesToBones();
                iterations = iterations == -1 ? IKIterations : iterations;
                float totalIterations = iterations;
                stabilizationPasses = stabilizationPasses == -1 ? this.defaultStabilizingPassCount
                        : stabilizationPasses;
                for (int i = 0; i < iterations; i++) {
                    if (!armature.isBasePinned()) {
                        armature.updateOptimalRotationToPinnedDescendants(armature.segmentRoot, MathUtils.PI, true,
                                stabilizationPasses, i, totalIterations);
                        armature.setProcessed(false);
                        for (ewbik.ik.ShadowSkeleton3D s : armature.childSegments) {
                            groupedRecursiveSegmentSolver(s, dampening, stabilizationPasses, i, totalIterations);
                        }
                    } else {
                        groupedRecursiveSegmentSolver(armature, dampening, stabilizationPasses, i, totalIterations);
                    }
                }
                armature.recursivelyAlignBonesToSimAxesFrom(armature.segmentRoot);
                recursivelyNotifyBonesOfCompletedIKSolution(armature);
            }
        }

    }

    public void groupedRecursiveSegmentSolver(ewbik.ik.ShadowSkeleton3D startFrom, float dampening, int stabilizationPasses,
                                              int iteration, float totalIterations) {
        recursiveSegmentSolver(startFrom, dampening, stabilizationPasses, iteration, totalIterations);
        for (ewbik.ik.ShadowSkeleton3D a : startFrom.pinnedDescendants) {
            for (ewbik.ik.ShadowSkeleton3D c : a.childSegments) {
                groupedRecursiveSegmentSolver(c, dampening, stabilizationPasses, iteration, totalIterations);
            }
        }
    }

    /**
     * given a segmented armature, solves each chain from its pinned
     * tips down to its pinned root.
     *
     * @param armature
     */
    public void recursiveSegmentSolver(ewbik.ik.ShadowSkeleton3D armature, float dampening, int stabilizationPasses,
                                       int iteration, float totalIterations) {
        if (armature.childSegments == null && !armature.isTipPinned()) {
            return;
        } else if (!armature.isTipPinned()) {
            for (ewbik.ik.ShadowSkeleton3D c : armature.childSegments) {
                recursiveSegmentSolver(c, dampening, stabilizationPasses, iteration, totalIterations);
                c.setProcessed(true);
            }
        }
        QCPSolver(armature, dampening, false, stabilizationPasses, iteration, totalIterations);
    }

    private void QCPSolver(
            ewbik.ik.ShadowSkeleton3D chain,
            float dampening,
            boolean inverseWeighting,
            int stabilizationPasses,
            int iteration,
            float totalIterations) {

        debug = false;
        Bone startFrom = debug && lastDebugBone != null ? lastDebugBone : chain.segmentTip;
        Bone stopAfter = chain.segmentRoot;

        Bone currentBone = startFrom;

        if (debug && chain.simulatedBones.size() < 2) {
        } else {
            while (currentBone != null) {
                if (!currentBone.getIKOrientationLock()) {
                    chain.updateOptimalRotationToPinnedDescendants(currentBone, dampening, false, stabilizationPasses,
                            iteration, totalIterations);
                }
                if (currentBone == stopAfter)
                    currentBone = null;
                else
                    currentBone = currentBone.getParent();

                if (debug) {
                    lastDebugBone = currentBone;
                    break;
                }
            }
        }
    }

    public void rootwardlyUpdateFalloffCacheFrom(Bone forBone) {
        ewbik.ik.ShadowSkeleton3D current = boneSegmentMap.get(forBone);
        while (current != null) {
            current.createHeadingArrays();
            current = current.getParentSegment();
        }
    }

    public boolean getAbilityBiasing() {
        return abilityBiasing;
    }

    /**
     * currently unused
     *
     * @param enabled
     */
    public void setAbilityBiasing(boolean enabled) {
        abilityBiasing = enabled;
    }

    /**
     * returns the rotation that would bring the right-handed orthonormal axes of a
     * into alignment with b
     *
     * @param a
     * @param b
     * @return
     */
    public ewbik.math.Quaternion getRotationBetween(ewbik.processing.sceneGraph.Node3D a, ewbik.processing.sceneGraph.Node3D b) {
        return new ewbik.math.Quaternion(a.x_().heading(), a.y_().heading(), b.x_().heading(), b.y_().heading());
    }

    public int getDefaultIterations() {
        return IKIterations;
    }

    /**
     * The default number of iterations to run over this armature whenever
     * IKSolver() is called.
     * The higher this value, the more likely the Armature is to have converged on a
     * solution when
     * by the time it returns. However, it will take longer to return (linear cost)
     *
     * @param iter
     */
    public void setDefaultIterations(int iter) {
        this.IKIterations = iter;
        updateArmatureSegments();
    }

    public float getDampening() {
        return dampening;
    }

    public void setPerformanceMonitor(boolean state) {
        monitorPerformance = state;
    }

    @Override
    public void makeSaveable(SaveManager saveManager) {
        saveManager.addToSaveState(this);
        if (this.localAxes().getParentAxes() != null)
            this.localAxes().getParentAxes().makeSaveable(saveManager);
        else
            this.localAxes().makeSaveable(saveManager);
        this.rootBone.makeSaveable(saveManager);
    }

    @Override
    public ewbik.asj.data.JSONObject getSaveJSON(SaveManager saveManager) {
        ewbik.asj.data.JSONObject saveJSON = new ewbik.asj.data.JSONObject();
        saveJSON.setString("identityHash", this.getIdentityHash());
        saveJSON.setString("localAxes", localAxes().getIdentityHash());
        saveJSON.setString("rootBone", getRootBone().getIdentityHash());
        saveJSON.setInt("defaultIterations", getDefaultIterations());
        saveJSON.setFloat("dampening", this.getDampening());
        // saveJSON.setBoolean("inverseWeighted", this.isInverseWeighted());
        saveJSON.setString("tag", this.getTag());
        return saveJSON;
    }

    public void loadFromJSONObject(ewbik.asj.data.JSONObject j, LoadManager l) {
        try {
            this.localNode3D = l.getObjectFor(ewbik.processing.sceneGraph.Node3D.class, j, "localAxes");
            this.rootBone = l.getObjectFor(Bone.class, j, "rootBone");
            this.IKIterations = j.getInt("defaultIterations");
            this.dampening = j.getFloat("dampening");
            this.tag = j.getString("tag");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void notifyOfSaveIntent(SaveManager saveManager) {
        this.makeSaveable(saveManager);
    }

    @Override
    public void notifyOfSaveCompletion(SaveManager saveManager) {
        // TODO Auto-generated method stub

    }

    @Override
    public void notifyOfLoadCompletion() {
        this.createRootBone(rootBone);
        refreshArmaturePins();
        updateArmatureSegments();
    }

    @Override
    public boolean isLoading() {
        return false;
    }

    @Override
    public void setLoading(boolean loading) {
    }

    public class PerformanceStats {
        int timedCalls = 0;
        int benchmarkWindow = 60;
        int iterationCount = 0;
        float averageSolutionTime = 0;
        float averageIterationTime = 0;
        int solutionCount = 0;
        float iterationsPerSecond = 0f;
        long totalSolutionTime = 0;

        long startTime = 0;

        public void startPerformanceMonitor() {
            if (monitorPerformance) {
                if (timedCalls > benchmarkWindow) {
                    performance.resetPerformanceStat();
                }
                startTime = System.nanoTime();
            }
        }

        public void solveFinished(int iterations) {
            if (monitorPerformance) {
                totalSolutionTime += System.nanoTime() - startTime;
                solutionCount++;
                iterationCount += iterations;

                if (timedCalls > benchmarkWindow) {
                    timedCalls = 0;
                    performance.printStats();
                }
                timedCalls++;
            }
        }

        public void resetPerformanceStat() {
            startTime = 0;
            iterationCount = 0;
            averageSolutionTime = 0;
            solutionCount = 0;
            iterationsPerSecond = 0f;
            totalSolutionTime = 0;
            averageIterationTime = 0;
        }

        public void printStats() {
            averageSolutionTime = (float) (totalSolutionTime / solutionCount) / 1000000f;
            averageIterationTime = (float) (totalSolutionTime / iterationCount) / 1000000f;
            System.out.println("solution time average: ");
            System.out.println("per call = " + (averageSolutionTime) + "ms");
            System.out.println("per iteration = " + (averageIterationTime) + "ms \n");
        }

    }
}
