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

package EWBIK;

import java.util.ArrayList;
import java.util.HashMap;

public class IKSkeleton {

    public IKNode3D localNode3D;
    public HashMap<IKBone3D, IKShadowNode> boneSegmentMap = new HashMap<IKBone3D, IKShadowNode>();
    public IKShadowNode shadowNode3D;
    public int defaultStabilizingPassCount = 1;
    protected IKNode3D tempWorkingNode3D;
    protected ArrayList<IKBone3D> bones = new ArrayList<IKBone3D>();
    protected HashMap<String, IKBone3D> boneNameMap = new HashMap<String, IKBone3D>();
    protected IKBone3D rootBone;
    protected String name;
    protected int IKIterations = 15;
    protected float dampening = IKMathUtils.toRadians(5f);
    PerformanceStats performance = new PerformanceStats();
    IKNode3D fauxParent;
    boolean debug = true;
    IKBone3D lastDebugBone = null;
    boolean monitorPerformance = false;

    public IKSkeleton(String name) {

        this.localNode3D = new IKNode3D(
                new IKVector3(0, 0, 0), new IKVector3(1, 0, 0), new IKVector3(0, 1, 0), new IKVector3(0, 0, 1), null);
        this.tempWorkingNode3D = IKSkeleton.this.localNode3D.getGlobalCopy();
        this.name = name;
        IKSkeleton.this.createRootBone(IKSkeleton.this.localNode3D.calculateY().heading(),
                IKSkeleton.this.localNode3D.calculateZ().heading(), IKSkeleton.this.name + " : rootBone", 1f,
                IKBone3D.frameType.GLOBAL);
    }

    protected void initializeRootBone(
            IKSkeleton armature,
            IKVector3 tipHeading,
            IKVector3 rollHeading,
            String inputTag,
            float boneHeight,
            IKBone3D.frameType coordinateType) {
        this.rootBone = new IKBone3D(armature.getRootBone(),
                new IKVector3(tipHeading.x, tipHeading.y, tipHeading.z),
                new IKVector3(rollHeading.x, rollHeading.y, rollHeading.z),
                inputTag,
                boneHeight,
                coordinateType);
    }

    /**
     * @return the rootBone of this armature.
     */
    public IKBone3D getRootBone() {
        return rootBone;
    }

    /**
     * @return a reference to the Axes serving as this Armature's coordinate system.
     */
    public IKNode3D localAxes() {
        return this.localNode3D;
    }

    private IKBone3D createRootBone(IKVector3 tipHeading, IKVector3 rollHeading, String boneName,
                                    float boneHeight, IKBone3D.frameType coordinateType) {
        initializeRootBone(this, tipHeading, rollHeading, boneName, boneHeight, coordinateType);
        this.shadowNode3D = new IKShadowNode(rootBone);
        fauxParent = rootBone.localAxes().getGlobalCopy();

        return rootBone;
    }

    /**
     * The armature maintains an internal hashmap of bone name's and their
     * corresponding
     * bone objects. This method should be called by any bone object if ever its
     * name is changed.
     *
     * @param bone
     * @param previousBoneName
     * @param newBoneName
     */
    public void setBoneName(IKBone3D bone, String previousBoneName, String newBoneName) {
        boneNameMap.remove(previousBoneName);
        boneNameMap.put(newBoneName, bone);
    }

    /**
     * this method should be called by any newly created bone object if the armature
     * is
     * to know it exists.
     *
     * @param bone
     */
    public void addToBoneList(IKBone3D Bone) {
        if (!bones.contains(Bone)) {
            bones.add(Bone);
            boneNameMap.put(Bone.getTag(), Bone);
        }
    }

    /**
     * this method should be called by any newly deleted bone object if the armature
     * is
     * to know it no longer exists
     */
    public void removeFromBoneList(IKBone3D Bone) {
        if (bones.contains(Bone)) {
            bones.remove(Bone);
            boneNameMap.remove(Bone);
            this.updateBonechains();
        }
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
    public void updateBonechains() {
        shadowNode3D.updateSegmentedArmature();
        boneSegmentMap.clear();
        recursivelyUpdateBonechainMapFrom(shadowNode3D);
        IKShadowNode.recursivelyCreateHeadingArraysFor(shadowNode3D);
    }

    private void recursivelyUpdateBonechainMapFrom(IKShadowNode startFrom) {
        for (IKBone3D b : startFrom.bonechainList) {
            boneSegmentMap.put(b, startFrom);
        }
        for (IKShadowNode c : startFrom.bonechainChild) {
            recursivelyUpdateBonechainMapFrom(c);
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
        IKBone3D rootBone = this.getRootBone();
        ArrayList<IKBone3D> pinnedBones = new ArrayList<>();
        rootBone.addSelfIfPinned(pinnedBones);

        for (IKBone3D b : pinnedBones) {
            b.notifyAncestorsOfPin(false);
            updateBonechains();
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
    public void IKSolver(IKBone3D bone) {
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
    public void IKSolver(IKBone3D bone, float dampening, int iterations, int stabilizingPasses) {
        performance.startPerformanceMonitor();
        iteratedSolver(bone, dampening, iterations, stabilizingPasses);
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
     * You're encouraged to experiment with this parameter as per your use case, but
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

    private void recursivelyNotifyBonesOfCompletedIKSolution(IKShadowNode startFrom) {
        for (IKBone3D b : startFrom.bonechainList) {
            b.IKUpdateNotification();
        }
        for (IKShadowNode s : startFrom.bonechainChild) {
            recursivelyNotifyBonesOfCompletedIKSolution(s);
        }
    }

    /**
     * @param startFrom
     * @param dampening
     * @param iterations
     */

    public void iteratedSolver(IKBone3D startFrom, float dampening, int iterations,
                               int stabilizationPasses) {
        IKShadowNode armature = boneSegmentMap.get(startFrom);

        if (armature != null) {
            IKShadowNode pinnedRootChain = armature.getPinnedRootChainFromHere();
            armature = pinnedRootChain == null ? armature.getAncestorSegmentContaining(rootBone) : pinnedRootChain;
            if (armature != null && armature.pinnedDescendants.size() > 0) {
                armature.alignSimulationAxesToBones();
                iterations = iterations == -1 ? IKIterations : iterations;
                float totalIterations = iterations;
                stabilizationPasses = stabilizationPasses == -1 ? this.defaultStabilizingPassCount
                        : stabilizationPasses;
                for (int i = 0; i < iterations; i++) {
                    if (!armature.isBasePinned()) {
                        armature.updateOptimalRotationToPinnedDescendants(armature.bonechainRoot, IKMathUtils.PI, true,
                                stabilizationPasses, i, totalIterations);
                        armature.setProcessed(false);
                        for (IKShadowNode s : armature.bonechainChild) {
                            groupedRecursiveBonechainSolver(s, dampening, stabilizationPasses, i, totalIterations);
                        }
                    } else {
                        groupedRecursiveBonechainSolver(armature, dampening, stabilizationPasses, i, totalIterations);
                    }
                }
                armature.recursivelyAlignBonesToSimAxesFrom(armature.bonechainRoot);
                recursivelyNotifyBonesOfCompletedIKSolution(armature);
            }
        }

    }

    public void groupedRecursiveBonechainSolver(IKShadowNode startFrom, float dampening,
                                                int stabilizationPasses,
                                                int iteration, float totalIterations) {
        recursiveBonechainSolver(startFrom, dampening, stabilizationPasses, iteration, totalIterations);
        for (IKShadowNode a : startFrom.pinnedDescendants) {
            for (IKShadowNode c : a.bonechainChild) {
                groupedRecursiveBonechainSolver(c, dampening, stabilizationPasses, iteration, totalIterations);
            }
        }
    }

    /**
     * given a segmented armature, solves each chain from its pinned
     * tips down to its pinned root.
     *
     * @param armature
     */
    public void recursiveBonechainSolver(IKShadowNode armature, float dampening, int stabilizationPasses,
                                         int iteration, float totalIterations) {
        if (armature.bonechainChild == null && !armature.isTipPinned()) {
            return;
        } else if (!armature.isTipPinned()) {
            for (IKShadowNode c : armature.bonechainChild) {
                recursiveBonechainSolver(c, dampening, stabilizationPasses, iteration, totalIterations);
                c.setProcessed(true);
            }
        }
        QCPSolver(armature, dampening, false, stabilizationPasses, iteration, totalIterations);
    }

    private void QCPSolver(
            IKShadowNode chain,
            float dampening,
            boolean inverseWeighting,
            int stabilizationPasses,
            int iteration,
            float totalIterations) {

        debug = false;
        IKBone3D startFrom = debug && lastDebugBone != null ? lastDebugBone : chain.bonechainTip;
        IKBone3D stopAfter = chain.bonechainRoot;

        IKBone3D currentBone = startFrom;

        if (debug && chain.simulatedBones.size() < 2) {
            return;
        }
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

    public void rootwardlyUpdateFalloffCacheFrom(IKBone3D forBone) {
        IKShadowNode current = boneSegmentMap.get(forBone);
        while (current != null) {
            current.createHeadingArrays();
            current = current.getBonechainParent();
        }
    }

    public int getDefaultIterations() {
        return IKIterations;
    }

    public float getDampening() {
        return dampening;
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
