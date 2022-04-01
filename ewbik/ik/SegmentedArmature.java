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
package ewbik.ik;

import ewbik.math.*;
import ewbik.processing.singlePrecision.Kusudama;
import ik.Bone;
import ik.IKPin;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * s
 *
 * @author Eron Gjoni
 */
public class SegmentedArmature {
    public Bone segmentRoot;
    public Bone segmentTip;

    public ArrayList<SegmentedArmature> childSegments = new ArrayList<SegmentedArmature>();
    public ArrayList<SegmentedArmature> pinnedDescendants = new ArrayList<SegmentedArmature>();
    public HashMap<Bone, WorkingBone> simulatedBones = new HashMap<>();
    public ArrayList<Bone> segmentBoneList = new ArrayList<Bone>();
    public int distanceToRoot = 0;
    public int chainLength = 0;
    public AbstractAxes debugTipAxes;
    public AbstractAxes debugTargetAxes;
    WorkingBone[] pinnedBones;
    boolean includeInIK = true;
    int pinDepth = 1;
    Vector3[] localizedTargetHeadings;
    Vector3[] localizedTipHeadings;
    float[] weights;
    private SegmentedArmature parentSegment = null;
    private boolean basePinned = false;
    private boolean tipPinned = false;
    private boolean processed = false;
    private boolean simAligned = false;

    public SegmentedArmature(Bone rootBone) {
        segmentRoot = armatureRootBone(rootBone);
        generateArmatureSegments();
        ensureAxesHeirarchy();
    }

    public SegmentedArmature(SegmentedArmature inputParentSegment, Bone inputSegmentRoot) {
        this.segmentRoot = inputSegmentRoot;
        this.setParentSegment(inputParentSegment);
        this.distanceToRoot = this.getParentSegment().distanceToRoot + 1;
        generateArmatureSegments();
    }

    /**
     * calculates the total number of bases the immediate effectors emanating from
     * this
     * segment reach for (based on modecode set in the IKPin)
     */
    public static void recursivelyCreateHeadingArraysFor(SegmentedArmature s) {
        s.createHeadingArrays();
        for (SegmentedArmature c : s.childSegments) {
            recursivelyCreateHeadingArraysFor(c);
        }
    }

    private void generateArmatureSegments() {
        childSegments.clear();
        // pinnedDescendants.clear();
        setTipPinned(false);
        if (segmentRoot.getParent() != null && segmentRoot.getParent().isPinned())
            this.setBasePinned(true);
        else
            this.setBasePinned(false);

        Bone tempSegmentTip = this.segmentRoot;
        this.chainLength = -1;
        while (true) {
            this.chainLength++;
            ArrayList<Bone> childrenWithPinnedDescendants = tempSegmentTip
                    .returnChildrenWithPinnedDescendants();

            if (childrenWithPinnedDescendants.size() > 1 || (tempSegmentTip.isPinned())) {
                if (tempSegmentTip.isPinned())
                    setTipPinned(true);
                // else tipPinned = false;
                this.segmentTip = tempSegmentTip;

                for (Bone childBone : childrenWithPinnedDescendants) {
                    this.childSegments.add(new SegmentedArmature(this, childBone));
                }

                break;
            } else if (childrenWithPinnedDescendants.size() == 1) {
                tempSegmentTip = childrenWithPinnedDescendants.get(0);
            } else {
                this.segmentTip = tempSegmentTip;
                break;
            }
        }
        updatePinnedDescendants();
        generateSegmentMaps();
    }

    public void createHeadingArrays() {
        ArrayList<ArrayList<Float>> penaltyArray = new ArrayList<ArrayList<Float>>();
        ArrayList<WorkingBone> pinSequence = new ArrayList<>();
        recursivelyCreatePenaltyArray(this, penaltyArray, pinSequence, 1f);
        pinnedBones = new WorkingBone[pinSequence.size()];
        int totalHeadings = 0;
        for (ArrayList<Float> a : penaltyArray) {
            totalHeadings += a.size();
        }
        for (int i = 0; i < pinSequence.size(); i++) {
            pinnedBones[i] = pinSequence.get(i);
        }
        localizedTargetHeadings = new Vector3[totalHeadings];
        localizedTipHeadings = new Vector3[totalHeadings];
        weights = new float[totalHeadings];
        int currentHeading = 0;
        for (ArrayList<Float> a : penaltyArray) {
            for (Float ad : a) {
                weights[currentHeading] = ad;
                localizedTargetHeadings[currentHeading] = new Vector3();
                localizedTipHeadings[currentHeading] = new Vector3();
                currentHeading++;
            }
        }
    }

    void recursivelyCreatePenaltyArray(SegmentedArmature from, ArrayList<ArrayList<Float>> weightArray,
                                       ArrayList<WorkingBone> pinSequence, float currentFalloff) {
        if (currentFalloff == 0) {
            return;
        } else {
            IKPin pin = from.segmentTip.getIKPin();
            if (pin != null) {
                ArrayList<Float> innerWeightArray = new ArrayList<Float>();
                weightArray.add(innerWeightArray);
                byte modeCode = pin.getModeCode();
                innerWeightArray.add(pin.getPinWeight() * currentFalloff);
                float maxPinWeight = 0f;
                if ((modeCode & IKPin.XDir) != 0)
                    maxPinWeight = ewbik.math.MathUtils.max(maxPinWeight, pin.getXPriority());
                if ((modeCode & IKPin.YDir) != 0)
                    maxPinWeight = ewbik.math.MathUtils.max(maxPinWeight, pin.getYPriority());
                if ((modeCode & IKPin.ZDir) != 0)
                    maxPinWeight = ewbik.math.MathUtils.max(maxPinWeight, pin.getZPriority());

                if (maxPinWeight == 0f)
                    maxPinWeight = 1f;

                maxPinWeight = 1f;

                if ((modeCode & IKPin.XDir) != 0) {
                    float subTargetWeight = pin.getPinWeight() * (pin.getXPriority() / maxPinWeight) * currentFalloff;
                    innerWeightArray.add(subTargetWeight);
                    innerWeightArray.add(subTargetWeight);
                }
                if ((modeCode & IKPin.YDir) != 0) {
                    float subTargetWeight = pin.getPinWeight() * (pin.getYPriority() / maxPinWeight) * currentFalloff;
                    innerWeightArray.add(subTargetWeight);
                    innerWeightArray.add(subTargetWeight);
                }
                if ((modeCode & IKPin.ZDir) != 0) {
                    float subTargetWeight = pin.getPinWeight() * (pin.getZPriority() / maxPinWeight) * currentFalloff;
                    innerWeightArray.add(subTargetWeight);
                    innerWeightArray.add(subTargetWeight);
                }
                pinSequence.add(pin.forBone().parentArmature.boneSegmentMap.get(pin.forBone()).simulatedBones
                        .get(pin.forBone()));
            }
            float thisFalloff = pin == null ? 1f : pin.getDepthFalloff();
            for (SegmentedArmature s : from.childSegments) {
                recursivelyCreatePenaltyArray(s, weightArray, pinSequence, currentFalloff * thisFalloff);
            }

        }
    }

    /**
     * Should only be called from the rootmost strand.
     * ensures the proper axes parent relationships
     * for simulatedAxes throughout the SegmentedArmature .
     */
    private void ensureAxesHeirarchy() {
        SegmentedArmature rootStrand = this;
        while (rootStrand.parentSegment != null) {
            rootStrand = rootStrand.parentSegment;
        }
        recursivelyEnsureAxesHeirarchyFor(rootStrand.segmentRoot, rootStrand.segmentRoot.parentArmature.localAxes());
    }

    private void recursivelyEnsureAxesHeirarchyFor(Bone b, AbstractAxes parentTo) {
        SegmentedArmature chain = getChainFor(b);
        if (chain != null) {
            WorkingBone sb = chain.simulatedBones.get(b);
            sb.simLocalAxes.setParent(parentTo);
            sb.simConstraintAxes.setParent(parentTo);
            for (Bone c : b.getChildren()) {
                chain.recursivelyEnsureAxesHeirarchyFor(c, sb.simLocalAxes);
            }
        }
    }

    public void updateSegmentedArmature() {
        if (this.getParentSegment() != null) {
            this.getParentSegment().updateSegmentedArmature();
        } else {
            generateArmatureSegments();
            ensureAxesHeirarchy();
        }
    }

    public void generateSegmentMaps() {
        for (WorkingBone b : simulatedBones.values()) {
            b.simConstraintAxes.emancipate();
            b.simLocalAxes.emancipate();
        }
        simulatedBones.clear();
        segmentBoneList.clear();

        Bone currentBone = segmentTip;
        Bone stopOn = segmentRoot;
        while (currentBone != null) {
            WorkingBone sb = simulatedBones.get(currentBone);
            if (sb == null) {
                simulatedBones.put(currentBone, new WorkingBone(currentBone));
                segmentBoneList.add(0, currentBone);
            }

            if (currentBone == stopOn)
                break;
            currentBone = currentBone.getParent();

        }
    }

    public ArrayList<Bone> getStrandFromTip(Bone pinnedBone) {
        ArrayList<Bone> result = new ArrayList<Bone>();

        if (pinnedBone.isPinned()) {
            result.add(pinnedBone);
            Bone currBone = pinnedBone.getParent();
            // note to self -- try removing the currbone.parent != null condition
            while (currBone != null && currBone.getParent() != null) {
                result.add(currBone);
                if (currBone.getParent().isPinned()) {
                    break;
                }
                currBone = currBone.getParent();
            }
        }

        return result;
    }

    public void updatePinnedDescendants() {
        pinnedDescendants.clear();
        pinnedDescendants = this.returnSegmentPinnedNodes();
    }

    public ArrayList<SegmentedArmature> returnSegmentPinnedNodes() {
        ArrayList<SegmentedArmature> innerPinnedChains = new ArrayList<>();
        if (this.isTipPinned()) {
            innerPinnedChains.add(this);
        } else {
            for (SegmentedArmature childSegment : childSegments) {
                innerPinnedChains.addAll(childSegment.returnSegmentPinnedNodes());
            }
        }
        return innerPinnedChains;
    }

    public float getManualMSD(Vector3[] locTips, Vector3[] locTargets, float[] weights) {
        float manualRMSD = 0f;
        float wsum = 0f;
        for (int i = 0; i < locTargets.length; i++) {
            float xd = locTargets[i].x - locTips[i].x;
            float yd = locTargets[i].y - locTips[i].y;
            float zd = locTargets[i].z - locTips[i].z;
            float magsq = weights[i] * (xd * xd + yd * yd + zd * zd);
            manualRMSD += magsq;
            wsum += weights[i];
        }
        manualRMSD /= wsum;// (float) locTargets.length;
        // manualRMSD = MathUtils.sqrt(manualRMSD);
        return manualRMSD;
    }

    /**
     * @param forBone
     * @param dampening
     * @param translate           set to true if you wish to allow translation in
     *                            addition to rotation of the bone (should only be
     *                            used for unpinned root bones)
     * @param stabilizationPasses If you know that your armature isn't likely to
     *                            succumb to instability in unsolvable
     *                            configurations, leave this value set to 0.
     *                            If you value stability in extreme situations more
     *                            than computational speed, then increase this
     *                            value. A value of 1 will be completely stable, and
     *                            just as fast
     *                            as a value of 0, however, it might result in small
     *                            levels of robotic looking jerk. The higher the
     *                            value, the less jerk there will be (but at
     *                            potentially significant computation cost).
     */
    public void updateOptimalRotationToPinnedDescendants(
            Bone forBone,
            float dampening,
            boolean translate,
            int stabilizationPasses,
            int iteration,
            float totalIterations) {

        WorkingBone sb = simulatedBones.get(forBone);
        AbstractAxes thisBoneAxes = sb.simLocalAxes;
        thisBoneAxes.updateGlobal();

        Quaternion bestOrientation = new Quaternion(thisBoneAxes.getGlobalMBasis().rotation.rotation);
        float newDampening = -1;
        if (forBone.getParent() == null || localizedTargetHeadings.length == 1)
            stabilizationPasses = 0;
        if (translate == true) {
            newDampening = MathUtils.PI;
        }

        updateTargetHeadings(localizedTargetHeadings, weights, thisBoneAxes);
        upateTipHeadings(localizedTipHeadings, thisBoneAxes);

        float bestRMSD = 0f;
        QCP qcpConvergenceCheck = new QCP(MathUtils.FLOAT_ROUNDING_ERROR, MathUtils.FLOAT_ROUNDING_ERROR);
        float newRMSD = 999999f;

        if (stabilizationPasses > 0)
            bestRMSD = getManualMSD(localizedTipHeadings, localizedTargetHeadings, weights);

        for (int i = 0; i < stabilizationPasses + 1; i++) {
            updateOptimalRotationToPinnedDescendants(
                    sb, newDampening,
                    translate,
                    localizedTipHeadings,
                    localizedTargetHeadings,
                    weights,
                    qcpConvergenceCheck,
                    iteration,
                    totalIterations);

            if (stabilizationPasses > 0) {
                // newDampening = dampening == -1 ? sb.forBone.parentArmature.dampening
                upateTipHeadings(localizedTipHeadings, thisBoneAxes);
                newRMSD = getManualMSD(localizedTipHeadings, localizedTargetHeadings, weights);

                if (bestRMSD >= newRMSD) {
                    if (sb.springy) {
                        if (dampening != -1 || totalIterations != sb.forBone.parentArmature.getDefaultIterations()) {
                            float returnfullness = ((Kusudama) sb.forBone.getConstraint()).getPainfullness();
                            float dampenedAngle = sb.forBone.getStiffness() * dampening * returnfullness;
                            float totaliterationssq = totalIterations * totalIterations;
                            float scaledDampenedAngle = dampenedAngle
                                    * ((totaliterationssq - (iteration * iteration)) / totaliterationssq);
                            float cosHalfAngle = MathUtils.cos(0.5f * scaledDampenedAngle);
                            sb.forBone.setAxesToReturnfulled(sb.simLocalAxes, sb.simConstraintAxes, cosHalfAngle,
                                    scaledDampenedAngle);
                        } else {
                            sb.forBone.setAxesToReturnfulled(sb.simLocalAxes, sb.simConstraintAxes,
                                    sb.cosHalfReturnfullnessDampened[iteration],
                                    sb.halfReturnfullnessDampened[iteration]);
                        }
                        upateTipHeadings(localizedTipHeadings, thisBoneAxes);
                        newRMSD = getManualMSD(localizedTipHeadings, localizedTargetHeadings, weights);
                    }
                    bestOrientation.set(thisBoneAxes.getGlobalMBasis().rotation.rotation);
                    bestRMSD = newRMSD;
                    break;
                }
            } else {
                break;
            }
        }
        if (stabilizationPasses > 0) {
            thisBoneAxes.setGlobalOrientationTo(bestOrientation);
            thisBoneAxes.markDirty();
        }
    }

    private void updateOptimalRotationToPinnedDescendants(
            WorkingBone sb,
            float dampening,
            boolean translate,
            Vector3[] localizedTipHeadings,
            Vector3[] localizedTargetHeadings,
            float[] weights,
            QCP qcpOrientationAligner,
            int iteration,
            float totalIterations) {

        qcpOrientationAligner.setMaxIterations(0);
        Quaternion qcpRot = qcpOrientationAligner.weightedSuperpose(localizedTipHeadings, localizedTargetHeadings, weights,
                translate);

        Vector3 translateBy = qcpOrientationAligner.getTranslation();
        float boneDamp = sb.cosHalfDampen;

        if (dampening != -1) {
            boneDamp = dampening;
            qcpRot.clampToAngle(boneDamp);
        } else {
            qcpRot.clampToQuadranceAngle(boneDamp);
        }
        sb.simLocalAxes.rotateBy(qcpRot);

        sb.simLocalAxes.updateGlobal();

        sb.forBone.setAxesToSnapped(sb.simLocalAxes, sb.simConstraintAxes, boneDamp);
        sb.simLocalAxes.translateByGlobal(translateBy);
        sb.simConstraintAxes.translateByGlobal(translateBy);

    }

    public void updateTargetHeadings(Vec3f<?>[] localizedTargetHeadings, float[] weights, AbstractAxes thisBoneAxes) {

        int hdx = 0;
        for (int i = 0; i < pinnedBones.length; i++) {
            WorkingBone sb = pinnedBones[i];
            IKPin pin = sb.forBone.getIKPin();
            AbstractAxes targetAxes = pin.forBone.getPinnedAxes();
            targetAxes.updateGlobal();
            Vec3f<?> origin = thisBoneAxes.origin_();
            localizedTargetHeadings[hdx].set(targetAxes.origin_()).sub(origin);
            byte modeCode = pin.getModeCode();
            hdx++;

            if ((modeCode & IKPin.XDir) != 0) {
                Ray3 xTarget = targetAxes.x_().getRayScaledBy(weights[hdx]);
                localizedTargetHeadings[hdx].set(xTarget.p2()).sub(origin);
                xTarget.setToInvertedTip(localizedTargetHeadings[hdx + 1]).sub(origin);
                hdx += 2;
            }
            if ((modeCode & IKPin.YDir) != 0) {
                Ray3 yTarget = targetAxes.y_().getRayScaledBy(weights[hdx]);
                localizedTargetHeadings[hdx] = Vec3f.sub(yTarget.p2(), origin);
                yTarget.setToInvertedTip(localizedTargetHeadings[hdx + 1]).sub(origin);
                hdx += 2;
            }
            if ((modeCode & IKPin.ZDir) != 0) {
                Ray3 zTarget = targetAxes.z_().getRayScaledBy(weights[hdx]);
                localizedTargetHeadings[hdx] = Vec3f.sub(zTarget.p2(), origin);
                zTarget.setToInvertedTip(localizedTargetHeadings[hdx + 1]).sub(origin);
                hdx += 2;
            }
        }

    }

    public void upateTipHeadings(Vec3f<?>[] localizedTipHeadings, AbstractAxes thisBoneAxes) {
        int hdx = 0;

        for (int i = 0; i < pinnedBones.length; i++) {
            WorkingBone sb = pinnedBones[i];
            IKPin pin = sb.forBone.getIKPin();
            AbstractAxes tipAxes = sb.simLocalAxes;
            tipAxes.updateGlobal();
            Vec3f<?> origin = thisBoneAxes.origin_();
            byte modeCode = pin.getModeCode();

            AbstractAxes targetAxes = pin.forBone.getPinnedAxes();
            targetAxes.updateGlobal();
            float scaleBy = thisBoneAxes.origin_().dist(targetAxes.origin_());
            hdx++;

            if ((modeCode & IKPin.XDir) != 0) {
                Ray3 xTip = tipAxes.x_().getRayScaledBy(scaleBy);
                localizedTipHeadings[hdx].set(xTip.p2()).sub(origin);
                xTip.setToInvertedTip(localizedTipHeadings[hdx + 1]).sub(origin);
                hdx += 2;
            }
            if ((modeCode & IKPin.YDir) != 0) {
                Ray3 yTip = tipAxes.y_().getRayScaledBy(scaleBy);
                localizedTipHeadings[hdx].set(yTip.p2()).sub(origin);
                yTip.setToInvertedTip(localizedTipHeadings[hdx + 1]).sub(origin);
                hdx += 2;
            }
            if ((modeCode & IKPin.ZDir) != 0) {
                Ray3 zTip = tipAxes.z_().getRayScaledBy(scaleBy);
                localizedTipHeadings[hdx].set(zTip.p2()).sub(origin);
                zTip.setToInvertedTip(localizedTipHeadings[hdx + 1]).sub(origin);
                ;
                hdx += 2;
            }
        }
    }

    /**
     * @param chainMember
     * @return returns the segment chain (pinned or unpinned, doesn't matter) to
     * which the inputBone belongs.
     */
    public SegmentedArmature getChainFor(Bone chainMember) {
        // Bone candidate = this.segmentTip;
        SegmentedArmature result = null;
        if (this.segmentBoneList.contains(chainMember))
            return this;
        if (this.parentSegment != null)
            result = this.parentSegment.getAncestorSegmentContaining(chainMember);
        if (result == null)
            result = getChildSegmentContaining(chainMember);
        return result;
    }

    public SegmentedArmature getChildSegmentContaining(Bone b) {
        if (segmentBoneList.contains(b)) {
            return this;
        } else {
            for (SegmentedArmature s : childSegments) {
                SegmentedArmature childContaining = s.getChildSegmentContaining(b);
                if (childContaining != null)
                    return childContaining;
            }
        }
        return null;
    }

    public SegmentedArmature getAncestorSegmentContaining(Bone b) {
        if (segmentBoneList.contains(b))
            return this;
        else if (this.parentSegment != null)
            return this.parentSegment.getAncestorSegmentContaining(b);
        else
            return null;
    }

    /**
     * this function travels rootward through the chain hierarchy until it reaches a
     * chain whose base is pinned.
     *
     * @return returns the first chain encountered with a pinned base. Or, null if
     * it reaches an unpinned armature root.
     */
    public SegmentedArmature getPinnedRootChainFromHere() {

        SegmentedArmature currentChain = this;
        while (currentChain != null) {
            if (currentChain.isBasePinned())
                return currentChain;
            else
                currentChain = currentChain.getParentSegment();
        }

        return null;

    }

    public Bone armatureRootBone(Bone rootBone2) {
        Bone rootBone = rootBone2;
        while (rootBone.getParent() != null) {
            rootBone = rootBone.getParent();
        }
        return rootBone;
    }

    public boolean isTipPinned() {
        return tipPinned;
    }

    public void setTipPinned(boolean tipPinned) {
        this.tipPinned = tipPinned;
    }

    public boolean isBasePinned() {
        return basePinned;
    }

    public void setBasePinned(boolean basePinned) {
        this.basePinned = basePinned;
    }

    public SegmentedArmature getParentSegment() {
        return parentSegment;
    }

    public void setParentSegment(SegmentedArmature parentSegment) {
        this.parentSegment = parentSegment;
    }

    /**
     * aligns all simulation axes from this root of this chain up until the pinned
     * tips
     * of any child chains with the constraint an local axes of their corresponding
     * bone.
     */

    public void alignSimulationAxesToBones() {
        if (!this.isBasePinned() && this.getParentSegment() != null) {
            this.getParentSegment().alignSimulationAxesToBones();
        } else {
            recursivelyAlignSimAxesOutwardFrom(segmentRoot, true);
        }
    }

    public void recursivelyAlignSimAxesOutwardFrom(Bone b, boolean forceGlobal) {
        SegmentedArmature bChain = getChildSegmentContaining(b);
        if (bChain != null) {
            WorkingBone sb = bChain.simulatedBones.get(b);
            AbstractAxes bAxes = sb.simLocalAxes;
            AbstractAxes cAxes = sb.simConstraintAxes;
            if (forceGlobal) {
                bAxes.alignGlobalsTo(b.localAxes());
                bAxes.markDirty();
                bAxes.updateGlobal();
                cAxes.alignGlobalsTo(b.getMajorRotationAxes());
                cAxes.markDirty();
                cAxes.updateGlobal();
            } else {
                bAxes.alignLocalsTo(b.localAxes());
                cAxes.alignLocalsTo(b.getMajorRotationAxes());
            }
            for (Bone bc : b.getChildren()) {
                bChain.recursivelyAlignSimAxesOutwardFrom(bc, false);
            }
        }
    }

    /**
     * aligns this bone and all relevant childBones to their coresponding
     * simulatedAxes (if any) in the SegmentedArmature
     *
     * @param b bone to start from
     */
    public void recursivelyAlignBonesToSimAxesFrom(Bone b) {
        SegmentedArmature chain = b.parentArmature.boneSegmentMap.get(b); // getChainFor(b);
        if (chain != null) {
            WorkingBone sb = chain.simulatedBones.get(b);
            AbstractAxes simulatedLocalAxes = sb.simLocalAxes;
            if (b.getParent() != null) {
                b.localAxes().alignOrientationTo(simulatedLocalAxes);
            } else {
                b.localAxes().alignLocalsTo(simulatedLocalAxes);
            }
            for (Bone bc : b.getChildren()) {
                recursivelyAlignBonesToSimAxesFrom(bc);
            }
            chain.simAligned = false;
            chain.processed = false;
        } else {
            int debug = 0;
        }

    }

    /**
     * populates the given arraylist with the rootmost unprocessed chains of this
     * segmented armature
     * and its descendants up until their pinned tips.
     *
     * @param segments
     */
    public void getRootMostUnprocessedChains(ArrayList<SegmentedArmature> segments) {
        if (!this.processed) {
            segments.add(this);
        } else {
            if (this.tipPinned)
                return;
            for (SegmentedArmature c : childSegments) {
                c.getRootMostUnprocessedChains(segments);
            }
        }
    }

    public void setProcessed(boolean b) {
        this.processed = b;
        if (processed == false) {
            for (SegmentedArmature c : childSegments) {
                c.setProcessed(false);
            }
        }
    }

    /**
     * Holds working information for the given bone.
     *
     * @author Eron Gjoni
     */
    public class WorkingBone {
        Bone forBone;
        AbstractAxes simLocalAxes;
        AbstractAxes simConstraintAxes;
        float cosHalfDampen = 0f;
        float cosHalfReturnfullnessDampened[];
        float halfReturnfullnessDampened[];
        boolean springy = false;

        public WorkingBone(Bone toSimulate) {
            forBone = toSimulate;
            simLocalAxes = forBone.localAxes().getGlobalCopy();
            simConstraintAxes = forBone.getMajorRotationAxes().getGlobalCopy();
            float predamp = 1f - forBone.getStiffness();
            float defaultDampening = forBone.parentArmature.getDampening();
            float dampening = forBone.getParent() == null ? MathUtils.PI : predamp * defaultDampening;
            cosHalfDampen = MathUtils.cos(dampening / 2f);
            Kusudama k = ((Kusudama) forBone.getConstraint());
            if (k != null && k.getPainfullness() != 0f) {
                springy = true;
                populateReturnDampeningIterationArray(k);
            } else {
                springy = false;
            }
        }

        public void updateCosDampening() {
            float predamp = 1f - forBone.getStiffness();
            float defaultDampening = forBone.parentArmature.getDampening();
            float dampening = forBone.getParent() == null ? MathUtils.PI : predamp * defaultDampening;
            cosHalfDampen = MathUtils.cos(dampening / 2f);
            Kusudama k = ((Kusudama) forBone.getConstraint());
            if (k != null && k.getPainfullness() != 0f) {
                springy = true;
                populateReturnDampeningIterationArray(k);
            } else {
                springy = false;
            }
        }

        public void populateReturnDampeningIterationArray(Kusudama k) {
            float predamp = 1f - forBone.getStiffness();
            float defaultDampening = forBone.parentArmature.getDampening();
            float dampening = forBone.getParent() == null ? MathUtils.PI : predamp * defaultDampening;
            float iterations = forBone.parentArmature.getDefaultIterations();
            float returnfullness = k.getPainfullness();
            float falloff = 0.2f;
            halfReturnfullnessDampened = new float[(int) iterations];
            cosHalfReturnfullnessDampened = new float[(int) iterations];
            float iterationspow = MathUtils.pow(iterations, falloff * iterations * returnfullness);
            for (float i = 0; i < iterations; i++) {
                float iterationScalar = ((iterationspow) - MathUtils.pow(i, falloff * iterations * returnfullness))
                        / (iterationspow);
                float iterationReturnClamp = iterationScalar * returnfullness * dampening;
                float cosIterationReturnClamp = MathUtils.cos(iterationReturnClamp / 2f);
                halfReturnfullnessDampened[(int) i] = iterationReturnClamp;
                cosHalfReturnfullnessDampened[(int) i] = cosIterationReturnClamp;
            }
        }
    }

}
