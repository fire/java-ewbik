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

/**
 * s
 *
 * @author Eron Gjoni
 */
public class ShadowNode3D {
    public Bone3D bonechainRoot;
    public Bone3D bonechainTip;

    public ArrayList<ShadowNode3D> bonechainChild = new ArrayList<ShadowNode3D>();
    public ArrayList<ShadowNode3D> pinnedDescendants = new ArrayList<ShadowNode3D>();
    public HashMap<Bone3D, ShadowBone> simulatedBones = new HashMap<>();
    public ArrayList<Bone3D> bonechainList = new ArrayList<Bone3D>();
    public int distanceToRoot = 0;
    public int chainLength = 0;
    public Node3D debugTipNode3D;
    public Node3D debugTargetNode3D;
    ShadowBone[] pinnedBones;
    boolean includeInIK = true;
    int pinDepth = 1;
    Vector3[] localizedTargetHeadings;
    Vector3[] localizedTipHeadings;
    float[] weights;
    private ShadowNode3D bonechainParent = null;
    private boolean basePinned = false;
    private boolean tipPinned = false;
    private boolean processed = false;
    private boolean simAligned = false;

    public ShadowNode3D(Bone3D rootBone) {
        bonechainRoot = armatureRootBone(rootBone);
        generateArmatureBonechains();
        ensureAxesHeirarchy();
    }

    public ShadowNode3D(ShadowNode3D inputParentSegment, Bone3D inputSegmentRoot) {
        this.bonechainRoot = inputSegmentRoot;
        this.setBonechainParent(inputParentSegment);
        this.distanceToRoot = this.getBonechainParent().distanceToRoot + 1;
        generateArmatureBonechains();
    }

    /**
     * calculates the total number of bases the immediate effectors emanating from
     * this
     * segment reach for (based on modecode set in the IKPin)
     */
    public static void recursivelyCreateHeadingArraysFor(ShadowNode3D s) {
        s.createHeadingArrays();
        for (ShadowNode3D c : s.bonechainChild) {
            recursivelyCreateHeadingArraysFor(c);
        }
    }

    private void generateArmatureBonechains() {
        bonechainChild.clear();
        setTipPinned(false);
        this.setBasePinned(bonechainRoot.getParent() != null && bonechainRoot.getParent().isPinned());

        Bone3D temporaryBonechainTip = this.bonechainRoot;
        this.chainLength = -1;
        while (true) {
            this.chainLength++;
            ArrayList<Bone3D> childrenWithPinnedDescendants = temporaryBonechainTip
                    .returnChildrenWithPinnedDescendants();

            if (childrenWithPinnedDescendants.size() > 1 || (temporaryBonechainTip.isPinned())) {
                if (temporaryBonechainTip.isPinned()) {
                    setTipPinned(true);
                }
                this.bonechainTip = temporaryBonechainTip;

                for (Bone3D childBone : childrenWithPinnedDescendants) {
                    this.bonechainChild.add(new ShadowNode3D(this, childBone));
                }

                break;
            } else if (childrenWithPinnedDescendants.size() == 1) {
                temporaryBonechainTip = childrenWithPinnedDescendants.get(0);
            } else {
                this.bonechainTip = temporaryBonechainTip;
                break;
            }
        }
        updatePinnedDescendants();
        generateSegmentMaps();
    }

    public void createHeadingArrays() {
        ArrayList<ArrayList<Float>> penaltyArray = new ArrayList<ArrayList<Float>>();
        ArrayList<ShadowBone> pinSequence = new ArrayList<>();
        recursivelyCreatePenaltyArray(this, penaltyArray, pinSequence, 1f);
        pinnedBones = new ShadowBone[pinSequence.size()];
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

    void recursivelyCreatePenaltyArray(ShadowNode3D from, ArrayList<ArrayList<Float>> weightArray,
                                       ArrayList<ShadowBone> pinSequence, float currentFalloff) {
        if (currentFalloff == 0) {
            return;
        } else {
            IKPin3D pin = from.bonechainTip.getIKPin();
            if (pin != null) {
                ArrayList<Float> innerWeightArray = new ArrayList<Float>();
                weightArray.add(innerWeightArray);
                byte modeCode = pin.getModeCode();
                innerWeightArray.add(pin.getPinWeight() * currentFalloff);
                float maxPinWeight = 0f;
                if ((modeCode & IKPin3D.XDir) != 0)
                    maxPinWeight = MathUtils.max(maxPinWeight, pin.getXPriority());
                if ((modeCode & IKPin3D.YDir) != 0)
                    maxPinWeight = MathUtils.max(maxPinWeight, pin.getYPriority());
                if ((modeCode & IKPin3D.ZDir) != 0)
                    maxPinWeight = MathUtils.max(maxPinWeight, pin.getZPriority());

                if (maxPinWeight == 0f)
                    maxPinWeight = 1f;

                maxPinWeight = 1f;

                if ((modeCode & IKPin3D.XDir) != 0) {
                    float subTargetWeight = pin.getPinWeight() * (pin.getXPriority() / maxPinWeight) * currentFalloff;
                    innerWeightArray.add(subTargetWeight);
                    innerWeightArray.add(subTargetWeight);
                }
                if ((modeCode & IKPin3D.YDir) != 0) {
                    float subTargetWeight = pin.getPinWeight() * (pin.getYPriority() / maxPinWeight) * currentFalloff;
                    innerWeightArray.add(subTargetWeight);
                    innerWeightArray.add(subTargetWeight);
                }
                if ((modeCode & IKPin3D.ZDir) != 0) {
                    float subTargetWeight = pin.getPinWeight() * (pin.getZPriority() / maxPinWeight) * currentFalloff;
                    innerWeightArray.add(subTargetWeight);
                    innerWeightArray.add(subTargetWeight);
                }
                pinSequence.add(pin.forBone().parentArmature.boneSegmentMap.get(pin.forBone()).simulatedBones
                        .get(pin.forBone()));
            }
            float thisFalloff = pin == null ? 1f : pin.getDepthFalloff();
            for (ShadowNode3D s : from.bonechainChild) {
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
        ShadowNode3D rootStrand = this;
        while (rootStrand.bonechainParent != null) {
            rootStrand = rootStrand.bonechainParent;
        }
        recursivelyEnsureAxesHeirarchyFor(rootStrand.bonechainRoot,
                rootStrand.bonechainRoot.parentArmature.localAxes());
    }

    private void recursivelyEnsureAxesHeirarchyFor(Bone3D b, Node3D parentTo) {
        ShadowNode3D chain = getChainFor(b);
        if (chain != null) {
            ShadowBone sb = chain.simulatedBones.get(b);
            sb.simLocalNode3D.setParent(parentTo);
            sb.simConstraintNode3D.setParent(parentTo);
            for (Bone3D c : b.getChildren()) {
                chain.recursivelyEnsureAxesHeirarchyFor(c, sb.simLocalNode3D);
            }
        }
    }

    public void updateSegmentedArmature() {
        if (this.getBonechainParent() != null) {
            this.getBonechainParent().updateSegmentedArmature();
        } else {
            generateArmatureBonechains();
            ensureAxesHeirarchy();
        }
    }

    public void generateSegmentMaps() {
        for (ShadowBone b : simulatedBones.values()) {
            b.simConstraintNode3D.emancipate();
            b.simLocalNode3D.emancipate();
        }
        simulatedBones.clear();
        bonechainList.clear();

        Bone3D currentBone = bonechainTip;
        Bone3D stopOn = bonechainRoot;
        while (currentBone != null) {
            ShadowBone sb = simulatedBones.get(currentBone);
            if (sb == null) {
                simulatedBones.put(currentBone, new ShadowBone(currentBone));
                bonechainList.add(0, currentBone);
            }

            if (currentBone == stopOn)
                break;
            currentBone = currentBone.getParent();

        }
    }

    public ArrayList<Bone3D> getStrandFromTip(Bone3D pinnedBone) {
        ArrayList<Bone3D> result = new ArrayList<Bone3D>();

        if (pinnedBone.isPinned()) {
            result.add(pinnedBone);
            Bone3D currBone = pinnedBone.getParent();
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

    public ArrayList<ShadowNode3D> returnSegmentPinnedNodes() {
        ArrayList<ShadowNode3D> innerPinnedChains = new ArrayList<>();
        if (this.isTipPinned()) {
            innerPinnedChains.add(this);
        } else {
            for (ShadowNode3D childSegment : bonechainChild) {
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
        manualRMSD /= wsum;
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
            Bone3D forBone,
            float dampening,
            boolean translate,
            int stabilizationPasses,
            int iteration,
            float totalIterations) {

        ShadowBone sb = simulatedBones.get(forBone);
        Node3D thisBoneNode3D = sb.simLocalNode3D;
        thisBoneNode3D.updateGlobal();

        Quaternion bestOrientation = new Quaternion(thisBoneNode3D.getGlobalMBasis().rotation.rotation);
        float newDampening = -1;
        if (forBone.getParent() == null || localizedTargetHeadings.length == 1)
            stabilizationPasses = 0;
        if (translate == true) {
            newDampening = MathUtils.PI;
        }

        updateTargetHeadings(localizedTargetHeadings, weights, thisBoneNode3D);
        upateTipHeadings(localizedTipHeadings, thisBoneNode3D);

        float bestRMSD = 0f;
        QuaternionBasedCharacteristicPolynomial qcpConvergenceCheck = new QuaternionBasedCharacteristicPolynomial(MathUtils.FLOAT_ROUNDING_ERROR, MathUtils.FLOAT_ROUNDING_ERROR);
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
                upateTipHeadings(localizedTipHeadings, thisBoneNode3D);
                newRMSD = getManualMSD(localizedTipHeadings, localizedTargetHeadings, weights);

                if (bestRMSD >= newRMSD) {
                    if (sb.springy) {
                        if (dampening != -1 || totalIterations != sb.forBone.parentArmature.getDefaultIterations()) {
                            float returnfullness = sb.forBone.getConstraint().getPainfullness();
                            float dampenedAngle = sb.forBone.getStiffness() * dampening * returnfullness;
                            float totaliterationssq = totalIterations * totalIterations;
                            float scaledDampenedAngle = dampenedAngle
                                    * ((totaliterationssq - (iteration * iteration)) / totaliterationssq);
                            float cosHalfAngle = MathUtils.cos(0.5f * scaledDampenedAngle);
                            sb.forBone.setAxesToReturnfulled(sb.simLocalNode3D, sb.simConstraintNode3D, cosHalfAngle,
                                    scaledDampenedAngle);
                        } else {
                            sb.forBone.setAxesToReturnfulled(sb.simLocalNode3D, sb.simConstraintNode3D,
                                    sb.cosHalfReturnfullnessDampened[iteration],
                                    sb.halfReturnfullnessDampened[iteration]);
                        }
                        upateTipHeadings(localizedTipHeadings, thisBoneNode3D);
                        newRMSD = getManualMSD(localizedTipHeadings, localizedTargetHeadings, weights);
                    }
                    bestOrientation.set(thisBoneNode3D.getGlobalMBasis().rotation.rotation);
                    bestRMSD = newRMSD;
                    break;
                }
            } else {
                break;
            }
        }
        if (stabilizationPasses > 0) {
            thisBoneNode3D.setGlobalOrientationTo(bestOrientation);
            thisBoneNode3D.markDirty();
        }
    }

    private void updateOptimalRotationToPinnedDescendants(
            ShadowBone sb,
            float dampening,
            boolean translate,
            Vector3[] localizedTipHeadings,
            Vector3[] localizedTargetHeadings,
            float[] weights,
            QuaternionBasedCharacteristicPolynomial qcpOrientationAligner,
            int iteration,
            float totalIterations) {

        qcpOrientationAligner.setMaxIterations(0);
        Quaternion qcpRot = qcpOrientationAligner.weightedSuperpose(localizedTipHeadings, localizedTargetHeadings,
                weights,
                translate);

        Vector3 translateBy = qcpOrientationAligner.getTranslation();
        float boneDamp = sb.cosHalfDampen;

        if (dampening != -1) {
            boneDamp = dampening;
            qcpRot.clampToAngle(boneDamp);
        } else {
            qcpRot.clampToQuadranceAngle(boneDamp);
        }
        sb.simLocalNode3D.rotateBy(qcpRot);

        sb.simLocalNode3D.updateGlobal();

        sb.forBone.setAxesToSnapped(sb.simLocalNode3D, sb.simConstraintNode3D, boneDamp);
        sb.simLocalNode3D.translateByGlobal(translateBy);
        sb.simConstraintNode3D.translateByGlobal(translateBy);

    }

    public void updateTargetHeadings(Vector3[] localizedTargetHeadings, float[] weights,
            Node3D thisBoneNode3D) {

        int hdx = 0;
        for (int i = 0; i < pinnedBones.length; i++) {
            ShadowBone sb = pinnedBones[i];
            IKPin3D pin = sb.forBone.getIKPin();
            Node3D effectorNode3D = pin.forBone.getPinnedAxes();
            effectorNode3D.updateGlobal();
            Vector3 origin = thisBoneNode3D.calculatePosition();
            localizedTargetHeadings[hdx].set(effectorNode3D.calculatePosition()).sub(origin);
            byte modeCode = pin.getModeCode();
            hdx++;

            if ((modeCode & IKPin3D.XDir) != 0) {
                Ray3D xTarget = effectorNode3D.calculateX().getRayScaledBy(weights[hdx]);
                localizedTargetHeadings[hdx].set(xTarget.p2()).sub(origin);
                xTarget.setToInvertedTip(localizedTargetHeadings[hdx + 1]).sub(origin);
                hdx += 2;
            }
            if ((modeCode & IKPin3D.YDir) != 0) {
                Ray3D yTarget = effectorNode3D.calculateY().getRayScaledBy(weights[hdx]);
                localizedTargetHeadings[hdx] = Vector3.sub(yTarget.p2(), origin);
                yTarget.setToInvertedTip(localizedTargetHeadings[hdx + 1]).sub(origin);
                hdx += 2;
            }
            if ((modeCode & IKPin3D.ZDir) != 0) {
                Ray3D zTarget = effectorNode3D.calculateZ().getRayScaledBy(weights[hdx]);
                localizedTargetHeadings[hdx] = Vector3.sub(zTarget.p2(), origin);
                zTarget.setToInvertedTip(localizedTargetHeadings[hdx + 1]).sub(origin);
                hdx += 2;
            }
        }

    }

    public void upateTipHeadings(Vector3[] localizedTipHeadings, Node3D thisBoneNode3D) {
        int hdx = 0;

        for (int i = 0; i < pinnedBones.length; i++) {
            ShadowBone sb = pinnedBones[i];
            IKPin3D pin = sb.forBone.getIKPin();
            Node3D tipNode3D = sb.simLocalNode3D;
            tipNode3D.updateGlobal();
            Vector3 origin = thisBoneNode3D.calculatePosition();
            byte modeCode = pin.getModeCode();

            Node3D targetNode3D = pin.forBone.getPinnedAxes();
            targetNode3D.updateGlobal();
            float scaleBy = thisBoneNode3D.calculatePosition().dist(targetNode3D.calculatePosition());
            hdx++;

            if ((modeCode & IKPin3D.XDir) != 0) {
                Ray3D xTip = tipNode3D.calculateX().getRayScaledBy(scaleBy);
                localizedTipHeadings[hdx].set(xTip.p2()).sub(origin);
                xTip.setToInvertedTip(localizedTipHeadings[hdx + 1]).sub(origin);
                hdx += 2;
            }
            if ((modeCode & IKPin3D.YDir) != 0) {
                Ray3D yTip = tipNode3D.calculateY().getRayScaledBy(scaleBy);
                localizedTipHeadings[hdx].set(yTip.p2()).sub(origin);
                yTip.setToInvertedTip(localizedTipHeadings[hdx + 1]).sub(origin);
                hdx += 2;
            }
            if ((modeCode & IKPin3D.ZDir) != 0) {
                Ray3D zTip = tipNode3D.calculateZ().getRayScaledBy(scaleBy);
                localizedTipHeadings[hdx].set(zTip.p2()).sub(origin);
                zTip.setToInvertedTip(localizedTipHeadings[hdx + 1]).sub(origin);
                hdx += 2;
            }
        }
    }

    /**
     * @param chainMember
     * @return returns the segment chain (pinned or unpinned, doesn't matter) to
     *         which the inputBone belongs.
     */
    public ShadowNode3D getChainFor(Bone3D chainMember) {
        ShadowNode3D result = null;
        if (this.bonechainList.contains(chainMember))
            return this;
        if (this.bonechainParent != null)
            result = this.bonechainParent.getAncestorSegmentContaining(chainMember);
        if (result == null)
            result = getChildSegmentContaining(chainMember);
        return result;
    }

    public ShadowNode3D getChildSegmentContaining(Bone3D b) {
        if (bonechainList.contains(b)) {
            return this;
        } else {
            for (ShadowNode3D s : bonechainChild) {
                ShadowNode3D childContaining = s.getChildSegmentContaining(b);
                if (childContaining != null)
                    return childContaining;
            }
        }
        return null;
    }

    public ShadowNode3D getAncestorSegmentContaining(Bone3D b) {
        if (bonechainList.contains(b))
            return this;
        else if (this.bonechainParent != null)
            return this.bonechainParent.getAncestorSegmentContaining(b);
        else
            return null;
    }

    /**
     * this function travels rootward through the chain hierarchy until it reaches a
     * chain whose base is pinned.
     *
     * @return returns the first chain encountered with a pinned base. Or, null if
     *         it reaches an unpinned armature root.
     */
    public ShadowNode3D getPinnedRootChainFromHere() {

        ShadowNode3D currentChain = this;
        while (currentChain != null) {
            if (currentChain.isBasePinned())
                return currentChain;
            else
                currentChain = currentChain.getBonechainParent();
        }

        return null;

    }

    public Bone3D armatureRootBone(Bone3D rootBone2) {
        Bone3D rootBone = rootBone2;
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

    public ShadowNode3D getBonechainParent() {
        return bonechainParent;
    }

    public void setBonechainParent(ShadowNode3D bonechainParent) {
        this.bonechainParent = bonechainParent;
    }

    /**
     * aligns all simulation axes from this root of this chain up until the pinned
     * tips
     * of any child chains with the constraint an local axes of their corresponding
     * bone.
     */

    public void alignSimulationAxesToBones() {
        if (!this.isBasePinned() && this.getBonechainParent() != null) {
            this.getBonechainParent().alignSimulationAxesToBones();
        } else {
            recursivelyAlignSimAxesOutwardFrom(bonechainRoot, true);
        }
    }

    public void recursivelyAlignSimAxesOutwardFrom(Bone3D b, boolean forceGlobal) {
        ShadowNode3D bChain = getChildSegmentContaining(b);
        if (bChain != null) {
            ShadowBone sb = bChain.simulatedBones.get(b);
            Node3D bNode3D = sb.simLocalNode3D;
            Node3D cNode3D = sb.simConstraintNode3D;
            if (forceGlobal) {
                bNode3D.alignGlobalsTo(b.localAxes());
                bNode3D.markDirty();
                bNode3D.updateGlobal();
                cNode3D.alignGlobalsTo(b.getMajorRotationAxes());
                cNode3D.markDirty();
                cNode3D.updateGlobal();
            } else {
                bNode3D.alignLocalsTo(b.localAxes());
                cNode3D.alignLocalsTo(b.getMajorRotationAxes());
            }
            for (Bone3D bc : b.getChildren()) {
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
    public void recursivelyAlignBonesToSimAxesFrom(Bone3D b) {
        ShadowNode3D chain = b.parentArmature.boneSegmentMap.get(b); // getChainFor(b);
        if (chain != null) {
            ShadowBone sb = chain.simulatedBones.get(b);
            Node3D simulatedLocalNode3D = sb.simLocalNode3D;
            if (b.getParent() != null) {
                b.localAxes().alignOrientationTo(simulatedLocalNode3D);
            } else {
                b.localAxes().alignLocalsTo(simulatedLocalNode3D);
            }
            for (Bone3D bc : b.getChildren()) {
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
    public void getRootMostUnprocessedChains(ArrayList<ShadowNode3D> segments) {
        if (!this.processed) {
            segments.add(this);
        } else {
            if (this.tipPinned)
                return;
            for (ShadowNode3D c : bonechainChild) {
                c.getRootMostUnprocessedChains(segments);
            }
        }
    }

    public void setProcessed(boolean b) {
        this.processed = b;
        if (processed == false) {
            for (ShadowNode3D c : bonechainChild) {
                c.setProcessed(false);
            }
        }
    }

    /**
     * Holds working information for the given bone.
     *
     * @author Eron Gjoni
     */
    public class ShadowBone {
        Bone3D forBone;
        Node3D simLocalNode3D;
        Node3D simConstraintNode3D;
        float cosHalfDampen = 0f;
        float[] cosHalfReturnfullnessDampened;
        float[] halfReturnfullnessDampened;
        boolean springy = false;

        public ShadowBone(Bone3D toSimulate) {
            forBone = toSimulate;
            simLocalNode3D = forBone.localAxes().getGlobalCopy();
            simConstraintNode3D = forBone.getMajorRotationAxes().getGlobalCopy();
            float predamp = 1f - forBone.getStiffness();
            float defaultDampening = forBone.parentArmature.getDampening();
            float dampening = forBone.getParent() == null ? MathUtils.PI : predamp * defaultDampening;
            cosHalfDampen = MathUtils.cos(dampening / 2f);
            Kusudama k = forBone.getConstraint();
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
            Kusudama k = forBone.getConstraint();
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
