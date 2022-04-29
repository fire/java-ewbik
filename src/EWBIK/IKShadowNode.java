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
public class IKShadowNode {
    public IKBone3D bonechainRoot;
    public IKBone3D bonechainTip;

    public ArrayList<IKShadowNode> bonechainChild = new ArrayList<IKShadowNode>();
    public ArrayList<IKShadowNode> pinnedDescendants = new ArrayList<IKShadowNode>();
    public HashMap<IKBone3D, ShadowBone> simulatedBones = new HashMap<>();
    public ArrayList<IKBone3D> bonechainList = new ArrayList<IKBone3D>();
    public int distanceToRoot = 0;
    public int chainLength = 0;
    public IKNode3D debugTipNode3D;
    public IKNode3D debugTargetNode3D;
    ShadowBone[] pinnedBones;
    boolean includeInIK = true;
    int pinDepth = 1;
    IKVector3[] localizedTargetHeadings;
    IKVector3[] localizedTipHeadings;
    float[] weights;
    private IKShadowNode bonechainParent = null;
    private boolean basePinned = false;
    private boolean tipPinned = false;
    private boolean processed = false;
    private boolean simAligned = false;

    public IKShadowNode(IKBone3D rootBone) {
        bonechainRoot = armatureRootBone(rootBone);
        generateArmatureBonechains();
        ensureAxesHeirarchy();
    }

    public IKShadowNode(IKShadowNode inputParentSegment, IKBone3D inputSegmentRoot) {
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
    public static void recursivelyCreateHeadingArraysFor(IKShadowNode s) {
        s.createHeadingArrays();
        for (IKShadowNode c : s.bonechainChild) {
            recursivelyCreateHeadingArraysFor(c);
        }
    }

    private void generateArmatureBonechains() {
        bonechainChild.clear();
        setTipPinned(false);
        this.setBasePinned(bonechainRoot.getParent() != null && bonechainRoot.getParent().isPinned());

        IKBone3D temporaryBonechainTip = this.bonechainRoot;
        this.chainLength = -1;
        while (true) {
            this.chainLength++;
            ArrayList<IKBone3D> childrenWithPinnedDescendants = temporaryBonechainTip
                    .returnChildrenWithPinnedDescendants();

            if (childrenWithPinnedDescendants.size() > 1 || (temporaryBonechainTip.isPinned())) {
                if (temporaryBonechainTip.isPinned()) {
                    setTipPinned(true);
                }
                this.bonechainTip = temporaryBonechainTip;

                for (IKBone3D childBone : childrenWithPinnedDescendants) {
                    this.bonechainChild.add(new IKShadowNode(this, childBone));
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
        localizedTargetHeadings = new IKVector3[totalHeadings];
        localizedTipHeadings = new IKVector3[totalHeadings];
        weights = new float[totalHeadings];
        int currentHeading = 0;
        for (ArrayList<Float> a : penaltyArray) {
            for (Float ad : a) {
                weights[currentHeading] = ad;
                localizedTargetHeadings[currentHeading] = new IKVector3();
                localizedTipHeadings[currentHeading] = new IKVector3();
                currentHeading++;
            }
        }
    }

    void recursivelyCreatePenaltyArray(IKShadowNode from, ArrayList<ArrayList<Float>> weightArray,
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
                    maxPinWeight = IKMathUtils.max(maxPinWeight, pin.getXPriority());
                if ((modeCode & IKPin3D.YDir) != 0)
                    maxPinWeight = IKMathUtils.max(maxPinWeight, pin.getYPriority());
                if ((modeCode & IKPin3D.ZDir) != 0)
                    maxPinWeight = IKMathUtils.max(maxPinWeight, pin.getZPriority());

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
                pinSequence.add(pin.forBone().parent_armature.boneSegmentMap.get(pin.forBone()).simulatedBones
                        .get(pin.forBone()));
            }
            float thisFalloff = pin == null ? 1f : pin.getDepthFalloff();
            for (IKShadowNode s : from.bonechainChild) {
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
        IKShadowNode rootStrand = this;
        while (rootStrand.bonechainParent != null) {
            rootStrand = rootStrand.bonechainParent;
        }
        recursivelyEnsureAxesHeirarchyFor(rootStrand.bonechainRoot,
                rootStrand.bonechainRoot.parent_armature.localAxes());
    }

    private void recursivelyEnsureAxesHeirarchyFor(IKBone3D b, IKNode3D parentTo) {
        IKShadowNode chain = getChainFor(b);
        if (chain != null) {
            ShadowBone sb = chain.simulatedBones.get(b);
            sb.simLocalNode3D.setParent(parentTo);
            sb.simConstraintNode3D.setParent(parentTo);
            for (IKBone3D c : b.getChildren()) {
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

        IKBone3D currentBone = bonechainTip;
        IKBone3D stopOn = bonechainRoot;
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

    public ArrayList<IKBone3D> getStrandFromTip(IKBone3D pinnedBone) {
        ArrayList<IKBone3D> result = new ArrayList<IKBone3D>();

        if (pinnedBone.isPinned()) {
            result.add(pinnedBone);
            IKBone3D currBone = pinnedBone.getParent();
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

    public ArrayList<IKShadowNode> returnSegmentPinnedNodes() {
        ArrayList<IKShadowNode> innerPinnedChains = new ArrayList<>();
        if (this.isTipPinned()) {
            innerPinnedChains.add(this);
        } else {
            for (IKShadowNode childSegment : bonechainChild) {
                innerPinnedChains.addAll(childSegment.returnSegmentPinnedNodes());
            }
        }
        return innerPinnedChains;
    }

    public float getManualMSD(IKVector3[] locTips, IKVector3[] locTargets, float[] weights) {
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
            IKBone3D forBone,
            float dampening,
            boolean translate,
            int stabilizationPasses,
            int iteration,
            float totalIterations) {

        ShadowBone sb = simulatedBones.get(forBone);
        IKNode3D thisBoneNode3D = sb.simLocalNode3D;
        thisBoneNode3D.updateGlobal();

        IKQuaternion bestOrientation = new IKQuaternion(thisBoneNode3D.getGlobalMBasis().rotation.rotation);
        float newDampening = -1;
        if (forBone.getParent() == null || localizedTargetHeadings.length == 1)
            stabilizationPasses = 0;
        if (translate == true) {
            newDampening = IKMathUtils.PI;
        }

        updateTargetHeadings(localizedTargetHeadings, weights, thisBoneNode3D);
        upateTipHeadings(localizedTipHeadings, thisBoneNode3D);

        float bestRMSD = 0f;
        IKQuaternionBasedCharacteristicPolynomial qcpConvergenceCheck = new IKQuaternionBasedCharacteristicPolynomial(IKMathUtils.FLOAT_ROUNDING_ERROR, IKMathUtils.FLOAT_ROUNDING_ERROR);
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
                        if (dampening != -1 || totalIterations != sb.forBone.parent_armature.getDefaultIterations()) {
                            float returnfullness = sb.forBone.getConstraint().getPainfullness();
                            float dampenedAngle = sb.forBone.getStiffness() * dampening * returnfullness;
                            float totaliterationssq = totalIterations * totalIterations;
                            float scaledDampenedAngle = dampenedAngle
                                    * ((totaliterationssq - (iteration * iteration)) / totaliterationssq);
                            float cosHalfAngle = IKMathUtils.cos(0.5f * scaledDampenedAngle);
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
            IKVector3[] localizedTipHeadings,
            IKVector3[] localizedTargetHeadings,
            float[] weights,
            IKQuaternionBasedCharacteristicPolynomial qcpOrientationAligner,
            int iteration,
            float totalIterations) {

        qcpOrientationAligner.setMaxIterations(0);
        IKQuaternion qcpRot = qcpOrientationAligner.weightedSuperpose(localizedTipHeadings, localizedTargetHeadings,
                weights,
                translate);

        IKVector3 translateBy = qcpOrientationAligner.getTranslation();
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

    public void updateTargetHeadings(IKVector3[] localizedTargetHeadings, float[] weights,
                                     IKNode3D thisBoneNode3D) {

        int hdx = 0;
        for (int i = 0; i < pinnedBones.length; i++) {
            ShadowBone sb = pinnedBones[i];
            IKPin3D pin = sb.forBone.getIKPin();
            IKNode3D effectorNode3D = pin.forBone.getPinnedAxes();
            effectorNode3D.updateGlobal();
            IKVector3 origin = thisBoneNode3D.calculatePosition();
            localizedTargetHeadings[hdx].set(effectorNode3D.calculatePosition()).sub(origin);
            byte modeCode = pin.getModeCode();
            hdx++;

            if ((modeCode & IKPin3D.XDir) != 0) {
                IKRay3D xTarget = effectorNode3D.calculateX().getRayScaledBy(weights[hdx]);
                localizedTargetHeadings[hdx].set(xTarget.p2()).sub(origin);
                xTarget.setToInvertedTip(localizedTargetHeadings[hdx + 1]).sub(origin);
                hdx += 2;
            }
            if ((modeCode & IKPin3D.YDir) != 0) {
                IKRay3D yTarget = effectorNode3D.calculateY().getRayScaledBy(weights[hdx]);
                localizedTargetHeadings[hdx] = IKVector3.sub(yTarget.p2(), origin);
                yTarget.setToInvertedTip(localizedTargetHeadings[hdx + 1]).sub(origin);
                hdx += 2;
            }
            if ((modeCode & IKPin3D.ZDir) != 0) {
                IKRay3D zTarget = effectorNode3D.calculateZ().getRayScaledBy(weights[hdx]);
                localizedTargetHeadings[hdx] = IKVector3.sub(zTarget.p2(), origin);
                zTarget.setToInvertedTip(localizedTargetHeadings[hdx + 1]).sub(origin);
                hdx += 2;
            }
        }

    }

    public void upateTipHeadings(IKVector3[] localizedTipHeadings, IKNode3D thisBoneNode3D) {
        int hdx = 0;

        for (int i = 0; i < pinnedBones.length; i++) {
            ShadowBone sb = pinnedBones[i];
            IKPin3D pin = sb.forBone.getIKPin();
            IKNode3D tipNode3D = sb.simLocalNode3D;
            tipNode3D.updateGlobal();
            IKVector3 origin = thisBoneNode3D.calculatePosition();
            byte modeCode = pin.getModeCode();

            IKNode3D targetNode3D = pin.forBone.getPinnedAxes();
            targetNode3D.updateGlobal();
            float scaleBy = thisBoneNode3D.calculatePosition().dist(targetNode3D.calculatePosition());
            hdx++;

            if ((modeCode & IKPin3D.XDir) != 0) {
                IKRay3D xTip = tipNode3D.calculateX().getRayScaledBy(scaleBy);
                localizedTipHeadings[hdx].set(xTip.p2()).sub(origin);
                xTip.setToInvertedTip(localizedTipHeadings[hdx + 1]).sub(origin);
                hdx += 2;
            }
            if ((modeCode & IKPin3D.YDir) != 0) {
                IKRay3D yTip = tipNode3D.calculateY().getRayScaledBy(scaleBy);
                localizedTipHeadings[hdx].set(yTip.p2()).sub(origin);
                yTip.setToInvertedTip(localizedTipHeadings[hdx + 1]).sub(origin);
                hdx += 2;
            }
            if ((modeCode & IKPin3D.ZDir) != 0) {
                IKRay3D zTip = tipNode3D.calculateZ().getRayScaledBy(scaleBy);
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
    public IKShadowNode getChainFor(IKBone3D chainMember) {
        IKShadowNode result = null;
        if (this.bonechainList.contains(chainMember))
            return this;
        if (this.bonechainParent != null)
            result = this.bonechainParent.getAncestorSegmentContaining(chainMember);
        if (result == null)
            result = getChildSegmentContaining(chainMember);
        return result;
    }

    public IKShadowNode getChildSegmentContaining(IKBone3D b) {
        if (bonechainList.contains(b)) {
            return this;
        } else {
            for (IKShadowNode s : bonechainChild) {
                IKShadowNode childContaining = s.getChildSegmentContaining(b);
                if (childContaining != null)
                    return childContaining;
            }
        }
        return null;
    }

    public IKShadowNode getAncestorSegmentContaining(IKBone3D b) {
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
    public IKShadowNode getPinnedRootChainFromHere() {

        IKShadowNode currentChain = this;
        while (currentChain != null) {
            if (currentChain.isBasePinned())
                return currentChain;
            else
                currentChain = currentChain.getBonechainParent();
        }

        return null;

    }

    public IKBone3D armatureRootBone(IKBone3D rootBone2) {
        IKBone3D rootBone = rootBone2;
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

    public IKShadowNode getBonechainParent() {
        return bonechainParent;
    }

    public void setBonechainParent(IKShadowNode bonechainParent) {
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

    public void recursivelyAlignSimAxesOutwardFrom(IKBone3D b, boolean forceGlobal) {
        IKShadowNode bChain = getChildSegmentContaining(b);
        if (bChain != null) {
            ShadowBone sb = bChain.simulatedBones.get(b);
            IKNode3D bNode3D = sb.simLocalNode3D;
            IKNode3D cNode3D = sb.simConstraintNode3D;
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
            for (IKBone3D bc : b.getChildren()) {
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
    public void recursivelyAlignBonesToSimAxesFrom(IKBone3D b) {
        IKShadowNode chain = b.parent_armature.boneSegmentMap.get(b); // getChainFor(b);
        if (chain != null) {
            ShadowBone sb = chain.simulatedBones.get(b);
            IKNode3D simulatedLocalNode3D = sb.simLocalNode3D;
            if (b.getParent() != null) {
                b.localAxes().alignOrientationTo(simulatedLocalNode3D);
            } else {
                b.localAxes().alignLocalsTo(simulatedLocalNode3D);
            }
            for (IKBone3D bc : b.getChildren()) {
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
    public void getRootMostUnprocessedChains(ArrayList<IKShadowNode> segments) {
        if (!this.processed) {
            segments.add(this);
        } else {
            if (this.tipPinned)
                return;
            for (IKShadowNode c : bonechainChild) {
                c.getRootMostUnprocessedChains(segments);
            }
        }
    }

    public void setProcessed(boolean b) {
        this.processed = b;
        if (processed == false) {
            for (IKShadowNode c : bonechainChild) {
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
        IKBone3D forBone;
        IKNode3D simLocalNode3D;
        IKNode3D simConstraintNode3D;
        float cosHalfDampen = 0f;
        float[] cosHalfReturnfullnessDampened;
        float[] halfReturnfullnessDampened;
        boolean springy = false;

        public ShadowBone(IKBone3D toSimulate) {
            forBone = toSimulate;
            simLocalNode3D = forBone.localAxes().getGlobalCopy();
            simConstraintNode3D = forBone.getMajorRotationAxes().getGlobalCopy();
            float predamp = 1f - forBone.getStiffness();
            float defaultDampening = forBone.parent_armature.getDampening();
            float dampening = forBone.getParent() == null ? IKMathUtils.PI : predamp * defaultDampening;
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
            float predamp = 1f - forBone.getStiffness();
            float defaultDampening = forBone.parent_armature.getDampening();
            float dampening = forBone.getParent() == null ? IKMathUtils.PI : predamp * defaultDampening;
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
            float predamp = 1f - forBone.getStiffness();
            float defaultDampening = forBone.parent_armature.getDampening();
            float dampening = forBone.getParent() == null ? IKMathUtils.PI : predamp * defaultDampening;
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

}
