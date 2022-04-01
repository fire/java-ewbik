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

import ewbik.math.AbstractAxes;
import ewbik.math.Vec3f;
import ewbik.processing.sceneGraph.Axes;
import ik.Bone;
import processing.core.PApplet;
import processing.core.PGraphics;
import processing.core.PMatrix;
import processing.core.PVector;

/**
 * Note, this class is a concrete implementation of the abstract class
 * AbstractSkeleton3D. Please refer to the {@link ewbik.ik.AbstractSkeleton3D
 * AbstractSkeleton3D docs.}
 */
public class Skeleton3D extends ewbik.ik.AbstractSkeleton3D {

    // default constructor required for file loading to work
    public Skeleton3D() {
    }

    public Skeleton3D(String name) {
        super(new Axes(
                new PVector(0, 0, 0), new PVector(1, 0, 0), new PVector(0, 1, 0), new PVector(0, 0, 1), null), name);
    }

    @Override
    protected void initializeRootBone(
            ewbik.ik.AbstractSkeleton3D armature,
            Vec3f<?> tipHeading,
            Vec3f<?> rollHeading,
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

    @Override
    public Bone getRootBone() {
        return (Bone) rootBone;
    }

    @Override
    public Bone getBoneTagged(String tag) {
        return (Bone) tagBoneMap.get(tag);
    }

    @Override
    public Axes localAxes() {
        return (Axes) super.localAxes();
    }

}
