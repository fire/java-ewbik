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

package ewbik.processing.sceneGraph;

import ewbik.math.*;
import processing.core.PGraphics;
import processing.core.PMatrix;
import processing.core.PMatrix3D;
import processing.core.PVector;

public class Axes extends Transform3D {
    public static int renderMode = 1;


    public Axes(AbstractBasis b, AbstractAxes parent) {
        super(b, parent);
    }


    /**
     * @param origin              the center of this axes basis. The basis vector parameters will be automatically ADDED to the origin in order to create this basis vector.
     * @param inX                 the direction of the X basis vector in global coordinates, given as an offset from this base's origin in global coordinates.
     * @param inY                 the direction of the Y basis vector in global coordinates, given as an offset from this base's origin in global coordinates.
     * @param inZ                 the direction of the Z basis vector in global coordinates, given as an offset from this base's origin in global coordinates.
     * @param forceOrthoNormality
     */
    public Axes(PVector origin,
                PVector inX,
                PVector inY,
                PVector inZ,
                AbstractAxes parent) {

        super(
                toVector3(origin),
                toVector3(inX),
                toVector3(inY),
                toVector3(inZ),
                parent
        );
    }

    public Axes(Vec3f<?> origin,
                Vec3f<?> x,
                Vec3f<?> y,
                Vec3f<?> z) {
        this(origin, x, y, z, true, null);
    }


    public Axes() {
        super(
                new Vector3(0, 0, 0),
                new Vector3(1, 0, 0),
                new Vector3(0, 1, 0),
                new Vector3(0, 0, 1),
                (AbstractAxes) null
        );
    }

    public Axes(Vec3f<?> origin, Vec3f<?> x, Vec3f<?> y, Vec3f<?> z, boolean forceOrthoNormality, AbstractAxes parent) {
        super(origin, x, y, z, parent);
    }

    /**
     * conversion functions. Replace these with functions that convert to and from your
     * framework's native vector and ray representations.
     */
    //////////////////////////////////////////////////////////////////////////////////////////////
    public static PVector toPVector(Vec3f<?> sv) {
        return new PVector(sv.x, sv.y, sv.z);
    }

    public static void toDVector(Vec3f<?> sv, PVector storeIn) {
        storeIn.x = sv.x;
        storeIn.y = sv.y;
        storeIn.z = sv.z;
    }


    public static Vector3 toVector3(PVector ev) {
        return new Vector3(ev.x, ev.y, ev.z);
    }

    //////////////////// END OF CONVERSION FUNCTIONS

    public PVector origin() {
        return toPVector(this.origin_());
    }


    ///WRAPPER FUNCTIONS. Basically just find + replace these with the appropriate class names and conversion functions above if you need them
    //and you should be good to go.


    @Override
    public Axes getGlobalCopy() {
        this.updateGlobal();
        return new Axes(getGlobalMBasis(), this.getParentAxes());
    }


    public PVector getGlobalOf(PVector local_input) {
        return toPVector(
                super.getGlobalOf(
                        toVector3(local_input))
        );
    }

    public PVector setToGlobalOf(PVector local_input) {
        return toPVector(
                super.setToGlobalOf(
                        toVector3(local_input)
                )
        );
    }

    public void setToGlobalOf(PVector local_input, PVector global_output) {
        toDVector(
                super.setToGlobalOf(
                        toVector3(local_input)
                ),
                global_output
        );
    }

    public void translateByLocal(PVector translate) {
        super.translateByLocal(
                toVector3(translate)
        );
    }

    public void translateByGlobal(PVector translate) {
        super.translateByGlobal(
                toVector3(translate)
        );
    }

    public void translateTo(PVector translate, boolean slip) {
        super.translateTo(
                toVector3(translate),
                false
        );

    }

    public void translateTo(PVector translate) {
        super.translateTo(
                toVector3(translate)
        );
    }

    public void rotateAboutX(float radians) {
        super.rotateAboutX(radians, true);
    }

    public void rotateAboutY(float radians) {
        super.rotateAboutY(radians, true);
    }

    public void rotateAboutZ(float radians) {
        super.rotateAboutZ(radians, true);
    }

    public PVector getOrigin() {
        return toPVector(origin_());
    }

    public PVector getLocalOf(PVector global_input) {
        return toPVector(
                super.getLocalOf(
                        toVector3(global_input)
                )
        );
    }

    public PVector setToLocalOf(PVector global_input) {
        toDVector(
                super.setToLocalOf(
                        toVector3(global_input)
                ),
                global_input
        );
        return global_input;
    }

    public void setToLocalOf(PVector global_input, PVector local_output) {
        Vector3 tempVec = new Vector3();
        super.setToLocalOf(
                toVector3(global_input),
                tempVec
        );
        toDVector(
                tempVec,
                local_output
        );
    }

    //////////////////////// End of wrapper functions

    float[][] outMatLocal = new float[4][4];
    float[][] outMatGlobal = new float[4][4];

    private void updateMatrix(AbstractBasis b, float[][] outputMatrix) {
        b.refreshPrecomputed();

        Vec3f<?> x = b.getXHeading();
        Vec3f<?> y = b.getYHeading();
        Vec3f<?> z = b.getZHeading();

        Vec3f<?> origin = b.getOrigin();

        outputMatrix[0][0] = x.x;
        outputMatrix[0][1] = x.y;
        outputMatrix[0][2] = x.z;

        outputMatrix[1][0] = y.x;
        outputMatrix[1][1] = y.y;
        outputMatrix[1][2] = y.z;

        outputMatrix[2][0] = z.x;
        outputMatrix[2][1] = z.y;
        outputMatrix[2][2] = z.z;

        outputMatrix[3][3] = 1;

        outputMatrix[3][0] = origin.x;
        outputMatrix[3][1] = origin.y;
        outputMatrix[3][2] = origin.z;

    }


    public PMatrix getLocalPMatrix() {
        updateMatrix(getLocalMBasis(), outMatLocal);
        float[][] m = outMatLocal;
        PMatrix result = new PMatrix3D(
                m[0][0], m[1][0], m[2][0], m[3][0],
                m[0][1], m[1][1], m[2][1], m[3][1],
                m[0][2], m[1][2], m[2][2], m[3][2],
                m[0][3], m[1][3], m[2][3], m[3][3]);
        return result;
    }

    public PMatrix getGlobalPMatrix() {
        this.updateGlobal();
        updateMatrix(getGlobalMBasis(), outMatGlobal);
        float[][] m = outMatGlobal;
        PMatrix result = new PMatrix3D(
                m[0][0], m[1][0], m[2][0], m[3][0],
                m[0][1], m[1][1], m[2][1], m[3][1],
                m[0][2], m[1][2], m[2][2], m[3][2],
                m[0][3], m[1][3], m[2][3], m[3][3]);
        return result;
    }


    public void drawMe(PGraphics pg, float size) {
        pg.noStroke();
        updateGlobal();
        pg.pushMatrix();
        pg.setMatrix(getGlobalPMatrix());
        if (renderMode == 1) pg.fill(0, 255, 0);
        else pg.fill(0, 0, 0, 255);
        pg.pushMatrix();
        pg.translate(size / 2f, 0, 0);
        pg.box(size, size / 10f, size / 10f);
        pg.popMatrix();
        drawRay(pg, x_().getRayScaledTo(size));
        if (renderMode == 1) pg.fill(255, 0, 0);
        else pg.fill(0, 0, 0, 255);
        pg.pushMatrix();
        pg.translate(0, size / 2f, 0);
        pg.box(size / 10f, size, size / 10f);
        pg.popMatrix();
        if (renderMode == 1) pg.fill(0, 0, 255);
        else pg.fill(0, 0, 0, 255);
        pg.pushMatrix();
        pg.translate(0, 0, size / 2f);
        pg.box(size / 10f, size / 10f, size);
        pg.popMatrix();
        pg.popMatrix();
    }

    public static void drawRay(PGraphics p, Ray3 r) {
        p.line(r.p1().x, r.p1().y, r.p1().z, r.p2().x, r.p2().y, r.p2().z);
    }

    public static void drawPoint(PGraphics p, Vector3 pt) {
        p.point(pt.x, pt.y, pt.z);
    }


}
