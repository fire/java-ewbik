package processing;

import InverseKinematics.*;
import processing.core.PApplet;
import processing.core.PConstants;
import processing.core.PGraphics;
import processing.core.PVector;
import processing.event.MouseEvent;

import java.util.ArrayList;


class UI {
    static final PVector up = new PVector(0, 1, 0);
    PApplet processingApplet;
    PGraphics display;
    PVector mouse = new PVector(0, 0, 0);
    PVector cameraPosition = new PVector(0, 0, 70);
    PVector lookAt = new PVector(0, 0, 0);
    float orthographicHeight, orthographicWidth;
    private PGraphics currentDrawSurface;

    public UI(PApplet p) {
        processingApplet = p;
        currentDrawSurface = processingApplet.g;
        display = processingApplet.createGraphics(p.width, p.height, PConstants.P3D);
        display.smooth(8);
        System.out.println(p.sketchPath());
        Kusudama.kusudamaShader = processingApplet.loadShader("kusudama.glsl",
                "kusudama_vert.glsl");

    }

    public void drawInstructions(PGraphics pg, String append) {
        String appended = append == null ? "" : "-" + append;

        String instructionText = "-Click and drag to move the selected pin.\n"
                + "-To select a different pin, use the Up and Down arrows.\n"
                + "-Use the mouse wheel to rotate the pin about its (red) Y axis.\n"
                + "-Hold shift while using the mouse wheel to rotate the pin about its (blue) Z axis.\n"
                + "-Hold ctrl while using the mouse wheel to rotate the pin about its (green) X axis. \n"
                + appended;
        pg.textSize(12);
        pg.fill(0, 0, 0, 90);
        float boxW = pg.textWidth(instructionText);
        float boxH = (pg.textAscent() + pg.textDescent()) * (instructionText.split("\n").length);
        pg.rect((-processingApplet.width / 2f) + 40, (-processingApplet.height / 2f) + 15, boxW + 45, boxH + 40);
        pg.fill(255, 255, 255, 255);
        pg.emissive(255, 255, 255);
        pg.text(instructionText, (-processingApplet.width / 2f) + 40f, -processingApplet.height / 2f + 30f);

    }

    public void line(PGraphics pg, PVector p1, PVector p2) {
        pg.line(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
    }

    public void point(PGraphics pg, PVector p) {
        pg.point(p.x, p.y, p.z);
    }

    public void drawPins(PGraphics pg, IKPin activePin,
                         float zoomScalar, float drawSize,
                         boolean cubeMode, Node3D cubeNode3D) {

        if (activePin != null) {
            Node3D ellipseAx;
            ellipseAx = cubeMode ? cubeNode3D : activePin.getAxes();
            PVector pinLoc = screenOf(pg, ellipseAx.origin(), zoomScalar);
            PVector pinX = screenOf(pg,
                    Node3D.toPVector(ellipseAx.calculateX().getScaledTo(drawSize)),
                    zoomScalar);
            PVector pinY = screenOf(pg,
                    Node3D.toPVector(ellipseAx.calculateY().getScaledTo(drawSize)),
                    zoomScalar);
            PVector pinZ = screenOf(pg,
                    Node3D.toPVector(ellipseAx.calculateZ().getScaledTo(drawSize)),
                    zoomScalar);
            pg.fill(255, 255, 255, 150);
            pg.stroke(255, 0, 255);
            float totalPriorities = activePin.getXPriority() + activePin.getYPriority()
                    + activePin.getZPriority();
            pg.ellipse(pinLoc.x, pinLoc.y, zoomScalar * 50, zoomScalar * 50);

            PVector effectorO = screenOf(pg,
                    Node3D.toPVector(activePin.forBone().localAxes().calculatePosition()),
                    zoomScalar);
            PVector effectorX = screenOf(pg,
                    Node3D
                            .toPVector(activePin.forBone().localAxes().calculateX().getScaledTo(drawSize)),
                    zoomScalar);
            PVector effectorY = screenOf(pg,
                    Node3D
                            .toPVector(activePin.forBone().localAxes().calculateY().getScaledTo(drawSize)),
                    zoomScalar);
            PVector effectorZ = screenOf(pg,
                    Node3D
                            .toPVector(activePin.forBone().localAxes().calculateZ().getScaledTo(drawSize)),
                    zoomScalar);
            pg.stroke(255, 255, 255, 150);

            if (!cubeMode) {
                float xPriority = activePin.getXPriority();
                float yPriority = activePin.getYPriority();
                float zPriority = activePin.getZPriority();
                drawPinEffectorHints(
                        pg,
                        pinLoc,
                        pinX, pinY, pinZ,
                        effectorO,
                        effectorX, effectorY, effectorZ,
                        xPriority, yPriority, zPriority, totalPriorities);
            }
        }
    }

    public void drawPinEffectorHints(PGraphics pg,
                                     PVector pinLoc,
                                     PVector pinX, PVector pinY, PVector pinZ,
                                     PVector effectorO,
                                     PVector effectorX, PVector effectorY, PVector effectorZ,
                                     float xPriority, float yPriority, float zPriority, float totalpriorities) {

        pg.line(pinLoc.x, pinLoc.y, pinLoc.z, effectorO.x, effectorO.y, effectorO.z);
        pg.stroke(2, 58, 0, 150);
        pg.strokeWeight(2f * xPriority / totalpriorities);
        pg.line(pinX.x, pinX.y, pinX.z, effectorX.x, effectorX.y, effectorX.z);
        pg.stroke(140, 0, 1, 150);
        pg.strokeWeight(2f * yPriority / totalpriorities);
        pg.line(pinY.x, pinY.y, pinY.z, effectorY.x, effectorY.y, effectorY.z);
        pg.stroke(0, 26, 130, 150);
        pg.strokeWeight(2f * zPriority / totalpriorities);
        pg.line(pinZ.x, pinZ.y, pinZ.z, effectorZ.x, effectorZ.y, effectorZ.z);

    }

    public void drawPass(float drawSize, Runnable preArmatureDraw, PGraphics buffer, Skeleton3D armature) {
        if (preArmatureDraw != null)
            preArmatureDraw.run();
        armature.drawMe(buffer, 100, drawSize);
    }

    public PVector screenOf(PGraphics pg, PVector pt, float zoomScalar) {
        return new PVector(
                (pg.screenX(pt.x, pt.y, pt.z) * zoomScalar) - orthographicWidth / 2f,
                (pg.screenY(pt.x, pt.y, pt.z) * zoomScalar) - orthographicHeight / 2f);
    }

    public void drawScene(float zoomScalar, float drawSize,
                          Runnable additionalDraw,
                          Skeleton3D armature,
                          String usageInstructions,
                          IKPin activePin, Node3D cubeNode3D, boolean cubeEnabled) {
        currentDrawSurface = display;
        display.beginDraw();
        setSceneAndCamera(display, zoomScalar);
        drawPass(drawSize, additionalDraw, display, armature);
        display.endDraw();

        currentDrawSurface = processingApplet.g;
        setCamera(processingApplet.g, zoomScalar);
        processingApplet.background(169, 202, 239);
        processingApplet.imageMode(PConstants.CENTER);
        processingApplet.image(display, 0, 0, orthographicWidth, orthographicHeight);
        processingApplet.resetMatrix();
        drawPins(processingApplet.g, activePin, zoomScalar, drawSize, cubeEnabled, cubeNode3D);
        processingApplet.resetMatrix();
        float cx = processingApplet.width;
        float cy = processingApplet.height;
        processingApplet.ortho(-cx / 2f, cx / 2f, -cy / 2f, cy / 2f, -1000, 1000);
        drawInstructions(processingApplet.g, usageInstructions);
        drawPins(processingApplet.g, activePin, drawSize, zoomScalar, cubeEnabled, cubeNode3D);
        drawInstructions(processingApplet.g, usageInstructions);

    }

    public void camera(PVector cp, PVector so, PVector up, PGraphics pg) {
        pg.camera(cp.x, cp.y, cp.z, so.x, so.y, so.z, up.x, up.y, up.z);
    }

    public void setSceneAndCamera(PGraphics pg, float zoomScalar) {
        setCamera(pg, zoomScalar);
        pg.directionalLight(148, 148, 148, 0, 100, 100);
        pg.directionalLight(148, 148, 148, 0, 100, -100);
        pg.directionalLight(148, 148, 148, 100, 100, 0);
        pg.directionalLight(148, 148, 148, -100, 100, 0);
        pg.directionalLight(48, 48, 48, 100, -10, 100);
        pg.directionalLight(48, 48, 48, 100, -10, -100);
        pg.directionalLight(48, 48, 48, -100, -10, 100);
        pg.directionalLight(48, 48, 48, -100, -10, -100);

    }

    public void setCamera(PGraphics pg, float zoomScalar) {
        pg.clear();
        orthographicHeight = processingApplet.height * zoomScalar;
        orthographicWidth = ((float) processingApplet.width / (float) processingApplet.height) * orthographicHeight;
        mouse.x = (processingApplet.mouseX - (processingApplet.width / 2f)) * (orthographicWidth / processingApplet.width);
        mouse.y = (processingApplet.mouseY - (processingApplet.height / 2f)) * (orthographicHeight / processingApplet.height);
        camera(cameraPosition, lookAt, up, pg);
        pg.ortho(-orthographicWidth / 2f, orthographicWidth / 2f, -orthographicHeight / 2f, orthographicHeight / 2f, -1000, 1000);
    }

    /**
     * @return the draw surface this class is currently operating on.
     * This is used as kind of hack, so I don't have to bother writing
     * interfaces just to render a box when using multi-pass.
     */
    public PGraphics getCurrentDrawSurface() {
        return currentDrawSurface;
    }

}


public class KusudamaVisualizer extends PApplet {
    public static void main(String[] args) {
        PApplet.main("processing.KusudamaVisualizer");
    }
    public void settings(){
        size(1200, 900, P3D);
        noSmooth();
    }

    float zoomScalar = 7f/height;
    float orthographicHeight = height;
    float orthographicWidth = width;

    Skeleton3D simpleArmature;
    Bone  rootBone, initialBone,
            seconBone, thirBone;

    UI ui;

    Node3D worldAxes;
    ArrayList<IKPin> pins = new ArrayList<>();
    public static IKPin activePin;

    public void setup() {
        ui = new UI(this);

        //Create global axes so we can easily manipulate the whole scene. (not necessary, just convenient)
        worldAxes = new Node3D();

        //Create an armature
        simpleArmature = new Skeleton3D("example");

        //attach the armature to the world axes (not necessary, just convenient)
        simpleArmature.localAxes().setParent(worldAxes);

        //translate everything down to where the user can see it,
        //and rotate it 180 degrees about the z-axis so it's not upside down.
        worldAxes.translateTo(new PVector(0, 150, 0));
        simpleArmature.localAxes().rotateAboutZ(PI, true);

        //specify that we want the solver to run 10 iteration whenever we call it.
        simpleArmature.setDefaultIterations(10);
        //specify the maximum amount any bone is allowed to rotate per iteration (slower convergence, nicer results)
        simpleArmature.setDefaultDampening(0.03f);
        //specify that the armature should avoid degenerate solutions.
        simpleArmature.setDefaultStabilizingPassCount(1);
        //benchmark performance
        simpleArmature.setPerformanceMonitor(true);

        initializeBones();
        setBoneConstraints();


        updatePinList();

        //Tell the Bone class that all bones should draw their kusudamas.
        Bone.setDrawKusudamas(true);
    }

    public void initializeBones() {
        rootBone = simpleArmature.getRootBone();
        rootBone.setBoneHeight(20f);
        initialBone = new Bone(rootBone, "initial", 74f);
        seconBone = new Bone(initialBone, "seconBone", 86f);
        thirBone = new Bone(seconBone, "thirBone", 98f);

        initialBone.rotAboutFrameX(.01f);

        //pin the root
        rootBone.enablePin();

        //intermediary pin a few bones up the chain.
        thirBone.enablePin();

        //determine how much precedence each of this pin's axes get
        //in relation to other axes on other pins being considered by the solver.
        //this line state that the solver should care about this bone's X and Y headings
        //aligning with its targets about 5 times as much as it cares about the X and Y headings of any other bones.
        //it also tells the solver to ignore the z heading entirely.
        thirBone.getIKPin().setTargetPriorities(5f, 5f,0f);
    }

    public void setBoneConstraints() {

        Kusudama firstConstraint = new Kusudama(initialBone);
        firstConstraint.addLimitConeAtIndex(0, new PVector(.5f, 1f, 0f), 0.5f);
        firstConstraint.addLimitConeAtIndex(1, new PVector(-.5f, 1f, 0f), 0.7f);
        firstConstraint.setAxialLimits(0.01f,0.03f);
        firstConstraint.enable();
        initialBone.addConstraint(firstConstraint);

        Kusudama secondConstraint = new Kusudama(seconBone);
        secondConstraint.addLimitConeAtIndex(0, new PVector(.5f, 1f, 0f),0.6f);
        secondConstraint.addLimitConeAtIndex(1, new PVector(-1f, 1f, 0f), 0.2f);
        secondConstraint.setAxialLimits(0.1f,0.9f);
        secondConstraint.enable();
        seconBone.addConstraint(secondConstraint);

        Kusudama thirdConstraint = new Kusudama(thirBone);
        thirdConstraint.addLimitConeAtIndex(0, new PVector(.5f, 1f, 0f), 0.8f);
        thirdConstraint.addLimitConeAtIndex(1, new PVector(-.5f, 1f, 0f), 0.8f);
        thirdConstraint.setAxialLimits(0.1f,0.3f);
        thirdConstraint.enable();
        thirBone.addConstraint(thirdConstraint);
    }

    public void draw() {
        if(mousePressed) {
            //Set the selected pin to the position of the mouse if the user is dragging it.
            activePin.translateTo(
                    new PVector(
                            ui.mouse.x,
                            ui.mouse.y,
                            activePin.getLocation_().z));

            //run the IK solver on the armature.
            simpleArmature.IKSolver(rootBone);

        }else {
            //rotate the world so the user can inspect the pose
            worldAxes.rotateAboutY(PI/500f, true);
        }

        zoomScalar = 350f/height;
        ui.drawScene(zoomScalar, 20f, null, simpleArmature, null, activePin, null, false);
    }


    public void mouseWheel(MouseEvent event) {
        float e = event.getCount();
        if(event.isShiftDown()) {
            activePin.getAxes().rotateAboutZ(e/TAU, true);
        }else if (event.isControlDown()) {
            activePin.getAxes().rotateAboutX(e/TAU, true);
        }  else {
            activePin.getAxes().rotateAboutY(e/TAU, true);
        }
        activePin.solveIKForThisAndChildren();
    }

    public void keyPressed() {
        if (key == CODED) {
            if (keyCode == DOWN) {
                int currentPinIndex =(pins.indexOf(activePin) + 1) % pins.size();
                activePin  = pins.get(currentPinIndex);
            } else if (keyCode == UP) {
                int idx = pins.indexOf(activePin);
                int currentPinIndex =  (pins.size()-1) -(((pins.size()-1) - (idx - 1)) % pins.size());
                activePin  = pins.get(currentPinIndex);
            }
        }
    }


    public void updatePinList() {
        pins.clear();
        recursivelyAddToPinnedList(pins, simpleArmature.getRootBone());
        if(pins .size() > 0) {
            activePin = pins.get(pins.size()-1);
        }
    }

    public void recursivelyAddToPinnedList(ArrayList<IKPin> pins, Bone descendedFrom) {
        ArrayList<Bone> pinnedChildren = (ArrayList<Bone>) descendedFrom.getMostImmediatelyPinnedDescendants();
        for(Bone b : pinnedChildren) {
            pins.add((IKPin)b.getIKPin());
            b.getIKPin().getAxes().setParent(worldAxes);
        }
        for(Bone b : pinnedChildren) {
            ArrayList<Bone> children = b.getChildren();
            for(Bone b2 : children) {
                recursivelyAddToPinnedList(pins, b2);
            }
        }
    }
}