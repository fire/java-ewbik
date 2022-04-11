package processing;

import ik.Bone;
import ik.IKPin;
import processing.Node3D;
import processing.Skeleton3D;
import processing.core.PApplet;
import processing.core.PGraphics;
import processing.core.PVector;
import processing.event.MouseEvent;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.cedarsoftware.util.io.JsonReader;
import com.cedarsoftware.util.io.JsonWriter;

import ewbik.processing.singlePrecision.Kusudama;
import processing.core.PConstants;

class UI {
    PApplet pa;
    PGraphics display;
    PVector mouse = new PVector(0, 0, 0);
    PVector cameraPosition = new PVector(0, 0, 70);
    PVector lookAt = new PVector(0, 0, 0);
    PVector up = new PVector(0, 1, 0);
    float orthoHeight, orthoWidth;
    private PGraphics currentDrawSurface;

    public UI(PApplet p) {
        pa = p;
        currentDrawSurface = pa.g;
        display = pa.createGraphics(p.width, p.height, PConstants.P3D);
        display.smooth(8);
        System.out.println(p.sketchPath());
        Kusudama.kusudamaShader = pa.loadShader("kusudama.glsl",
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
        pg.rect((-pa.width / 2f) + 40, (-pa.height / 2f) + 15, boxW + 45, boxH + 40);
        pg.fill(255, 255, 255, 255);
        pg.emissive(255, 255, 255);
        pg.text(instructionText, (-pa.width / 2f) + 40f, -pa.height / 2f + 30f);

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
                (pg.screenX(pt.x, pt.y, pt.z) * zoomScalar) - orthoWidth / 2f,
                (pg.screenY(pt.x, pt.y, pt.z) * zoomScalar) - orthoHeight / 2f);
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

        currentDrawSurface = pa.g;
        setCamera(pa.g, zoomScalar);
        pa.background(169, 202, 239);
        pa.imageMode(PConstants.CENTER);
        pa.image(display, 0, 0, orthoWidth, orthoHeight);
        pa.resetMatrix();
        drawPins(pa.g, activePin, zoomScalar, drawSize, cubeEnabled, cubeNode3D);
        pa.resetMatrix();
        float cx = pa.width;
        float cy = pa.height;
        pa.ortho(-cx / 2f, cx / 2f, -cy / 2f, cy / 2f, -1000, 1000);
        drawInstructions(pa.g, usageInstructions);
        drawPins(pa.g, activePin, drawSize, zoomScalar, cubeEnabled, cubeNode3D);
        drawInstructions(pa.g, usageInstructions);

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
        orthoHeight = pa.height * zoomScalar;
        orthoWidth = ((float) pa.width / (float) pa.height) * orthoHeight;
        mouse.x = (pa.mouseX - (pa.width / 2f)) * (orthoWidth / pa.width);
        mouse.y = (pa.mouseY - (pa.height / 2f)) * (orthoHeight / pa.height);
        camera(cameraPosition, lookAt, up, pg);
        pg.ortho(-orthoWidth / 2f, orthoWidth / 2f, -orthoHeight / 2f, orthoHeight / 2f, -1000, 1000);
    }

    /**
     * @return the draw surface this class is currently operating on.
     *         This is used as kind of hack, so I don't have to bother writing
     *         interfaces just to render a box when using multi-pass.
     */
    public PGraphics getCurrentDrawSurface() {
        return currentDrawSurface;
    }

}

public class ItemHolding extends PApplet {

    Skeleton3D loadedArmature;
    ArrayList<IKPin> pins = new ArrayList<>();
    UI ui;
    IKPin activePin;
    Node3D worldNode3D, cubeNode3D;
    float zoomScalar = 200f / height;
    boolean cubeMode = true;

    public static void main(String[] args) {
        PApplet.main("processing.ItemHolding");
    }

    public void settings() {
        size(1200, 900, P3D);
    }

    public void loadArmature() {
        try (BufferedReader br = new BufferedReader(new FileReader("Humanoid_Holding_Item.json"))) {
            StringBuilder sb = new StringBuilder();
            String line = br.readLine();

            while (line != null) {
                sb.append(line);
                sb.append(System.lineSeparator());
                line = br.readLine();
            }
            String json = sb.toString();
            loadedArmature = (Skeleton3D) JsonReader.jsonToJava(json);
        } catch (IOException e) {
            e.printStackTrace();
        }
        loadedArmature.updateBonechains();
        loadedArmature.IKSolver(loadedArmature.getRootBone(), 0.5f, 20, 1);

        Bone.setDrawKusudamas(true);
    }

    public void setup() {
        ui = new UI(this);
        loadArmature();
        worldNode3D = loadedArmature.localAxes().getParentAxes();
        if (worldNode3D == null) {
            worldNode3D = new Node3D();
            loadedArmature.localAxes().setParent(worldNode3D);
        }
        updatePinList();
        cubeNode3D = new Node3D();

        activePin = pins.get(pins.size() - 1);

        loadedArmature.setPerformanceMonitor(true); // print performance stats

        // Tell the Bone class that all bones should draw their kusudamas.
        Bone.setDrawKusudamas(false);

        /**
         * The armature we're loading is already posed such that its hands touch
         * a box. So all we need to do is , first
         * move our box into the appropriate position
         */
        cubeNode3D.translateTo(new PVector(-13, -27, 32));
        cubeNode3D.setRelativeToParent(worldNode3D);
        /**
         * and then specify that the transformations of the left hand and right hand
         * pins
         * should be computed relative to the axes of the cube we're drawing,
         * Thereby, any time we transform the parent cube's axes, the pins will follow.
         */
        loadedArmature.getBoneName("left hand").getIKPin().getAxes().setParent(cubeNode3D);
        loadedArmature.getBoneName("right hand").getIKPin().getAxes().setParent(cubeNode3D);

    }

    public void draw() {
        if (mousePressed) {
            if (cubeMode) {
                cubeNode3D.translateTo(new PVector(ui.mouse.x, ui.mouse.y, cubeNode3D.calculatePosition().z));
            } else {
                activePin.translateTo(new PVector(ui.mouse.x, ui.mouse.y, activePin.getLocation_().z));
            }
            loadedArmature.IKSolver(loadedArmature.getRootBone());
        } else {
            worldNode3D.rotateAboutY(0.0f, true);
        }

        String additionalInstructions = "Hit the 'C' key to select or deselect the cube";
        additionalInstructions += "\n HIT THE S KEY TO SAVE."
                + "\n HIT THE L KEY TO LOAD THE CURRENT ARMATURE CONFIGURATION.";
        // Decrease the numerator to increase the zoom.
        zoomScalar = 200f / height;
        ui.drawScene(zoomScalar, 12f, () -> drawHoldCube(), loadedArmature, additionalInstructions, activePin,
                cubeNode3D,
                cubeMode);
    }

    public void drawHoldCube() {
        PGraphics currentDisplay = ui.getCurrentDrawSurface();
        if (ui.display == currentDisplay) {
            currentDisplay.fill(60, 60, 60);
            currentDisplay.strokeWeight(1);
            currentDisplay.stroke(255);
        } else {
            currentDisplay.fill(0, 0, 0, 255);
            currentDisplay.emissive(0);
            currentDisplay.noStroke();
        }
        currentDisplay.pushMatrix();
        currentDisplay.applyMatrix(cubeNode3D.getGlobalPMatrix());
        currentDisplay.box(40, 20, 20);
        currentDisplay.popMatrix();
    }

    public void mouseWheel(MouseEvent event) {
        float e = event.getCount();
        Node3D node3D = cubeMode ? cubeNode3D
                : activePin.getAxes();
        if (event.isShiftDown()) {
            node3D.rotateAboutZ(e / TAU, true);
        } else if (event.isControlDown()) {
            node3D.rotateAboutX(e / TAU, true);
        } else {
            node3D.rotateAboutY(e / TAU, true);
        }
        activePin.solveIKForThisAndChildren();
    }

    public void keyPressed() {
        if (key == CODED) {
            if (keyCode == DOWN) {
                cubeMode = false;
                int currentPinIndex = (pins.indexOf(activePin) + 1) % pins.size();
                activePin = pins.get(currentPinIndex);
            } else if (keyCode == UP) {
                cubeMode = false;
                int idx = pins.indexOf(activePin);
                int currentPinIndex = (pins.size() - 1) - (((pins.size() - 1) - (idx - 1)) % pins.size());
                activePin = pins.get(currentPinIndex);
            }
        } else if (key == 'c') {
            cubeMode = !cubeMode;
        } else if (key == 's') {
            println("Saving");
            Map args = new HashMap();
            args.put(JsonWriter.PRETTY_PRINT, true);
            args.put(JsonWriter.SKIP_NULL_FIELDS, true);
            String json = JsonWriter.objectToJson(loadedArmature, args);
            BufferedWriter writer = null;
            try {
                writer = new BufferedWriter(new FileWriter("Humanoid_Holding_Item.json"));
                writer.write(json);

            } catch (IOException e) {
            } finally {
                try {
                    if (writer != null)
                        writer.close();
                } catch (IOException e) {
                }
            }
        } else if (key == 'l') {
            loadArmature();
        }
    }

    public void updatePinList() {
        pins.clear();
        recursivelyAddToPinnedList(pins, loadedArmature.getRootBone());
    }

    public void recursivelyAddToPinnedList(ArrayList<IKPin> pins, Bone descendedFrom) {
        @SuppressWarnings("unchecked")
        ArrayList<Bone> pinnedChildren = (ArrayList<Bone>) descendedFrom.getMostImmediatelyPinnedDescendants();
        for (Bone b : pinnedChildren) {
            IKPin pin = b.getIKPin();
            pins.add(pin);
        }
        for (Bone b : pinnedChildren) {
            ArrayList<Bone> children = b.getChildren();
            for (Bone b2 : children) {
                recursivelyAddToPinnedList(pins, b2);
            }
        }
    }

}
