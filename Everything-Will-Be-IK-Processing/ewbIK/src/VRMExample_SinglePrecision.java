import ewbik.processing.singlePrecision.Armature;
import ewbik.processing.singlePrecision.Bone;
import ewbik.processing.singlePrecision.IKPin;
import ewbik.processing.singlePrecision.sceneGraph.Axes;
import processing.core.PApplet;
import processing.core.PVector;
import processing.event.MouseEvent;
import java.util.Map;
import java.util.ArrayList;
import math.floatV.Rot;
import math.floatV.AbstractAxes;
import math.floatV.SGVec_3f;
import data.EWBIKSaver;

public class VRMExample_SinglePrecision extends PApplet {
	public static void main(String[] args) {

		PApplet.main("VRMExample_SinglePrecision");
	}

	public void settings() {
		size(1200, 900, P3D);
	}

	Armature simpleArmature;

	UI ui;

	ArrayList<IKPin> pins = new ArrayList<IKPin>();
	Axes worldAxes;

	IKPin activePin;

	Bone root;

	public void setup() {
		ui = new UI(this, false);
		simpleArmature = new Armature("VRMArmatureExample");
		worldAxes = (Axes) simpleArmature.localAxes().getParentAxes();
		if (worldAxes == null) {
			worldAxes = new Axes();
			simpleArmature.localAxes().setParent(worldAxes);
		}
		// attach the armature to the world axes (not necessary, just convenient)
		simpleArmature.localAxes().setParent(worldAxes);

		// specify that we want the solver to run 10 iteration whenever we call it.
		simpleArmature.setDefaultIterations(10);
		// specify the maximum amount any bone is allowed to rotate per iteration
		// (slower convergence, nicer results)
		simpleArmature.setDefaultDampening(10f);
		// benchmark performance
		simpleArmature.setPerformanceMonitor(true);

		// Add some bones to the armature
		initializeBones();

		// add the pins/targets to an array so we can cycle through them easily with
		// keyboad input.
		updatePinList();
	}

	public void draw() {
		if (mousePressed) {
			ui.mouse.z = (float) activePin.getAxes().origin_().z;
			activePin.translateTo(new PVector(ui.mouse.x, ui.mouse.y, activePin.getLocation_().z));
			simpleArmature.IKSolver(simpleArmature.getRootBone());
		} else {
			worldAxes.rotateAboutY(PI / 200f, true);
		}
		String additionalInstructions = "\n HIT THE S KEY TO SAVE THE CURRENT ARMATURE CONFIGURATION.";
		ui.drawScene(0.4f, 30f, null, simpleArmature, additionalInstructions, activePin, null, false);
	}

	PVector mouse = new PVector(0, 0, 0);

	public void mouseWheel(MouseEvent event) {
		float e = event.getCount();
		if (event.isShiftDown()) {
			activePin.getAxes().rotateAboutZ(e / TAU, true);
		} else if (event.isControlDown()) {
			activePin.getAxes().rotateAboutX(e / TAU, true);
		} else {
			activePin.getAxes().rotateAboutY(e / TAU, true);
		}
		activePin.solveIKForThisAndChildren();
	}

	public void keyPressed() {
		if (key == CODED) {
			if (keyCode == DOWN) {
				int currentPinIndex = (pins.indexOf(activePin) + 1) % pins.size();
				activePin = pins.get(currentPinIndex);
			} else if (keyCode == UP) {
				int idx = pins.indexOf(activePin);
				int currentPinIndex = (pins.size() - 1) - (((pins.size() - 1) - (idx - 1)) % pins.size());
				activePin = pins.get(currentPinIndex);
			}
		} else if (key == 's') {
			println("Saving");
			EWBIKSaver newSaver = new EWBIKSaver();
			newSaver.saveArmature(simpleArmature, "VRM_Humanoid.arm");
		}
	}

	Map<String, Bone> boneMap = new java.util.HashMap<String, Bone>();

	public void addBone(String name, String parentName, double x, double y, double z, double roll_x, double roll_y,
			double roll_z, double height) {
		Bone bone = new Bone(boneMap.get(parentName),
				new PVector((float) -(x * 100.0f), (float) -(y * 100.0f), (float) (z * 100.0f)),
				new PVector((float) -(roll_x * 100.0f), (float) -(roll_y * 100.0f), (float) (roll_z * 100.0f)),
				name,
				(float) (height * 100.0f),
				IK.AbstractBone.frameType.RELATIVE);
		boneMap.put(name, bone);
	}

	public void initializeBones() {
		root = simpleArmature.getRootBone();
		root.localAxes().markDirty();
		root.localAxes().updateGlobal();
		boneMap.put("", root);
		addBone("RootBone", "", 1.0, 0.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.30000001192092896);
		addBone("HipsBone", "RootBone", 1.0, 0.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.15937495231628418);
		addBone("SpineBone", "HipsBone", -4.371138828673793e-08, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.11979162693023682);
		addBone("ChestBone", "SpineBone", -4.371138828673793e-08, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.2875000238418579);
		addBone("NeckBone", "ChestBone", -4.371138828673793e-08, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.14166676998138428);
		addBone("HeadBone", "NeckBone", -4.371138828673793e-08, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.14166665077209473);
		addBone("LeftEyeBone", "HeadBone", -1.0, 0.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.029999999329447746);
		addBone("RightEyeBone", "HeadBone", -1.0, 0.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.029999999329447746);
		addBone("LeftShoulderBone", "ChestBone", 0.0, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.07999999821186066);
		addBone("LeftUpperArmBone", "LeftShoulderBone", 0.0, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.3187500238418579);
		addBone("LeftLowerArmBone", "LeftUpperArmBone", 0.0, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.2550000250339508);
		addBone("LeftHandBone", "LeftLowerArmBone", 0.0, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.07968747615814209);
		addBone("LeftThumbProximalBone", "LeftHandBone", -0.7071080207824707, -0.7071064710617065, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.03445944935083389);
		addBone("LeftThumbIntermediateBone", "LeftThumbProximalBone", -0.7071064710617065, -0.7071073651313782, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.025844594463706017);
		addBone("LeftThumbDistalBone", "LeftThumbIntermediateBone", -0.7071069478988647, -0.7071064114570618, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.01938343048095703);
		addBone("LeftIndexProximalBone", "LeftHandBone", 0.0, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.029492318630218506);
		addBone("LeftIndexIntermediateBone", "LeftIndexProximalBone", 0.0, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.022119224071502686);
		addBone("LeftIndexDistalBone", "LeftIndexIntermediateBone", 0.0, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.016589462757110596);
		addBone("LeftMiddleProximalBone", "LeftHandBone", 0.0, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.034459471702575684);
		addBone("LeftMiddleIntermediateBone", "LeftMiddleProximalBone", 0.0, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.025844573974609375);
		addBone("LeftMiddleDistalBone", "LeftMiddleIntermediateBone", 0.0, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.01938343048095703);
		addBone("LeftRingProximalBone", "LeftHandBone", 0.0, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.029492318630218506);
		addBone("LeftRingIntermediateBone", "LeftRingProximalBone", 0.0, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.022119224071502686);
		addBone("LeftRingDistalBone", "LeftRingIntermediateBone", 0.0, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.016589462757110596);
		addBone("LeftLittleProximalBone", "LeftHandBone", 0.0, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.022318542003631592);
		addBone("LeftLittleIntermediateBone", "LeftLittleProximalBone", 0.0, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.0167388916015625);
		addBone("LeftLittleDistalBone", "LeftLittleIntermediateBone", 0.0, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.012554168701171875);
		addBone("RightShoulderBone", "ChestBone", 0.0, -1.0, 8.742277657347586e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.07999999821186066);
		addBone("RightUpperArmBone", "RightShoulderBone", 0.0, -1.0, 8.742277657347586e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.3187500238418579);
		addBone("RightLowerArmBone", "RightUpperArmBone", 0.0, -1.0, 8.742277657347586e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.2550000250339508);
		addBone("RightHandBone", "RightLowerArmBone", 0.0, -1.0, 8.742277657347586e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.07968747615814209);
		addBone("RightThumbProximalBone", "RightHandBone", 0.7071073055267334, -0.7071072459220886, 8.742284762774943e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.03445944935083389);
		addBone("RightThumbIntermediateBone", "RightThumbProximalBone", 0.7071061730384827, -0.7071075439453125, 8.742279078433057e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.025844594463706017);
		addBone("RightThumbDistalBone", "RightThumbIntermediateBone", 0.7071071267127991, -0.7071062326431274, 8.742276236262114e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.01938343048095703);
		addBone("RightIndexProximalBone", "RightHandBone", 0.0, -1.0, 8.742277657347586e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.029492318630218506);
		addBone("RightIndexIntermediateBone", "RightIndexProximalBone", 0.0, -1.0, 8.742277657347586e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.022119224071502686);
		addBone("RightIndexDistalBone", "RightIndexIntermediateBone", 0.0, -1.0, 8.742277657347586e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.016589462757110596);
		addBone("RightMiddleProximalBone", "RightHandBone", 0.0, -1.0, 8.742277657347586e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.034459471702575684);
		addBone("RightMiddleIntermediateBone", "RightMiddleProximalBone", 0.0, -1.0, 8.742277657347586e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.025844573974609375);
		addBone("RightMiddleDistalBone", "RightMiddleIntermediateBone", 0.0, -1.0, 8.742277657347586e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.01938343048095703);
		addBone("RightRingProximalBone", "RightHandBone", 0.0, -1.0, 8.742277657347586e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.029492318630218506);
		addBone("RightRingIntermediateBone", "RightRingProximalBone", 0.0, -1.0, 8.742277657347586e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.022119224071502686);
		addBone("RightRingDistalBone", "RightRingIntermediateBone", 0.0, -1.0, 8.742277657347586e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.016589462757110596);
		addBone("RightLittleProximalBone", "RightHandBone", 0.0, -1.0, 8.742277657347586e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.022318542003631592);
		addBone("RightLittleIntermediateBone", "RightLittleProximalBone", 0.0, -1.0, 8.742277657347586e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.0167388916015625);
		addBone("RightLittleDistalBone", "RightLittleIntermediateBone", 0.0, -1.0, 8.742277657347586e-08, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.012554168701171875);
		addBone("LeftUpperLegBone", "HipsBone", -4.371138828673793e-08, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.42234373092651367);
		addBone("LeftLowerLegBone", "LeftUpperLegBone", -4.371138828673793e-08, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.42234376072883606);
		addBone("LeftFootBone", "LeftLowerLegBone", -4.371138828673793e-08, -0.44029369950294495, 0.8978537917137146, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.19305294752120972);
		addBone("LeftToesBone", "LeftFootBone", 4.371138828673793e-08, 0.0, -1.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.08666665852069855);
		addBone("RightUpperLegBone", "HipsBone", -4.371138828673793e-08, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.42234373092651367);
		addBone("RightLowerLegBone", "RightUpperLegBone", -4.371138828673793e-08, -1.0, 0.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.42234376072883606);
		addBone("RightFootBone", "RightLowerLegBone", -4.371138828673793e-08, -0.44029369950294495, 0.8978537917137146, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.19305294752120972);
		addBone("RightToesBone", "RightFootBone", 4.371138828673793e-08, 0.0, -1.0, 0.44029372930526733, 0.8978536128997803, 2.3676154370377844e-08, 0.08666665852069855);

		Bone rootBone = boneMap.get("RootBone");
		rootBone.enablePin();
		Bone hipsBone = boneMap.get("HipsBone");
		hipsBone.enablePin();
		Bone rightFootBone = boneMap.get("RightFootBone");
		rightFootBone.enablePin();
		Bone leftFootBone = boneMap.get("LeftFootBone");
		leftFootBone.enablePin();
		Bone rightHandBone = boneMap.get("RightHandBone");
		rightHandBone.enablePin();
		Bone leftHandBone = boneMap.get("LeftHandBone");
		leftHandBone.enablePin();
		Bone headBone = boneMap.get("HeadBone");
		headBone.enablePin();
		rootBone.rotAboutFrameX(.01f);
		activePin = rootBone.getIKPin();
	}

	public void updatePinList() {
		pins.clear();
		recursivelyAddToPinnedList(pins, simpleArmature.getRootBone());
	}

	public void recursivelyAddToPinnedList(ArrayList<IKPin> pins, Bone descendedFrom) {
		@SuppressWarnings("unchecked")
		ArrayList<Bone> pinnedChildren = (ArrayList<Bone>) descendedFrom.getMostImmediatelyPinnedDescendants();
		for (Bone b : pinnedChildren) {
			pins.add(b.getIKPin());
		}
		for (Bone b : pinnedChildren) {
			ArrayList<Bone> children = b.getChildren();
			for (Bone b2 : children) {
				recursivelyAddToPinnedList(pins, b2);
			}
		}
	}

}
