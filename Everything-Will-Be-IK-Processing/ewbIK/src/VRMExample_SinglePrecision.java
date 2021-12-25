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
		worldAxes = new Axes();
		simpleArmature = new Armature("VRMArmatureExample");
		// attach the armature to the world axes (not necessary, just convenient)
		simpleArmature.localAxes().setParent(worldAxes);

		// specify that we want the solver to run 10 iteration whenever we call it.
		simpleArmature.setDefaultIterations(10);
		// specify the maximum amount any bone is allowed to rotate per iteration
		// (slower convergence, nicer results)
		simpleArmature.setDefaultDampening(10f);
		// benchmark performance
		simpleArmature.setPerformanceMonitor(true);

		// translate everything down to where the user can see it,
		// and rotate it 180 degrees about the z-axis so it's not upside down.
		worldAxes.translateTo(new PVector(0, 150, 0));
		simpleArmature.localAxes().rotateAboutZ(PI, true);

		// Add some bones to the armature
		initializeBones();

		// Pin some of the bones.
		root.enablePin();
		// fifthBone.enablePin();
		// bSixthBone.enablePin();

		// add the pins/targets to an array so we can cycle through them easily with
		// keyboad input.
		updatePinList();

		// select which pin we'll be manipulating to start with.
		activePin = root.getIKPin();
	}

	public void draw() {
		if (mousePressed) {
			ui.mouse.z = (float) activePin.getAxes().origin_().z;
			activePin.translateTo(new PVector(ui.mouse.x, ui.mouse.y, activePin.getLocation_().z));
			simpleArmature.IKSolver(simpleArmature.getRootBone());
		} else {
			worldAxes.rotateAboutY(PI / 200f, true);
		}
		ui.drawScene(0.4f, 30f, null, simpleArmature, null, activePin, null, false);
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
		}
	}

	Map<String, Bone> boneMap = new java.util.HashMap<String, Bone>();

	public void addBone(String name, String parentName, double meter_length, double rotX, double rotY,
			double rotZ, double rotW, double x, double y,
			double z) {
		Axes localAxes = new Axes();
		Rot rot = new math.floatV.Rot(
				new math.floatV.MRotation((float) rotW, (float) rotX, (float) rotY, (float) rotZ));
		localAxes.translateTo(new PVector((float) (x * 100.0f), (float) (y * 100.0f), (float) (z * 100.0f)));
		localAxes.rotateBy(rot);
		PVector tipHeading = new PVector(localAxes.y_().heading().x, localAxes.y_().heading().y,
				localAxes.y_().heading().z);
		PVector rollHeading = new PVector(localAxes.z_().heading().x, localAxes.z_().heading().y,
				localAxes.z_().heading().z);
		Bone bone = new Bone(
				boneMap.get(parentName),
				tipHeading,
				rollHeading,
				name,
				(float) (meter_length * 100.0f),
				IK.AbstractBone.frameType.RELATIVE);
		boneMap.put(name, bone);
	}

	public void initializeBones() {
		root = simpleArmature.getRootBone();
		root.localAxes().markDirty();
		root.localAxes().updateGlobal();
		boneMap.put("", root);
		addBone("RootBone", "", 0.30000001192092896, 0.6926643252372742, -0.1421835571527481, -0.1421835571527481,
				0.6926643252372742, 0.0, 0.0, 0.0);
		addBone("HipsBone", "RootBone", 0.15937495231628418, 0.5, 0.5, 0.5, 0.5, 0.0, 0.0, 0.8500000238418579);
		addBone("SpineBone", "HipsBone", 0.11979162693023682, 0.5, -0.5000000596046448, -0.5, 0.5, 0.0, 0.0,
				1.009374976158142);
		addBone("ChestBone", "SpineBone", 0.2875000238418579, 0.5, -0.5, -0.5, 0.5, 0.0, 0.0, 1.129166603088379);
		addBone("NeckBone", "ChestBone", 0.14166676998138428, 0.5, -0.5, -0.5, 0.5, 0.0, 0.0, 1.4166666269302368);
		addBone("HeadBone", "NeckBone", 0.14166665077209473, 0.5, -0.5, -0.5, 0.5, 0.0, 0.0, 1.558333396911621);
		addBone("LeftEyeBone", "HeadBone", 0.029999999329447746, -6.704862443029924e-09, 0.0, 1.0, 0.0,
				0.028333334252238274, 0.0, 1.59375);
		addBone("RightEyeBone", "HeadBone", 0.029999999329447746, -6.704862443029924e-09, 0.0, 1.0, 0.0,
				-0.028333334252238274, 0.0, 1.59375);
		addBone("LeftShoulderBone", "ChestBone", 0.07999999821186066, 1.2560740513293481e-15, 1.2560740513293481e-15,
				-0.7071068286895752, 0.7071068286895752, 0.02500000037252903, 0.0, 1.4166666269302368);
		addBone("LeftUpperArmBone", "LeftShoulderBone", 0.3187500238418579, -1.1230178408338183e-22,
				-1.1230178408338183e-22, -0.7071068286895752, 0.7071068286895752, 0.10499999672174454, 0.0,
				1.4166666269302368);
		addBone("LeftLowerArmBone", "LeftUpperArmBone", 0.2550000250339508, 3.7433933286866646e-23,
				3.7433933286866646e-23, -0.7071068286895752, 0.7071068286895752, 0.42375001311302185, 0.0,
				1.4166666269302368);
		addBone("LeftHandBone", "LeftLowerArmBone", 0.07968747615814209, 3.743391435420492e-23, 3.743391435420492e-23,
				-0.7071068286895752, 0.7071068286895752, 0.6787500381469727, 0.0, 1.4166666269302368);
		addBone("LeftThumbProximalBone", "LeftHandBone", 0.03445944935083389, 3.120540051996379e-16,
				1.292568968787612e-16, -0.9238796830177307, 0.38268306851387024, 0.6787500381469727,
				-0.02988281287252903, 1.4166666269302368);
		addBone("LeftThumbIntermediateBone", "LeftThumbProximalBone", 0.025844594463706017, 3.120536610925031e-16,
				1.2925700275787962e-16, -0.9238794445991516, 0.38268375396728516, 0.7031165361404419,
				-0.05424933880567551, 1.4166666269302368);
		addBone("LeftThumbDistalBone", "LeftThumbIntermediateBone", 0.01938343048095703, 3.120540316694175e-16,
				1.292568968787612e-16, -0.9238796234130859, 0.38268324732780457, 0.7213914394378662,
				-0.07252421230077744, 1.4166666269302368);
		addBone("LeftIndexProximalBone", "LeftHandBone", 0.029492318630218506, 3.743391435420492e-23,
				3.743391435420492e-23, -0.7071068286895752, 0.7071068286895752, 0.7584375143051147,
				-0.02988281287252903, 1.4166666269302368);
		addBone("LeftIndexIntermediateBone", "LeftIndexProximalBone", 0.022119224071502686, 3.743391435420492e-23,
				3.743391435420492e-23, -0.7071068286895752, 0.7071068286895752, 0.7879298329353333,
				-0.02988281287252903, 1.4166666269302368);
		addBone("LeftIndexDistalBone", "LeftIndexIntermediateBone", 0.016589462757110596, 3.743391435420492e-23,
				3.743391435420492e-23, -0.7071068286895752, 0.7071068286895752, 0.8100490570068359,
				-0.02988281287252903, 1.4166666269302368);
		addBone("LeftMiddleProximalBone", "LeftHandBone", 0.034459471702575684, 3.743391435420492e-23,
				3.743391435420492e-23, -0.7071068286895752, 0.7071068286895752, 0.7584375143051147,
				-0.00996093824505806, 1.4166666269302368);
		addBone("LeftMiddleIntermediateBone", "LeftMiddleProximalBone", 0.025844573974609375, 3.743391435420492e-23,
				3.743391435420492e-23, -0.7071068286895752, 0.7071068286895752, 0.7928969860076904,
				-0.00996093824505806, 1.4166666269302368);
		addBone("LeftMiddleDistalBone", "LeftMiddleIntermediateBone", 0.01938343048095703, 3.743391435420492e-23,
				3.743391435420492e-23, -0.7071068286895752, 0.7071068286895752, 0.8187415599822998,
				-0.00996093824505806, 1.4166666269302368);
		addBone("LeftRingProximalBone", "LeftHandBone", 0.029492318630218506, 3.743391435420492e-23,
				3.743391435420492e-23, -0.7071068286895752, 0.7071068286895752, 0.7584375143051147, 0.00996093824505806,
				1.4166666269302368);
		addBone("LeftRingIntermediateBone", "LeftRingProximalBone", 0.022119224071502686, 3.743391435420492e-23,
				3.743391435420492e-23, -0.7071068286895752, 0.7071068286895752, 0.7879298329353333, 0.00996093824505806,
				1.4166666269302368);
		addBone("LeftRingDistalBone", "LeftRingIntermediateBone", 0.016589462757110596, 3.743391435420492e-23,
				3.743391435420492e-23, -0.7071068286895752, 0.7071068286895752, 0.8100490570068359, 0.00996093824505806,
				1.4166666269302368);
		addBone("LeftLittleProximalBone", "LeftHandBone", 0.022318542003631592, 3.743391435420492e-23,
				3.743391435420492e-23, -0.7071068286895752, 0.7071068286895752, 0.7584375143051147, 0.02988281287252903,
				1.4166666269302368);
		addBone("LeftLittleIntermediateBone", "LeftLittleProximalBone", 0.0167388916015625, 3.743391435420492e-23,
				3.743391435420492e-23, -0.7071068286895752, 0.7071068286895752, 0.7807560563087463, 0.02988281287252903,
				1.4166666269302368);
		addBone("LeftLittleDistalBone", "LeftLittleIntermediateBone", 0.012554168701171875, 3.743391435420492e-23,
				3.743391435420492e-23, -0.7071068286895752, 0.7071068286895752, 0.7974949479103088, 0.02988281287252903,
				1.4166666269302368);
		addBone("RightShoulderBone", "ChestBone", 0.07999999821186066, 0.7071068286895752, -0.7071068286895752,
				5.338508302088485e-08, 5.338508302088485e-08, -0.02500000037252903, 0.0, 1.4166666269302368);
		addBone("RightUpperArmBone", "RightShoulderBone", 0.3187500238418579, 0.7071068286895752, -0.7071068286895752,
				5.338508302088485e-08, 5.338508302088485e-08, -0.10499999672174454, 0.0, 1.4166666269302368);
		addBone("RightLowerArmBone", "RightUpperArmBone", 0.2550000250339508, 0.7071068286895752, -0.7071068286895752,
				5.338508302088485e-08, 5.338508302088485e-08, -0.42375001311302185, 0.0, 1.4166666269302368);
		addBone("RightHandBone", "RightLowerArmBone", 0.07968747615814209, 0.7071068286895752, -0.7071068286895752,
				5.338508302088485e-08, 5.338508302088485e-08, -0.6787500381469727, 0.0, 1.4166666269302368);
		addBone("RightThumbProximalBone", "RightHandBone", 0.03445944935083389, 0.9238796234130859,
				-0.38268330693244934, 6.975098187922413e-08, 2.8891781411743978e-08, -0.6787500381469727,
				-0.02988281287252903, 1.4166666269302368);
		addBone("RightThumbIntermediateBone", "RightThumbProximalBone", 0.025844594463706017, 0.9238794445991516,
				-0.3826839327812195, 6.975098187922413e-08, 2.889182937337864e-08, -0.7031165361404419,
				-0.05424933880567551, 1.4166666269302368);
		addBone("RightThumbDistalBone", "RightThumbIntermediateBone", 0.01938343048095703, 0.9238796830177307,
				-0.3826831877231598, 6.975097477379677e-08, 2.889177608267346e-08, -0.7213914394378662,
				-0.07252421230077744, 1.4166666269302368);
		addBone("RightIndexProximalBone", "RightHandBone", 0.029492318630218506, 0.7071068286895752,
				-0.7071068286895752, 5.338508302088485e-08, 5.338508302088485e-08, -0.7584375143051147,
				-0.02988281287252903, 1.4166666269302368);
		addBone("RightIndexIntermediateBone", "RightIndexProximalBone", 0.022119224071502686, 0.7071068286895752,
				-0.7071068286895752, 5.338508302088485e-08, 5.338508302088485e-08, -0.7879298329353333,
				-0.02988281287252903, 1.4166666269302368);
		addBone("RightIndexDistalBone", "RightIndexIntermediateBone", 0.016589462757110596, 0.7071068286895752,
				-0.7071068286895752, 5.338508302088485e-08, 5.338508302088485e-08, -0.8100490570068359,
				-0.02988281287252903, 1.4166666269302368);
		addBone("RightMiddleProximalBone", "RightHandBone", 0.034459471702575684, 0.7071068286895752,
				-0.7071068286895752, 5.338508302088485e-08, 5.338508302088485e-08, -0.7584375143051147,
				-0.00996093824505806, 1.4166666269302368);
		addBone("RightMiddleIntermediateBone", "RightMiddleProximalBone", 0.025844573974609375, 0.7071068286895752,
				-0.7071068286895752, 5.338508302088485e-08, 5.338508302088485e-08, -0.7928969860076904,
				-0.00996093824505806, 1.4166666269302368);
		addBone("RightMiddleDistalBone", "RightMiddleIntermediateBone", 0.01938343048095703, 0.7071068286895752,
				-0.7071068286895752, 5.338508302088485e-08, 5.338508302088485e-08, -0.8187415599822998,
				-0.00996093824505806, 1.4166666269302368);
		addBone("RightRingProximalBone", "RightHandBone", 0.029492318630218506, 0.7071068286895752, -0.7071068286895752,
				5.338508302088485e-08, 5.338508302088485e-08, -0.7584375143051147, 0.00996093824505806,
				1.4166666269302368);
		addBone("RightRingIntermediateBone", "RightRingProximalBone", 0.022119224071502686, 0.7071068286895752,
				-0.7071068286895752, 5.338508302088485e-08, 5.338508302088485e-08, -0.7879298329353333,
				0.00996093824505806, 1.4166666269302368);
		addBone("RightRingDistalBone", "RightRingIntermediateBone", 0.016589462757110596, 0.7071068286895752,
				-0.7071068286895752, 5.338508302088485e-08, 5.338508302088485e-08, -0.8100490570068359,
				0.00996093824505806, 1.4166666269302368);
		addBone("RightLittleProximalBone", "RightHandBone", 0.022318542003631592, 0.7071068286895752,
				-0.7071068286895752, 5.338508302088485e-08, 5.338508302088485e-08, -0.7584375143051147,
				0.02988281287252903, 1.4166666269302368);
		addBone("RightLittleIntermediateBone", "RightLittleProximalBone", 0.0167388916015625, 0.7071068286895752,
				-0.7071068286895752, 5.338508302088485e-08, 5.338508302088485e-08, -0.7807560563087463,
				0.02988281287252903, 1.4166666269302368);
		addBone("RightLittleDistalBone", "RightLittleIntermediateBone", 0.012554168701171875, 0.7071068286895752,
				-0.7071068286895752, 5.338508302088485e-08, 5.338508302088485e-08, -0.7974949479103088,
				0.02988281287252903, 1.4166666269302368);
		addBone("LeftUpperLegBone", "HipsBone", 0.42234373092651367, -0.5, 0.5, -0.5, 0.5, 0.05312500149011612, 0.0,
				0.9296875);
		addBone("LeftLowerLegBone", "LeftUpperLegBone", 0.42234379053115845, -0.5, 0.5, -0.5, 0.5, 0.05312500149011612,
				0.0, 0.5073437690734863);
		addBone("LeftFootBone", "LeftLowerLegBone", 0.19305294752120972, -0.6888130307197571, 0.1598014384508133,
				-0.6888131499290466, 0.15980160236358643, 0.05312500149011612, 0.0, 0.08499997854232788);
		addBone("LeftToesBone", "LeftFootBone", 0.08666664361953735, 0.7071068286895752, 2.5121476791422227e-15,
				-0.7071067094802856, 4.214686555314984e-08, 0.05312500149011612, -0.1733333170413971,
				-2.2351741790771484e-08);
		addBone("RightUpperLegBone", "HipsBone", 0.42234373092651367, -0.5, 0.5, -0.5, 0.5, -0.05312500149011612, 0.0,
				0.9296875);
		addBone("RightLowerLegBone", "RightUpperLegBone", 0.42234379053115845, -0.5, 0.5, -0.5, 0.5,
				-0.05312500149011612, 0.0, 0.5073437690734863);
		addBone("RightFootBone", "RightLowerLegBone", 0.19305294752120972, -0.6888130307197571, 0.1598014384508133,
				-0.6888131499290466, 0.15980160236358643, -0.05312500149011612, 0.0, 0.08499997854232788);
		addBone("RightToesBone", "RightFootBone", 0.08666664361953735, 0.7071068286895752, 2.5121476791422227e-15,
				-0.7071067094802856, 4.214686555314984e-08, -0.05312500149011612, -0.1733333170413971,
				-2.2351741790771484e-08);
		Bone rootBone = boneMap.get("");
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
