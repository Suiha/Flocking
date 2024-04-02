#include "ofApp.h"


void Boid::turnBoid(glm::vec3 p) {
	ofApp* theApp = (ofApp*)ofGetAppPtr();

	glm::mat4 rot = rotateToVector(p);
	glm::vec3 eulerAngles = glm::eulerAngles(glm::quat_cast(rot));
	float eps = 0.4;

	glm::vec3 crossProduct = glm::cross(heading(), p - position);
	if (eulerAngles.x < (1.0 - eps)) {
		angularForce.x = theApp->turnSpeed;
		angularForce.x *= (crossProduct.x > 0) ? -1 : 1;
	}

	if (eulerAngles.y < (1.0 - eps)) {
		angularForce.y = theApp->turnSpeed;
		angularForce.y *= (crossProduct.y > 0) ? -1 : 1;
	}

	if (eulerAngles.z < (1.0 - eps)) {
		angularForce.z = theApp->turnSpeed;
		angularForce.z *= (crossProduct.z > 0) ? 1 : -1;
	}
}

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetBackgroundColor(ofColor::lightGray);


	// gui setup
	bHide = false;
	gui.setup("Simulation Settings");
	gui.add(startSim.set("Start Simulation (SPC)", false));
	gui.add(targetMode.set("Target Mode (T)", false));
	gui.add(toggleHeader.set("Toggle Boid Headers", true));
	gui.add(sep.set("Separation", true));
	gui.add(coh.set("Cohesion", true));
	gui.add(ali.set("Alignment", true));

	robotSettings.setName("Robot Boid Settings");
	robotSettings.add(predatorMode.set("Predator Mode (P)", false));
	robotSettings.add(leaderMode.set("Leader Mode (L)", false));
	robotSettings.add(thrust.set("Thrust", 25, 10, 50));

	flockSettings.setName("Flock Settings");
	flockSettings.add(numBoids.set("# of Boids", 1, 0, 500));
	flockSettings.add(scale.set("Boid Scale", 1, 1, 5));
	flockSettings.add(neighborDist.set("Neighbor Distance", 40, 10, 50));
	flockSettings.add(separationVal.set("Desired Separation", 10, 1, 100));
	flockSettings.add(fleeSpeed.set("Flee Speed", 5, 1, 10));

	movement.setName("Flock Movement");
	movement.add(flapFreq.set("Flap Frequency", 1, 1, 10));
	movement.add(minSpeed.set("Min Speed", 1, 1, 5));
	movement.add(maxSpeed.set("Max Speed", 4, 1, 5));
	movement.add(turnSpeed.set("Turn Speed", 50, 0, 100));

	gui.add(robotSettings);
	gui.add(flockSettings);
	gui.add(movement);


	// load model
	// this specific fish model has 7 animation states (0-6)
	for (int i = 0; i < 7; ++i) {
		auto model = make_unique<ofxAssimpModelLoader>();
		string path = "geo/fish-" + to_string(i);

		if (model->loadModel(path + ".obj")) {

			// correct model position so that it's center is at 0, 0, 0
			glm::vec3 minBound = model->getSceneMin();
			glm::vec3 maxBound = model->getSceneMax();
			glm::vec3 center = (minBound + maxBound) / 2;
			headerYOffset = center.y / 2;

			// determine max dimension of model
			if (center.x > modelRadius) modelRadius = center.x;
			if (center.y > modelRadius) modelRadius = center.y;
			if (center.z > modelRadius) modelRadius = center.z;

			model->setScaleNormalization(false);
			model->setScale(0.5, -0.5, -0.5);

			materials.push_back(model->getMaterialForMesh(path + ".mtl"));
			boidModels.push_back(model.release());
		}
		else {
			cout << "error loading " + path + ".obj" << endl;
		}
	}
	cout << modelRadius << endl;

	// light setup
	ofSetSmoothLighting(true);
	light.enable();
	light.setPosition(0, 20, 0);
	light.setDiffuseColor(ofColor::white);
	light.setSpecularColor(ofColor::white);
	light.setAmbientColor(ofColor(150, 150, 150));


	// flock & robotBoid setup
	createFlock();
	robotBoid = new RobotBoid(glm::vec3(0, 0, 0)); // default parameters: center, no speed
	robotBoid->header.y = headerYOffset;
	robotBoid->modelColor = ofColor::dimGray;
	robotBoid->headerColor = ofColor::red;
	robotBoid->animState = 0;


	// camera setup
	theCam = &freeCam;
	freeCam.setDistance(10);
	freeCam.setNearClip(.1);
	robotCamPos = robotBoid->position + glm::vec3(0, 0, -1);
	robotCam.setPosition(robotCamPos);
	rbLookAt = robotBoid->position + robotBoid->heading();
	robotCam.lookAt(rbLookAt);


	// target point
	targetPoint = glm::vec3(0, 0, 0);
}

// create new flock
void ofApp::createFlock() {
	flock.clear();
	/*float w = ofGetWindowWidth(); // CHANGE BOUNDS FOR 3D
	float h = ofGetWindowHeight();

	glm::vec3 minBounds = theCam.screenToWorld(glm::vec3(0, 0, 0));
	minBounds.z = theCam.getPosition().z + theCam.getNearClip();
	glm::vec3 maxBounds = theCam.screenToWorld(glm::vec3(w, h, 0));*/


	for (int i = 0; i < numBoids; i++) {
		createBoid();
	}
}

// create random new boid within bounds
void ofApp::createBoid() {
	Boid* b = new Boid(glm::vec3(ofRandom(minBounds.x, maxBounds.x),
		ofRandom(minBounds.y, maxBounds.y), ofRandom(minBounds.z, maxBounds.z)));
	b->header.y = headerYOffset;
	b->rotation = glm::vec3(ofRandom(0, 359), ofRandom(0, 359), ofRandom(0, 359));
	b->scale = glm::vec3(scale, scale, scale);

	// randomly select starting animation
	b->modelColor = ofColor::lightBlue;
	b->headerColor = ofColor::green;
	b->animState = (int)ofRandom(0, boidModels.size());

	// initial speed
	b->force = b->heading() * ofRandom(minSpeed, maxSpeed);

	flock.push_back(b);
}

// push boid away from neighbors
glm::vec3 ofApp::separate(Boid* boid, int index) {
	glm::vec3 direction = glm::vec3(0, 0, 0);
	float numNeighbors = 0;

	for (int i = 0; i < flock.size(); i++) {
		if (i == index) continue;
		Boid* b = flock[i];

		// determine if b is a neighbor
		float dist = glm::distance(boid->position, b->position);
		if ((dist < (modelRadius * 2)) && (dist < separationVal)) {

			// find direction from neighbor to boid
			glm::vec3 diff = glm::normalize(boid->position - b->position);

			direction += diff / dist;
			numNeighbors++;
		}
	}

	glm::vec3 robotForce = glm::vec3(0, 0, 0);
	if (predatorMode) { // predator mode: flee from robot boid

		float dist = glm::distance(boid->position, robotBoid->position);

		// check if robot boid is in range AND getting closer
		if ((dist > 0) && (dist < neighborDist) && (dist < boid->predatorDist)) {

			// direction away from robot boid
			glm::vec3 diff = boid->position - robotBoid->position;
			robotForce = diff * fleeSpeed.get();
		}

		boid->predatorDist = dist;
	}
	else if (leaderMode) { // leader mode: boid is following robot boid, maintain regular separation

		float dist = glm::distance(boid->position, robotBoid->position);
		if ((dist > 0) && (dist < separationVal)) {

			// find direction from neighbor to robot boid
			glm::vec3 diff = glm::normalize(boid->position - robotBoid->position);

			robotForce = diff / dist;
			numNeighbors++;
		}
	}

	if (numNeighbors > 0) {
		// return avg direction away from neighbors
		direction /= numNeighbors;
		return direction + robotForce;
	}

	return robotForce; // no neighbors
}

// find center of a neighborhood of boids and push them towards it
glm::vec3 ofApp::cohesion(Boid* boid, int index) {
	glm::vec3 avgPosition = glm::vec3(0, 0, 0);
	float numNeighbors = 0;

	for (int i = 0; i < flock.size(); i++) {
		if (i == index) continue;
		Boid* b = flock[i];

		// determine if b is a neighbor & prevent from moving closer if their spaces are overlapping
		float dist = glm::distance(boid->position, b->position);
		if ((dist > (modelRadius * 2)) && (dist < neighborDist)) {
			avgPosition += b->position;
			numNeighbors++;
		}
	}

	// leader (robot boid) has greater say on position of flock
	glm::vec3 robotForce = glm::vec3(0, 0, 0);
	if (leaderMode) {
		float dist = glm::distance(boid->position, robotBoid->position);
		if ((dist > (modelRadius * 2)) && (dist < neighborDist)) {
			glm::vec3 diff = robotBoid->position - boid->position;
			robotForce = diff * fleeSpeed.get();
		}
	}

	if (numNeighbors > 0) {
		// return direction to avg position
		avgPosition /= numNeighbors;
		return (avgPosition - boid->position) + robotForce;
	}

	return robotForce; // no neighbors
}

// get difference between boid velocity & average velocity of neighbors
glm::vec3 ofApp::align(Boid* boid, int index) {
	glm::vec3 avgHeading = glm::vec3(0, 0, 0);
	float avgSpeed = 0;
	float numNeighbors = 0;

	// get velocity of neighboring boids
	for (int i = 0; i < flock.size(); i++) {
		if (i == index) continue;
		Boid* b = flock[i];

		// determine if b is a neighbor
		float dist = glm::distance(boid->position, b->position);
		if ((dist > 0) && (dist < neighborDist)) {
			avgHeading += b->heading();
			avgSpeed = glm::length(b->velocity);
			numNeighbors++;
		}
	}

	// leader (robot boid) has greater say on velocity of flock
	glm::vec3 robotForce = glm::vec3(0, 0, 0);
	if (leaderMode) {
		float dist = glm::distance(boid->position, robotBoid->position);
		if ((dist > 0) && (dist < neighborDist)) {
			robotForce = robotBoid->heading() * glm::length(robotBoid->velocity);
			numNeighbors++;
		}
	}

	if (numNeighbors > 0) {
		// get average heading, speed of neighbors
		avgHeading /= numNeighbors;
		avgSpeed /= numNeighbors;

		// cap boid velocity
		if (abs(glm::length(boid->velocity)) > maxSpeed) avgSpeed = 0;

		// return avg velocity
		return (avgHeading * avgSpeed) + robotForce;
	}

	return robotForce; // no neighbors
}

//--------------------------------------------------------------
void ofApp::update() {
	/*float width = ofGetWindowWidth();
	float height = ofGetWindowHeight();
	glm::vec3 bounds = theCam.screenToWorld(glm::vec3(width, height, 0));*/


	// update robot boid
	animTime = 5000 / (10 * flapFreq);
	if (ofGetElapsedTimeMillis() - robotBoid->timer >= animTime) {
		if (robotBoid->animState == 0) robotBoid->animUpdate = 1;
		else if (robotBoid->animState == boidModels.size() - 1) {
			robotBoid->animUpdate = -1;
		}

		robotBoid->animState += robotBoid->animUpdate;
		robotBoid->timer = ofGetElapsedTimeMillis();
	}
	if (rbIntegrate) {
		robotBoid->integrate();
	}
	else if (glm::length(robotBoid->velocity) == 0 && glm::length(robotBoid->angularVelocity) == 0) {
		rbIntegrate = false;
	}


	// update robot boid cam
	robotCamPos = robotBoid->position + glm::vec3(0, 0, 1);
	robotCam.setPosition(robotCamPos);
	rbLookAt = robotBoid->position + robotBoid->heading();
	robotCam.lookAt(rbLookAt);


	// update flock size based on numBoids slider
	if (numBoids < flock.size()) {

		// decrease flock size
		int diff = flock.size() - numBoids;
		for (int i = 0; i < diff; i++) {
			flock.pop_back();
		}
	}
	else if (numBoids > flock.size()) {

		// increase flock size
		int diff = numBoids - flock.size();
		for (int i = 0; i < diff; i++) {
			createBoid();
		}
	}


	// update all boids in the flock
	for (int i = 0; i < flock.size(); i++) {

		Boid* b = flock[i];
		b->scale = glm::vec3(scale, scale, scale);


		// update boid animation by switching to next model
		animTime = 5000 / (10 * flapFreq);
		if (ofGetElapsedTimeMillis() - b->timer >= animTime) {
			if (b->animState == 0) b->animUpdate = 1;
			else if (b->animState == boidModels.size() - 1) b->animUpdate = -1;

			b->animState += b->animUpdate;
			b->timer = ofGetElapsedTimeMillis();
		}


		// target mode - test turn & movement
		if (targetMode) {

			b->turnBoid(targetPoint);

			if (startSim) b->force = targetPoint - b->position;
			else b->force = glm::vec3(0, 0, 0);

			b->integrate();
			continue;
		}


		// flocking simulation
		if (startSim) {

			// determine boid movement based on flock algorithm
			// separation: keep boid a certain distance from neighbors
			if (sep) b->force += separate(b, i);

			// cohesion: keep boid a certain distance within neighbors
			if (coh) b->force += cohesion(b, i);

			// alignment: match boid speed with neighbor's speeds
			if (ali) b->force += align(b, i);

			// turn boid towards direction its moving
			b->turnBoid(b->position + b->velocity);

			// integrate
			b->integrate();

			// cap velocity
			if (glm::length(b->velocity) > maxSpeed) {
				b->velocity = glm::normalize(b->velocity) * maxSpeed.get();
			}

			// wrap around edges of bounds
			// FOR 3D - MAKE BOUNDS BASED ON CAMERA VIEW?
			// FAR DISTANCE BOUND BY CAMERA DISTANCE (ZOOM)?
			// make if position + velocity * dt > bounds, add opposing force? or just wrap around
			if (b->position.x < minBounds.x) b->position.x += (maxBounds.x - minBounds.x);
			else if (b->position.x > maxBounds.x) b->position.x -= (maxBounds.x - minBounds.x);

			if (b->position.y < minBounds.y) b->position.y += (maxBounds.y - minBounds.y);
			else if (b->position.y > maxBounds.y) b->position.y -= (maxBounds.y - minBounds.y);

			if (b->position.z < minBounds.z) b->position.z += (maxBounds.z - minBounds.z);
			else if (b->position.z > maxBounds.z) b->position.z -= (maxBounds.z - minBounds.z);
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofEnableDepthTest();
	theCam->begin();
	ofEnableLighting();


	// grid for ground plane reference
	ofPushMatrix();
	ofSetColor(ofColor::dimGray);
	ofRotateDeg(90);
	ofDrawGridPlane();
	ofPopMatrix();


	// draw target point
	if (targetMode) {
		ofSetColor(ofColor::orange);
		ofDrawSphere(targetPoint, 0.2);
	}


	// draw robot boid
	ofPushMatrix();
	ofMultMatrix(robotBoid->getTransform());

	if (toggleHeader) { // show boid direction
		ofSetColor(robotBoid->headerColor);
		ofDrawLine(glm::vec3(0, headerYOffset, 0), robotBoid->header);
	}

	if (bWireFrame) {
		ofSetColor(robotBoid->modelColor);
		boidModels[robotBoid->animState]->drawWireframe();
	}
	else {
		materials[robotBoid->animState].setDiffuseColor(robotBoid->modelColor);
		materials[robotBoid->animState].begin();

		ofSetColor(robotBoid->modelColor);
		boidModels[robotBoid->animState]->enableMaterials();
		boidModels[robotBoid->animState]->enableColors();
		boidModels[robotBoid->animState]->enableNormals();
		boidModels[robotBoid->animState]->drawFaces();

		materials[robotBoid->animState].end();
	}

	ofPopMatrix();


	// draw flock
	for (Boid* b : flock) {
		ofPushMatrix();
		ofMultMatrix(b->getTransform());

		if (toggleHeader) { // show boid direction
			ofSetColor(b->headerColor);
			ofDrawLine(glm::vec3(0, headerYOffset, 0), b->header);
		}

		if (bWireFrame) {
			ofSetColor(b->modelColor);
			boidModels[b->animState]->drawWireframe();
		}
		else {
			ofEnableLighting();

			boidModels[b->animState]->enableMaterials();
			boidModels[b->animState]->enableColors();
			boidModels[b->animState]->enableNormals();
			boidModels[b->animState]->drawFaces();

			ofDisableLighting();
		}

		ofPopMatrix();
	}


	ofDisableLighting();
	theCam->end();
	ofDisableDepthTest();


	// draw gui
	if (!bHide) gui.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	keymap[key] = true;

	// show/hide gui
	if (keymap['h'] || keymap['H']) bHide = !bHide;

	if (keymap['f'] || keymap['F']) ofToggleFullscreen();

	// freely look at world space
	if (keymap[OF_KEY_F1]) theCam = &freeCam;

	// view robot boid's pov
	if (keymap[OF_KEY_F2]) theCam = &robotCam;

	// start/stop simulation
	if (keymap[' ']) startSim = !startSim;

	// reset all boids
	if (keymap['r'] || keymap['R']) {
		createFlock();
		robotBoid->position = glm::vec3(0, 0, 0);
		robotBoid->velocity = glm::vec3(0, 0, 0);
		robotBoid->rotation = glm::vec3(0, 0, 0);
	}

	// enable/disable target mode
	if (keymap['t'] || keymap['T']) targetMode = !targetMode;

	// enable/disable wireframe on models
	if (keymap['z'] || keymap['Z']) bWireFrame = !bWireFrame;

	// enable/disable predator mode for robot boid
	if (keymap['p'] || keymap['P']) {
		predatorMode = !predatorMode;
		if (predatorMode) leaderMode = false;
	}

	// enable/disable leader mode for robot boid
	if (keymap['l'] || keymap['L']) {
		leaderMode = !leaderMode;
		if (leaderMode) predatorMode = false;
	}


	// robot boid movement
	if (keymap[OF_KEY_UP]) { // move forward
		robotBoid->force = robotBoid->heading() * thrust.get();
		rbIntegrate = true;
	}

	if (keymap['a'] || keymap['A']) { // turn left about y-axis
		robotBoid->angularForce = glm::vec3(0, 1, 0) * thrust.get() * 10;
		rbIntegrate = true;
	}

	if (keymap['d'] || keymap['D']) { // turn right about y-axis
		robotBoid->angularForce = glm::vec3(0, -1, 0) * thrust.get() * 10;
		rbIntegrate = true;
	}

	if (keymap['w'] || keymap['W']) { // lift up
		robotBoid->force = glm::vec3(0, 1, 0) * thrust.get();
		rbIntegrate = true;
	}

	if (keymap['s'] || keymap['S']) { // drip down
		robotBoid->force = glm::vec3(0, -1, 0) * thrust.get();
		rbIntegrate = true;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {
	keymap[key] = false;
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {}
//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {}

// intersect mouse ray with scene based on camera
bool ofApp::getMouseIntersect(glm::vec3 p) {

	glm::vec3 origin = theCam->getPosition();
	glm::vec3 camAxis = theCam->getZAxis();
	glm::vec3 mouseWorld = theCam->screenToWorld(p);
	glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);
	float distance;

	bool hit = glm::intersectRayPlane(origin, mouseDir, glm::vec3(0, 0, 0), camAxis, distance);

	if (hit) mouseIntersect = origin + distance * mouseDir;

	return hit;
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	if (keymap[OF_KEY_CONTROL] && getMouseIntersect(glm::vec3(x, y, 0))) {

		if (targetMode) targetPoint = mouseIntersect;
		else {
			Boid* b = new Boid(mouseIntersect);
			b->rotation = glm::vec3(ofRandom(0, 359), ofRandom(0, 359), ofRandom(0, 359));
			b->scale = glm::vec3(scale, scale, scale);
			b->modelColor = ofColor::lightBlue;
			b->headerColor = ofColor::green;

			// randomly select starting animation
			b->animState = (int)ofRandom(0, boidModels.size());

			// initial speed
			b->force = b->heading() * ofRandom(minSpeed, maxSpeed);

			flock.push_back(b);
			numBoids++;
		}
	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {}
//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {}
//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {}
//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {}
//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {}
//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {}
