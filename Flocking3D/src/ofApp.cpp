#include "ofApp.h"


void Boid::turnBoid(glm::vec3 p) {
	ofApp* theApp = (ofApp*)ofGetAppPtr();

	// find angle between heading & target point
	glm::vec3 h = heading();
	glm::vec3 v = glm::normalize(p - position);
	
	// x axis (yz plane) where x is constant
	glm::vec3 vX = glm::vec3(0, v.y, v.z);
	float dotProductX = glm::dot(h, vX);

	// y axis (xz plane) where y is constant
	glm::vec3 vY = glm::vec3(v.x, 0, v.z);
	float dotProductY = glm::dot(h, vY);

	// z axis (xy plane) where z is constant
	glm::vec3 vZ = glm::vec3(v.x, v.y, 0);
	float dotProductZ = glm::dot(h, v);

	float eps = 0.3; // margin of error for turning

	// add angular force for each axis
	glm::vec3 crossProduct;
	if (dotProductX < (1.0 - eps)) {
		crossProduct = glm::cross(h, vX);
		angularForce.x = theApp->turnSpeed;
		angularForce.x *= (crossProduct.x > 0) ? -1 : 1;
	}

	if (dotProductY < (1.0 - eps)) {
		crossProduct = glm::cross(h, vY);
		angularForce.y = theApp->turnSpeed;
		angularForce.y *= (crossProduct.y > 0) ? -1 : 1;
	}

	if (dotProductZ < (1.0 - eps)) {
		glm::vec3 crossProduct = glm::cross(h, vZ);
		angularForce.z = theApp->turnSpeed;
		angularForce.z *= (crossProduct.z > 0) ? 1 : -1;
	}
}

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetBackgroundColor(ofColor::lightGray);

	// camera setup
	theCam.setDistance(10);
	theCam.setNearClip(.1);


	// gui setup
	bHide = false;
	gui.setup("Simulation Settings");
	gui.add(startSim.set("Start Simulation (S)", false));
	gui.add(targetMode.set("Target Mode (T)", false));
	gui.add(sep.set("Separation", true));
	gui.add(coh.set("Cohesion", true));
	gui.add(ali.set("Alignment", true));

	flockSettings.setName("Flock Settings");
	flockSettings.add(numBoids.set("# of Boids", 1, 1, 100));
	flockSettings.add(scale.set("Boid Scale", 1, 1, 5));
	flockSettings.add(neighborDistance.set("Neighbor Distance", 20, 10, 100));
	flockSettings.add(separationValue.set("Desired Separation", 250, 100, 500));
	flockSettings.add(toggleHeader.set("Toggle Boid Headers", false));

	movement.setName("Boid Movement");
	movement.add(minSpeed.set("Min Speed", 25, 0, 100));
	movement.add(maxSpeed.set("Max Speed", 100, 0, 100));
	movement.add(turnSpeed.set("Turn Speed", 50, 0, 100));

	gui.add(flockSettings);
	gui.add(movement);


	// load models


	// light setup
	ofSetSmoothLighting(true);
	light1.enable();
	light1.setPosition(0, 20, 0);
	light1.setDiffuseColor(ofColor::white);
	light1.setSpecularColor(ofColor::white);
	light1.setAmbientColor(ofColor(150, 150, 150));

	// flock setup
	createFlock();


	// target point
	targetPoint = glm::vec3(0, 0, 0);
}

// create new flock
void ofApp::createFlock() {
	flock.clear();
	float w = ofGetWindowWidth(); // CHANGE BOUNDS FOR 3D
	float h = ofGetWindowHeight();
	
	glm::vec3 minBounds = theCam.screenToWorld(glm::vec3(0, 0, 0));
	minBounds.z = theCam.getPosition().z + theCam.getNearClip();
	glm::vec3 maxBounds = theCam.screenToWorld(glm::vec3(w, h, 0));
	

	for (int i = 0; i < numBoids; i++) {
		createBoid(minBounds, maxBounds);
	}
}

// create random new boid within bounds of window - CHANGE TO WORLD COORDS
void ofApp::createBoid(glm::vec3 min, glm::vec3 max) {
	
	//Boid* b = new Boid(glm::vec3(ofRandom(min.x, max.x), ofRandom(min.y, max.y), ofRandom(min.z, max.z)));
	Boid* b = new Boid(glm::vec3(0, 0, 0)); // tmp for 3D
	b->rotation = glm::vec3(ofRandom(0, 359), ofRandom(0, 359), ofRandom(0, 359));
	b->scale = glm::vec3(scale, scale, scale);
	
	// randomly select starting animation
	b->animState = (int)ofRandom(0, boidModel.size());

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
		if ((dist > 0) && (dist < separationValue)) {
			
			// find direction from neighbor to boid
			glm::vec3 diff = glm::normalize(boid->position - b->position);

			direction += diff / dist;
			numNeighbors++;
		}
	}

	if (numNeighbors > 0) {
		// return avg direction away from neighbors
		direction /= numNeighbors;
		return direction;
	}

	return direction; // 0, 0, 0
}

// find center of a neighborhood of boids and push them towards it
glm::vec3 ofApp::cohesion(Boid* boid, int index) {
	glm::vec3 avgPosition = glm::vec3(0, 0, 0);
	float numNeighbors = 0;

	for (int i = 0; i < flock.size(); i++) {
		if (i == index) continue;
		Boid* b = flock[i];

		// determine if b is a neighbor
		float dist = glm::distance(boid->position, b->position);
		if ((dist > 0) && (dist < neighborDistance)) {
			avgPosition += b->position;
			numNeighbors++;
		}
	}

	if (numNeighbors > 0) {
		// return direction to avg position
		avgPosition /= numNeighbors;
		return avgPosition - boid->position;
	}

	return avgPosition; // 0, 0, 0
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
		if ((dist > 0) && (dist < neighborDistance)) {
			avgHeading += b->heading();
			avgSpeed = glm::length(b->velocity);
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
		return avgHeading * avgSpeed;
	}

	return avgHeading; // 0, 0, 0
}

//--------------------------------------------------------------
void ofApp::update() {
	float width = ofGetWindowWidth();
	float height = ofGetWindowHeight();
	glm::vec3 bounds = theCam.screenToWorld(glm::vec3(width, height, 0));
	
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
			//createBoid(bounds.x, bounds.y);
		}
	}


	// update all boids in the flock
	for (int i = 0; i < flock.size(); i++) {

		Boid* b = flock[i];
		b->scale = glm::vec3(scale, scale, scale);
		b->bToggleHeader = toggleHeader;
		b->drawWireFrame = bWireFrame;


		// target mode - test turn & movement
		if (targetMode) {

			// update boid animation by switching to next model
			//b->model = boidModel[b->animState++];

			b->turnBoid(targetPoint);

			if (startSim) b->force = targetPoint - b->position;
			else b->force = glm::vec3(0, 0, 0);

			b->integrate();
			continue;
		}


		// flocking simulation
		if (startSim) {

			// update boid animation by switching to next model
			//b->model = boidModel[b->animState++];
			

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


			// wrap around edges of window
			// FOR 3D - MAKE BOUNDS BASED ON CAMERA VIEW?
			// FAR DISTANCE BOUND BY CAMERA DISTANCE (ZOOM)?
			/*if (b->position.x < 0) b->position.x += width;
			else if (b->position.x > width) b->position.x -= width;

			if (b->position.y < 0) b->position.y += height;
			else if (b->position.y > height) b->position.y -= height;*/
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofEnableDepthTest();
	theCam.begin();
	//ofEnableLighting();

	// grid for ground plane reference
	ofPushMatrix();
	ofSetColor(ofColor::dimGray);
	ofRotateDeg(90);
	ofDrawGridPlane();
	ofPopMatrix();

	if (targetMode) {
		ofSetColor(ofColor::orange);
		ofDrawSphere(targetPoint, 0.2);
	}
	
	//material.begin();

	// draw flock
	for (Boid* b : flock) {
		b->draw();
		//if (bWireFrame) // draw boid model as wireframe
		// model.drawFaces()
		// boidModel[b->animState].drawFaces();
		// move draw method here to call on boidModel?
	}

	//material.end();
	//ofDisableLighting();

	theCam.end();
	ofDisableDepthTest();

	// draw gui
	if (!bHide) gui.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	keymap[key] = true;

	if (keymap['h'] || keymap['H']) bHide = !bHide;

	if (keymap['f'] || keymap['F']) ofToggleFullscreen();

	if (keymap['s'] || keymap['S']) startSim = !startSim;

	// reset flock
	if (keymap['r'] || keymap['R']) createFlock();

	if (keymap['w'] || keymap['W']) bWireFrame = !bWireFrame;

	if (keymap['t'] || keymap['T']) targetMode = !targetMode;
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
	keymap[key] = false;
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ) {}
//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {}

// intersect mouse ray with scene based on camera
bool ofApp::getMouseIntersect(glm::vec3 p) {

	glm::vec3 origin = theCam.getPosition();
	glm::vec3 camAxis = theCam.getZAxis();
	glm::vec3 mouseWorld = theCam.screenToWorld(p);
	glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);
	float distance;

	bool hit = glm::intersectRayPlane(origin, mouseDir, glm::vec3(0, 0, 0), camAxis, distance);

	if (hit) mouseIntersect = origin + distance * mouseDir;

	return hit;
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	if (keymap[OF_KEY_CONTROL] &&  getMouseIntersect(glm::vec3(x, y, 0))) {

		if (targetMode) targetPoint = mouseIntersect;
		else {
			Boid* b = new Boid(mouseIntersect);
			b->rotation = glm::vec3(ofRandom(0, 359), ofRandom(0, 359), ofRandom(0, 359));
			b->scale = glm::vec3(scale, scale, scale);

			// randomly select starting animation
			b->animState = (int)ofRandom(0, boidModel.size());

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
