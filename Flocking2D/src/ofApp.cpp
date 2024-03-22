#include "ofApp.h"


void Boid::turnBoid(glm::vec3 p) {
	ofApp* theApp = (ofApp*)ofGetAppPtr();

	// find angle between heading & target point
	glm::vec3 h = heading();
	glm::vec3 v = glm::normalize(p - position);
	float dotProduct = glm::dot(h, v);
	float eps = 0.3;

	if (dotProduct < (1.0 - eps)) {
		// turn clockwise/counterclockwise depending on axis of rotation
		glm::vec3 crossProduct = glm::cross(h, v);
		angularForce = theApp->turnSpeed;
		angularForce *= (crossProduct.z > 0) ? 1 : -1;
	}
}

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetBackgroundColor(ofColor::lightGray);

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

	forces.setName("Forces");
	forces.add(minTurbulence.set("Min Turbulence", glm::vec3(0, 0, 0), glm::vec3(-100, -100, -100),
		glm::vec3(100, 100, 100)));
	forces.add(maxTurbulence.set("Max Turbulence", glm::vec3(0, 0, 0), glm::vec3(-100, -100, -100),
		glm::vec3(100, 100, 100)));

	gui.add(flockSettings);
	gui.add(movement);
	gui.add(forces);


	// flock setup
	createFlock();


	// target point
	targetPoint = glm::vec3(ofGetWindowWidth() / 2, ofGetWindowHeight() / 2, 0);
}

// create new flock
void ofApp::createFlock() {
	flock.clear();
	float w = ofGetWindowWidth();
	float h = ofGetWindowHeight();

	for (int i = 0; i < numBoids; i++) {
		createBoid(w, h);
	}
}

// create random new boid within bounds of window
void ofApp::createBoid(float w, float h) {
	Boid* b = new Boid(glm::vec3(ofRandom(0, w), ofRandom(0, h), 0));
	b->rotation = ofRandom(0, 359);
	b->scale = glm::vec3(scale, scale, scale);

	// initial speed
	b->force = b->heading() * ofRandom(minSpeed, maxSpeed) * 100;

	flock.push_back(b);
}

// push boid away from neighbors NEEDS FIXING
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
		direction /= numNeighbors; // avg direction to neighbors

		// return difference between desired pos and current pos
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
		avgPosition /= numNeighbors;

		// return difference between desired pos and current pos
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
		// get average velocity of neighbors
		avgHeading /= numNeighbors;
		avgSpeed /= numNeighbors;

		// cap boid velocity
		if (abs(glm::length(boid->velocity)) > maxSpeed) avgSpeed = 0;

		glm::vec3 alignForce = avgHeading * avgSpeed;

		return alignForce;
	}

	return avgHeading; // 0, 0, 0
}

//--------------------------------------------------------------
void ofApp::update() {
	float width = ofGetWindowWidth();
	float height = ofGetWindowHeight();

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
			createBoid(width, height);
		}
	}


	// update all boids in the flock
	//glm::vec3 minT = minTurbulence.get() * 10;
	//glm::vec3 maxT = maxTurbulence.get() * 10;
	for (int i = 0; i < flock.size(); i++) {

		Boid* b = flock[i];
		b->scale = glm::vec3(scale, scale, scale);
		b->bToggleHeader = toggleHeader;


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

			// turbulence force
			/*b->force = glm::vec3(ofRandom(minT.x, maxT.x), ofRandom(minT.y, maxT.y),
				ofRandom(minT.z, maxT.z));*/

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
			if (b->position.x < 0) b->position.x += width;
			else if (b->position.x > width) b->position.x -= width;

			if (b->position.y < 0) b->position.y += height;
			else if (b->position.y > height) b->position.y -= height;
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw() {

	if (targetMode) {
		ofSetColor(ofColor::orange);
		ofDrawCircle(targetPoint, 10);
	}

	// draw flock
	for (Boid* b : flock) {
		b->draw();
	}

	// draw gui
	if (!bHide) gui.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	keymap[key] = true;

	if (keymap['h'] || keymap['H']) bHide = !bHide;

	if (keymap['s'] || keymap['S']) startSim = !startSim;

	// reset flock
	if (keymap['r'] || keymap['R']) createFlock();

	if (keymap['t'] || keymap['T']) targetMode = !targetMode;
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {
	keymap[key] = false;
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	if (targetMode) targetPoint = glm::vec3(x, y, 0);
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
	if (!targetMode) {

		// add new boid at mouse position
		Boid* b = new Boid(glm::vec3(x, y, 0));
		b->rotation = ofRandom(0, 359);
		b->scale = glm::vec3(scale, scale, scale);

		flock.push_back(b);
		numBoids++; // update slider
	}
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
