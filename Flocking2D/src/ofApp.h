#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include <glm/gtx/intersect.hpp>

class Boid {
public:
	Boid() {
		position = glm::vec3(0, 0, 0);
		verts.push_back(glm::vec3(-10, 15, 0));
		verts.push_back(glm::vec3(10, 15, 0));
		verts.push_back(glm::vec3(0, -15, 0));
	}

	Boid(glm::vec3 p) {
		position = p;
		verts.push_back(glm::vec3(-10, 15, 0));
		verts.push_back(glm::vec3(10, 15, 0));
		verts.push_back(glm::vec3(0, -15, 0));
	}

	// get boid's transformation matrix
	glm::mat4 getTransform() {
		glm::mat4 T = glm::translate(glm::mat4(1.0), position);
		glm::mat4 R = glm::rotate(glm::mat4(1.0), glm::radians(rotation), glm::vec3(0, 0, 1));
		glm::mat4 S = glm::scale(glm::mat4(1.0), scale);

		return (T * R * S);
	}

	// get boid's heading direction
	glm::vec3 heading() {
		glm::mat4 rot = glm::rotate(glm::mat4(1.0), glm::radians(rotation), glm::vec3(0, 0, 1));
		return glm::normalize(rot * glm::vec4(0, -1, 0, 1));
	}

	void draw() {
		ofPushMatrix();
		ofMultMatrix(getTransform());

		if (bToggleHeader) { // show boid direction
			ofSetColor(ofColor::red);
			ofDrawLine(glm::vec3(0, 0, 0), header);
		}

		ofFill();
		ofSetColor(ofColor::black);
		ofDrawTriangle(verts[0], verts[1], verts[2]);

		ofPopMatrix();
	}

	void integrate() {
		// calculate time interval
		float dt = 1.0 / ofGetFrameRate();

		// update position from velocity & time interval
		position += heading() * glm::length(velocity) * dt;

		// update velocity (from acceleration)
		glm::vec3 accel = acceleration;
		accel += (force * 1.0 / mass);
		velocity += accel * dt;

		// update rotation from angular velocity & time
		rotation += angularVelocity * dt;

		// update angular velocity (from angular acceleration)
		float angAccel = angularAcceleration;
		angAccel += angularForce / mass;
		angularVelocity += angAccel * dt;

		// multiply final result by the damping factor to sim drag
		//velocity *= damping;
		angularVelocity *= angularDamping;

		// reset all forces
		force = glm::vec3(0, 0, 0);
		angularVelocity *= angularDamping;

		// reset all forces
		force = glm::vec3(0, 0, 0);
		angularForce = 0;
	}

	void turnBoid(glm::vec3 p);

	// boid physical traits
	glm::vec3 position;
	glm::vec3 header = glm::vec3(0, -30, 0);
	vector<glm::vec3> verts; // vertices of the boid triangle
	glm::vec3 scale = glm::vec3(1, 1, 1);
	float mass = 1.0; // placeholder

	// 3d motion
	glm::vec3 velocity = glm::vec3(0, 0, 0);
	glm::vec3 acceleration = glm::vec3(0, 0, 0);
	glm::vec3 force = glm::vec3(0, 0, 0);
	float damping = 0.99;

	// angular motion
	float rotation = 0.0;
	float angularVelocity = 0;
	float angularAcceleration = 0;
	float angularForce = 0;
	float angularDamping = .95;

	bool bToggleHeader = false;
};


class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	void createFlock();
	void createBoid(float w, float h);

	glm::vec3 separate(Boid* boid, int index);
	glm::vec3 cohesion(Boid* boid, int index);
	glm::vec3 align(Boid* boid, int index);

	map<int, bool> keymap;
	vector<Boid*> flock;

	glm::vec3 targetPoint = glm::vec3(0, 0, 0);


	// gui
	bool bHide;
	ofxPanel gui;
	ofParameter<bool> startSim;
	ofParameter<bool> targetMode;
	ofParameter<bool> sep, coh, ali;

	ofParameterGroup flockSettings;
	ofParameter<int> numBoids;
	ofParameter<float> scale;
	ofParameter<float> neighborDistance;
	ofParameter<float> separationValue;
	ofParameter<bool> toggleHeader;

	ofParameterGroup movement;
	ofParameter<float> minSpeed;
	ofParameter<float> maxSpeed;
	ofParameter<float> turnSpeed;

	ofParameterGroup forces;
	ofParameter<glm::vec3> minTurbulence;
	ofParameter<glm::vec3> maxTurbulence;
};
