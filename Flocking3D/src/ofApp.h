#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxAssimpModelLoader.h"
#include <glm/gtx/intersect.hpp>

class Boid {
public:
	Boid() {
		position = glm::vec3(0, 0, 0);
	}

	Boid(glm::vec3 p) {
		position = p;
	}

	// get boid's transformation matrix
	glm::mat4 getTransform() {
		glm::mat4 T = glm::translate(glm::mat4(1.0), position);
		glm::mat4 R = getRotationMatrix();
		glm::mat4 S = glm::scale(glm::mat4(1.0), scale);

		return (T * R * S);
	}

	// 3D rotation matrix
	glm::mat4 getRotationMatrix() {
		glm::mat4 rX = glm::rotate(glm::mat4(1.0), glm::radians(rotation.x), glm::vec3(1, 0, 0));
		glm::mat4 rY = glm::rotate(glm::mat4(1.0), glm::radians(rotation.y), glm::vec3(0, 1, 0));
		glm::mat4 rZ = glm::rotate(glm::mat4(1.0), glm::radians(rotation.z), glm::vec3(0, 0, 1));

		return rZ * rY * rZ;
	}

	// get boid's heading direction
	glm::vec3 heading() {
		glm::mat4 r = getRotationMatrix();
		return glm::normalize(r * glm::vec4(0, 0, -1, 1));
	}

	glm::mat4 rotateToVector(glm::vec3 p) {
		glm::vec3 axis = glm::cross(position, p);
		glm::quat q = glm::angleAxis(glm::angle(position, p), glm::normalize(axis));
		return glm::toMat4(q);
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
		glm::vec3 angAccel = angularAcceleration;
		angAccel += angularForce / mass;
		angularVelocity += angAccel * dt;

		// multiply final result by the damping factor to sim drag
		velocity *= damping;
		angularVelocity *= angularDamping;

		// reset all forces
		force = glm::vec3(0, 0, 0);
		angularVelocity *= angularDamping;

		// reset all forces
		force = glm::vec3(0, 0, 0);
		angularForce = glm::vec3(0, 0, 0);
	}

	void turnBoid(glm::vec3 p);

	// boid traits
	glm::vec3 position;
	glm::vec3 header = glm::vec3(0, 0, -3);
	glm::vec3 scale = glm::vec3(1, 1, 1);
	float mass = 1.0; // placeholder

	// boid model
	ofColor modelColor;
	ofColor headerColor;
	int animState = 0;
	int animUpdate = 1;
	float timer = 0;

	// 3d motion
	glm::vec3 velocity = glm::vec3(0, 0, 0);
	glm::vec3 acceleration = glm::vec3(0, 0, 0);
	glm::vec3 force = glm::vec3(0, 0, 0);
	float damping = 0.99;

	// angular motion
	glm::vec3 rotation = glm::vec3(0, 0, 0);
	glm::vec3 angularVelocity = glm::vec3(0, 0, 0);
	glm::vec3 angularAcceleration = glm::vec3(0, 0, 0);
	glm::vec3 angularForce = glm::vec3(0, 0, 0);
	float angularDamping = .95;

	float predatorDist = -std::numeric_limits<float>::infinity();

};

class RobotBoid : public Boid {
public:
	RobotBoid() {
		position = glm::vec3(0, 0, 0);
		damping = 0.97;
	}

	RobotBoid(glm::vec3 p) {
		position = p;
		damping = 0.97;
	}

	void integrate() {
		// calculate time interval
		float dt = 1.0 / ofGetFrameRate();

		// update position from velocity & time interval
		position += velocity * dt;

		// update velocity (from acceleration)
		glm::vec3 accel = acceleration;
		accel += (force * 1.0 / mass);
		velocity += accel * dt;

		// update rotation from angular velocity & time
		rotation += angularVelocity * dt;

		// update angular velocity (from angular acceleration)
		glm::vec3 angAccel = angularAcceleration;
		angAccel += angularForce / mass;
		angularVelocity += angAccel * dt;

		// multiply final result by the damping factor to sim drag
		velocity *= damping;
		angularVelocity *= angularDamping;

		// reset all forces
		force = glm::vec3(0, 0, 0);
		angularVelocity *= angularDamping;

		// reset all forces
		force = glm::vec3(0, 0, 0);
		angularForce = glm::vec3(0, 0, 0);
	}
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
	void createBoid();

	glm::vec3 separate(Boid* boid, int index);
	glm::vec3 cohesion(Boid* boid, int index);
	glm::vec3 align(Boid* boid, int index);

	bool getMouseIntersect(glm::vec3 p);

	map<int, bool> keymap;
	ofEasyCam* theCam; // current camera view
	ofEasyCam freeCam, robotCam;
	glm::vec3 robotCamPos = glm::vec3(0, 1, 0);
	glm::vec3 rbLookAt; // point for robotCam to look at
	ofLight light;

	glm::vec3 minBounds = glm::vec3(-30, 0, -30);
	glm::vec3 maxBounds = glm::vec3(30, 30, 30);
	glm::vec3 targetPoint = glm::vec3(0, 0, 0);
	glm::vec3 mouseIntersect = glm::vec3(0, 0, 0);


	// flock
	RobotBoid* robotBoid;
	bool rbIntegrate = false;
	vector<Boid*> flock;
	vector<ofxAssimpModelLoader*> boidModels; // shared between entire flock
	vector<ofMaterial> materials;
	float headerYOffset;
	float modelRadius = 0;
	bool bWireFrame = false;
	float animTime = 100;


	// gui
	bool bHide;
	ofxPanel gui;
	ofParameter<bool> startSim;
	ofParameter<bool> targetMode;
	ofParameter<bool> sep, coh, ali;

	ofParameterGroup robotSettings;
	ofParameter<bool> predatorMode, leaderMode;
	ofParameter<float> thrust;

	ofParameterGroup flockSettings;
	ofParameter<int> numBoids;
	ofParameter<float> scale;
	ofParameter<float> neighborDist;
	ofParameter<float> separationVal;
	ofParameter<float> fleeSpeed;
	ofParameter<bool> toggleHeader;

	ofParameterGroup movement;
	ofParameter<float> flapFreq;
	ofParameter<float> minSpeed, maxSpeed, turnSpeed;

};
