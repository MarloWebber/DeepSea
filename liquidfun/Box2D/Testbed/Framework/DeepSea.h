#ifndef DEEPSEA_H
#define DEEPSEA_H

#include "Test.h"
#include "Main.h"

#define N_FINGERS 8 // the amount of bones a fish can have. you know, fish fingers.
#define N_FISHES 8
#define N_FOODPARTICLES 8

b2Vec2 rotatePoint(float cx,float cy,float angle, b2Vec2 p);

struct JointUserData; // forward class declaration
struct BoneUserData;
struct BonyFish;

void setUserControlInputA() ;
void setUserControlInputB() ;

// a condensed form of fish used for data storage and transfer.
// it is made so that it is safe to serialize and mutate, and that any changes will not crash the program.
struct boneAndJointDescriptor_t {
		uint8_t attachedTo = 0; // the INDEX (out of N_FINGERS) of the bone it is attached to. Storing data in this way instead of a pointer means that mutating it will have hilarious rather than alarming results.
		float length = 0.1f;
		float rootThickness = 0.1f;
		float tipThickness = 0.1f;
		bool isRoot = false;
		bool isMouth = false;
		bool isSensor = false;
		bool isWeapon  = false;
		float torque = 0.0f;
		float speedLimit = 0.0f;
		float upperAngle = 0.0f;
		float normalAngle = 0.0f;
		float lowerAngle = 0.0f;
		bool used = false;
};

struct fishDescriptor_t {

	boneAndJointDescriptor_t bones[N_FINGERS];

	uint8_t heartSpeed;
};

struct JointUserData {
	float torque; 	
	float speed; 	
	float speedLimit;
	bool driveCW;	// a signal that tells the motor to turn in one direction. This is much simpler than trying to drive it with a number and having to remember positon etc. With this, you just hit the button, or don't.
	bool driveCCW;	// a signal that tells the motor to turn in the other direction.
	float upperAngle;
	float normalAngle;
	float lowerAngle;

	b2RevoluteJointDef jointDef;
	b2RevoluteJoint * p_joint; // the joint that this user data struct gets pinned to

	bool init;
	bool isUsed;

	BoneUserData * attaches;
	BoneUserData * attachedTo;

	JointUserData(boneAndJointDescriptor_t boneDescription, BoneUserData * p_bone, BonyFish * fish, b2World * m_world, b2ParticleSystem * m_particleSystem) ;
} ;

struct BoneUserData {
	float length;
	float rootThickness;
	float tipThickness;
	float density;
	b2Vec2 tipCenter; 				// these are used so the skeleton master can remember his place as he traverses the heirarchy of souls.
	b2Vec2 rootCenter; 
	JointUserData * joint; 		// the joint that attaches it into its socket		
	uint8_t attachedTo;	
	bool isRoot ;
	bool isMouth ;
	bool isSensor ;
	// uint8_t sensorType; 
	float sensation;
	bool isWeapon ;					// weapons destroy joints to snip off a limb for consumption. optionally, they can produce a physical effect such as an explosion.
	float energy; 					// the nutritive energy stored in the tissue of this limb; used by predators and scavengers

	b2BodyDef bodyDef;
	b2Body * p_body;
	b2PolygonShape shape;
	BonyFish * p_owner;
	b2Fixture * p_fixture;

	b2Vec2 position;
	bool init;		// if the struct has been created properly
	bool isUsed;	// if the bone is actually used in the game, presumably not all animals will use all 8

	BoneUserData(boneAndJointDescriptor_t boneDescription,
		BonyFish * fish,
		b2World * m_world, b2ParticleSystem * m_particleSystem,
		b2Vec2 positionOffset);
} ;

struct connectionDescriptor {
	bool isUsed;
	unsigned int connectedTo;
	float connectionWeight;	

	connectionDescriptor();
};

struct neuronDescriptor {
	bool isUsed;
	unsigned int n_inputs;
	unsigned int activation_function;
	float activation_steepness;

	unsigned int n_connections;
	connectionDescriptor connections[16];

	neuronDescriptor();
};

struct layerDescriptor {
	bool isUsed;
	unsigned int n_neurons;
	neuronDescriptor neurons[16]; // an array is a pointer to the start of the array, and this is actually an array of pointers to objects, so neurons[] is a double pointer.

	layerDescriptor();
};

struct networkDescriptor {
	/*
	1. it's impossible to modify a FANN network once it is created
	2. modifying the network stored in text is possible, but hard
	3. i might as well make an easily modifiable descriptor file, and methods to turn it back into the text file.
	*/

	unsigned int n_layers;
	layerDescriptor layers[8];

	networkDescriptor( );
};


void LoadFishFromName (uint8_t fishIndex, b2World * m_world, b2ParticleSystem * m_particleSystem) ;

struct BonyFish {
	float hunger; 	// the animal spends energy to move and must replenish it by eating
	bool init; 		// true after the particle has been initialized. In most cases, uninitalized particles will be ignored.
	bool isUsed;	// 

	BoneUserData * bones[N_FINGERS]; // for now, let's just gnetworkDet fish working with a small, hard-linked, flat level set of bones.
	uint8_t n_bones_used;

	uint8_t heartCountA; 	// the heart is a neuro input used for timing and frequency control. 
	uint8_t heartSpeed; 	//  
	float heartOutputA;	// every heartSpeed timesteps, the output changes state between 1 and 0.

	uint8_t heartCountB; 	// the heart is a neuro input used for timing and frequency control. 
	// uint8_t heartSpeedB; 	//  
	float heartOutputB;	// every heartSpeed timesteps, the output changes state between 1 and 0.

	uint8_t heartCountC; 	// the heart is a neuro input used for timing and frequency control. 
	// uint8_t heartSpeedC; 	//  
	float heartOutputC;	// every heartSpeed timesteps, the output changes state between 1 and 0.

	uint8_t heartCountD; 	// the heart is a neuro input used for timing and frequency control. 
	// uint8_t heartSpeedD; 	//  
	float heartOutputD;	// every heartSpeed timesteps, the output changes state between 1 and 0.


	bool flagDelete; // flag this whole animal for deletion at the next convenient time.
	// bool flagWinner; // flag the animal as winner for this turn.

	struct fann *ann;

	uint8_t slot; // for reference, the slot the fish is loaded into

	fishDescriptor_t genes; // the fish carries a copy of its own descriptor which is the genetic infomshun it will pass along.
	
	BonyFish(fishDescriptor_t driedFish, uint8_t fishIndex, b2World * m_world, b2ParticleSystem * m_particleSystem, fann * nann, b2Vec2 startingPosition);

};

struct foodParticle_t {
	b2Vec2 position; 			// starting position of the food in the game world
	float energy; 				// the nutritive value of the food

	b2BodyDef bodyDef;
	b2Body * p_body;
	b2PolygonShape shape; 

	bool init; 					// true after the particle has been initialized. In most cases, uninitalized particles will be ignored.
	bool isUsed;

	foodParticle_t(b2Vec2 position, b2World * m_world, b2ParticleSystem * m_particleSystem);
};

#define TYPE_DEFAULT 0
#define TYPE_MOUTH 1
#define TYPE_FOOD 2
#define TYPE_DANGER 3

struct uDataWrap {
	void * uData;
	uint8_t dataType;

	uDataWrap(void * dat, uint8_t type);
};

void deepSeaControlA () ;
void deepSeaControlB () ;

void addFoodParticle ( b2Vec2 position, b2World * m_world, b2ParticleSystem * m_particleSystem) ;
void fishIncorporator (BonyFish * p_fish,  b2World * m_world, b2ParticleSystem * m_particleSystem) ;

void deepSeaSetup(b2World * m_world, b2ParticleSystem * m_particleSystem, DebugDraw * p_debugDraw) ;
void deepSeaLoop () ;

void makeAJellyfish (BonyFish * p_fish, b2World * m_world, b2ParticleSystem * m_particleSystem) ;

extern foodParticle_t * food[N_FOODPARTICLES];
extern BonyFish * fishes[N_FISHES];

extern int currentNumberOfFood;
extern int currentNumberOfFish;

void collisionHandler (void * boneA, void * boneB) ;

void vote(BonyFish * winner);

void  mutateFANNFileDirectly();
#endif