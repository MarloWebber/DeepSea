#ifndef DEEPSEA_H
#define DEEPSEA_H

#include "Test.h"
#include "Main.h"

#define N_FINGERS 8 // the amount of bones a fish can have. you know, fish fingers.
#define N_FISHES 8
#define N_FOODPARTICLES 8

struct jointUserData_t {
	float torque; 	
	float speed; 	
	float speedLimit;

	bool driveCW;	// a signal that tells the motor to turn in one direction. This is much simpler than trying to drive it with a number and having to remember positon etc. With this, you just hit the button, or don't.
	bool driveCCW;	// a signal that tells the motor to turn in the other direction.

	float upperAngle;
	float normalAngle;
	float lowerAngle;

	b2RevoluteJoint * joint; // the joint that this user data struct gets pinned to

	bool init;
	bool isUsed;
} ;


struct boneUserData_t {

	float length;
	float rootThickness;
	float tipThickness;
	float density;

	b2Vec2 tipCenter; 				// these are used so the skeleton master can remember his place as he traverses the heirarchy of souls.
	b2Vec2 rootCenter; 

	jointUserData_t * joint; 		// the joint that attaches it into its socket		
	boneUserData_t * attachedTo;	

	bool isRoot ;
	bool isMouth ;

	bool isSensor ;
	float sensation;

	bool isWeapon ;					// weapons destroy joints to snip off a limb for consumption. optionally, they can produce a physical effect.

	float energy; 					// the nutritive energy stored in the tissue of this limb; used by predators and scavengers

	b2Body * body;
	b2PolygonShape * shape; 

	b2Vec2 position;

	bool init;		// if the struct has been created properly
	bool isUsed;	// if the bone is actually used in the game, presumably not all animals will use all 8

	boneUserData_t();
} ;

b2Vec2 rotatePoint(float cx,float cy,float angle, b2Vec2 p);

void fishBrainCreator ();

class BonyFish
{
	
public:
	boneUserData_t bones[N_FINGERS]; // for now, let's just get fish working with a small, hard-linked, flat level set of bones.

	float hunger; // the animal spends energy to move and must replenish it by eating
	b2Vec2 position; // the starting position of the fish in the game world
	bool init; // true after the particle has been initialized. In most cases, uninitalized particles will be ignored.
	bool isUsed;

	BonyFish();
};

struct foodParticle_t {
	b2Vec2 position; 			// starting position of the food in the game world
	float energy; 				// the nutritive value of the food

	b2Body * body;
	b2CircleShape * shape; 

	bool init; 					// true after the particle has been initialized. In most cases, uninitalized particles will be ignored.
	bool isUsed;
};



void addFoodParticle ( b2Vec2 position, b2World * m_world, b2ParticleSystem * m_particleSystem) ;
void fishIncorporator (BonyFish * p_fish,  b2World * m_world, b2ParticleSystem * m_particleSystem) ;

void deepSeaSetup(b2World * m_world, b2ParticleSystem * m_particleSystem) ;
void deepSeaLoop () ;

void makeAJellyfish (BonyFish * p_fish, b2World * m_world, b2ParticleSystem * m_particleSystem) ;

extern foodParticle_t food[N_FOODPARTICLES];
extern BonyFish * fishes[N_FISHES];

extern int currentNumberOfFood;
extern int currentNumberOfFish;

#endif