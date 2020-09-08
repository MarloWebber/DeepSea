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
	// this will comprise the organs of the rigid animals.
	// it is like a linked list that connects to other bones.

	float length;
	float rootThickness;
	float tipThickness;

	float density;

	b2Vec2 tipCenter; // these are used so the skeleton master can remember his place as he traverses the heirarchy of souls.
	b2Vec2 rootCenter; 

	jointUserData_t * joint; // the joint that attaches it into its socket		
	boneUserData_t * attachedTo;	

	// the non recursive data model does not have this.
	// boneUserData_t* bones[N_FINGERS]; // a collection of the other bones that are attached to it. the root bone is 0
	// int n_bones;			  // number of child bones that are actually used.

	bool isRoot ;
	bool isMouth ;

	bool isSensor ;
	float sensation;

	bool isWeapon ;	// weapons destroy joints to snip off a limb for consumption. optionally, they can produce a physical effect.

	float energy; // the nutritive energy stored in the tissue of this limb; used by predators and scavengers

	b2Body * body;
	b2PolygonShape * shape; 

	b2Vec2 position;

	bool init;		// if the struct has been created properly
	bool isUsed;	// if the bone is actually used in the game, presumably not all animals will use all 8


	boneUserData_t();


} ;

// https://stackoverflow.com/questions/2259476/rotating-a-point-about-another-point-2d
b2Vec2 rotatePoint(float cx,float cy,float angle, b2Vec2 p);

// void recursiveBoneIncorporator(boneUserData_t * p_bone, b2Vec2 cumulativeBonePosition, b2World * m_world, b2ParticleSystem * m_particleSystem,boneUserData_t * previousBone) ;

// void recursiveSensorUpdater (boneUserData_t * p_bone) ;




void fishBrainCreator ();




class BonyFish
{
	
public:
	boneUserData_t bones[N_FINGERS]; // for now, let's just get fish working with a small, hard-linked, flat level set of bones.
	// jointUserData_t joints[N_FINGERS]; 

	// fishBrain_t * brain;

	float hunger; // the animal spends energy to move and must replenish it by eating
	b2Vec2 position; // the starting position of the fish in the game world
	bool init; // true after the particle has been initialized. In most cases, uninitalized particles will be ignored.
	bool isUsed;


	BonyFish();
};



// struct BonyFish {
// 	// these are the animals made from rigid physical objects in the game world.
// 	// they are comprised of skeletons that brachiate from an origin point. each bone ends in none, one, or several others. bones can be jointed and can move to apply force.
// 	// these have the potential to be simple to implement.

// 	// boneUserData_t * bone;
// 	boneUserData_t bones[N_FINGERS]; // for now, let's just get fish working with a small, hard-linked, flat level set of bones.
// 	// jointUserData_t joints[N_FINGERS]; 

// 	// fishBrain_t * brain;

// 	float hunger; // the animal spends energy to move and must replenish it by eating
// 	b2Vec2 position; // the starting position of the fish in the game world
// 	bool init; // true after the particle has been initialized. In most cases, uninitalized particles will be ignored.

// 	BonyFish () { // this is supposed to be a constructor that populates all of the objects so none of them are uninitizlied.

// 	}

// 	// incorporate () {  // add the fih into the game world

// 	// }

// 	// decorporate () { // remove the fih from the game worl

// 	// }
// };


struct foodParticle_t {
	b2Vec2 position; // starting position of the food in the game world
	float energy; // the nutritive value of the food

	b2Body * body;
	b2CircleShape * shape; 

	bool init; // true after the particle has been initialized. In most cases, uninitalized particles will be ignored.
	bool isUsed;
};



void addFoodParticle ( b2Vec2 position, b2World * m_world, b2ParticleSystem * m_particleSystem) ;
void fishIncorporator (BonyFish * p_fish,  b2World * m_world, b2ParticleSystem * m_particleSystem) ;

// typedef struct SquishyFish {
// 	// this is the type of animal made of particle groups.
// 	// liquidfun already provides a range of softbody materials that is enough to construct simple animal bodies, with stiff parts, flexible and elastic parts.
// 	// these have the potential to be cuter and more hilarious than the very serious rigid-body animals.
// 	// particle groups can be made into finer and more detailed shapes than the rigid bones.
// 	// to create a muscle, we must provide force onto the tissue using liquidfun's force tool. we have to figure that out ourselves.
// }

void deepSeaSetup(b2World * m_world, b2ParticleSystem * m_particleSystem) ;
void deepSeaLoop () ;


void makeAJellyfish (BonyFish * p_fish, b2World * m_world, b2ParticleSystem * m_particleSystem) ;


extern foodParticle_t food[N_FOODPARTICLES];
extern BonyFish * fishes[N_FISHES];

extern int currentNumberOfFood;
extern int currentNumberOfFish;



#endif