#include "DeepSea.h"
#include "fann.h"
#include "Test.h"
#include "Main.h"

#include <iostream>
#include <fstream>
#include <random>
#include <string>
#include <limits>
#include <stdio.h>
#include <chrono>
#include <thread>

#include <math.h>
#include <cmath>

// #include <b2_draw.h>

float pi = 3.14159f;

// global settings to keep track of the game parameters.
int currentNumberOfFish = 0;
int generationsThisGame = 0;
bool startNextGeneration = false;
deepSeaSettings m_deepSeaSettings = {
0,	// int gameMode;
0,	//  	int exploratory_nFood;
64,	//  	int exploratory_nFish;
b2Vec2(0.0f,0.0f),	//  	b2Vec2 gravity;
0.1,	//  	float mutationRate;
0.2,	//  	float mutationSeverity;
0.1,	//  	float mentalMutationRate;
0.5,	//  	float mentalMutationSeverity;
0	//  	int terrainPaintType;
};
uint64 loopCounter = 0 ;
uint32 loopSafetyLimit = 100;
bool flagAddFood = false;
bool flagAddPlant= false;

bool userControlInputA;
bool userControlInputB;

int currentlySelectedLimb =0;

// collections of objects in the game world.
std::list<BonyFish> fishes;
std::list<Lamp> lamps;
BoneUserData * food[N_FOODPARTICLES];

// local copies of pointers mean that the references to these objects only need to be given once at setup (they come from main).
b2World * local_m_world = nullptr;
b2ParticleSystem * local_m_particleSystem = nullptr;
DebugDraw * local_debugDraw_pointer = nullptr;

// these are to do with particle drawing.
bool m_drawing;
b2ParticleGroup* m_lastGroup;
uint32 m_particleFlags;
uint32 m_groupFlags;
uint32 m_colorIndex;

// these FANN parameters should be common to all networks.
const float desired_error = (const float) 0.01;
const unsigned int max_epochs = 50000;
const unsigned int epochs_between_reports = 100;

void unused_variable(void * bullshit) {
	; // do nothing
}

float magnitude (b2Vec2 vector) {
	return sqrt( (vector.x * vector.x) + (vector.y * vector.y));
}

// https://stackoverflow.com/questions/2259476/rotating-a-point-about-another-point-2d
b2Vec2 rotatePoint(float cx,float cy,float angle, b2Vec2 p) {
	float s = sin(angle);
	float c = cos(angle);

	// translate point back to origin:
	p.x -= cx;
	p.y -= cy;

	// rotate point
	float xnew = p.x * c - p.y * s;
	float ynew = p.x * s + p.y * c;

	// translate point back:
	p.x = xnew + cx;
	p.y = ynew + cy;
	return b2Vec2(p.x,p.y);
};

// modifies the array you point it at
void rotatePolygon(b2Vec2 rotationCenter, b2Vec2 * vertices, int count, float angle) {
	for (int i = 0; i < count; ++i)
	{
		vertices[i] = rotatePoint(rotationCenter.x, rotationCenter.y, angle, vertices[i]);
	}
}

float RNG() { //
    static std::default_random_engine e;
    static std::uniform_real_distribution<> dis(0, 1); // rage 0 - 1
    return dis(e);
}

uint32 DetermineParticleParameter() {
	if (m_drawing) {
		if (m_groupFlags == (b2_rigidParticleGroup |
							 b2_solidParticleGroup))
		{
			return (uint32)e_parameterRigid;
		}
		if (m_groupFlags == b2_rigidParticleGroup &&
			m_particleFlags == b2_barrierParticle)
		{
			return (uint32)e_parameterRigidBarrier;
		}
		if (m_particleFlags == (b2_elasticParticle | b2_barrierParticle))
		{
			return (uint32)e_parameterElasticBarrier;
		}
		if (m_particleFlags == (b2_springParticle | b2_barrierParticle))
		{
			return (uint32)e_parameterSpringBarrier;
		}
		if (m_particleFlags == (b2_wallParticle | b2_repulsiveParticle))
		{
			return (uint32)e_parameterRepulsive;
		}
		return m_particleFlags;
	}
	return (uint32)e_parameterMove;
}

void setUserControlInputA() {
	userControlInputA = true;
}

void setUserControlInputB () {
	userControlInputB = true;
}

JointUserData::JointUserData(boneAndJointDescriptor_t boneDescription, BoneUserData * p_bone, BonyFish * fish) {
	torque = boneDescription.torque ;
	speedLimit = boneDescription.speedLimit;
	upperAngle = boneDescription.upperAngle;
	normalAngle = boneDescription.normalAngle;
	lowerAngle = boneDescription.lowerAngle;
	isUsed = false;

	// the following code prepares the box2d objects.
	if (boneDescription.isRoot) { // used for food particles which aren't jointed to anything.
		;
	}
	else {
		jointDef.bodyA = fish->bones[boneDescription.attachedTo]->p_body;
		jointDef.bodyB = p_bone->p_body;
		jointDef.localAnchorA =  b2Vec2(0.0f, fish->bones[boneDescription.attachedTo]->length/2);
		jointDef.localAnchorB =  b2Vec2(0.0f, -p_bone->length/2);
		jointDef.enableLimit = true;
		jointDef.lowerAngle = lowerAngle;
		jointDef.upperAngle = upperAngle;
		jointDef.enableMotor = true;
	    jointDef.maxMotorTorque = torque;
	}
    jointDef.userData = this;
    init = true;
}

boneAndJointDescriptor_t::boneAndJointDescriptor_t () {
	attachedTo = 0; 				// the INDEX (out of N_FINGERS) of the bone it is attached to. Storing data in this way instead of a pointer means that mutating it will have hilarious rather than alarming results.
	length = 1.0f;
	rootThickness = 0.2f;
	tipThickness = 0.2f;
	isRoot = false;
	isMouth = false;
	sensor_radar = true; 			// like an olfactory sensor . senses distance from food
	sensor_touch = false; 			// like how you can feel when things touch your skin.
	sensor_jointangle = true;
	isWeapon  = false;
	torque = 200.0f;				// torque
	speedLimit =		10.0f;		// speedLimit
	upperAngle = pi  * 0.9f;		// upperAngle
	normalAngle = 		0.0f;		// normalAngle
	lowerAngle = 		pi * -0.9f;	// lowerAngle
	used = false;
	color = b2Color(1,1,1);
	outlineColor = b2Color(1,1,1);
}

uDataWrap::uDataWrap(void * dat, uint8_t typ) {
		uData = dat;
		dataType = typ;
}

void printab2Vec2(b2Vec2 v) {
	printf("x%f y%f\n", v.x, v.y);
}

b2Vec2 getRandomPosition() {
		   return  b2Vec2(  (RNG()-0.5) * 100, (RNG()-0.5) * 100  );
}

BoneUserData::BoneUserData(
		boneAndJointDescriptor_t boneDescription,
		BonyFish * fish,
		b2Vec2 positionOffset,
		int newCollisionGroup,
		bool attached
	) {

	p_owner = fish;

	// initialize everything to default, sane values
	length = boneDescription.length;
	rootThickness = boneDescription.rootThickness;
	tipThickness = boneDescription.tipThickness;
	density = 1.2f; 														// the original density of the water is 1.2f
	isRoot = boneDescription.isRoot;
	isMouth = boneDescription.isMouth;
	isLeaf = boneDescription.isLeaf;

	sensor_touch = boneDescription.sensor_touch;
	sensor_radar = boneDescription.sensor_radar;
	sensor_jointangle = boneDescription.sensor_jointangle;
	sensation_radar = 0.0f;
	sensation_touch = 0.0f;
	sensation_jointangle = 0.0f;

	flagPhotosynth = false;

	isFood = false;

	isWeapon  = boneDescription.isWeapon;									// weapons destroy joints to snip off a limb for consumption. optionally, they can produce a physical effect.
	energy = ((rootThickness + tipThickness)/2) * length * density; 		// the nutritive energy stored in the tissue of this limb; used by predators and scavengers

	// accessory features cost more so you can't just go ham on them. And the effect stacks so it's better to specialize.
	// touch and jointangle are considered necessary for any moving limb, so they come free.
	if (isMouth				&& !isRoot) 	{ energy = energy * 1.5; };
	if (sensor_radar 		&& !isRoot) 	{ energy = energy * 1.5; };
	if (isWeapon 			&& !isRoot) 	{ energy = energy * 1.5; };
	if (isLeaf 				&& !isRoot) 	{ energy = energy * 1.5; };

	energy = energy * 10000; // this is a constant that is used to help the value of energy in food roughly match the effort required to get food.

	color = boneDescription.color;
	outlineColor = boneDescription.outlineColor;

	tipCenter = b2Vec2(0.0f,0.1f); 											// these are used so the skeleton master can remember his place as he traverses the heirarchy of souls.
	rootCenter = b2Vec2(0.0f,0.0f); 	

	int count = 4;

	// bones in a set can't collide with each other.
	collisionGroup = newCollisionGroup;

	if (!attached) {
		isRoot = true;
	}

	// the following code is used to generate box2d structures and shapes from the bone parameters.
	if (isRoot) {

		offsetOnBody = b2Vec2(0.0f, 0.0f);
		tipCenter = b2Vec2(0.0f, length);

		b2Vec2 vertices[] = {
			b2Vec2( + (rootThickness/2),  -(length/2)), //b2Vec2 rootVertexA = 
			b2Vec2(- (rootThickness/2),  -(length/2)), // b2Vec2 rootVertexB =
			b2Vec2( + (tipThickness/2),  +(length/2)), //b2Vec2 tipVertexA = 
			b2Vec2(- (tipThickness/2),  +(length/2)) // b2Vec2 tipVertexB = 
		};
		
		bodyDef.type = b2_dynamicBody;
		p_body = local_m_world->CreateBody(&bodyDef);
		
		shape.Set(vertices, count);

		// reference the physics object from the user data.
		tipCenter = tipCenter;
		rootCenter = b2Vec2(0.0f, 0.0f);
	}
	else {
		b2Vec2 vertices[] = {
			b2Vec2( + (rootThickness/2), -(length/2)),
			b2Vec2( - (rootThickness/2),  -(length/2)),
			b2Vec2( + tipThickness/2, +(length/2)),
			b2Vec2( - tipThickness/2, +(length/2))
		};

		bodyDef.type = b2_dynamicBody;

		// move the body to the appropriate position on the model.
		p_body = local_m_world->CreateBody(&bodyDef);

		tipCenter = tipCenter;
		rootCenter = b2Vec2(0.0f, 0.0f);

		if (attached) {
			BoneUserData * attachesTo = fish->bones[boneDescription.attachedTo];	
			offsetOnBody = (attachesTo->offsetOnBody + (attachesTo->length/2)) + length/2;	
			rootCenter = attachesTo->tipCenter;
		}

		// printf("new bone with position offset x: %f, y%f\n", positionOffset.x, positionOffset.y);
		p_body->SetTransform(b2Vec2(positionOffset.x, positionOffset.y + offsetOnBody.y),0);
		
		shape.Set(vertices, count);

		joint = new JointUserData( boneDescription, this, fish); 	// the joint that attaches it into its socket 
	}	

	init = true;
	isUsed=  false;
};

void nonRecursiveBoneIncorporator(BoneUserData * p_bone) {
	if (!p_bone->init) {
		return;
	}

	p_bone->p_fixture = p_bone->p_body->CreateFixture(&(p_bone->shape), p_bone->density);	// this endows the shape with mass and is what adds it to the physical world.

	if (p_bone->isMouth) {
		uDataWrap * p_dataWrapper = new uDataWrap(p_bone, TYPE_MOUTH);
		p_bone->p_body->SetUserData((void *)p_dataWrapper);
	}
	else if (p_bone->sensor_touch) {
		uDataWrap * p_dataWrapper = new uDataWrap(p_bone, TYPE_TOUCHSENSOR);
		// bodyDef.userData = (void *)p_dataWrapper;
		p_bone->p_body->SetUserData((void *)p_dataWrapper);
	}
	else if (p_bone->isLeaf) {
		uDataWrap * p_dataWrapper = new uDataWrap(p_bone, TYPE_LEAF);
		// bodyDef.userData = (void *)p_dataWrapper;
		p_bone->p_body->SetUserData((void *)p_dataWrapper);
	}
	else if (p_bone->isFood) {
		uDataWrap * p_dataWrapper = new uDataWrap(p_bone, TYPE_FOOD);
		// bodyDef.userData = (void *)p_dataWrapper;
		p_bone->p_body->SetUserData((void *)p_dataWrapper);
	}
	else {
		uDataWrap * p_dataWrapper = new uDataWrap(p_bone, TYPE_DEFAULT);
		// bodyDef.userData = (void *)p_dataWrapper;
		p_bone->p_body->SetUserData((void *)p_dataWrapper);
	}

	b2Filter tempFilter = p_bone->p_fixture->GetFilterData();
	tempFilter.groupIndex = p_bone->collisionGroup;
	p_bone->p_fixture->SetFilterData(tempFilter);

	if (!p_bone->isRoot) {
            p_bone->joint->isUsed = true;

			p_bone->joint->p_joint = (b2RevoluteJoint*)local_m_world->CreateJoint( &(p_bone->joint->jointDef) );
	}
	p_bone->isUsed = true;
}

void nonRecursiveSensorUpdater (BoneUserData * p_bone) {
	if (!p_bone->init || !p_bone->isUsed) {
		return;
	}

	if (!p_bone->isRoot) {
		if (p_bone->joint->isUsed) {
			p_bone->sensation_jointangle= p_bone->joint->p_joint->GetJointAngle();
		}
	}
	
	if (p_bone->sensor_radar) {
		p_bone->sensation_radar = 0.0f;
		for  (int i = 0; i < N_FOODPARTICLES; i++) {

			if (food[i]->init && food[i]->isUsed) {
				b2Vec2 boneCenterWorldPosition = p_bone->p_body->GetWorldCenter();
				b2Vec2 positionalDifference = b2Vec2((boneCenterWorldPosition.x - food[i]->position.x),(boneCenterWorldPosition.y - food[i]->position.y));
				float distance = magnitude (positionalDifference);
				if (distance > 0) {
					p_bone->sensation_radar += 1/distance;
				}
			}
		}


		// this part is heinously computationally inefficient: iterating through the whole global list of bones, once per bone, every step.
		// you can do better, but i don't know how right now
		std::list<BonyFish>::iterator fish;
		for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{

			for (int i = 0; i < N_FINGERS; ++i)
			{
				if ( fish->bones[i]->isLeaf ) {

					if (fish->bones[i]->init && fish->bones[i]->isUsed) {
						b2Vec2 boneCenterWorldPosition = p_bone->p_body->GetWorldCenter();
						b2Vec2 positionalDifference = b2Vec2((boneCenterWorldPosition.x - fish->bones[i]->position.x),(boneCenterWorldPosition.y - fish->bones[i]->position.y));
						float distance = magnitude (positionalDifference);
						if (distance > 0) {
							p_bone->sensation_radar += 1/distance;
						}
					}

				}
			}
		}
	}
}

void addFoodParticle(b2Vec2 position) {
	unsigned int emptyFoodIndex = 0;
	for (int i = 0; i < N_FOODPARTICLES; ++i)
	{
		if (food[i]->isUsed) {
			emptyFoodIndex ++;
		}
		else {
			break;
		}
	}

		boneAndJointDescriptor_t foodDescriptor = *(new boneAndJointDescriptor_t());
		foodDescriptor.used = true;
		foodDescriptor.isRoot = true;
		food[emptyFoodIndex] = new BoneUserData(foodDescriptor, nullptr, position, 0, false);

		//(boneAndJointDescriptor_t boneDescription, BoneUserData * p_bone, BonyFish * fish)
		food[emptyFoodIndex]->joint = new JointUserData(foodDescriptor, food[emptyFoodIndex], nullptr);

		food[emptyFoodIndex]->isFood = true;

		food[emptyFoodIndex]->joint->init = false;
		food[emptyFoodIndex]->joint->isUsed = false;
		nonRecursiveBoneIncorporator(food[emptyFoodIndex]);

		food[emptyFoodIndex]->energy = food[emptyFoodIndex]->energy * 5; // this is a constant that sets the value of food. Typical creatures are made from 4 segments; setting this to 4 or above should allow the creature to reproduce after eating just 1 segment.
}

void addRandomFoodParticle(int arg) {
	addFoodParticle(getRandomPosition());
}

void saveFishToFile(const std::string& file_name, fishDescriptor_t& data) {
  std::ofstream out(file_name.c_str());
  out.write(reinterpret_cast<char*>(&data), sizeof(fishDescriptor_t));
}

void loadFishFromFile(const std::string& file_name, fishDescriptor_t& data) {
  std::ifstream in(file_name.c_str());
  in.read(reinterpret_cast<char*>(&data), sizeof(fishDescriptor_t));
}

// saves the worldlist of particles to file so that it can be retrieved later
void saveParticleStateToFile() {
	//https://google.github.io/liquidfun/API-Ref/html/classb2_particle_system.html
}

void loadParticleStateFromFile() {

}

// makes sure that the starting creature at least has senses wired up to something near the motor controls.
// prevents generations of completely braindead creatures
void wormTrainer () {
	int n_examples = 1000;

	FILE *fp;
    fp = fopen("wormTrainer.data","wb");

    fprintf(fp, "%i %i% i\n", n_examples*4, 28, 8 );

	for (int i = 0; i < 2*n_examples; ++i) {
		float senseDiffPerSegment = RNG() * 0.25;
		float noize = RNG() * 0.5;
	
		// four samples that show a swimming motion when the food is ahead
		// ------- SEQ 1
		fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", 
			4*senseDiffPerSegment + (RNG() * noize),
			3*senseDiffPerSegment + (RNG() * noize),
			2*senseDiffPerSegment + (RNG() * noize),
			1*senseDiffPerSegment + (RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),(RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),1+(RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),(RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),1+(RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),1+(RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),1+(RNG() * noize)

			);
		fprintf(fp, "%f %f %f %f %f %f %f %f\n", 
			0  + (RNG() * noize),
			1 + (RNG() * noize),
			-1 + (RNG() * noize),
			0 + (RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),(RNG() * noize)
			);
		// -------- SEQ 2
		fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", 
			4*senseDiffPerSegment + (RNG() * noize),
			3*senseDiffPerSegment + (RNG() * noize),
			2*senseDiffPerSegment + (RNG() * noize),
			1*senseDiffPerSegment + (RNG() * noize),
			0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),
			0 + (RNG() * noize),0 + (RNG() * noize),1 + (RNG() * noize),0 + (RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),(RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),1+(RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),1+(RNG() * noize),

			(RNG() * noize),(RNG() * noize),(RNG() * noize),1+(RNG() * noize)
			);

		fprintf(fp, "%f %f %f %f %f %f %f %f\n", 
			-1 + (RNG() * noize),
			0 + (RNG() * noize),
			0 + (RNG() * noize),
			1 + (RNG() * noize),
			0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize)
			);
		// -------- SEQ 3
		
		fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",  
			4*senseDiffPerSegment + (RNG() * noize),
			3*senseDiffPerSegment + (RNG() * noize),
			2*senseDiffPerSegment + (RNG() * noize),
			1*senseDiffPerSegment + (RNG() * noize),
			0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),
			0 + (RNG() * noize),1 + (RNG() * noize),0 + (RNG() * noize),1 + (RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),(RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),1+(RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),1+(RNG() * noize),

			(RNG() * noize),(RNG() * noize),(RNG() * noize),1+(RNG() * noize)
			);

		fprintf(fp, "%f %f %f %f %f %f %f %f\n", 
			1 + (RNG() * noize),
			0 + (RNG() * noize),
			0 + (RNG() * noize),
			-1 + (RNG() * noize),
			0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize)
			);
		// -------- SEQ 4
		
		fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",  
			4*senseDiffPerSegment + (RNG() * noize),
			3*senseDiffPerSegment + (RNG() * noize),
			2*senseDiffPerSegment + (RNG() * noize),
			1*senseDiffPerSegment + (RNG() * noize),
			0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),
			0 + (RNG() * noize),1 + (RNG() * noize),1 + (RNG() * noize),0 + (RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),(RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),1+(RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),1+(RNG() * noize),

			(RNG() * noize),(RNG() * noize),(RNG() * noize),1+(RNG() * noize)
			);

		fprintf(fp, "%f %f %f %f %f %f %f %f\n", 
			0 + (RNG() * noize),
			-1 + (RNG() * noize),
			1 + (RNG() * noize),
			0 + (RNG() * noize),
			0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize)
			);
		// --------
	}
}

networkDescriptor * createEmptyNetworkOfCorrectSize (fann * temp_ann) {
	return new networkDescriptor(temp_ann);
}

void BonyFish::feed(float amount) {
	energy += amount;
}

BonyFish::BonyFish(fishDescriptor_t driedFish, fann * nann, b2Vec2 startingPosition) {
	genes = driedFish;
	reproductionEnergyCost = 0.0f; // the amount of energy required to make a clutch of viable offspring. to be calculated

	flagDelete = false;
	selected = false;

	int randomCollisionGroup = - (RNG() * 16.0f);

	if (TestMain::getNoClipStatus()) {
		randomCollisionGroup = -1; // if this option box is checked, the fish never collide with each other.
	}

	for (int i = 0; i < N_FINGERS; ++i) {
		if (i == 0) {
			driedFish.bones[i].isRoot = true;
		}
		bones[i] = new BoneUserData(driedFish.bones[i], this, startingPosition, randomCollisionGroup, true);
		reproductionEnergyCost += bones[i]->energy;
	}

	energy = reproductionEnergyCost * 0.5;

	n_bones_used = 0;
	for (int i = 0; i < N_FINGERS; ++i) {
		if (driedFish.bones[i].used) {
			n_bones_used ++;

		}
	}

	init = false; 										// true after the particle has been initialized. In most cases, uninitalized particles will be ignored.
	isUsed = false; 									// only true when the part is added to the world

	distanceMovedSoFar = 0.0f;
	filteredOutputWiggle = 0.0f;
	previousOutputs = nullptr;

	for (unsigned int i = 0; i < N_FINGERS; ++i) { // if the limb doesn't already have sense and motor connectors, add them in.

		if (driedFish.bones[i].used) {

			bool connected_radar = false;
			bool connected_angle = false;
			bool connected_motor = false;

			if (driedFish.bones[i].sensor_radar) {
				for (unsigned int j = 0; j < N_SENSECONNECTORS; ++j) {
					if (driedFish.inputMatrix[j].connectedToLimb == i ) {
						if (driedFish.inputMatrix[j].sensorType == SENSOR_FOODRADAR) {connected_radar = true;}
						if (driedFish.inputMatrix[j].sensorType == SENSOR_JOINTANGLE) {connected_angle = true;}
					}
					if (driedFish.outputMatrix[j].connectedToLimb == i ) { 
						if (driedFish.outputMatrix[j].sensorType == SENSECONNECTOR_MOTOR) {connected_motor = true;}
					}
				}
			}

			if (!connected_radar) {
				for (unsigned int j = 0; j < N_SENSECONNECTORS; ++j) {
					if (driedFish.inputMatrix[j].sensorType == SENSECONNECTOR_UNUSED ) {
						driedFish.inputMatrix[j].sensorType = SENSOR_FOODRADAR;
						driedFish.inputMatrix[j].connectedToLimb = i;
						break;
					}
				}
			}

			if (!connected_angle) {
				for (unsigned int j = 0; j < N_SENSECONNECTORS; ++j) {
					if (driedFish.inputMatrix[j].sensorType == SENSECONNECTOR_UNUSED ) {
						driedFish.inputMatrix[j].sensorType = SENSOR_JOINTANGLE;
						driedFish.inputMatrix[j].connectedToLimb = i;
						break;
					}
				}
			}

			if (!connected_motor) {
				for (unsigned int j = 0; j < N_SENSECONNECTORS; ++j) {
					if (driedFish.outputMatrix[j].sensorType == SENSECONNECTOR_UNUSED ) {
						driedFish.outputMatrix[j].sensorType = SENSECONNECTOR_MOTOR;
						driedFish.outputMatrix[j].connectedToLimb = i;
						break;
					}
				}
			}
		}
	}

	unsigned int senseInputsUsedSoFar = 0;
	unsigned int motorOutputsUsed = 0;
	for (int i = 0; i < N_SENSECONNECTORS; ++i)
	{
		inputMatrix[i] = driedFish.inputMatrix[i];
		outputMatrix[i] = driedFish.outputMatrix[i];

		if (inputMatrix[i].sensorType != SENSECONNECTOR_UNUSED) {
			senseInputsUsedSoFar ++;
		}

		if (outputMatrix[i].sensorType != SENSECONNECTOR_UNUSED) {
			motorOutputsUsed ++;
		}
	}

    if (nann == NULL) {
    		for (int i = 0; i < N_FINGERS; ++i) {
				driedFish.bones[i].color = b2Color(RNG()* 255, RNG() * 255, RNG() * 255);
			}

			unsigned int innerLayerSmall = (senseInputsUsedSoFar + motorOutputsUsed + motorOutputsUsed)/3;
			unsigned int innerLayerBig = (senseInputsUsedSoFar + senseInputsUsedSoFar + motorOutputsUsed)/3;

			//this is what your basic bob fish will start with.
    	    unsigned int creationLayerCake[] = {
	    	senseInputsUsedSoFar,
	    	innerLayerBig,
	    	innerLayerSmall,
	    	motorOutputsUsed
	    };
	    	ann = fann_create_standard_array(4, creationLayerCake);
		    fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
		    fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);

		    brain = createEmptyNetworkOfCorrectSize (ann) ;
	    }
    else { // a brain is provided
    	ann = nann;

    	brain = createEmptyNetworkOfCorrectSize (nann) ;
    }
};

void moveAWholeFish (BonyFish * fish, b2Vec2 position) {
	for (int i = 0; i < N_FINGERS; ++i)
	{
		if ( !fish->bones[i]->isUsed && !fish->bones[i]->init) {
				continue;
		}
		fish->bones[i]->p_body->SetTransform(position, fish->bones[i]->p_body->GetAngle());
	}
}

void deleteJoint(BoneUserData * bone) {
	if (!bone->isRoot) { // root bones dont have joints
		if (bone->joint != NULL && bone->joint != nullptr) {
			if (bone->joint->isUsed && bone->joint->init) {
				if (bone->joint->p_joint != NULL && bone->joint->p_joint != nullptr) {
					local_m_world->DestroyJoint(bone->joint->p_joint);	
					bone->joint->isUsed = false;
					bone->joint->init = false;
				}
			}
		}
	}
}

void deleteBone (BoneUserData * bone) {
	if (bone->isUsed && bone->init && bone->flagDelete) {		
		local_m_world->DestroyBody(bone->p_body);
		bone->isUsed = false;
		bone->init = false;
	}
}

// delete a fish from the game world and remove it from memory
void deleteFish (BonyFish * fish) {
	if (!fish->isUsed) {
		return;
	}

	if (fish->flagDelete) {
		for (unsigned int i = 0; i < N_FINGERS ; ++i) {
		fish->bones[i]->flagDelete = true;
			deleteJoint(fish->bones[i]);
		}
		for (unsigned int i = 0; i < N_FINGERS ; ++i) {
			deleteBone(fish->bones[i]);
		}
		fish->isUsed = false;
	}
}

// goes through the brain and adds 'biasNeuron' flag to neurons at the end of each layer. This is mainly so they can be drawn properly.
void flagBiasNeurons( BonyFish * fish) {

	std::list<layerDescriptor>::iterator layer;

	unsigned int layerIndex = 0;

	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{

		std::list<neuronDescriptor>::iterator neuron;

		// iterate through all and flag them false
		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
			neuron->biasNeuron = false;
		}

		// on every layer except the last, flag the last neuron as bias.
		if ( layerIndex < (fish->brain->layers.size() -1 )) {

			neuron = layer->neurons.end();
			neuron--;
			neuron->biasNeuron = true;
		}

		layerIndex++;
	}
}


void loadFish (fishDescriptor_t driedFish, fann * nann, b2Vec2 startingPosition) {
	fishes.push_back(  *(new BonyFish(driedFish, nann, startingPosition)) );
	BonyFish * fish = &(fishes.back());
	fish->isUsed = true;

	for (int i = 0; i < N_FINGERS; ++i) {
		if (driedFish.bones[i].used) {
			nonRecursiveBoneIncorporator( fish->bones[i]);
		}
		fish->bones[i]->p_owner = fish; // you need to update the user data pointer, because when you pushed the fish onto the list you pushed a copy of it not the actual thing.
	}

	flagBiasNeurons(fish);
}

fann * loadFishBrainFromFile (std::string fileName) {
	return fann_create_from_file( (fileName + std::string(".net")).c_str() );
}

connectionDescriptor::connectionDescriptor (unsigned int toNeuron) {
	isUsed = false;
	connectedTo = toNeuron;
	connectionWeight = 0.0f;	
}

neuronDescriptor::neuronDescriptor() {
	// n_connections = 0;
	n_inputs = 0;
	isUsed = false;
	aabb.upperBound = b2Vec2(0.0f,0.0f);
	aabb.lowerBound = b2Vec2(0.0f,0.0f);
	selected = false;
}

layerDescriptor::layerDescriptor () {
	// n_neurons = 0;
	isUsed = false;
}

senseConnector::senseConnector () {
	connectedToLimb = 0;							// what limb the sense is coming from, or motor signal is going to.
	connectedToNeuron = 0;							// neuron index. The position of this neuron's layer will determine how the program uses it.
	sensorType =  SENSECONNECTOR_UNUSED; 			// what kind of sense it is (touch, smell, etc.how the number will be treated)
	timerFreq = 0;									// if a timer, the frequency.

}

fishDescriptor_t::fishDescriptor_t () {				//initializes the blank fish as the 4 segment nematode creature.
	for (int i = 0; i < N_SENSECONNECTORS; ++i) {
		senseConnector moshuns = senseConnector();
		outputMatrix[i] = moshuns;
		inputMatrix[i] = moshuns;
	}

	for (int i = 0; i < N_FINGERS; ++i) {
	
		bones[i].color.r = 0.5f;
		bones[i].color.g = 0.5f;
		bones[i].color.b = 0.5f;

		bones[i].length = 0.5f;
		bones[i].rootThickness = 0.2f;
		bones[i].tipThickness = 0.2f;

		if (i == 0) {
			bones[i].isRoot = true;
			bones[i].isMouth = true;
		}
		else {
			bones[i].attachedTo = i-1;
		}

		if (i < 4) {
			bones[i].used = true;
		}
	}

	for (unsigned int i = 0; i < 4; ++i) {
		outputMatrix[i].connectedToLimb = i;
		outputMatrix[i].sensorType = SENSECONNECTOR_MOTOR;
	}
	unsigned int j = 0;
	for (unsigned int i = 0; i < 4; ++i) {
		inputMatrix[j].connectedToLimb = i;
		inputMatrix[j].sensorType = SENSOR_FOODRADAR;
		j++;
		inputMatrix[j].connectedToLimb = i;
		inputMatrix[j].sensorType = SENSOR_JOINTANGLE;
		j++;
	}

	// a wide range of timing options is most helpful to the creature.
	for (unsigned int i = 0; i < 4; ++i) {
		inputMatrix[j].connectedToLimb = 0;
		inputMatrix[j].sensorType = SENSOR_TIMER;
		switch (i){
			case 0:
			inputMatrix[j].timerFreq = 4;
			break;
			case 1:
			inputMatrix[j].timerFreq = 12;
			break;
			case 2:
			inputMatrix[j].timerFreq = 36;
			break;
			case 3:
			inputMatrix[j].timerFreq = 64;
			break;
		}

		inputMatrix[j].timerPhase = RNG();
		j++;
	}
}

fishDescriptor_t * basicPlant() {

	fishDescriptor_t * newPlant = new fishDescriptor_t();

	for (int i = 0; i < N_FINGERS; ++i)
	{
		newPlant->bones[i].used = false;	
	}

	newPlant->bones[0].used = true;
	newPlant->bones[0].isLeaf = true;
	newPlant->bones[0].sensor_touch = false;
	newPlant->bones[0].isMouth = false;
	newPlant->bones[0].color = b2Color(0.1f, 1.0f, 0.3f);

	return newPlant;
}

neuronDescriptor * getNeuronByIndex (networkDescriptor * network, unsigned int windex) {
	std::list<layerDescriptor>::iterator layer;
	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{
		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 			if (neuron -> index == windex) {
 				return &(*neuron);
 			}
		}
	}
	printf("getNeuronByIndex returned null! index: %u\n", windex);
	return nullptr;
}

void deleteNeuronByIndex (networkDescriptor * network, unsigned int windex) {

	// go through the entire brain and destroy any connection mentioning the target.
	// gather the connections to destroy in a separate list because you can't operate on a list while iterating through it.
	std::list<layerDescriptor>::iterator layer;
	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{

		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {

 			std::list<connectionDescriptor>::iterator connection;

 			// note that the increment is removed from the loop and only performed if a deletion is NOT performed. https://stackoverflow.com/questions/16269696/erasing-while-iterating-an-stdlist
 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); ) {

 				if (connection->connectedTo == windex ) {
 					connection = neuron->connections.erase(connection);
 					// neuron->n_connections --;
 				}
 				else {
 					connection++;
 				}
			}
		}
	}

	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{

		// you can delete one neuron while iterating the list so long as you just do one and GTFO.
		bool allDone = false;
		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ;) {
 			if (neuron->index == windex) {
 				neuron = layer->neurons.erase(neuron);
 				// layer->n_neurons--;
 				allDone = true;
 				break;
 			}
 			else {
 				neuron++;
 			}
 		}
 		if (allDone) {
 			break;
 		}
	}

	// go through the entire brain and decrement any neuron index greater than the target's index by 1.
	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{

		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {

 			if (neuron->index > windex) {
 				neuron->index--;
 			}
		}
	}

	// go through the entire brain and decrement any connection index greater than the target's index.
	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{

		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {

 			std::list<connectionDescriptor>::iterator connection;
 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {

 				if (connection->connectedTo > windex) {
 					connection->connectedTo --;
 				}
			}
		}
	}
}

void deleteSelectedNeuron (int arg) {
	std::list<BonyFish>::iterator fish;

	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{

		if (fish->selected ){

			std::list<layerDescriptor>::iterator layer;
			for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{

				std::list<neuronDescriptor>::iterator neuron;
 				for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {

 					if (neuron -> selected) {
 						deleteNeuronByIndex(fish->brain, neuron->index);
 						return;
 					}
 				}
			}
		}
	}
}

// method to create a network descriptor in memory
networkDescriptor::networkDescriptor (fann * pann) {

	// query the number of layers.
	unsigned int num_layers = fann_get_num_layers(pann);
  	unsigned int activation_function_hidden = 5;
  	float activation_steepness_hidden = 0.5f;
  	unsigned int activation_function_output = 0;
  	float activation_steepness_output = 0; 
  	
	// get the layer cake. because FANN provides layer information as an array of integers, this is just a temporary variable to hold it.
	unsigned int layerCake[num_layers];

	fann_get_layer_array(pann, layerCake);

	b2AABB partywaist;
	networkWindow = partywaist;
	partywaist.lowerBound = b2Vec2(0.0f,0.0f);
	partywaist.upperBound = b2Vec2(0.0f,0.0f);

	unsigned int rollingIndexCounter = 0;

	for (unsigned int i = 0; i < num_layers; ++i) {
		layerDescriptor layer = layerDescriptor();

		unsigned int nNeuronsIncludingBias = layerCake[i];
		if (i == num_layers-1) {
			;
		} else {
			nNeuronsIncludingBias +=1;
		}

		for (unsigned int j = 0; j <nNeuronsIncludingBias; ++j) {
			neuronDescriptor neuron = neuronDescriptor();
			

			// if (j == layerCake[i]-1) {
			// 	neuron.biasNeuron = true;
			// }
			// else {
			// 	neuron.biasNeuron = false;
			// }

			neuron.index = rollingIndexCounter;
			rollingIndexCounter ++;

			neuron.activation_function = activation_function_hidden;
  			neuron.activation_steepness = activation_steepness_hidden;
  			// neuron.n_connections = 0; 	// so not used uninitialized
  			neuron.n_inputs = 0; 
  			neuron.isUsed = true;

  			// output neurons have a different function than the others. this applies to all in the last row.
  			if (i == num_layers-1) {
				neuron.activation_function = activation_function_output;
  				neuron.activation_steepness = activation_steepness_output;
  			}

			layer.neurons.push_back(neuron);
		}

		this->layers.push_back(layer); // add a new layer descriptor
	}

	// to create the connection map, you must read in from the file.
  	// figure out the total number of neurons, which is how they are indexed in FANN file.
  	unsigned int sumOfNeurons = 0;
  	for (unsigned int i = 0; i < num_layers; ++i) {
  		sumOfNeurons += layerCake[i];
  	}

  	std::list<layerDescriptor>::iterator layer;
  	unsigned int i = 0;
	unsigned int num_connections = fann_get_total_connections(pann);

  	for (layer = this->layers.begin(); layer != this->layers.end(); ++layer)  {
  		// layer->n_neurons = layerCake[i];
  		layer->isUsed = true;

  		std::list<neuronDescriptor>::iterator neuron;
		for (neuron = layer->neurons.begin(); neuron != layer->neurons.end(); ++neuron) {
  			neuron->activation_function = activation_function_hidden;
  			neuron->activation_steepness = activation_steepness_hidden;
  			// neuron->n_connections = 0; 	// so not used uninitialized
  			neuron->n_inputs = 0; 
  			neuron->isUsed = true;

  			// output neurons have a different function than the others. this applies to all in the last row.
  			if (i == num_layers-1) {
				neuron->activation_function = activation_function_output;
  				neuron->activation_steepness = activation_steepness_output;
  			}
  		}
  	}

	// get connection and weight information.
	struct fann_connection margles[num_connections] ;
  	memset(&margles, 0x00, sizeof(fann_connection[num_connections]));
  	struct fann_connection *con = margles;

  	fann_get_connection_array(pann, con); // this DOES include bias neuron information. 
	
  	for (unsigned int c = 0; c < num_connections; ++c) {
		connectionDescriptor connection = connectionDescriptor(con[c].to_neuron);
		connection.connectionWeight = con[c].weight;
	
		getNeuronByIndex(this, con[c].from_neuron)->connections.push_back(connection);
  			
		getNeuronByIndex(this, con[c].to_neuron)->n_inputs++;
	}				
}

void advanceCursor(FILE * cursor, int charToMoveAhead) {
	int c;
	c = fgetc(cursor);
	unused_variable((void *)&c);
}

void goToLine (FILE * cursor, int linesToMoveAhead) {
	int linesMovedSoFar = 0;
	while(1) {
		int c;
		c = fgetc(cursor);
		if( feof(cursor) ) { 
			break ;
		}
		unused_variable((void *)&c);
		if (c == '\n') {
			linesMovedSoFar ++;
		}
		if (linesMovedSoFar >= linesToMoveAhead) {
			break;
		}
   	}
}

void seekUntil (FILE * cursor, char trigger) {
	while(1) {
		int c;
		c = fgetc(cursor);
		if( c == trigger ) { 
			break ;
		}
	}
}

// i have started converting this function from using arrays to using the list networkdescriptors, but i haven't finished yet.
networkDescriptor  * createNeurodescriptorFromFANN (fann * temp_ann) {

	// build everything in memory and link it together.
	networkDescriptor * newCake = new networkDescriptor(temp_ann);
	return newCake;
}

// from: https://stackoverflow.com/questions/7132957/c-scientific-notation-format-number
void my_print_scientific(char *dest, double value) {
    snprintf(dest, 28, "%.20e", value); // 20 digits between the decimal place and the e
}

fann * createFANNbrainFromDescriptor (networkDescriptor * network) { //create an empty fann brain of the right size and layer cake.
	unsigned int creationLayerCake[(unsigned long)network->layers.size()];	
	std::list<layerDescriptor>::iterator layer;
	unsigned int layerIndex = 0;

	// if you are not on the last layer, ignore bias neurons because they are included implicitly.
	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{
		if (layerIndex == ((unsigned long)network->layers.size() - 1 )) {
			creationLayerCake[layerIndex] = layer->neurons.size();
		}
		else {
			creationLayerCake[layerIndex] = layer->neurons.size() - 1;
		}
		layerIndex++;
	}

	fann * ann = fann_create_standard_array((unsigned long)network->layers.size(), creationLayerCake);
    fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
    fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);

    unsigned int num_connections = fann_get_total_connections(ann);

    struct fann_connection margles[num_connections] ;
  	memset(&margles, 0x00, sizeof(fann_connection[num_connections]));

	layerIndex = 0;
	unsigned int neuronIndex = 0;
	unsigned int connectionIndex = 0;

	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{
		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 			std::list<connectionDescriptor>::iterator connection;
 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {
 				fann_connection conc;
 				conc.from_neuron = neuron->index;
 				conc.to_neuron = connection->connectedTo;
 				conc.weight = connection->connectionWeight;
 				margles[connectionIndex] = conc;
				connectionIndex++;
			}
			neuronIndex ++;
		}
		layerIndex ++;
	}

	fann_set_weight_array(ann, margles, num_connections);
	return ann;
}

// // this method is discontinued in favor of ones that add to the living creature.
// void polydactyly (fishDescriptor_t * driedFish) {

// 	// generate the limb as the next-unoccupied lowest limb index. attach it to a random existing limb
// 	int eventualLimb = 0;
// 	for (unsigned int i = 0; i < N_FINGERS; ++i) {
// 		if (!driedFish->bones[i].used) {
// 			driedFish->bones[i].attachedTo = RNG() * i;
// 			driedFish->bones[i].used = true;
// 			eventualLimb = i;
// 			break;
// 		}
// 	}

// 	// do up the output connector
// 	int j = 0;
// 	while(1){
// 		if (driedFish->outputMatrix[j].sensorType == SENSECONNECTOR_UNUSED) {
// 			driedFish->outputMatrix[j].sensorType = SENSECONNECTOR_MOTOR;
// 			driedFish->outputMatrix[j].connectedToLimb = eventualLimb;
// 			break;
// 		}
// 		j ++;
// 	}
// }




void addNeuronIntoLivingBrain (BonyFish * fish, unsigned int targetLayerIndex) {

	neuronDescriptor * newNeuron = new neuronDescriptor();
	newNeuron->isUsed = true;
	newNeuron->biasNeuron = false;
	newNeuron->index = 0;
	newNeuron->position = b2Vec2(0.0f, 0.0f);

	std::list<layerDescriptor>::iterator layer;
	std::list<layerDescriptor>::iterator targetLayerIterator;
	std::list<neuronDescriptor>::iterator neuron;// = layer->neurons.end();



	unsigned int layerIndex = 0;

	// unsigned int newNeuronIndex = 0;
	printf("---\n");

	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer)  {

		printf("layer of %lu neurons\n", layer->neurons.size());

		// newNeuron.index += layer->n_neurons; // always adding new neuron at the end of the layer.
		newNeuron->index += layer->neurons.size();

		if (layerIndex == targetLayerIndex) {
			targetLayerIterator = layer;	
			break;
		}
		
		layerIndex++;
	}

	// if the neuron is not on the last layer, due to the presence of a bias neuron on the target layer, decrement index by 1.
	if (! (targetLayerIndex == fish->brain->layers.size()-1) ){
		newNeuron->index--;
	}

	 // 	// all indexes in the connection map greater than the index of this neuron are incremented by 1.
	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{
		
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {

 			if (neuron->index >= newNeuron->index) {
 				neuron->index ++;
 			}

 			std::list<connectionDescriptor>::iterator connection;
 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {

 				if (connection->connectedTo >= newNeuron->index) {
 					connection->connectedTo ++;
 				}
 			}
 		}
 	}	


	
 	// make connections for all the next-layer neurons, set to 0, add them to the new neuron
 	if (layer != fish->brain->layers.end() ) { // if this isn't the last layer
 		layer++;	
 		if (layer != fish->brain->layers.end() ) { // and the next layer is not off the end of the array

 			for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 				unsigned int targetIndex = neuron->index;
 				connectionDescriptor * newConnection = new connectionDescriptor(   targetIndex);
 				newConnection->isUsed = true;
 				newConnection->connectionWeight = 0.0f;
 				newNeuron->connections.push_back( *newConnection  );
 			}

 		}
 	}


 	// if the neuron is not on the first layer, all the previous-layer neurons get connections to this neuron.
 	// if (true) {
	 	layer = targetLayerIterator;
	 	if (layer != fish->brain->layers.begin() ) { // and the next layer is not off the end of the array
			layer--;
			for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
				// unsigned int targetIndex = neuron->index;
				connectionDescriptor * newConnection = new connectionDescriptor(   newNeuron->index);
				newConnection->isUsed = true;
 				newConnection->connectionWeight = 0.0f;
				neuron->connections.push_back( *newConnection  );

				// printf("new connection from %u to %u\n", neuron->index, newNeuron.index);
			}

	 	}
 	// }




	
	if (targetLayerIndex == fish->brain->layers.size()-1) {
		targetLayerIterator->neurons.push_back( *newNeuron);		// there is no bias neuron on the last layer so you can drop it right at the end.
	}
	else {
		neuron = targetLayerIterator->neurons.end();			// 'end' is actually 1 past the last element, in C++ list syntax. So retract by 1 to get the last element.
		neuron --; 

		targetLayerIterator->neurons.insert( neuron, *newNeuron); 
	}

	flagBiasNeurons(fish);
}


// add a limb onto the end of the selected one.
void polydactyly2 (BonyFish * fish) {
	uint targetFinger = 0;

	// boneAndJointDescriptor_t boneToDuplicate;
	// bool dupeExistingBone = false;
	boneAndJointDescriptor_t  boneAlone = boneAndJointDescriptor_t();

	for (unsigned int i = 0; i < N_FINGERS; ++i) {
		if (!fish->bones[i]->isUsed) {
			targetFinger = i;
		}
		// else {
		if (fish->bones[i]->attachedTo == currentlySelectedLimb) {
			boneAlone = fish->genes.bones[i];
			// dupeExistingBone = true;
		}
		// }
	}

	



	boneAlone.used = true;
	boneAlone.isLeaf = false;
	boneAlone.sensor_radar = true;
	boneAlone.sensor_jointangle = true;
	boneAlone.attachedTo = currentlySelectedLimb;


	fish->genes.bones[targetFinger] = boneAlone;



	fish->bones[targetFinger] = new BoneUserData(boneAlone, fish, b2Vec2(0.0f, 0.0f), 0, true);

	fish->bones[targetFinger]->joint = new JointUserData(boneAlone, fish->bones[targetFinger], fish);
	fish->bones[targetFinger]->joint->init = true;
	fish->bones[targetFinger]->joint->isUsed = true;

	nonRecursiveBoneIncorporator(fish->bones[targetFinger]);



// 	// add the bone's senses to new input connectors.
// 		// foodradar
// 		int j = 0;
// 		int sense_neurons_to_add = 0;
// 		while(1){
// 			if (fish->outputMatrix[j].sensorType == SENSECONNECTOR_UNUSED) {
// 				fish->outputMatrix[j].sensorType = SENSECONNECTOR_FOODRADAR;
// 				fish->outputMatrix[j].connectedToLimb = targetFinger;
// 				sense_neurons_to_add ++;
// 				break;
// 			}
// 			j ++;
// 		}

// 		// jointangle
// 		j = 0;
// 		while(1){
// 			if (fish->outputMatrix[j].sensorType == SENSECONNECTOR_UNUSED) {
// 				fish->outputMatrix[j].sensorType = SENSECONNECTOR_JOINTANGLE;
// 				fish->outputMatrix[j].connectedToLimb = targetFinger;
// 				sense_neurons_to_add ++;
// 				break;
// 			}
// 			j ++;
// 		}

	if (boneAlone.sensor_radar) {
		for (int i = 0; i < N_SENSECONNECTORS; ++i)
		{
			if (fish->inputMatrix[i].sensorType == SENSECONNECTOR_UNUSED ) {
				fish->inputMatrix[i].sensorType = SENSOR_FOODRADAR;
				fish->inputMatrix[i].connectedToLimb = targetFinger;
				addNeuronIntoLivingBrain (fish, 0) ;
				fish->ann = createFANNbrainFromDescriptor(fish->brain);

				break;
			}
		}
	}

// if (false) {
	if (boneAlone.sensor_jointangle) {
		for (int i = 0; i < N_SENSECONNECTORS; ++i)
		{
			if (fish->inputMatrix[i].sensorType == SENSECONNECTOR_UNUSED ) {
				fish->inputMatrix[i].sensorType = SENSOR_JOINTANGLE ;
				fish->inputMatrix[i].connectedToLimb = targetFinger;
				addNeuronIntoLivingBrain (fish, 0) ;
				fish->ann = createFANNbrainFromDescriptor(fish->brain);

				break;
			}
		}
	}
	


	// add the joint motor senseconnector
	for (int i = 0; i < N_SENSECONNECTORS; ++i)
	{
		if (fish->outputMatrix[i].sensorType == SENSECONNECTOR_UNUSED ) {
			fish->outputMatrix[i].sensorType = SENSECONNECTOR_MOTOR ;
			fish->outputMatrix[i].connectedToLimb = targetFinger;

			// printf("BANANA OMOTO %lu\n", fish->brain->layers.size()-1);

			addNeuronIntoLivingBrain (fish, fish->brain->layers.size()-1 ) ;
			fish->ann = createFANNbrainFromDescriptor(fish->brain);

			break;
		}
	}
// }
	


// std::list<layerDescriptor>::iterator layer;
// 		layer = Mugh->layers.begin();
// 	newNeuronIndex = layer.size()-2;





 

// 	// add the bone's motor control to a new output connector.
// 	int j = 0;
// 	while(1){
// 		if (driedFish->outputMatrix[j].sensorType == SENSECONNECTOR_UNUSED) {
// 			driedFish->outputMatrix[j].sensorType = SENSECONNECTOR_MOTOR;
// 			driedFish->outputMatrix[j].connectedToLimb = eventualLimb;
// 			break;
// 		}
// 		j ++;
// 	}


// 	// rearrange the network to have the appropriate number of new n layer neurons.
	


	
	return;





}

void placeLimbOnSelectedFish(int arg) {

	unused_variable((void *) &arg);
	std::list<BonyFish>::iterator fish;
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{

		if (fish->selected && TestMain::getBodyWindowStatus()) {
			polydactyly2( &(*fish));
			break;
		}
	}
}

void verifyNetworkDescriptor (networkDescriptor * network) {
	// printf(" printConnectionArrayForDebug: %u layers\n", network->n_layers);

	std::list<layerDescriptor>::iterator layer;
	unsigned int layerIndex = 0;
	unsigned int neuronIndex = 0;
	unsigned int connectionIndex = 0;

	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{
		printf("	layer %u neurons: %lu\n", layerIndex, (unsigned long)layer->neurons.size());

		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 			printf("		neuron %u connections: %lu inputs: %u bias: %i\n", neuronIndex, (unsigned long)neuron->connections.size(), neuron->n_inputs, neuron->biasNeuron);

 			std::list<connectionDescriptor>::iterator connection;
 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {
 				printf("			connection %u to: %u, weight:%f\n", connectionIndex, connection->connectedTo, connection->connectionWeight );

				connectionIndex++;
			}
			neuronIndex ++;
		}
		layerIndex ++;
	}
}

void amputation (int arg) {
	unused_variable((void *) &arg);
	std::list<BonyFish>::iterator fish;

	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{

		if (fish->selected && TestMain::getBodyWindowStatus()) {

			if (fish->bones[currentlySelectedLimb]->isRoot) { // you can't amputate someone's head... well you can... but..
					return;
				}

			for ( int i = 0; i < N_FINGERS; ++i) {
				if (fish->bones[i]->attachedTo == currentlySelectedLimb) {
					fish->bones[i]->flagDelete = true;
					deleteJoint(fish->bones[i]);
					deleteBone(fish->bones[i]);
					fish->genes.bones[i].used = false;
				}
			}

		 	fish->bones[currentlySelectedLimb]->flagDelete = true;
			deleteJoint(fish->bones[currentlySelectedLimb]);
			deleteBone(fish->bones[currentlySelectedLimb]);
			fish->genes.bones[currentlySelectedLimb].used = false;

			// list of neuron indexes to remove
			std::list<unsigned int> neuronsToRemove;    

			unsigned int retractInputConnectorsThisManyPlaces = 0;
			unsigned int retractInputConnectorsStartingAt = 0;
			bool chosenPlaceForInputConnectorRetraction = false;

			unsigned int retractOutputConnectorsThisManyPlaces = 0;
			unsigned int retractOutputConnectorsStartingAt = 0;
			bool chosenPlaceForOutputConnectorRetraction = false;

			// delete output and input connectors, noting which neurons will be removed.
			uint8_t currentlySelectedLimbUnsigned = currentlySelectedLimb;
			for (int i = 0; i < N_SENSECONNECTORS; ++i)
			{
				if (fish->inputMatrix[i].connectedToLimb == currentlySelectedLimbUnsigned) {
					fish->inputMatrix[i].sensorType = SENSECONNECTOR_UNUSED;
					// neuronsToRemove.push_back(fish->inputMatrix[i].connectedToNeuron);

					// turns out the 'connectedToNeuron' parameter isnt even used. So for the input layer can can just use the index
					neuronsToRemove.push_back(i);

					retractInputConnectorsThisManyPlaces++;
					if (!chosenPlaceForInputConnectorRetraction) {
						chosenPlaceForInputConnectorRetraction = true;
						retractInputConnectorsStartingAt = i;
					}
				}

				if (fish->outputMatrix[i].connectedToLimb == currentlySelectedLimbUnsigned) {
					fish->outputMatrix[i].sensorType = SENSECONNECTOR_UNUSED;

					retractOutputConnectorsThisManyPlaces++;
					if (!chosenPlaceForOutputConnectorRetraction) {
						chosenPlaceForOutputConnectorRetraction = true;
						retractOutputConnectorsStartingAt = i;
					}

					std::list<neuronDescriptor>::iterator neuron = fish->brain->layers.back().neurons.begin();
					unsigned int outputLayerStartIndex = neuron->index;
					neuronsToRemove.push_back(outputLayerStartIndex + i);
				}
			}

			// push back the input and output connectors by the number of neurons that were removed.
			for (unsigned int i = 0; i < retractInputConnectorsThisManyPlaces; ++i) {
				for (unsigned int j = retractInputConnectorsStartingAt; j < N_SENSECONNECTORS; ++j)
				{
					if (j == N_SENSECONNECTORS-1) {
						fish->inputMatrix[j].sensorType = SENSECONNECTOR_UNUSED;
					}
					else {
						fish->inputMatrix[j] = fish->inputMatrix[j+1];
					}
				}
			}

			for (unsigned int i = 0; i < retractOutputConnectorsThisManyPlaces; ++i) {
				for (unsigned int j = retractOutputConnectorsStartingAt; j < N_SENSECONNECTORS; ++j)
				{
					if (j == N_SENSECONNECTORS-1) {
						fish->outputMatrix[j].sensorType = SENSECONNECTOR_UNUSED;
					}
					else {
						fish->outputMatrix[j] = fish->outputMatrix[j+1];
					}
				}
			}

			// re-fry the brain without those connectors.

			std::list<unsigned int>::iterator neuronsToRemoveIterator;

			for (neuronsToRemoveIterator = neuronsToRemove.begin(); neuronsToRemoveIterator !=  neuronsToRemove.end(); ++neuronsToRemoveIterator) 	{

				deleteNeuronByIndex (fish->brain, *neuronsToRemoveIterator);

				// if the neuron to be removed is in the output layer, shuffle all the output connectors along by one.
				std::list<unsigned int>::iterator decrementIndexesInListIterator;
				for (decrementIndexesInListIterator = neuronsToRemove.begin(); decrementIndexesInListIterator !=  neuronsToRemove.end(); ++decrementIndexesInListIterator) 	{
				  	if (*decrementIndexesInListIterator > *neuronsToRemoveIterator) {
				  		(*decrementIndexesInListIterator)--;
				  	}
				}
			}

			// verifyNetworkDescriptor(fish->brain);

			fish->ann = createFANNbrainFromDescriptor(fish->brain);

			return;
		}
	}
}


void mutateFishDescriptor (fishDescriptor_t * fish, float mutationChance, float mutationSeverity) {
	for (int i = 0; i < N_FINGERS; ++i) {
		if (fish->bones[i].used) {

			// mutate color
			if (RNG() < mutationChance) {
				fish->bones[i].color.Set(fish->bones[i].color.r +( mutationSeverity*(RNG()-0.5 )),  fish->bones[i].color.g, fish->bones[i].color.b);
			}
			if (RNG() < mutationChance) {
				fish->bones[i].color.Set(fish->bones[i].color.r,	 fish->bones[i].color.g + (mutationSeverity*(RNG()-0.5)), fish->bones[i].color.b );
			}
			if (RNG() < mutationChance) {
				fish->bones[i].color.Set(	fish->bones[i].color.r, fish->bones[i].color.g,  fish->bones[i].color.b + (mutationSeverity*(RNG()-0.5)));
			}	

			// mutate floats
			if (RNG() < mutationChance) {	fish->bones[i].length += fish->bones[i].length 				*mutationSeverity*(RNG()-0.5); }
			if (RNG() < mutationChance) {	fish->bones[i].rootThickness += fish->bones[i].rootThickness *mutationSeverity*(RNG()-0.5); }
			if (RNG() < mutationChance) {	fish->bones[i].tipThickness += fish->bones[i].tipThickness 	*mutationSeverity*(RNG()-0.5); }
			if (RNG() < mutationChance) {	fish->bones[i].torque += fish->bones[i].torque 				*mutationSeverity*(RNG()-0.5); }
			if (RNG() < mutationChance) {	fish->bones[i].speedLimit += fish->bones[i].speedLimit 		*mutationSeverity*(RNG()-0.5); }
			if (RNG() < mutationChance) {	fish->bones[i].upperAngle += fish->bones[i].upperAngle 		*mutationSeverity*(RNG()-0.5); }
			if (RNG() < mutationChance) {	fish->bones[i].lowerAngle += fish->bones[i].lowerAngle 		*mutationSeverity*(RNG()-0.5); }

			// if (RNG() < (mutationChance / 50)) {	
			// 	polydactyly(fish);
			// }

			// if (RNG() < mutationChance) {	
			// 	amputation(fish);
			// }

			// mutate attachment points
			// if (RNG() > mutationChance) {	fish->bones[i].attachedTo = (RNG() * fish->n_bones_used ) }

			// mutate bools
			// if (RNG() < mutationChance) {	fish->bones[i].isMouth = !fish->bones[i].isMouth; }
			// if (RNG() < mutationChance) {	fish->bones[i].sensor_touch = !fish->bones[i].sensor_touch; }
		}
	}
}

void jellyfishTrainer () {
	int n_inputs = 8;
	int n_outputs = 8;
	int n_examples = 10000;
	// there are actually 6*n_examples examples.

	float noise = 0.25f;
	float maxSensation = 0.75f;

	float bellAOpen = 0.3f;//0.9f;
	float bellAClose = -0.9f; //0.3f;
	float bellASoftClose = 0.3f;

	float bellBOpen = -0.3f;//0.9f;
	float bellBClose = 0.9f; //0.3f;
	float bellBSoftClose = -0.3f;

	FILE *fp;
    fp = fopen("jellyfishTrainer.data","wb");

    fprintf(fp, "%i %i% i\n", n_examples*6, n_inputs, n_outputs );

	for (int i = 0; i < n_examples; ++i) {

		float noiseThisTurn = ((RNG() - 0.5) * noise);
		float outputNoiseThisTurn = ((RNG() - 0.5) * noise) * 0.5;
		float sensationThisTurn = RNG() * maxSensation;
		float sensationAThisTime = (sensationThisTurn) + ((RNG() - 0.5) * noiseThisTurn);
		float sensationBThisTime =  ((RNG() - 0.5) * noiseThisTurn);

		// for a sense on side A, jiggle the bell on side B
		// one with heartbeat OFF, bell open
		fprintf(fp, "%f %f %f\n", sensationAThisTime,sensationBThisTime,-0.9f + ((RNG() - 0.5) * noiseThisTurn));
		fprintf(fp, "%f %f\n", 
										bellAOpen + ((RNG() - 0.5) * outputNoiseThisTurn),
										bellBOpen +((RNG() - 0.5) * outputNoiseThisTurn)	
										);

		// one with heartbeat ON, bell closed
		fprintf(fp, "%f %f %f\n", sensationAThisTime,sensationBThisTime,0.9f + ((RNG() - 0.5) * noiseThisTurn));
		fprintf(fp, "%f %f\n",			
										bellASoftClose + ((RNG() - 0.5) * outputNoiseThisTurn),
										bellBClose + ((RNG() - 0.5) * outputNoiseThisTurn)
										);

		// for a sense on side B, jiggle the bell on side A
		sensationAThisTime = ((RNG() - 0.5) * noiseThisTurn);
		sensationBThisTime =  (sensationThisTurn) + ((RNG() - 0.5) * noiseThisTurn);
		
		// for a sense on side A, jiggle the bell on side B
		// one with heartbeat OFF, bell open
		fprintf(fp, "%f %f %f\n", sensationAThisTime,sensationBThisTime,-0.9f + ((RNG() - 0.5) * noiseThisTurn));
		fprintf(fp, "%f %f\n", 			
										bellAOpen+((RNG() - 0.5) * outputNoiseThisTurn),
										bellBOpen + ((RNG() - 0.5) * outputNoiseThisTurn));

		// one with heartbeat ON, bell closed
		fprintf(fp, "%f %f %f\n", sensationAThisTime,sensationBThisTime,0.9f + ((RNG() - 0.5) * noiseThisTurn));
		fprintf(fp, "%f %f\n", 
										bellAClose + ((RNG() - 0.5) * outputNoiseThisTurn),
										bellBSoftClose + ((RNG() - 0.5) * outputNoiseThisTurn)
										);

		// for straight ahead, jiggle both?
		sensationAThisTime = (sensationThisTurn) +((RNG() - 0.5) * noiseThisTurn);
		sensationBThisTime =  (sensationThisTurn) + ((RNG() - 0.5) * noiseThisTurn);
		
		// // for a sense on side A, jiggle the bell on side B
		// one with heartbeat OFF, bell open
		fprintf(fp, "%f %f %f\n", sensationAThisTime,sensationBThisTime,-0.9f + ((RNG() - 0.5) * noiseThisTurn));
		fprintf(fp, "%f %f\n", 			
										bellAOpen + ((RNG() - 0.5) * outputNoiseThisTurn),
										bellBOpen + ((RNG() - 0.5) * outputNoiseThisTurn));

		// one with heartbeat ON, bell closed
		fprintf(fp, "%f %f %f\n", sensationAThisTime,sensationBThisTime,0.9f + ((RNG() - 0.5) * noiseThisTurn));
		fprintf(fp, "%f %f\n",
										bellAClose + ((RNG() - 0.5) * outputNoiseThisTurn),
										bellBClose + ((RNG() - 0.5) * outputNoiseThisTurn));
	}
	fclose(fp);
}

// this function does the legwork of actually erasing stuff from the world. it is done outside of the step.
void removeDeletableFish() {
	std::list<BonyFish>::iterator fish;

	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		if (fish->flagDelete && fish->isUsed) {
			deleteFish ( &(*fish)) ;
			fishes.erase(fish++);
		}
	}

	for (int i = 0; i < N_FOODPARTICLES; ++i) {
		if (food[i]->flagDelete) {
			deleteBone(food[i]);
		}
	}
}

// this function just sets the flags and labels. it is done inside the step.
void reloadTheSim  (int arg) {
	unused_variable((void *) &arg);
	std::list<BonyFish>::iterator fish;
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		fish->flagDelete = true;
	}
	if (!TestMain::getPersistentFoodStatus()) {
		for (int i = 0; i < N_FOODPARTICLES; ++i) {
			food[i]->flagDelete = true;
		}
	}
	
	startNextGeneration = true;
}

//  prints the winner to file immediately.
void  vote (BonyFish * winner) {
	if ( loopCounter > loopSafetyLimit) {

		fann * wann = winner->ann;

		// save the winner to file with a new name.
		std::string nnfilename =  std::string("mostCurrentWinner.net");
	    std::string fdescfilename =  std::string("mostCurrentWinner.fsh");
	    std::ofstream file { nnfilename };
	    saveFishToFile (fdescfilename, winner->genes);
	    fann_save(  wann , nnfilename.c_str()); 

		reloadTheSim(0);	
	}
}

void mutateFANNFileDirectly (std::string filename) {

	std::ofstream outFile("mutantGimp.net");
	std::string line;

	std::ifstream inFile(filename);
	int count = 0;

	while(getline(inFile, line)){
		if (inFile.eof()){
 			break;
 		} 
	 	if (count == 35) { // its that crazy line.
	 		char desireCharacter = '=';
	 		bool skipTheRest = false;
	 		for(char& c : line) {

	 			// skip over the 27 scientific notation characters you copied.
			    if (!skipTheRest) {

			    	// if you're not skipping or writing down the character, forward the information into the new file.
	 				outFile << c;
		 			
		 			// if you find the ' ', you're ready to copy over the next 27 characters.
		 			if (c == desireCharacter) {
		 				desireCharacter = ' ';
		 				if (c == '=' || *((&c) +1) == '(' )  {
		 					continue;
		 				}

		 				// you need to find out how long the number is.
		 				int sciNumberLength = 0;

		 				while(1) {
		 					if (*((&c) +sciNumberLength) == ')') {
		 						break;
		 					}else {
		 						sciNumberLength++;
		 					}
		 				}

						char sciNumber[sciNumberLength];
						memset(sciNumber, 0x00, sciNumberLength);
						memcpy(sciNumber, &c, sciNumberLength);
					    float val = std::stof(sciNumber); 

					    // float brainMutationChance = 0.2;
					    // float brainMutationAmount = 2;

					    if (RNG() < m_deepSeaSettings.mentalMutationRate) {							// chance of a mutation occurring
					    	if (val > 1 || val < -1) { 				// if it's a big number, apply the mutation as a fraction of htat number. else, apply a random amount in a small range. this is to prevent weights being stuck at very small numbers.
					    		val += ( (RNG() -0.5f) * val * m_deepSeaSettings.mentalMutationSeverity);
					    	}	
					    	else {
					    		val += ( (RNG() -0.5f) * 0.5f * m_deepSeaSettings.mentalMutationSeverity ); 	// how much mutation to apply
					    	}
						    
					    }

				    	char sciNotationBuffer[27];					// = "0.00000000000000000000e+00";
			 			my_print_scientific(sciNotationBuffer,val);
			 			outFile << sciNotationBuffer;
			 			outFile << ") ";

			 			skipTheRest = true;
		 			}
				}
				else  {
					if (c == ' ') {
						skipTheRest = false;
					}
				}
			}
	 	}
	 	else {
	 		outFile << line;
	        outFile << "\n";
	 	}
	    count++;
	}
	outFile.close();
	inFile.close();
}


// you need to find out how long the number is. Some have - sign, some have 2 digits in front of the decimal place.
int getSciNumberLength (char c) {
	int sciNumberLength = 0;

	while(1) {
		char fugnutz = *((&c) +sciNumberLength);
		if (fugnutz == ')') {
			break;
		}else {
			sciNumberLength++;
		}
	}

	return sciNumberLength;
}

float  getSciNumberFromFANNFile (char c) {
	int sciNumberLength = getSciNumberLength(c);

	char sciNumber[sciNumberLength];
	memset(sciNumber, 0x00, sciNumberLength);
	memcpy(sciNumber, &c, sciNumberLength);

	float val = std::stof(sciNumber);
	return val;
}

fishDescriptor_t sexBetweenTwoBodies (fishDescriptor_t partnerA, fishDescriptor_t partnerB) {
	fishDescriptor_t offspring;

	for (int i = 0; i < N_FINGERS; ++i)
	{
		if (RNG() > 0.5) {
			offspring.bones[i] = partnerA.bones[i];
		}
		else {
			offspring.bones[i] = partnerB.bones[i];
		}
	}

	return offspring;
}

// combines two neuro descriptors to produce an offspring with a random mix of traits
void sexBetweenTwoMinds () {
	;
}

void drawingTest() {

	// draw the particle system
	if (false) {
		int32 particleCount = local_m_particleSystem->GetParticleCount();
		if (particleCount)
		{
			float32 radius = local_m_particleSystem->GetRadius();
			const b2Vec2* positionBuffer = local_m_particleSystem->GetPositionBuffer();
				local_debugDraw_pointer->DrawParticles(positionBuffer, radius, NULL, particleCount);
		}
	}
	
	if (true) {
		local_m_world->DrawParticleSystem(*local_m_particleSystem);
	}

	// draw the food particles
	b2Color fishFoodDye 		= b2Color(0.2f, 0.6f, 0.1f);
	b2Color fishFoodDyeOutline 	= b2Color(0.5f, 0.9f, 0.4f);
	b2Color fishFoodDyeOutlineWhite 	= b2Color(1.0f, 1.0f, 1.0f);

	for (unsigned int i = 0; i < N_FOODPARTICLES; ++i) {
		if (food[i]-> init && food[i]->isUsed) {
			b2Vec2 vertices[4];
			b2Vec2 boneCenterWorldPosition = food[i]->p_body->GetWorldCenter();
			for (int j = 0; j < 4; ++j) {	
				b2Vec2 adjustedVertex = food[i]->shape.GetVertex(j);
				b2Vec2 boneLocalCenter =food[i]->p_body->GetLocalCenter();
				b2Vec2 rotatedVertex = rotatePoint( boneLocalCenter.x,boneLocalCenter.y, food[i]->p_body->GetAngle(), adjustedVertex);
				rotatedVertex.x += boneCenterWorldPosition.x;
				rotatedVertex.y +=boneCenterWorldPosition.y;
				vertices[j] = rotatedVertex;
			}
			local_debugDraw_pointer->DrawFlatPolygon(vertices, 4 , fishFoodDye);

			if (food[i]->selected) {
				local_debugDraw_pointer->DrawPolygon(vertices, 4 , fishFoodDyeOutlineWhite);
			}
			else {
				local_debugDraw_pointer->DrawPolygon(vertices, 4 , fishFoodDyeOutline);
			}

			
		}
	}

	// for each bone, get the vertices, then add the body's world location to them, and rotate by the body angle around the body world location.
	// to do this you will need to extend the rotate point method to rotate a polygon.
	std::list<BonyFish>::iterator fish;
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		for (int i = 0; i < N_FINGERS; ++i) {
			if (!fish->bones[i]->init || !fish->bones[i]->isUsed) {
				;
			}
			else {
				if (fish->bones[i]->p_body == NULL || fish->bones[i]->p_body == nullptr) {
					continue;
				}
				b2Vec2 vertices[4];
				b2Vec2 boneCenterWorldPosition = fish->bones[i]->p_body->GetWorldCenter();
				for (int j = 0; j < 4; ++j) {	
					b2Vec2 adjustedVertex = fish->bones[i]->shape.GetVertex(j);
					b2Vec2 boneLocalCenter =fish->bones[i]->p_body->GetLocalCenter();
					b2Vec2 rotatedVertex = rotatePoint( boneLocalCenter.x,boneLocalCenter.y, fish->bones[i]->p_body->GetAngle(), adjustedVertex);
					rotatedVertex.x += boneCenterWorldPosition.x;
					rotatedVertex.y +=boneCenterWorldPosition.y;
					vertices[j] = rotatedVertex;
				}

				local_debugDraw_pointer->DrawFlatPolygon(vertices, 4 , fish->bones[i]->color);

				if (fish->bones[i]->flagPhotosynth) {
					local_debugDraw_pointer->DrawFlatPolygon(vertices, 4 , b2Color(0.9f,0.9f,0.0f));
				}

				if (fish->selected) {
					local_debugDraw_pointer->DrawPolygon(vertices, 4 , b2Color(1,1,1));
				}
				
				else {
					local_debugDraw_pointer->DrawPolygon(vertices, 4 , b2Color(0,0,0));
				}
			}
		}
	}
}



class RayCastClosestCallback : public b2RayCastCallback
{
public:
	RayCastClosestCallback() {
		m_hit = false;
	}

	float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction) {
		b2Body* body = fixture->GetBody();
		void* userData = body->GetUserData();
		uDataWrap* myUserData = (uDataWrap* )body->GetUserData();

		if (myUserData->dataType == TYPE_LEAF && m_deepSeaSettings.gameMode == GAME_MODE_ECOSYSTEM) {
			 ((BoneUserData *)(myUserData->uData))->flagPhotosynth = true;
		}

		if (userData)
		{
			int32 index = *(int32*)userData;
			if (index == 0)
			{
				// By returning -1, we instruct the calling code to ignore this fixture and
				// continue the ray-cast to the next fixture.
				return -1.0f;
			}
		}

		m_hit = true;
		m_point = point;
		m_normal = normal;

		// By returning the current fraction, we instruct the calling code to clip the ray and
		// continue the ray-cast to the next fixture. WARNING: do not assume that fixtures
		// are reported in order. However, by clipping, we can always get the closest fixture.
		// printf("fraction: %f\n", fraction);
		return fraction;
	}
	
	bool m_hit;
	b2Vec2 m_point;
	b2Vec2 m_normal;
};

// a ray of light is cast from the star in a random direction. If it falls on the photosynthetic organ, the organ gains energy.
void shine (Lamp * nancy) {
	b2RayCastInput sunbeam;
	sunbeam.p1 = nancy->position;

	float randomDirection = (RNG() * 2 * pi);

	sunbeam.p2 = b2Vec2(nancy->illuminationRadius * cos(randomDirection),nancy->illuminationRadius* sin(randomDirection));
	sunbeam.maxFraction = 1.0f;

	if (true) { // print the rays of light

	   local_debugDraw_pointer->DrawSegment(sunbeam.p1, sunbeam.p2, b2Color(1.0f, 1.0f, 1.0f) );
	}

	RayCastClosestCallback stupidMotherFucker;

	local_m_world->RayCast( &stupidMotherFucker, sunbeam.p1, sunbeam.p2);
}

void ecosystemModeBeginGeneration (BonyFish * fish) {
	for (int i = 0; i < 3; ++i) {
		fishDescriptor_t newFishBody = fish->genes;

		mutateFishDescriptor (&newFishBody, m_deepSeaSettings.mutationRate, m_deepSeaSettings.mutationSeverity);

		networkDescriptor ickyBrain = *(fish->brain);

		fann * wann = createFANNbrainFromDescriptor(&ickyBrain);

		std::string nnfilename =  std::string("someFishBrain.net");
	    fann_save(  wann , nnfilename.c_str()); 

	    mutateFANNFileDirectly(std::string("someFishBrain.net"));

		// now you can load the mutant ANN.
		fann *jann = loadFishBrainFromFile (std::string("mutantGimp")) ;

		// create a neurodescriptor.
		// networkDescriptor * muscleCars=  createNeurodescriptorFromFANN (mann) ;
		// verifyNetworkDescriptor(muscleCars);
		// fann * jann = createFANNbrainFromDescriptor(muscleCars);

		for (int i = 0; i < N_SENSECONNECTORS; ++i)
		{
			if (newFishBody.inputMatrix[i].sensorType == SENSOR_TIMER) {
					newFishBody.inputMatrix[i].timerPhase = RNG();
			}
		}

		b2Vec2 desiredPosition = fish->bones[0]->p_body->GetWorldCenter();

		loadFish ( newFishBody, jann, desiredPosition) ;

		moveAWholeFish ( &fishes.back(),  desiredPosition);
	}
}

void exploratoryModeBeginGeneration ( ) { // select an animal as an evolutionary winner, passing its genes on to the next generation

	for (int i = 0; i < m_deepSeaSettings.exploratory_nFish; ++i) {

		bool thereIsAFile = false;

		if (FILE *file = fopen("mostCurrentWinner.net", "r")) {
	        fclose(file);
	        if (FILE *file = fopen("mostCurrentWinner.fsh", "r")) {
		        thereIsAFile = true;
		        fclose(file);
		    }
	    } 

		if (thereIsAFile ) { // if there is a previous winner, load many of its mutant children

			fishDescriptor_t newFishBody;
			loadFishFromFile(std::string("mostCurrentWinner.fsh"), newFishBody);

			mutateFishDescriptor (&newFishBody, m_deepSeaSettings.mutationRate, m_deepSeaSettings.mutationSeverity);
		
		    mutateFANNFileDirectly(std::string("mostCurrentWinner.net"));

			// now you can load the mutant ANN.
			fann *mann = loadFishBrainFromFile (std::string("mutantGimp")) ;

			// create a neurodescriptor.
			networkDescriptor * muscleCars=  createNeurodescriptorFromFANN (mann) ;

			// verifyNetworkDescriptor(muscleCars);

			fann * jann = createFANNbrainFromDescriptor(muscleCars);

			for (int i = 0; i < N_SENSECONNECTORS; ++i) {
				if (newFishBody.inputMatrix[i].sensorType == SENSOR_TIMER) {
						newFishBody.inputMatrix[i].timerPhase = RNG();
				}
			}

			b2Vec2 position = getRandomPosition();



			if (TestMain::getOriginStartStatus()) {
				// printf("junpamd a\n");
				position = b2Vec2(0.0f,0.0f);
			}
			else {
				// printf("notuo \n");
			}

			loadFish ( newFishBody, jann, position) ;

		}
		else { 						// if there is no winner, its probably a reset or new install. make one up
			// printf("No genetic material found in game folder. Loading default animal.\n");
			fishDescriptor_t nematode = fishDescriptor_t();

			loadFish ( nematode, NULL,  getRandomPosition()) ;
		}
	}
	startNextGeneration = false;
}

inline bool exists_test1 (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }   
}

Lamp::Lamp() {
		brightness = 100;
		illuminationRadius = 10;
		position = b2Vec2(0.0f, 2.0f);
		illuminationColor = b2Color(0.0f, 0.0f, 0.0f);
}

void deepSeaSetup (b2World * m_world, b2ParticleSystem * m_particleSystem, DebugDraw * p_debugDraw) { //, b2World * m_world_sci, b2ParticleSystem * m_particleSystem_sci) {

	// store a single copy to the pointers so we don't have to give them as arguments 1 million times.
	local_debugDraw_pointer = p_debugDraw;
	local_m_world = m_world;
	local_m_particleSystem = m_particleSystem;

	// initialize foodparticles so they don't crash the program. This step does not add any to the world, and is required even for unused foodparticles.
	for (int i = 0; i < N_FOODPARTICLES; ++i)
	{
		boneAndJointDescriptor_t foodDescriptor = *(new boneAndJointDescriptor_t());
		food[i] = new BoneUserData(foodDescriptor, NULL, b2Vec2(0.0f, 0.0f), 0, false);
		food[i]->init = false;
		food[i]->isUsed = false;
	}

	// from particle drawing
	b2Assert((k_paramDef[0].CalculateValueMask() & e_parameterBegin) == 0);
	TestMain::SetParticleParameters(k_paramDef, k_paramDefCount);
	TestMain::SetRestartOnParticleParameterChange(false);
	m_particleFlags = TestMain::GetParticleParameterValue();
	m_groupFlags = 0;

	// add a lamp
	Lamp monog = Lamp();
	lamps.push_back(monog);
}

// completely nullifies the animals brain
void brainMelter (BonyFish * fish) {
	std::list<layerDescriptor>::iterator layer;
	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{
		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 			std::list<connectionDescriptor>::iterator connection;
 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {
 				connection->connectionWeight = 0.0f;
			}
		}
	}
}

// sets the brain to a random connection map
void brainScrambler (BonyFish * fish) {
	std::list<layerDescriptor>::iterator layer;
	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{
		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 			std::list<connectionDescriptor>::iterator connection;
 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {
 				connection->connectionWeight = (RNG()-0.5f) * 1.0f;
			}
		}
	}
}

void meltSelectedFish (int arg) {
	std::list<BonyFish>::iterator fish;
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		if (fish->selected) {
			brainMelter ( &(*fish));

			// refresh the water in the brain jar.
			fish->ann = createFANNbrainFromDescriptor(fish->brain);
			// return;
		}
		else {
			continue;
		}
	}
}

void scrambleSelectedFish (int arg) {
	std::list<BonyFish>::iterator fish;
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		if (fish->selected) {
			brainScrambler ( &(*fish));

			// refresh the water in the brain jar.
			fish->ann = createFANNbrainFromDescriptor(fish->brain);
			// return;
		}
		else {
			continue;
		}
	}
}

void modifyAConnection (BonyFish * fish, float amount) {
	unsigned int selectedA;
	unsigned int selectedB;

	bool gotA = false;
	bool gotB=  false;

	std::list<layerDescriptor>::iterator layer;
	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{
		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 			if (neuron->selected) {
 				
 				if (gotA) {
 					selectedB = neuron->index;
					gotB = true;
					break;
 				}
 				else {
 					selectedA = neuron->index;
					gotA = true;
				}

			}
			if (gotB) {
				break;
			}
		}
	}

	unsigned int from = 0;
	unsigned int to = 0;

	if (selectedB > selectedA) {
		to = selectedB;
		from = selectedA;
	}
	else {
		to = selectedA;
		from = selectedB;
	}

	std::list<connectionDescriptor>::iterator connection;
	for (connection = getNeuronByIndex( fish->brain, from)->connections.begin(); connection != getNeuronByIndex( fish->brain, from)->connections.end(); connection++) {
		if (connection->connectedTo == to) {
			connection->connectionWeight += amount;
		}
	}	

	// refresh the water in the brain jar.
	fish->ann = createFANNbrainFromDescriptor(fish->brain);
}

void changeSelectedConnection(float amount) {
	std::list<BonyFish>::iterator fish;
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		if (fish->selected) {
			modifyAConnection (&(*fish),amount);
			return;
		}
		else {
			continue;
		}
	}
}

void incrementSelectedConnection() {
	changeSelectedConnection(0.1);
}

void decrementSelectedConnection() {
	changeSelectedConnection(-0.1);
}

// instead of drawing from the FANN struct, this function draws from the neurodescriptor.
void drawNeuralNetworkFromDescriptor (float * motorSignals, float * sensorium, unsigned int * spacesUsedSoFar, BonyFish * fish) {

	unsigned int sizeOfBiggestLayer = 0;

	// sensorium size is based on the size of the ANN. Whether or not it is populated with numbers depends on the size of the input connector matrix.
	unsigned long sizeOfInputLayer = 0;
	unsigned long sizeOfOutputLayer = 0;
	unsigned long num_layers =  (unsigned long)(fish->brain->layers.size());

	std::list<layerDescriptor>::iterator layer;
	layer = fish->brain->layers.begin();
	sizeOfInputLayer = layer->neurons.size();
	std::advance(layer, num_layers-1);
	sizeOfOutputLayer = layer->neurons.size();

	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{
		if ((unsigned long)layer->neurons.size() > sizeOfBiggestLayer) {
			sizeOfBiggestLayer = (unsigned long)layer->neurons.size();
		}
	}

	float fcompatiblespaces = *spacesUsedSoFar;
	b2Vec2 drawingStartingPosition = b2Vec2(  fcompatiblespaces + 1 ,4.0f);
	float spacingDistance = 0.5f;

	b2Vec2 windowVertices[] = {
		b2Vec2(drawingStartingPosition.x -spacingDistance , drawingStartingPosition.y- spacingDistance), 
		b2Vec2(drawingStartingPosition.x - spacingDistance, drawingStartingPosition.y + ((num_layers *spacingDistance ) + ( spacingDistance) ) ), 
		b2Vec2(drawingStartingPosition.x + ((sizeOfBiggestLayer *spacingDistance ) + (spacingDistance) ), drawingStartingPosition.y+ ((num_layers *spacingDistance ) + (spacingDistance) )), 
		b2Vec2(drawingStartingPosition.x + ((sizeOfBiggestLayer *spacingDistance ) + ( spacingDistance) ), drawingStartingPosition.y- spacingDistance)
	};

	fish->brain->networkWindow.lowerBound = b2Vec2(drawingStartingPosition.x -spacingDistance , drawingStartingPosition.y- spacingDistance);
	fish->brain->networkWindow.upperBound = b2Vec2(drawingStartingPosition.x + ((sizeOfBiggestLayer *spacingDistance ) + (spacingDistance) ), drawingStartingPosition.y+ ((num_layers *spacingDistance ) + (spacingDistance) ));

	local_debugDraw_pointer->DrawFlatPolygon(windowVertices, 4 ,b2Color(0.1,0.1,0.1) );

	// work out the neuron positions.
	unsigned int layerIndex = 0;
	unsigned int neuronIndex = 0;
	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{
		neuronIndex = 0;
		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 			neuron->position = b2Vec2(drawingStartingPosition.x + (neuronIndex * spacingDistance), drawingStartingPosition.y + (layerIndex * spacingDistance));
 			neuronIndex ++;
		}
		layerIndex++;
	}

 	// draw in sensorium
	for (unsigned int j = 0; j < sizeOfInputLayer; ++j) {
		b2Vec2 neuron_position = b2Vec2(drawingStartingPosition.x +j * spacingDistance,drawingStartingPosition.y );

		if (sensorium[j] > 0) {
			local_debugDraw_pointer->DrawPoint(neuron_position, 8.0f, b2Color( 0, 0, sensorium[j]));
		}
		else {
			local_debugDraw_pointer->DrawPoint(neuron_position, 8.0f, b2Color( abs(sensorium[j]), 0, 0));
		}

		b2Vec2 gigggle[] = {
			b2Vec2(neuron_position.x+0.1f, neuron_position.y-0.1f - 0.2f), 
			b2Vec2(neuron_position.x+0.1f, neuron_position.y+0.1f - 0.2f), 
			b2Vec2(neuron_position.x-0.1f, neuron_position.y+0.1f - 0.2f), 
			b2Vec2(neuron_position.x-0.1f, neuron_position.y-0.1f - 0.2f), 
		};
// local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.5f,0.1f,0.05f) );
		std::string connectorLabel = std::string("");

		b2Vec2 mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.2);
		switch (fish->inputMatrix[j].sensorType) {

			case SENSECONNECTOR_UNUSED:	
				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.1f,0.1f,0.1f) );
			break;

			case SENSOR_FOODRADAR:	
				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.05f,0.5f,0.1f) );
				connectorLabel =  "Food";

				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.2);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());

				connectorLabel = std::string("Limb ");
				connectorLabel += std::to_string(fish->inputMatrix[j].connectedToLimb);
				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.3);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
			break;

			case SENSOR_JOINTANGLE:	
				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.05f,0.1f,0.5f) );
				connectorLabel =  "Angle";

				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.2);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());

				connectorLabel = std::string("Limb ");
				connectorLabel += std::to_string(fish->inputMatrix[j].connectedToLimb);
				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.3);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
			break;

			case SENSOR_TOUCH:	
				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.5f,0.2f,0.1f) );
				connectorLabel =  "Touch";

				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.2);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());

				connectorLabel = std::string("Limb ");
				connectorLabel += std::to_string(fish->inputMatrix[j].connectedToLimb);
				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.3);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
			break;

			case SENSOR_TIMER:	
				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.5f,0.1f,0.05f) );
				connectorLabel =  "Timer";
				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.2);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
			break;
		}
	}

	for (unsigned int j = 0; j < sizeOfOutputLayer; ++j) {
		b2Vec2 neuron_position = b2Vec2(drawingStartingPosition.x +j * spacingDistance,(drawingStartingPosition.y + ((num_layers-1) * spacingDistance)));

		if (motorSignals[j] > 0) {
			local_debugDraw_pointer->DrawPoint(neuron_position, 8.0f, b2Color( 0.0f, 0.0f, motorSignals[j]));
		}
		else {
			local_debugDraw_pointer->DrawPoint(neuron_position, 8.0f, b2Color( abs(motorSignals[j]), 0.0f, 0.0f));
		}

		b2Vec2 gigggle[] = {
			b2Vec2(neuron_position.x+0.1f, neuron_position.y-0.1f + 0.2f), 
			b2Vec2(neuron_position.x+0.1f, neuron_position.y+0.1f + 0.2f), 
			b2Vec2(neuron_position.x-0.1f, neuron_position.y+0.1f + 0.2f), 
			b2Vec2(neuron_position.x-0.1f, neuron_position.y-0.1f + 0.2f), 
		};


		switch (fish->outputMatrix[j].sensorType) {
			case SENSECONNECTOR_UNUSED:	
				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.1f,0.1f,0.1f) );
			break;

			case SENSECONNECTOR_MOTOR:	
				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.5f,0.3f,0.15f) );

				std::string connectorLabel = std::string("");
				b2Vec2 mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.2);

				connectorLabel = std::string("Limb ");
				connectorLabel += std::to_string(fish->outputMatrix[j].connectedToLimb);
				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y+0.2);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
				connectorLabel =  "Motor";
				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y+0.3);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());

			break;
		}
	}



	// now you know all the positions, you can draw the connections really easily.
	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{
		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 			if (neuron->selected){
 				b2Vec2 gigggle[] = {
					b2Vec2(neuron->position.x+0.05f, neuron->position.y-0.05f), 
					b2Vec2(neuron->position.x+0.05f, neuron->position.y+0.05f), 
					b2Vec2(neuron->position.x-0.05f, neuron->position.y+0.05f), 
					b2Vec2(neuron->position.x-0.05f, neuron->position.y-0.05f), 
				};
				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.3f,0.3f,0.3f) );
 			}


 			if (neuron->biasNeuron) {

 				b2Vec2 gigggle[] = {
					b2Vec2(neuron->position.x+0.1f + 0.2f, neuron->position.y-0.1f), 
					b2Vec2(neuron->position.x+0.1f + 0.2f, neuron->position.y+0.1f), 
					b2Vec2(neuron->position.x-0.1f + 0.2f, neuron->position.y+0.1f), 
					b2Vec2(neuron->position.x-0.1f + 0.2f, neuron->position.y-0.1f), 
				};
 				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.5f,0.5f,0.5f) );

				std::string connectorLabel = std::string("");
				b2Vec2 mocesfef = b2Vec2(neuron->position.x-0.05, neuron->position.y-0.2);

				connectorLabel = std::string("Bias");
				mocesfef = b2Vec2(neuron->position.x-0.05 + 0.2f, neuron->position.y);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());

 			}

 			// printf("Got to here without crashing A!\n");

 			std::list<connectionDescriptor>::iterator connection;
 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {

		    	b2Color segmentColor = b2Color(connection->connectionWeight,connection->connectionWeight,connection->connectionWeight);
		    	// printf("Got to here without crashing B!\n");

		    	if (connection->connectionWeight > 0) {
		    		// segmentColor = b2Color(0.0f,0.0f,connection->connectionWeight,connection->connectionWeight ); // the connection weight is also used as alpha, so if the connection is less than 1, it will be partially transparent (and invisible at 0)
		    		segmentColor.Set(0.0f,0.0f,connection->connectionWeight );
		    	}
		    	else {
		    		// segmentColor = b2Color(abs(connection->connectionWeight),0.0f,0.0f, connection->connectionWeight);
		    		segmentColor.Set(abs(connection->connectionWeight),0.0f,0.0f);
		    	}
		    	// printf("Got to here without crashing C!\n");

		    	// printf("posA x: %f y: %f, posB x: %f, y: %f\n", neuron->position.x, neuron->position.y, (getNeuronByIndex(fish->brain, connection->connectedTo))->position.x, (getNeuronByIndex(fish->brain, connection->connectedTo))->position.y);

		    	// b2Color altSegmentColor = b2Color(1.0f, 1.0f, 1.);

			    local_debugDraw_pointer->DrawSegment(neuron->position, (getNeuronByIndex(fish->brain, connection->connectedTo))->position,segmentColor );  // <--- this one is crashing it
			    // printf("Got to here without crashing D!\n");
			}

			// printf("Got to here without crashing E!\n");

			// write the neuron index on the neuron.
			std::string neuronIndexLabel = std::string("");
			neuronIndexLabel +=  std::to_string(neuron->index);
				b2Vec2 neuronIndexLabelPos = b2Vec2(neuron->position.x, neuron->position.y);
				local_debugDraw_pointer->DrawString(neuronIndexLabelPos, neuronIndexLabel.c_str());


		}
	}
}

// return a positive number if finds a box or negative if not.
BonyFish * checkNeuroWindow (b2AABB mousePointer) {
	std::list<BonyFish>::iterator fish;
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		// printab2Vec2(fish->brain->networkWindow.upperBound);
		if (fish->selected) {
			if (fish->brain->networkWindow.Contains(mousePointer)) {
				return &(*fish);
			}
		}
	}
	return (BonyFish*)nullptr;
}

int checkNeuronsInWindow (b2AABB mousePointer, BonyFish * fish) {
	
	std::list<layerDescriptor>::iterator layer;
	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{

		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {

			if (neuron->position.x < mousePointer.upperBound.x && neuron->position.x > mousePointer.lowerBound.x) {
				if (neuron->position.y < mousePointer.upperBound.y && neuron->position.y > mousePointer.lowerBound.y) {

					neuron->selected = !(neuron->selected);
					return neuron->index;
				}
			}
		}
	}
	return -1;
}



void drawBodyEditingWindow(BonyFish * fish) {

	// float fcompatiblespaces = 1;
	// b2Vec2 drawingStartingPosition = b2Vec2(  fcompatiblespaces + 1 ,4.0f);
	// float spacingDistance = 0.5f;

	b2Vec2 windowVertices[] = {
			b2Vec2(+10.0f, -10.0f), 
			b2Vec2(+10.0f, +10.0f), 
			b2Vec2(-10.0f, +10.0f), 
			b2Vec2(-10.0f, -10.0f)
		};

	// fish->brain->networkWindow.lowerBound = b2Vec2(drawingStartingPosition.x -spacingDistance , drawingStartingPosition.y- spacingDistance);
	// fish->brain->networkWindow.upperBound = b2Vec2(drawingStartingPosition.x + ((sizeOfBiggestLayer *spacingDistance ) + (spacingDistance) ), drawingStartingPosition.y+ ((num_layers *spacingDistance ) + (spacingDistance) ));

	local_debugDraw_pointer->DrawFlatPolygon(windowVertices, 4 ,b2Color(0.1,0.1,0.1) );



	b2Vec2 rootPosition = b2Vec2(0.0f, 0.0f);
	for (int i = 0; i < N_FINGERS; ++i) {
		if (!fish->bones[i]->init || !fish->bones[i]->isUsed) {
			;
		}
		else {
			if (fish->bones[i]->p_body == NULL || fish->bones[i]->p_body == nullptr) {
				continue;
			}

			if (fish->bones[i]->isRoot) {
				rootPosition = fish->bones[i]->p_body->GetWorldCenter();
				break;
			}

		}
	}


	for (int i = 0; i < N_FINGERS; ++i) {
		if (!fish->bones[i]->init || !fish->bones[i]->isUsed) {
			;
		}
		else {
			if (fish->bones[i]->p_body == NULL || fish->bones[i]->p_body == nullptr) {
				continue;
			}
			b2Vec2 vertices[4];
			b2Vec2 boneCenterWorldPosition = fish->bones[i]->p_body->GetWorldCenter();
			for (int j = 0; j < 4; ++j) {	
				b2Vec2 adjustedVertex = fish->bones[i]->shape.GetVertex(j);
				b2Vec2 boneLocalCenter =fish->bones[i]->p_body->GetLocalCenter();
				b2Vec2 rotatedVertex = rotatePoint( boneLocalCenter.x,boneLocalCenter.y, fish->bones[i]->p_body->GetAngle(), adjustedVertex);
				rotatedVertex.x += boneCenterWorldPosition.x;
				rotatedVertex.y +=boneCenterWorldPosition.y;
				vertices[j] = rotatedVertex - rootPosition;
			}

			
			local_debugDraw_pointer->DrawFlatPolygon(vertices, 4 , fish->bones[i]->color);

			if (i == currentlySelectedLimb) {
				local_debugDraw_pointer->DrawPolygon(vertices, 4 , b2Color(0.9,0.4,0.05));
			}
			else {
				local_debugDraw_pointer->DrawPolygon(vertices, 4 , b2Color(0,0,0));
			}
		}
	}
}



bool chooseDMostVertex(float angle, b2Vec2 p1, b2Vec2 p2) {
// given 2 vertices, identify which one lies most in a given direction. return false if it is p1 and true if it is p2.

	// unrotate the set of vertices by the direction, so that the distance to measure is along the Y axis.
	// get the centroid, this is your rotation center
	b2Vec2 thisFaceCentroid = b2Vec2((p1.x + p2.x)/2,(p1.y+p2.y)/2);

	// perform the rotation
	b2Vec2 p1r = rotatePoint(thisFaceCentroid.x, thisFaceCentroid.y, angle, p1);
	b2Vec2 p2r = rotatePoint(thisFaceCentroid.x, thisFaceCentroid.y, angle, p2);

	if (p1r.y > p2r.y) {
		return false;
	}
	else {
		return true;
	}
}


// makes sure the function 'chooseDMostVertex' works properly.
bool test_chooseDMostVertex() {

	int n_failures = 0;

	// A: ( -5, 5), B: (5,-5). Angle 0. B should be the 0-most vertex, returning false
	if (		chooseDMostVertex(0.0f, b2Vec2(-5, 5), b2Vec2(5, -5) )	) { n_failures++; }

	// A: ( -5, 5), B: (5,-5). Angle 0.5 * pi radians. A should be the 0-most vertex, returning true
	if (	!	chooseDMostVertex(0.5f * pi, b2Vec2(-5, 5), b2Vec2(5, -5) )	) { n_failures++; }

	// A: ( -5, 5), B: (5,-5). Angle -0.5 * pi radians. B should be the 0-most vertex, returning false
	if (		chooseDMostVertex(-0.5f* pi, b2Vec2(-5, 5), b2Vec2(5, -5) )	) { n_failures++; }

	// A: ( -5, 5), B: (5,-5). Angle pi radians. A should be the 0-most vertex, returning true
	if (	!	chooseDMostVertex(1.0f * pi, b2Vec2(-5, 5), b2Vec2(5, -5) )	) { n_failures++; }

	// A: ( -5, 5), B: (5,-5). Angle 3* pi radians. A should be the 0-most vertex, returning true
	if (	!	chooseDMostVertex(3.0f * pi, b2Vec2(-5, 5), b2Vec2(5, -5) )	) { n_failures++; }

	// A: ( -5, 5), B: (5,-5). Angle -2* pi radians. B should be the 0-most vertex, returning false
	if (		chooseDMostVertex(-2.0f * pi, b2Vec2(-5, 5), b2Vec2(5, -5) )	) { n_failures++; }



	if (n_failures > 0) {
		printf("chooseDMostVertex failed %u tests\n", n_failures);
		return true;		
	}
	else {
		printf("test_chooseDMostVertex");
		return false;
	}
 
}


// find the distance between the projections of a pair of points. 0 is measured from the face normal.
float findIncidentArea (float angle, b2Vec2 p1, b2Vec2 p2) {
	
	b2Vec2 distanceBetweenPoints = b2Vec2(p1.x - p2.x, p1.y - p2.y);
	float magnitudeArea = magnitude(distanceBetweenPoints);
	float incidentArea = cos(angle) * magnitudeArea;

	return abs(incidentArea);
}


// provides FEA-based lift and drag calculations
void flightModel(BoneUserData * bone) {

	uint nVertices = bone->shape.GetVertexCount();

	for (uint j = 0; j < nVertices; ++j)
	{
		// get the face vertices from the b2 shape
		b2Vec2 p1 = bone->shape.GetVertex(j);
		b2Vec2 p2 = b2Vec2(0.0f, 0.0f);

		// use the pair n and n-1 to make a face. if n is zero, make a face from the first to the last vertices.
		if (j == 0) {
			p2 = bone->shape.GetVertex(nVertices-1);
		}
		else {
			p2 = bone->shape.GetVertex(j-1);
		}
		
		// get the position of the face center point, taking into account the body's present rotation.
		b2Vec2 worldCenter = bone->p_body->GetWorldCenter();
		b2Vec2 p1r = rotatePoint(0.0f, 0.0f, bone->p_body->GetAngle(), p1 );
		p1r += worldCenter;
		b2Vec2 p2r = rotatePoint(0.0f, 0.0f, bone->p_body->GetAngle(), p2 );
		p2r += worldCenter;
		b2Vec2 faceCenter = b2Vec2( (p1r.x + p2r.x)/2, (p1r.y + p2r.y)/2 ) ;
		float faceAngle = atan2(p1r.y - p2r.y, p1r.x - p2r.x);

		// calculate the angle of incidence into the oncoming 'wind'
		b2Vec2 linearVelocity = bone->p_body->GetLinearVelocity();// must get the linear velocity of the face center, not just of the body itself.

		// you can calculate the face center velocity, by taking the radius and multiplying it by the angular velocity.
		b2Vec2 localFaceCenter = b2Vec2( (p1.x + p2.x)/2, (p1.y + p2.y)/2 ) ;
		float magLinearVelocityOfRotatingFace = (bone->p_body->GetAngularVelocity() * magnitude( localFaceCenter));  // https://courses.lumenlearning.com/boundless-physics/chapter/velocity-acceleration-and-force/
		b2Vec2 faceAngularVelocity = b2Vec2( cos(faceAngle) * magLinearVelocityOfRotatingFace, sin(faceAngle) * magLinearVelocityOfRotatingFace);
		b2Vec2 totalVelocity = b2Vec2(linearVelocity.x + faceAngularVelocity.x, (linearVelocity.y + faceAngularVelocity.y ) );
		float magnitudeVelocity = magnitude(totalVelocity);

		float angleOfForwardDirection = atan2(totalVelocity.x, totalVelocity.y * -1) - 0.5 * pi;

		float incidentAngle = atan2(sin(angleOfForwardDirection-faceAngle), cos(angleOfForwardDirection-faceAngle)); // https://stackoverflow.com/questions/1878907/how-can-i-find-the-difference-between-two-angles

		// float incidentAngle = angleOfForwardDirection - faceAngle;

		b2Vec2 distanceBetweenPoints = b2Vec2(p2r.x - p1r.x, p2r.y - p1r.y);
		float magnitudeArea = magnitude(distanceBetweenPoints);
		float incidentArea = findIncidentArea (incidentAngle, p1r, p2r);

		// calculate the force of drag
		float dragCoefficient = 0.5;
		float dragForce = magnitudeVelocity * incidentArea * dragCoefficient * -1; 												// the -1 in this statement is what makes it an opposing force.
		b2Vec2 dragVector = b2Vec2( cos(angleOfForwardDirection) * dragForce , sin(angleOfForwardDirection) * dragForce);	// the -1 here is used to correct for the physics engine's inverted y axis.

		// There is another drag force which represents the fluid viscosity. It acts to slow spinning objects.
		float viscosityDragCoeff = 0.5;
		float viscDragForce = viscosityDragCoeff * magnitudeArea * magLinearVelocityOfRotatingFace * -1;
		float viscDragAngle = atan2( faceCenter.y - worldCenter.y, faceCenter.x - worldCenter.x) + 0.5 * pi; //angle = atan2(y2 - y1, x2 - x1) /// the visc drag angle is orthogonal to the angle between the face center and the center of the spinning body.
		b2Vec2 viscDragVector = b2Vec2( cos(viscDragAngle) * viscDragForce , sin(viscDragAngle) * viscDragForce );

		b2Vec2 totalDragVector = dragVector + viscDragVector;

		// calculate the force of lift
		float liftCoeff  = 0.5;
		float atmosphericDensity = 1;
		float liftForce = liftCoeff * ((atmosphericDensity * (magnitudeVelocity*magnitudeVelocity))/2) * magnitudeArea * -1;

		// finally, the lift force must not exceed the drag force. This is a fundamental physical fact.
		if (abs(liftForce) > abs(dragForce)) {
			liftForce = liftForce * (dragForce/liftForce);
		}

		float liftAngle = angleOfForwardDirection;

		// the lift angle can be calculated by. 
		// basically find the trailing edge of the plane and check if it is in a left or right hand direction. Then rotate the lift angle left or right.
 		bool isP2theDMost = chooseDMostVertex(angleOfForwardDirection,  p1r,  p2r) ; // given 2 vertices, identify which one lies most in a given direction. return false if it is p1 and true if it is p2.
		if (isP2theDMost) {
			bool isP1TheLeftMost = chooseDMostVertex(angleOfForwardDirection + (0.5 * pi),  p1r,  p2r) ; // given 2 vertices, identify which one lies most in a given direction. return false if it is p1 and true if it is p2.
			if (isP1TheLeftMost) {
				liftAngle -= (0.5 * pi);
			}
			else {
				liftAngle += (0.5 * pi);
			}
		}
		else {
			bool isP1TheLeftMost = chooseDMostVertex(angleOfForwardDirection + (0.5 * pi),  p1r,  p2r) ; // given 2 vertices, identify which one lies most in a given direction. return false if it is p1 and true if it is p2.
			if (isP1TheLeftMost) {
				liftAngle += (0.5 * pi);
			}
			else {
				liftAngle -= (0.5 * pi);
			}
		}

		b2Vec2 fluidDynamicForce = b2Vec2(cos(liftAngle ) * liftForce, sin(liftAngle ) * liftForce );

		// draw a line so you can visualize the lift force.
		if (TestMain::getFluidDynamicForcesViewStatus()) {
			b2Vec2 visPosFluid = b2Vec2(faceCenter.x + totalDragVector.x, faceCenter.y + totalDragVector.y);
			b2Color segmentColorB = b2Color(255, 0, 00);
			local_debugDraw_pointer->DrawSegment(faceCenter ,visPosFluid ,segmentColorB );

			visPosFluid = b2Vec2(faceCenter.x + fluidDynamicForce.x, faceCenter.y + fluidDynamicForce.y);
			segmentColorB = b2Color(0, 0, 255);
			local_debugDraw_pointer->DrawSegment(faceCenter ,visPosFluid ,segmentColorB );
		}
	
		// apply the force to the body directly in the center of the face.
		bone->p_body->ApplyForce(totalDragVector, faceCenter, true);

		if (	fluidDynamicForce.x < 1000 && fluidDynamicForce.y < 1000
			&& 	fluidDynamicForce.x > -1000 && fluidDynamicForce.y > -1000 ) {
			bone->p_body->ApplyForce(fluidDynamicForce, faceCenter, true);
		}
	}
}



void runBiomechanicalFunctions () {
	unsigned int spacesUsedSoFar =0;

	bool alreadyDrawnThisTurn = false;

	std::list<BonyFish>::iterator fish;

	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{

			float energyUsedThisTurn = 0;

			// update the fish's senses
			for (int j = 0; j < N_FINGERS; ++j) {
				nonRecursiveSensorUpdater (fish->bones[j]);

				// perform the flight simulation on the fish
				flightModel( fish->bones[j] );
			}
		
			// sensorium size is based on the size of the ANN. Whether or not it is populated with numbers depends on the size of the input connector matrix.
			unsigned long sizeOfInputLayer = 0;
			unsigned long sizeOfOutputLayer = 0;
			unsigned long num_layers =  (unsigned long)(fish->brain->layers.size());

			std::list<layerDescriptor>::iterator layer;
			layer = fish->brain->layers.begin();
			sizeOfInputLayer = layer->neurons.size();
			std::advance(layer, num_layers-1);
			sizeOfOutputLayer = layer->neurons.size();

			// float previousOutputs [sizeOfOutputLayer];

			float sensorium[ sizeOfInputLayer ];

			for (unsigned int j = 0; j < sizeOfInputLayer; ++j)
			{
				if (j >= N_SENSECONNECTORS) {
					continue;	// if the sensorium array is bigger than the array of input connectors, you can just leave the rest blank.
				}

				switch (fish->inputMatrix[j].sensorType) {
					case SENSOR_FOODRADAR:
						sensorium[j] = fish->bones[ fish->inputMatrix[j].connectedToLimb  ]->sensation_radar;
					break;

					case SENSOR_TOUCH:
						sensorium[j] = fish->bones[ fish->inputMatrix[j].connectedToLimb  ]->sensation_touch;
					break;

					case SENSOR_JOINTANGLE:
						sensorium[j] = fish->bones[ fish->inputMatrix[j].connectedToLimb  ]->sensation_jointangle;
					break;

					case SENSOR_TIMER:
						sensorium[j] = sin(2 * pi * fish->inputMatrix[j].timerFreq * 	fish->inputMatrix[j].timerPhase);
						fish->inputMatrix[j].timerPhase += 0.001;
						if (fish->inputMatrix[j].timerPhase > 1) {
							fish->inputMatrix[j].timerPhase = 0;
						}
 
					break;

					case SENSECONNECTOR_UNUSED:
						sensorium[j] = 0.0f;
					break;
				 
				}		
			}

			// feed information into brain
			float * motorSignals = fann_run(fish->ann, sensorium);
		
			for (unsigned int j = 0; j < sizeOfOutputLayer; ++j)
			{
				if (j >= N_SENSECONNECTORS) {
					continue;	// if the sensorium array is bigger than the array of input connectors, you can just leave the rest blank.
				}

				switch (fish->outputMatrix[j].sensorType) {
					case SENSECONNECTOR_MOTOR:
	
						// first just check to make sure the bone and joint is good.
						// this paragraph could be condensed very easily
						if ( !fish->bones[fish->outputMatrix[j].connectedToLimb]->isUsed || !fish->bones[fish->outputMatrix[j].connectedToLimb]->init) {
							continue;
						}
						else {
							if (fish->bones[fish->outputMatrix[j].connectedToLimb]->isRoot) {
								continue;
							}
							if (fish->bones[fish->outputMatrix[j].connectedToLimb]->joint->isUsed && fish->bones[fish->outputMatrix[j].connectedToLimb]->joint->init) {
								if (fish->bones[fish->outputMatrix[j].connectedToLimb]->joint->p_joint != nullptr) {

									float motorSpeed = motorSignals[j] * 10;

									energyUsedThisTurn += abs(motorSignals[j]);

									if (false) {

										// clip the possible motor speed to the speed limit.
										float speedLimit = fish->bones[fish->outputMatrix[j].connectedToLimb]->joint->speedLimit;
										if (speedLimit > 100) {
											speedLimit = 100;
										}
										else if (speedLimit < -100) {
											speedLimit = -100;
										}
										
										if (motorSpeed > speedLimit) {
											motorSpeed = speedLimit;
										}
										else if (motorSpeed < -speedLimit) {
											motorSpeed =-speedLimit;
										}
									}
									fish->bones[fish->outputMatrix[j].connectedToLimb]->joint->p_joint->SetMotorSpeed(motorSpeed);
								}
							}
						}	
					break;
				}
			}

			if (m_deepSeaSettings.gameMode == GAME_MODE_ECOSYSTEM) {
				for (int i = 0; i < N_FINGERS; ++i) {

					// if sunlight fell on a leaf, give energy to it.
					if (fish->bones[i]->isLeaf && fish->bones[i]->flagPhotosynth) {
						fish->bones[i]->flagPhotosynth = false;
						fish->energy += 100.0f;
					}

					// 1 energy is lost per bone per turn for homeostasis or whatever. this is also so that plants can't live forever without sunlight.
					fish->energy -= 0.1f;
				}

				// kill the fish if it is out of energy.
				fish->energy -= energyUsedThisTurn;
				// printf("charhe: %f\n", fish->energy);
				if (fish->energy < 0) {
					fish->flagDelete = true;
				}

				// give the fish some bebes if it has collected enough food.
				if (fish->energy > fish->reproductionEnergyCost) {
					ecosystemModeBeginGeneration( &(*fish) );
					fish->energy -= fish->reproductionEnergyCost;
				}

			}

			if (TestMain::getBrainWindowStatus()) {
				if (fish->selected && !alreadyDrawnThisTurn) {
					alreadyDrawnThisTurn = true;
					drawNeuralNetworkFromDescriptor(motorSignals, sensorium, &spacesUsedSoFar, &(*fish));
				}
			}

			if (TestMain::getBodyWindowStatus()) {
				if (fish->selected && !alreadyDrawnThisTurn) {
					alreadyDrawnThisTurn = true;
					// drawNeuralNetworkFromDescriptor(motorSignals, sensorium, &spacesUsedSoFar, &(*fish));
					drawBodyEditingWindow(&(*fish));
				}

			}
			

			// calculate output wiggle
			float outputWiggleThisTurn = 0;
			for (uint j = 0; j < sizeOfOutputLayer; ++j)
			{
				if (fish->previousOutputs != nullptr) { // it is null on the first rurn.
					outputWiggleThisTurn += abs( (motorSignals[j]) - (fish->previousOutputs[j])  );	
				}
			}
			fish->previousOutputs = motorSignals;
			fish->filteredOutputWiggle -= 0.5 * fish->filteredOutputWiggle;
			fish->filteredOutputWiggle += outputWiggleThisTurn;
			
			// calculate distance moved
			b2Vec2 rootCenter = fish->bones[0]->p_body->GetWorldCenter();
			b2Vec2 distanceThisTurn =  b2Vec2( rootCenter.x - fish->previousRootPosition.x, rootCenter.y - fish->previousRootPosition.y) ;
			fish->distanceMovedSoFar += magnitude(distanceThisTurn);
			fish->previousRootPosition = rootCenter;

	}
}

void flagSelectedFishForDeletion(int arg) {
	unused_variable((void *)&arg);
	std::list<BonyFish>::iterator fish;
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		if (fish->selected) {
			fish->flagDelete = true;
			fish->selected = false;
		}
	}

	// iterate through food particles
	for  (int i = 0; i < N_FOODPARTICLES; i++) {
		if (food[i]->selected) {
			food[i]->flagDelete = true;
			food[i]->selected = false;
		}
	}
}

void voteSelectedFish(int arg) {

	unused_variable((void *)&arg);

	std::list<BonyFish>::iterator fish;
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		if (fish->selected) {
			vote( &(*fish) );
		}
	}
}

void deselectAll (int arg) {

	unused_variable((void *)&arg);

	std::list<BonyFish>::iterator fish;
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		if (fish->selected) {
			fish->selected = false;
		}
	}
}


void selectAll (int arg) {

	unused_variable((void *)&arg);

	std::list<BonyFish>::iterator fish;
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		if (!fish->selected) {
			fish->selected = true;
		}
	}
}


void invertSelection (int arg) {

	unused_variable((void *)&arg);

	std::list<BonyFish>::iterator fish;
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		fish->selected = !fish->selected;
	}
}

void pinToGrid(int arg) {

	unused_variable((void *)&arg);

	std::list<BonyFish>::iterator fish;

	uint gridSize = int(sqrt(m_deepSeaSettings.exploratory_nFish));

	// b2Vec2 gridPosition = b2Vec2(0.0f, 0.0f);
	float gridSpacing = 5.0f;

	uint index = 0;

	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{

		uint rootBone = 0;

		for (uint i = 0; i < N_FINGERS; ++i)
		{
			if (fish->bones[i]->isRoot) {
				rootBone = i;
				break;
			}
		}


		uint row = index / gridSize;
		uint column = index % gridSize;

		b2Vec2 mool = b2Vec2(row * gridSpacing, column * gridSpacing);
		fish->bones[rootBone]->p_body->SetTransform(mool, 0.0f);

		fish->bones[rootBone]->p_body->SetType(b2_staticBody);

		index += 1;
	}
}

void releaseFromGrid(int arg) {

	unused_variable((void *)&arg);

	std::list<BonyFish>::iterator fish;

	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{

		for (uint i = 0; i < N_FINGERS; ++i)
		{
			if (fish->bones[i]->isRoot) {

				fish->bones[i]->p_body->SetType(b2_dynamicBody);

			}
		}
	}
}


void selectFishWithGreatestWiggle (int arg) {
	unused_variable((void *)&arg);
	std::list<BonyFish>::iterator fish;
	fish = fishes.begin();

	BonyFish * theWiggliest = &(*fish);

	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{

		if (fish->filteredOutputWiggle > theWiggliest->filteredOutputWiggle) {

			printf("this fish is even wigglier: %f\n",fish->filteredOutputWiggle );

			theWiggliest = &(*fish);
		}
	}
	theWiggliest->selected = true;
}

void selectFishWhoMovedTheFurthest (int arg) {
	unused_variable((void *)&arg);

	std::list<BonyFish>::iterator fish;
	fish = fishes.begin();

	BonyFish * theFurthest = &(*fish);

	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		if (fish->distanceMovedSoFar > theFurthest->distanceMovedSoFar && isinf(fish->distanceMovedSoFar) == false) {

			printf("this fish has gone even further: %f\n",fish->distanceMovedSoFar );
			theFurthest = &(*fish);
		}
	}
	theFurthest->selected = true;
}

void selectFurthestFromOrigin (int arg) {
	unused_variable((void *)&arg);

	std::list<BonyFish>::iterator fish;
	fish = fishes.begin();

	BonyFish * theFurthest = &(*fish);

	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		if (magnitude(fish->bones[0]->p_body->GetWorldCenter() ) > magnitude(theFurthest->bones[0]->p_body->GetWorldCenter())) {

			printf("this fish has gone even further: %f\n",fish->distanceMovedSoFar );
			theFurthest = &(*fish);
		}
	}
	theFurthest->selected = true;
}

void selectClosestToFood (int arg) {

	unused_variable((void *)&arg);

	std::list<BonyFish>::iterator fish;

	BonyFish * theClosest = &(*fish);
	float theBestDistanceSoFar = 1000000000.0f; // not going to work if all the fish are more than a billion units away.
	bool chosen = false;

	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{

		// iterate through food particles
		for  (int i = 0; i < N_FOODPARTICLES; i++) {

			if (food[i]->init && food[i]->isUsed) {

				b2Vec2 vectorToFood = b2Vec2(food[i]->position.x - fish->bones[0]->p_body->GetWorldCenter().x , food[i]->position.y - fish->bones[0]->p_body->GetWorldCenter().y);

				float distanceToFood = magnitude(vectorToFood);

				if (distanceToFood < theBestDistanceSoFar) {
					theClosest = &(*fish);
					theBestDistanceSoFar = distanceToFood;
					chosen = true;
				}

			}
		}
	}

	if (chosen) {
		theClosest->selected = true;
	}
}



void deepSeaLoop () {

	TestMain::PreStep();

	TestMain::Step();

	if (!local_m_world->IsLocked() ) {

		if (loopCounter < loopSafetyLimit) {
			// startNextGeneration = false;
		}		

		loopCounter ++;

		removeDeletableFish();

		if (startNextGeneration && m_deepSeaSettings.gameMode == GAME_MODE_EXPLORATION ) {
			if (loopCounter > loopSafetyLimit) {
				exploratoryModeBeginGeneration ( );
				loopCounter = 0;
			}
		}
		
		startNextGeneration = false;

		std::list<Lamp>::iterator lomp;
		for (lomp = lamps.begin(); lomp !=  lamps.end(); ++lomp) 	{
			if (false) {
				shine(&(*lomp));
			}
			
		}

		drawingTest();

		runBiomechanicalFunctions();

		if (flagAddFood) {
			flagAddFood = false;
			addRandomFoodParticle(0);
		}

		if (flagAddPlant) {
			flagAddPlant = false;


			fishDescriptor_t * newFishBody = basicPlant();

			mutateFishDescriptor (newFishBody, 0.1, 0.5);

			loadFish ( *newFishBody, NULL, b2Vec2(0.0f, 0.0f)) ;
		}

		for  (int i = 0; i < N_FOODPARTICLES; i++) {

			if (food[i]->init && food[i]->isUsed) {

				// perform the flight simulation on the fish
				flightModel( food[i] );

				food[i]->position = food[i]->p_body->GetWorldCenter(); // update positions of all the food particles
			}
		}
	}

	TestMain::PostStep();
}

void deepSeaControlP () {
	flagAddPlant = true;
}

void deepSeaControlA () { 
	flagAddFood = true;
}
void deepSeaControlB () {
	// std::list<BonyFish>::iterator fish;
	// for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
	// 	if (fish->selected) {
	// 		fish->flagDelete = true;
	// 		fish->selected = false;
	// 	}
	// }
}

void collisionHandler (void * userDataA, void * userDataB, b2Contact * contact) {
	bool et = false;
	bool fud = false;
	bool lef = false;

	if (userDataA == nullptr || userDataB == nullptr || startNextGeneration || loopCounter < loopSafetyLimit) {
		return;
	}
	else {
		;
	}

	uDataWrap * p_dataA = (uDataWrap *) userDataA;
	uDataWrap * p_dataB = (uDataWrap *) userDataB;
	uDataWrap dataA = *p_dataA;
	uDataWrap dataB = *p_dataB;

	if( dataA.dataType == TYPE_TOUCHSENSOR ) {
		((BoneUserData *)(dataA.uData))->sensation_touch +=0.5f;
	}

	if( dataB.dataType == TYPE_TOUCHSENSOR ) {
		((BoneUserData *)(dataB.uData))->sensation_touch +=0.5f; 
	}

	if( dataA.dataType == TYPE_MOUTH ) {
		et = true;
		if( dataB.dataType == TYPE_FOOD && !
				((foodParticle_t*)(dataB.uData) )->flagDelete ) {
			fud = true;
		}
		if( dataB.dataType == TYPE_LEAF && !
				((foodParticle_t*)(dataA.uData) )->flagDelete) {
			lef = true;
		}
	}

	else if( dataB.dataType == TYPE_MOUTH ) {
		et = true;
		if( dataA.dataType == TYPE_FOOD && !
				((foodParticle_t*)(dataA.uData) )->flagDelete) {
			fud = true;
		}
		if( dataA.dataType == TYPE_LEAF && !
				((foodParticle_t*)(dataA.uData) )->flagDelete) {
			lef = true;
		}
	}
	
	if (et && fud) {
		// printf("monch");

		if( dataB.dataType == TYPE_MOUTH ) {
			BonyFish * fish = (((BoneUserData *)(dataB.uData))->p_owner );
			BoneUserData * food = ((BoneUserData *)(dataA.uData) );

			if (m_deepSeaSettings.gameMode == GAME_MODE_EXPLORATION) {
			    vote(fish);
			}

			else if (m_deepSeaSettings.gameMode == GAME_MODE_ECOSYSTEM) {
				food->flagDelete = true;
				fish->feed(food->energy);
			}
		}
		else if (dataA.dataType == TYPE_MOUTH) {
			BonyFish * fish = (((BoneUserData *)(dataA.uData))->p_owner );
			BoneUserData * food = ((BoneUserData *)(dataB.uData) );
			if (m_deepSeaSettings.gameMode == GAME_MODE_EXPLORATION) {
				vote(fish);
			}
			else if (m_deepSeaSettings.gameMode == GAME_MODE_ECOSYSTEM) {
				food->flagDelete = true;
				fish->feed(food->energy);
			}
		}
	}
	if (et && lef) {
		// printf("monch");

		if( dataB.dataType == TYPE_MOUTH ) {
			BonyFish * fish = (((BoneUserData *)(dataB.uData))->p_owner );
			BoneUserData * food = ((BoneUserData *)(dataA.uData) );

			if (m_deepSeaSettings.gameMode == GAME_MODE_EXPLORATION) {
			    vote(fish);
			}

			else if (m_deepSeaSettings.gameMode == GAME_MODE_ECOSYSTEM) {
				food->p_owner->flagDelete = true;
				fish->feed(food->energy);
			}
		}
		else if (dataA.dataType == TYPE_MOUTH) {
			BonyFish * fish = (((BoneUserData *)(dataA.uData))->p_owner );
			BoneUserData * food = ((BoneUserData *)(dataB.uData) );
			if (m_deepSeaSettings.gameMode == GAME_MODE_EXPLORATION) {
				vote(fish);
			}
			else if (m_deepSeaSettings.gameMode == GAME_MODE_ECOSYSTEM) {
				food->p_owner->flagDelete = true;
				fish->feed(food->energy);
			}
		}
	}
}

// ------------------------------------------------------------------

// the following code is an attempt to integrate particle drawing (from the test of the same name) into deepsea, in a way that can be applied to all deepsea maps.

void ParticleDrawingKeyboard(int key)
{
	m_drawing = TestMain::getPaintingStatus(); //key != 'X';

	// printf("%i\n", TestMain::getPaintingStatus());

	m_particleFlags = 0;
	m_groupFlags = 0;
	switch (key)
	{
	case 0:
		m_particleFlags = b2_elasticParticle;
		m_groupFlags = b2_solidParticleGroup;
		break;
	case 1:
		m_particleFlags = b2_powderParticle;
		break;
	case 2:
		m_groupFlags = b2_rigidParticleGroup | b2_solidParticleGroup;
		break;
	case 3:
		m_particleFlags = b2_springParticle;
		m_groupFlags = b2_solidParticleGroup;
		break;
	case 4:
		m_particleFlags = b2_tensileParticle;
		break;
	case 5:
		m_particleFlags = b2_viscousParticle;
		break;
	case 6:
		m_particleFlags = b2_wallParticle;
		m_groupFlags = b2_solidParticleGroup;
		break;
	case 7:
		m_particleFlags = b2_barrierParticle | b2_wallParticle;
		break;
	case 8:
		m_particleFlags = b2_barrierParticle;
		m_groupFlags = b2_rigidParticleGroup;
		break;
	case 9:
		m_particleFlags = b2_barrierParticle | b2_elasticParticle;
		m_groupFlags = b2_solidParticleGroup;
		break;
	case 10:
		m_particleFlags = b2_barrierParticle | b2_springParticle;
		m_groupFlags = b2_solidParticleGroup;
		break;
	case 11:
		m_particleFlags = b2_wallParticle | b2_repulsiveParticle;
		break;
	case 12:
		m_particleFlags = b2_colorMixingParticle;
		break;
	case 13:
		m_particleFlags = b2_zombieParticle;
		break;
	default:
		break;
	}
	TestMain::SetParticleParameterValue(DetermineParticleParameter());
}


void SplitParticleGroups()
{
	for (b2ParticleGroup* group = local_m_particleSystem->
			GetParticleGroupList(); group; group = group->GetNext())
	{
		if (group != m_lastGroup &&
			(group->GetGroupFlags() & b2_rigidParticleGroup) &&
			(group->GetAllParticleFlags() & b2_zombieParticle))
		{
			// Split a rigid particle group which may be disconnected
			// by destroying particles.
			local_m_particleSystem->SplitParticleGroup(group);
		}
	}
}

// i dont think this function is ever called by anything.
void stepForParticleDrawing () {
	const uint32 parameterValue = TestMain::GetParticleParameterValue();
		m_drawing = (parameterValue & e_parameterMove) != e_parameterMove;
		printf("%i\n",TestMain::getPaintingStatus());
		if (TestMain::getPaintingStatus() == 0) {
			m_drawing = 0;
		}
		if (m_drawing)
		{
			switch (parameterValue)
			{
				case b2_elasticParticle:
				case b2_springParticle:
				case b2_wallParticle:
					m_particleFlags = parameterValue;
					m_groupFlags = b2_solidParticleGroup;
					break;
				case e_parameterRigid:
					// b2_waterParticle is the default particle type in
					// LiquidFun.
					m_particleFlags = b2_waterParticle;
					m_groupFlags = b2_rigidParticleGroup |
					               b2_solidParticleGroup;
					break;
				case e_parameterRigidBarrier:
					m_particleFlags = b2_barrierParticle;
					m_groupFlags = b2_rigidParticleGroup;
					break;
				case e_parameterElasticBarrier:
					m_particleFlags = b2_barrierParticle | b2_elasticParticle;
					m_groupFlags = 0;
					break;
				case e_parameterSpringBarrier:
					m_particleFlags = b2_barrierParticle | b2_springParticle;
					m_groupFlags = 0;
					break;
				case e_parameterRepulsive:
					m_particleFlags = b2_repulsiveParticle | b2_wallParticle;
					m_groupFlags = b2_solidParticleGroup;
					break;
				default:
					m_particleFlags = parameterValue;
					m_groupFlags = 0;
					break;
			}
		}

		if (local_m_particleSystem->GetAllParticleFlags() & b2_zombieParticle)
		{
			SplitParticleGroups();
		}
}

void test_runAllUnitTests() {

	int n_failures = 0;
	int tests_so_far = 0;

	if ( test_chooseDMostVertex() ) { n_failures++; } tests_so_far++;

	printf("performed %u tests with %u failures\n", tests_so_far, n_failures);
}