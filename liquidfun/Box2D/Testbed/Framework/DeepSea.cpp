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

#include "glui/glui.h"

float pi = 3.14159f;

// global settings to keep track of the game parameters.
unsigned int currentNumberOfFish = 0;
unsigned int generationsThisGame = 0;
bool startNextGeneration = false;
deepSeaSettings m_deepSeaSettings;					// = {

uint64 loopCounter = 0 ;
uint32 loopSafetyLimit = 100;
bool flagAddFood = false;	// true so it adds once on startup!
bool flagAddPlant= false;

bool userControlInputA;
bool userControlInputB;

unsigned int currentlySelectedLimb =0;

const std::string defaultFishName = std::string("default") ;

std::list<Lamp> lamps;
// BoneUserData * food[N_FOODPARTICLES];

std::list<Species> ecosystem = *(new std::list<Species>);
Species * defaultSpecies = new Species; // a default start because the list cant be used uninitialized. Also, used as the only active species in the laboratory mode.

std::list<Terrain> environment = *(new std::list<Terrain>);

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
    static std::uniform_real_distribution<> dis(0, 1); 
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
}

boneAndJointDescriptor_t::boneAndJointDescriptor_t () {
	attachedTo = 0; 				// the INDEX (out of N_FINGERS) of the bone it is attached to. Storing data in this way instead of a pointer means that mutating it will have hilarious rather than alarming results.
	length = 1.0f;
	rootThickness = 0.2f;
	tipThickness = 0.2f;
	isRoot = false;
	isMouth = false;
	sensor_radar = false; 			// like an olfactory sensor . senses distance from food
	sensor_altradar= false;
	sensor_eye = false;
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

uDataWrap::uDataWrap(void * dat, unsigned int typ) {
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
	flagDelete = false;

	// if (isLeaf && !isRoot){
	hasGrown = false;
	// }
	// else {
	// 	hasGrown = true;
	// }

	isFood =  boneDescription.isFood;

	isWeapon  = boneDescription.isWeapon;									// weapons destroy joints to snip off a limb for consumption. optionally, they can produce a physical effect.
	energy = ((rootThickness + tipThickness)/2) * length * density * 10000; 		// the nutritive energy stored in the tissue of this limb; used by predators and scavengers

	// accessory features cost more so you can't just go ham on them. And the effect stacks so it's better to specialize.
	// touch and jointangle are considered necessary for any moving limb, so they come free.
	// if (isMouth				&& !isRoot) 	{ energy = energy * 1.5; };
	// if (sensor_radar 		&& !isRoot) 	{ energy = energy * 1.5; };
	// if (isWeapon 			&& !isRoot) 	{ energy = energy * 1.5; };
	// if (isLeaf 				&& !isRoot) 	{ energy = energy * 1.5; };

	// energy = energy * 10000; // this is a constant that is used to help the value of energy in food roughly match the effort required to get food.

	color = boneDescription.color;
	outlineColor = boneDescription.outlineColor;

	tipCenter = b2Vec2(0.0f,0.1f); 											// these are used so the skeleton master can remember his place as he traverses the heirarchy of souls.
	rootCenter = b2Vec2(0.0f,0.0f); 	

	unsigned int count = 4;

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

			attachedTo = boneDescription.attachedTo;

			offsetOnBody = (attachesTo->offsetOnBody + (attachesTo->length/2)) + length/2;	
			rootCenter = attachesTo->tipCenter;
		}

		p_body->SetTransform(b2Vec2(positionOffset.x, positionOffset.y + offsetOnBody.y),0);
		
		shape.Set(vertices, count);

		joint = new JointUserData( boneDescription, this, fish); 	// the joint that attaches it into its socket 
	}	

	isUsed=  boneDescription.used;
};

Terrain::Terrain(b2Vec2 startingPosition) {

	density = 1.0f;

	position = startingPosition;

	isUsed = false;
	flagDelete = false;

	selected = false;

	color = b2Color(0.5f, 0.5f, 0.5f);
	outlineColor = b2Color(0.8f, 0.8f, 0.8f);

	collisionGroup = 1;
}

void nonRecursiveBoneIncorporator(BoneUserData * p_bone) {

	// if (!p_bone->isLeaf || p_bone->isRoot) {
	// printf("_b\n");
	p_bone->p_fixture = p_bone->p_body->CreateFixture(&(p_bone->shape), p_bone->density);	// this endows the shape with mass and is what adds it to the physical world.

	// 	if (p_bone->isLeaf) {
	// 		p_bone->hasGrown = true; // so happy to write this
	// 	}
	// }

	// printf("_c\n");
	
	if (p_bone->isMouth) {
		uDataWrap * p_dataWrapper = new uDataWrap(p_bone, TYPE_MOUTH);
		p_bone->p_body->SetUserData((void *)p_dataWrapper);
	}
	else if (p_bone->sensor_touch) {
		uDataWrap * p_dataWrapper = new uDataWrap(p_bone, TYPE_TOUCHSENSOR);
		p_bone->p_body->SetUserData((void *)p_dataWrapper);
	}
	else if (p_bone->isLeaf) {
		uDataWrap * p_dataWrapper = new uDataWrap(p_bone, TYPE_LEAF);
		p_bone->p_body->SetUserData((void *)p_dataWrapper);
	}
	// else if (p_bone->isFood) {
	// 	uDataWrap * p_dataWrapper = new uDataWrap(p_bone, TYPE_FOOD);
	// 	p_bone->p_body->SetUserData((void *)p_dataWrapper);
	// }
	else {
		uDataWrap * p_dataWrapper = new uDataWrap(p_bone, TYPE_DEFAULT);
		p_bone->p_body->SetUserData((void *)p_dataWrapper);
	}

	// printf("_d\n");

	b2Filter tempFilter = p_bone->p_fixture->GetFilterData();
	tempFilter.groupIndex = p_bone->collisionGroup;
	p_bone->p_fixture->SetFilterData(tempFilter);

	if (!p_bone->isRoot) {
		p_bone->joint->jointDef.collideConnected = false; // this means that limb segments dont collide with their children

		p_bone->joint->p_joint = (b2RevoluteJoint*)local_m_world->CreateJoint( &(p_bone->joint->jointDef) );

		p_bone->joint->hasGrown = true;


			
	}
	// printf("_e\n");
	// if the body is root and is also a leaf, fix its rotation. This is because trees are 'planted in the ground'.
	// if () {

	// }

	// p_bone->isUsed = true;
	p_bone->selected = false;
	p_bone->hasGrown = true;
}


void nonRecursiveTerrainIncorporator(Terrain * p_bone) {

	p_bone->p_fixture = p_bone->p_body->CreateFixture(&(p_bone->shape), p_bone->density);	// this endows the shape with mass and is what adds it to the physical world.

	p_bone->p_body->SetTransform(p_bone->position, p_bone->p_body->GetAngle());

	uDataWrap * p_dataWrapper = new uDataWrap(p_bone, TYPE_DEFAULT);
	p_bone->p_body->SetUserData((void *)p_dataWrapper);

	b2Filter tempFilter = p_bone->p_fixture->GetFilterData();
	tempFilter.groupIndex = p_bone->collisionGroup;
	p_bone->p_fixture->SetFilterData(tempFilter);

	p_bone->isUsed = true;
	p_bone->selected = false;

	
}

void nonRecursiveSensorUpdater (BoneUserData * p_bone) {
	if ( !p_bone->isUsed) {
		return;
	}

	if (!p_bone->hasGrown) {
		return;
	}

	// update all joint angle sensors
	if (!p_bone->isRoot) {
		if (p_bone->joint->isUsed) {
			if (p_bone->sensor_jointangle) {
				p_bone->sensation_jointangle= p_bone->joint->p_joint->GetJointAngle();
			}
		}
	}
	
	// update scalar food radars
	// they react more strongly the closer they get to food
	if (p_bone->sensor_radar) {
		p_bone->sensation_radar = 0.0f;
		// for  (int i = 0; i < N_FOODPARTICLES; i++) {

		// 	if (food[i]->isUsed) {
		// 		b2Vec2 boneCenterWorldPosition = p_bone->p_body->GetWorldCenter();
		// 		b2Vec2 positionalDifference = b2Vec2((boneCenterWorldPosition.x - food[i]->position.x),(boneCenterWorldPosition.y - food[i]->position.y));
		// 		float distance = magnitude (positionalDifference);
		// 		if (distance > 0) {
		// 			p_bone->sensation_radar += 1/distance;
		// 		}
		// 	}
		// }


		// this part is heinously computationally inefficient: iterating through the whole global list of bones, once per sensor, every step.
		// you can do better, but i don't know how right now
		std::list<Species>::iterator currentSpecies;
		for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
		{
			std::list<BonyFish>::iterator fish;
			for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
			{
				for (int i = 0; i < N_FINGERS; ++i)
				{
					if ( fish->bones[i]->isLeaf ) {

						if (fish->bones[i]->isUsed) {
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

	// alt radars measure the angle to the closest piece of food. They don't get weaker with distance. it's supposed to be easier to interpret than the scalar one.
	if (p_bone->sensor_altradar) {
		// p_bone->sensation_altradar = 0.0f;

		// // find the closest food
		// float closestFoodDistance = 1000000000.0f;
		// int closestFoodIndex = 0;

		// b2Vec2 boneCenterWorldPosition = p_bone->p_body->GetWorldCenter();

		// for  (int i = 0; i < N_FOODPARTICLES; i++) {

		// 	if (food[i]->isUsed) {
		// 		b2Vec2 positionalDifference = b2Vec2((boneCenterWorldPosition.x - food[i]->position.x),(boneCenterWorldPosition.y - food[i]->position.y));
		// 		float distance = magnitude (positionalDifference);
		// 		if (distance < closestFoodDistance) {
		// 			closestFoodIndex = i;
		// 			closestFoodDistance = distance;
		// 		}
		// 	}
		// }

		// // find the angle between it and this bone
		// if (food[closestFoodIndex]->isUsed) {
		// 	p_bone->sensation_altradar = atan2(  (boneCenterWorldPosition.y - food[closestFoodIndex]->position.y )   ,  (boneCenterWorldPosition.x - food[closestFoodIndex]->position.x)   );

		// 	// printf("original value: %f\n",p_bone->sensation_altradar );

		// 	// rotate it 90 degrees so the discontinuity at 180 degrees happens behind the sensor not in front of it
		// 	p_bone->sensation_altradar  += pi;


		// 	// printf("value + add: %f\n",p_bone->sensation_altradar );

		// 	// adding radians, wrap around 0 if the result is larger than a full circle
		// 	if (p_bone->sensation_altradar > (pi) ) {
		// 		p_bone->sensation_altradar -= (2 * pi);
		// 	}


		// 	// printf("wrapped: %f\n",p_bone->sensation_altradar );
		// }
	}


	// the eye works by measuring the color of bodies within a narrow field of vision. it does get weaker with distance, but only slightly as it is very directional.
	if (p_bone->sensor_eye) {


		// go through the list of all the other bones.

			// get the global angle to each one.

			// get the current bone's angle.

			// find the difference between the current bone's angle and the global angle to the other bone.

			// if it is less than this eye's FOV, prepare to add it to the sum.

			// it is reduced according to its distance from the eye.

		// the sum for this turn is multiplied by the gain, which is the ratio of a whole circle to the eye's FOV.
			
		

	}

}

// void addFoodParticle(b2Vec2 position) {
// 	unsigned int emptyFoodIndex = 0;
// 	for (unsigned int i = 0; i < N_FOODPARTICLES; ++i)
// 	{
// 		if (food[i]->isUsed) {
// 			emptyFoodIndex ++;
// 		}
// 		else {
// 			break;
// 		}
// 	}

// 	boneAndJointDescriptor_t foodDescriptor = *(new boneAndJointDescriptor_t());
// 	foodDescriptor.used = true;
// 	foodDescriptor.isRoot = true;
// 	foodDescriptor.isFood = true;
// 	foodDescriptor.isMouth = false;
// 	foodDescriptor.sensor_touch = false;
// 	foodDescriptor.isLeaf = false;
// 	food[emptyFoodIndex] = new BoneUserData(foodDescriptor, nullptr, position, 0, false);
// 	food[emptyFoodIndex]->joint = new JointUserData(foodDescriptor, food[emptyFoodIndex], nullptr);
// 	food[emptyFoodIndex]->isFood = true;
// 	nonRecursiveBoneIncorporator(food[emptyFoodIndex]);
// 	food[emptyFoodIndex]->energy = 10000 + (RNG() * 5000);
// 	food[emptyFoodIndex]->p_body->SetTransform(position, (RNG() * 2 * pi) );
// }

void addRandomFoodParticle(int arg) {
	// float randomFoodAngle = 2 * pi * RNG();
	// b2Vec2 position = b2Vec2( cos( randomFoodAngle)* m_deepSeaSettings.originFoodRadius , sin(randomFoodAngle)* m_deepSeaSettings.originFoodRadius );
	// addFoodParticle(position);
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

networkDescriptor * createEmptyNetworkOfCorrectSize (fann * temp_ann) {
	return new networkDescriptor(temp_ann);
}

void BonyFish::feed(float amount) {
	printf("fed %f\n", amount);

	energy += amount;
}

BonyFish::BonyFish(fishDescriptor_t driedFish, fann * nann, b2Vec2 startingPosition) {

	numberOfTimesReproduced= 0;

	genes = driedFish;
	reproductionEnergyCost = 0.0f; // the amount of energy required to make a clutch of viable offspring. to be calculated

	flagDelete = false;
	selected = false;

	int randomCollisionGroup;	//= - (RNG() * 16.0f);

	if ( TestMain::getNoClipStatus() )  {
		randomCollisionGroup = -1; // set to -1 to disable all.
	}
	else {
		randomCollisionGroup = 1; // set to -1 to disable all.
	}

	float lowestLimbEnergy = 1000000000;
	for (int i = 0; i < N_FINGERS; ++i) {
		if (i == 0) {
			driedFish.bones[i].isRoot = true;
		}

		bones[i] = new BoneUserData(driedFish.bones[i], this, startingPosition, randomCollisionGroup, true);
		bones[i]->index = i;

		reproductionEnergyCost += bones[i]->energy;
		if ( bones[i]->energy < lowestLimbEnergy) {
			lowestLimbEnergy = bones[i]->energy;
		}
	}
	
	energy = lowestLimbEnergy - 200; //(reproductionEnergyCost * 0.5); // this was what was crashing it? lmao
	// energy = energy + (RNG() * 0.25 * energy);

	n_bones_used = 0;
	for (int i = 0; i < N_FINGERS; ++i) {
		if (driedFish.bones[i].used) {
			n_bones_used ++;

		}
	}

	// init = false; 										// true after the particle has been initialized. In most cases, uninitalized particles will be ignored.
	isUsed = false; 									// only true when the part is added to the world

	distanceMovedSoFar = 0.0f;
	filteredOutputWiggle = 0.0f;
	previousOutputs = nullptr;


	// for (unsigned int i = 0; i < N_FINGERS; ++i) { // if the limb doesn't already have sense and motor connectors, add them in.

	// 	if (driedFish.bones[i].used) {

	// 		bool connected_radar = false;
	// 		bool connected_angle = false;
	// 		bool connected_motor = false;

	// 		if (driedFish.bones[i].sensor_radar) {
	// 			for (unsigned int j = 0; j < N_SENSECONNECTORS; ++j) {
	// 				if (driedFish.inputMatrix[j].connectedToLimb == i ) {
	// 					if (driedFish.inputMatrix[j].sensorType == SENSOR_FOODRADAR) {connected_radar = true;}
	// 					if (driedFish.inputMatrix[j].sensorType == SENSOR_JOINTANGLE) {connected_angle = true;}
	// 				}
	// 				if (driedFish.outputMatrix[j].connectedToLimb == i ) { 
	// 					if (driedFish.outputMatrix[j].sensorType == SENSECONNECTOR_MOTOR) {connected_motor = true;}
	// 				}
	// 			}
	// 		}

	// 		if (!connected_radar) {
	// 			for (unsigned int j = 0; j < N_SENSECONNECTORS; ++j) {
	// 				if (driedFish.inputMatrix[j].sensorType == SENSECONNECTOR_UNUSED ) {
	// 					driedFish.inputMatrix[j].sensorType = SENSOR_FOODRADAR;
	// 					driedFish.inputMatrix[j].connectedToLimb = i;
	// 					break;
	// 				}
	// 			}
	// 		}

	// 		if (!connected_angle) {
	// 			for (unsigned int j = 0; j < N_SENSECONNECTORS; ++j) {
	// 				if (driedFish.inputMatrix[j].sensorType == SENSECONNECTOR_UNUSED ) {
	// 					driedFish.inputMatrix[j].sensorType = SENSOR_JOINTANGLE;
	// 					driedFish.inputMatrix[j].connectedToLimb = i;
	// 					break;
	// 				}
	// 			}
	// 		}

	// 		if (!connected_motor) {
	// 			for (unsigned int j = 0; j < N_SENSECONNECTORS; ++j) {
	// 				if (driedFish.outputMatrix[j].sensorType == SENSECONNECTOR_UNUSED ) {
	// 					driedFish.outputMatrix[j].sensorType = SENSECONNECTOR_MOTOR;
	// 					driedFish.outputMatrix[j].connectedToLimb = i;
	// 					break;
	// 				}
	// 			}
	// 		}
	// 	}
	// }

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

    // printf("Bony fish created, reproduces at %f energy\n", reproductionEnergyCost);

    // printf("bonyfish ready");
};

void moveAWholeFish (BonyFish * fish, b2Vec2 position) {
	for (int i = 0; i < N_FINGERS; ++i)
	{
		if ( !fish->bones[i]->isUsed ) {
			continue;
		}

		if (!fish->bones[i]->hasGrown) {
			continue;
		}

		fish->bones[i]->p_body->SetTransform(position, fish->bones[i]->p_body->GetAngle());
	}
}

void deleteJoint(BoneUserData * bone) {
	if (!bone->isRoot) { // root bones dont have joints

		if ( !bone->hasGrown) {
			return;
		}

		if (bone->joint != NULL && bone->joint != nullptr) {
			if (bone->joint->isUsed ) {
				if (bone->joint->p_joint != NULL && bone->joint->p_joint != nullptr) {
					local_m_world->DestroyJoint(bone->joint->p_joint);	
					// bone->joint->isUsed = false;
					// bone->joint->init = false;
					bone->joint->hasGrown = false;
				}
			}
		}
	}
}

// doesn't really delete. more like disable bone.
void deleteBone (BoneUserData * bone) {
	if (bone->isUsed && bone->flagDelete) {	

	if (!bone->hasGrown) {
			return;
		}

		bone->p_body->DestroyFixture(bone->p_fixture); // you can still grow it back after this.
		// local_m_world->DestroyBody(bone->p_body);

		// bone->isUsed = false;
		// bone->init = false; // the bone is still used in the fish's design, it's just missing at the moment.
		bone->hasGrown = false;
		bone->flagDelete = false;
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
			local_m_world->DestroyBody(fish->bones[i]->p_body); // this action is for real, you can't grow it back.
		}
		fish->isUsed = false;
		fish->flagDelete = false;
	}
}

// goes through the brain and adds 'biasNeuron' flag to neurons at the end of each layer. This is mainly so they can be drawn properly.
void flagBiasNeurons( networkDescriptor * network) {

	std::list<layerDescriptor>::iterator layer;

	unsigned int layerIndex = 0;

	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{

		std::list<neuronDescriptor>::iterator neuron;

		// iterate through all and flag them false
		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
			neuron->biasNeuron = false;
		}

		// on every layer except the last, flag the last neuron as bias.
		if ( layerIndex < (network->layers.size() -1 )) {

			neuron = layer->neurons.end();
			neuron--;
			neuron->biasNeuron = true;
		}

		layerIndex++;
	}
}



void loadFish (fishDescriptor_t driedFish, fann * nann, b2Vec2 startingPosition, std::list<Species>::iterator currentSpecies ) {


// std::list<Species>::iterator currentSpecies = ecosystem.begin(); 
				// currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
		// {
			// std::list<BonyFish>::iterator fish;
			// for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
			// {

	// printf("gingethrine %lu\n", currentSpecies->population.size() );


	BonyFish newFish = *(new BonyFish(driedFish, nann, startingPosition));


	// printf("sonos\n");

	currentSpecies->population.push_back(  newFish );

	// printf("mout loadde\n");

	BonyFish * fish = &(currentSpecies->population.back());
	fish->isUsed = true;

	for (unsigned int i = 0; i < N_FINGERS; ++i) {
		if (driedFish.bones[i].used) {

			if( driedFish.bones[i].isRoot ) {
				nonRecursiveBoneIncorporator( fish->bones[i]);
				// fish->bones[i]->hasGrown = true;
				
			}
			else {
				if ( !(driedFish.bones[i].isLeaf) ){

			
				nonRecursiveBoneIncorporator( fish->bones[i]);
				}
				// if ( (driedFish.bones[i].isLeaf) ){

			
				// // nonRecursiveBoneIncorporator( fish->bones[i]);
				// 	fish->bones[i]->hasGrown = false;
				// }

			}

		

			
		}
		fish->bones[i]->p_owner = fish; // you need to update the user data pointer, because when you pushed the fish onto the list you pushed a copy of it not the actual thing.
	}


	// printf("mesines\n");

	flagBiasNeurons(fish->brain);
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
	// n_inputs = 0;
	isUsed = false;
	aabb.upperBound = b2Vec2(0.0f,0.0f);
	aabb.lowerBound = b2Vec2(0.0f,0.0f);
	selected = false;
}

layerDescriptor::layerDescriptor () {
	// n_neurons = 0;
	isUsed = false;
	selected = false;
}

senseConnector::senseConnector () {
	connectedToLimb = 0;							// what limb the sense is coming from, or motor signal is going to.
	connectedToNeuron = 0;							// neuron index. The position of this neuron's layer will determine how the program uses it.
	sensorType =  SENSECONNECTOR_UNUSED; 			// what kind of sense it is (touch, smell, etc.how the number will be treated)
	timerFreq = 0;									// if a timer, the frequency.
	recursorChannel = 0; 		// 
	recursorDelay = 0;
	recursorCursor = 0;

	eyeFOV = 0.5 * pi;
	eyeColor = b2Color(0.0f, 1.0f, 0.0f); // default eye color is green for seeing food

	for (int i = 0; i < SENSECONNECTOR_BUFFERSIZE; ++i)
	{
		recursorBuffer[i] = 0.0f;
	}

}


fishDescriptor_t::fishDescriptor_t () {				//initializes the blank fish as the 4 segment nematode creature.

	for (unsigned int i = 0; i < N_SENSECONNECTORS; ++i) {
		// senseConnector moshuns = senseConnector();
		outputMatrix[i] = senseConnector();
		inputMatrix[i] = senseConnector();
	}

	for (unsigned int i = 0; i < N_FINGERS; ++i) {

		bones[i] = boneAndJointDescriptor_t();
	
		bones[i].color.r = 0.5f;
		bones[i].color.g = 0.5f;
		bones[i].color.b = 0.5f;

		bones[i].length = 0.5f;
		bones[i].rootThickness = 0.2f;
		bones[i].tipThickness = 0.2f;

		if (i == 0) {
			bones[i].isRoot = true;
			// bones[i].isMouth = true;
			bones[i].attachedTo = 0;
		}
		else {
			bones[i].attachedTo = i-1;
		}

		// if (i < 4) {
		// 	bones[i].used = true;
		// }
		// else {
			bones[i].used = false;
		// }
	}

	// flagDelete = false;

	// unsigned int j = 0;

	// for (unsigned int i = 0; i < N_SENSECONNECTORS; ++i) {
	// // 	outputMatrix[j].connectedToLimb = i;
	// // 	outputMatrix[j].sensorType = SENSECONNECTOR_MOTOR;
	// // 	j++;
	// }

	// 	unsigned int channel = 0;
	// for (unsigned int i = 0; i < numberOfRecursorsToStartWith; ++i)
	// {
	// 	for (unsigned int k = 0; k < numberOfEachRecursor; ++k)
	// 	{
	// 		outputMatrix[j].sensorType = SENSECONNECTOR_RECURSORTRANSMITTER;

	// 	outputMatrix[j].recursorChannel = channel;
	// 	outputMatrix[j].recursorDelay = (i*i )* 8;

	// 	channel++;
	// 	j++;
	// 	}
		
	// }

	// j = 0; // reset for next matrix.

	// for (unsigned int i = 0; i < 4; ++i) {
	// 	// inputMatrix[j].connectedToLimb = i;
	// 	// inputMatrix[j].sensorType = SENSOR_FOODRADAR;
	// 	// j++;
	// 	inputMatrix[j].connectedToLimb = i;
	// 	inputMatrix[j].sensorType = SENSOR_JOINTANGLE;
	// 	j++;
	// }

	// // a wide range of timing options is most helpful to the creature.

	// // float wholeFishTimerPhase = RNG();

	// for (unsigned int i = 0; i < 4; ++i) {
	// 	inputMatrix[j].connectedToLimb = 0;
	// 	inputMatrix[j].sensorType = SENSOR_TIMER;
	// 	switch (i){
	// 		case 0:
	// 		inputMatrix[j].timerFreq = 4;
	// 		break;
	// 		case 1:
	// 		inputMatrix[j].timerFreq = 12;
	// 		break;
	// 		case 2:
	// 		inputMatrix[j].timerFreq = 36;
	// 		break;
	// 		case 3:
	// 		inputMatrix[j].timerFreq = 64;
	// 		break;
	// 	}

	// 	inputMatrix[j].timerPhase = 0.0f;//wholeFishTimerPhase;//RNG();
	// 	j++;
	// }

	//  channel = 0;
	// for (unsigned int i = 0; i < numberOfRecursorsToStartWith; ++i)
	// {
	// 	for (unsigned int k = 0; k < numberOfEachRecursor; ++k)
	// 	{
	// 			inputMatrix[j].sensorType = SENSECONNECTOR_RECURSORRECEIVER;

	// 	inputMatrix[j].recursorChannel = channel;
	// 	inputMatrix[j].recursorDelay = (i*i )* 8;

	// 	channel++;
	// 	j++;
	// 	}
	
	// }

	// while (j < N_SENSECONNECTORS) {
	// 	inputMatrix[j].sensorType = SENSECONNECTOR_UNUSED;
	// 	j++;
	// }
}


fishDescriptor_t nematode () {
	fishDescriptor_t driedFish  =  fishDescriptor_t();

	unsigned int numberOfRecursorsToStartWith = 3;
	unsigned int numberOfEachRecursor = 2;

	for (unsigned int i = 0; i < N_FINGERS; ++i) {


	if (i < 4) {
			driedFish.bones[i].used = true;
		}
	}

	driedFish.bones[0].isMouth = true;

	unsigned int j = 0;

	for (unsigned int i = 0; i < 4; ++i) {
		driedFish.outputMatrix[j].connectedToLimb = i;
		driedFish.outputMatrix[j].sensorType = SENSECONNECTOR_MOTOR;
		j++;
	}

		unsigned int channel = 0;
	for (unsigned int i = 0; i < numberOfRecursorsToStartWith; ++i)
	{
		for (unsigned int k = 0; k < numberOfEachRecursor; ++k)
		{
			driedFish.outputMatrix[j].sensorType = SENSECONNECTOR_RECURSORTRANSMITTER;

		driedFish.outputMatrix[j].recursorChannel = channel;
		driedFish.outputMatrix[j].recursorDelay = (i*i )* 8;

		channel++;
		j++;
		}
		
	}

	j = 0; // reset for next matrix.

	for (unsigned int i = 0; i < 4; ++i) {
		// inputMatrix[j].connectedToLimb = i;
		// inputMatrix[j].sensorType = SENSOR_FOODRADAR;
		// j++;
		driedFish.inputMatrix[j].connectedToLimb = i;
		driedFish.inputMatrix[j].sensorType = SENSOR_JOINTANGLE;
		j++;
	}

	// a wide range of timing options is most helpful to the creature.

	// float wholeFishTimerPhase = RNG();

	for (unsigned int i = 0; i < 4; ++i) {
		driedFish.inputMatrix[j].connectedToLimb = 0;
		driedFish.inputMatrix[j].sensorType = SENSOR_TIMER;
		switch (i){
			case 0:
			driedFish.inputMatrix[j].timerFreq = 4;
			break;
			case 1:
			driedFish.inputMatrix[j].timerFreq = 12;
			break;
			case 2:
			driedFish.inputMatrix[j].timerFreq = 36;
			break;
			case 3:
			driedFish.inputMatrix[j].timerFreq = 64;
			break;
		}

		driedFish.inputMatrix[j].timerPhase = 0.0f;//wholeFishTimerPhase;//RNG();
		j++;
	}

	 channel = 0;
	for (unsigned int i = 0; i < numberOfRecursorsToStartWith; ++i)
	{
		for (unsigned int k = 0; k < numberOfEachRecursor; ++k)
		{
				driedFish.inputMatrix[j].sensorType = SENSECONNECTOR_RECURSORRECEIVER;

		driedFish.inputMatrix[j].recursorChannel = channel;
		driedFish.inputMatrix[j].recursorDelay = (i*i )* 8;

		channel++;
		j++;
		}
	
	}

	while (j < N_SENSECONNECTORS) {
		driedFish.inputMatrix[j].sensorType = SENSECONNECTOR_UNUSED;
		j++;
	}


	return driedFish;
}

fishDescriptor_t  basicPlant() {

	fishDescriptor_t newPlant = fishDescriptor_t();


	for (unsigned int i = 0; i < 4; ++i)
	{

		if (i > 0) {
			newPlant.bones[i].attachedTo = i-1;
		}

		newPlant.bones[i].used = true;	
	// 	newPlant.bones[i].used = true;
		newPlant.bones[i].isLeaf = true;
	// 	// newPlant->bones[i].sensor_touch = false;
	// 	// newPlant->bones[i].isMouth = false;
		newPlant.bones[i].color = b2Color(0.1f, 1.0f, 0.3f);

		newPlant.bones[i].isFood = true;

	}

	unsigned int j = 0;

	for (unsigned int i = 0; i < 4; ++i) {
		newPlant.outputMatrix[j].connectedToLimb = i;
		newPlant.outputMatrix[j].sensorType = SENSECONNECTOR_MOTOR;
		j++;
	}

	j = 0; // reset for next matrix.

	for (unsigned int i = 0; i < 4; ++i) {
		// inputMatrix[j].connectedToLimb = i;
		// inputMatrix[j].sensorType = SENSOR_FOODRADAR;
		// j++;
		newPlant.inputMatrix[j].connectedToLimb = i;
		newPlant.inputMatrix[j].sensorType = SENSOR_JOINTANGLE;
		j++;
	}

	
		// newPlant.bones[0].isLeaf = false;

	return newPlant;
}

fishDescriptor_t  fishFoodFlake() {

	fishDescriptor_t newPlant = fishDescriptor_t();


	// for (unsigned int i = 0; i < 4; ++i)
	// {

		// if (i > 0) {
			// newPlant.bones[i].attachedTo = i-1;
		// }

		newPlant.bones[0].used = true;	
	// 	newPlant.bones[i].used = true;
		newPlant.bones[0].isLeaf = false;
	// 	// newPlant->bones[i].sensor_touch = false;
	// 	// newPlant->bones[i].isMouth = false;
		newPlant.bones[0].color = b2Color(0.5f, 0.4f, 0.3f);

		newPlant.bones[0].isFood = true;

	// }

	// unsigned int j = 0;

	// for (unsigned int i = 0; i < 4; ++i) {
		newPlant.outputMatrix[0].connectedToLimb = 0;
		newPlant.outputMatrix[0].sensorType = SENSECONNECTOR_MOTOR;
	// 	j++;
	// }

	// j = 0; // reset for next matrix.

	// for (unsigned int i = 0; i < 4; ++i) {
		newPlant.inputMatrix[0].connectedToLimb = 0;
		// inputMatrix[j].sensorType = SENSOR_FOODRADAR;
	// 	// j++;
	// 	newPlant.inputMatrix[j].connectedToLimb = i;
		newPlant.inputMatrix[0].sensorType = SENSOR_JOINTANGLE;
	// 	j++;
	// }

	
		// newPlant.bones[0].isLeaf = false;

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
  			// neuron.n_inputs = 0; 
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
  			// neuron->n_inputs = 0; 
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
  			
		// getNeuronByIndex(this, con[c].to_neuron)->n_inputs++;
	}				
}


Species::Species () {
	population = *(new std::list<BonyFish>);
	name = std::string("unnamed_species");
	nominalPopulation = 1;
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
		creationLayerCake[layerIndex] = 0;
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

	unsigned int connectionIndex = 0;

	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{
		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 			std::list<connectionDescriptor>::iterator connection;
 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {
 				fann_connection conc ;
 				conc.from_neuron = neuron->index;
 				conc.to_neuron = connection->connectedTo;
 				conc.weight = connection->connectionWeight;
 				margles[connectionIndex] = conc;
				connectionIndex++;
			}
		}
	}

	fann_set_weight_array(ann, margles, num_connections);
	return ann;
}

// if that neuron has a sense connector associated with it, undo the sense connector.
void deleteSenseConnector(int arg) {

std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected ){

				std::list<layerDescriptor>::iterator layer;

				unsigned int layerIndex = 0;

				for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{

					std::list<neuronDescriptor>::iterator neuron;
					unsigned int neuronIndexInThisLayer = 0;
	 				for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {

	 					if (neuron -> selected) {

	 						// if the neuron is on the first or last layer, delete any associated sense connector.
	 						if (layerIndex == 0 ) {	
	 							fish->inputMatrix[neuronIndexInThisLayer].sensorType = SENSECONNECTOR_UNUSED; 
	 						}

	 						if ( layerIndex == fish->brain->layers.size() -1) {
 								fish->outputMatrix[neuronIndexInThisLayer].sensorType = SENSECONNECTOR_UNUSED; 
	 						}

	 						// the fish's input and output matrix configurations should be passed on to offspring
							for (int i = 0; i < N_SENSECONNECTORS; ++i)
							{
								fish->genes.inputMatrix[i] = fish->inputMatrix[i];
								fish->genes.outputMatrix[i] = fish->outputMatrix[i];
							}

	 						return;
	 					}
	 					neuronIndexInThisLayer++;
	 				}
	 				layerIndex++;
				}
			}
		}
	}
}

void deleteSelectedNeuron (int arg) {
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected ){

				std::list<layerDescriptor>::iterator layer;

				unsigned int layerIndex = 0;

				for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{

					std::list<neuronDescriptor>::iterator neuron;
					unsigned int neuronIndexInThisLayer = 0;
	 				for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {

	 					if (neuron -> selected) {
	 						deleteNeuronByIndex(fish->brain, neuron->index);

	 						// if the neuron is on the first or last layer, delete any associated sense connector.
	 						if (layerIndex == 0 ) {	

	 							//now you have to decrement the positions of all other sense connectors.
		 						for (unsigned int i = neuronIndexInThisLayer; i < N_SENSECONNECTORS; i++)
		 						{
		 							// fish->inputMatrix[i] = fish->inputMatrix[i+1];
		 							if (i == N_SENSECONNECTORS-1) {
		 								fish->inputMatrix[i].sensorType = SENSECONNECTOR_UNUSED; //= fish->inputMatrix[i+1];
		 							} 
		 							else {
		 								fish->inputMatrix[i] = fish->inputMatrix[i+1];
		 							}
		 						}
	 						}

	 						if ( layerIndex == fish->brain->layers.size() -1) {

 								//now you have to decrement the positions of all other sense connectors.
		 						for (unsigned int i = neuronIndexInThisLayer; i < N_SENSECONNECTORS; i++)
		 						{
		 							if (i == N_SENSECONNECTORS-1) {
		 								fish->outputMatrix[i].sensorType = SENSECONNECTOR_UNUSED; //= fish->inputMatrix[i+1];
		 							} 
		 							else {
		 								fish->outputMatrix[i] = fish->outputMatrix[i+1];
		 							}
		 						}
	 						}

	 						fish->ann = createFANNbrainFromDescriptor (fish->brain) ;

	 						return;
	 					}
	 					neuronIndexInThisLayer++;
	 				}
	 				layerIndex++;
				}
			}
		}
	}
}

void addNeuronIntoLivingBrain (networkDescriptor * network, unsigned int targetLayerIndex, bool bias) {
	neuronDescriptor * newNeuron = new neuronDescriptor();
	newNeuron->isUsed = true;
	newNeuron->biasNeuron = false;
	newNeuron->index = 0;
	newNeuron->position = b2Vec2(0.0f, 0.0f);

	std::list<layerDescriptor>::iterator layer;
	std::list<layerDescriptor>::iterator targetLayerIterator;
	std::list<neuronDescriptor>::iterator neuron;

	unsigned int layerIndex = 0;

	// set the target layer iterator, so you can refer back to it later
	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer)  {
		newNeuron->index += layer->neurons.size(); // always adding new neuron at the end of the layer.

		if (layerIndex == targetLayerIndex) {
			targetLayerIterator = layer;	
			break;
		}
		
		layerIndex++;
	}

	// if the neuron is not on the last layer, due to the presence of a bias neuron on the target layer, decrement index by 1.
	if (! (targetLayerIndex == network->layers.size()-1) ){
		newNeuron->index--;
	}

	 // all indexes in the connection map greater than the index of this neuron are incremented by 1.
	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 			if (neuron->index >= newNeuron->index) {
 				neuron->index ++;
 			}

 			std::list<connectionDescriptor>::iterator connection;
 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {
 				if (connection->connectedTo > newNeuron->index) {
 					connection->connectedTo ++;
 				}
 			}
 		}
 	}	

 	// make connections for all the next-layer neurons, set to 0, add them to the new neuron
 	if (targetLayerIndex != (network->layers.size() -1 ) ) { // if this isn't the last layer
 		layer= targetLayerIterator;
 		layer++;

		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {

			if ( !(neuron->biasNeuron)) {
					unsigned int targetIndex = neuron->index;
 				connectionDescriptor * newConnection = new connectionDescriptor(   targetIndex);
 				newConnection->isUsed = true;
 				newConnection->connectionWeight = 0.0f;
 				newNeuron->connections.push_back( *newConnection  );
			}
		}
 	}

 	// if the neuron is not on the first layer (and not a bias neuron), all the previous-layer neurons get connections to this neuron.
 	if (!bias) {
 		layer = targetLayerIterator;
	 	if (layer != network->layers.begin() ) { // and the next layer is not off the end of the array
			layer--;
			for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
				connectionDescriptor * newConnection = new connectionDescriptor(   newNeuron->index);
				newConnection->isUsed = true;
 				newConnection->connectionWeight = 0.0f;
				neuron->connections.push_back( *newConnection  );
			}
	 	}	
	 	newNeuron ->biasNeuron = false;
 	}
 	else {
 		newNeuron ->biasNeuron = true;
 	}
	
	if (targetLayerIndex == network->layers.size()-1) {
		targetLayerIterator->neurons.push_back( *newNeuron);		// there is no bias neuron on the last layer so you can drop it right at the end.
	}
	else {
		neuron = targetLayerIterator->neurons.end();			// 'end' is actually 1 past the last element, in C++ list syntax. So retract by 1 to get the last element.
		neuron --; 

		targetLayerIterator->neurons.insert( neuron, *newNeuron); 
	}

	flagBiasNeurons(network);
}

void deleteLayer(networkDescriptor * network, unsigned int layerToDelete) {

	// connections from this layer to the next will be automatically destroyed when the neurons are deleted

	std::list<layerDescriptor>::iterator layer;
	std::list<layerDescriptor>::iterator targetLayerIterator;
	std::list<neuronDescriptor>::iterator neuron;


	unsigned int layerIndex = 0;
	unsigned int numberOfNeuronsInLayer = 0;

	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer)  {
		if (layerIndex == layerToDelete) {
			numberOfNeuronsInLayer = layer->neurons.size();
			break;
		}
		layerIndex++;
	}

	network->layers.erase(layer);

	// decrement the indexes of all subsequent neurons and connections by the amount that was removed
	unsigned int actualNeuronIndex = 0;
	layerIndex = 0;
	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer)  {
		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
			if (layerIndex >=layerToDelete ) {

				neuron->index -= numberOfNeuronsInLayer;

	 			std::list<connectionDescriptor>::iterator connection;
	 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {
 					connection->connectedTo -= numberOfNeuronsInLayer;
	 			}
	 		}
			actualNeuronIndex++;
		}
		layerIndex++;
	}


	// go through the preceding layer and make sure all the neurons are connected to the ones in the new layer (it might have changed if the layers were different sizes)
	layerIndex = 0;
	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer)  {

		// scroll ahead 1 layer, find the highest index in it.
		layer++;
		unsigned int maxIndexInNextLayer = 0;
		if (layer != network->layers.end()) {
			neuron = layer->neurons.end();
			neuron--;

			if (layerIndex < network->layers.size()-1) { // not the last layer
				neuron--;
				maxIndexInNextLayer = neuron->index;
			}
		}	
		layer--;

		// if a neuron has connections to an index greater than what is available in the next level, add the spurious connection to a list 
		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 			neuron->connections.remove_if( [maxIndexInNextLayer](connectionDescriptor connection)
 				{
 					if (connection.connectedTo > maxIndexInNextLayer) {
 						return true;
 					}
 					else {
 						return false;
 					}
 				} 
 			);
		}

		layerIndex++;
	}

	
	layerIndex = 0;

	std::list<neuronDescriptor>::iterator neuronB;
	std::list<connectionDescriptor>::iterator connection;

	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer)  {

			for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {

				layer++;
				if (layer != network->layers.end() ){

					// go through the indexes in the layer after this one, make sure none of the neurons on this layer are missing potential connections.
					for ( neuronB = layer->neurons.begin(); neuronB != layer->neurons.end() ; neuronB++) {

						if (neuronB->biasNeuron) {
							continue;
						}

						// check if neuron has a connection to neuronB
						bool connectedToNeuronB = false;
			 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {
			 				if (connection->connectedTo == neuronB->index) {
			 					connectedToNeuronB = true;
			 				}
			 			}

			 			if (!connectedToNeuronB) {
			 				// printf("added connection %u to %u\n", neuron->index, neuronB->index);
			 				connectionDescriptor * newConnection = new connectionDescriptor(   neuronB->index);
							newConnection->isUsed = true;
			 				newConnection->connectionWeight = 0.0f;
							neuron->connections.push_back( *newConnection  );
			 			}
					}
				}
				layer--;
			}
		layerIndex++;
	}
}

void deleteSelectedLayer(int arg) {

	unused_variable((void *) &arg);
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected && TestMain::getBrainWindowStatus()) {
				std::list<layerDescriptor>::iterator layer;
				unsigned int layerIndex = 0;

				for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{

					if (	layer->selected	) {

						 deleteLayer( fish->brain , layerIndex) ;
						 fish->ann = createFANNbrainFromDescriptor(fish->brain);

						return;
					}

					layerIndex ++;
				}
			}
		}
	}
}

void addRecursorPair(int arg) {

	unused_variable((void *) &arg);
	
		std::list<Species>::iterator currentSpecies;
		for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
		{
			std::list<BonyFish>::iterator fish;
			for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
			{

			if (fish->selected && TestMain::getBrainWindowStatus()) {

				unsigned int greatestUsedRecursionChannel = 0;
				unsigned int lowestAvailableRecursionChannel = 0;

				bool channelsFound = false;

				for (unsigned int i = 0; i < N_SENSECONNECTORS; ++i)
				{
					if (fish->inputMatrix[i].sensorType == SENSECONNECTOR_RECURSORRECEIVER || fish->inputMatrix[i].sensorType == SENSECONNECTOR_RECURSORTRANSMITTER) {
						if (fish->inputMatrix[i].recursorChannel > greatestUsedRecursionChannel ) {
							greatestUsedRecursionChannel = fish->inputMatrix[i].recursorChannel;
						}
						channelsFound = true;
					}

					if (fish->outputMatrix[i].sensorType == SENSECONNECTOR_RECURSORTRANSMITTER  || fish->outputMatrix[i].sensorType == SENSECONNECTOR_RECURSORTRANSMITTER) {
						if (fish->outputMatrix[i].recursorChannel > greatestUsedRecursionChannel ) {
							greatestUsedRecursionChannel = fish->outputMatrix[i].recursorChannel;
						}
						channelsFound = true;
					}
				}

				if (channelsFound) {
					lowestAvailableRecursionChannel = greatestUsedRecursionChannel+1;
				}

				for (unsigned int i = 0; i < N_SENSECONNECTORS; ++i)
				{
					if (fish->inputMatrix[i].sensorType == SENSECONNECTOR_UNUSED ) {
						fish->inputMatrix[i].sensorType = SENSECONNECTOR_RECURSORRECEIVER ;
						fish->inputMatrix[i].recursorChannel = lowestAvailableRecursionChannel;
						addNeuronIntoLivingBrain ( fish->brain, 0, false) ;
						fish->ann = createFANNbrainFromDescriptor(fish->brain);

					break;
					}
				}

				for (int i = 0; i < N_SENSECONNECTORS; ++i)
				{
					if (fish->outputMatrix[i].sensorType == SENSECONNECTOR_UNUSED ) {
						fish->outputMatrix[i].sensorType = SENSECONNECTOR_RECURSORTRANSMITTER ;
						fish->outputMatrix[i].recursorChannel = lowestAvailableRecursionChannel;
						addNeuronIntoLivingBrain (fish->brain, fish->brain->layers.size()-1, false ) ;
						fish->ann = createFANNbrainFromDescriptor(fish->brain);

						break;
					}
				}
			}
		}
	}
}


void selectedLimbEye(int arg) {

	unused_variable((void *) &arg);

	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected) 
			{
				if (fish->bones[currentlySelectedLimb]->isUsed) 
				{
					if (fish->bones[currentlySelectedLimb]->sensor_eye) 
					{
						// if the limb is already an eye, go through the input matrix and snip out any eye connector associated with it.
						for (unsigned int j = 0; j < N_SENSECONNECTORS; ++j)
						{
							if (fish->inputMatrix[j].sensorType == SENSOR_EYE) {
								if (fish->inputMatrix[j].connectedToLimb == currentlySelectedLimb) {
									fish->inputMatrix[j].sensorType = SENSECONNECTOR_UNUSED;

									// because you are deleting layer 0 neurons only, the index in the layer is always the total index.
									deleteNeuronByIndex(fish->brain, j);

									//now you have to decrement the positions of all other sense connectors.
				 						for (unsigned int k = j; k < N_SENSECONNECTORS; k++)
				 						{
				 							if (k == N_SENSECONNECTORS-1) {
				 								fish->inputMatrix[k].sensorType = SENSECONNECTOR_UNUSED; 
				 							} 
				 							else {
				 								fish->inputMatrix[k] = fish->inputMatrix[k+1];
				 							}
				 						}

									fish->bones[currentlySelectedLimb]->sensor_eye = false;
									break;
								}
							}
						}
					}
					else {

						// if the limb is not already an eye, add an eye connector to it, and a new neuron if required.
						addNeuronIntoLivingBrain ( fish->brain, 0, false) ;

						for (unsigned int j = 0; j < N_SENSECONNECTORS; ++j)
						{
							if (fish->inputMatrix[j].sensorType == SENSECONNECTOR_UNUSED) {
								fish->inputMatrix[j].connectedToLimb = currentlySelectedLimb;
								fish->inputMatrix[j].sensorType = SENSOR_EYE;

								fish->bones[currentlySelectedLimb]->sensor_eye = true;

								// the fish's input and output matrix configurations should be passed on to offspring
									for (int k = 0; k < N_SENSECONNECTORS; ++k)
									{
										fish->genes.inputMatrix[k] = fish->inputMatrix[k];
										fish->genes.outputMatrix[k] = fish->outputMatrix[k];
									}

								return;
							}
						}
					}	
				}
			}
		}
	}
}

void selectedLimbFoodradar(int arg) {
	unused_variable((void *) &arg);

	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected) 
			{
				if (fish->bones[currentlySelectedLimb]->isUsed) 
				{
					if (fish->bones[currentlySelectedLimb]->sensor_radar) 
					{

						// if the limb is already an eye, go through the input matrix and snip out any eye connector associated with it.
						for (unsigned int j = 0; j < N_SENSECONNECTORS; ++j)
						{
							if (fish->inputMatrix[j].sensorType == SENSOR_FOODRADAR) {
								if (fish->inputMatrix[j].connectedToLimb == currentlySelectedLimb) {
									fish->inputMatrix[j].sensorType = SENSECONNECTOR_UNUSED;

									// because you are deleting layer 0 neurons only, the index in the layer is always the total index.
									deleteNeuronByIndex(fish->brain, j);

									//now you have to decrement the positions of all other sense connectors.
			 						for (unsigned int k = j; k < N_SENSECONNECTORS; k++)
			 						{
			 							if (k == N_SENSECONNECTORS-1) {
			 								fish->inputMatrix[k].sensorType = SENSECONNECTOR_UNUSED; 
			 							} 
			 							else {
			 								fish->inputMatrix[k] = fish->inputMatrix[k+1];
			 							}
			 						}


									fish->bones[currentlySelectedLimb]->sensor_radar = false;
									break;
								}
							}
						}
					}
					else {

						// if the limb is not already an eye, add an eye connector to it, and a new neuron if required.
						addNeuronIntoLivingBrain ( fish->brain, 0, false) ;

						for (unsigned int j = 0; j < N_SENSECONNECTORS; ++j)
						{
							if (fish->inputMatrix[j].sensorType == SENSECONNECTOR_UNUSED) {
								fish->inputMatrix[j].connectedToLimb = currentlySelectedLimb;
								fish->inputMatrix[j].sensorType = SENSOR_FOODRADAR;
								fish->bones[currentlySelectedLimb]->sensor_radar = true;

								// the fish's input and output matrix configurations should be passed on to offspring
									for (int k = 0; k < N_SENSECONNECTORS; ++k)
									{
										fish->genes.inputMatrix[k] = fish->inputMatrix[k];
										fish->genes.outputMatrix[k] = fish->outputMatrix[k];
									}

								return;
							}
						}
					}
				}
			}
		}
	}
}

void selectedLimbAltradar(int arg) {

	unused_variable((void *) &arg);

	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected) 
			{
				if (fish->bones[currentlySelectedLimb]->isUsed) 
				{
					if (fish->bones[currentlySelectedLimb]->sensor_altradar) {

						// if the limb is already an eye, go through the input matrix and snip out any eye connector associated with it.
						for (unsigned int j = 0; j < N_SENSECONNECTORS; ++j)
						{
							if (fish->inputMatrix[j].sensorType == SENSOR_ALTRADAR) {
								if (fish->inputMatrix[j].connectedToLimb == currentlySelectedLimb) {
									fish->inputMatrix[j].sensorType = SENSECONNECTOR_UNUSED;

									// because you are deleting layer 0 neurons only, the index in the layer is always the total index.
									deleteNeuronByIndex(fish->brain, j);

									//now you have to decrement the positions of all other sense connectors.
			 						for (unsigned int k = j; k < N_SENSECONNECTORS; k++)
			 						{
			 							if (k == N_SENSECONNECTORS-1) {
			 								fish->inputMatrix[k].sensorType = SENSECONNECTOR_UNUSED; 
			 							} 
			 							else {
			 								fish->inputMatrix[k] = fish->inputMatrix[k+1];
			 							}
			 						}

									fish->bones[currentlySelectedLimb]->sensor_altradar = false;
									break;
								}
							}
						}
					}
					else {

						// if the limb is not already an eye, add an eye connector to it, and a new neuron if required.
						addNeuronIntoLivingBrain ( fish->brain, 0, false) ;

						for (unsigned int j = 0; j < N_SENSECONNECTORS; ++j)
						{
							if (fish->inputMatrix[j].sensorType == SENSECONNECTOR_UNUSED) {
								fish->inputMatrix[j].connectedToLimb = currentlySelectedLimb;
								fish->inputMatrix[j].sensorType = SENSOR_ALTRADAR;

								
								fish->bones[currentlySelectedLimb]->sensor_altradar = true;


								// update the fish's genes to reflect its current state of being.

								// the fish's input and output matrix configurations should be passed on to offspring
								for (int k = 0; k < N_SENSECONNECTORS; ++k)
								{
									fish->genes.inputMatrix[k] = fish->inputMatrix[k];
									fish->genes.outputMatrix[k] = fish->outputMatrix[k];
								}

								return;
							}
						}
					}
				}
			}
		}
	}
}

void verifyNetworkDescriptor (networkDescriptor * network) {

	std::list<layerDescriptor>::iterator layer;
	unsigned int layerIndex = 0;
	unsigned int neuronIndex = 0;
	unsigned int connectionIndex = 0;

	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{
		printf("	layer %u neurons: %lu\n", layerIndex, (unsigned long)layer->neurons.size());

		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 			printf("		neuron %u connections: %lu bias: %i\n", neuronIndex, (unsigned long)neuron->connections.size(), neuron->biasNeuron);

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

void addLayerIntoLivingBrain(networkDescriptor * network) {

	std::list<layerDescriptor>::iterator layer;

	layerDescriptor * newLayer = new layerDescriptor;
	newLayer ->isUsed = true;
	newLayer ->selected = false;

	network->layers.push_back(*newLayer);

	layer = network->layers.end();
	layer --;
	layer --; // scroll back to the layer before the one you just added

	// add as many neurons as there are ouput connectors
	unsigned int sizeOfPrevLayer = layer->neurons.size();

	for (unsigned int i = 0; i < sizeOfPrevLayer; ++i)
	{
		addNeuronIntoLivingBrain( network, (network->layers.size() -1), false );
	}

	// add a bias neuron to the layer that used to be last. and connect it to all the new neurons.
	addNeuronIntoLivingBrain( network, (network->layers.size() -2), true);

	// fish->ann = createFANNbrainFromDescriptor(network);
}



// add neuron to a selected INTERNAL layer.
void addNeuronInSelectedLayer(int arg) {

	unused_variable((void *) &arg);
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected && TestMain::getBrainWindowStatus()) {
				std::list<layerDescriptor>::iterator layer;
				unsigned int layerIndex = 0;

				for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{

					if (	layer->selected	) {

					
						addNeuronIntoLivingBrain ( fish->brain, layerIndex, false) ;

						return;
					}

					layerIndex ++;
				}
			}
		}
	}
}

void addLayerToSelectedFish(int arg) {
	unused_variable((void *) &arg);
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected && TestMain::getBrainWindowStatus()) {
				addLayerIntoLivingBrain( fish->brain);
				fish->ann = createFANNbrainFromDescriptor(fish->brain);
				return;
			}
		}
	}
}

// add a limb onto the end of the selected one.
void polydactyly2 (BonyFish * fish) {
	uint targetFinger = 0;
	boneAndJointDescriptor_t  boneAlone = boneAndJointDescriptor_t();

	for (unsigned int i = 0; i < N_FINGERS; ++i) {
		if (!fish->bones[i]->isUsed) {
			targetFinger = i;
		}
		if (fish->bones[i]->attachedTo == currentlySelectedLimb) {
			boneAlone = fish->genes.bones[i];
		}
	}

	boneAlone.used = true;
	boneAlone.isLeaf = false;
	boneAlone.sensor_radar = true;
	boneAlone.sensor_jointangle = true;
	boneAlone.attachedTo = currentlySelectedLimb;

	fish->genes.bones[targetFinger] = boneAlone;

	fish->bones[targetFinger] = new BoneUserData(boneAlone, fish, b2Vec2(0.0f, 0.0f), 0, true);

	fish->bones[targetFinger]->joint = new JointUserData(boneAlone, fish->bones[targetFinger], fish);
	// fish->bones[targetFinger]->joint->init = true;
	fish->bones[targetFinger]->joint->isUsed = true;

	nonRecursiveBoneIncorporator(fish->bones[targetFinger]);

	if (boneAlone.sensor_radar) {
		for (int i = 0; i < N_SENSECONNECTORS; ++i)
		{
			if (fish->inputMatrix[i].sensorType == SENSECONNECTOR_UNUSED ) {
				fish->inputMatrix[i].sensorType = SENSOR_FOODRADAR;
				fish->inputMatrix[i].connectedToLimb = targetFinger;
				addNeuronIntoLivingBrain (fish->brain, 0, false) ;
				fish->ann = createFANNbrainFromDescriptor(fish->brain);

				break;
			}
		}
	}

	if (boneAlone.sensor_jointangle) {
		for (int i = 0; i < N_SENSECONNECTORS; ++i)
		{
			if (fish->inputMatrix[i].sensorType == SENSECONNECTOR_UNUSED ) {
				fish->inputMatrix[i].sensorType = SENSOR_JOINTANGLE ;
				fish->inputMatrix[i].connectedToLimb = targetFinger;
				addNeuronIntoLivingBrain (fish->brain, 0, false) ;
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

			addNeuronIntoLivingBrain (fish->brain, fish->brain->layers.size()-1, false ) ;
			fish->ann = createFANNbrainFromDescriptor(fish->brain);

			break;
		}
	}
	
	return;
}

void placeLimbOnSelectedFish(int arg) {

	unused_variable((void *) &arg);
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			// for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{

				if (fish->selected && TestMain::getBodyWindowStatus()) {
					polydactyly2( &(*fish));
					break;
				}
			// }
		}
	}
}

// this function is intended for when a fish eats a limb off of another fish. it should not remove brain material associated with the limb
void amputate (BonyFish * fish, unsigned int i) {


		// std::list<Species>::iterator currentSpecies;
	// for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	// {
	// 	std::list<BonyFish>::iterator fish;
	// 	for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		// {

			// if (fish->selected && TestMain::getBodyWindowStatus()) {
			// for (int i = 0; i < N_FINGERS; ++i)
			// {
				/* code */




			// }
				if (fish->bones[i]->isRoot) { // you can't amputate someone's head... well you can... but..
						// return;
						// continue;
						fish->flagDelete = true;
				}


				// if (fish->bones[i]->flagDelete) {

				



				// for ( int i = 0; i < N_FINGERS; ++i) {
				// 	if (fish->bones[i]->attachedTo == currentlySelectedLimb) {
				// 		fish->bones[i]->flagDelete = true;
				// 		deleteJoint(fish->bones[i]);
				// 		deleteBone(fish->bones[i]);
				// 		fish->genes.bones[i].used = false;
				// 	}
				// }

			 	// fish->bones[i]->flagDelete = true;
				deleteJoint(fish->bones[i]);
				deleteBone(fish->bones[i]);
				// fish->bones[i]->isUsed = true;
				fish->bones[i]->hasGrown = false;

				// list of neuron indexes to remove
				// std::list<unsigned int> neuronsToRemove;    

				// unsigned int retractInputConnectorsThisManyPlaces = 0;
				// unsigned int retractInputConnectorsStartingAt = 0;
				// bool chosenPlaceForInputConnectorRetraction = false;

				// unsigned int retractOutputConnectorsThisManyPlaces = 0;
				// unsigned int retractOutputConnectorsStartingAt = 0;
				// bool chosenPlaceForOutputConnectorRetraction = false;

				// // delete output and input connectors, noting which neurons will be removed.
				// unsigned int currentlySelectedLimbUnsigned = i;
				// for (int j = 0; j < N_SENSECONNECTORS; ++j)
				// {
				// 	if (fish->inputMatrix[j].connectedToLimb == currentlySelectedLimbUnsigned) {
				// 		fish->inputMatrix[j].sensorType = SENSECONNECTOR_UNUSED;

				// 		// turns out the 'connectedToNeuron' parameter isnt even used. So for the input layer can can just use the index
				// 		neuronsToRemove.push_back(j);

				// 		retractInputConnectorsThisManyPlaces++;
				// 		if (!chosenPlaceForInputConnectorRetraction) {
				// 			chosenPlaceForInputConnectorRetraction = true;
				// 			retractInputConnectorsStartingAt = j;
				// 		}
				// 	}

				// 	if (fish->outputMatrix[j].connectedToLimb == currentlySelectedLimbUnsigned) {
				// 		fish->outputMatrix[j].sensorType = SENSECONNECTOR_UNUSED;

				// 		retractOutputConnectorsThisManyPlaces++;
				// 		if (!chosenPlaceForOutputConnectorRetraction) {
				// 			chosenPlaceForOutputConnectorRetraction = true;
				// 			retractOutputConnectorsStartingAt = j;
				// 		}

				// 		std::list<neuronDescriptor>::iterator neuron = fish->brain->layers.back().neurons.begin();
				// 		unsigned int outputLayerStartIndex = neuron->index;
				// 		neuronsToRemove.push_back(outputLayerStartIndex + j);
				// 	}
				// }

				// // push back the input and output connectors by the number of neurons that were removed.
				// for (unsigned int j = 0; j < retractInputConnectorsThisManyPlaces; ++j) {
				// 	for (unsigned int k = retractInputConnectorsStartingAt; k < N_SENSECONNECTORS; ++k)
				// 	{
				// 		if (k == N_SENSECONNECTORS-1) {
				// 			fish->inputMatrix[k].sensorType = SENSECONNECTOR_UNUSED;
				// 		}
				// 		else {
				// 			fish->inputMatrix[k] = fish->inputMatrix[k+1];
				// 		}
				// 	}
				// }

				// for (unsigned int j = 0; j < retractOutputConnectorsThisManyPlaces; ++j) {
				// 	for (unsigned int k = retractOutputConnectorsStartingAt; k < N_SENSECONNECTORS; ++k)
				// 	{
				// 		if (k == N_SENSECONNECTORS-1) {
				// 			fish->outputMatrix[k].sensorType = SENSECONNECTOR_UNUSED;
				// 		}
				// 		else {
				// 			fish->outputMatrix[k] = fish->outputMatrix[k+1];
				// 		}
				// 	}
				// }

				// // re-fry the brain without those connectors.
				// std::list<unsigned int>::iterator neuronsToRemoveIterator;

				// for (neuronsToRemoveIterator = neuronsToRemove.begin(); neuronsToRemoveIterator !=  neuronsToRemove.end(); ++neuronsToRemoveIterator) 	{

				// 	deleteNeuronByIndex (fish->brain, *neuronsToRemoveIterator);

				// 	// if the neuron to be removed is in the output layer, shuffle all the output connectors along by one.
				// 	std::list<unsigned int>::iterator decrementIndexesInListIterator;
				// 	for (decrementIndexesInListIterator = neuronsToRemove.begin(); decrementIndexesInListIterator !=  neuronsToRemove.end(); ++decrementIndexesInListIterator) 	{
				// 	  	if (*decrementIndexesInListIterator > *neuronsToRemoveIterator) {
				// 	  		(*decrementIndexesInListIterator)--;
				// 	  	}
				// 	}
				// }

				// fish->ann = createFANNbrainFromDescriptor(fish->brain);

				// return;
			// }
		// }
	// 	}
	// }



}

// this function is intended for when the user wishes to remove a limb. it removes brain material associated with the limb.
void amputation (int arg) {
	// printf("amputation\n");
	unused_variable((void *) &arg);
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{

			// if (fish->selected && TestMain::getBodyWindowStatus()) {
			for (int i = 0; i < N_FINGERS; ++i)
			{
				/* code */
			// }
				if (fish->bones[i]->isRoot) { // you can't amputate someone's head... well you can... but..
						// return;
						continue;
				}


				if (fish->bones[i]->flagDelete) {

				



				// for ( int i = 0; i < N_FINGERS; ++i) {
				// 	if (fish->bones[i]->attachedTo == currentlySelectedLimb) {
				// 		fish->bones[i]->flagDelete = true;
				// 		deleteJoint(fish->bones[i]);
				// 		deleteBone(fish->bones[i]);
				// 		fish->genes.bones[i].used = false;
				// 	}
				// }

			 	// fish->bones[i]->flagDelete = true;
				deleteJoint(fish->bones[i]);
				deleteBone(fish->bones[i]);
				fish->genes.bones[i].used = false;

				// list of neuron indexes to remove
				std::list<unsigned int> neuronsToRemove;    

				unsigned int retractInputConnectorsThisManyPlaces = 0;
				unsigned int retractInputConnectorsStartingAt = 0;
				bool chosenPlaceForInputConnectorRetraction = false;

				unsigned int retractOutputConnectorsThisManyPlaces = 0;
				unsigned int retractOutputConnectorsStartingAt = 0;
				bool chosenPlaceForOutputConnectorRetraction = false;

				// delete output and input connectors, noting which neurons will be removed.
				unsigned int currentlySelectedLimbUnsigned = i;
				for (int j = 0; j < N_SENSECONNECTORS; ++j)
				{
					if (fish->inputMatrix[j].connectedToLimb == currentlySelectedLimbUnsigned) {
						fish->inputMatrix[j].sensorType = SENSECONNECTOR_UNUSED;

						// turns out the 'connectedToNeuron' parameter isnt even used. So for the input layer can can just use the index
						neuronsToRemove.push_back(j);

						retractInputConnectorsThisManyPlaces++;
						if (!chosenPlaceForInputConnectorRetraction) {
							chosenPlaceForInputConnectorRetraction = true;
							retractInputConnectorsStartingAt = j;
						}
					}

					if (fish->outputMatrix[j].connectedToLimb == currentlySelectedLimbUnsigned) {
						fish->outputMatrix[j].sensorType = SENSECONNECTOR_UNUSED;

						retractOutputConnectorsThisManyPlaces++;
						if (!chosenPlaceForOutputConnectorRetraction) {
							chosenPlaceForOutputConnectorRetraction = true;
							retractOutputConnectorsStartingAt = j;
						}

						std::list<neuronDescriptor>::iterator neuron = fish->brain->layers.back().neurons.begin();
						unsigned int outputLayerStartIndex = neuron->index;
						neuronsToRemove.push_back(outputLayerStartIndex + j);
					}
				}

				// push back the input and output connectors by the number of neurons that were removed.
				for (unsigned int j = 0; j < retractInputConnectorsThisManyPlaces; ++j) {
					for (unsigned int k = retractInputConnectorsStartingAt; k < N_SENSECONNECTORS; ++k)
					{
						if (k == N_SENSECONNECTORS-1) {
							fish->inputMatrix[k].sensorType = SENSECONNECTOR_UNUSED;
						}
						else {
							fish->inputMatrix[k] = fish->inputMatrix[k+1];
						}
					}
				}

				for (unsigned int j = 0; j < retractOutputConnectorsThisManyPlaces; ++j) {
					for (unsigned int k = retractOutputConnectorsStartingAt; k < N_SENSECONNECTORS; ++k)
					{
						if (k == N_SENSECONNECTORS-1) {
							fish->outputMatrix[k].sensorType = SENSECONNECTOR_UNUSED;
						}
						else {
							fish->outputMatrix[k] = fish->outputMatrix[k+1];
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

				fish->ann = createFANNbrainFromDescriptor(fish->brain);

				// return;
			}
		}
		}
	}
}


void mutateFishDescriptor (fishDescriptor_t * fish, float mutationChance, float mutationSeverity) {


	/*

	

	how should mutation randomness be calculated?

	for every copy operation on every parameter, this is the chance of messing it up.






	*/


	// int intMutationSeverity = mutationSeverity;

	for (int i = 0; i < N_SENSECONNECTORS; ++i)
	{
		if (RNG() < mutationChance) {
			if (fish->inputMatrix[i].sensorType == SENSECONNECTOR_RECURSORRECEIVER) {

				if (RNG() > 0.5) {
					fish->inputMatrix[i].recursorDelay += (mutationSeverity * 10 * RNG()) + 1 ;


				}
				else {
					fish->inputMatrix[i].recursorDelay -= (mutationSeverity * 10 * RNG()) + 1;
				}


			}
			if (fish->inputMatrix[i].sensorType == SENSOR_TIMER) {

				// randomly decrement or increment.
				if (RNG() > 0.5) {
					fish->inputMatrix[i].timerFreq += (mutationSeverity * 10 * RNG()) + 1 ;
				}
				else {
					fish->inputMatrix[i].timerFreq -= (mutationSeverity * 10 * RNG()) + 1;
				}

			}
		}

		if (fish->inputMatrix[i].recursorDelay > SENSECONNECTOR_BUFFERSIZE) {
			fish->inputMatrix[i].recursorDelay = SENSECONNECTOR_BUFFERSIZE;
				
		}
		if (fish->inputMatrix[i].recursorDelay < 0) {
			fish->inputMatrix[i].recursorDelay = 0;
		}

	
	}


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

// this function does the legwork of actually erasing stuff from the world. it is done outside of the step.
void removeDeletableFish() {
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{

		// std::list<BonyFish> fishToDelete ;//= *(new std::list<BonyFish>() );

		// first make a separate list of fish to remove, then take them out of the original list.

		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->flagDelete && fish->isUsed) {
				deleteFish ( &(*fish)) ;
				currentSpecies->population.erase(fish++);
				// fishToDelete.push_back( (*fish) );
			}
		}


		// for (fish = fishToDelete.begin(); fish !=  fishToDelete.end(); ++fish) 	
		// {
		// 	// if (fish->flagDelete && fish->isUsed) {
		// 	// 	deleteFish ( &(*fish)) ;
		// 		currentSpecies->population.remove( (*fish) );
		// 		// fishToDelete.push_back( *(&fish) );
		// 	// }
		// }




	}
	// for (int i = 0; i < N_FOODPARTICLES; ++i) {
	// 	if (food[i]->flagDelete) {
	// 		deleteBone(food[i]);
	// 	}
	// }
}

// this function just sets the flags and labels. it is done inside the step.
void reloadTheSim  (int arg) {
	unused_variable((void *) &arg);
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			fish->flagDelete = true;
		}
	}		

	// if (!TestMain::getPersistentFoodStatus()) {
	// 	for (int i = 0; i < N_FOODPARTICLES; ++i) {
	// 		food[i]->flagDelete = true;
	// 	}
	// }
	
	startNextGeneration = true;
}

//  prints the winner to file immediately.
void  vote (BonyFish * winner) {
	if ( loopCounter > loopSafetyLimit) {

		// the fish's input and output matrix configurations should be passed on to offspring
	for (int i = 0; i < N_SENSECONNECTORS; ++i)
	{
		winner->genes.inputMatrix[i] = winner->inputMatrix[i];
		winner->genes.outputMatrix[i] = winner->outputMatrix[i];
	}

		fann * wann = winner->ann;

		// save the winner to file with a new name.
		std::string nnfilename =  std::string("child.net");
	    std::string fdescfilename =  std::string("child.fsh");
	    std::ofstream file { nnfilename };
	    saveFishToFile (fdescfilename, winner->genes);
	    fann_save(  wann , nnfilename.c_str()); 

		reloadTheSim(0);	
	}
}

void mutateFANNFileDirectly (std::string filename) {

	std::ofstream outFile("mutant.net");
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
					    		

					    		val += ( (RNG() -0.5f) * abs(val) * m_deepSeaSettings.mentalMutationSeverity);
						    	// val += (  RNG() - 0.5f) * mentalMutationSeverity;	


					    	}	
					    	else {


					    		val += ( (RNG() -0.5f)  * m_deepSeaSettings.mentalMutationSeverity ); 	// part of the mutation is applied as a multiplication of the existing number
					    		// val += (  RNG() - 0.5f) * mentalMutationSeverity;																	// part of the mutation is applied as a new addition

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

networkDescriptor * combineTwoNetworks2 (networkDescriptor * partnerA, networkDescriptor * partnerB) {

	// copies are made of both parents.
	// blank neurons are added to the copies to make them the same shape.
	// then, connections can be chosen at random without having to compute anything special.

	networkDescriptor copyParentA = *partnerA;
	networkDescriptor copyParentB = *partnerB;
	
	
	// first, find the tallest network.
	unsigned int layersA = copyParentA.layers.size();
	unsigned int layersB = copyParentB.layers.size();
	unsigned int tallestPartner;

	if (layersA > layersB) {
		tallestPartner = layersA;
		for (unsigned int i = 0; i < (layersA - layersB); ++i)
		{
			addLayerIntoLivingBrain(&copyParentB);
		}
	}
	else if (layersB > layersA) {
		tallestPartner = layersB;
		for (unsigned int i = 0; i < (layersB - layersA); ++i)
		{
			addLayerIntoLivingBrain(&copyParentA);
		}
	}
	else {
		// they are the same!
		tallestPartner = layersA;
	}

	// now go through them layer at a time.
	std::list<layerDescriptor>::iterator layerA = copyParentA.layers.begin();
	std::list<layerDescriptor>::iterator layerB = copyParentB.layers.begin();

	for (unsigned int i = 0; i < tallestPartner; ++i)
	{


		// find which is the widest.
		// unsigned int widestPartner;

		unsigned int widthA = layerA->neurons.size();
		unsigned int widthB = layerB->neurons.size();

		// add enough neurons to the other that makes them the same.
		if (widthA > widthB) {
			// widestPartner = widthA;
			for (unsigned int j = 0; j < (widthA - widthB); ++j)
			{
				addNeuronIntoLivingBrain(&copyParentB, i, false);
			}
		}
		else if (widthB > widthA) {
			// widestPartner = widthB;
			for (unsigned int j = 0; j < (widthB - widthA); ++j)
			{
				addNeuronIntoLivingBrain(&copyParentA, i, false);
			}
		}
		else {
			// they are the same! do nothing.
		}
		
		layerA++;
		layerB++;

	}

	// congrats, the network copies are now exactly the same shape.
	// duplicate one of them to create the child.
	fann * wann = createFANNbrainFromDescriptor(&copyParentA);
	networkDescriptor * child = new networkDescriptor(wann);

	layerA = copyParentA.layers.begin();
	layerB = copyParentB.layers.begin();
	std::list<layerDescriptor>::iterator layerC = child->layers.begin();

	std::list<neuronDescriptor>::iterator neuronA; 
	std::list<neuronDescriptor>::iterator neuronB ;
	std::list<neuronDescriptor>::iterator neuronC ;

	std::list<connectionDescriptor>::iterator connectionA;
	std::list<connectionDescriptor>::iterator connectionB;
	std::list<connectionDescriptor>::iterator connectionC;

	for (unsigned int i = 0; i < tallestPartner; ++i)
	{
		
		neuronA = layerA->neurons.begin();
		neuronB = layerB->neurons.begin();
		neuronC = layerC->neurons.begin();

		for (unsigned int j = 0; j < layerA->neurons.size(); ++j)
		{
						
			connectionA = neuronA->connections.begin();
			connectionB = neuronB->connections.begin();
			connectionC = neuronC->connections.begin();

			for (unsigned int k = 0; k < neuronA->connections.size(); ++k)
			{
				

				// printf("connection A, B. %u %u %f %f\n", connectionA->connectedTo, connectionB->connectedTo, connectionA->connectionWeight, connectionB->connectionWeight);


				if (connectionA->connectionWeight != 0.0f && connectionB->connectionWeight !=  0.0f) {

					if (RNG() > 0.5) { // if they are both nonzero, choose one at random
						connectionC -> connectionWeight = connectionA->connectionWeight ;
					}
					else {
						connectionC -> connectionWeight = connectionB->connectionWeight ;
					}

				}
				else if (connectionA->connectionWeight !=  0.0f) {
					connectionC -> connectionWeight = connectionA->connectionWeight ;

				}
				else if (connectionB->connectionWeight !=  0.0f) {
					connectionC -> connectionWeight = connectionB->connectionWeight ;

				}
				else {
					// the weight will be 0

				}


				connectionA++;
				connectionB++;
				connectionC++;
			}


			neuronA++;
			neuronB++;
			neuronC++;
		}


		layerA++;
		layerB++;
		layerC++;
	}


	return child;

}


// combines two neuro descriptors to produce an offspring with a random mix of traits
void sex (BonyFish * partnerA, BonyFish * partnerB, std::list<Species>::iterator currentSpecies) {
	

	// networkDescriptor child = new networkDescriptor;
	// child = partnerA;


	fishDescriptor_t * childDescriptor = new fishDescriptor_t;

	for (int i = 0; i < N_SENSECONNECTORS; ++i)
	{

		// input connectors
		bool connectorPresentA = true;
		bool connectorPresentB = true;

		if (partnerA->inputMatrix[i].sensorType == SENSECONNECTOR_UNUSED ) { connectorPresentA = false;}
		if (partnerB->inputMatrix[i].sensorType == SENSECONNECTOR_UNUSED ) { connectorPresentB = false;}

		if (connectorPresentA && !(connectorPresentB) ) {childDescriptor->inputMatrix[i] = partnerA->inputMatrix[i]; }
		else if (connectorPresentB && !(connectorPresentA) ) {childDescriptor->inputMatrix[i] = partnerB->inputMatrix[i]; }
		else if (connectorPresentA && connectorPresentB) 
		{  
			if (RNG() > 0.5) {
				childDescriptor->inputMatrix[i] = partnerA->inputMatrix[i];
			}
			else {
				childDescriptor->inputMatrix[i] = partnerB->inputMatrix[i];
			}
		} 

		// output connectors
		 connectorPresentA = true;
		 connectorPresentB = true;

		if (partnerA->outputMatrix[i].sensorType == SENSECONNECTOR_UNUSED ) { connectorPresentA = false;}
		if (partnerB->outputMatrix[i].sensorType == SENSECONNECTOR_UNUSED ) { connectorPresentB = false;}

		if (connectorPresentA && !(connectorPresentB) ) {childDescriptor->outputMatrix[i] = partnerA->outputMatrix[i]; }
		else if (connectorPresentB && !(connectorPresentA) ) {childDescriptor->outputMatrix[i] = partnerB->outputMatrix[i]; }
		else if (connectorPresentA && connectorPresentB) 
		{  
			if (RNG() > 0.5) {
				childDescriptor->outputMatrix[i] = partnerA->outputMatrix[i];
			}
			else {
				childDescriptor->outputMatrix[i] = partnerB->outputMatrix[i];
			}
		} 

	}


	// bones
	for (int i = 0; i < N_FINGERS; ++i)
	{
		bool bonePresentA = false;
		bool bonePresentB = false;

		if (partnerA->bones[i]->isUsed ) { bonePresentA = true;}
		if (partnerB->bones[i]->isUsed ) { bonePresentA = true;}

		if (bonePresentA && !(bonePresentB) ) { childDescriptor->bones[i] = partnerA->genes.bones[i]; }
		else if (bonePresentB && !(bonePresentA) ) { childDescriptor->bones[i] = partnerB->genes.bones[i]; }
		else if (bonePresentA && bonePresentB) {
			if (RNG() >0.5) {
				childDescriptor->bones[i] = partnerA->genes.bones[i];
			}
			else {
				childDescriptor->bones[i] = partnerB->genes.bones[i];
			}
		}
		
	}

	// brain

	networkDescriptor * childBrain = combineTwoNetworks2 (partnerA->brain, partnerB->brain) ;

	b2Vec2 startingPosition = partnerA->bones[0]->p_body->GetWorldCenter();

	loadFish (*childDescriptor, createFANNbrainFromDescriptor(childBrain), startingPosition, currentSpecies) ;



}

void mateSelectedFish (int arg) {
	BonyFish * partnerA;
	BonyFish * partnerB;

	bool foundPartnerA = false;

	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected) {
				if (foundPartnerA) {
					partnerB = &(*fish);

					sex(partnerA, partnerB, currentSpecies);


						

					return;
				}
				else {
					foundPartnerA = true;
					partnerA = &(*fish);
				}
			}
		}
	}
}

void drawingTest() {
	if (TestMain::getTriggerRadiusStatus()  ) {
		b2Vec2 position = b2Vec2(0.0f, 0.0f);
		b2Color color = b2Color(0.5f, 0.1f, 0.05f);
		local_debugDraw_pointer->DrawCircle(position, m_deepSeaSettings.originTriggerRadius, color);
	}

	if (TestMain::getFoodRadiusStatus()  ) {
		b2Vec2 position = b2Vec2(0.0f, 0.0f);
		b2Color color = b2Color(0.3f, 0.3f, 0.3f);
		local_debugDraw_pointer->DrawCircle(position, m_deepSeaSettings.originFoodRadius, color);
	}

	if (TestMain::getBarrierRadiusStatus()  ) { // works in both lab and eco modes
		b2Vec2 position = b2Vec2(0.0f, 0.0f);
		b2Color color = b2Color(0.25f, 0.5f, 0.75f);
		local_debugDraw_pointer->DrawCircle(position, m_deepSeaSettings.barrierRadius, color);
	}



	// draw terrain
	std::list<Terrain>::iterator rock;
	for (rock = environment.begin(); rock !=  environment.end(); ++rock) 	
	{
					b2Vec2 vertices[4];
					b2Vec2 boneCenterWorldPosition = rock->p_body->GetWorldCenter();
					for (int j = 0; j < 4; ++j) {	
						b2Vec2 adjustedVertex = rock->shape.GetVertex(j);
						b2Vec2 boneLocalCenter =rock->p_body->GetLocalCenter();
						b2Vec2 rotatedVertex = rotatePoint( boneLocalCenter.x,boneLocalCenter.y, rock->p_body->GetAngle(), adjustedVertex);
						rotatedVertex.x += boneCenterWorldPosition.x;
						rotatedVertex.y +=boneCenterWorldPosition.y;
						vertices[j] = rotatedVertex;
					}

					local_debugDraw_pointer->DrawFlatPolygon(vertices, 4 , rock->color);	
					local_debugDraw_pointer->DrawPolygon(vertices, 4 , rock->outlineColor);
	}


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
	// b2Color fishFoodDye 		= b2Color(0.2f, 0.6f, 0.1f);
	// b2Color fishFoodDyeOutline 	= b2Color(0.5f, 0.9f, 0.4f);
	// b2Color fishFoodDyeOutlineWhite 	= b2Color(1.0f, 1.0f, 1.0f);

	// for (unsigned int i = 0; i < N_FOODPARTICLES; ++i) {
	// 	if ( food[i]->isUsed) {
	// 		b2Vec2 vertices[4];
	// 		b2Vec2 boneCenterWorldPosition = food[i]->p_body->GetWorldCenter();
	// 		for (int j = 0; j < 4; ++j) {	
	// 			b2Vec2 adjustedVertex = food[i]->shape.GetVertex(j);
	// 			b2Vec2 boneLocalCenter =food[i]->p_body->GetLocalCenter();
	// 			b2Vec2 rotatedVertex = rotatePoint( boneLocalCenter.x,boneLocalCenter.y, food[i]->p_body->GetAngle(), adjustedVertex);
	// 			rotatedVertex.x += boneCenterWorldPosition.x;
	// 			rotatedVertex.y +=boneCenterWorldPosition.y;
	// 			vertices[j] = rotatedVertex;
	// 		}
	// 		local_debugDraw_pointer->DrawFlatPolygon(vertices, 4 , fishFoodDye);

	// 		if (food[i]->selected) {
	// 			local_debugDraw_pointer->DrawPolygon(vertices, 4 , fishFoodDyeOutlineWhite);
	// 		}
	// 		else {
	// 			local_debugDraw_pointer->DrawPolygon(vertices, 4 , fishFoodDyeOutline);
	// 		}
	// 	}
	// }

	// for each bone, get the vertices, then add the body's world location to them, and rotate by the body angle around the body world location.
	// to do this you will need to extend the rotate point method to rotate a polygon.
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			for (int i = 0; i < N_FINGERS; ++i) {
				if (!fish->bones[i]->isUsed) {
					;
				}
				else {
					if (fish->bones[i]->p_body == NULL || fish->bones[i]->p_body == nullptr) {
						continue;
					}
					if (!fish->bones[i]->hasGrown ){
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

	// draw the species window
	if (TestMain::getSpeciesWindowStatus()) {


		// b2Vec2 edgeOfGLUIWindow = ;
		b2Vec2 offset = TestMain::getUpperScreenBoundary();
		float32 zoom = TestMain::getZoom();

		offset = b2Vec2(offset.x - (5 * zoom), offset.y);


		b2Vec2 windowVertices[] = {
			b2Vec2( (+10.0f * zoom) + offset.x, (-100.0f * zoom) ), 
			b2Vec2( (+10.0f * zoom) + offset.x, (+100.0f * zoom) ), 
			b2Vec2( (-2.0f * zoom) + offset.x, (+100.0f * zoom) ), 
			b2Vec2( (-2.0f * zoom) + offset.x, (-100.0f * zoom) )
		};

		local_debugDraw_pointer->DrawFlatPolygon(windowVertices, 4 ,b2Color(0.1,0.1,0.1) );

		// go through the list of species and figure out their window positions.
		unsigned int i = 0;
		for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
		{	
			currentSpecies->windowVertices[0] = b2Vec2((+0.5f * zoom) + offset.x, (-0.5f * zoom) + (i * 3));
			currentSpecies->windowVertices[1] = b2Vec2((+0.5f * zoom) + offset.x, (+0.5f * zoom) + (i * 3)) ;
			currentSpecies->windowVertices[2] = b2Vec2((-0.5f * zoom) + offset.x, (+0.5f * zoom) + (i * 3)); 
			currentSpecies->windowVertices[3] = b2Vec2((-0.5f * zoom) + offset.x, (-0.5f * zoom) + (i * 3));
			i++;
		}

		// now draw them
		for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
		{

			local_debugDraw_pointer->DrawFlatPolygon(currentSpecies->windowVertices, 4 ,b2Color(0.35,0.3,0.1) );

			if (currentSpecies->selected) {
				local_debugDraw_pointer->DrawPolygon(currentSpecies->windowVertices, 4 , b2Color(1,0.8,2));
			}

			// say the species name
			std::string connectorLabel = std::string("");
			connectorLabel +=   currentSpecies->name  ;
			b2Vec2 mocesfef = b2Vec2(currentSpecies->windowVertices[0].x-1.5, currentSpecies->windowVertices[0].y+0.5);
			local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());

			// say the current population
			connectorLabel = std::string("population: ");
			connectorLabel +=  std::to_string(currentSpecies->population.size());
			mocesfef = b2Vec2(currentSpecies->windowVertices[0].x-1.5, currentSpecies->windowVertices[0].y+0.5 -0.5);
			local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
		}
	}
}

void populateSpeciesFromFile(int arg) {
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{	
		if (currentSpecies->selected) {

			for (unsigned int i = 0; i < currentSpecies->nominalPopulation ; ++i)
			{
				bool thereIsAFile = false;
				std::string nnfilename =  std::string("Aquarium/")  + currentSpecies->name + std::string(".net");
			    std::string fdescfilename =  std::string("Aquarium/") + currentSpecies->name + std::string(".fsh");

				if (FILE *file = fopen(nnfilename.c_str(), "r")) {
			        fclose(file);
			        if (FILE *file = fopen(fdescfilename.c_str(), "r")) {
				        thereIsAFile = true;
				        fclose(file);
				    }
			    } 

				if (thereIsAFile ) { // if there is a previous winner, load many of its mutant children

					fishDescriptor_t newFishBody;
					loadFishFromFile(std::string(fdescfilename), newFishBody);

					mutateFishDescriptor (&newFishBody, m_deepSeaSettings.mutationRate, m_deepSeaSettings.mutationSeverity);

				    mutateFANNFileDirectly(std::string(nnfilename));

					// now you can load the mutant ANN.
					fann *mann = loadFishBrainFromFile (std::string("mutant")) ;

					networkDescriptor * muscleCars=  createNeurodescriptorFromFANN (mann) ;

					fann * jann = createFANNbrainFromDescriptor(muscleCars);

					b2Vec2 position = b2Vec2(0.0f,0.0f);

					loadFish ( newFishBody, jann, position, currentSpecies) ;

				}
				// else { 						// if there is no winner, its probably a reset or new install. make one up
				// 	// fishDescriptor_t kargas = nematode();
				// 	// b2Vec2 position = b2Vec2(0.0f,0.0f);
					
				// 	// loadFish ( kargas, NULL,  position, currentSpecies) ;
				// }
			}
				return;
		}
	}
}

void saveIndividualToFile (int arg) {

	// the first selected individual of a particular species is saved to that species' file.
	std::list<Species>::iterator currentSpecies;

	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{	
			
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			
			if (fish->selected ) {

				// the fish's input and output matrix configurations should be passed on to offspring
				for (int i = 0; i < N_SENSECONNECTORS; ++i)
				{
					fish->genes.inputMatrix[i] = fish->inputMatrix[i];
					fish->genes.outputMatrix[i] = fish->outputMatrix[i];
				}

				fann * wann = fish->ann;

				// save the winner to file with a new name.
				std::string nnfilename =  std::string("Aquarium/")  + currentSpecies->name + std::string(".net");
			    std::string fdescfilename =  std::string("Aquarium/") + currentSpecies->name + std::string(".fsh");
			    std::ofstream file { nnfilename };
			    saveFishToFile (fdescfilename, fish->genes);
			    fann_save(  wann , nnfilename.c_str()); 

				return;
			}
		}
	}
}

void selectAllInSpecies (int arg ) {

	std::list<Species>::iterator currentSpecies;

	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{	
		if (currentSpecies->selected) {

			std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			fish->selected = true;
		}

		}
	}
}

void addNewSpecies (int arg) {
	Species * newSpecies = new Species();

	ecosystem.push_back(*newSpecies);
}

void deleteSelectedSpecies (int arg) {
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{	
		if (currentSpecies->selected) {

			deselectAll (0) ;

			selectAllInSpecies(0);

			// first go through and delete all the fish
			flagSelectedFishForDeletion(0) ;

			currentSpecies->flagDelete = true;
		}
	}
}

void deleteFlaggedSpecies() {

	ecosystem.remove_if( [](Species currentSpecies)
 				{
 					return currentSpecies.flagDelete;
 				} 
 			);
}

void mapNameBarCallback(int arg) {
	unused_variable((void *)&arg);
}

void loadSavedMapFromFile(int arg) {
	unused_variable((void *)&arg);
}
void saveCurrentMapToFile(int arg) {
	unused_variable((void *)&arg);
}


void loadExperimentalMap() {

	// populates the terrain list with some stuuuufff

	Terrain * barrierA = new Terrain( b2Vec2(0.0f, -5.0f) );

	b2Vec2 barrierAVertices[] = {
		b2Vec2( + 20,  -0.5), //b2Vec2 rootVertexA = 
		b2Vec2( - 20,  -0.5), // b2Vec2 rootVertexB =
		b2Vec2( + 20,  +0.5), //b2Vec2 tipVertexA = 
		b2Vec2( - 20,  +0.5) // b2Vec2 tipVertexB = 
	};
	
	barrierA->bodyDef.type = b2_staticBody;
	barrierA->p_body = local_m_world->CreateBody( &(barrierA->bodyDef) );
	
	barrierA->shape.Set(barrierAVertices, 4);

	nonRecursiveTerrainIncorporator(barrierA);

	// add to the terrain list also
	environment.push_back( *barrierA );

		// add a lamp
	Lamp monog = Lamp();

	monog.lampType = LAMP_SOLAR;
	monog.direction = pi; // 0 is actually straight up not straight down
	monog.beamWidth = 40;
	monog.brightness = 2;
	monog.position = b2Vec2(0.0f, 5.5f);
	monog.illuminationColor = b2Color( 0.3f, 0.2f ,  0.1f);

	lamps.push_back(monog);

	m_deepSeaSettings.gravity = b2Vec2( 0.0f, -1.0f);

}



void nominalPopulationCallback (int arg) {
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{	
		if (currentSpecies->selected) {
			currentSpecies->nominalPopulation = TestMain::getSpeciesNominalPopulationSpinner() ->get_float_val();
			return;

		}
	}
}

void speciesNameBarCallback(int arg) {
	unused_variable((void *)&arg);
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{	
		if (currentSpecies->selected) {

			currentSpecies->name = (TestMain::getSpeciesNameBar() )->get_text();
			return;
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

		if (myUserData->dataType == TYPE_LEAF) {
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

	sunbeam.maxFraction = 1.0f;

	unsigned int totalBrightness = TestMain::getLampIntensity() * nancy->brightness;

	// cast each ray, more for brighter lamps
	for (unsigned int i = 0; i < totalBrightness; ++i)
	{

		if (nancy->lampType == LAMP_POINTSOURCE) {

			sunbeam.p1 = nancy->position;

			float randomDirection = (RNG() * 2 * pi);

			sunbeam.p2 = b2Vec2(nancy->illuminationRadius * cos(randomDirection),nancy->illuminationRadius* sin(randomDirection));
			

			if (true) { // print the rays of light

			   local_debugDraw_pointer->DrawSegment(sunbeam.p1, sunbeam.p2, nancy->illuminationColor);
			}

			RayCastClosestCallback stupidMotherFucker;

			local_m_world->RayCast( &stupidMotherFucker, sunbeam.p1, sunbeam.p2);
		
		}
		else if (nancy->lampType == LAMP_DIRECTIONAL) {

			// pick a random direction sideways across the beam width
			float sidewaysRandomness = (RNG()-0.5) * nancy->beamWidth ;

			// pick a point that is 'sidewaysRandomness' distance away at right angles to the beam.
			// b2Vec2 photonStartingPosition
			sunbeam.p1 = b2Vec2(  nancy->position.x +  (cos(nancy->direction + (pi*0.5) ) * sidewaysRandomness)  , nancy->position.y +  (sin(nancy->direction + (pi*0.5)) * sidewaysRandomness)    );

			// photon starts from this direction and travels straight
			// b2Vec2 photonEndPosition = 
			sunbeam.p2 = b2Vec2( sunbeam.p1.x +  (cos(nancy->direction ) * nancy->illuminationRadius)  ,  sunbeam.p1.y +  (sin(nancy->direction ) * nancy->illuminationRadius)    );

			if (true) { // print the rays of light

			   local_debugDraw_pointer->DrawSegment(sunbeam.p1, sunbeam.p2,  nancy->illuminationColor );
			}

			RayCastClosestCallback stupidMotherFucker;

			local_m_world->RayCast( &stupidMotherFucker, sunbeam.p1, sunbeam.p2);


		}
		else if (nancy->lampType == LAMP_ORTHOGONAL) {

			

		}
		else if (nancy->lampType == LAMP_SOLAR) {

			// pick a random point on the upper hemisphere.

			float randomAngle =  ( (RNG()-0.5) * 2 * pi );
			float incidentDirection = nancy->direction + randomAngle ;
			float mirrorDirection = nancy->direction - randomAngle ;


			sunbeam.p1 = b2Vec2( nancy->illuminationRadius * cos(incidentDirection)   ,  nancy->illuminationRadius * sin(incidentDirection)  );
			
			sunbeam.p2 = b2Vec2( nancy->illuminationRadius * cos(mirrorDirection)   ,  nancy->illuminationRadius * sin(mirrorDirection)  );


			if (true) { // print the rays of light

			   local_debugDraw_pointer->DrawSegment(sunbeam.p1, sunbeam.p2,  nancy->illuminationColor );
			}

			RayCastClosestCallback stupidMotherFucker;

			local_m_world->RayCast( &stupidMotherFucker, sunbeam.p1, sunbeam.p2);

		}


	
	}


}

void selectNLowestEnergyFish (unsigned int n, std::list<Species>::iterator currentSpecies) {

	// traverse the list n times and find the lowest each time, excluding ones that have previously been found.
	std::list<BonyFish>::iterator fish;

	BonyFish * lowestFishThisTurn = &(*currentSpecies->population.begin());

	float lowestEnergyThisTurn = 1000000000.0f;

	for (unsigned int i = 0; i < n; ++i)
	{
		lowestEnergyThisTurn = 1000000000.0f;
		lowestFishThisTurn = &(*currentSpecies->population.begin());
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if ( !fish->selected) 
			{
				if (fish->energy < lowestEnergyThisTurn) 
				{
					lowestFishThisTurn = &(*fish);
					lowestEnergyThisTurn = lowestFishThisTurn->energy;
				}
			}
		}

		lowestFishThisTurn->selected = true;
	}
}


void selectLowestEnergyFish(int arg) {
	unsigned int n = TestMain::getNumberToSelect(); //3;

	std::list<Species>::iterator currentSpecies;

	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{	
		selectNLowestEnergyFish(n , currentSpecies);
	}
}

void ecosystemModeBeginGeneration (BonyFish * fish, std::list<Species>::iterator currentSpecies ) {

	// the fish's input and output matrix configurations should be passed on to offspring
	for (int i = 0; i < N_SENSECONNECTORS; ++i)
	{
		fish->genes.inputMatrix[i] = fish->inputMatrix[i];
		fish->genes.outputMatrix[i] = fish->outputMatrix[i];
	}

	// go through the species list and find the members with the lowest energy and replace them

	#define N_OFFSPRING 1

	// deselectAll(0);

	// selectNLowestEnergyFish(N_OFFSPRING , currentSpecies);

	// flagSelectedFishForDeletion(0);

	// then add the new ones
	for (int i = 0; i < N_OFFSPRING; ++i) {
		fishDescriptor_t newFishBody = fish->genes;

		mutateFishDescriptor (&newFishBody, m_deepSeaSettings.mutationRate, m_deepSeaSettings.mutationSeverity);

		networkDescriptor ickyBrain = *(fish->brain);

		fann * wann = createFANNbrainFromDescriptor(&ickyBrain);

		std::string nnfilename =  std::string("child.net");
	    fann_save(  wann , nnfilename.c_str()); 

	    mutateFANNFileDirectly(std::string("child.net"));

		// now you can load the mutant ANN.
		fann *jann = loadFishBrainFromFile (std::string("mutant")) ;

		b2Vec2 desiredPosition = fish->bones[0]->p_body->GetWorldCenter();

		desiredPosition.x += ((RNG()-0.5) * 5);
		desiredPosition.y += ((RNG()-0.5) * 5);

		loadFish ( newFishBody, jann, desiredPosition, currentSpecies) ;

		moveAWholeFish ( &(currentSpecies->population.back() ),  desiredPosition);
	}
}

void laboratoryModeBeginGeneration ( std::list<Species>::iterator currentSpecies) { // select an animal as an evolutionary winner, passing its genes on to the next generation

	for (unsigned int i = 0; i < currentSpecies->nominalPopulation; ++i) {

		bool thereIsAFile = false;

		if (FILE *file = fopen("child.net", "r")) {
	        fclose(file);
	        if (FILE *file = fopen("child.fsh", "r")) {
		        thereIsAFile = true;
		        fclose(file);
		    }
	    } 

		if (thereIsAFile ) { // if there is a previous winner, load many of its mutant children

			fishDescriptor_t newFishBody;
			loadFishFromFile(std::string("child.fsh"), newFishBody);

			mutateFishDescriptor (&newFishBody, m_deepSeaSettings.mutationRate, m_deepSeaSettings.mutationSeverity);

		    mutateFANNFileDirectly(std::string("child.net"));

			// now you can load the mutant ANN.
			fann *mann = loadFishBrainFromFile (std::string("mutant")) ;

			// create a neurodescriptor.
			networkDescriptor * muscleCars=  createNeurodescriptorFromFANN (mann) ;

			fann * jann = createFANNbrainFromDescriptor(muscleCars);

			b2Vec2 position = b2Vec2(0.0f,0.0f);

			loadFish ( newFishBody, jann, position, currentSpecies) ;
		}
		else { 						// if there is no winner, its probably a reset or new install. make one up
			 printf("No genetic material found in game folder\n");
			// fishDescriptor_t nematode = nematode(); //fishDescriptor_t();
		// 	b2Vec2 position = b2Vec2(0.0f,0.0f);
		
		// 	loadFish ( nematode, NULL,  position, currentSpecies) ;
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
		brightness = 2;
		illuminationRadius = 10;
		position = b2Vec2(0.0f, 2.0f);
		illuminationColor = b2Color(0.0f, 0.0f, 0.0f);

		 lampType = LAMP_DIRECTIONAL;

	 direction =  0;//0.3 * pi;
	 beamWidth = 2;
}

// void saveDefaultPlant() {

// 	// save the default nematode into the aquarium so that it is accesible
// 	fishDescriptor_t driedWorm  = basicPlant();//  fishDescriptor_t();
// 	BonyFish rehydratedWorm =  BonyFish(driedWorm, NULL, b2Vec2(0.0f, 0.0f));

// 	  // and the default plant
//     std::string nnfilename =  std::string("Aquarium/")  + std::string("defaultPlant") + std::string(".net");
//     std::string fdescfilename =  std::string("Aquarium/") + std::string("defaultPlant") + std::string(".fsh");
//     std::ofstream file { nnfilename };
//     saveFishToFile (fdescfilename, driedWorm);
//     fann_save(  rehydratedWorm.ann , nnfilename.c_str()); 

// }


void saveDefaultPlant() {
	// save the default nematode into the aquarium so that it is accessible
	fishDescriptor_t driedWorm  = basicPlant(); //nematode();//  fishDescriptor_t();
	BonyFish rehydratedWorm =  BonyFish(driedWorm, NULL, b2Vec2(0.0f, 0.0f));

	// save the winner to file with a new name.
	std::string nnfilename =  std::string("Aquarium/")  + std::string("defaultPlant")  + std::string(".net");
    std::string fdescfilename =  std::string("Aquarium/") + std::string("defaultPlant")  + std::string(".fsh");
    std::ofstream file { nnfilename };
    saveFishToFile (fdescfilename, driedWorm);
    fann_save(  rehydratedWorm.ann , nnfilename.c_str()); 
}


void saveDefaultFish() {
	// save the default nematode into the aquarium so that it is accessible
	fishDescriptor_t driedWorm  = nematode();//  fishDescriptor_t();
	BonyFish rehydratedWorm =  BonyFish(driedWorm, NULL, b2Vec2(0.0f, 0.0f));

	// save the winner to file with a new name.
	std::string nnfilename =  std::string("Aquarium/")  + defaultFishName + std::string(".net");
    std::string fdescfilename =  std::string("Aquarium/") + defaultFishName + std::string(".fsh");
    std::ofstream file { nnfilename };
    saveFishToFile (fdescfilename, driedWorm);
    fann_save(  rehydratedWorm.ann , nnfilename.c_str()); 
}

void deepSeaSetup (b2World * m_world, b2ParticleSystem * m_particleSystem, DebugDraw * p_debugDraw) { //, b2World * m_world_sci, b2ParticleSystem * m_particleSystem_sci) {

	// store a single copy to the pointers so we don't have to give them as arguments 1 million times.
	local_debugDraw_pointer = p_debugDraw;
	local_m_world = m_world;
	local_m_particleSystem = m_particleSystem;

	m_deepSeaSettings.gameMode = GAME_MODE_ECOSYSTEM; 	// 0,	// int gameMode;
	m_deepSeaSettings.laboratory_nFood = 0;													// 0,	//  	int laboratory_nFood;
	m_deepSeaSettings.laboratory_nFish = 64;													// 64,	//  	int laboratory_nFish;
	m_deepSeaSettings.gravity = b2Vec2(0.0f, 0.0f);													// b2Vec2(0.0f,0.0f),	//  	b2Vec2 gravity;
	m_deepSeaSettings.mutationRate = 0.05f;													// 0.1,	//  	float mutationRate;
	m_deepSeaSettings.mutationSeverity = 0.5f;													// 	// std::list<BonyFish>::iterator fish;
	m_deepSeaSettings.mentalMutationRate = 0.05f;													// 	// for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
	m_deepSeaSettings.mentalMutationSeverity = 1.0f;													// 0.2,	//  	float mutationSeverity;
	m_deepSeaSettings.terrainPaintType = 0;													// 0.1,	//  	float mentalMutationRate;
	m_deepSeaSettings.originTriggerRadius = 15.0f;													// 0.5,	//  	float mentalMutationSeverity;
	m_deepSeaSettings.originFoodRadius = 10.0f;													// 0,	//  	int terrainPaintType;
	m_deepSeaSettings.foodRadiusAngleJitter = 0.2f;													// 10,
	m_deepSeaSettings.currentlySelectedSpecies = 0;													// 0.1,
	m_deepSeaSettings.barrierRadius = 50.0f;													// 0,
	m_deepSeaSettings.barrierRadiusStatus = 0;													// 0.1f
	m_deepSeaSettings.entropy = 0.1f;													// };

	// initialize foodparticles so they don't crash the program. This step does not add any to the world, and is required even for unused foodparticles.
	// for (int i = 0; i < N_FOODPARTICLES; ++i)
	// {
	// 	boneAndJointDescriptor_t foodDescriptor = *(new boneAndJointDescriptor_t());
	// 	food[i] = new BoneUserData(foodDescriptor, NULL, b2Vec2(0.0f, 0.0f), 0, false);
	// 	food[i]->isUsed = false;
	// }

	// from particle drawing
	b2Assert((k_paramDef[0].CalculateValueMask() & e_parameterBegin) == 0);
	TestMain::SetParticleParameters(k_paramDef, k_paramDefCount);
	TestMain::SetRestartOnParticleParameterChange(false);
	m_particleFlags = TestMain::GetParticleParameterValue();
	m_groupFlags = 0;


	

	saveDefaultPlant();

	saveDefaultFish();


	loadExperimentalMap();


    // std::list<Species>::iterator startingSpecies = ecosystem.begin();
    // startingSpecies->selected = true;

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
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected) {
				brainMelter ( &(*fish));
				fish->ann = createFANNbrainFromDescriptor(fish->brain);
			}
			else {
				continue;
			}
		}
	}
}

void scrambleSelectedFish (int arg) {
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected) {
				brainScrambler ( &(*fish));
				fish->ann = createFANNbrainFromDescriptor(fish->brain);
			}
			else {
				continue;
			}
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




	// if only one neuron is selected, and it is a layer 0 neuron, increment or decrement the primary property of its sense connector.
	if (gotA && !gotB) {
		if (selectedA < fish->brain->layers.begin()->neurons.size() ) {


			if (fish->inputMatrix[selectedA].sensorType == SENSOR_TIMER) {

				if (amount > 0) {
					fish->inputMatrix[selectedA].timerFreq += 1;
				}
				else {
					fish->inputMatrix[selectedA].timerFreq -= 1;
				}

				

			}
			else if (fish->inputMatrix[selectedA].sensorType == SENSECONNECTOR_RECURSORRECEIVER) {

				if (amount > 0) {
					fish->inputMatrix[selectedA].recursorDelay += 1;
				}
				else {
					fish->inputMatrix[selectedA].recursorDelay -= 1;
				}

			}

			
		}

	}



	// refresh the water in the brain jar.
	fish->ann = createFANNbrainFromDescriptor(fish->brain);
}

void changeSelectedConnection(float amount) {
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected) {
				modifyAConnection (&(*fish),amount);
				return;
			}
			else {
				continue;
			}
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

	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) {
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
	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) {
		neuronIndex = 0;
		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 			neuron->position = b2Vec2(drawingStartingPosition.x + (neuronIndex * spacingDistance), drawingStartingPosition.y + (layerIndex * spacingDistance));
 			neuronIndex ++;
		}
		layerIndex++;
	}

	// while you're here, draw a dark grey box around selected layers. but not as light as the selected neuron.
	b2Vec2 layerSelectionIndicator[4];

	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) {

		if (layer->selected) {

			std::list<neuronDescriptor>::iterator neuron;

	 		neuron = layer->neurons.begin(); 
	 		layerSelectionIndicator[0] = b2Vec2( neuron->position.x-0.1f , neuron->position.y-0.1f);
	 		layerSelectionIndicator[1] = b2Vec2( neuron->position.x-0.1f , neuron->position.y+0.1f);

	 		neuron = layer->neurons.end(); 
	 		neuron --;
	 		layerSelectionIndicator[2] = b2Vec2( neuron->position.x+0.1f , neuron->position.y+0.1f);
	 		layerSelectionIndicator[3] = b2Vec2( neuron->position.x+0.1f , neuron->position.y-0.1f);

	 		local_debugDraw_pointer->DrawFlatPolygon(layerSelectionIndicator, 4 ,b2Color(0.2,0.2,0.2) );
		}
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

		std::string connectorLabel = std::string("");

		b2Vec2 mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.2);

		int mint = 0;
		switch (fish->inputMatrix[j].sensorType) {

			case SENSECONNECTOR_UNUSED:	
				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.1f,0.1f,0.1f) );
			break;

			case SENSOR_FOODRADAR:	
				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.05f,0.5f,0.1f) );
				connectorLabel =  "Food Int";

				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.2);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());

				connectorLabel = std::string("Limb ");
				connectorLabel += std::to_string(fish->inputMatrix[j].connectedToLimb);
				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.3);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
			break;

			case SENSOR_ALTRADAR:	
				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.05f,0.5f,0.5f) );
				connectorLabel =  "Food Dir";

				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.2);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());

				connectorLabel = std::string("Limb ");
				connectorLabel += std::to_string(fish->inputMatrix[j].connectedToLimb);
				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.3);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
			break;

			case SENSOR_EYE:	
				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,fish->inputMatrix[j].eyeColor );
				connectorLabel =  "Eye";

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

				connectorLabel = std::string("Freq  ");


				mint =  fish->inputMatrix[j].timerFreq;
				connectorLabel += std::to_string(  mint );

				// https://stackoverflow.com/questions/29200635/convert-float-to-string-with-precision-number-of-decimal-digits-specified
				// connectorLabel = connectorLabel.substr(0, connectorLabel.find(".")+3);

				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.3);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());

			break;

			case SENSECONNECTOR_RECURSORRECEIVER:	
				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.5f,0.5f,0.5f) );
				connectorLabel =  "Rx";

				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.2);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());

				connectorLabel = std::string("Ch ");
				connectorLabel += std::to_string(fish->inputMatrix[j].recursorChannel);
				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.3);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());

				connectorLabel = std::string("Delay ");
				connectorLabel += std::to_string(fish->inputMatrix[j].recursorDelay);
				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.4);
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
			case SENSECONNECTOR_UNUSED:	{
				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.1f,0.1f,0.1f) );
				break;
			}

			case SENSECONNECTOR_MOTOR:	{
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

			case SENSECONNECTOR_RECURSORTRANSMITTER:
			{	
				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.5f,0.5f,0.5f) );

				std::string connectorLabel = std::string("Tx ");
				b2Vec2 mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.2);

				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y+0.2);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
				connectorLabel =  "Ch ";

				connectorLabel += std::to_string(fish->outputMatrix[j].recursorChannel);

				mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y+0.3);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());

				break;
			}
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
 				local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.2f,0.2f,0.2f) );

				std::string connectorLabel = std::string("");
				b2Vec2 mocesfef = b2Vec2(neuron->position.x-0.05, neuron->position.y-0.2);

				connectorLabel = std::string("Bias");
				mocesfef = b2Vec2(neuron->position.x-0.05 + 0.2f, neuron->position.y);
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());

 			}

 			std::list<connectionDescriptor>::iterator connection;
 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {

		    	b2Color segmentColor = b2Color(connection->connectionWeight,connection->connectionWeight,connection->connectionWeight);

		    	if (connection->connectionWeight > 0) {
		    		segmentColor.Set(0.0f,0.0f,connection->connectionWeight );
		    	}
		    	else {
		    		segmentColor.Set(abs(connection->connectionWeight),0.0f,0.0f);
		    	}

			    local_debugDraw_pointer->DrawSegment(neuron->position, (getNeuronByIndex(fish->brain, connection->connectedTo))->position,segmentColor ); 
			}


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
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected) {
				if (fish->brain->networkWindow.Contains(mousePointer)) {
					return &(*fish);
				}
			}
		}
	}
	return (BonyFish*)nullptr;
}

int checkNeuronsInWindow (b2AABB mousePointer, BonyFish * fish) {
	std::list<layerDescriptor>::iterator layer;
	std::list<neuronDescriptor>::iterator neuron;

	unsigned int layerIndex = 0;
	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{

 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {

			if (neuron->position.x < mousePointer.upperBound.x && neuron->position.x > mousePointer.lowerBound.x) {
				if (neuron->position.y < mousePointer.upperBound.y && neuron->position.y > mousePointer.lowerBound.y) {

					neuron->selected = !(neuron->selected);
					return neuron->index;
				}
			}
		}

		b2Vec2 clickCenter = b2Vec2( (mousePointer.lowerBound.x + mousePointer.upperBound.x )/ 2  ,(mousePointer.lowerBound.y + mousePointer.upperBound.y )/ 2   );

		neuron = layer->neurons.begin();

		if ( clickCenter.y >  (neuron->position.y  - 0.2f ) && ( clickCenter.y < neuron->position.y + 0.2f ) ) {
			if (clickCenter.x > (neuron->position.x -0.2f)) {

				neuron = layer->neurons.end();
				neuron--;

				if (clickCenter.x < (neuron->position.x +0.2f)) {

					// do not perform this operation on in or output layers. their arrangement is controlled by the set of body peripherals.
					if (! (layerIndex == 0) && ! (layerIndex == (fish->brain->layers.size() -1 ) ) ) {
						layer->selected = !(layer->selected);
					}
				}
			}
		}

		layerIndex ++;
	}
	return -1;
}

void checkClickInSpeciesWindow( b2AABB mousePointer ) {
	b2Vec2 click = b2Vec2( (mousePointer.upperBound.x + mousePointer.lowerBound.x )/ 2 , (mousePointer.upperBound.y + mousePointer.lowerBound.y) /2 );

	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		if (
			// this will actually break if you change the order of the vertices in the species window. bad design.
			click.x < currentSpecies->windowVertices[0].x &&
			click.x > currentSpecies->windowVertices[2].x &&

			click.y > currentSpecies->windowVertices[0].y &&
			click.y < currentSpecies->windowVertices[1].y
			) 
		{


			std::list<Species>::iterator unclickedSpecies;
			for (unclickedSpecies = ecosystem.begin(); unclickedSpecies !=  ecosystem.end(); ++unclickedSpecies) 	
			{
				unclickedSpecies->selected = false;;
			}

			currentSpecies->selected = true;

		TestMain::getSpeciesNameBar() -> set_text(	currentSpecies->name );
		TestMain::getSpeciesNominalPopulationSpinner() ->set_float_val(currentSpecies->nominalPopulation);

		}
	}
}

void drawBodyEditingWindow(BonyFish * fish) {

	std::string connectorLabel;

	b2Vec2 limbLabelPosition = b2Vec2( 10, 0 );
	int labelsDrawnSoFar = 0;
	float limbLabelSpacingDistance = -0.5f;


	b2Vec2 windowVertices[] = {
			b2Vec2(+10.0f, -10.0f), 
			b2Vec2(+10.0f, +10.0f), 
			b2Vec2(-10.0f, +10.0f), 
			b2Vec2(-10.0f, -10.0f)
		};

	local_debugDraw_pointer->DrawFlatPolygon(windowVertices, 4 ,b2Color(0.1,0.1,0.1) );

	b2Vec2 rootPosition = b2Vec2(0.0f, 0.0f);
	for (unsigned int i = 0; i < N_FINGERS; ++i) {
		if ( !fish->bones[i]->isUsed) {
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

	connectorLabel =  "Available energy ";
	connectorLabel +=  std::to_string(fish->energy);
	b2Vec2 mocesfef = b2Vec2(limbLabelPosition.x-0.05, limbLabelPosition.y-0.2 + (labelsDrawnSoFar * limbLabelSpacingDistance) );
	local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
	labelsDrawnSoFar ++;

	connectorLabel =  "Reproduces at ";
	connectorLabel +=  std::to_string(fish->reproductionEnergyCost);
	 mocesfef = b2Vec2(limbLabelPosition.x-0.05, limbLabelPosition.y-0.2 + (labelsDrawnSoFar * limbLabelSpacingDistance) );
	local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
	labelsDrawnSoFar ++;

	labelsDrawnSoFar ++; // line break
	

	for (unsigned int i = 0; i < N_FINGERS; ++i) {
		if ( !fish->bones[i]->isUsed) {
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


				
				connectorLabel =  "Limb ";
				connectorLabel +=  std::to_string(i);
				b2Vec2 mocesfef = b2Vec2(limbLabelPosition.x-0.05, limbLabelPosition.y-0.2 + (labelsDrawnSoFar * limbLabelSpacingDistance) );
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
				labelsDrawnSoFar ++;


				if (fish->bones[i]->isRoot) {
					connectorLabel =  "   isRoot";
					mocesfef = b2Vec2(limbLabelPosition.x-0.05, limbLabelPosition.y-0.2 + (labelsDrawnSoFar * limbLabelSpacingDistance) );
					local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
					labelsDrawnSoFar ++;
				}

				if (fish->bones[i]->isMouth) {
					connectorLabel =  "   isMouth";
					mocesfef = b2Vec2(limbLabelPosition.x-0.05, limbLabelPosition.y-0.2 + (labelsDrawnSoFar * limbLabelSpacingDistance) );
					local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
					labelsDrawnSoFar ++;
				}

				if (fish->bones[i]->isWeapon) {
					connectorLabel =  "   isWeapon";
					mocesfef = b2Vec2(limbLabelPosition.x-0.05, limbLabelPosition.y-0.2 + (labelsDrawnSoFar * limbLabelSpacingDistance) );
					local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
					labelsDrawnSoFar ++;
				}

				if (fish->bones[i]->isLeaf) {
					connectorLabel =  "   isLeaf";
					mocesfef = b2Vec2(limbLabelPosition.x-0.05, limbLabelPosition.y-0.2 + (labelsDrawnSoFar * limbLabelSpacingDistance) );
					local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
					labelsDrawnSoFar ++;
				}

				if (fish->bones[i]->isFood) {
					connectorLabel =  "   isFood";
					mocesfef = b2Vec2(limbLabelPosition.x-0.05, limbLabelPosition.y-0.2 + (labelsDrawnSoFar * limbLabelSpacingDistance) );
					local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
					labelsDrawnSoFar ++;
				}


				labelsDrawnSoFar ++; // a line break;

				connectorLabel =  "   Energy ";
				connectorLabel +=  std::to_string(fish->bones[i]->energy);
				mocesfef = b2Vec2(limbLabelPosition.x-0.05, limbLabelPosition.y-0.2 + (labelsDrawnSoFar * limbLabelSpacingDistance) );
				local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
				labelsDrawnSoFar ++;

				if (fish->bones[i]->hasGrown) {
					connectorLabel =  "   hasGrown";
					mocesfef = b2Vec2(limbLabelPosition.x-0.05, limbLabelPosition.y-0.2 + (labelsDrawnSoFar * limbLabelSpacingDistance) );
					local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
					labelsDrawnSoFar ++;
				}
				else {
					connectorLabel =  "   not grown";
					mocesfef = b2Vec2(limbLabelPosition.x-0.05, limbLabelPosition.y-0.2 + (labelsDrawnSoFar * limbLabelSpacingDistance) );
					local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
					labelsDrawnSoFar ++;
				}








				
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

	unsigned int n_failures = 0;

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

	if (bone->isLeaf && !bone->hasGrown) {
			return;
		}

	unsigned nVertices = bone->shape.GetVertexCount();

	for (unsigned int j = 0; j < nVertices; ++j)
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


// void incorporateLeaf();

void gravitate (BoneUserData * p_bone) {

	p_bone->p_body->ApplyForce(m_deepSeaSettings.gravity, p_bone->p_body->GetWorldCenter() , true);

}

void runBiomechanicalFunctions () {
	unsigned int spacesUsedSoFar =0;

	bool alreadyDrawnThisTurn = false;

	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{

			float energyUsedThisTurn = 0;

			// update the fish's senses
			for (int j = 0; j < N_FINGERS; ++j) {


				if (!fish->bones[j]->isUsed) {
					continue;
				}

				if (fish->bones[j]->flagDelete) {
					amputate( &(*fish), j);
				}

		
				nonRecursiveSensorUpdater (fish->bones[j]);

				// perform the flight simulation on the fish
				flightModel( fish->bones[j] );

				// add the force of gravity
				gravitate ( fish->bones[j] );

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

					case SENSOR_ALTRADAR:
						sensorium[j] = fish->bones[ fish->inputMatrix[j].connectedToLimb  ]->sensation_altradar;
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

					case SENSECONNECTOR_RECURSORRECEIVER:
						// how this works.
						// the transmitters just set their buffer first place to the sample. that's it
						// the receivers trawl the transmitter list and look for one on the same channel
						float receivedSample = 0.0f;
						for (unsigned int k = 0; k < sizeOfOutputLayer; ++k)
						{
							if (k >= N_SENSECONNECTORS) {
								continue;	// if the sensorium array is bigger than the array of input connectors, you can just leave the rest blank.
							}

							if (fish->outputMatrix[k].sensorType == SENSECONNECTOR_RECURSORTRANSMITTER && fish->outputMatrix[k].recursorChannel == fish->inputMatrix[j].recursorChannel ) {
								receivedSample = fish->outputMatrix[k].recursorBuffer[0];// = motorSignals[j];
								break;
							}
						}

						// they set their own sample, at the scrolling cursor, to it
						fish->inputMatrix[j].recursorBuffer[ fish->inputMatrix[j].recursorCursor ] = receivedSample;

						// then they take a sample from [delay] samples ago and apply it
						unsigned int adjustedCursor = fish->inputMatrix[j].recursorCursor;

						unsigned int cursorPlusDelay = fish->inputMatrix[j].recursorCursor + fish->inputMatrix[j].recursorDelay ;
						if (cursorPlusDelay > SENSECONNECTOR_BUFFERSIZE-1) {
							adjustedCursor = 0;
							unsigned int bufferSize = SENSECONNECTOR_BUFFERSIZE;
							adjustedCursor += (cursorPlusDelay - bufferSize);
						}	
						else {
							adjustedCursor += fish->inputMatrix[j].recursorDelay ;
						}

						sensorium[j] = fish->inputMatrix[j].recursorBuffer[adjustedCursor];
						fish->inputMatrix[j].recursorCursor ++;

						// the buffer loops around the max size, but there is no performance downside to doing this because you're not rolling the entire buffer
						if (fish->inputMatrix[j].recursorCursor >= SENSECONNECTOR_BUFFERSIZE) {
							fish->inputMatrix[j].recursorCursor = 0;	
						}

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

					case SENSECONNECTOR_RECURSORTRANSMITTER:
						fish->outputMatrix[j].recursorBuffer[0] = motorSignals[j];
					break;

					case SENSECONNECTOR_MOTOR:
	
						// first just check to make sure the bone and joint is good.
						// this paragraph could be condensed very easily
						if ( !fish->bones[fish->outputMatrix[j].connectedToLimb]->isUsed) {
							continue;
						}
						else {
							if (fish->bones[fish->outputMatrix[j].connectedToLimb]->isRoot) {
								continue;
							}
							if (fish->bones[fish->outputMatrix[j].connectedToLimb]->joint->isUsed) {
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


				// printf("6.4\n");
			// if (m_deepSeaSettings.gameMode == GAME_MODE_ECOSYSTEM) {
			if (true) {



				for (unsigned int i = 0; i < N_FINGERS; ++i) {

					if (fish->bones[i]->isUsed) {

						// printf("s");
						// continue;

						// if you have enough energy to grow a new branch, do it
						if (fish->bones[i]->isLeaf) 
						{
							// continue;

							// printf("e");
							if (!fish->bones[i]->hasGrown) {

							// printf("a");
								// continue;

								if (fish->bones[i]->attachedTo < 0 || fish->bones[i]->attachedTo > N_FINGERS) {
									continue;
								}
								if (! (fish->bones[ fish->bones[i]->attachedTo ] ->isUsed) ) {
									continue;
								}

							if (fish->bones[ fish->bones[i]->attachedTo ] ->hasGrown) 
							{

									// continue;
									// printf("b");
								if (fish->energy > fish->bones[i]->energy) 
								{


									// printf(  "the limb %i is attached to the grown limb %i\n",  i, fish->bones[i]->attachedTo );
										// printf("c");
								
									// if (  !   {

										if (true) {
												// printf("added limb %i on fish\n", i);

												// add the limb on
												nonRecursiveBoneIncorporator( fish->bones[i]);

												// printf("malamar");

												// turn collisions off, move the limb, and then turn them back on again
												b2Filter tempFilterOn = fish->bones[i]->p_fixture->GetFilterData();
												tempFilterOn.groupIndex = fish->bones[i]->collisionGroup;

												b2Filter tempFilterOff = fish->bones[i]->p_fixture->GetFilterData();
												tempFilterOff.groupIndex = -1;

												fish->bones[i]->p_fixture->SetFilterData(tempFilterOff);


												fish->bones[i]->p_body->SetTransform(fish->bones[  fish->bones[i]->attachedTo ]->p_body->GetWorldCenter() ,  fish->bones[  fish->bones[i]->attachedTo ]->p_body->GetAngle() + pi  );


												fish->bones[i]->p_fixture->SetFilterData(tempFilterOn);


												fish->bones[i]->hasGrown = true;

												fish->energy -= fish->bones[i]->energy;

												printf("a\n");

										}
									}
								}

								// if you're here, the limb is a leaf but has not grown. you should skip the rest
								continue;
							}
						}

						// if sunlight fell on a leaf, give energy to it.
						if (fish->bones[i]->isLeaf && fish->bones[i]->flagPhotosynth) {
							fish->bones[i]->flagPhotosynth = false;
							fish->energy += 100.0f;
						}

						// 1 energy is lost per bone per turn for homeostasis or whatever. this is also so that plants can't live forever without sunlight.
						if (TestMain::getEntropyStatus()) {

							for (int i = 0; i < N_FINGERS; ++i)
							{
								if (fish->bones[i]->isUsed) {
									fish->energy -= fish->bones[i]->energy *  m_deepSeaSettings.entropy * 0.01f;	
								}
							}

							
						}
					}

					// printf("6.5\n");
				}


				// printf("6.6\n");

				// kill the fish if it is out of energy.
				if (TestMain::getEntropyStatus()) {
					fish->energy -= energyUsedThisTurn;

					// printf("charhe: %f\n", fish->energy);
					if (fish->energy < 0) {
						fish->flagDelete = true;
					}

				}

				if (true) {

					// give the fish some bebes if it has collected enough food. This always costs energy, even if entropy is turned off.
					if (fish->energy > fish->reproductionEnergyCost) {
						ecosystemModeBeginGeneration( &(*fish), currentSpecies );
						fish->energy -= fish->reproductionEnergyCost;
						fish->numberOfTimesReproduced ++;
						if (fish->numberOfTimesReproduced > REPRODUCTIONCAP) {
							fish->flagDelete = true;
						}
					}


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
					drawBodyEditingWindow(&(*fish));
				}
			}
			
			// calculate output wiggle
			float outputWiggleThisTurn = 0;
			for (unsigned int j = 0; j < sizeOfOutputLayer; ++j)
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
}

void flagSelectedFishForDeletion(int arg) {
	unused_variable((void *)&arg);
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected) {
				fish->flagDelete = true;
				fish->selected = false;
			}
		}
	}

	// iterate through food particles
	// for  (int i = 0; i < N_FOODPARTICLES; i++) {
	// 	if (food[i]->selected) {
	// 		food[i]->flagDelete = true;
	// 		food[i]->selected = false;
	// 	}
	// }
}


void voteSelectedFish(int arg) {

	unused_variable((void *)&arg);

	std::list<Species>::iterator currentSpecies;
		for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
		{
			std::list<BonyFish>::iterator fish;
			for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
			{
		if (fish->selected) {
			vote( &(*fish) );
			}
		}
	}
}


// toggles whether the fish is a tree or not.
// void makeFishAPlant (int arg) {

// 	unused_variable((void *)&arg);

// 	std::list<Species>::iterator currentSpecies;
// 	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
// 	{
// 		std::list<BonyFish>::iterator fish;
// 		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
// 		{
// 			if (fish->selected) {
// 				if ( fish->bones[0]->isLeaf ) {
// 					for (int i = 0; i < N_FINGERS; ++i)
// 					{
// 						if (fish->bones[i]->isUsed) {
// 							fish->bones[i]->isLeaf = false;
// 							fish->genes.bones[i].isLeaf = false;

// 						}
// 					}		
// 				}
// 				else {
// 					for (int i = 0; i < N_FINGERS; ++i)
// 					{
// 						if (fish->bones[i]->isUsed) {
// 							fish->bones[i]->isLeaf = true;
// 							fish->genes.bones[i].isLeaf = true;

// 							fish->bones[i]->isMouth = false;
// 							fish->genes.bones[i].isMouth = false;
// 						}
// 					}	
// 				}
// 			}
// 		}
// 	}
// }

void makeLimbAWeapon (int arg) {

	unused_variable((void *)&arg);

	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected) 
			{
				for (unsigned int i = 0; i < N_FINGERS; ++i)
				{
					if (fish->bones[i]->isUsed) 
					{
						if (i == currentlySelectedLimb) 
						{
							fish->bones[i]->isWeapon = !fish->bones[i]->isWeapon;
							fish->genes.bones[i].isWeapon = !fish->genes.bones[i].isWeapon;
						}
					}	
				}
			}
		}
	}
}

void makeLimbAMouth (int arg) {

	unused_variable((void *)&arg);

	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected) 
			{
				for (unsigned int i = 0; i < N_FINGERS; ++i)
				{
					if (fish->bones[i]->isUsed) 
					{
						if (i == currentlySelectedLimb) 
						{
							fish->bones[i]->isMouth = !fish->bones[i]->isMouth;
							fish->genes.bones[i].isMouth = !fish->genes.bones[i].isMouth;
						}
					}	
				}
			}
		}
	}
}

void makeLimbALeaf (int arg) {

	unused_variable((void *)&arg);

	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected) 
			{
				for (unsigned int i = 0; i < N_FINGERS; ++i)
				{
					if (fish->bones[i]->isUsed) 
					{
						if (i == currentlySelectedLimb) 
						{
							fish->bones[i]->isLeaf = !fish->bones[i]->isLeaf;

							fish->genes.bones[i].isLeaf = !fish->genes.bones[i].isLeaf;


							// update udatawrapper
							// else if (p_bone->isLeaf) {
							uDataWrap * p_dataWrapper = new uDataWrap(fish->bones[i], TYPE_LEAF);
							fish->bones[i]->p_body->SetUserData((void *)p_dataWrapper);
	// }

						}
					}	
				}
			}
		}
	}
}

// in lab mode, this clears out all the fish and makes n of the selected one.
// in eco mode, this clears out the selected fish only and drops children at it.
void handleReproduceSelectedButton (int arg) {
	if (m_deepSeaSettings.gameMode == GAME_MODE_LABORATORY) {
		voteSelectedFish(arg);
	}
	else  
	{
		std::list<Species>::iterator currentSpecies;
		for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
		{
			std::list<BonyFish>::iterator fish;
			for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
			{
				if (fish->selected) 
				{
					ecosystemModeBeginGeneration (  &(*fish), currentSpecies ) ;
				}
			}
		}
	}
}

void deselectAll (int arg) {
	unused_variable((void *)&arg);
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->selected) {
				fish->selected = false;
			}
		}
	}
}

void selectAll (int arg) {
	unused_variable((void *)&arg);
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (!fish->selected) {
				fish->selected = true;
			}
		}
	}
}

void invertSelection (int arg) {
	unused_variable((void *)&arg);
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			fish->selected = !fish->selected;
		}
	}
}

void pinToGrid(int arg) {
	unused_variable((void *)&arg);
	float gridSpacing = 5.0f;

	uint index = 0;

	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		uint gridSize = int(sqrt(currentSpecies->nominalPopulation));
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
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
}

void releaseFromGrid(int arg) {
	unused_variable((void *)&arg);
	std::list<Species>::iterator currentSpecies;
	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		std::list<BonyFish>::iterator fish;
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			for (uint i = 0; i < N_FINGERS; ++i)
			{
				if (fish->bones[i]->isRoot) {
					fish->bones[i]->p_body->SetType(b2_dynamicBody);
				}
			}
		}
	}
}

void selectFishWhoMovedTheFurthest (int arg) {
	unused_variable((void *)&arg);

	std::list<Species>::iterator currentSpecies;
	std::list<BonyFish>::iterator fish;
	currentSpecies= ecosystem.begin();
	fish = currentSpecies->population.begin();


	BonyFish * theFurthest = &(*fish);
	// BonyFish * referenceFish = &(*fish); // used to reset the chosen fish for each selection cycle... otherwise it remains as the winner and next round is unsuccessful!


	for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
	{
		for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
		{
			if (fish->distanceMovedSoFar > theFurthest->distanceMovedSoFar && isinf(fish->distanceMovedSoFar) == false)
			{
				theFurthest = &(*fish);
			}
		}
		theFurthest->selected = true;
		// theFurthest = referenceFish;
	}
}

void selectFurthestFromOrigin (int arg) {
	unused_variable((void *)&arg);

	// TestMain::getNumberToSelect();

	std::list<Species>::iterator currentSpecies;
	std::list<BonyFish>::iterator fish;
	currentSpecies= ecosystem.begin();
	fish = currentSpecies->population.begin();
	BonyFish * theFurthest = &(*fish); 
	BonyFish * referenceFish = &(*fish); // used to reset the chosen fish for each selection cycle... otherwise it remains as the winner and next round is unsuccessful!

	deselectAll(0);

	// printf("smeseis: %i\n", TestMain::getNumberToSelect());

	for (int i = 0; i < TestMain::getNumberToSelect(); ++i)
	{
		for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
		{
			for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
			{
				if (fish->selected) { 
					continue;
				}
				if (magnitude(fish->bones[0]->p_body->GetWorldCenter() ) > magnitude(theFurthest->bones[0]->p_body->GetWorldCenter())) {
					theFurthest = &(*fish);
				}
				
			}

			theFurthest->selected = true;
			theFurthest = referenceFish;
		}
	}
}

void selectClosestToFood (int arg) {

	return; // i am in the process of removing the list of 'food particles', which was like a weird secondary list of creatures that had to be separately handled. If regular fish can be eaten, a 'fish food flakes' should just be a regular species with 1 segment. This function should be rewritten to work that way.

	unused_variable((void *)&arg);
	std::list<Species>::iterator currentSpecies;
	std::list<BonyFish>::iterator fish;
	currentSpecies= ecosystem.begin();
	fish = currentSpecies->population.begin();
	BonyFish * theClosest = &(*fish);
	BonyFish * referenceFish = &(*fish); // used to reset the chosen fish for each selection cycle... otherwise it remains as the winner and next round is unsuccessful!

	// float theBestDistanceSoFar = 1000000000.0f; // not going to work if all the fish are more than a billion units away.
	bool chosen = false;


	deselectAll(0);

	for (int j = 0; j < TestMain::getNumberToSelect(); ++j)
	{
		/* code */
	
		for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
		{
			for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
			{

				if (fish->selected) {
					continue;
				}

				// for  (int i = 0; i < N_FOODPARTICLES; i++) {

				// 	if (food[i]->isUsed) {

				// 		b2Vec2 vectorToFood = b2Vec2(food[i]->position.x - fish->bones[0]->p_body->GetWorldCenter().x , food[i]->position.y - fish->bones[0]->p_body->GetWorldCenter().y);

				// 		float distanceToFood = magnitude(vectorToFood);

				// 		if (distanceToFood < theBestDistanceSoFar) {
				// 			theClosest = &(*fish);
				// 			theBestDistanceSoFar = distanceToFood;
				// 			chosen = true;
				// 		}
				// 	}
				// }
			}
		}

		if (chosen) {
			theClosest->selected = true;
			theClosest = referenceFish;
			// theBestDistanceSoFar = 1000000000.0f; // not going to work if all the fish are more than a billion units away.
		 	chosen = false;
		}

		

	}
}

void deepSeaLoop () {


if (TestMain::gameIsPaused()) {
	local_debugDraw_pointer->DrawString(b2Vec2(100,100), std::string("Paused").c_str());
		return;
	}

	// printf("1\n");
	
	TestMain::PreStep();

	// printf("2\n");
	

	TestMain::Step();

	// printf("3\n");
	

	if (!local_m_world->IsLocked() ) {

		std::list<Species>::iterator currentSpecies;
		std::list<BonyFish>::iterator fish;

		if ( TestMain::getBarrierRadiusStatus()) 
		{
			currentSpecies= ecosystem.begin();
			fish = currentSpecies->population.begin();

			for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
			{
				for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
				{
					// get distance from center
					b2Vec2 fishPosition = fish->bones[0]->p_body->GetWorldCenter();
					float fishDistance = magnitude(fishPosition);

					if (fishDistance > m_deepSeaSettings.barrierRadius) {

						// measure angle and relocate 
						float fishAngle = atan2(fishPosition.y, fishPosition.x);
						b2Vec2 newPosition = b2Vec2(cos(fishAngle) * m_deepSeaSettings.barrierRadius, sin(fishAngle) * m_deepSeaSettings.barrierRadius);

						moveAWholeFish(&(*fish), newPosition);

					}
				}
			}
		}

		if ( TestMain::getTriggerRadiusStatus()   ) 
		{

			// if radius trigger is turned on, check the distances of the fish. when the average distance is greater than the trigger radius, the generation is reset.			
			currentSpecies= ecosystem.begin();
			fish = currentSpecies->population.begin();

			float avgDistance = 0.0f;

			unsigned int tally = 0;

			for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
			{
				for (fish = currentSpecies->population.begin(); fish !=  currentSpecies->population.end(); ++fish) 	
				{
					avgDistance += magnitude(fish->bones[0]->p_body->GetWorldCenter() );
					tally += 1;
				}
			}

			float floatTally = tally;
			avgDistance = avgDistance	/ floatTally;

			if (avgDistance > m_deepSeaSettings.originTriggerRadius) {
				reloadTheSim(0);
			} 
		}

		if (loopCounter < loopSafetyLimit) {
			// startNextGeneration = false;
		}		

		loopCounter ++;

		// printf("4\n");
	

		removeDeletableFish();

		deleteFlaggedSpecies() ;

		// printf("5\n");
	

		if (startNextGeneration && m_deepSeaSettings.gameMode == GAME_MODE_LABORATORY ) 
		{
			if (loopCounter > loopSafetyLimit) 
			{
				if (TestMain::getFoodRadiusStatus() ) 
				{

					// take the food, measure its angle from the origin, and then place it at the designated radius with some angle jitter
					// iterate through food particles
					// for  (int i = 0; i < N_FOODPARTICLES; i++) 
					// {
					// 	if  ( food[i]->isUsed)
					// 	{

					// 		b2Vec2 origin   = b2Vec2(0.0f, 0.0f);
					// 		b2Vec2 foodPosition =  food[i]->p_body->GetWorldCenter();

					// 		float angle = atan2( foodPosition.y - origin.y,  foodPosition.x - origin.x );

					// 		angle += ( m_deepSeaSettings.foodRadiusAngleJitter * (RNG()-0.5) );

					// 		b2Vec2 foodRadiusPosition = b2Vec2( cos(angle)* m_deepSeaSettings.originFoodRadius , sin(angle)* m_deepSeaSettings.originFoodRadius );
					// 		food[i]->p_body->SetTransform(foodRadiusPosition, 0.0f);

					// 	}
					// }
				}

				for (currentSpecies = ecosystem.begin(); currentSpecies !=  ecosystem.end(); ++currentSpecies) 	
				{
					laboratoryModeBeginGeneration (currentSpecies );
				}
				loopCounter = 0;
			}
		}

		// printf("6\n");
	

		startNextGeneration = false;

		if (TestMain::getLampStatus()) {
			std::list<Lamp>::iterator lomp;
			for (lomp = lamps.begin(); lomp !=  lamps.end(); ++lomp) 	{
				
				shine(&(*lomp));
			
			}
		}

		drawingTest();

		// drawTerrain();

		runBiomechanicalFunctions();

		// printf("7\n");
	

		// if (flagAddFood) {
		// 	flagAddFood = false;
		// 	addRandomFoodParticle(0);
		// }

		// for  (int i = 0; i < N_FOODPARTICLES; i++) {
		// 	if ( food[i]->isUsed) {

		// 		// perform the flight simulation on the fish
		// 		flightModel( food[i] );

		// 		food[i]->position = food[i]->p_body->GetWorldCenter(); // update positions of all the food particles
		// 	}
		// }
	}

	// printf("8\n");
	

	TestMain::PostStep();

	// printf("9\n");
	
}

void collisionHandler (void * userDataA, void * userDataB, b2Contact * contact) {
	bool et = false;
	bool fud = false;
	// bool lef = false;

	// printf("1\n");

	if (m_deepSeaSettings.gameMode == GAME_MODE_LABORATORY) {
		if (startNextGeneration || loopCounter < loopSafetyLimit) {
			return;
		}
	}

	if (userDataA == nullptr  ) {
		return;
	}

	if ( userDataB == nullptr ) {
		return;
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

	// printf("2\n");
		if( dataB.dataType == TYPE_FOOD || dataB.dataType == TYPE_LEAF )
		{
			fud = true;

	// printf("3\n");
		}
	}

	if( dataB.dataType == TYPE_MOUTH ) {
		et = true;

	// printf("2\n");
		if( dataA.dataType == TYPE_FOOD )
		{
			fud = true;

	// printf("3\n");
		}
		// if (dataA.dataType == TYPE_LEAF ) {
		// 	lef = true;
		// }
	}

	if (et && fud  ) {

	// printf("4\n");
		if( dataB.dataType == TYPE_MOUTH ) {
			BonyFish * fish = (((BoneUserData *)(dataB.uData))->p_owner );
			BoneUserData * food = ((BoneUserData *)(dataA.uData) );

			if (fish->flagDelete || food->p_owner->flagDelete || food->flagDelete) {
				return;
			}

			if (m_deepSeaSettings.gameMode == GAME_MODE_LABORATORY) {

	// printf("5\n");
			    vote(fish);
			}

			else if (m_deepSeaSettings.gameMode == GAME_MODE_ECOSYSTEM) {

				// if the part is not terminal, it is smashed off and becomes food
				bool terminal = true;
				for (int i = 0; i < N_FINGERS; ++i) {
					if (food->p_owner->bones[i]->isUsed) {
					 	if ( food->p_owner->bones[i]->hasGrown ) {
							if (food->p_owner->bones[i]->attachedTo == food->index) {
								terminal = false;
							}
						}
					}
				}
				// unused_variable((void *)terminal);

				if (terminal) {
					food->flagDelete = true;
					
					if (food->isRoot) {
						food->p_owner->flagDelete = true;
					}


					fish->feed(food->energy);
					// else {
					// 	food->flagDelete = true;
					// 	// amputate(food->p_owner, food->index);
					// }
				}
				


			}
		}
		else if (dataA.dataType == TYPE_MOUTH) {
			BonyFish * fish = (((BoneUserData *)(dataA.uData))->p_owner );
			BoneUserData * food = ((BoneUserData *)(dataB.uData) );

			
			if (fish->flagDelete || food->p_owner->flagDelete || food->flagDelete) {
				return;
			}
			if (m_deepSeaSettings.gameMode == GAME_MODE_LABORATORY) {

	// printf("5\n");
				vote(fish);
			}
			else if (m_deepSeaSettings.gameMode == GAME_MODE_ECOSYSTEM) {

	// if the part is not terminal, it is smashed off and becomes food
				bool terminal = true;
				for (int i = 0; i < N_FINGERS; ++i) {
					if (food->p_owner->bones[i]->isUsed) {
					 	if ( food->p_owner->bones[i]->hasGrown ) {
							if (food->p_owner->bones[i]->attachedTo == food->index) {
								terminal = false;
							}
						}
					}
				}

				// unused_variable((void *)terminal);


				if (terminal) {
					food->flagDelete = true;
					if (food->isRoot) {
						food->p_owner->flagDelete = true;
					}


					fish->feed(food->energy);
					
					// else {
					// 	// amputate(food->p_owner, food->index);
					// }
				}
				

			}
		}
	}

	// if (et && lef) {

	// }
}

// ------------------------------------------------------------------

// the following code is an attempt to integrate particle drawing (from the test of the same name) into deepsea, in a way that can be applied to all deepsea maps.

void ParticleDrawingKeyboard(int key)
{
	m_drawing = TestMain::getPaintingStatus(); //key != 'X';

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

// game logic which is run only once at startup
void deepSeaStart() {
	m_deepSeaSettings.originTriggerRadius = 15.0f;

	m_deepSeaSettings.originFoodRadius = 10.0f;

	m_deepSeaSettings.barrierRadius = 50.0f;

	defaultSpecies->selected = true;
	ecosystem.push_back(*defaultSpecies);
}