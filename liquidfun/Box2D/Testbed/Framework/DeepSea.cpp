#include "DeepSea.h"
#include "fann.h"
#include "Test.h"

#include <iostream>
#include <fstream>
#include <random>
#include <string>
#include <limits>
#include <stdio.h>
#include <chrono>
#include <thread>

int currentNumberOfFood = 0;
int currentNumberOfFish = 0;

int generationsThisGame = 0;
bool startNextGeneration = false;

bool fishSlotLoaded[N_FISHES];
bool foodSlotLoaded[N_FOODPARTICLES];

bool userControlInputA;
bool userControlInputB;

b2World * local_m_world = nullptr;
b2ParticleSystem * local_m_particleSystem = nullptr;

float pi = 3.14159f;

void setUserControlInputA() {
userControlInputA = true;
}

void setUserControlInputB () {
userControlInputB = true;
}


DebugDraw * local_debugDraw_pointer;

// these FANN parameters should be common to all networks.
const float desired_error = (const float) 0.01;
const unsigned int max_epochs = 50000;
const unsigned int epochs_between_reports = 100;

foodParticle_t * food[N_FOODPARTICLES];
BonyFish * fishes[N_FISHES];

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

float RNG() { //
    static std::default_random_engine e;
    static std::uniform_real_distribution<> dis(0, 1); // rage 0 - 1
    return dis(e);
}

JointUserData::JointUserData(boneAndJointDescriptor_t boneDescription, BoneUserData * p_bone, BonyFish * fish, b2World * m_world, b2ParticleSystem * m_particleSystem) {
	torque = boneDescription.torque;
	speedLimit = boneDescription.speedLimit;
	upperAngle = boneDescription.upperAngle;
	normalAngle = boneDescription.normalAngle;
	lowerAngle = boneDescription.lowerAngle;
	isUsed = false;

	// the following code prepares the box2d objects.
	if (boneDescription.isRoot) {
		;
	}
	else {
		jointDef.bodyA = fish->bones[boneDescription.attachedTo]->p_body;
		jointDef.bodyB = p_bone->p_body;
		jointDef.localAnchorA =  fish->bones[boneDescription.attachedTo]->tipCenter;
		jointDef.localAnchorB =  fish->bones[boneDescription.attachedTo]->tipCenter;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = lowerAngle;
		jointDef.upperAngle = upperAngle;
		jointDef.enableMotor = true;
	    jointDef.maxMotorTorque = torque;
	}
    jointDef.userData = this;
    init = true;
}

uDataWrap::uDataWrap(void * dat, uint8_t typ) {
		uData = dat;
		dataType = typ;
}

void printab2Vec2(b2Vec2 v) {
	printf("x%f y%f\n", v.x, v.y);
}

BoneUserData::BoneUserData(
		boneAndJointDescriptor_t boneDescription,
		BonyFish * fish,
		b2World * m_world, b2ParticleSystem * m_particleSystem, // primarily needed to create the body
		b2Vec2 positionOffset
	) {

	if (!boneDescription.used) {
		return;
	}

	printf("creating a bone\n");

	p_owner = fish;
	BoneUserData * attachesTo = fish->bones[boneDescription.attachedTo];	

	// initialize everything to default, sane values
	length = boneDescription.length;
	rootThickness = boneDescription.rootThickness;
	tipThickness = boneDescription.tipThickness;
	density = 1.5f;
	isRoot = boneDescription.isRoot;
	isMouth = boneDescription.isMouth;
	isSensor = boneDescription.isSensor;
	sensation = 0.0f;
	isWeapon  = boneDescription.isWeapon;									// weapons destroy joints to snip off a limb for consumption. optionally, they can produce a physical effect.
	energy = ((rootThickness + tipThickness)/2) * (length * density); 		// the nutritive energy stored in the tissue of this limb; used by predators and scavengers

	tipCenter = b2Vec2(positionOffset.x,positionOffset.y+0.1f); 											// these are used so the skeleton master can remember his place as he traverses the heirarchy of souls.
	rootCenter = b2Vec2(positionOffset.x,positionOffset.y); 		
	int count = 4;

	// the following code is used to generate box2d structures and shapes from the bone parameters.
	if (isRoot) {
		printf("its a root bone\n");

		tipCenter = b2Vec2(positionOffset.x, positionOffset.y +  length);

		b2Vec2 vertices[] = {
			b2Vec2(rootCenter.x + (rootThickness/2), rootCenter.y), //b2Vec2 rootVertexA = 
			b2Vec2(rootCenter.x - (rootThickness/2), rootCenter.y), // b2Vec2 rootVertexB =
			b2Vec2(tipCenter.x + (tipThickness/2), tipCenter.y), //b2Vec2 tipVertexA = 
			b2Vec2(tipCenter.x - (tipThickness/2), tipCenter.y) // b2Vec2 tipVertexB = 
		};
		
		
		
		// figure out the center point.
		// b2Vec2 boneCenter = b2Vec2(0.0f, 0.0f + (0.5*length));
	
		if (isMouth) {
			uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_MOUTH);
			bodyDef.userData = (void *)p_dataWrapper;
		}
		else {
			uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_DEFAULT);
			bodyDef.userData = (void *)p_dataWrapper;

		}
		
		bodyDef.type = b2_dynamicBody;
		p_body = m_world->CreateBody(&bodyDef);
		
		// shape.SetAsBox(rootThickness, length, boneCenter,0.0f);	
		shape.Set(vertices, count);

		// reference the physics object from the user data.
		tipCenter = tipCenter;
		rootCenter = b2Vec2(0.0f, 0.0f);
	}
	else {
		printf("its not a root bone\n");
		tipCenter = b2Vec2(attachesTo->tipCenter.x, attachesTo->tipCenter.y + length);
		// b2Vec2 rootVertexA = b2Vec2(attachesTo->tipCenter.x + (rootThickness/2), attachesTo->tipCenter.y);
		// b2Vec2 rootVertexB = b2Vec2(attachesTo->tipCenter.x - (rootThickness/2), attachesTo->tipCenter.y);
		// b2Vec2 tipVertexA = b2Vec2(tipCenter.x + (tipThickness/2), tipCenter.y);
		// b2Vec2 tipVertexB = b2Vec2(tipCenter.x - (tipThickness/2), tipCenter.y);

		b2Vec2 vertices[] = {
			b2Vec2(attachesTo->tipCenter.x + (rootThickness/2), attachesTo->tipCenter.y), //b2Vec2 rootVertexA = 
			b2Vec2(attachesTo->tipCenter.x - (rootThickness/2), attachesTo->tipCenter.y), // b2Vec2 rootVertexB =
			b2Vec2(tipCenter.x + (tipThickness/2), tipCenter.y), //b2Vec2 tipVertexA = 
			b2Vec2(tipCenter.x - (tipThickness/2), tipCenter.y) // b2Vec2 tipVertexB = 
		};

		// figure out the center point.
		// b2Vec2 boneCenter = b2Vec2(attachesTo->tipCenter.x, attachesTo->tipCenter.y + (2*length));

		// attach user data to the body
		
		if (isMouth) {
			uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_MOUTH);
			bodyDef.userData = (void *)p_dataWrapper;
		}
		else {
			uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_DEFAULT);
			bodyDef.userData = (void *)p_dataWrapper;

		}

		bodyDef.type = b2_dynamicBody;
		p_body = m_world->CreateBody(&bodyDef);

		printf("tip center: ");
		printab2Vec2(tipCenter);
		
		for (int i = 0; i < 4; ++i)
		{
			printf(" ");
			printab2Vec2(vertices[i]);
		}
		
		// shape.SetAsBox(rootThickness, length, boneCenter, 0.0f);	
		shape.Set(vertices, count);

		// reference the physics object from the user data.
		tipCenter = tipCenter;
		rootCenter = attachesTo->tipCenter;
	}

	if (!isRoot) {
		joint = new JointUserData( boneDescription, this, fish, m_world, m_particleSystem); 	// the joint that attaches it into its socket 
	}	

	init = true;
	isUsed=  false;
	printf("\n");
};

void nonRecursiveBoneIncorporator(BoneUserData * p_bone, b2World * m_world, b2ParticleSystem * m_particleSystem, uint8_t boneIndex) {
	if (!p_bone->init) {
		return;
	}
	p_bone->p_fixture = p_bone->p_body->CreateFixture(&(p_bone->shape), p_bone->density);	// this endows the shape with mass and is what adds it to the physical world.

	// uint16_t mask = 1;
	// p_fixture->filter->maskbits = mask << boneIndex; // shift the mask up by 1 on each bone so that none of them 

	if (!p_bone->isRoot) {
            p_bone->joint->isUsed = true;
			p_bone->joint->p_joint = (b2RevoluteJoint*)m_world->CreateJoint( &(p_bone->joint->jointDef) );
	}
	p_bone->isUsed = true;
}



void nonRecursiveSensorUpdater (BoneUserData * p_bone) {
	if (!p_bone->init || !p_bone->isUsed) {
		return;
	}
	p_bone->position = p_bone->p_body->GetPosition();
	p_bone->sensation = 0.0f;

	for  (int i = 0; i < N_FOODPARTICLES; i++) {
		if (!foodSlotLoaded[i]) {
			break;
		}
		if (food[i]->init && food[i]->isUsed) {
			b2Vec2 positionalDifference = b2Vec2((p_bone->position.x - food[i]->position.x),(p_bone->position.y - food[i]->position.y));
			float distance = magnitude (positionalDifference);
			if (distance > 0) {
				p_bone->sensation += 1/distance;
			}
		}
	}
}

// add a food particle to the world and register it so the game knows it exists.
foodParticle_t::foodParticle_t ( b2Vec2 position, b2World * m_world, b2ParticleSystem * m_particleSystem) {	
	energy = 1.0f;
	uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_FOOD);
	bodyDef.userData = (void*)p_dataWrapper;
	bodyDef.type = b2_dynamicBody;
	p_body = m_world->CreateBody(&bodyDef);
	shape.SetAsBox(0.25f, 0.25f, position,0.0f);	
	p_body->CreateFixture(&shape, 4.0f);
	init = true;
	isUsed = true;
};

void addFoodParticle(b2Vec2 position, b2World * m_world, b2ParticleSystem * m_particleSystem) {
	food[currentNumberOfFood] = new foodParticle_t( position, m_world,  m_particleSystem);
	foodSlotLoaded[currentNumberOfFood] = true;
	currentNumberOfFood++;
}

void saveFishToFile(const std::string& file_name, fishDescriptor_t& data) {
  std::ofstream out(file_name.c_str());
  out.write(reinterpret_cast<char*>(&data), sizeof(fishDescriptor_t));
}

void loadFishFromFile(const std::string& file_name, fishDescriptor_t& data) {
  std::ifstream in(file_name.c_str());
  in.read(reinterpret_cast<char*>(&data), sizeof(fishDescriptor_t));
}

BonyFish::BonyFish(fishDescriptor_t driedFish, uint8_t fishIndex, b2World * m_world, b2ParticleSystem * m_particleSystem, fann * nann, b2Vec2 startingPosition) {
	genes = driedFish;
	hunger = 0.0f; // the animal spends energy to move and must replenish it by eating

	flagDelete = false;




	for (int i = 0; i < N_FINGERS; ++i) {

		if (i == 0) {
			driedFish.bones[i].isRoot = true;
		}

		bones[i] = new BoneUserData(driedFish.bones[i], this,  m_world, m_particleSystem, startingPosition);
	}

	n_bones_used = 0;
	for (int i = 0; i < N_FINGERS; ++i) {
		if (driedFish.bones[i].used) {
			n_bones_used ++;
		}
	}

	init = true; // true after the particle has been initialized. In most cases, uninitalized particles will be ignored.
	isUsed = false; // only true when the part is added to the world

	heartSpeed = driedFish.heartSpeed;
	if (heartSpeed < 1) {
		heartSpeed = 50;
	}

    if (nann == NULL) {
    	    unsigned int creationLayerCake[] = {
	    	12,
	    	8,
	    	8,
	    	8
	    };
	    	ann = fann_create_standard_array(4, creationLayerCake);
		    fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
		    fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);
		    fann_train_on_file(ann, "wormTrainer.data", max_epochs, epochs_between_reports, desired_error);
	    }
    else { // a brain is provided
    	ann = nann;
    }
    
    // make a random but sane file name. It would be better to have sequential, but i can't be assed to code it.
    // name = RNG() * 255;
    // std::string nnfilename =  std::to_string(name).c_str() + std::string(".net");
    // std::string fdescfilename =  std::to_string(name).c_str() + std::string(".fsh");
    // std::ofstream file { nnfilename };
    // saveFishToFile (fdescfilename, genes);
    // fann_save(ann, nnfilename.c_str()); 
};

// this describes the original 3 boned jellyfish.
fishDescriptor_t simpleJellyfish = {
	{
		{
				0,		// attachesTo
				0.175f,	// length
				0.015f,	// rootThickness
				0.01f,	// tipThickness
				true,	// isRoot
				true,	// isMouth
				false,	// isSensor
				false,	// isWeapon
				0.0f,	// torque
				0.0f,	// speedLimit
				0.0f,	// upperAngle
				0.0f,	// normalAngle
				0.0f,	// lowerAngle
				true
		},
		 {
				0,		// attachesTo
				0.15f,	// length
				0.015f,	// rootThickness
				0.01f,	// tipThickness
				false,	// isRoot
				false,	// isMouth
				true,	// isSensor
				false,	// isWeapon
				0.5f,	// torque
				10.0f,	// speedLimit
				(2* 3.1415) -0.075f,// upperAngle
				(2* 3.1415) -0.15f,	// normalAngle
				(2* 3.1415) -0.5f,	// lowerAngle
				true
		},
		{
				0,		// attachesTo
				0.15f,	// length
				0.015f,	// rootThickness
				0.01f,	// tipThickness
				false,	// isRoot
				false,	// isMouth
				true,	// isSensor
				false,	// isWeapon
				0.5f,	// torque
				10.0f,	// speedLimit
				0.50f,	// upperAngle
				0.15f,	// normalAngle
				0.075f,	// lowerAngle
				true
		}
	}
};





// this describes the original 3 boned jellyfish.
fishDescriptor_t koiCarp = {
	{
		{		// root mouth bone
				0,		// attachesTo
				0.2f,	// length
				0.1f,	// rootThickness
				0.3f,	// tipThickness
				true,	// isRoot
				true,	// isMouth
				false,	// isSensor
				false,	// isWeapon
				0.0f,	// torque
				0.0f,	// speedLimit
				0.0f,	// upperAngle
				0.0f,	// normalAngle
				0.0f,	// lowerAngle
				true
		},
		

		{		// body segment
				0,		// attachesTo
				0.5f,	// length
				0.3f,	// rootThickness
				0.5f,	// tipThickness
				false,	// isRoot
				false,	// isMouth
				false,	// isSensor
				false,	// isWeapon
				50.0f,	// torque
				10.0f,	// speedLimit
				pi * 1.0f * 0.5f,// upperAngle
				0.0f,	// normalAngle
				pi * -1.0f * 0.5f,	// lowerAngle
				true
		},

		//  {		// pec fin A
		// 		1,		// attachesTo
		// 		0.5f,	// length
		// 		0.1f,	// rootThickness
		// 		0.1f,	// tipThickness
		// 		false,	// isRoot
		// 		false,	// isMouth
		// 		true,	// isSensor
		// 		false,	// isWeapon
		// 		0.5f,	// torque
		// 		10.0f,	// speedLimit
		// 		(0.75f * pi) + (pi * 0.5f),// upperAngle
		// 		(0.75f * pi) + (0.0f),	// normalAngle
		// 		(0.75f * pi) + (pi * -0.5f),	// lowerAngle
		// 		true
		// },
		// {		// pec fin B
		// 		1,		// attachesTo
		// 		0.5f,	// length
		// 		0.1f,	// rootThickness
		// 		0.1f,	// tipThickness
		// 		false,	// isRoot
		// 		false,	// isMouth
		// 		true,	// isSensor
		// 		false,	// isWeapon
		// 		0.5f,	// torque
		// 		10.0f,	// speedLimit
		// 		(-0.75f * pi) + (pi * 0.5f),// upperAngle
		// 		(-0.75f * pi) + (0.0f),	// normalAngle
		// 		(-0.75f * pi) + (pi * -0.5f),	// lowerAngle
		// 		true
		// },

		{		// tail segment 1 segment
				1,		// attachesTo
				1.5f,	// length
				0.5f,	// rootThickness
				0.25f,	// tipThickness
				false,	// isRoot
				false,	// isMouth
				false,	// isSensor
				false,	// isWeapon
				50.0f,	// torque
				10.0f,	// speedLimit
				pi * 1.0f * 0.5f,// upperAngle
				0.0f,	// normalAngle
				pi * -1.0f * 0.5f,	// lowerAngle
				true
		},
		{		// tail segment 2 segment
				2,		// attachesTo
				1.5f,	// length
				0.25f,	// rootThickness
				0.1f,	// tipThickness
				false,	// isRoot
				false,	// isMouth
				false,	// isSensor
				false,	// isWeapon
				50.0f,	// torque
				10.0f,	// speedLimit
				pi * 1.0f * 0.5f,// upperAngle
				0.0f,	// normalAngle
				pi * -1.0f * 0.5f,	// lowerAngle
				true
		}//,//,,

		// {		// tail segment 1
		// 		1,		// attachesTo
		// 		0.15f,	// length
		// 		0.015f,	// rootThickness
		// 		0.01f,	// tipThickness
		// 		false,	// isRoot
		// 		false,	// isMouth
		// 		false,	// isSensor
		// 		false,	// isWeapon
		// 		0.5f,	// torque
		// 		10.0f,	// speedLimit
		// 		 0.5f,// upperAngle
		// 		 0.0f,	// normalAngle
		// 		-0.5f,	// lowerAngle
		// 		true
		// },
		// {		// tail segment 2
		// 		3,		// attachesTo
		// 		0.15f,	// length
		// 		0.015f,	// rootThickness
		// 		0.01f,	// tipThickness
		// 		false,	// isRoot
		// 		false,	// isMouth
		// 		false,	// isSensor
		// 		false,	// isWeapon
		// 		0.5f,	// torque
		// 		10.0f,	// speedLimit
		// 		0.5f,// upperAngle
		// 		0.0f,	// normalAngle
		// 		-0.5f,	// lowerAngle
		// 		true
		// }

	}
};



void moveAWholeFish (unsigned int fishIndex, b2Vec2 position) {
	if ( fishes[fishIndex] == NULL || fishes[fishIndex] == nullptr) { 	return; }
		if (fishSlotLoaded[fishIndex] ) {
			for (int i = 0; i < N_FINGERS; ++i)
			{
				fishes[fishIndex]->bones[i]->p_body->SetTransform(position, 0.0f);
			}
		}
}


void totalFishIncorporator (uint8_t fishIndex, b2World * m_world, b2ParticleSystem * m_particleSystem) {



	for (int i = 0; i < N_FINGERS; ++i) {
		if (fishes[fishIndex]->bones[i]->init) {
			nonRecursiveBoneIncorporator( fishes[fishIndex]->bones[i] , m_world, m_particleSystem, i);
		}
	}
}

// delete a fish from the game world and remove it from memory
void deleteFish (uint8_t fishIndex,  b2World * m_world, b2ParticleSystem * m_particleSystem) {




	if (fishSlotLoaded[fishIndex]) {

		printf("deleting %i of 8.", fishIndex);

		
		fishes[fishIndex]->init = false;
		fishes[fishIndex]->isUsed = false;

		fishSlotLoaded[fishIndex] = false;

		for (int i = 0; i < N_FINGERS; ++i) {

							if ( !fishes[fishIndex]->bones[i]->isUsed && !fishes[fishIndex]->bones[i]->init) {
								continue;
							}

		// 					else if (fishes[fishIndex]->bones[i]->init && !fishes[fishIndex]->bones[i]->isUsed) { // if the object is initialized, but not applied in the game world, you can still delete it from the game logic.
		// 						delete fishes[fishIndex]->bones[i]->joint;
		// 						delete fishes[fishIndex]->bones[i];
		// 					}

							else if (fishes[fishIndex]->bones[i]->isUsed && fishes[fishIndex]->bones[i]->init) {
								printf("deleting bone %i of 8.", i);


								try { m_world->DestroyBody(fishes[fishIndex]->bones[i]->p_body);} catch (...) {printf("a fuckup occurred while deleting something from the world\n");
}


								// m_world->DestroyBody(fishes[fishIndex]->bones[i]->p_body);

								fishes[fishIndex]->bones[i]->init = false;
								fishes[fishIndex]->bones[i]->isUsed = false;



								// delete fishes[fishIndex]->bones[i]->joint;
								// delete fishes[fishIndex]->bones[i];
							}
		}
		// delete fishes[fishIndex];	 // delete does, in fact, take a pointer

		// fishSlotLoaded[fishIndex] = false;

	}
}

void loadFish (uint8_t fishIndex, fishDescriptor_t driedFish, b2World * m_world, b2ParticleSystem * m_particleSystem, fann * nann, b2Vec2 startingPosition) {
	fishes[fishIndex] = new BonyFish(driedFish, fishIndex , m_world, m_particleSystem, nann, startingPosition);
	fishes[fishIndex]->slot = fishIndex;
	fishSlotLoaded[fishIndex] = true;
}

fann * loadFishBrainFromFile (std::string fileName) {
	return fann_create_from_file( (fileName + std::string(".net")).c_str() );
}

connectionDescriptor::connectionDescriptor () {
	isUsed = false;
	connectedTo = 0;
	connectionWeight = 0.0f;	
}

neuronDescriptor::neuronDescriptor() {
	n_connections = 0;
	n_inputs = 0;
	isUsed = false;
	for (int i = 0; i < 12; ++i)
	{
		connections[i] = connectionDescriptor();
	};
}

layerDescriptor::layerDescriptor () {
	n_neurons = 0;
	isUsed = false;
	for (int i = 0; i < 8; ++i)
	{
		neurons[i] = neuronDescriptor();
	};
}

// method to create a network descriptor in memory
networkDescriptor::networkDescriptor () {
	n_layers = 0;
	for (int i = 0; i < 8; ++i) {
		layers[i] = layerDescriptor();
	};
}

void unused_variable(void * bullshit) {
	; // do nothing
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

networkDescriptor  * createNeurodescriptorFromFANN (fann * temp_ann) {
	// making a descriptor from a file was too hard
	// you can make a FANN from the file and then query it for information you need to build the model.
	// std::string fileName = "225";

	// fann * temp_ann  = loadFishBrainFromFile (fileName);

	// query the number of layers.
	unsigned int num_layers = fann_get_num_layers(temp_ann);
	printf("new %u layer networkDescriptor\n", num_layers);

	 // get activation function information
  	unsigned int activation_function_hidden = 5; //fann::fann_get_activation_function_hidden(temp_ann);
  	float activation_steepness_hidden = 0.5f;//fann::fann_get_activation_steepness_hidden(temp_ann);
  	unsigned int activation_function_output = 0; //fann::fann_get_activation_function_output(temp_ann);
  	float activation_steepness_output = 0; //fann::fann_get_activation_steepness_output(temp_ann);
  	
	// get the layer cake.
	unsigned int layerCake[num_layers];

	// flip the cake 
	fann_get_layer_array(temp_ann, layerCake);



	// build everything in memory and link it together.
	networkDescriptor * newCake = new networkDescriptor();	//
  	newCake->n_layers = num_layers;
  	printf ("\ncreated network descriptor\n") ;

  	unsigned int sumOfNeurons = 0;
  	for (unsigned int i = 0; i < num_layers; ++i) {
  		sumOfNeurons += layerCake[i];
  		printf("a LAYER %i has %i neurons!\n", i, layerCake[i]);
  	}

  	for (unsigned int i = 0; i < num_layers; ++i) {
  		newCake->layers[i].n_neurons = layerCake[i];
  		// printf("LAYER %i has %i neurons!\n", i, layerCake[i]);

  		newCake->layers[i].isUsed = true;
  		printf ("created layer descriptor %i\n", layerCake[i]) ;

		for (unsigned int j = 0; j < layerCake[i]; ++j) {
  			// newCake.layers[i].neurons[j] = new neuronDescriptor();//*(new neuronDescriptor());
  			newCake->layers[i].neurons[j].activation_function = activation_function_hidden;
  			newCake->layers[i].neurons[j].activation_steepness = activation_steepness_hidden;
  			newCake->layers[i].neurons[j].n_connections = 0; // so not used uninitialized
  			newCake->layers[i].neurons[j].n_inputs = 0; // so not used uninitialized
  			newCake->layers[i].neurons[j].isUsed = true;

  			if (i == num_layers-1) {
				newCake->layers[i].neurons[j].activation_function = activation_function_output;
  				newCake->layers[i].neurons[j].activation_steepness = activation_steepness_output;
  			}
  			printf ("created neuron descriptor\n") ;
  		}
  	}

  	// get connection and weight information.
  	unsigned int num_connections = fann_get_total_connections(temp_ann);
  	static struct fann_connection margles[512] ;
  	memset(&margles, 0x00, sizeof(fann_connection[512]));
  	struct fann_connection *con = margles; 
  	fann_get_connection_array(temp_ann, con);

  	for (unsigned int c = 0; c < num_connections; ++c) {

  		// printf("alert: bush mong titties; %u %u\n", con[c].from_neuron, con[c].to_neuron );
// 
  		if (con[c].from_neuron < 0 || con[c].from_neuron > sumOfNeurons ) {
  			// unsigned int newDestination = RNG() * num_connections;
  			// con[c].from_neuron = newDestination;
  			continue;
  		}
  		if (con[c].to_neuron < 0 || con[c].to_neuron > sumOfNeurons ) {
  			// unsigned int newDestination = RNG() * num_connections;
  			// con[c].to_neuron = newDestination;
  			continue;
  		}
  	}



  	// create connections
  	for (unsigned int c = 0; c < num_connections; ++c) {
  		unsigned int layer = 0;
  		unsigned int index = con[c].from_neuron +1; // so as to not start from 0
  		while (1) {
  			if (index <= layerCake[layer]) {
  				break; }
  			else {
  				index -= layerCake[layer];
  				layer ++;
  			}
  		}
  		index--;
 
  		printf("%u %u ", con[c].to_neuron, con[c].from_neuron);
  		unsigned int newestConnectionIndex = newCake->layers[layer].neurons[index].n_connections;

  		if (newestConnectionIndex > sumOfNeurons) {
  			continue;
  		}

  		// printf("%u\n", newestConnectionIndex);
  		newCake->layers[layer].neurons[index].connections[newestConnectionIndex].connectedTo = con[c].to_neuron;
  		printf("%u ", newCake->layers[layer].neurons[index].connections[newestConnectionIndex].connectedTo);
  		newCake->layers[layer].neurons[index].connections[newestConnectionIndex].connectionWeight = con[c].weight;
  		newCake->layers[layer].neurons[index].connections[newestConnectionIndex].isUsed = true;
  		newCake->layers[layer].neurons[index].n_connections ++;
 
		unsigned int toLayer = 0;
  		unsigned int toIndex = con[c].from_neuron +1; // so as to not start from 0
  		
  		while (1) {
  			if (toIndex <= layerCake[toLayer]) {
  				break; }
  			else {
  				toIndex -= layerCake[toLayer];
  				toLayer ++;
  			}
  		}
  		toIndex--;

		newCake->layers[toLayer].neurons[toIndex].n_inputs ++;
		printf ("created connection descriptor f%u t%u w%f, %u of %u\n", con[c].from_neuron, con[c].to_neuron, con[c].weight, c, num_connections-1) ;	
	}
return newCake;
}

// from: https://stackoverflow.com/questions/7132957/c-scientific-notation-format-number
void my_print_scientific(char *dest, double value) {
    snprintf(dest, 28, "%.20e", value); // 20 digits between the decimal place and the e
}

// method to create a fann save file from a network descriptor
void createFANNFileFromDescriptor (networkDescriptor network) {

	printf("createFANNFileFromDescriptor\n");

	std::string s = std::string("FANN_FLO_2.1\nnum_layers=");; // string to hold the information.

	s.append(std::to_string(network.n_layers));

	// print this
 	s.append("\nlearning_rate=0.700000\nconnection_rate=1.000000\nnetwork_type=0\nlearning_momentum=0.000000\ntraining_algorithm=2\ntrain_error_function=1\ntrain_stop_function=0\ncascade_output_change_fraction=0.010000\nquickprop_decay=-0.000100\nquickprop_mu=1.750000\nrprop_increase_factor=1.200000\nrprop_decrease_factor=0.500000\nrprop_delta_min=0.000000\nrprop_delta_max=50.000000\nrprop_delta_zero=0.100000\ncascade_output_stagnation_epochs=12\ncascade_candidate_change_fraction=0.010000\ncascade_candidate_stagnation_epochs=12\ncascade_max_out_epochs=150\ncascade_min_out_epochs=50\ncascade_max_cand_epochs=150\ncascade_min_cand_epochs=50\ncascade_num_candidate_groups=2\nbit_fail_limit=3.49999994039535522461e-01\ncascade_candidate_limit=1.00000000000000000000e+03\ncascade_weight_multiplier=4.00000005960464477539e-01\ncascade_activation_functions_count=10\ncascade_activation_functions=3 5 7 8 10 11 14 15 16 17 \ncascade_activation_steepnesses_count=4\ncascade_activation_steepnesses=2.50000000000000000000e-01 5.00000000000000000000e-01 7.50000000000000000000e-01 1.00000000000000000000e+00\n");
 	s.append("layer_sizes=");

 	// print layer sizes separated by a space
	for (unsigned int i = 0; i < network.n_layers; ++i) {
		s.append(std::to_string(network.layers[i].n_neurons));
		s.append(" ");
	}

	printf("print activation information\n");

	// print activation information
 	s += "\nscale_included=0\nneurons (num_inputs, activation_function, activation_steepness)=";
 	for (unsigned int i = network.n_layers-1; i > 0; --i)	{
 		for (unsigned int j = 0; j<network.layers[i].n_neurons ; ++j) {
 			char chalkboard[9];

 			printf("%i of %i\n", j, network.layers[i].n_neurons);

 			// one of the layers in the file is supposed to have 0 inputs.
 			if (i == 0) {
					sprintf(chalkboard, "(%u, %u, ", 0, 0);	
 			s += chalkboard;
 			}
 			else {
 					sprintf(chalkboard, "(%u, %u, ", network.layers[i].neurons[j].n_inputs, network.layers[i].neurons[j].activation_function);	
	 			s += chalkboard;
 			}

 			
 			char sciNotationBuffer[] = "0.00000000000000000000e+00";
 			my_print_scientific(sciNotationBuffer, network.layers[i].neurons[j].activation_steepness);
 			s += sciNotationBuffer;

 			s.append(") ");
 		}
 	}

 	printf("print connection information\n");
 	// 
	s += "\nconnections (connected_to_neuron, weight)=";
	for (unsigned int i = network.n_layers-1; i > 0; --i) 	{
 		for (unsigned int j = 0; j < network.layers[i].n_neurons;  ++j) {
 			for (unsigned int k = 0; k < network.layers[i].neurons[j].n_connections; ++k) {

 				// if (k >= 8) {
 				// 	break;
 				// }

 				char chalkboard[5];

 				printf(" |%u|, ", network.layers[i].neurons[j].connections[k].connectedTo); // <- it is already fucked up here.

 				sprintf(chalkboard, "(%u, ", network.layers[i].neurons[j].connections[k].connectedTo);
 				s += chalkboard;
 				char sciNotationBuffer[30];
	 			my_print_scientific(sciNotationBuffer, network.layers[i].neurons[j].connections[k].connectionWeight);
	 			s+= sciNotationBuffer;
	 			s.append(") ");
 			}
 		}
 	}

 	printf("print to file\n");

    std::ofstream out("mutantGimp.net");
    out << s;
    out.close();
}

void mutateFishBrain (networkDescriptor * newCake, float mutationChance, float mutationSeverity) {


	// count connections.
	unsigned int sumOfConnections = 0;
	for (unsigned int i = 0; i < newCake->n_layers; ++i) 	{
		
 		for (unsigned int j = 0; j < newCake->layers[i].n_neurons; ++j) {



			if (newCake->layers[i].neurons[j].n_connections > 8) {
				continue;
			}
			if (i > newCake->n_layers) {
				continue;
			}
			if (j > newCake->layers[i].n_neurons) {
				continue;
			}
			// if (n > newCake->layers[l].neurons[m].n_connections) {
			// 	continue;
			// }



 			sumOfConnections += newCake->layers[i].neurons[j].n_connections;
 			// for (unsigned int k = 0; k < newCake->layers[i].neurons[j].n_connections; ++k) {

 			// }
 		}

 	}

 	printf("sum of connections: %i\n", sumOfConnections);

 	// while(1) {;}
	// traverse the mutated body and count the total number of sensors and hearts and limbs.
	// make sure the number of inputs matches the number of sensors and hearts
	// make sure the number of outputs matches the number of joint motors

	// chance to grow a new layer
	// chance to lose a layer

	// chance to grow a new neuron
	// chance to lose a neuron

	// chance to grow a new connection
	// chance to lose a connection

	// chance to modify the weight of an existing connection .. s
	for (unsigned int i = 0; i < sumOfConnections; ++i)
	{
		if ( RNG() < mutationChance) {
			unsigned int l =  RNG() * newCake->n_layers ;
			unsigned int m =  RNG() * newCake->layers[l].n_neurons ;
			unsigned int n =  RNG() * newCake->layers[l].neurons[m].n_connections ;

			if (newCake->layers[l].neurons[m].n_connections > 8) {
				continue;
			}

			printf("max l:%u m:%u n:%u", newCake->n_layers ,  newCake->layers[l].n_neurons, newCake->layers[l].neurons[m].n_connections );

			float mutationAmount = ((RNG() -0.5) *mutationSeverity  );

			if (l > newCake->n_layers) {
				continue;
			}
			if (m > newCake->layers[l].n_neurons) {
				continue;
			}
			if (n > newCake->layers[l].neurons[m].n_connections) {
				continue;
			}

			newCake->layers[l].neurons[m].connections[n].connectionWeight += mutationAmount;

			printf("warped layer %u neuron %u connection %u by %f\n", l, m, n,  mutationAmount);

		}
	}
	
	// chance to change the target of an existing connection
	
}

void mutateFishDescriptor (fishDescriptor_t * fish, float mutationChance, float mutationSeverity) {

	// mutate heart rate
	if (RNG() > mutationChance) {	fish->heartSpeed += fish->heartSpeed *mutationSeverity*(RNG()-0.5); }

	for (int i = 0; i < N_FINGERS; ++i)
	{
		if (fish->bones[i].used) {

			// mutate floats
			if (RNG() > mutationChance) {	fish->bones[i].length += fish->bones[i].length 				*mutationSeverity*(RNG()-0.5); }
			if (RNG() > mutationChance) {	fish->bones[i].rootThickness += fish->bones[i].rootThickness *mutationSeverity*(RNG()-0.5); }
			if (RNG() > mutationChance) {	fish->bones[i].tipThickness += fish->bones[i].tipThickness 	*mutationSeverity*(RNG()-0.5); }
			if (RNG() > mutationChance) {	fish->bones[i].torque += fish->bones[i].torque 				*mutationSeverity*(RNG()-0.5); }
			if (RNG() > mutationChance) {	fish->bones[i].speedLimit += fish->bones[i].speedLimit 		*mutationSeverity*(RNG()-0.5); }
			if (RNG() > mutationChance) {	fish->bones[i].upperAngle += fish->bones[i].upperAngle 		*mutationSeverity*(RNG()-0.5); }
			if (RNG() > mutationChance) {	fish->bones[i].lowerAngle += fish->bones[i].lowerAngle 		*mutationSeverity*(RNG()-0.5); }

			// mutate attachment points
			// if (RNG() > mutationChance) {	fish->bones[i].attachedTo = (RNG() * fish->n_bones_used ) }

			// mutate bools
			if (RNG() > mutationChance) {	fish->bones[i].isMouth = !fish->bones[i].isMouth; }
			if (RNG() > mutationChance) {	fish->bones[i].isSensor = !fish->bones[i].isSensor; }

		}
	}
}

// void conformBrainToNewAnimal (networkDescriptor * newCake, fishDescriptor_t * newFish) {
// 	// ok, so, what this does is
// 	// updates a neurodescriptor to match the number of sensors, hearts, and motors on an animal.

// 	// count the number of each thing on the animal.

// 	uint8_t num_motors = 0;
// 	uint8_t num_sensors = 0;

// 	for (int i = 0; i < newFish->n_bones_used; ++i)
// 	{
// 		if (newFish->bones[i].isSensor) {
// 			num_sensors ++;
// 		}

// 		// all bones are motor outputs now
// 		num_motors ++ ;

// 		// if (newFish->bones[i].isSensor) {

// 		// }
// 	}



	// compare it to the input and output layer sizes of the ANN.

	// if () {

	// }

	/**

	DEVELOPMENT NOT COMPLETE!
	**/


// }

void LoadFishFromName (uint8_t fishIndex, b2World * m_world, b2ParticleSystem * m_particleSystem) {

	fishDescriptor_t newFish;

	std::string fileName = "225";

	loadFishFromFile(fileName + std::string(".fsh"), newFish);

	mutateFishDescriptor (&newFish, 0.1, 0.1) ;

	fann *wann = loadFishBrainFromFile (fileName) ;	

	networkDescriptor * tamberlina = createNeurodescriptorFromFANN(wann);
	
	mutateFishBrain(tamberlina, 0.1f, 1.0f );

	createFANNFileFromDescriptor(*tamberlina);

	fann * ann  = loadFishBrainFromFile ("mouptut"); // load the mutated file

	bool loadWithBlankBrain = true;
	if (loadWithBlankBrain) {
		loadFish ( fishIndex,  newFish, m_world,  m_particleSystem, NULL, b2Vec2(0.0f, 0.0f) ) ;
	}
	else {
		loadFish ( fishIndex,  newFish, m_world,  m_particleSystem, ann , b2Vec2(0.0f, 0.0f)) ;
	}

 	

}





void wormTrainer () {

	int n_examples = 1000;



	// float inputValues[12]; 
	// float outputValues[8];


	// float bodysegments = 4;

	FILE *fp;
    fp = fopen("wormTrainer.data","wb");

    fprintf(fp, "%i %i% i\n", n_examples*4, 12, 8 );


	// for (int i = 0; i < 12; ++i)
	// {
	// 	inputValues[i] = 0.0f;
	// }
	// for (int i = 0; i < 8; ++i)
	// {
	// 	outputValues[i] = 0.0f;
	// }


	for (int i = 0; i < 2*n_examples; ++i)
	{
		

		float senseDiffPerSegment = RNG() * 0.25;

		float noize = RNG() * 0.5;

		// four samples that show a swimming motion when the food is ahead
		// ------- SEQ 1
	
		// four samples that show a swimming motion when the food is ahead
		// ------- SEQ 1
		fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f\n", 
			4*senseDiffPerSegment + (RNG() * noize),
			3*senseDiffPerSegment + (RNG() * noize),
			2*senseDiffPerSegment + (RNG() * noize),
			1*senseDiffPerSegment + (RNG() * noize),
			(RNG() * noize),(RNG() * noize),(RNG() * noize),(RNG() * noize),
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
		fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f\n", 
			4*senseDiffPerSegment + (RNG() * noize),
			3*senseDiffPerSegment + (RNG() * noize),
			2*senseDiffPerSegment + (RNG() * noize),
			1*senseDiffPerSegment + (RNG() * noize),
			0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),
			0 + (RNG() * noize),0 + (RNG() * noize),1 + (RNG() * noize),0 + (RNG() * noize)
			);

		fprintf(fp, "%f %f %f %f %f %f %f %f\n", 
			-1 + (RNG() * noize),
			0 + (RNG() * noize),
			0 + (RNG() * noize),
			1 + (RNG() * noize),
			0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize)
			);
		// -------- SEQ 3
		
fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f\n", 
			4*senseDiffPerSegment + (RNG() * noize),
			3*senseDiffPerSegment + (RNG() * noize),
			2*senseDiffPerSegment + (RNG() * noize),
			1*senseDiffPerSegment + (RNG() * noize),
			0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),
			0 + (RNG() * noize),1 + (RNG() * noize),0 + (RNG() * noize),1 + (RNG() * noize)
			);

		fprintf(fp, "%f %f %f %f %f %f %f %f\n", 
			1 + (RNG() * noize),
			0 + (RNG() * noize),
			0 + (RNG() * noize),
			-1 + (RNG() * noize),
			0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize)
			);
		// -------- SEQ 4
		
fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f\n", 
			4*senseDiffPerSegment + (RNG() * noize),
			3*senseDiffPerSegment + (RNG() * noize),
			2*senseDiffPerSegment + (RNG() * noize),
			1*senseDiffPerSegment + (RNG() * noize),
			0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),0 + (RNG() * noize),
			0 + (RNG() * noize),1 + (RNG() * noize),1 + (RNG() * noize),0 + (RNG() * noize)
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


void removeDeletableFish() {
	for (int i = 0; i < N_FISHES; ++i)
	{
		// if ( !fishSlotLoaded[winner->slot]) {
		// 	return;
		// }
		// else {
		if ( fishes[i] == NULL || fishes[i] == nullptr) { 	continue; }
		// if ( !fishes[i]->isUsed || !fishes[i]->init) { 		continue; } else if (fishes[i]->isUsed && fishes[i]->init) {

			if (fishes[i]->flagDelete) {
				deleteFish (i, local_m_world, local_m_particleSystem) ;
			}

			// for (int i = 0; i < N_FISHES; ++i)
			// {
				
			// }
 
		// }
	}
}



//  prints the winner to file immediately.
void  vote (BonyFish * winner) {
if ( !fishSlotLoaded[winner->slot]) {
		return;
	}
	else {

		printf("winner: %i\n", winner->slot);

		// printf("winner: %i\n", winner->slot);

	fann * wann = winner->ann;


	// save the winner to file with a new name.
	std::string nnfilename =  std::string("mostCurrentWinner.net");
    // std::string fdescfilename =  std::string("225.fsh");
    std::ofstream file { nnfilename };
    // saveFishToFile (fdescfilename, ((BoneUserData *)(dataB.uData))->p_owner->genes);
    fann_save(  wann , nnfilename.c_str()); 

	}



	// label every fish as delete.
	for (int i = 0; i < N_FISHES; ++i)
	{

		if ( fishes[i] == NULL || fishes[i] == nullptr) { 	continue; }
		// if ( !fishes[i]->isUsed || !fishes[i]->init) { 		continue; } else if (fishes[i]->isUsed && fishes[i]->init) {
			fishes[i]->flagDelete = true;
		// }
	}


	startNextGeneration = true;
}


void printConnectionArrayForDebug (networkDescriptor * network) {
	printf(" printConnectionArrayForDebug: %i layers\n", network->n_layers);

	for (unsigned int i = 0; i < network->n_layers; ++i) 	{
		printf(" layer %i neurons: %i\n", i, network->layers[i].n_neurons);

 		for (unsigned int j = 0; j < network->layers[i].n_neurons; ++j) {

 			printf(" neuron %i connections: %i\n", j, network->layers[i].neurons[j].n_connections);
 			for (unsigned int k = 0; k < network->layers[i].neurons[j].n_connections; ++k) {

 				// if (k >= 8) {
 				// 	continue;
 				// }

 				// char chalkboard[5];

 				printf(" |%u|, ", network->layers[i].neurons[j].connections[k].connectedTo); // <- it is already fucked up here.
			}
		}
	}
	printf("\n");
}

void mutateFANNFileDirectly () {

std::ofstream outFile("mutantGimp.net");
 std::string line;

 std::ifstream inFile("mostCurrentWinner.net");
 int count = 0;
 int amountCount = 0;
 while(getline(inFile, line)){

 	if (count == 35) { // its that crazy line.

 		// int countUp = 0;
 
 		outFile << std::string("neurons (num_inputs, activation_function, activation_steepness)=");

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
	 				// charCount ++;

					char sciNumber[27];
					memset(sciNumber, 0x00, 27);
					memcpy(sciNumber, &c, 27);

					// if (sciNumber[26] == ')') {
					// 	sciNumber[26] = '0';	
					// }

					printf("%s\n", sciNumber);
				    float val = std::stof(sciNumber); 

			    	char sciNotationBuffer[27];// = "0.00000000000000000000e+00";
		 			my_print_scientific(sciNotationBuffer,val);
		 			outFile << sciNotationBuffer;
		 			outFile << ") ";

		 			amountCount ++;

		 			if (amountCount >= 248) {
		 				return;
		 			}

		 			skipTheRest = true;
	 			}
	 			
			}
			else  {

				if (c == ' ') {
					skipTheRest = false;
				}

				// if (countUp >= 27) {
				// 	countUp = 0;
				// 	// gimmeSomeSpace = false;
				// 	skipTheRest = false;
				// }

				// countUp ++;

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

			// //copy a file.
			// //https://stackoverflow.com/questions/10195343/copy-a-file-in-a-sane-safe-and-efficient-way
			// std::ifstream  src("mostCurrentWinner.net", std::ios::binary);
		 //    std::ofstream  dst("muntante.net",   std::ios::binary);
	// 	 //    dst << src.rdbuf();

 //    printf("mutating FANN file");


	// // FILE *fp;
 // //    fp = fopen("muntante.net","wb");

 //    FILE *rp;
 //    rp = fopen("mostCurrentWinner.net","r");

 //    std::string messyBumFuck = std::string("");

 //    // fprintf(fp, "FANN_FLO_2.1\nnum_layers=4\nlearning_rate=0.700000\nconnection_rate=1.000000\nnetwork_type=0\nlearning_momentum=0.000000\ntraining_algorithm=2\ntrain_error_function=1\ntrain_stop_function=0\ncascade_output_change_fraction=0.010000\nquickprop_decay=-0.000100\nquickprop_mu=1.750000\nrprop_increase_factor=1.200000\nrprop_decrease_factor=0.500000\nrprop_delta_min=0.000000\nrprop_delta_max=50.000000\nrprop_delta_zero=0.100000\ncascade_output_stagnation_epochs=12\ncascade_candidate_change_fraction=0.010000\ncascade_candidate_stagnation_epochs=12\ncascade_max_out_epochs=150\ncascade_min_out_epochs=50\ncascade_max_cand_epochs=150\ncascade_min_cand_epochs=50\ncascade_num_candidate_groups=2\nbit_fail_limit=3.49999994039535522461e-01\ncascade_candidate_limit=1.00000000000000000000e+03\ncascade_weight_multiplier=4.00000005960464477539e-01\ncascade_activation_functions_count=10\ncascade_activation_functions=3 5 7 8 10 11 14 15 16 17 \ncascade_activation_steepnesses_count=4\ncascade_activation_steepnesses=2.50000000000000000000e-01 5.00000000000000000000e-01 7.50000000000000000000e-01 1.00000000000000000000e+00 \nlayer_sizes=13 9 9 9 \nscale_included=0\nneurons (num_inputs, activation_function, activation_steepness)=(0, 0, 0.00000000000000000000e+00) (0, 0, 0.00000000000000000000e+00) (0, 0, 0.00000000000000000000e+00) (0, 0, 0.00000000000000000000e+00) (0, 0, 0.00000000000000000000e+00) (0, 0, 0.00000000000000000000e+00) (0, 0, 0.00000000000000000000e+00) (0, 0, 0.00000000000000000000e+00) (0, 0, 0.00000000000000000000e+00) (0, 0, 0.00000000000000000000e+00) (0, 0, 0.00000000000000000000e+00) (0, 0, 0.00000000000000000000e+00) (0, 0, 0.00000000000000000000e+00) (13, 5, 5.00000000000000000000e-01) (13, 5, 5.00000000000000000000e-01) (13, 5, 5.00000000000000000000e-01) (13, 5, 5.00000000000000000000e-01) (13, 5, 5.00000000000000000000e-01) (13, 5, 5.00000000000000000000e-01) (13, 5, 5.00000000000000000000e-01) (13, 5, 5.00000000000000000000e-01) (0, 5, 0.00000000000000000000e+00) (9, 5, 5.00000000000000000000e-01) (9, 5, 5.00000000000000000000e-01) (9, 5, 5.00000000000000000000e-01) (9, 5, 5.00000000000000000000e-01) (9, 5, 5.00000000000000000000e-01) (9, 5, 5.00000000000000000000e-01) (9, 5, 5.00000000000000000000e-01) (9, 5, 5.00000000000000000000e-01) (0, 5, 0.00000000000000000000e+00) (9, 5, 5.00000000000000000000e-01) (9, 5, 5.00000000000000000000e-01) (9, 5, 5.00000000000000000000e-01) (9, 5, 5.00000000000000000000e-01) (9, 5, 5.00000000000000000000e-01) (9, 5, 5.00000000000000000000e-01) (9, 5, 5.00000000000000000000e-01) (9, 5, 5.00000000000000000000e-01) (0, 5, 0.00000000000000000000e+00) \nconnections (connected_to_neuron, weight)="  );




	// // open the FANN file line 35

	// goToLine(rp, 36);

	// // // advanced forward to the first closing bracket
	// seekUntil(rp, '=');

	// for (unsigned int i = 0; i < 10; ++i)
	// {
 //   printf("%i\n", i);

	// 	/* code */
	// 	seekUntil(rp, '(');
	// 	// advanceCursor(5);


	// 	for (int j = 0; j < 5; ++j)
	// 	{
	// 		/* code */
	// 		// fprintf(fp, "%c", fgetc(rp));
	// 		messyBumFuck += fgetc(rp);
	// 	}



	// 	char sciNumber[27];
	// 	memset(sciNumber, 0x00, 27);




	// 	for (unsigned int j = 0; j < 27; ++j)
	// 	{
	// 		// char c;
	// sciNumber[i] = fgetc(rp);

	// 	}

	// 	// std::string str = "100.80"; 
  
 //    // val to store parsed floating type number 
 //    float val = std::stof(sciNumber); 



 //    // (mutate the number 'val')


 //    	char sciNotationBuffer[27];// = "0.00000000000000000000e+00";
 // 			my_print_scientific(sciNotationBuffer,val);
 // 			messyBumFuck += sciNotationBuffer;
 // 			// fprintf(fp, "%s)", sciNotationBuffer);



		

	// }

	// // // the following code is repeated until end of line?
	// 

	// bool thisWeightIsNegative = false;

	// if (<the character> == '-') {
	// 	thisWeightIsNegative = true;
	// } 




}

void beginGeneration ( b2World * m_world, b2ParticleSystem * m_particleSystem) { // select an animal as an evolutionary winner, passing its genes on to the next generation


	// select the fish you clicked
	// fishDescriptor_t winnerSyrup = body->userData->uData->p_owner->genes;

	// uDataWrap * myUserDataStruct = (uDataWrap *)body->m_userData;

	// BonyFish * winner = (BonyFish *)myUserDataStruct->uData;

	
	// if ( !fishSlotLoaded[winner->slot]) {
	// 	return;
	// }
	// else {


	// 	printf("winner: %i\n", winner->slot);

	// fann * wann = winner->ann;



	// // save the winner to file with a new name.
	// std::string nnfilename =  std::string("mostCurrentWinner.net");
 //    // std::string fdescfilename =  std::string("225.fsh");
 //    std::ofstream file { nnfilename };
 //    // saveFishToFile (fdescfilename, ((BoneUserData *)(dataB.uData))->p_owner->genes);
 //    fann_save(  wann , nnfilename.c_str()); 

	// destroy all creatures in the world and delete them from memory. make sure their udata is deleted too.

	removeDeletableFish();
	// for (int i = 0; i < N_FISHES; ++i)
	// {
	// 	deleteFish (i, m_world, m_particleSystem) ;
	// }
 
	// create 8 mutant copies of the winner

		
	for (int i = 0; i < 8; ++i)
	{
		

		bool thereIsAFile = false;
		b2Vec2 positionalRandomness = b2Vec2(  (RNG()-0.5) * 15, (RNG()-0.5) * 15  );



		 if (FILE *file = fopen("mostCurrentWinner.net", "r")) { //if (FILE *file = fopen(name.c_str(), "r")) {
	        fclose(file);
	        thereIsAFile = true;
	    } 


		if (thereIsAFile ) {
			// fann *wann;
			// wann = loadFishBrainFromFile (std::string("mostCurrentWinner")) ;
			// thereWasAFile = true;


		// first load the winner as a fann ANN.
		// fann *wann = loadFishBrainFromFile (std::string("mostCurrentWinner")) ;

		// then make a descriptor of it.
		// networkDescriptor * mutantGimp = createNeurodescriptorFromFANN(wann);
		// printf("check 01\n");
		// printConnectionArrayForDebug (mutantGimp);

		// then you can mutate the descriptor.
		// mutateFishBrain(mutantGimp, 0.1f, 0.1f);


		// printf("check 02\n");	printConnectionArrayForDebug (mutantGimp);

		// now you have to save it as a fann file, which is stupid, but at least you can do it in a temporary file.
		// createFANNFileFromDescriptor (*mutantGimp) ;




		    // mutateFANNFileDirectly();


		// now you can load the mutant ANN.
		fann *mann = loadFishBrainFromFile (std::string("mutantGimp")) ;

		
		loadFish (i, koiCarp, m_world, m_particleSystem, mann, positionalRandomness) ;

		}
		else {
		// b2Vec2 positionalRandomness = b2Vec2(  (RNG()-0.5) * 15, (RNG()-0.5) * 15  );
		loadFish (i, koiCarp, m_world, m_particleSystem, NULL, positionalRandomness) ;

		}

		// if (thereWasAFile) {

			
		// }
		// else {

			
		// }

		// unused_variable((void*) mann);

		totalFishIncorporator(i, m_world, m_particleSystem);


		// b2Vec2 positionalRandomness = b2Vec2(  (RNG()-0.5) * 5, (RNG()-0.5) * 5  );

		// moveAWholeFish (i, positionalRandomness) ;

	}



	// spawn them into the world to repeat the cycle

	startNextGeneration = false;
// }


}

inline bool exists_test1 (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }   
}



void deepSeaSetup (b2World * m_world, b2ParticleSystem * m_particleSystem, DebugDraw * p_debugDraw) {

	// jellyfishTrainer();

	// wormTrainer();

	// store the debugdraw pointer in here so we can use it.
	local_debugDraw_pointer = p_debugDraw;
	local_m_world = m_world;
	local_m_particleSystem = m_particleSystem;

	addFoodParticle(b2Vec2(2.5f, 3.5f), m_world, m_particleSystem);

	// beginGeneration ( local_m_world,local_m_particleSystem);
	startNextGeneration = true;


	// int howManyNewFishToAdd = 3;
	// for (int i = 0; i < howManyNewFishToAdd; ++i) {


	// 	//  loadFishFromName is used to load the named files associated with the evolution system.
	// 	// LoadFishFromName(i, m_world, m_particleSystem);


	// 	// this one is good to just load in a desfault fish from the descriptor.

	// 	if (exists_test1("mostCurrentWinner.net")) {
	// 		fann * ann  = loadFishBrainFromFile (std::string("mostCurrentWinner")); // unfortunately you still need to load some kind of brain or it wont work.
	// 		loadFish ( i,  koiCarp, m_world,  m_particleSystem, ann , b2Vec2(0.0f, 0.0f)) ;



	// 		totalFishIncorporator(i, m_world, m_particleSystem);
	// 	}

		
	// }
}

void drawNeuralNetwork(struct 	fann 	*	ann	, float * motorSignals, float * sensorium, int index, unsigned int * spacesUsedSoFar) {

	// get the number of layers. FANN_EXTERNAL unsigned int FANN_API fann_get_num_layers(	struct 	fann 	*	ann	)
	unsigned int n_layers = fann_get_num_layers(ann);

	unsigned int sizeOfBiggestLayer = 0;

	// get the number of neurons on each layer.
	unsigned int layerArray[n_layers];
	fann_get_layer_array(ann,layerArray);
 
    struct fann_connection *con;   /* weight matrix */
    unsigned int connum;           /* connections number */
    size_t i;

	connum = fann_get_total_connections(ann);
    if (connum == 0) {
        fprintf(stderr, "Error: connections count is 0\n");
    }

    con = (fann_connection *)calloc(connum, sizeof(*con));
    if (con == NULL) {
        fprintf(stderr, "Error: unable to allocate memory\n");
    }

    /* Get weight matrix */
    fann_get_connection_array(ann, con);

    // int icompatiblespaces = spacesUsedSoFar
    float fcompatiblespaces = *spacesUsedSoFar;
	b2Vec2 drawingStartingPosition = b2Vec2(  fcompatiblespaces + 1 ,4.0f);
	float spacingDistance = 0.5f;


	// compute the size of biggest layer so you know how far to move over all subsequent layers.
	for (uint8_t j = 0; j < n_layers; ++j)
	{
		if (layerArray[j] > sizeOfBiggestLayer) {
			sizeOfBiggestLayer = layerArray[j];
		}
	}
	*spacesUsedSoFar += sizeOfBiggestLayer;


	for (uint8_t j = 0; j < layerArray[0]; ++j) {
		b2Vec2 neuron_position = b2Vec2(drawingStartingPosition.x +j * spacingDistance,drawingStartingPosition.y );

		local_debugDraw_pointer->DrawPoint(neuron_position, 8.0f, b2Color( sensorium[j], sensorium[j], sensorium[j]));
	}

	for (uint8_t j = 0; j < layerArray[n_layers-1]; ++j) {
		b2Vec2 neuron_position = b2Vec2(drawingStartingPosition.x +j * spacingDistance,(drawingStartingPosition.y + ((n_layers-1) * spacingDistance)));
		local_debugDraw_pointer->DrawPoint(neuron_position, 8.0f, b2Color( motorSignals[j]+0.5f, motorSignals[j]+0.5f, motorSignals[j]+0.5f));
	}

    /* Print weight matrix */
    for (i = 0; i < connum; ++i) {
    	b2Vec2 printConnectionSideA;
    	b2Vec2 printConnectionSideB;

    	uint8_t neuronA_tally = con[i].from_neuron;
    	for (uint8_t j = 0; j< n_layers; ++j) {
    		if (neuronA_tally >= layerArray[j]) {
    			neuronA_tally -= layerArray[j];
    		}	
    		else {
    			printConnectionSideA = b2Vec2(drawingStartingPosition.x + neuronA_tally * spacingDistance, drawingStartingPosition.y +j * spacingDistance);
    			break;
    		}
    		neuronA_tally --;
    	}
    	uint8_t neuronB_tally = con[i].to_neuron;
    	for (uint8_t j = 0; j< n_layers; ++j) {
    		if (neuronB_tally >= layerArray[j]) {
    			neuronB_tally -= layerArray[j];
    		}	
    		else {
    			printConnectionSideB = b2Vec2(drawingStartingPosition.x + neuronB_tally * spacingDistance, drawingStartingPosition.y +j * spacingDistance);
    			break;
    		}
    		neuronB_tally --;
    	}

    	b2Color segmentColor = b2Color(con[i].weight*10,con[i].weight*10,con[i].weight*10);
	    local_debugDraw_pointer->DrawSegment(printConnectionSideA, printConnectionSideB,segmentColor );
    }
    free(con);
}

void deepSeaLoop () {

	if (startNextGeneration) {
		beginGeneration ( local_m_world,local_m_particleSystem);
	}

	for  (int i = 0; i < N_FOODPARTICLES; i++) {
		if (!foodSlotLoaded[i]) {
			break;
		}
		else {
			food[i]->position = food[i]->p_body->GetPosition(); // update positions of all the food particles
		}
	}

	unsigned int spacesUsedSoFar =0;

	for (int i = 0; i < N_FISHES; ++i) {
		if (fishSlotLoaded[i]) {
			

				// cause heart to beat. Heart A is the slowest
				if (fishes[i]->heartCountA > fishes[i]->heartSpeed) {
					fishes[i]->heartCountA = 0;
					if (fishes[i]->heartOutputA > 0) { fishes[i]->heartOutputA = -1; }
					else { fishes[i]->heartOutputA = 1; }
				}
				else { fishes[i]->heartCountA++; }

				if (fishes[i]->heartCountB > fishes[i]->heartSpeed/2) {
					fishes[i]->heartCountB = 0;
					if (fishes[i]->heartOutputB > 0) { fishes[i]->heartOutputB = -1; }
					else { fishes[i]->heartOutputB = 1; }
				}
				else { fishes[i]->heartCountB++; }

				if (fishes[i]->heartCountC > fishes[i]->heartSpeed/4) {
					fishes[i]->heartCountC = 0;
					if (fishes[i]->heartOutputC > 0) { fishes[i]->heartOutputC = -1; }
					else { fishes[i]->heartOutputC = 1; }
				}
				else { fishes[i]->heartCountC++; }

				if (fishes[i]->heartCountD > fishes[i]->heartSpeed/8) {
					fishes[i]->heartCountD = 0;
					if (fishes[i]->heartOutputD > 0) { fishes[i]->heartOutputD = -1; }
					else { fishes[i]->heartOutputD = 1; }
				}
				else { fishes[i]->heartCountD++; }






				// update the fish's senses
				for (int j = 0; j < N_FINGERS; ++j) {
					nonRecursiveSensorUpdater (fishes[i]->bones[j]);
				}

				// eight motors and four timing inputs
				float sensorium[12] = {
						fishes[i]->bones[0]->sensation, 
						fishes[i]->bones[1]->sensation,
						fishes[i]->bones[2]->sensation,
						fishes[i]->bones[3]->sensation,
						fishes[i]->bones[4]->sensation,
						fishes[i]->bones[5]->sensation,
						fishes[i]->bones[6]->sensation,
						fishes[i]->bones[7]->sensation,

						(float)fishes[i]->heartOutputA,
						(float)fishes[i]->heartOutputB,
						(float)fishes[i]->heartOutputC,
						(float)fishes[i]->heartOutputD};


				// printf("sense: %.2f %.2f\n", fishes[i]->bones[1]->sensation, fishes[i]->bones[2]->sensation);
				// feed information into brain
				float * motorSignals = fann_run(fishes[i]->ann, sensorium);

				// printf("motor: %.2f %.2f\n", motorSignals[0], motorSignals[1]);//, motorSignals[2], motorSignals[3]);

				if (true) {
					// float jointAngleA = fishes[i]->bones[1]->joint->p_joint->GetJointAngle();
					// float jointAngleB = fishes[i]->bones[2]->joint->p_joint->GetJointAngle();

					// printf("joint: %.2f %.2f\n", jointAngleA, jointAngleB);

					for (int j = 1; j < 8; ++j) // dont even try to move the 0th one
					{

						if ( !fishes[i]->bones[j]->isUsed || !fishes[i]->bones[j]->init) {
							continue;
						}
						else if (fishes[i]->bones[j]->isUsed && fishes[i]->bones[j]->init) {

							if (fishes[i]->bones[j]->joint->p_joint != nullptr) {
								fishes[i]->bones[j]->joint->p_joint->SetMotorSpeed(motorSignals[j]*fishes[i]->bones[j]->joint->speedLimit);
								// printf("the joint speed was set");

							}

							
							
						}
						
						
					}

					 //speedForJointA*10);
					// fishes[i]->bones[1]->joint->p_joint->SetMotorSpeed(motorSignals[1]*fishes[i]->bones[1]->joint->speedLimit);//speedForJointB*10);
					// fishes[i]->bones[2]->joint->p_joint->SetMotorSpeed(motorSignals[2]*fishes[i]->bones[2]->joint->speedLimit); //speedForJointA*10);
					// fishes[i]->bones[3]->joint->p_joint->SetMotorSpeed(motorSignals[3]*fishes[i]->bones[3]->joint->speedLimit);//speedForJointB*10);
					// fishes[i]->bones[4]->joint->p_joint->SetMotorSpeed(motorSignals[4]*fishes[i]->bones[4]->joint->speedLimit); //speedForJointA*10);
					// fishes[i]->bones[5]->joint->p_joint->SetMotorSpeed(motorSignals[5]*fishes[i]->bones[5]->joint->speedLimit);//speedForJointB*10);
					// fishes[i]->bones[6]->joint->p_joint->SetMotorSpeed(motorSignals[6]*fishes[i]->bones[6]->joint->speedLimit); //speedForJointA*10);
					// fishes[i]->bones[7]->joint->p_joint->SetMotorSpeed(motorSignals[7]*fishes[i]->bones[7]->joint->speedLimit);//speedForJointB*10);
				}

				// print the brainal output
				drawNeuralNetwork( fishes[i]->ann, motorSignals, sensorium, i, &spacesUsedSoFar);




		}
		else{
			;
		}









	}
}

void deepSeaControlA () {
	// fishes[0]->bones[2]->joint->p_joint->SetMotorSpeed(1.0f);
}
void deepSeaControlB () {
	// fishes[0]->bones[2]->joint->p_joint->SetMotorSpeed(-1.0f);
}

void collisionHandler (void * userDataA, void * userDataB) {
	bool et = false;
	bool fud = false;


	if (true) {
		return;   // this function ends the program so nice to turn it off
	}



	if (userDataA == nullptr || userDataB == nullptr) {
		return;
	}
	else {
		;
	}

	uDataWrap * p_dataA = (uDataWrap *) userDataA;
	uDataWrap * p_dataB = (uDataWrap *) userDataB;

	uDataWrap dataA = *p_dataA;
	uDataWrap dataB = *p_dataB;

	if( dataA.dataType == TYPE_MOUTH ) {
		et = true;
		if( dataB.dataType == TYPE_FOOD ) {
			fud = true;
		}
	}

	if( dataB.dataType == TYPE_MOUTH ) {
		et = true;
		if( dataA.dataType == TYPE_FOOD ) {
			fud = true;
		}
	}
	
	if (et && fud) {
		printf("monch");

		// save the fish that touched the food as 225
		if( dataB.dataType == TYPE_MOUTH ) {
			std::string nnfilename =  std::string("225.net");
		    std::string fdescfilename =  std::string("225.fsh");
		    std::ofstream file { nnfilename };
		    saveFishToFile (fdescfilename, ((BoneUserData *)(dataB.uData))->p_owner->genes);
		    fann_save(((BoneUserData *)(dataB.uData))->p_owner->ann, nnfilename.c_str()); 
			exit(EXIT_SUCCESS);
		}
		else if (dataA.dataType == TYPE_MOUTH) {
			std::string nnfilename =  std::string("225.net");
		    std::string fdescfilename =  std::string("225.fsh");
		    std::ofstream file { nnfilename };
		    saveFishToFile (fdescfilename, ((BoneUserData *)(dataA.uData))->p_owner->genes);
		    fann_save(((BoneUserData *)(dataA.uData))->p_owner->ann, nnfilename.c_str()); 
			exit(EXIT_SUCCESS);
		}

	 
	}
}