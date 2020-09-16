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

bool fishSlotLoaded[N_FISHES];
bool foodSlotLoaded[N_FOODPARTICLES];

bool userControlInputA;
bool userControlInputB;

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
		b2World * m_world, b2ParticleSystem * m_particleSystem // primarily needed to create the body
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
	density = 1.0f;
	isRoot = boneDescription.isRoot;
	isMouth = boneDescription.isMouth;
	isSensor = boneDescription.isSensor;
	sensation = 0.0f;
	isWeapon  = boneDescription.isWeapon;									// weapons destroy joints to snip off a limb for consumption. optionally, they can produce a physical effect.
	energy = ((rootThickness + tipThickness)/2) * (length * density); 		// the nutritive energy stored in the tissue of this limb; used by predators and scavengers

	tipCenter = b2Vec2(0.0f,0.1f); 											// these are used so the skeleton master can remember his place as he traverses the heirarchy of souls.
	rootCenter = b2Vec2(0.0f,0.0f); 		
	int count = 4;

	// the following code is used to generate box2d structures and shapes from the bone parameters.
	if (isRoot) {
		printf("its a root bone\n");

		tipCenter = b2Vec2(0.0f, 0.0f +  length);

		b2Vec2 vertices[] = {
			b2Vec2(0.0f + (rootThickness/2), 0.0f), //b2Vec2 rootVertexA = 
			b2Vec2(0.0f - (rootThickness/2), 0.0f), // b2Vec2 rootVertexB =
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
	p_body->CreateFixture(&shape, 1.0f);
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

BonyFish::BonyFish(fishDescriptor_t driedFish, uint8_t fishIndex, b2World * m_world, b2ParticleSystem * m_particleSystem, fann * nann) {
	genes = driedFish;
	hunger = 0.0f; // the animal spends energy to move and must replenish it by eating

	for (int i = 0; i < N_FINGERS; ++i) {

		if (i == 0) {
			driedFish.bones[i].isRoot = true;
		}

		bones[i] = new BoneUserData(driedFish.bones[i], this,  m_world, m_particleSystem);
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
		    // fann_train_on_file(ann, "jellyfishTrainer.data", max_epochs, epochs_between_reports, desired_error);
	    }
    else { // a brain is provided
    	ann = nann;
    }
    
    // make a random but sane file name. It would be better to have sequential, but i can't be assed to code it.
    name = RNG() * 255;
    std::string nnfilename =  std::to_string(name).c_str() + std::string(".net");
    std::string fdescfilename =  std::to_string(name).c_str() + std::string(".fsh");
    std::ofstream file { nnfilename };
    saveFishToFile (fdescfilename, genes);
    fann_save(ann, nnfilename.c_str()); 
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
				0.005f,	// torque
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
				0.005f,	// torque
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
				0.005f,	// torque
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






void totalFishIncorporator (uint8_t fishIndex, b2World * m_world, b2ParticleSystem * m_particleSystem) {
	for (int i = 0; i < N_FINGERS; ++i) {
		if (fishes[fishIndex]->bones[i]->init) {
			nonRecursiveBoneIncorporator( fishes[fishIndex]->bones[i] , m_world, m_particleSystem, i);
		}
	}
}

// delete a fish from the game world and remove it from memory
void deleteFish (uint8_t fishIndex,  b2World * m_world, b2ParticleSystem * m_particleSystem) {
	for (int i = 0; i < N_FINGERS; ++i) {
		m_world->DestroyBody(fishes[fishIndex]->bones[i]->p_body);
		delete fishes[fishIndex]->bones[i];
	}
	delete fishes[fishIndex];	 
}

void loadFish (uint8_t fishIndex, fishDescriptor_t driedFish, b2World * m_world, b2ParticleSystem * m_particleSystem, fann * nann) {
	fishes[fishIndex] = new BonyFish(driedFish, fishIndex , m_world, m_particleSystem, nann);
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
	for (int i = 0; i < 8; ++i)
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

networkDescriptor createNeurodescriptorFromFANN () {
	// making a descriptor from a file was too hard
	// you can make a FANN from the file and then query it for information you need to build the model.
	std::string fileName = "225";

	fann * temp_ann  = loadFishBrainFromFile (fileName);

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
	fann_get_layer_array(temp_ann, layerCake);

	// build everything in memory and link it together.
	networkDescriptor newCake = networkDescriptor();	//
  	newCake.n_layers = num_layers;
  	printf ("\ncreated network descriptor\n") ;

  	for (unsigned int i = 0; i < num_layers; ++i) {
  		layerCake[i] += 1;
  	}

  	for (unsigned int i = 0; i < num_layers; ++i) {
  		newCake.layers[i].n_neurons = layerCake[i];
  		newCake.layers[i].isUsed = true;
  		printf ("created layer descriptor %i\n", layerCake[i]) ;

		for (unsigned int j = 0; j < layerCake[i]; ++j) {
  			// newCake.layers[i].neurons[j] = new neuronDescriptor();//*(new neuronDescriptor());
  			newCake.layers[i].neurons[j].activation_function = activation_function_hidden;
  			newCake.layers[i].neurons[j].activation_steepness = activation_steepness_hidden;
  			newCake.layers[i].neurons[j].n_connections = 0; // so not used uninitialized
  			newCake.layers[i].neurons[j].n_inputs = 0; // so not used uninitialized
  			newCake.layers[i].neurons[j].isUsed = true;

  			if (i == num_layers-1) {
				newCake.layers[i].neurons[j].activation_function = activation_function_output;
  				newCake.layers[i].neurons[j].activation_steepness = activation_steepness_output;
  			}
  			printf ("created neuron descriptor\n") ;
  		}
  	}

  	// get connection and weight information.
  	unsigned int num_connections = fann_get_total_connections(temp_ann);
  	static struct fann_connection margles[256] ;
  	struct fann_connection *con = margles; 
  	fann_get_connection_array(temp_ann, con);

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
 
  		printf("%u %u\n", index, layer);
  		unsigned int newestConnectionIndex = newCake.layers[layer].neurons[index].n_connections;
  		printf("%u\n", newestConnectionIndex);
  		newCake.layers[layer].neurons[index].connections[newestConnectionIndex].connectedTo = con[c].to_neuron;
  		newCake.layers[layer].neurons[index].connections[newestConnectionIndex].connectionWeight = con[c].weight;
  		newCake.layers[layer].neurons[index].connections[newestConnectionIndex].isUsed = true;
  		newCake.layers[layer].neurons[index].n_connections ++;
 
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

		newCake.layers[toLayer].neurons[toIndex].n_inputs ++;
		printf ("created connection descriptor f%u t%u w%f, %u of %u\n", con[c].from_neuron, con[c].to_neuron, con[c].weight, c, num_connections-1) ;	
	}
return newCake;
}

// from: https://stackoverflow.com/questions/7132957/c-scientific-notation-format-number
void my_print_scientific(char *dest, double value) {
    snprintf(dest, 20, "%.0e", value); // 20 digits between the decimal place and the e
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
 	for (unsigned int i = 0; i < network.n_layers; ++i) 	{
 		for (unsigned int j = 0; j < network.layers[i].n_neurons; ++j) {
 			char chalkboard[9];
 			sprintf(chalkboard, "(%u, %u, ", network.layers[i].neurons[j].n_inputs, network.layers[i].neurons[j].activation_function);	
 			s += chalkboard;
 			char sciNotationBuffer[] = "0.00000000000000000000e+00";
 			my_print_scientific(sciNotationBuffer, network.layers[i].neurons[j].activation_steepness);
 			s += sciNotationBuffer;

 			s.append(") ");
 		}
 	}

 	printf("print connection information\n");
 	// 
	s += "\nconnections (connected_to_neuron, weight)=";
	for (unsigned int i = 0; i < network.n_layers; ++i) 	{
 		for (unsigned int j = 0; j < network.layers[i].n_neurons; ++j) {
 			for (unsigned int k = 0; k < network.layers[i].neurons[j].n_connections; ++k) {
 				char chalkboard[5];
 				sprintf(chalkboard, "(%u, ", network.layers[i].neurons[j].connections[k].connectedTo);
 				s += chalkboard;
 				char sciNotationBuffer[] = "0.00000000000000000000e+00";
	 			my_print_scientific(sciNotationBuffer, network.layers[i].neurons[j].connections[k].connectionWeight);
	 			s+= sciNotationBuffer;
	 			s.append(") ");
 			}
 		}
 	}

 	printf("print to file\n");

    std::ofstream out("mouptut.net");
    out << s;
    out.close();
}

void mutateFishBrain (networkDescriptor * newCake, float mutationChance, float mutationSeverity) {

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
	for (int i = 0; i < 50; ++i)
	{
			if ( true ) {// RNG() > mutationChance) {
		int l =  RNG() * newCake->n_layers ;
		int m =  RNG() * newCake->layers[l].n_neurons ;
		int n =  RNG() * newCake->layers[l].neurons[m].n_connections ;

		float mutationAmount = ((RNG() -0.5) *mutationSeverity  );
		printf("warped layer %i neuron %i connection %i by %f\n", l, m, n,  mutationAmount);

		newCake->layers[l].neurons[m].connections[n].connectionWeight += mutationAmount;

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

	networkDescriptor tamberlina = createNeurodescriptorFromFANN();
	
	mutateFishBrain(&tamberlina, 0.1f, 1.0f );

	createFANNFileFromDescriptor(tamberlina);

	fann * ann  = loadFishBrainFromFile ("mouptut"); // load the mutated file

	bool loadWithBlankBrain = true;
	if (loadWithBlankBrain) {
		loadFish ( fishIndex,  newFish, m_world,  m_particleSystem, NULL ) ;
	}
	else {
		loadFish ( fishIndex,  newFish, m_world,  m_particleSystem, ann ) ;
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
    fp = fopen("koiTrainer.data","wb");

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

void deepSeaSetup (b2World * m_world, b2ParticleSystem * m_particleSystem, DebugDraw * p_debugDraw) {

	jellyfishTrainer();

	// store the debugdraw pointer in here so we can use it.
	local_debugDraw_pointer = p_debugDraw;

	addFoodParticle(b2Vec2(2.5f, 3.5f), m_world, m_particleSystem);

	int howManyNewFishToAdd = 1;
	for (int i = 0; i < howManyNewFishToAdd; ++i) {


		//  loadFishFromName is used to load the named files associated with the evolution system.
		// LoadFishFromName(i, m_world, m_particleSystem);


		// this one is good to just load in a desfault fish from the descriptor.

		// fann * ann  = loadFishBrainFromFile ("mouptut"); // unfortunately you still need to load some kind of brain or it wont work.
		loadFish ( i,  koiCarp, m_world,  m_particleSystem, NULL ) ;



		totalFishIncorporator(i, m_world, m_particleSystem);
	}
}

void drawNeuralNetwork(struct 	fann 	*	ann	, float * motorSignals, float * sensorium, int index) {

	// get the number of layers. FANN_EXTERNAL unsigned int FANN_API fann_get_num_layers(	struct 	fann 	*	ann	)
	unsigned int n_layers = fann_get_num_layers(ann);

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

	b2Vec2 drawingStartingPosition = b2Vec2( (2.0f * index) ,4.0f);
	float spacingDistance = 0.5f;

	for (uint8_t j = 0; j < layerArray[0]; ++j)
	{
		b2Vec2 neuron_position = b2Vec2(drawingStartingPosition.x +j * spacingDistance,drawingStartingPosition.y );

		local_debugDraw_pointer->DrawPoint(neuron_position, 8.0f, b2Color( sensorium[j], sensorium[j], sensorium[j]));
	}

	for (uint8_t j = 0; j < layerArray[n_layers-1]; ++j)
	{
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

	for  (int i = 0; i < N_FOODPARTICLES; i++) {
		if (!foodSlotLoaded[i]) {
			break;
		}
		else {
			food[i]->position = food[i]->p_body->GetPosition(); // update positions of all the food particles
		}
	}

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

				if (false) {
					// float jointAngleA = fishes[i]->bones[1]->joint->p_joint->GetJointAngle();
					// float jointAngleB = fishes[i]->bones[2]->joint->p_joint->GetJointAngle();

					// printf("joint: %.2f %.2f\n", jointAngleA, jointAngleB);

					for (int j = 0; j < 8; ++j)
					{
						fishes[i]->bones[j]->joint->p_joint->SetMotorSpeed(motorSignals[j]*fishes[i]->bones[j]->joint->speedLimit);
					}

					 //speedForJointA*10);
					fishes[i]->bones[1]->joint->p_joint->SetMotorSpeed(motorSignals[1]*fishes[i]->bones[1]->joint->speedLimit);//speedForJointB*10);
					fishes[i]->bones[2]->joint->p_joint->SetMotorSpeed(motorSignals[2]*fishes[i]->bones[2]->joint->speedLimit); //speedForJointA*10);
					fishes[i]->bones[3]->joint->p_joint->SetMotorSpeed(motorSignals[3]*fishes[i]->bones[3]->joint->speedLimit);//speedForJointB*10);
					fishes[i]->bones[4]->joint->p_joint->SetMotorSpeed(motorSignals[4]*fishes[i]->bones[4]->joint->speedLimit); //speedForJointA*10);
					fishes[i]->bones[5]->joint->p_joint->SetMotorSpeed(motorSignals[5]*fishes[i]->bones[5]->joint->speedLimit);//speedForJointB*10);
					fishes[i]->bones[6]->joint->p_joint->SetMotorSpeed(motorSignals[6]*fishes[i]->bones[6]->joint->speedLimit); //speedForJointA*10);
					fishes[i]->bones[7]->joint->p_joint->SetMotorSpeed(motorSignals[7]*fishes[i]->bones[7]->joint->speedLimit);//speedForJointB*10);
				}

				// print the brainal output
				drawNeuralNetwork( fishes[i]->ann, motorSignals, sensorium, i);




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

	    std::string nnfilename =  std::string("225.net");
	    std::string fdescfilename =  std::string("225.fsh");
	    std::ofstream file { nnfilename };
	    saveFishToFile (fdescfilename, ((BoneUserData *)(dataA.uData))->p_owner->genes);
	    fann_save(((BoneUserData *)(dataA.uData))->p_owner->ann, nnfilename.c_str()); 
		exit(EXIT_SUCCESS);
	}
}