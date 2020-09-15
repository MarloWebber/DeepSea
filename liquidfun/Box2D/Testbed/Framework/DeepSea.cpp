#include "DeepSea.h"
#include "fann.h"
#include "Test.h"

#include <iostream>
#include <fstream>
#include <random>
#include <string>

// #include <fstream>
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

void setUserControlInputA() {
userControlInputA = true;
}

void setUserControlInputB () {
userControlInputB = true;
}

float pi = 3.14159f;

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
	// speed = 0.0f; 	
	speedLimit = boneDescription.speedLimit;
	upperAngle = boneDescription.upperAngle;
	normalAngle = boneDescription.normalAngle;
	lowerAngle = boneDescription.lowerAngle;

	// driveCW = false;	// a signal that tells the motor to turn in one direction. This is much simpler than trying to drive it with a number and having to remember positon etc. With this, you just hit the button, or don't.
	// driveCCW = false;	// a signal that tells the motor to turn in the other direction.

	// init = false;
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
	    // jointDef.motorSpeed = speed;
	}

    jointDef.userData = this;

    init = true;
}

uDataWrap::uDataWrap(void * dat, uint8_t typ) {
		uData = dat;
		dataType = typ;
}

BoneUserData::BoneUserData(
		boneAndJointDescriptor_t boneDescription,
		BonyFish * fish,
		b2World * m_world, b2ParticleSystem * m_particleSystem // primarily needed to create the body
	) {

	if (!boneDescription.used) {
		return;
	}

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

	// the following code is used to generate box2d structures and shapes from the bone parameters.
	if (isRoot) {

		b2Vec2 tipCenter = b2Vec2(0.0f, 0.0f + length);
		b2Vec2 rootVertexA = b2Vec2(0.0f + (rootThickness/2), 0.0f);
		b2Vec2 rootVertexB = b2Vec2(0.0f - (rootThickness/2), 0.0f);
		b2Vec2 tipVertexA = b2Vec2(tipCenter.x + (tipThickness/2), tipCenter.y);
		b2Vec2 tipVertexB = b2Vec2(tipCenter.x - (tipThickness/2), tipCenter.y);

		int count = 4;
		b2Vec2 vertices[count];
		vertices[0].Set(rootVertexB.x, rootVertexB.y);
		vertices[1].Set(tipVertexB.x, tipVertexB.y);
		vertices[2].Set(tipVertexA.x, tipVertexB.y);
		vertices[3].Set(rootVertexA.x, rootVertexA.y);

		// figure out the center point.
		b2Vec2 boneCenter = b2Vec2(0.0f, 0.0f + (2*length));

		// attach user data to the body
		// bodyDef.userData = this;

		// this is mainly used for high performance object type detection.
// enum gameObjectType { 
// 	DEFAULT, 
// 	FOOD, 
// 	MOUTH 
// };

// struct uDataWrap() {
// 	void * userData;
// 	uint8_t dataType;
// };


		
		uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_MOUTH);
		bodyDef.userData = (void *)p_dataWrapper;


		bodyDef.type = b2_dynamicBody;
		p_body = m_world->CreateBody(&bodyDef);
		
		shape.SetAsBox(rootThickness, length, boneCenter,boneDescription.normalAngle);	

		// reference the physics object from the user data.
		tipCenter = tipCenter;
		rootCenter = b2Vec2(0.0f, 0.0f);
	}
	else {
		b2Vec2 tipCenter = b2Vec2(attachesTo->tipCenter.x, attachesTo->tipCenter.y + length);
		b2Vec2 rootVertexA = b2Vec2(attachesTo->tipCenter.x + (rootThickness/2), attachesTo->tipCenter.y);
		b2Vec2 rootVertexB = b2Vec2(attachesTo->tipCenter.x - (rootThickness/2), attachesTo->tipCenter.y);
		b2Vec2 tipVertexA = b2Vec2(tipCenter.x + (tipThickness/2), tipCenter.y);
		b2Vec2 tipVertexB = b2Vec2(tipCenter.x - (tipThickness/2), tipCenter.y);

		int count = 4;
		b2Vec2 vertices[count];
		vertices[0].Set(rootVertexB.x, rootVertexB.y);
		vertices[1].Set(tipVertexB.x, tipVertexB.y);
		vertices[2].Set(tipVertexA.x, tipVertexB.y);
		vertices[3].Set(rootVertexA.x, rootVertexA.y);

		// figure out the center point.
		b2Vec2 boneCenter = b2Vec2(attachesTo->tipCenter.x, attachesTo->tipCenter.y + (2*length));

		// attach user data to the body
		uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_MOUTH);
		bodyDef.userData = (void *)p_dataWrapper;


		bodyDef.type = b2_dynamicBody;
		p_body = m_world->CreateBody(&bodyDef);
		
		shape.SetAsBox(rootThickness, length, boneCenter, 0.0f);	

		// reference the physics object from the user data.
		tipCenter = tipCenter;
		rootCenter = attachesTo->tipCenter;

	}

	if (!isRoot) {
		joint = new JointUserData( boneDescription, this, fish, m_world, m_particleSystem); 	// the joint that attaches it into its socket 
	}	

	init = true;
	isUsed=  false;


};

void nonRecursiveBoneIncorporator(BoneUserData * p_bone, b2World * m_world, b2ParticleSystem * m_particleSystem) {
	if (!p_bone->init) {
		return;
	}
	p_bone->p_body->CreateFixture(&(p_bone->shape), p_bone->density);	// this endows the shape with mass and is what adds it to the physical world.
	// m_particleSystem->DestroyParticlesInShape( &(p_bone->shape) ,p_bone->body->GetTransform());
	if (!p_bone->isRoot) {
            p_bone->joint->isUsed = true;
			p_bone->joint->p_joint = (b2RevoluteJoint*)m_world->CreateJoint( &(p_bone->joint->jointDef) );
	}
	p_bone->isUsed = true;
}

void printab2Vec2(b2Vec2 v) {
	printf("x%f y%f\n", v.x, v.y);
}

void nonRecursiveSensorUpdater (BoneUserData * p_bone) {

	if (!p_bone->init || !p_bone->isUsed) {
		return;
	}
	p_bone->position = p_bone->p_body->GetPosition();
	// printf("bone position ");printab2Vec2(p_bone->position); << bone position is good
	p_bone->sensation = 0.0f;

	for  (int i = 0; i < N_FOODPARTICLES; i++) {
		if (!foodSlotLoaded[i]) {
			break;
		}
		if (food[i]->init && food[i]->isUsed) {
			
			b2Vec2 positionalDifference = b2Vec2((p_bone->position.x - food[i]->position.x),(p_bone->position.y - food[i]->position.y));

			float distance = magnitude (positionalDifference);

			// printf("positionalDistance ");printab2Vec2(positionalDifference);

			if (distance > 0) {
				p_bone->sensation += 1/distance;
			}
		}
	}
	// printf("sensation: %f\n", p_bone->sensation);
}

// add a food particle to the world and register it so the game knows it exists.
foodParticle_t::foodParticle_t ( b2Vec2 position, b2World * m_world, b2ParticleSystem * m_particleSystem) {
	
	energy = 1.0f;

	// b2BodyDef bodyDef;

	// uDataWrap userD
	// bodyDef.userData = this; // register the fishfood struct as user data on the body
	uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_FOOD);
	bodyDef.userData = (void*)p_dataWrapper;


	bodyDef.type = b2_dynamicBody;
	p_body = m_world->CreateBody(&bodyDef);
	// b2CircleShape shape;
	// shape.SetUserData(this)	// and also on the shape
	
	shape.SetAsBox(0.025f, 0.025f, position,0.0f);	
	p_body->CreateFixture(&shape, 1.0f);
	// m_particleSystem->DestroyParticlesInShape(shape,p_body->GetTransform()); // snip out the particles that are already in that spot so it doesn't explode

	init = true;
	isUsed = true;

	
};

void addFoodParticle(b2Vec2 position, b2World * m_world, b2ParticleSystem * m_particleSystem) {
	food[currentNumberOfFood] = new foodParticle_t( position, m_world,  m_particleSystem);
	foodSlotLoaded[currentNumberOfFood] = true;
	currentNumberOfFood++;

}


void saveFishToFile(const std::string& file_name, fishDescriptor_t& data)
{
  std::ofstream out(file_name.c_str());
  out.write(reinterpret_cast<char*>(&data), sizeof(fishDescriptor_t));
}

void loadFishFromFile(const std::string& file_name, fishDescriptor_t& data)
{
  std::ifstream in(file_name.c_str());
  in.read(reinterpret_cast<char*>(&data), sizeof(fishDescriptor_t));
}




BonyFish::BonyFish(fishDescriptor_t driedFish, uint8_t fishIndex, b2World * m_world, b2ParticleSystem * m_particleSystem, fann * nann) {

	genes = driedFish;

	hunger = 0.0f; // the animal spends energy to move and must replenish it by eating
	// position = b2Vec2(0.0f, 0.0f); // the starting position of the fish in the game world

	for (int i = 0; i < N_FINGERS; ++i) {

		if (i == 0) {
			// set the bone as root
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
	isUsed = false;

	heartSpeed = driedFish.heartSpeed;
	if (heartSpeed < 1) {
		heartSpeed = 50;
	}



    if (nann == NULL) {
    	    unsigned int creationLayerCake[] = {
	    	3,
	    	3,
	    	4,
	    	2
	    };
	    	ann = fann_create_standard_array(4, creationLayerCake);
		    fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
		    fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);
		    fann_train_on_file(ann, "jellyfishTrainer.data", max_epochs, epochs_between_reports, desired_error);
		    // fann_cascadetrain_on_file(ann, "jellyfishTrainer.data", 25, 1, desired_error); // cascade training is real slow ?

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

    // name = filename;
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

void totalFishIncorporator (uint8_t fishIndex, b2World * m_world, b2ParticleSystem * m_particleSystem) {
	for (int i = 0; i < N_FINGERS; ++i)
	{
		if (fishes[fishIndex]->bones[i]->init) {
			nonRecursiveBoneIncorporator( fishes[fishIndex]->bones[i] , m_world, m_particleSystem);
		}
	}
}

// delete a fish from the game world and remove it from memory
void deleteFish (uint8_t fishIndex,  b2World * m_world, b2ParticleSystem * m_particleSystem) {

	// m_world->DestroyBody(fishes[fishIndex]->body);

	for (int i = 0; i < N_FINGERS; ++i)
	{
		m_world->DestroyBody(fishes[fishIndex]->bones[i]->p_body);
		delete fishes[fishIndex]->bones[i];
	}

	delete fishes[fishIndex];	 


	// you will also need to delete the uDataWrap structures, which were heap allocated, or a memory leak will be caused.

}




void loadFish (uint8_t fishIndex, fishDescriptor_t driedFish, b2World * m_world, b2ParticleSystem * m_particleSystem, fann * nann) {
	fishes[fishIndex] = new BonyFish(driedFish, fishIndex , m_world, m_particleSystem, nann);
	fishSlotLoaded[fishIndex] = true;
}


fann * loadFishBrainFromFile (std::string fileName) {
	return fann_create_from_file( (fileName + std::string(".net")).c_str() );
}


connectionDescriptor::connectionDescriptor () {

}

neuronDescriptor::neuronDescriptor() {
	n_connections = 0;
	n_inputs = 0;
}

layerDescriptor::layerDescriptor () {

}


// std::fstream& goToLine(std::fstream& file,  int num){
//     file.seekg(std::ios::beg);
//     for(int i=0; i < num - 1; ++i){
//         file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
//     }
//     return file;
// }

void unused_variable(void * bullshit) {
	; // do nothing
}

void advanceCursor(FILE * cursor, int charToMoveAhead) {
int c;
c = fgetc(cursor);
unused_variable((void *)&c);

// 	int charMovedSoFar = 0;
// while(1) {
// 	int c;
//       c = fgetc(cursor);
//       if( feof(cursor) ) { 
//          break ;
//       }
//       // printf("%c", c);
//       unused_variable((void *)&c);
//       if (charMovedSoFar >= charToMoveAhead) {
//       	// scanning = false;
//       	// printf("trememnsoi");
//       	break;
//       }
//       charMovedSoFar ++;
//   }
}

void goToLine (FILE * cursor, int linesToMoveAhead) {
	
	int linesMovedSoFar = 0;
	// bool scanning = true;
	while(1) {
	int c;
      c = fgetc(cursor);
      if( feof(cursor) ) { 
         break ;
      }
      // printf("%c", c);
      unused_variable((void *)&c);


      if (c == '\n') {
      	// printf("mondo jung");
      	linesMovedSoFar ++;


      }

      if (linesMovedSoFar >= linesToMoveAhead) {
      	// scanning = false;
      	// printf("trememnsoi");
      	break;
      }


   	}


	// uint8_t linesMovedSoFar = 0;

	// for (int i = 0; i < linesToMoveAhead; ++i)
	// {
	// 	bool scanning = false;
	// 	while (scanning) {
	// 		fseek(cursor, 1, SEEK_CUR);
	// 		char sample = fgetc(cursor);
	// 		printf("%c", sample);
	// 		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	// 		if (sample == '\n') {
	// 			scanning = false;
	// 			linesMovedSoFar ++;
	// 			continue;
	// 		}
	// 	}


	// }

		
			

		
	
	// cursor is now advanced to the desired location.
}

// method to create a network descriptor from a stored file
networkDescriptor::networkDescriptor () {
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


void createNeurodescriptorFromFANN () {
	// making a descriptor from a file was too hard
	// you can make a FANN from the file and then query it for information you need to build the model.
	std::string fileName = "225";

	// loadFishFromFile(fileName + std::string(".fsh"), newFish);

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
	networkDescriptor * newCake = new networkDescriptor();	//
  	newCake->n_layers = num_layers;
  	printf ("\ncreated network descriptor\n") ;



  	for (unsigned int i = 0; i < num_layers; ++i) {
  		newCake->layers[i] =  new layerDescriptor(); 				// create the layer descriptor
  		newCake->layers[i]->n_neurons = layerCake[i];
  		printf ("created layer descriptor\n") ;

		for (unsigned int j = 0; j <= layerCake[i]; ++j) {
  			newCake->layers[i]->neurons[j] = new neuronDescriptor();//*(new neuronDescriptor());
  			newCake->layers[i]->neurons[j]->activation_function = activation_function_hidden;
  			newCake->layers[i]->neurons[j]->activation_steepness = activation_steepness_hidden;
  			newCake->layers[i]->neurons[j]->n_connections = 0; // so not used uninitialized
  			newCake->layers[i]->neurons[j]->n_inputs = 0; // so not used uninitialized

  			if (i == num_layers-1) {
				newCake->layers[i]->neurons[j]->activation_function = activation_function_output;
  				newCake->layers[i]->neurons[j]->activation_steepness = activation_steepness_output;
  			}
  			printf ("created neuron descriptor\n") ;
  		}
  	}

  	// get connection and weight information.
  	// unsigned int theMostConnectionsThereWillEverBe = 1024;
  	unsigned int num_connections = fann_get_total_connections(temp_ann);
  	static struct fann_connection margles[256] ;
  	struct fann_connection *con = margles; 
  	fann_get_connection_array(temp_ann, con);

 
  	// apply them to the model
  	// for (unsigned int i = 0; i < num_layers; ++i) {
  	// 	for (unsigned int j = 0; j <= layerCake[i]; ++j) {

  	// 		if (i == num_layers-1) {
  	// 			newCake->layers[i]->neurons[j]->activation_function = activation_function_hidden;
  	// 			newCake->layers[i]->neurons[j]->activation_steepness = activation_steepness_hidden;
  	// 		}
  	// 		else {
  	// 			newCake->layers[i]->neurons[j]->activation_function = activation_function_output;
  	// 			newCake->layers[i]->neurons[j]->activation_steepness = activation_steepness_output;
  	// 		}
  	// 		printf ("set activation parameters l%u n%u\n", i, j) ;
  	// 	}
  	// }

  	// create connections
  	for (unsigned int c = 0; c < num_connections; ++c) {
  		unsigned int layer = 0;
  		unsigned int index = con[c].from_neuron;
  		while (1) {
  			if (index < layerCake[layer]) { break; }
  			else {
  				index -= layerCake[layer];
  				layer ++;
  			}
  			// printf("0");
  		}

  		// if (layer > num_layers) {
  		// 	continue;
  		// }

  		// add connection descriptor and adjust parameters of 'from' neuron
  		printf("%u %u\n", index, layer);
  		// printf("%u\n", newCake->layers[layer]->neurons[index]->n_connections)
  		newCake->layers[layer]->neurons[index]->connections[newCake->layers[layer]->neurons[index]->n_connections] = new connectionDescriptor();
  		newCake->layers[layer]->neurons[index]->connections[newCake->layers[layer]->neurons[index]->n_connections]->connectedTo = con[c].to_neuron;
  		newCake->layers[layer]->neurons[index]->connections[newCake->layers[layer]->neurons[index]->n_connections]->connectionWeight = con[c].weight;
  		newCake->layers[layer]->neurons[index]->n_connections ++;
 
  		// adjust the input number on the 'to' neuron.
  		unsigned int toLayer = 0;
  		unsigned int toIndex = con[c].to_neuron;
  		while (1) {
  			if (toIndex < layerCake[toLayer]) { break; }
  			else {
  				toIndex -= layerCake[toLayer];
  				toLayer ++;
  			}
  			// printf("1");
  		}
  		printf("%u %u ", toIndex, toLayer-1);
		newCake->layers[toLayer-1]->neurons[toIndex]->n_inputs ++;
		printf ("created connection descriptor f%u t%u w%f, %u of %u\n", con[c].from_neuron, con[c].to_neuron, con[c].weight, c, num_connections) ;	



  	}

  	// for (int i = 0; i < newCake->n_layers; ++i)
  	// {
  	// 	for (int j = 0; j < newCake->layers[i]->n_neurons; ++j)
  	// 	{
  	// 		for (int i = 0; i < count; ++i)
  	// 		{
  	// 			/* code */
  	// 		}
  	// 	}
  	// }
}

// void createNeurodescriptorFromFile () {
// 	FILE * pFile;
// 	pFile = fopen ( "209.net" , "r" );

// 	// get array sizes

// 	printf ("createNeurodescriptorFromFile:\n") ;// read in number of layers
// 	fseek(pFile, 0, SEEK_SET); // set cursor to beginning
//   	goToLine(pFile, 1); 			// advance pFile to line 2
//   	// printf("selanemod");
//   	// pFile += sizeof("num_layers=");			// advance to the layer number position
//   	// advanceCursor(pFile, 12);
//   	fseek(pFile, 11, SEEK_CUR); // set cursor to beginning
//   	int num_layers = fgetc(pFile) -48; 	// get one character, the -48 is used to convert ASCII encoding to positive integer.
//   	printf ("Number of layers: %i\n", num_layers) ;// read in number of layers

// 	int layerCake[num_layers];

//   	// read in layer cake structure
//   	printf ("Reading layer cake:\n") ;
//   	// fseek(pFile, 0, SEEK_SET); // set cursor to beginning
//   	goToLine(pFile, 31); 			// go forward 33 lines to the line with layer size information 
//   	// advanceCursor(pFile, 12);

//   	fseek(pFile, 12, SEEK_CUR); // 

//   	for (int i = 0; i < num_layers; ++i) {
//   		int num_neurons = fgetc(pFile) - 48; 	// read the number
//   		// int milne = fgetc(pFile);
//   		printf("%i", num_neurons);
//   		// printf ("Reading fsefs cake:\n") ;
  		
//   		layerCake[i] = num_neurons;
//   		fseek(pFile, 1, SEEK_CUR); // set cursor to beginning
  		
//   		// neuronDescriptor * newLayer
//   	}

//   	// build everything in memory and link it together

//   	networkDescriptor * newCake = new networkDescriptor();	//
//   	newCake->n_layers = num_layers;
//   	printf ("\ncreated network descriptor\n") ;

//   	for (int i = 0; i < num_layers; ++i) {
//   		newCake->layers[i] =  new layerDescriptor(); 				// create the layer descriptor
//   		newCake->layers[i]->n_neurons = layerCake[i];
//   		printf ("created layer descriptor\n") ;

// 		for (int j = 0; j < layerCake[i]; ++j) {
//   			// neuronDescriptor * p_neuron 
//   			newCake->layers[i]->neurons[j] = new neuronDescriptor();//*(new neuronDescriptor());
//   			if (i > 0) {
//   				newCake->layers[i]->neurons[j]->n_connections = newCake->layers[i-1]->n_neurons;
//   			}
//   			else {
//   				newCake->layers[i]->neurons[j]->n_connections = 0;
//   			}

//   			for (int k = 0; k < newCake->layers[i]->neurons[j]->n_connections; ++k)
//   			{
//   				newCake->layers[i]->neurons[j]->connections[k] = new connectionDescriptor();
//   				printf ("created connection descriptor\n") ;
//   			}
  			
//   			printf ("created neuron descriptor\n") ;

//   		}
//   	}









//   		/// populate it

//   	printf ("\nReading activation information") ;

// 	// read in neuron connection numbers and activation function information
// 	// goToLine(pFile, 2);				// go forward two lines
// 	// pFile += sizeof("neurons (num_inputs, activation_function, activation_steepness)=(");			// advance to the layer number position
// 	// fseek(pFile, 65, SEEK_CUR); // set cursor to beginning

//   	if (true) {
// 	  		fseek(pFile, 0, SEEK_SET); // set cursor to beginning
// 	  		goToLine(pFile, 34);
// 	  		fseek(pFile, 65, SEEK_CUR);


// 			for (uint8_t i = 0; i < newCake->n_layers; ++i)	{ // loop over the neurons in this layer
// 				printf("neurons in this layer: %i\n", newCake->layers[i]->n_neurons);
// 					for (uint8_t j = 0; j < newCake->layers[i]->n_neurons; ++j) {

// 						// get number of inputs
// 						// printf("%i\n", newCake->layers[i]->neurons[j]->n_connections);

// 						int ninpits = fgetc(pFile) - 48;
// 						printf("num_inputs: %i\n", ninpits);

// 						// newCake->layers[i]->neurons[j]->n_inputs = ninpits; 	// get one character, the -48 is used to convert ASCII encoding to positive integer.

// 						// get activation function type
// 						fseek(pFile, 2, SEEK_CUR);
// 						ninpits = fgetc(pFile) - 48;

// 						printf("activation_function: %i\n", ninpits);

// 						// newCake->layers[i]->neurons[j]->activation_function = ninpits;

						

// 						// get activation function number
// 						// fseek(pFile, 2, SEEK_CUR);
// 						// char mingTheString[27]; 
// 						// char writtenValue[27];
// 						// char * charPointer = writtenValue;  // required by strtod to be a pointer to a char pointer
// 						// if (fgetc(pFile) == '-') 		{ fgets(mingTheString, 27, pFile);	}  // if the string is preceded by a negative symbol it will be 1 character longer.
// 						// else 							{ fgets(mingTheString, 26, pFile); }

// 						// printf("mingTheString: %s", mingTheString);

// 						// newCake->layers[i]->neurons[j]->activation_steepness = strtod( mingTheString,&(charPointer) );	
// 						seekUntil(pFile, '(');
// 					}
// 				}

//   	}		

	
// 	// read in neuron connection weights
// 	if (false) {
// 		goToLine(pFile, 1); 
// 		pFile += sizeof("connections (connected_to_neuron, weight)=(");			// advance to the layer number position
		
// 		for (uint8_t i = 0; i < newCake->n_layers; ++i)	{
// 			for (uint8_t j = 0; j < newCake->layers[i]->n_neurons; ++j) {
// 				for (uint8_t k = 0; k < newCake->layers[i]->neurons[j]->n_connections ; ++k){
// 					// connectionDescriptor connection =  new connectionDescriptor();
// 					newCake->layers[i]->neurons[j]->connections[k] = new connectionDescriptor();
// 					newCake->layers[i]->neurons[j]->connections[k]->connectedTo = fgetc(pFile) - 48; 	// get one character, the -48 is used to convert ASCII encoding to positive integer.

// 					// fseek(pFile, 2, SEEK_CUR);
// 					// char writtenValue[26];
// 					// newCake->layers[i].neurons[j].connections[k].connectionWeight = strtod( pFile,writtenValue );

// 					// get connection weight
// 					fseek(pFile, 2, SEEK_CUR);
// 					char mingTheString[27]; 
// 					char writtenValue[27];
// 					char * charPointer = writtenValue;  // required by strtod to be a pointer to a char pointer
// 					if (fgetc(pFile) == '-') 	{ fgets(mingTheString, 27, pFile);	}  // if the string is preceded by a negative symbol it will be 1 character longer.
// 					else 						{ fgets(mingTheString, 26, pFile); }
// 					newCake->layers[i]->neurons[j]->connections[k]->connectionWeight = strtod( mingTheString,&(charPointer) );
// 				}		
// 			}
// 		}

// 	}


// // save the file
//   	  fclose ( pFile );
//   	  // fclose(cursor);

// }

// from: https://stackoverflow.com/questions/7132957/c-scientific-notation-format-number
// C version; you can rewrite this to use std::string in C++ if you want
void my_print_scientific(char *dest, double value) {
    // First print out using scientific notation with 0 mantissa digits
    snprintf(dest, 20, "%.0e", value); // 20 digits between the decimal place and the e

    // Find the exponent and skip the "e" and the sign
    char *exponent = strchr(dest, 'e') + 2;

    // If we have an exponent starting with 0, drop it
    if(exponent != NULL && exponent[0] == '0')
    {
        exponent[0] = exponent[1];
        exponent[1] = '\0';
  }
}

// method to create a fann save file from a network descriptor
void createFANNFileFromDescriptor (networkDescriptor * network) {
	std::string s = std::string("FANN_FLO_2.1\nnum_layers=");; // string to hold the information.

	char * t = 0;
	sprintf(t, "%u",network->n_layers);	// print number of layers to position
	s += t;

	// print this
 	s += "\nlearning_rate=0.700000\nconnection_rate=1.000000\nnetwork_type=0\nlearning_momentum=0.000000\ntraining_algorithm=2\ntrain_error_function=1\ntrain_stop_function=0\ncascade_output_change_fraction=0.010000\nquickprop_decay=-0.000100\nquickprop_mu=1.750000\nrprop_increase_factor=1.200000\nrprop_decrease_factor=0.500000\nrprop_delta_min=0.000000\nrprop_delta_max=50.000000\nrprop_delta_zero=0.100000\ncascade_output_stagnation_epochs=12\ncascade_candidate_change_fraction=0.010000\ncascade_candidate_stagnation_epochs=12\ncascade_max_out_epochs=150\ncascade_min_out_epochs=50\ncascade_max_cand_epochs=150\ncascade_min_cand_epochs=50\ncascade_num_candidate_groups=2\nbit_fail_limit=3.49999994039535522461e-01\ncascade_candidate_limit=1.00000000000000000000e+03\ncascade_weight_multiplier=4.00000005960464477539e-01\ncascade_activation_functions_count=10\ncascade_activation_functions=3 5 7 8 10 11 14 15 16 17 \ncascade_activation_steepnesses_count=4\ncascade_activation_steepnesses=2.50000000000000000000e-01 5.00000000000000000000e-01 7.50000000000000000000e-01 1.00000000000000000000e+00\n";
 	s += "layer_sizes=";

 	// print layer sizes separated by a space
	for (unsigned int i = 0; i < network->n_layers; ++i) {
		sprintf(t, "%u", network->layers[i]->n_neurons);
		s += t;
	}

	// print activation information
 	s += "\nscale_included=0\nneurons (num_inputs, activation_function, activation_steepness)=";
 	for (unsigned int i = 0; i < network->n_layers; ++i) 	{
 		for (unsigned int j = 0; j < network->layers[i]->n_neurons; ++j) {
 			char chalkboard[9];
 			sprintf(chalkboard, "(%u, %u, ", network->layers[i]->neurons[j]->n_inputs, network->layers[i]->neurons[j]->activation_function);	
 			s += chalkboard;

 			// std::string sciNotationBuffer = std::string("0.00000000000000000000e+00) ");
 			char sciNotationBuffer[] = "0.00000000000000000000e+00) ";
 			my_print_scientific(sciNotationBuffer, network->layers[i]->neurons[j]->activation_steepness);
 			s += sciNotationBuffer;
 		}
 	}

 	// print connection information
	s += "\nconnections (connected_to_neuron, weight)=";
	for (unsigned int i = 0; i < network->n_layers; ++i) 	{
 		for (unsigned int j = 0; j < network->layers[i]->n_neurons; ++j) {
 			// s += sprintf("(%u, %u, ", i+j, );	

 			// get the sum of neurons before this layer.
 			// uint8_t sum = 0;
 			// for (int k = 0; k < network.n_layers; ++k) {
 			// 	sum += network.layers[k].n_neurons;
 			// }

 			// print the connections from each neuron to each of the neurons on the next layer.
 			// if ((i + 1) < network.n_layers) {
 			for (unsigned int k = 0; k < network->layers[i]->neurons[j]->n_connections; ++k) {
 				char chalkboard[5];
 				sprintf(chalkboard, "(%u, ", network->layers[i]->neurons[j]->connections[k]->connectedTo);
 				s += chalkboard;

 				// std::string sciNotationBuffer = std::string("0.00000000000000000000e+00) ");
 				char sciNotationBuffer[] = "0.00000000000000000000e+00) ";
	 			my_print_scientific(sciNotationBuffer, network->layers[i]->neurons[j]->connections[k]->connectionWeight);
	 			s+= sciNotationBuffer;
 			}
 			// }
 		}
 	}
}

void mutateFishBrain () {
;


}

void mutateFishDescriptor (fishDescriptor_t * fish, float mutationChance, float mutationSeverity) {

	// mutate heart rate
	if (RNG() > mutationChance) {	fish->heartSpeed += fish->heartSpeed *mutationSeverity*(RNG()-0.5); }

	for (int i = 0; i < N_FINGERS; ++i)
	{
		if (fish->bones[i].used) {
		// 	uint8_t attachedTo = 0; // the INDEX (out of N_FINGERS) of the bone it is attached to. Storing data in this way instead of a pointer means that mutating it will have hilarious rather than alarming results.
		// float length = 0.1f;
		// float rootThickness = 0.1f;
		// float tipThickness = 0.1f;
		// bool isRoot = false;
		// bool isMouth = false;
		// bool isSensor = false;
		// bool isWeapon  = false;
		// float torque = 0.0f;
		// float speedLimit = 0.0f;
		// float upperAngle = 0.0f;
		// float normalAngle = 0.0f;
		// float lowerAngle = 0.0f;
		// bool used = false;

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

void LoadFishFromName (uint8_t fishIndex, b2World * m_world, b2ParticleSystem * m_particleSystem) {

	fishDescriptor_t newFish;

	std::string fileName = "225";

	loadFishFromFile(fileName + std::string(".fsh"), newFish);

	fann * ann  = loadFishBrainFromFile (fileName);

	mutateFishDescriptor (&newFish, 0.1, 0.1) ;

 	loadFish ( fishIndex,  newFish, m_world,  m_particleSystem, ann ) ;

}

void jellyfishTrainer () {
	int n_inputs = 3;
	int n_outputs = 2;
	int n_examples = 10000;
	// there are actually 6*n_examples examples.

	float noise = 0.25f;
	float maxSensation = 0.95f;


	float bellAOpen = 0.3f;//0.9f;
	float bellAClose = -0.9f; //0.3f;
	float bellASoftClose = 0.3f;

	float bellBOpen = -0.3f;//0.9f;
	float bellBClose = 0.9f; //0.3f;
	float bellBSoftClose = -0.3f;

	FILE *fp;
    fp = fopen("jellyfishTrainer.data","wb");

    fprintf(fp, "%i %i% i\n", n_examples*6, n_inputs, n_outputs );

	for (int i = 0; i < n_examples; ++i)
	{

		float noiseThisTurn = ((RNG() - 0.5) * noise);
		float outputNoiseThisTurn = ((RNG() - 0.5) * noise) * 0.5;




		// float noisyHardOutputThisTurn = RNG() * hardOutput; 
		// float noisySoftOutputThisTurn = RNG() * softOutput;




		float sensationThisTurn = RNG() * maxSensation;




		float sensationAThisTime = (sensationThisTurn) + ((RNG() - 0.5) * noiseThisTurn);
		float sensationBThisTime =  ((RNG() - 0.5) * noiseThisTurn);

		
		// for a sense on side A, jiggle the bell on side B
			// one with heartbeat OFF, bell open
		// fprintf(fp, "%f %f %f %f\n", sensationAThisTime,sensationBThisTime,((RNG() - 0.5) * noiseThisTurn),0.0f);
		// fprintf(fp, "0 0 0 0.5 0\n");
		fprintf(fp, "%f %f %f\n", sensationAThisTime,sensationBThisTime,-0.9f + ((RNG() - 0.5) * noiseThisTurn));
		fprintf(fp, "%f %f\n", 
										bellAOpen + ((RNG() - 0.5) * outputNoiseThisTurn),
										bellBOpen +((RNG() - 0.5) * outputNoiseThisTurn)	
										);
										// ((RNG() - 0.5) * outputNoiseThisTurn));
		// fprintf(fp, "%f %f %f %f\n", sensationAThisTime,sensationBThisTime,((RNG() - 0.5) * noiseThisTurn),0.0f);
		// fprintf(fp, "0 0 0 0.5 0\n");


// fprintf(fp, "%d. %s\n", i+1, lang[i]);

			// one with heartbeat ON, bell closed
		// fprintf(fp, "%f %f %f %f\n", sensationAThisTime,sensationBThisTime,((RNG() - 0.5) * noiseThisTurn),1.0f);
		// fprintf(fp, "0 0 0.5 0 0\n");
		fprintf(fp, "%f %f %f\n", sensationAThisTime,sensationBThisTime,0.9f + ((RNG() - 0.5) * noiseThisTurn));
		fprintf(fp, "%f %f\n",			
										bellASoftClose + ((RNG() - 0.5) * outputNoiseThisTurn),
										bellBClose + ((RNG() - 0.5) * outputNoiseThisTurn)
										);
										// ((RNG() - 0.5) * outputNoiseThisTurn));
		// fprintf(fp, "%f %f %f %f\n", sensationAThisTime,sensationBThisTime,((RNG() - 0.5) * noiseThisTurn),1.0f);
		// fprintf(fp, "0 0 0.5 0 0\n");



		// for a sense on side B, jiggle the bell on side A
		sensationAThisTime = ((RNG() - 0.5) * noiseThisTurn);
		sensationBThisTime =  (sensationThisTurn) + ((RNG() - 0.5) * noiseThisTurn);
		
		// for a sense on side A, jiggle the bell on side B
			// one with heartbeat OFF, bell open
		// fprintf(fp, "%f %f %f %f\n", sensationAThisTime,sensationBThisTime,((RNG() - 0.5) * noiseThisTurn),0.0f);
		// fprintf(fp, "0.5 0 0 0 0\n");
		fprintf(fp, "%f %f %f\n", sensationAThisTime,sensationBThisTime,-0.9f + ((RNG() - 0.5) * noiseThisTurn));
		fprintf(fp, "%f %f\n", 			
										bellAOpen+((RNG() - 0.5) * outputNoiseThisTurn),
										bellBOpen + ((RNG() - 0.5) * outputNoiseThisTurn));
										// ((RNG() - 0.5) * outputNoiseThisTurn));
		// fprintf(fp, "%f %f %f %f\n", sensationAThisTime,sensationBThisTime,((RNG() - 0.5) * noiseThisTurn),0.0f);
		// fprintf(fp, "0.5 0 0 0 0\n");



			// one with heartbeat ON, bell closed
		// fprintf(fp, "%f %f %f %f\n", sensationAThisTime,sensationBThisTime,((RNG() - 0.5) * noiseThisTurn),1.0f);
		// fprintf(fp, "0 0.5 0 0 0\n");
		fprintf(fp, "%f %f %f\n", sensationAThisTime,sensationBThisTime,0.9f + ((RNG() - 0.5) * noiseThisTurn));
		fprintf(fp, "%f %f\n", 
										bellAClose + ((RNG() - 0.5) * outputNoiseThisTurn),
										bellBSoftClose + ((RNG() - 0.5) * outputNoiseThisTurn)
										);
										// ((RNG() - 0.5) * outputNoiseThisTurn));
		// fprintf(fp, "%f %f %f %f\n", sensationAThisTime,sensationBThisTime,((RNG() - 0.5) * noiseThisTurn),1.0f);
		// fprintf(fp, "0 0.5 0 0 0\n");

		// for straight ahead, jiggle both?

		sensationAThisTime = (sensationThisTurn) +((RNG() - 0.5) * noiseThisTurn);
		sensationBThisTime =  (sensationThisTurn) + ((RNG() - 0.5) * noiseThisTurn);
		
		// // for a sense on side A, jiggle the bell on side B
			// one with heartbeat OFF, bell open
		fprintf(fp, "%f %f %f\n", sensationAThisTime,sensationBThisTime,-0.9f + ((RNG() - 0.5) * noiseThisTurn));
		// fprintf(fp, "0.5 0 0 0.5 0\n");
		fprintf(fp, "%f %f\n", 			
										bellAOpen + ((RNG() - 0.5) * outputNoiseThisTurn),
										bellBOpen + ((RNG() - 0.5) * outputNoiseThisTurn));
										// ((RNG() - 0.5) * outputNoiseThisTurn));
		// fprintf(fp, "0.5 0 0 0.5 0\n");
			// one with heartbeat ON, bell closed
		fprintf(fp, "%f %f %f\n", sensationAThisTime,sensationBThisTime,0.9f + ((RNG() - 0.5) * noiseThisTurn));
		// fprintf(fp, "0 0.75 0.75 0 0\n");
		fprintf(fp, "%f %f\n",
										bellAClose + ((RNG() - 0.5) * outputNoiseThisTurn),
										bellBClose + ((RNG() - 0.5) * outputNoiseThisTurn));
										// ((RNG() - 0.5) * outputNoiseThisTurn));
		// fprintf(fp, "0 0.5 0.5 0 0\n");







		// // going just a little bit to one side.
		// sensationAThisTime =  (sensationThisTurn) +((RNG() - 0.5) * noiseThisTurn) - (0.5 * RNG() * sensationThisTurn); 
		// sensationBThisTime =  (sensationThisTurn) + ((RNG() - 0.5) * noiseThisTurn) + (0.5 * RNG() * sensationThisTurn);
		
		// // // for a sense on side A, jiggle the bell on side B
		// 	// one with heartbeat OFF, bell open
		// fprintf(fp, "%f %f %f\n", sensationAThisTime,sensationBThisTime,-0.9f + ((RNG() - 0.5) * noiseThisTurn));
		// // fprintf(fp, "0.5 0 0 0.5 0\n");
		// fprintf(fp, "%f %f %f %f\n", noisySoftOutputThisTurn + ((RNG() - 0.5) * outputNoiseThisTurn),
		// 								((RNG() - 0.5) * outputNoiseThisTurn),
		// 								((RNG() - 0.5) * outputNoiseThisTurn),
		// 								noisyHardOutputThisTurn + ((RNG() - 0.5) * outputNoiseThisTurn));
		// 								// ((RNG() - 0.5) * outputNoiseThisTurn));
		// // fprintf(fp, "0.5 0 0 0.5 0\n");
		// 	// one with heartbeat ON, bell closed
		// fprintf(fp, "%f %f %f\n", sensationAThisTime,sensationBThisTime,0.9f + ((RNG() - 0.5) * noiseThisTurn));
		// // fprintf(fp, "0 0.75 0.75 0 0\n");
		// fprintf(fp, "%f %f %f %f\n", ((RNG() - 0.5) * outputNoiseThisTurn),
		// 								noisySoftOutputThisTurn + ((RNG() - 0.5) * outputNoiseThisTurn),
		// 								noisySoftOutputThisTurn + ((RNG() - 0.5) * outputNoiseThisTurn),
		// 								((RNG() - 0.5) * outputNoiseThisTurn));
		// 								// ((RNG() - 0.5) * outputNoiseThisTurn));
		// // fprintf(fp, "0 0.5 0.5 0 0\n");





		// 	// and the other side.
		// sensationAThisTime =  (sensationThisTurn) +((RNG() - 0.5) * noiseThisTurn) + (0.5 * RNG() * sensationThisTurn); 
		// sensationBThisTime =  (sensationThisTurn) + ((RNG() - 0.5) * noiseThisTurn) - (0.5 * RNG() * sensationThisTurn);
		
		// // // for a sense on side A, jiggle the bell on side B
		// 	// one with heartbeat OFF, bell open
		// fprintf(fp, "%f %f %f\n", sensationAThisTime,sensationBThisTime,-0.9f + ((RNG() - 0.5) * noiseThisTurn));
		// // fprintf(fp, "0.5 0 0 0.5 0\n");
		// fprintf(fp, "%f %f %f %f\n", noisyHardOutputThisTurn + ((RNG() - 0.5) * outputNoiseThisTurn),
		// 								((RNG() - 0.5) * outputNoiseThisTurn),
		// 								((RNG() - 0.5) * outputNoiseThisTurn),
		// 								noisySoftOutputThisTurn + ((RNG() - 0.5) * outputNoiseThisTurn));
		// 								// ((RNG() - 0.5) * outputNoiseThisTurn));
		// // fprintf(fp, "0.5 0 0 0.5 0\n");
		// 	// one with heartbeat ON, bell closed
		// fprintf(fp, "%f %f %f\n", sensationAThisTime,sensationBThisTime,0.9f + ((RNG() - 0.5) * noiseThisTurn));
		// // fprintf(fp, "0 0.75 0.75 0 0\n");
		// fprintf(fp, "%f %f %f %f\n", ((RNG() - 0.5) * outputNoiseThisTurn),
		// 								noisySoftOutputThisTurn + ((RNG() - 0.5) * outputNoiseThisTurn),
		// 								noisySoftOutputThisTurn + ((RNG() - 0.5) * outputNoiseThisTurn),
		// 								((RNG() - 0.5) * outputNoiseThisTurn));
		// 								// ((RNG() - 0.5) * outputNoiseThisTurn));
		// // fprintf(fp, "0 0.5 0.5 0 0\n");












	}


	// 
  // myfile.close();

	fclose(fp);

}

void deepSeaSetup (b2World * m_world, b2ParticleSystem * m_particleSystem, DebugDraw * p_debugDraw) {

	jellyfishTrainer();

	// printf("%f\n", RNG());

	// store the debugdraw pointer in here so we can use it.
	local_debugDraw_pointer = p_debugDraw;

	addFoodParticle(b2Vec2(2.5f, 3.5f), m_world, m_particleSystem);





	// create a neurodescriptor from the saved fann file.
	// createNeurodescriptorFromFile();
	createNeurodescriptorFromFANN();

	// print the neurodescriptor parameters.


	// output a fann file from the created descriptor.


// 

	for (int i = 0; i < N_FISHES; ++i) {
		LoadFishFromName(i, m_world, m_particleSystem);
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
        // return EXIT_FAILURE;
    }

    con = (fann_connection *)calloc(connum, sizeof(*con));
    if (con == NULL) {
        fprintf(stderr, "Error: unable to allocate memory\n");
        // return EXIT_FAILURE;
    }

    /* Get weight matrix */
    fann_get_connection_array(ann, con);


	b2Vec2 drawingStartingPosition = b2Vec2( (2.0f * index) ,4.0f);
	float spacingDistance = 0.5f;

	// float max = 0.0f;

	// for (int j = 0; j < layerArray[0]; ++j)
	// {
	// 	if (motorSignals[j]) > max {
	// 		max = motorSignals[j];
	// 	}
	// }

	// if (max == 0.0f) {
	// 	max = 0.01f;
	// }

	// float ratio = 1/max;

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
		// get output

		// do motor control

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
			// printf("i: %i\n", i);
			food[i]->position = food[i]->p_body->GetPosition(); // update positions of all the food particles

			// printf("food position ");printab2Vec2(food[i]->p_body->GetPosition()); // this number is bugged, go fix
		}
	}

	for (int i = 0; i < N_FISHES; ++i) {

		if (!fishSlotLoaded[i]) {
			break;
		}

		// cause heart to beat
		if (fishes[i]->heartCount > fishes[i]->heartSpeed) {
			fishes[i]->heartCount = 0;
			// fishes[i]->heartOutput = !fishes[i]->heartOutput; // NOT operator used on a uint8_t.

			if (fishes[i]->heartOutput > 0) {
				fishes[i]->heartOutput = -0.95;
			}
			else {
				fishes[i]->heartOutput = 0.95;
			}

		}
		else {
			fishes[i]->heartCount++;
		}

		for (int j = 0; j < N_FINGERS; ++j)
		{
			// run sensors
			nonRecursiveSensorUpdater (fishes[i]->bones[j]);
		}

		float range = fishes[i]->bones[1]->sensation - fishes[i]->bones[2]->sensation;

		
		float senseA = 0;// fishes[i]->bones[1]->sensation ;//(2* abs(range) + range);
		float senseB = 0;//fishes[i]->bones[2]->sensation ;//(2* abs(range) - range);
		if (fishes[i]->bones[1]->sensation > fishes[i]->bones[2]->sensation) {
			senseA=1.0f;
		}
		else {
			senseB=1.0f;
		}

		if( abs(range) < 0.0001) {
			senseA=1.0f + (RNG()* 0.5 * (RNG()-0.5f));
			senseB=1.0f + (RNG() * 0.5 * (RNG()-0.5f));;
		}


		if (i == 0) { // if fish is player, #0

			if (userControlInputA || userControlInputB) {
				senseA = 0;
				senseB = 0;
			}

			if (userControlInputA) { 
				senseA = 1.0;
				userControlInputA = false;
			}	
			if (userControlInputB) { 
				senseB = 1.0;
				userControlInputB = false;
			}
		}

		
		float sensorium[3] = {senseA, senseB, (float)fishes[i]->heartOutput * 100};


		// 
		// float visualAdjustedMotorSignals[4];



			// feed information into brain
		float * motorSignals = fann_run(fishes[i]->ann, sensorium);

		// float maxOutputThisTurn = 0.0f;

		// ceiling and floor the motor output
		// for (int j = 0; j < 4; ++j){
		// 	if (motorSignals[i] > 1.0f) {
		// 		motorSignals[i] = 1.0f;
		// 	}
		// 	if (motorSignals[i] < 0.0f) {
		// 		motorSignals[i] = 0.001f; // not zero, lol.
		// 	}
		// 	if (motorSignals[i] > maxOutputThisTurn) {
		// 		maxOutputThisTurn = motorSignals[i];
		// 	}
		// }

		printf("motor: %.2f %.2f ", motorSignals[0], motorSignals[1]);//, motorSignals[2], motorSignals[3]);

		// output compressor. now that everything is clipped to range(0,1), autoscale it so that the biggest is increased to 1.
		// float ratio = 1/maxOutputThisTurn;
		// for (int i = 0; i < 4; ++i) {
		// 	motorSignals[i] = motorSignals[i] * ratio;
		// }

		// for (int j = 0; j < 4; ++j){
		// 	if (motorSignals[i] > 1.0f) {
		// 		motorSignals[i] = 1.0f;
		// 	}
		// 	if (motorSignals[i] < 0.0f) {
		// 		motorSignals[i] = 0.001f; // not zero, lol.
		// 	}
		// 	if (motorSignals[i] > maxOutputThisTurn) {
		// 		maxOutputThisTurn = motorSignals[i];
		// 	}
		// }



		// float speedForJointA = motorSignals[0] - motorSignals[1];
		// float speedForJointB = motorSignals[2] - motorSignals[3];

		if (true) {



			float jointAngleA = fishes[i]->bones[1]->joint->p_joint->GetJointAngle();
			float jointAngleB = fishes[i]->bones[2]->joint->p_joint->GetJointAngle();

			printf("joint: %.2f %.2f\n", jointAngleA, jointAngleB);


			fishes[i]->bones[1]->joint->p_joint->SetMotorSpeed(motorSignals[0]*fishes[i]->bones[1]->joint->speedLimit); //speedForJointA*10);
			fishes[i]->bones[2]->joint->p_joint->SetMotorSpeed(motorSignals[1]*fishes[i]->bones[2]->joint->speedLimit);//speedForJointB*10);
		}

		// print the brainal output
		drawNeuralNetwork( fishes[i]->ann, motorSignals, sensorium, i);
	}
}

void deepSeaControlA () {
	// fishes[0]->bones[2]->joint->p_joint->SetMotorSpeed(1.0f);
	// printf("deepSeaControlA\n");

}
void deepSeaControlB () {
	// fishes[0]->bones[2]->joint->p_joint->SetMotorSpeed(-1.0f);
}



void collisionHandler (void * userDataA, void * userDataB) {

	// if (boneB->isMouth) {
	// 	printf("f");
	// }

	bool et = false;
	bool fud = false;



	// if(dynamic_cast<BoneUserData*>((BoneUserData*)userDataA)) // this is, apparently, terribly bad practice. // https://stackoverflow.com/questions/11951121/checking-if-a-pointer-points-to-a-particular-class-c
	// { // if boneA is a BoneUserData
	//   // printf("i didnt fuck up");
	// 	et = true;
	// }

	// if(dynamic_cast<foodParticle_t*>((foodParticle_t*)userDataB)) // this is, apparently, terribly bad practice. // https://stackoverflow.com/questions/11951121/checking-if-a-pointer-points-to-a-particular-class-c
	// { // if boneA is a BoneUserData
	//   // printf("i didnt fuck up");
	//   fud = true;
	// }


	if (userDataA == nullptr || userDataB == nullptr) {
		return;
	}
	else {
		// printf("theprtwaedsasnit");
	}

	uDataWrap * p_dataA = (uDataWrap *) userDataA;
	uDataWrap * p_dataB = (uDataWrap *) userDataB;

	uDataWrap dataA = *p_dataA;
	uDataWrap dataB = *p_dataB;



	// printf("A %u, B %u\n", dataA.dataType, dataB.dataType);



	if( dataA.dataType == TYPE_MOUTH ) { //
		et = true;
		// printf("mouth collided");
		if( dataB.dataType == TYPE_FOOD ) {
		fud = true;
		// printf("food collided");
	}
	}

	if( dataB.dataType == TYPE_MOUTH ) {
		et = true;
		// printf("mouth collided");
		if( dataA.dataType == TYPE_FOOD ) {
		fud = true;
		// printf("food collided");
	}
	}
	
	


	if (et && fud) {
		printf("monch");
	}


		// if (boneA == isMouth) {

	// }

}