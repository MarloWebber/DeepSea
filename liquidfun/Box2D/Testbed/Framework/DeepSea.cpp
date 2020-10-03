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

#include <math.h>
// #include <iostream>
#include <cmath>

int currentNumberOfFood = 0;
int currentNumberOfFish = 0;
int generationsThisGame = 0;
bool startNextGeneration = false;
// bool readyToEnterScienceMode = false;

// // int selected = 0;
// bool scienceMode = false;

bool fishSlotLoaded[N_FISHES];
bool foodSlotLoaded[N_FOODPARTICLES];

bool userControlInputA;
bool userControlInputB;

b2World * local_m_world = nullptr;
b2ParticleSystem * local_m_particleSystem = nullptr;
DebugDraw * local_debugDraw_pointer = nullptr;

b2World * local_m_world_sci = nullptr;
b2ParticleSystem * local_m_particleSystem_sci = nullptr;
DebugDraw * local_debugDraw_pointer_sci = nullptr;


float pi = 3.14159f;

void setUserControlInputA() {
userControlInputA = true;
}

void setUserControlInputB () {
userControlInputB = true;
}

// these FANN parameters should be common to all networks.
const float desired_error = (const float) 0.01;
const unsigned int max_epochs = 50000;
const unsigned int epochs_between_reports = 100;

foodParticle_t * food[N_FOODPARTICLES];
BonyFish * fishes[N_FISHES];
BonyFish * sciFish;
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

JointUserData::JointUserData(boneAndJointDescriptor_t boneDescription, BoneUserData * p_bone, BonyFish * fish) {
	torque = boneDescription.torque ;
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
	attachedTo = 0; // the INDEX (out of N_FINGERS) of the bone it is attached to. Storing data in this way instead of a pointer means that mutating it will have hilarious rather than alarming results.
	length = 0.5f;
	rootThickness = 0.2f;
	tipThickness = 0.2f;
	isRoot = false;
	isMouth = false;
	sensor_radar = true; // like an olfactory sensor . senses distance from food
	sensor_touch = false; // like how you can feel when things touch your skin.
	sensor_jointangle = true;
	isWeapon  = false;
	torque = 200.0f,	// torque
	speedLimit =		10.0f,	// speedLimit
	upperAngle = pi  * 0.9f,// upperAngle
	normalAngle = 		0.0f,	// normalAngle
	lowerAngle = 		pi * -0.9f,	// lowerAngle
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

		   return  b2Vec2(  (RNG()-0.5) * 25, (RNG()-0.5) * 5.0f  );
}

BoneUserData::BoneUserData(
		boneAndJointDescriptor_t boneDescription,
		BonyFish * fish,
		b2Vec2 positionOffset,
		int newCollisionGroup
	) {

	if (!boneDescription.used) {
		return;
	}

	// printf("creating a bone\n");

	p_owner = fish;
	BoneUserData * attachesTo = fish->bones[boneDescription.attachedTo];	

	// initialize everything to default, sane values
	length = boneDescription.length;
	rootThickness = boneDescription.rootThickness;
	tipThickness = boneDescription.tipThickness;
	density = 1.2f; 														// the original density of the water is 1.2f
	isRoot = boneDescription.isRoot;
	isMouth = boneDescription.isMouth;
	// isSensor = boneDescription.isSensor;
	// isTouchSensor = boneDescription.isTouchSensor;

	sensor_touch = boneDescription.sensor_touch;
	sensor_radar = boneDescription.sensor_radar;
	sensor_jointangle = boneDescription.sensor_jointangle;

	sensation_radar = 0.0f;
	sensation_touch = 0.0f;
	sensation_jointangle = 0.0f;

	// sensation = 0.0f;
	// touchSensation = 0.0f;
	isWeapon  = boneDescription.isWeapon;									// weapons destroy joints to snip off a limb for consumption. optionally, they can produce a physical effect.
	energy = ((rootThickness + tipThickness)/2) * (length * density); 		// the nutritive energy stored in the tissue of this limb; used by predators and scavengers

	color = boneDescription.color;
	outlineColor = boneDescription.outlineColor;

	tipCenter = b2Vec2(0.0f,0.1f); 											// these are used so the skeleton master can remember his place as he traverses the heirarchy of souls.
	rootCenter = b2Vec2(0.0f,0.0f); 	

	int count = 4;

	collisionGroup = newCollisionGroup;

	// the following code is used to generate box2d structures and shapes from the bone parameters.
	if (isRoot) {

		offsetOnBody = b2Vec2(0.0f, 0.0f);

		// printf("its a root bone\n");
		tipCenter = b2Vec2(0.0f, length);

		b2Vec2 vertices[] = {
			b2Vec2( + (rootThickness/2),  -(length/2)), //b2Vec2 rootVertexA = 
			b2Vec2(- (rootThickness/2),  -(length/2)), // b2Vec2 rootVertexB =
			b2Vec2( + (tipThickness/2),  +(length/2)), //b2Vec2 tipVertexA = 
			b2Vec2(- (tipThickness/2),  +(length/2)) // b2Vec2 tipVertexB = 
		};
		
		if (isMouth) {
			uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_MOUTH);
			bodyDef.userData = (void *)p_dataWrapper;
		}
		else if (sensor_touch) {

			uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_TOUCHSENSOR);
			bodyDef.userData = (void *)p_dataWrapper;
			}
			else {

				uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_DEFAULT);
				bodyDef.userData = (void *)p_dataWrapper;
			}

		
		
		bodyDef.type = b2_dynamicBody;

		// bodyDef.filter.groupIndex = -2; // if it is root, the filter group index is 2. then it alernates between 2 and 4 for all subsequent sets of bones.

		p_body = local_m_world->CreateBody(&bodyDef);
		
		shape.Set(vertices, count);

		// reference the physics object from the user data.
		tipCenter = tipCenter;
		rootCenter = b2Vec2(0.0f, 0.0f);
	}
	else {

			offsetOnBody = (attachesTo->offsetOnBody + (attachesTo->length/2)) + length/2;

		// bones in a set can't collide with each other.

		 tipCenter = b2Vec2(0.0f,  length);

		b2Vec2 vertices[] = {
			b2Vec2( + (rootThickness/2), -(length/2)),
			b2Vec2( - (rootThickness/2),  -(length/2)),
			b2Vec2( + tipThickness/2, +(length/2)),
			b2Vec2( - tipThickness/2, +(length/2))
		};

		// attach user data to the body
		if (isMouth) {
			uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_MOUTH);
			bodyDef.userData = (void *)p_dataWrapper;
		}
		else if (sensor_touch) {

			uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_TOUCHSENSOR);
			bodyDef.userData = (void *)p_dataWrapper;
			}
			else {

				uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_DEFAULT);
				bodyDef.userData = (void *)p_dataWrapper;
			}

		bodyDef.type = b2_dynamicBody;

		// move the body to the appropriate position on the model.
		p_body = local_m_world->CreateBody(&bodyDef);

		p_body->SetTransform(b2Vec2(positionOffset.x, positionOffset.y + offsetOnBody.y),0);

		// printf("tip center: ");
		// printab2Vec2(tipCenter);
		
		for (int i = 0; i < 4; ++i)
		{
			// printf(" ");
			printab2Vec2(vertices[i]);
		}
		
		// shape.SetAsBox(rootThickness, length, boneCenter, 0.0f);	
		shape.Set(vertices, count);

		// reference the physics object from the user data.
		tipCenter = tipCenter;
		rootCenter = attachesTo->tipCenter;
	}

	if (!isRoot) {
		joint = new JointUserData( boneDescription, this, fish); 	// the joint that attaches it into its socket 
	}	

	init = true;
	isUsed=  false;
	printf("bone complete\n");
};

void nonRecursiveBoneIncorporator(BoneUserData * p_bone, uint8_t boneIndex) {
	if (!p_bone->init) {
		return;
	}

	p_bone->p_fixture = p_bone->p_body->CreateFixture(&(p_bone->shape), p_bone->density);	// this endows the shape with mass and is what adds it to the physical world.


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





			p_bone->sensation_jointangle= p_bone->joint->p_joint->GetJointAngle();// - pi;


		}
	}
	
	if (p_bone->sensor_radar) {
		p_bone->sensation_radar = 0.0f;
		for  (int i = 0; i < N_FOODPARTICLES; i++) {
			if (!foodSlotLoaded[i]) {
				break;
			}
			if (food[i]->init && food[i]->isUsed) {
				b2Vec2 boneCenterWorldPosition = p_bone->p_body->GetWorldCenter();
				b2Vec2 positionalDifference = b2Vec2((boneCenterWorldPosition.x - food[i]->position.x),(boneCenterWorldPosition.y - food[i]->position.y));
				float distance = magnitude (positionalDifference);
				if (distance > 0) {

					// printf("%f\n", distance);

					p_bone->sensation_radar += 1/distance;
				}
			}
		}
	}
	
}

// add a food particle to the world and register it so the game knows it exists.
foodParticle_t::foodParticle_t ( b2Vec2 position) {	
	energy = 1.0f;
	uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_FOOD);
	bodyDef.userData = (void*)p_dataWrapper;
	bodyDef.type = b2_dynamicBody;
	p_body = local_m_world->CreateBody(&bodyDef);

	b2Vec2 vertices[] = {
		b2Vec2(-0.25, -0.25), 
		b2Vec2(0.25, -0.25), 
		b2Vec2(0.25, 0.25), 
		b2Vec2(-0.25, 0.25)
	};
		
	shape.Set(vertices, 4);

	p_body->CreateFixture(&shape, 1.2f);
	init = true;
	isUsed = true;
};

void addFoodParticle(b2Vec2 position) {
	food[currentNumberOfFood] = new foodParticle_t( position);
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
	// unsigned int num_layers = fann_get_num_layers(temp_ann);
	// unsigned int layerCake[num_layers];
	// fann_get_layer_array(temp_ann, layerCake);
	return new networkDescriptor(temp_ann);
}


BonyFish::BonyFish(fishDescriptor_t driedFish, uint8_t fishIndex, fann * nann, b2Vec2 startingPosition) {
	genes = driedFish;
	hunger = 0.0f; // the animal spends energy to move and must replenish it by eating

	flagDelete = false;
	selected = false;

	// heartCountA = 0;
	// heartCountB = 0;
	// heartCountC = 0;
	// heartCountD = 0;

	int randomCollisionGroup = - (RNG() * 16.0f);
		// collisionGroup = newCollisionGroup;//randomCollisionGroup ;

	for (int i = 0; i < N_FINGERS; ++i) {
		if (i == 0) {
			driedFish.bones[i].isRoot = true;
		}
		bones[i] = new BoneUserData(driedFish.bones[i], this, startingPosition, randomCollisionGroup);
	}

	n_bones_used = 0;
	for (int i = 0; i < N_FINGERS; ++i) {
		if (driedFish.bones[i].used) {
			n_bones_used ++;

		}
	}

	init = true; // true after the particle has been initialized. In most cases, uninitalized particles will be ignored.
	isUsed = false; // only true when the part is added to the world

	// heartSpeed = driedFish.heartSpeed;// +  ((RNG()-0.5) * driedFish.heartSpeed * 0.5);


		unsigned int senseInputsUsedSoFar = 0;
		unsigned int motorOutputsUsed = 0;
	for (int i = 0; i < 32; ++i)
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
				// driedFish.bones[i].outlineColor = b2Color(RNG()* 255, RNG() * 255, RNG() * 255);

				// if (driedFish.bones[i].sensor_radar) {
				// 	// inputMatrix[senseInputsUsedSoFar].sensorType = SENSOR_FOODRADAR;
				// }
				// if (driedFish.bones[i].sensor_touch) {
				// 	// inputMatrix[senseInputsUsedSoFar].sensorType = SENSOR_TOUCH;
				// }
				// if (driedFish.bones[i].sensor_jointangle) {
				// 	// inputMatrix[senseInputsUsedSoFar].sensorType = SENSOR_JOINTANGLE;
				// }
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

		    // wormTrainer();

		    // fann_train_on_file(ann, "wormTrainer.data", max_epochs, epochs_between_reports, desired_error);
	    }
    else { // a brain is provided
    	ann = nann;

    	brain = createEmptyNetworkOfCorrectSize (nann) ;
    }
};


bool fishChecker(unsigned int fishIndex) {
	if ( fishes[fishIndex] == NULL || fishes[fishIndex] == nullptr) { 	return false; }
		if (fishSlotLoaded[fishIndex] ) {
			return true;
		}
		else {
			return false;
		}
}

void moveAWholeFish (unsigned int fishIndex, b2Vec2 position) {
	if ( fishes[fishIndex] == NULL || fishes[fishIndex] == nullptr) { 	return; }
	if (fishSlotLoaded[fishIndex] ) {
		for (int i = 0; i < N_FINGERS; ++i)
		{
			if ( !fishes[fishIndex]->bones[i]->isUsed && !fishes[fishIndex]->bones[i]->init) {
					continue;
			}
			fishes[fishIndex]->bones[i]->p_body->SetTransform(position, 0.0f);
		}
	}
}

void totalFishIncorporator (uint8_t fishIndex) {
	for (int i = 0; i < N_FINGERS; ++i) {
		if (fishes[fishIndex]->bones[i]->init) {
			nonRecursiveBoneIncorporator( fishes[fishIndex]->bones[i] , i);
		}
	}
}

// delete a fish from the game world and remove it from memory
void deleteFish (uint8_t fishIndex) {

	if (fishSlotLoaded[fishIndex]) {

		// printf("deleting %i of 8.", fishIndex);
		if (!fishChecker(fishIndex)) {
			return;
		}
	
		for (int i = 0; i < N_FINGERS; ++i) {
			if ( !fishes[fishIndex]->bones[i]->isUsed && !fishes[fishIndex]->bones[i]->init) {
						continue;
			}
			else {
				if (!fishes[fishIndex]->bones[i]->isRoot) { // root bones dont have joints
					if (fishes[fishIndex]->bones[i]->joint->isUsed) {
							local_m_world->DestroyJoint(fishes[fishIndex]->bones[i]->joint->p_joint);	
							fishes[fishIndex]->bones[i]->joint = nullptr;
					}
				}
			}
			
		}

		for (int i = 0; i < N_FINGERS; ++i) {
			if ( !fishes[fishIndex]->bones[i]->isUsed && !fishes[fishIndex]->bones[i]->init) {
				continue;
			}

			if (fishes[fishIndex]->bones[i]->isUsed && fishes[fishIndex]->bones[i]->init) {
				// printf("deleting bone %i of 8.", i);

				 local_m_world->DestroyBody(fishes[fishIndex]->bones[i]->p_body);
				 fishes[fishIndex]->bones[i]->p_body = nullptr;
				 fishes[fishIndex]->bones[i]->isUsed = false;
				 fishes[fishIndex]->bones[i]->init = false;
			}
		}
	}

	fishes[fishIndex]->isUsed = false;
	fishSlotLoaded[fishIndex] = false;
}

void loadFish (uint8_t fishIndex, fishDescriptor_t driedFish, fann * nann, b2Vec2 startingPosition) {
	fishes[fishIndex] = new BonyFish(driedFish, fishIndex, nann, startingPosition);
	fishes[fishIndex]->slot = fishIndex;
	fishSlotLoaded[fishIndex] = true;
}

// void loadFishForScience (fishDescriptor_t driedFish, fann * nann, b2Vec2 startingPosition) {
// 	// return new BonyFish(driedFish, fishIndex, nann, startingPosition);
// 	sciFish = new BonyFish(driedFish, 0, nann, startingPosition);
// }

fann * loadFishBrainFromFile (std::string fileName) {




	return fann_create_from_file( (fileName + std::string(".net")).c_str() );
}

connectionDescriptor::connectionDescriptor (int toNeuron) {
	isUsed = false;
	connectedTo = toNeuron;
	connectionWeight = 0.0f;	
}

neuronDescriptor::neuronDescriptor() {
	n_connections = 0;
	n_inputs = 0;
	isUsed = false;
	// for (int i = 0; i < 12; ++i)
	// {
	// 	connections[i] = connectionDescriptor();
	// };
	aabb.upperBound = b2Vec2(0.0f,0.0f);
	aabb.lowerBound = b2Vec2(0.0f,0.0f);
	selected = false;
}

layerDescriptor::layerDescriptor () {
	n_neurons = 0;
	isUsed = false;
	// for (int i = 0; i < 8; ++i)
	// {
	// 	neurons[i] = neuronDescriptor();
	// };
}

senseConnector::senseConnector () {
	connectedToLimb = 0;		// what limb the sense is coming from, or motor signal is going to.
	connectedToNeuron = 0;		// neuron index. The position of this neuron's layer will determine how the program uses it.
	sensorType =  SENSECONNECTOR_UNUSED; 			// what kind of sense it is (touch, smell, etc.how the number will be treated)
	timerFreq = 0;				// if a timer, the frequency.

}

fishDescriptor_t::fishDescriptor_t () {
	
	//initializes the blank fish as the 4 segment nematode creature.

	// heartSpeed = 50;

	for (int i = 0; i < 32; ++i)
	{
		senseConnector moshuns = senseConnector();
		outputMatrix[i] = moshuns;
		inputMatrix[i] = moshuns;
	}

	for (int i = 0; i < 4; ++i)
	{
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
			// bones[i].isRoot = false;
			// bones[i].isMouth = false;
		}

		// bones[i].sensor_radar =true; 	// like an olfactory sensor . senses distance from food
		// bones[i].sensor_touch = false; 		// like how you can feel when things touch your skin.
		// bones[i].sensor_jointangle;
		bones[i].used = true;
	}

	for (unsigned int i = 0; i < 4; ++i)
	{
		// senseConnector moshuns = senseConnector();
		outputMatrix[i].connectedToLimb = i;
		outputMatrix[i].sensorType = SENSECONNECTOR_MOTOR;
		// outputMatrix[i] = moshuns;
	}
	unsigned int j = 0;
	for (unsigned int i = 0; i < 4; ++i)
	{

		// senseConnector moshuns = senseConnector();
		inputMatrix[j].connectedToLimb = i;
		inputMatrix[j].sensorType = SENSOR_FOODRADAR;
		// inputMatrix[j] = moshuns;
		j++;
	// }
	// for (unsigned int i = 0; i < 4; ++i)
	// {
		// senseConnector moshuns = senseConnector();
		inputMatrix[j].connectedToLimb = i;
		inputMatrix[j].sensorType = SENSOR_JOINTANGLE;
		// inputMatrix[j] = moshuns;
		j++;
	}
	for (unsigned int i = 0; i < 4; ++i)
	{
		// senseConnector moshuns = senseConnector();
		inputMatrix[j].connectedToLimb = 0;
		inputMatrix[j].sensorType = SENSOR_TIMER;
		// float mint=  i + 1;
		// float nugangeouds = (mint*mint);

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




		// inputMatrix[j].timerFreq = nugangeouds;
		inputMatrix[j].timerPhase = RNG(); //0;
		// inputMatrix[j] = moshuns;
		j++;

	}

	

}


// neuronDescriptor * networkDescriptor::getNeuronByIndex (unsigned int windex) {

// 	printf("getting neuron by index: %i\n", windex);

// 	std::list<layerDescriptor>::iterator layerIterator = layers.begin();
	
// 		// unsigned int layer = 0;
//   		unsigned int index = windex; // so as to not start from 0
//   		while (1) {

//   			// printf("index is: %i in layer of %lu\n", index, (unsigned long)layerIterator->neurons.size());

//   			if (index < layerIterator->neurons.size()+1) { // if the index is a valid position in this layer
//   				break; }
//   			else {
//   				index -= layerIterator->neurons.size();
//   				std::advance(layerIterator, 1);

//   				// printf("advanced over layer with %lu neurons\n", (unsigned long)layerIterator->neurons.size());
//   				// layer ++;
//   			}
//   		}
  		
//   		std::list<neuronDescriptor>::iterator neuronIterator = layerIterator->neurons.begin();
//   		// index--; // to undo the +1;

//   		// printf("landed on layer %i neuron %i\n", layer, index);

//   		std::advance(neuronIterator, index);
  		
//   		return &*neuronIterator; // turn the interator into a pointer https://stackoverflow.com/questions/2754650/getting-value-of-stdlistiterator-to-pointer


  		
  		

  		
  		



// }





// // neuronDescriptor * 
// void indexAllNeurons (networkDescriptor * network) {
// 		// std::list<layerDescriptor>::iterator layerIterator = network->layers.begin();
	
//   // 		unsigned int index = windex; // so as to not start from 0
//   // 		while (1) {
//   // 			// printf("index is: %i in layer of %lu\n", index, (unsigned long)layerIterator->neurons.size());

//   // 			if (index < layerIterator->neurons.size()) { // if the index is a valid position in this layer
//   // 				break; }
//   // 			else {
//   // 				index -= layerIterator->neurons.size();
//   // 				std::advance(layerIterator, 1);
//   // 			}
//   // 		}

//   // 		std::list<neuronDescriptor>::iterator neuronIterator = layerIterator->neurons.begin();

//   // 		neuronIterator = layerIterator->neurons.begin();
  
//   // 		std::advance(neuronIterator, index);

//   // 		// neuronIterator->connections.push_back(connection);
  		
//   // 		// nConnectionsMade ++;

//   // 		return &(*neuronIterator);


// 	unsigned int rollingIndexCounter = 0;

// 	std::list<layerDescriptor>::iterator layer;
// 	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{
// 		std::list<neuronDescriptor>::iterator neuron;
//  		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {

//  			if (neuron -> index == windex) {
//  				return &(*neuron);
//  			}
//  		// 	std::list<connectionDescriptor>::iterator connection;
//  		// 	for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {
//  		// 		;
// 			// }
// 		}
// 	}



// }



neuronDescriptor * getNeuronByIndex (networkDescriptor * network, unsigned int windex) {

	std::list<layerDescriptor>::iterator layer;
	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{
		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {

 			if (neuron -> index == windex) {
 				return &(*neuron);
 			}
 		// 	std::list<connectionDescriptor>::iterator connection;
 		// 	for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {
 		// 		;
			// }
		}
	}


	return nullptr;

}







// method to create a network descriptor in memory
networkDescriptor::networkDescriptor (fann * pann) {

	// query the number of layers.
	unsigned int num_layers = fann_get_num_layers(pann);
	// printf("new %u layer networkDescriptor\n", num_layers);

  	unsigned int activation_function_hidden = 5;
  	float activation_steepness_hidden = 0.5f;
  	unsigned int activation_function_output = 0;
  	float activation_steepness_output = 0; 
  	
	// get the layer cake. because FANN provides layer information as an array of integers, this is just a temporary variable to hold it.
	unsigned int layerCake[num_layers];
	// int nConnectionsMade = 0;
	// flip the cake 
	fann_get_layer_array(pann, layerCake);

	b2AABB partywaist;

	networkWindow = partywaist;
	partywaist.lowerBound = b2Vec2(0.0f,0.0f);
	partywaist.upperBound = b2Vec2(0.0f,0.0f);



	// unsigned int numberOfConnectionsCreated = 0;
	// printf("constructing into memory\n");


	unsigned int rollingIndexCounter = 0;

	for (unsigned int i = 0; i < num_layers; ++i)
	{
	// printf("\n");

		layerDescriptor layer = layerDescriptor();
		

		unsigned int nNeuronsIncludingBias = layerCake[i];
		if (i == num_layers-1) {
			;
		}else {
			nNeuronsIncludingBias +=1;
		}

		for (unsigned int j = 0; j <nNeuronsIncludingBias; ++j)
		{
			neuronDescriptor neuron = neuronDescriptor();
			

			if (j == layerCake[i]-1) {
				neuron.biasNeuron = true;
				// neuron.index = 0;
			}
			else {
				neuron.biasNeuron = false;
			
			}

				neuron.index = rollingIndexCounter;
				printf("created neuron index %i\n", neuron.index);
				rollingIndexCounter ++;

			neuron.activation_function = activation_function_hidden;
  			neuron.activation_steepness = activation_steepness_hidden;
  			neuron.n_connections = 0; 	// so not used uninitialized
  			neuron.n_inputs = 0; 
  			neuron.isUsed = true;

  			// find the index by summing the previous layers and then adding the index in this layer.
  	// 		neuron.index =  0;
  	// 		for (unsigned int k = 0; k < i; ++k)
  	// 		{
  	// 			neuron.index += layerCake[k];
  	// 			// neuron.indexedex += 1; /// add 1 for the bias neuron.
  	// 		}
			// neuron.index += j;

  			// output neurons have a different function than the others. this applies to all in the last row.
  			if (i == num_layers-1) {
				neuron.activation_function = activation_function_output;
  				neuron.activation_steepness = activation_steepness_output;
  			}

			layer.neurons.push_back(neuron);
			
		}

		printf("-\n");

		// there is an additional bias neuron in each layer which always emits 1. this is apparently why the number of neurons in the fann file is always 1 too big.
		// except not in the last layer.
		// if (i == num_layers-1) {
		// 	;
		// }
		// else {
		// 	neuronDescriptor neuron = neuronDescriptor();
		// 	neuron.activation_function = activation_function_hidden;
		// 	neuron.activation_steepness = activation_steepness_hidden;
		// 	neuron.n_connections = 0; 	// so not used uninitialized
		// 	neuron.n_inputs = 0; 
		// 	neuron.isUsed = true;
		// 	neuron.biasNeuron = true;
		// 	// find the index by summing the previous layers and then adding the index in this layer.
	 //  			neuron.index =  0;
	 //  			for (unsigned int k = 0; k <= i; ++k)
	 //  			{
	 //  				neuron.index += layerCake[k];
	 //  				neuron.index += 1; /// add 1 for the bias neuron.
	 //  			}
		// 		neuron.index -=1; // i don't know why but it makes it work.

		
		// 	layer.neurons.push_back(neuron);

		// }
		


		// add a new layer descriptor
		this->layers.push_back(layer);

	}

	// to create the connection map, you must read in from the file.

	// newCake->n_layers = num_layers;

  	// figure out the total number of neurons, which is how they are indexed in FANN file.
  	unsigned int sumOfNeurons = 0;
  	for (unsigned int i = 0; i < num_layers; ++i) {
  		sumOfNeurons += layerCake[i];
  		// printf("a LAYER %i has %i neurons!\n", i, layerCake[i]);
  	}

  	std::list<layerDescriptor>::iterator layer;
  	unsigned int i = 0;
	unsigned int num_connections = fann_get_total_connections(pann);
  	// printf("%i total connections required\n", num_connections);

  	for (layer = this->layers.begin(); layer != this->layers.end(); ++layer)  {
  		// printf("climbdamger\n");
  		layer->n_neurons = layerCake[i];

  		layer->isUsed = true;
  		// printf ("created layer descriptor %i with %i neurons\n", layerCake[i], layer->n_neurons) ;

  		// iterate neurons in layer.
  		std::list<neuronDescriptor>::iterator neuron;
		for (neuron = layer->neurons.begin(); neuron != layer->neurons.end(); ++neuron) {
  			neuron->activation_function = activation_function_hidden;
  			neuron->activation_steepness = activation_steepness_hidden;
  			neuron->n_connections = 0; 	// so not used uninitialized
  			neuron->n_inputs = 0; 
  			neuron->isUsed = true;

  			// output neurons have a different function than the others. this applies to all in the last row.
  			if (i == num_layers-1) {
				neuron->activation_function = activation_function_output;
  				neuron->activation_steepness = activation_steepness_output;
  			}
  			// printf ("created neuron descriptor\n") ;
  		}
  	}

	// get connection and weight information.
	struct fann_connection margles[num_connections] ;
  	memset(&margles, 0x00, sizeof(fann_connection[num_connections]));
  	struct fann_connection *con = margles;


  	fann_get_connection_array(pann, con); // this DOES include bias neuron information. 

	// printf ("chunky borks and portly babies: %i\n", num_connections) ; // create connections
	
  	for (unsigned int c = 0; c < num_connections; ++c) {


		// printf("conc A to: %i, conc from: %i\n",con[c].to_neuron, con[c].from_neuron);

  	}


  	for (unsigned int c = 0; c < num_connections; ++c) {
		connectionDescriptor connection = connectionDescriptor(con[c].to_neuron);
		connection.connectionWeight = con[c].weight;

		// printf("A conn from: %i to: %i weight: %f\n",con[c].from_neuron ,con[c].to_neuron ,con[c].weight );

		//------------------
		// first find the neuron that the connection comes FROM.
		// std::list<layerDescriptor>::iterator layerIterator = this->layers.begin();
	
		getNeuronByIndex(this, con[c].from_neuron)->connections.push_back(connection);
  			
  		// neuronIterator->connections.push_back(connection);
  		
  		// nConnectionsMade ++;

  		

  		//---------------
  		// then find the one that it goes TO.

		// printf("conc to: %i, conc from: %i\n",con[c].to_neuron, con[c].from_neuron);

		getNeuronByIndex(this, con[c].to_neuron)->n_inputs++;

		// std::list<layerDescriptor>::iterator //
		// layerIterator = this->layers.begin();
		// std::list<layerDescriptor>::iterator layerIteratorB = this->layers.begin();

	
		// // unsigned int layer = 0;
  // 		index = con[c].to_neuron; // so as to not start from 0
  // 		while (1) {
  // 			// printf("index is: %i in layer of %lu\n", index, (unsigned long)layerIterator->neurons.size());

  // 			if (index < layerIteratorB->neurons.size()) { // if the index is a valid position in this layer
  // 				break; }
  // 			else {
  // 				index -= layerIteratorB->neurons.size();
  // 				std::advance(layerIteratorB, 1);
  // 			}
  // 		}

		// std::list<neuronDescriptor>::iterator neuronIteratorB = layerIteratorB->neurons.begin();
  // 		std::advance(neuronIteratorB, index); // then finally, move to the nth neuron in this layer, which is the one you were looking for.
  		
  		
  		
	}	
	// printf("number of connections produced: %i\n", nConnectionsMade);
					
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



//
fann * createFANNbrainFromDescriptor (networkDescriptor * network) {
	//create an empty fann brain of the right size and layer cake.

	unsigned int creationLayerCake[(unsigned long)network->layers.size()];	
	std::list<layerDescriptor>::iterator layer;
	unsigned int layerIndex = 0;

	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{


		// if you are not on the last layer, ignore bias neurons because they are included implicitly.
		if (layerIndex == ((unsigned long)network->layers.size() - 1 )) {
			creationLayerCake[layerIndex] = layer->neurons.size();
		}
		else {
			creationLayerCake[layerIndex] = layer->neurons.size() - 1;

		}

		layerIndex++;

	}

	    // unsigned int creationLayerCake[] = {
	    // 	28,
	    // 	8,
	    // 	8,
	    // 	8,
	    // 	8
	    // };
	    	fann * ann = fann_create_standard_array((unsigned long)network->layers.size(), creationLayerCake);
		    fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
		    fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);

		    // brain = createEmptyNetworkOfCorrectSize (ann) ;

		    unsigned int num_connections = fann_get_total_connections(ann);

	// create a connection array from the descriptor.

		    struct fann_connection margles[num_connections] ;
  	memset(&margles, 0x00, sizeof(fann_connection[num_connections]));
  	// struct fann_connection *con = margles; 

	// use 
		    // 





	// std::list<layerDescriptor>::iterator layer;
	layerIndex = 0;
	unsigned int neuronIndex = 0;
	unsigned int connectionIndex = 0;

	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{
		// printf("	layer %u neurons: %lu\n", layerIndex, (unsigned long)layer->neurons.size());

		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 			// printf("		neuron %u connections: %lu inputs: %u\n", neuronIndex, (unsigned long)neuron->connections.size(), neuron->n_inputs);


 			// figire out this nueorns inex

 			// unsigned int thisNeuronsIndex = 0;
 			// for (unsigned int i = 0; i < layerIndex; ++i)
 			// {
 			// 	thisNeuronsIndex += creationLayerCake[i];
 			// }
 			// thisNeuronsIndex += neuronIndex;

//  			printf(" neuron %i connections: %i\n", j, network->layers[i].neurons[j].n_connections);
 			std::list<connectionDescriptor>::iterator connection;
 			// for (unsigned int k = 0; k < fishes[fishIndex]->brain.layers[i].neurons[j].n_connections; ++k) {
 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {
 				// printf("			connection %u to: %u, weight:%f\n", connectionIndex, connection->connectedTo, connection->connectionWeight );

 				fann_connection conc;

 				conc.from_neuron = neuron->index;
 				conc.to_neuron = connection->connectedTo;
 				conc.weight = connection->connectionWeight;

 				margles[connectionIndex] = conc;

 				  	// printf("B conn from: %i to: %i weight: %f\n",conc.from_neuron ,conc.to_neuron ,conc.weight );



 				;
 				// printf(" |%u|, ", fishes[fishIndex]->brain.layers[i].neurons[j].connections[k].connectedTo); // <- it is already fucked up here.
				connectionIndex++;
			}
			neuronIndex ++;
		}
// 	printf("\n");
		layerIndex ++;
	}



fann_set_weight_array(ann, margles, num_connections);

return ann;

}



// method to create a fann save file from a network descriptor
// this function is buggy, and the whole neurodescriptor scheme does not really work yet. At the moment I am modifying the FANN text file directly.
void createFANNFileFromDescriptor (networkDescriptor * network) {
	std::string s = std::string("FANN_FLO_2.1\nnum_layers=");; // string to hold the information.
	s.append(std::to_string(network->layers.size()));

// 	// print this
 	s.append("\nlearning_rate=0.700000\nconnection_rate=1.000000\nnetwork_type=0\nlearning_momentum=0.000000\ntraining_algorithm=2\ntrain_error_function=1\ntrain_stop_function=0\ncascade_output_change_fraction=0.010000\nquickprop_decay=-0.000100\nquickprop_mu=1.750000\nrprop_increase_factor=1.200000\nrprop_decrease_factor=0.500000\nrprop_delta_min=0.000000\nrprop_delta_max=50.000000\nrprop_delta_zero=0.100000\ncascade_output_stagnation_epochs=12\ncascade_candidate_change_fraction=0.010000\ncascade_candidate_stagnation_epochs=12\ncascade_max_out_epochs=150\ncascade_min_out_epochs=50\ncascade_max_cand_epochs=150\ncascade_min_cand_epochs=50\ncascade_num_candidate_groups=2\nbit_fail_limit=3.49999994039535522461e-01\ncascade_candidate_limit=1.00000000000000000000e+03\ncascade_weight_multiplier=4.00000005960464477539e-01\ncascade_activation_functions_count=10\ncascade_activation_functions=3 5 7 8 10 11 14 15 16 17 \ncascade_activation_steepnesses_count=4\ncascade_activation_steepnesses=2.50000000000000000000e-01 5.00000000000000000000e-01 7.50000000000000000000e-01 1.00000000000000000000e+00\n");
 	s.append("layer_sizes=");

//  	// print layer sizes separated by a space

	// for (unsigned int i = 0; i < network.n_layers; ++i) {
 	// std::list<layerDescriptor>::iterator layerIterator = network->layers.begin();
 	// for (neuron = layer->neurons.begin(); neuron != layer->neurons.end(); ++neuron) {

 	std::list<layerDescriptor>::iterator layer;
	unsigned int layerIndex = 0;
	// unsigned int neuronIndex = 0;
	// unsigned int connectionIndex = 0;


	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{
		// printf("	layer %u neurons: %lu\n", layerIndex, (unsigned long)layer->neurons.size());


		s.append(std::to_string(layer->neurons.size() ));
		s.append(" ");
		layerIndex++;
	}
	// }

// 	// print activation information
 	s += "\nscale_included=0\nneurons (num_inputs, activation_function, activation_steepness)=";
//  	for (unsigned int i = network.n_layers-1; i > 0; --i)	{
		for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{


//  		for (unsigned int j = 0; j<network.layers[i].n_neurons ; ++j) {

			std::list<neuronDescriptor>::iterator neuron;
			int i = 0;
	 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 				char chalkboard[9];



 			// one of the layers in the file is supposed to have 0 inputs.
	 			// if (i == 0) {
					// 	sprintf(chalkboard, "(%u, %u, ", 0, 0);	
	 			// s += chalkboard;
	 			// }
	 			// else {
	 				sprintf(chalkboard, "(%u, %u, ", neuron->n_inputs, neuron->activation_function);	
		 			s += chalkboard;
	 			// }

	 			char sciNotationBuffer[] = "0.00000000000000000000e+00";
	 			my_print_scientific(sciNotationBuffer, neuron->activation_steepness);
	 			s += sciNotationBuffer;
	 			s.append(") ");
	 			i++;
	 		}
	 	}

	s += "\nconnections (connected_to_neuron, weight)=";
	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{
//  		for (unsigned int j = 0; j < network.layers[i].n_neurons;  ++j) {

			std::list<neuronDescriptor>::iterator neuron;
			// int i = 0;
	 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {

//  			for (unsigned int k = 0; k < network.layers[i].neurons[j].n_connections; ++k) {

 			std::list<connectionDescriptor>::iterator connection;
 			// for (unsigned int k = 0; k < fishes[fishIndex]->brain.layers[i].neurons[j].n_connections; ++k) {
 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {
 				char chalkboard[5];
 				sprintf(chalkboard, "(%u, ", connection->connectedTo);
 				s += chalkboard;
 				char sciNotationBuffer[30];
	 			my_print_scientific(sciNotationBuffer, connection->connectionWeight);
	 			s+= sciNotationBuffer;
	 			s.append(") ");
 			}
 		}
 	}

    std::ofstream out("Pleco.net");
    out << s;
    out.close();
}

// void mutateFishBrain (networkDescriptor * newCake, float mutationChance, float mutationSeverity) {
// 	// count connections.
// 	unsigned int sumOfConnections = 0;
// 	for (unsigned int i = 0; i < newCake->n_layers; ++i) 	{
//  		for (unsigned int j = 0; j < newCake->layers[i].n_neurons; ++j) {
// 			if (newCake->layers[i].neurons[j].n_connections > 8) {
// 				continue;
// 			}
// 			if (i > newCake->n_layers) {
// 				continue;
// 			}
// 			if (j > newCake->layers[i].n_neurons) {
// 				continue;
// 			}
//  			sumOfConnections += newCake->layers[i].neurons[j].n_connections;
//  		}
//  	}

//  	// while(1) {;}
// 	// traverse the mutated body and count the total number of sensors and hearts and limbs.
// 	// make sure the number of inputs matches the number of sensors and hearts
// 	// make sure the number of outputs matches the number of joint motors

// 	// chance to grow a new layer
// 	// chance to lose a layer

// 	// chance to grow a new neuron
// 	// chance to lose a neuron

// 	// chance to grow a new connection
// 	// chance to lose a connection

// 	// chance to modify the weight of an existing connection .. s
// 	for (unsigned int i = 0; i < sumOfConnections; ++i)
// 	{
// 		if ( RNG() < mutationChance) {
// 			unsigned int l =  RNG() * newCake->n_layers ;
// 			unsigned int m =  RNG() * newCake->layers[l].n_neurons ;
// 			unsigned int n =  RNG() * newCake->layers[l].neurons[m].n_connections ;
// 			if (newCake->layers[l].neurons[m].n_connections > 8) {
// 				continue;
// 			}
// 			printf("max l:%u m:%u n:%u", newCake->n_layers ,  newCake->layers[l].n_neurons, newCake->layers[l].neurons[m].n_connections );
// 			float mutationAmount = ((RNG() -0.5) *mutationSeverity  );

// 			if (l > newCake->n_layers) {
// 				continue;
// 			}
// 			if (m > newCake->layers[l].n_neurons) {
// 				continue;
// 			}
// 			if (n > newCake->layers[l].neurons[m].n_connections) {
// 				continue;
// 			}
// 			newCake->layers[l].neurons[m].connections[n].connectionWeight += mutationAmount;
// 			printf("warped layer %u neuron %u connection %u by %f\n", l, m, n,  mutationAmount);
// 		}
// 	}
// 	// chance to change the target of an existing connection
// }

void mutateFishDescriptor (fishDescriptor_t * fish, float mutationChance, float mutationSeverity) {

	// mutate heart rate
	// if (RNG() > mutationChance) {	fish->heartSpeed += fish->heartSpeed * mutationSeverity *(RNG()-0.5); }

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

			// mutate attachment points
			// if (RNG() > mutationChance) {	fish->bones[i].attachedTo = (RNG() * fish->n_bones_used ) }

			// mutate bools
			// if (RNG() < mutationChance) {	fish->bones[i].isMouth = !fish->bones[i].isMouth; }
			// if (RNG() < mutationChance) {	fish->bones[i].sensor_touch = !fish->bones[i].sensor_touch; }
		}
	}
}

// void LoadFishFromName (uint8_t fishIndex) {
// 	fishDescriptor_t newFish;
// 	std::string fileName = "225";

// 	loadFishFromFile(fileName + std::string(".fsh"), newFish);
// 	mutateFishDescriptor (&newFish, 0.1, 0.1) ;

// 	fann *wann = loadFishBrainFromFile (fileName) ;	
// 	// networkDescriptor * tamberlina = createNeurodescriptorFromFANN(wann);
// 	mutateFishBrain(tamberlina, 0.1f, 1.0f );
// 	createFANNFileFromDescriptor(*tamberlina);
// 	fann * ann  = loadFishBrainFromFile ("mouptut"); // load the mutated file

// 	bool loadWithBlankBrain = true;
// 	if (loadWithBlankBrain) {
// 		loadFish ( fishIndex,  newFish, NULL, b2Vec2(0.0f, 0.0f) ) ;
// 	}
// 	else {
// 		loadFish ( fishIndex,  newFish, ann , b2Vec2(0.0f, 0.0f)) ;
// 	}
// }

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
	for (int i = 0; i < N_FISHES; ++i) {
		if ( fishes[i] == NULL || fishes[i] == nullptr) { 	continue; }
		if (fishes[i]->flagDelete) {
			deleteFish (i) ;
		}
	}
}

// random new generation from the old winner. reset
void reloadTheSim  () {
	// label every fish as delete.
	for (int i = 0; i < N_FISHES; ++i) {
		if ( fishes[i] == NULL || fishes[i] == nullptr) { 	continue; }
		fishes[i]->flagDelete = true;
	}

	startNextGeneration = true;
}


//  prints the winner to file immediately.
void  vote (BonyFish * winner) {
	if ( !fishSlotLoaded[winner->slot]) { return; }
	else {
	printf("winner: %i\n", winner->slot);

	fann * wann = winner->ann;

	// save the winner to file with a new name.
	std::string nnfilename =  std::string("mostCurrentWinner.net");
    std::string fdescfilename =  std::string("mostCurrentWinner.fsh");
    std::ofstream file { nnfilename };
    saveFishToFile (fdescfilename, winner->genes);
    fann_save(  wann , nnfilename.c_str()); 

	}

	// label every fish as delete.
	// for (int i = 0; i < N_FISHES; ++i) {
	// 	if ( fishes[i] == NULL || fishes[i] == nullptr) { 	continue; }
	// 	fishes[i]->flagDelete = true;
	// }

	// startNextGeneration = true;
	reloadTheSim();	
}

// void printConnectionArrayForDebug (networkDescriptor * network) {
// 	printf(" printConnectionArrayForDebug: %i layers\n", network->n_layers);

// 	for (unsigned int i = 0; i < network->n_layers; ++i) 	{
// 		printf(" layer %i neurons: %i\n", i, network->layers[i].n_neurons);

//  		for (unsigned int j = 0; j < network->layers[i].n_neurons; ++j) {

//  			printf(" neuron %i connections: %i\n", j, network->layers[i].neurons[j].n_connections);
//  			for (unsigned int k = 0; k < network->layers[i].neurons[j].n_connections; ++k) {
//  				printf(" |%u|, ", network->layers[i].neurons[j].connections[k].connectedTo); // <- it is already fucked up here.
// 			}
// 		}
// 	}
// 	printf("\n");
// }

void mutateFANNFileDirectly (std::string filename) {


	std::ofstream outFile("mutantGimp.net");
	std::string line;

	std::ifstream inFile(filename);
	int count = 0;
	// int amountCount = 0;
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

						// printf("%s\n", sciNumber);
					    float val = std::stof(sciNumber); 

					    if (RNG() < 0.1) {		// chance of a mutation occurring

					    	if (val > 1 || val < -1) { // if it's a big number, apply the mutation as a fraction of htat number. else, apply a random amount in a small range. this is to prevent weights being stuck at very small numbers.
					    		val += ( (RNG() -0.5f) * val * 1);
					    	}	
					    	else {
					    		val += ( (RNG() -0.5f) * 0.5f ); // how much mutation to apply
					    	}
						    
					    }

				    	char sciNotationBuffer[27];// = "0.00000000000000000000e+00";
			 			my_print_scientific(sciNotationBuffer,val);
			 			outFile << sciNotationBuffer;
			 			outFile << ") ";

			 			// amountCount ++;

			 			// if (amountCount >= 248) {
			 			// 	return;
			 			// }

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

int getSciNumberLength (char c) {
	// you need to find out how long the number is. Some have - sign, some have 2 digits in front of the decimal place.
	int sciNumberLength = 0;

	while(1) {

		char fugnutz = *((&c) +sciNumberLength);

		// printf("%c ", fugnutz);

		if (fugnutz == ')') {
			break;
		}else {
			// printf("%i ", sciNumberLength);
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

	// printf("%s\n", sciNumber);
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

	// mutateFishDescriptor (&newFishBody, 0.1, 0.25);
	return offspring;

}

// create an offspring with a mix of traits.
void sexBetweenTwoMinds (std::string fileNameA, std::string fileNameB) {
	std::ofstream offspring("offspring.net");
	
	std::ifstream partnerA(fileNameA);
	std::string lineA;

	std::ifstream partnerB(fileNameB);
	std::string lineB;

	int count = 0;
	int amountCount = 0;
	while(getline(partnerA, lineA)){

		getline(partnerB, lineB);

		// int lineALength = lineA.length();
		// int lineBLength = lineB.length();

	 	if (count == 35) { // its that crazy line.
	 		char desireCharacter = '=';
	 		bool skipTheRest = false;
	 		bool skipTheRestB = false;
	 		for(char& c : lineA) {

	 			// printf("%c",c);

	 			if (c == '\n' || partnerA.eof()) {
	 				break;
	 			}

	 			// skip over the 27 scientific notation characters you copied.
			    if (!skipTheRest) {

			    	// if you're not skipping or writing down the character, forward the information into the new file.
	 				offspring << c;
		 			
		 			// if you find the ' ', you're ready to copy over the next 27 characters all in one go.
		 			// but you need to skip all the BS at the start of the line. So first we set desirecharacter to =, to skip everything up to the = sign.
		 			// then we change it back to ' ' for the rest of the line.
		 			if (c == desireCharacter) {
		 				desireCharacter = ' ';
		 				if (c == '=' || *((&c) +1) == '(' )  { // 
		 					continue;
		 				}

		 				

		 				// // you need to find out how long the number is. Some have - sign, some have 2 digits in front of the decimal place.
		 				// int sciNumberLength = 0;

		 				// while(1) {
		 				// 	if (*((&c) +sciNumberLength) == ')') {
		 				// 		break;
		 				// 	}else {
		 				// 		sciNumberLength++;
		 				// 	}
		 				// }

		 				// if (c == '\n' || partnerA.eof() || (amountCount + 25) > lineALength) { // sci number never less than25
			 			// 	break;
			 			// }

			 			// printf("amountCount: %i, lineALength: %i, lineBLength: %i\n", amountCount, lineALength, lineBLength);

			 			// int sciNumberLength = getSciNumberLength(c);

			 			//---------------------------------------
			 				int sciNumberLength = 0;
			 				bool gettingLength = false;

							while(gettingLength) {

								char fugnutz = lineA[amountCount + sciNumberLength]; //*((&c) +sciNumberLength); // <<this broken AF

								// printf("%c ", fugnutz);

								if (fugnutz == ')') {
									// printf("PFETHECHETCH");
									gettingLength = true;
								}else {
									// printf("%i ", sciNumberLength);
									sciNumberLength++;
								}
							}
						// -----------------------------------

			 			// if (c == '\n' || partnerA.eof() || (amountCount + sciNumberLength) > lineALength) {
			 			// 	// break;
			 			// }

		 				float partnerAValue =  getSciNumberFromFANNFile(c);
		 				float partnerBValue = 0.0f;
		 				float val = 0.0f;

		 				// printf("%c", c);
		 				// ok now scroll the other file forward and get the number out of it.

		 				for(char& d : lineB) {
						    if (!skipTheRestB) {
									offspring << d;
									if (c == desireCharacter) {
										desireCharacter = ' ';
										if (d == '=' || *((&d) +1) == '(' )  { // 
											continue;
										}
									partnerBValue =  getSciNumberFromFANNFile(c);
						 			skipTheRestB = true;
									}
							}
							else  {
								if (d == ' ') {
									skipTheRestB = false;
								}
							}
						}


						// roll for inheritance
						if (RNG() > 0.5) {
							val = partnerAValue;
						}
						else {
							val = partnerBValue;
						}


					    if (RNG() < 0.2) {		// chance of a mutation occurring
						    val += ( (RNG() -0.5f) * 0.5f ); // how much mutation to apply
					    }

				    	char sciNotationBuffer[27];// = "0.00000000000000000000e+00";
			 			my_print_scientific(sciNotationBuffer,val);
			 			offspring << sciNotationBuffer;
			 			offspring << ") ";

			 			// expected number of connections in the file. This was only true for a certain version and should be removed.
			 			// amountCount ++;
			 			// if (amountCount >= 248) {
			 			// 	return;
			 			// }

			 			skipTheRest = true;
		 			}
		 			
				}
				else  {

					if (c == ' ') {
						skipTheRest = false;
					}
				}
				amountCount ++;
			}
	 	}
	 	else {
	 		// if its not a target line, just copy the line directly into the file without editing it, and replace the newline that got stripped out.
	 		offspring << lineA;
	        offspring << "\n";
	 	}
	    count++;
	}
	partnerA.close();
	partnerB.close();
	offspring.close();

return;
}

void drawingTest() {
	/* the following code actually works for drawing.
	b2Color testColor = b2Color(0.85f,0.21f,0.11f);

	b2Vec2 vertexA = b2Vec2(0.0f,0.0f);
	b2Vec2 vertexB = b2Vec2(1.0f,0.0f);
	b2Vec2 vertexC = b2Vec2(1.0f,1.0f);
	b2Vec2 vertexD = b2Vec2(0.0f,1.0f);



	b2Vec2 vertices[4] = {vertexA, vertexB, vertexC, vertexD};

	local_debugDraw_pointer->DrawFlatPolygon(vertices, 4 , testColor);
	*/


	// local_m_world->DrawParticleSystem(local_m_particleSystem);

	// draw the particle system
	int32 particleCount = local_m_particleSystem->GetParticleCount();
	if (particleCount)
	{
		float32 radius = local_m_particleSystem->GetRadius();
		const b2Vec2* positionBuffer = local_m_particleSystem->GetPositionBuffer();
		// if (local_m_particleSystem->m_colorBuffer.data)
		// {
		// 	const b2ParticleColor* colorBuffer = local_m_particleSystem->GetColorBuffer();
		// 	local_debugDraw_pointer->DrawParticles(positionBuffer, radius, colorBuffer, particleCount);
		// }
		// else
		// {
			local_debugDraw_pointer->DrawParticles(positionBuffer, radius, NULL, particleCount);
		// }
	}


	// draw the food particles

	b2Color fishFoodDye 		= b2Color(0.2f, 0.6f, 0.1f);
	b2Color fishFoodDyeOutline 	= b2Color(0.5f, 0.9f, 0.4f);

	for (int i = 0; i < N_FOODPARTICLES; ++i) {
		if (!foodSlotLoaded[i] || !food[i]->init || !food[i]->isUsed) {
			continue;
		}
		else {
			// food[i]->position 
			// food[i]->p_body->GetAngle()

			b2Vec2 vertices[4];


					// b2Vec2 boneCenterWorldPosition = ;
					for (int j = 0; j < 4; ++j) {	
						b2Vec2 adjustedVertex = food[i]->shape.GetVertex(j);//vertices[j];
						b2Vec2 boneLocalCenter =food[i]->p_body->GetLocalCenter();
						b2Vec2 rotatedVertex = rotatePoint( boneLocalCenter.x,boneLocalCenter.y, food[i]->p_body->GetAngle(), adjustedVertex);
						rotatedVertex.x += food[i]->position.x;
						rotatedVertex.y +=food[i]->position.y;
						vertices[j] = rotatedVertex;
					}
					local_debugDraw_pointer->DrawFlatPolygon(vertices, 4 ,fishFoodDye );
					local_debugDraw_pointer->DrawPolygon(vertices, 4 ,fishFoodDyeOutline );
					// local_debugDraw_pointer->DrawPolygon(vertices, 4 , fishFoodDye);


		}
	}

	// for each bone, get the vertices, then add the body's world location to them, and rotate by the body angle around the body world location.
	// to do this you will need to extend the rotate point method to rotate a polygon.
	for (unsigned int fishIndex = 0; fishIndex < N_FISHES; ++fishIndex) {
		if (fishSlotLoaded[fishIndex]) {
			for (int i = 0; i < N_FINGERS; ++i) {
				if (!fishes[fishIndex]->bones[i]->init || !fishes[fishIndex]->bones[i]->isUsed) {
					;
				}
				else {
					if (fishes[fishIndex]->bones[i]->p_body == NULL || fishes[fishIndex]->bones[i]->p_body == nullptr) {
						continue;
					}
					b2Vec2 vertices[4];
					b2Vec2 boneCenterWorldPosition = fishes[fishIndex]->bones[i]->p_body->GetWorldCenter();
					for (int j = 0; j < 4; ++j) {	
						b2Vec2 adjustedVertex = fishes[fishIndex]->bones[i]->shape.GetVertex(j);
						b2Vec2 boneLocalCenter =fishes[fishIndex]->bones[i]->p_body->GetLocalCenter();
						b2Vec2 rotatedVertex = rotatePoint( boneLocalCenter.x,boneLocalCenter.y, fishes[fishIndex]->bones[i]->p_body->GetAngle(), adjustedVertex);
						rotatedVertex.x += boneCenterWorldPosition.x;
						rotatedVertex.y +=boneCenterWorldPosition.y;
						vertices[j] = rotatedVertex;
					}

					// printf("eafe: %f\n", fishes[fishIndex]->bones[i]->outlineColor.r);

					local_debugDraw_pointer->DrawFlatPolygon(vertices, 4 , fishes[fishIndex]->bones[i]->color);
					local_debugDraw_pointer->DrawPolygon(vertices, 4 , b2Color(0,0,0));

					if (fishes[fishIndex]->selected) {

						local_debugDraw_pointer->DrawPolygon(vertices, 4 , b2Color(1,1,1));
					}
				}
			}
		}
	}
}


void verifyNetworkDescriptor (networkDescriptor * network) {
	// void printConnectionArrayForDebug (networkDescriptor * network) {
	printf(" printConnectionArrayForDebug: %i layers\n", network->n_layers);

	std::list<layerDescriptor>::iterator layer;
	unsigned int layerIndex = 0;
	unsigned int neuronIndex = 0;
	unsigned int connectionIndex = 0;

	for (layer = network->layers.begin(); layer !=  network->layers.end(); ++layer) 	{
		printf("	layer %u neurons: %lu\n", layerIndex, (unsigned long)layer->neurons.size());

		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 			printf("		neuron %u connections: %lu inputs: %u bias: %i\n", neuronIndex, (unsigned long)neuron->connections.size(), neuron->n_inputs, neuron->biasNeuron);

//  			printf(" neuron %i connections: %i\n", j, network->layers[i].neurons[j].n_connections);
 			std::list<connectionDescriptor>::iterator connection;
 			// for (unsigned int k = 0; k < fishes[fishIndex]->brain.layers[i].neurons[j].n_connections; ++k) {
 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {
 				printf("			connection %u to: %u, weight:%f\n", connectionIndex, connection->connectedTo, connection->connectionWeight );

 				;
 				// printf(" |%u|, ", fishes[fishIndex]->brain.layers[i].neurons[j].connections[k].connectedTo); // <- it is already fucked up here.
				connectionIndex++;
			}
			neuronIndex ++;
		}
// 	printf("\n");
		layerIndex ++;
	}


	// return -1;
}



void beginGeneration ( ) { // select an animal as an evolutionary winner, passing its genes on to the next generation

	// bool romance = false;
	int n_selected  = 0;
	int partner1index = 0;
	int partner2index = 0;

	// count the number of selected fish to see if you can do a fish fucking.
	for (int i = 0; i < N_FISHES; ++i) {
		if (fishSlotLoaded[i]) {
			if (fishes[i]->selected) {
				if (partner1index == 0) {
					partner1index = i;
				}
				else {
					partner2index = i;
				}
				n_selected ++;
				fishes[i]->selected = false;
			}
		}
	}



	if (n_selected >= 2) {

		// save partners information to file
		 saveFishToFile(std::string("partner1.fsh"), fishes[partner1index]->genes);
		 saveFishToFile(std::string("partner2.fsh"), fishes[partner2index]->genes);
		 fann_save(  fishes[partner1index]->ann , std::string("partner1.net").c_str()); 
		 fann_save(  fishes[partner2index]->ann , std::string("partner2.net").c_str()); 

		 // empty the world
		removeDeletableFish();

		// create new generation
		for (int i = 0; i < N_FISHES; ++i) {

			printf("SEX THEYRE HAVING SEX LOOK THEYRE HAVING SEX");

			fishDescriptor_t newBabyFish = sexBetweenTwoBodies(fishes[partner1index]->genes, fishes[partner2index]->genes);

			sexBetweenTwoMinds(std::string("partner1.net"), std::string("partner1.net"));

			mutateFishDescriptor (&newBabyFish, 0.1, 0.25);

		    mutateFANNFileDirectly(std::string("offspring.net"));
// 
		    // b2Vec2 positionalRandomness = b2Vec2(  (RNG()-0.5) * 25, (RNG()-0.5) * 5.0f  );

		    fann *wowGeezBoye = loadFishBrainFromFile (std::string("mutantGimp")) ;
		    loadFish (i, newBabyFish, wowGeezBoye, getRandomPosition()) ;

			totalFishIncorporator(i);	// spawn them into the world to repeat the cycle
		}

	}
	else {


			printf("LOOK HES WANKING HE WANKING OFF LOOK");
			// otherwise just reproduce 1 of them asexually

			// clear out the old generation
			removeDeletableFish();
			
		
			for (int i = 0; i < N_FISHES; ++i) {
				
				bool thereIsAFile = false;
				// b2Vec2 positionalRandomness = b2Vec2(  (RNG()-0.5) * 25, (RNG()-0.5) * 5.0f  );

				if (FILE *file = fopen("mostCurrentWinner.net", "r")) { //if (FILE *file = fopen(name.c_str(), "r")) {
			        fclose(file);
			        if (FILE *file = fopen("mostCurrentWinner.fsh", "r")) {
				        thereIsAFile = true;
				        fclose(file);
				    }
			    } 

				if (thereIsAFile ) { // if there is a previous winner, load many of its mutant children

					fishDescriptor_t newFishBody;
					loadFishFromFile(std::string("mostCurrentWinner.fsh"), newFishBody);

					// if (true) { // freeze body


						mutateFishDescriptor (&newFishBody, 0.0f, 0.0f);
						// mutateFishDescriptor (&newFishBody, 0.1, 0.5);
					// }
				    mutateFANNFileDirectly(std::string("mostCurrentWinner.net"));

					// now you can load the mutant ANN.
					fann *mann = loadFishBrainFromFile (std::string("mutantGimp")) ;


					// bool thereIsAMutant = false;
					// b2Vec2 positionalRandomness = b2Vec2(  (RNG()-0.5) * 25, (RNG()-0.5) * 5.0f  );

					// fann * mann;					
				 //        if (FILE *file = fopen("mutantGimp.net", "r")) {
					//         // thereIsAMutant = true;
					//         fclose(file);

							// mann = loadFishBrainFromFile (std::string("mutantGimp")) ;
					    // }
					    // else {
					    // 	mann = loadFishBrainFromFile (std::string("mostCurrentWinner"));
					    // }
				    


					// fann *dann = loadFishBrainFromFile (std::string("mostCurrentWinner")) ;


					// create a neurodescriptor.
					networkDescriptor * muscleCars=  createNeurodescriptorFromFANN (mann) ;

					verifyNetworkDescriptor(muscleCars);

					// now turn it back into a file and you can see if theyre the same.
					// createFANNFileFromDescriptor(muscleCars);

					fann * jann = createFANNbrainFromDescriptor(muscleCars);

					// unused_variable((void*) jann);
					for (int i = 0; i < 32; ++i)
					{
						if (newFishBody.inputMatrix[i].sensorType == SENSOR_TIMER) {
								newFishBody.inputMatrix[i].timerPhase = RNG();
						}
					}

					loadFish (i, newFishBody, jann, getRandomPosition()) ;

				}
				else { 						// if there is no winner, its probably a reset or new install. make one up

					printf("No genetic material found in game folder. Loading default animal.\n");
					fishDescriptor_t nematode = fishDescriptor_t();

			printf("look at his little dick");

					loadFish (i, nematode, NULL,  getRandomPosition()) ;
				}



				totalFishIncorporator(i);	// spawn them into the world to repeat the cycle

				// moveAWholeFish(i, positionalRandomness);
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

void deepSeaSetup (b2World * m_world, b2ParticleSystem * m_particleSystem, DebugDraw * p_debugDraw) { //, b2World * m_world_sci, b2ParticleSystem * m_particleSystem_sci) {

	// store the debugdraw pointer in here so we can use it.
	local_debugDraw_pointer = p_debugDraw;
	local_m_world = m_world;
	local_m_particleSystem = m_particleSystem;

	// local_m_world_sci = m_world_sci;
	// local_m_particleSystem_sci = m_particleSystem_sci;

	if (RNG() > 0.5f) {
		addFoodParticle(b2Vec2(24.0f, 3.5f));
	}
	else {
		addFoodParticle(b2Vec2(-24.0f, 3.5f));
	}
}

// this gets recalculated every round.
// void clearTouchSensations () {

// 	for (int i = 0; i < N_FISHES; ++i)
// 	{

// 		if (fishSlotLoaded[i]) {


// 			for (int j = 0; j < N_FINGERS; ++j)
// 		{
// 			if (fishes[i].bones[j].isUsed){



// 					fishes[i].bones[j]->touchSensation = 0.0f
// 			}
// 		}


// 		}

		
// 	}

// }

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


void meltSelectedFish () {
	for (int i = 0; i < N_FISHES; ++i) {
		if (fishes[i]->selected) {
			brainMelter (fishes[i]);


			// refresh the water in the brain jar.
			fishes[i]->ann = createFANNbrainFromDescriptor(fishes[i]->brain);

			return;
		}
		else {
			continue;
		}
	}
}

void modifyAConnection (BonyFish * fish, float amount) {
	// getNeuronByIndex( fish->brain, from)

		unsigned int selectedA;
	unsigned int selectedB;

	bool gotA = false;
	bool gotB=  false;

	// int layerIndex = 0;
	// int neuronIndex = 0;

	std::list<layerDescriptor>::iterator layer;
	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{
		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 		// 	std::list<connectionDescriptor>::iterator connection;
 		// 	for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {
 		// 		;
			// }

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
			// neuronIndex++;
		}
		// layerIndex++;
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
	for (int i = 0; i < N_FISHES; ++i) {
		if (fishes[i]->selected) {
			modifyAConnection (fishes[i],amount);
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
	// unsigned int n_layers = (unsigned long)fish->brain->layers.size();

	// sensorium size is based on the size of the ANN. Whether or not it is populated with numbers depends on the size of the input connector matrix.
			unsigned long sizeOfInputLayer = 0;
			unsigned long sizeOfOutputLayer = 0;
			unsigned long num_layers =  (unsigned long)(fish->brain->layers.size());

			std::list<layerDescriptor>::iterator layer;
			layer = fish->brain->layers.begin();
			sizeOfInputLayer = layer->neurons.size();
			std::advance(layer, num_layers-1);
			sizeOfOutputLayer = layer->neurons.size();

	// get the size of biggest layer
	// std::list<layerDescriptor>::iterator layer;
	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{
		if ((unsigned long)layer->neurons.size() > sizeOfBiggestLayer) {
			sizeOfBiggestLayer = (unsigned long)layer->neurons.size();
		}
		// std::list<neuronDescriptor>::iterator neuron;
 	// 	for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 	// 		std::list<connectionDescriptor>::iterator connection;
 	// 		for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {
 	// 			;
		// 	}
		// }
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
	// std::list<layerDescriptor>::iterator layer;
	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{

		neuronIndex = 0;

		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
 			

 			neuron->position = b2Vec2(drawingStartingPosition.x + (neuronIndex * spacingDistance), drawingStartingPosition.y + (layerIndex * spacingDistance));

 			// printf("dacfae");
 			// printab2Vec2(neuron->position);
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



				// draw color fill around input connector types
 			// if (neuron->selected){
 				b2Vec2 gigggle[] = {
				b2Vec2(neuron_position.x+0.1f, neuron_position.y-0.1f - 0.2f), 
				b2Vec2(neuron_position.x+0.1f, neuron_position.y+0.1f - 0.2f), 
				b2Vec2(neuron_position.x-0.1f, neuron_position.y+0.1f - 0.2f), 
				b2Vec2(neuron_position.x-0.1f, neuron_position.y-0.1f - 0.2f), 
				};

				// printf("fascingalg:%i\n",fish->inputMatrix[j].connectedToLimb);


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

		// b2Vec2 mocesfef = b2Vec2(neuron_position.x, neuron_position.y-0.2);
		// local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
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

		// b2Vec2 mocesfef = b2Vec2(neuron_position.x, neuron_position.y-0.2);
		// local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
					break;
					case SENSOR_TIMER:	

						local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.5f,0.1f,0.05f) );
						connectorLabel =  "Timer";


		 mocesfef = b2Vec2(neuron_position.x-0.05, neuron_position.y-0.2);
		local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());

		// b2Vec2 mocesfef = b2Vec2(neuron_position.x, neuron_position.y-0.2);
		// local_debugDraw_pointer->DrawString(mocesfef, connectorLabel.c_str());
					break;
				}


		//b2Vec2

		// std::string 

	}

	// for (int i = 0; i < count; ++i)
	// {
	// 	/* code */
	// }

	for (unsigned int j = 0; j < sizeOfOutputLayer; ++j) {
		b2Vec2 neuron_position = b2Vec2(drawingStartingPosition.x +j * spacingDistance,(drawingStartingPosition.y + ((num_layers-1) * spacingDistance)));
		// local_debugDraw_pointer->DrawPoint(neuron_position, 8.0f, b2Color( motorSignals[j], motorSignals[j], motorSignals[j]));

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

				// printf("fascingalg:%f\n",motorSignals[j]);

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

	// std::list<layerDescriptor>::iterator layer;
	for (layer = fish->brain->layers.begin(); layer !=  fish->brain->layers.end(); ++layer) 	{
		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {




 					// draw around it to indicate if it is selected
 			if (neuron->selected){
 				b2Vec2 gigggle[] = {
				b2Vec2(neuron->position.x+0.05f, neuron->position.y-0.05f), 
				b2Vec2(neuron->position.x+0.05f, neuron->position.y+0.05f), 
				b2Vec2(neuron->position.x-0.05f, neuron->position.y+0.05f), 
				b2Vec2(neuron->position.x-0.05f, neuron->position.y-0.05f), 
				};
			local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.3f,0.3f,0.3f) );


 			}



 			// draw a square at the neuron
 		// 	b2Vec2 neuronSquareVerts[] = {
			// 	b2Vec2(neuron->position.x+0.01f, neuron->position.y-0.01f), 
			// 	b2Vec2(neuron->position.x+0.01f, neuron->position.y+0.01f), 
			// 	b2Vec2(neuron->position.x-0.01f, neuron->position.y+0.01f), 
			// 	b2Vec2(neuron->position.x-0.01f, neuron->position.y-0.01f), 
			// 	};
			// local_debugDraw_pointer->DrawFlatPolygon(neuronSquareVerts, 4 ,b2Color(1.0f,1.0f,1.0f) );






			

 			std::list<connectionDescriptor>::iterator connection;
 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {


		    	b2Color segmentColor = b2Color(connection->connectionWeight,connection->connectionWeight,connection->connectionWeight);

		    	if (connection->connectionWeight > 0) {
		    		segmentColor = b2Color(0.0f,0.0f,connection->connectionWeight);
		    	}
		    	else {
		    		// if (connection->connectionWeight > 0) {
		    		segmentColor = b2Color(abs(connection->connectionWeight),0.0f,0.0f);
		    	// }
		    	}


			    local_debugDraw_pointer->DrawSegment(neuron->position, (getNeuronByIndex(fish->brain, connection->connectedTo))->position,segmentColor );

 				;
			}
		}
	}



}



// void drawNeuralNetwork(struct 	fann 	*	ann	, float * motorSignals, float * sensorium, int index, unsigned int * spacesUsedSoFar, BonyFish * fish) {

// 	// get the number of layers. FANN_EXTERNAL unsigned int FANN_API fann_get_num_layers(	struct 	fann 	*	ann	)
// 	unsigned int n_layers = fann_get_num_layers(ann);

// 	unsigned int sizeOfBiggestLayer = 0;

// 	// get the number of neurons on each layer.
// 	unsigned int layerArray[n_layers];
// 	fann_get_layer_array(ann,layerArray);
 
//     struct fann_connection *con;   /* weight matrix */
//     unsigned int connum;           /* connections number */
//     size_t i;

// 	connum = fann_get_total_connections(ann);
//     if (connum == 0) {
//         fprintf(stderr, "Error: connections count is 0\n");
//     }

//     con = (fann_connection *)calloc(connum, sizeof(*con));
//     if (con == NULL) {
//         fprintf(stderr, "Error: unable to allocate memory\n");
//     }

//     /* Get weight matrix */
//     fann_get_connection_array(ann, con);

//     // int icompatiblespaces = spacesUsedSoFar
//     float fcompatiblespaces = *spacesUsedSoFar;
// 	b2Vec2 drawingStartingPosition = b2Vec2(  fcompatiblespaces + 1 ,4.0f);
// 	float spacingDistance = 0.5f;

// 	// compute the size of biggest layer so you know how far to move over all subsequent layers.
// 	for (uint8_t j = 0; j < n_layers; ++j)
// 	{
// 		if (layerArray[j] > sizeOfBiggestLayer) {
// 			sizeOfBiggestLayer = layerArray[j];
// 		}
// 	}
// 	*spacesUsedSoFar += sizeOfBiggestLayer;


// 	// make a black color window that the drawing sits in.
// 	b2Vec2 windowVertices[] = {
// 		b2Vec2(drawingStartingPosition.x -spacingDistance , drawingStartingPosition.y- spacingDistance), 
// 		b2Vec2(drawingStartingPosition.x - spacingDistance, drawingStartingPosition.y + ((n_layers *spacingDistance ) + ( spacingDistance) ) ), 
// 		b2Vec2(drawingStartingPosition.x + ((sizeOfBiggestLayer *spacingDistance ) + (spacingDistance) ), drawingStartingPosition.y+ ((n_layers *spacingDistance ) + (spacingDistance) )), 
// 		b2Vec2(drawingStartingPosition.x + ((sizeOfBiggestLayer *spacingDistance ) + ( spacingDistance) ), drawingStartingPosition.y- spacingDistance)
// 	};

// 	fish->brain->networkWindow.lowerBound = b2Vec2(drawingStartingPosition.x -spacingDistance , drawingStartingPosition.y- spacingDistance);
// 	fish->brain->networkWindow.upperBound = b2Vec2(drawingStartingPosition.x + ((sizeOfBiggestLayer *spacingDistance ) + (spacingDistance) ), drawingStartingPosition.y+ ((n_layers *spacingDistance ) + (spacingDistance) ));


// 	local_debugDraw_pointer->DrawFlatPolygon(windowVertices, 4 ,b2Color(0.1,0.1,0.1) );

// /*

// 	std::list<layerDescriptor>::iterator layer;
// 	for (layer = fishes[fishIndex]->brain->layers.begin(); layer !=  fishes[fishIndex]->brain->layers.end(); ++layer) 	{
// 		std::list<neuronDescriptor>::iterator neuron;
//  		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {
//  			std::list<connectionDescriptor>::iterator connection;
//  			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {
//  				;
// 			}
// 		}
// 	}

// */

// 	for (uint8_t j = 0; j < layerArray[0]; ++j) {
// 		b2Vec2 neuron_position = b2Vec2(drawingStartingPosition.x +j * spacingDistance,drawingStartingPosition.y );
// 		local_debugDraw_pointer->DrawPoint(neuron_position, 8.0f, b2Color( sensorium[j], sensorium[j], sensorium[j]));
// 	}

// 	for (uint8_t j = 0; j < layerArray[n_layers-1]; ++j) {
// 		b2Vec2 neuron_position = b2Vec2(drawingStartingPosition.x +j * spacingDistance,(drawingStartingPosition.y + ((n_layers-1) * spacingDistance)));
// 		local_debugDraw_pointer->DrawPoint(neuron_position, 8.0f, b2Color( motorSignals[j]+0.5f, motorSignals[j]+0.5f, motorSignals[j]+0.5f));
// 	}

//     /* Print weight matrix */
//     for (i = 0; i < connum; ++i) {
//     	b2Vec2 printConnectionSideA;
//     	b2Vec2 printConnectionSideB;

//     	uint8_t neuronA_tally = con[i].from_neuron;
//     	for (uint8_t j = 0; j< n_layers; ++j) {
//     		if (neuronA_tally >= layerArray[j]) {
//     			neuronA_tally -= layerArray[j];
//     		}	
//     		else {
//     			printConnectionSideA = b2Vec2(drawingStartingPosition.x + neuronA_tally * spacingDistance, drawingStartingPosition.y +j * spacingDistance);
//     			b2AABB neuronBox;
//     			neuronBox.upperBound = b2Vec2(printConnectionSideA.x+0.01f,printConnectionSideA.y+0.01f );
//     			neuronBox.lowerBound = b2Vec2(printConnectionSideA.x-0.01f,printConnectionSideA.y-0.01f );
//     			getNeuronByIndex(fish->brain, con[i].from_neuron)->aabb = neuronBox;
//     			break;
//     		}
//     		neuronA_tally --;
//     	}
//     	uint8_t neuronB_tally = con[i].to_neuron;
//     	for (uint8_t j = 0; j< n_layers; ++j) {
//     		if (neuronB_tally >= layerArray[j]) {
//     			neuronB_tally -= layerArray[j];
//     		}	
//     		else {
//     			printConnectionSideB = b2Vec2(drawingStartingPosition.x + neuronB_tally * spacingDistance, drawingStartingPosition.y +j * spacingDistance);
//     			b2AABB neuronBox;
//     			neuronBox.upperBound = b2Vec2(printConnectionSideB.x+0.01f,printConnectionSideB.y+0.01f );
//     			neuronBox.lowerBound = b2Vec2(printConnectionSideB.x-0.01f,printConnectionSideB.y-0.01f );
//     			getNeuronByIndex(fish->brain, con[i].to_neuron)->aabb = neuronBox;
//     			// printf("mokkneyk: %i fussy: %i\n", getNeuronByIndex(fish->brain, con[i].to_neuron)->index, con[i].to_neuron);
//     			break;
//     		}
//     		neuronB_tally --;
//     	}

//     	b2Color segmentColor = b2Color(con[i].weight*10,con[i].weight*10,con[i].weight*10);
// 	    local_debugDraw_pointer->DrawSegment(printConnectionSideA, printConnectionSideB,segmentColor );
//     }
//     free(con);
// }

// // void enterScienceModeInterruptableEntry () {
// // 	printf("enterScienceModeInterruptableEntry\n");

// // 	for (int i = 0; i < N_FISHES; ++i) {
// // 		if ( fishes[i] == NULL || fishes[i] == nullptr) { 	continue; }
// // 		fishes[i]->flagDelete = true;
// // 	}

// // 	readyToEnterScienceMode = true;
// // }


// // void enterScienceMode ( ) {


// // 	printf("enterScienceMode\n");
// // 	removeDeletableFish();


// // 				// addFoodParticle(getRandomPosition());
	
		
// // 		scienceMode = true;
// // 	// for (int i = 0; i < N_FISHES; ++i) {
		
// // 		bool thereIsAFile = false;
// // 		// b2Vec2 positionalRandomness = b2Vec2(  (RNG()-0.5) * 25, (RNG()-0.5) * 5.0f  );

// // 		if (FILE *file = fopen("mostCurrentWinner.net", "r")) { //if (FILE *file = fopen(name.c_str(), "r")) {
// // 	        fclose(file);
// // 	        if (FILE *file = fopen("mostCurrentWinner.fsh", "r")) {
// // 		        thereIsAFile = true;
// // 		        fclose(file);
// // 		    }
// // 	    } 

// // 		if (thereIsAFile ) { // if there is a previous winner, load many of its mutant children
// // 			fishDescriptor_t newFishBody;
// // 			loadFishFromFile(std::string("mostCurrentWinner.fsh"), newFishBody);

// // 			mutateFishDescriptor (&newFishBody, 0.1, 0.25);
// // 		    mutateFANNFileDirectly(std::string("mostCurrentWinner.net"));

// // 			// now you can load the mutant ANN.
// // 			// fann *mann = loadFishBrainFromFile (std::string("mutantGimp")) ;
// // 			fann *mann = loadFishBrainFromFile (std::string("mutantGimp")) ;

// // 		    b2Vec2 positionalRandomness = b2Vec2(  (RNG()-0.5) * 25, (RNG()-0.5) * 5.0f  );

// // 			loadFish (0, newFishBody, mann, positionalRandomness) ;
			

// // 		}
// // 		// else { 						// if there is no winner, its probably a reset or new install. make one up
// // 		// 	prepareNematode(&nematode);

// // 		// 	loadFish (i, nematode, NULL,  positionalRandomness) ;
// // 		// }



	
// // 		// moveAWholeFish(i, positionalRandomness);
// // 	// }
// // 	// startNextGeneration = false;


// // }


// // // void exitScienceMode () {
// // // 	scienceMode = false;
// // // }

// // // bool queryScienceMode () {
// // // 	return scienceMode;
// // // }


// // // void scienceModeGraphics () {
// // // 		unsigned int spacesUsedSoFar =0;


// // 		// print the brainal output
// // 		drawNeuralNetwork( fishes[i]->ann, motorSignals, sensorium, i, &spacesUsedSoFar);


// // }


// // the science menu allows you to study and modify individual organisms.
// // a single copy of the selected organism is pinned to the center of the screen
// // void scienceModeLoop () {
// // if (!local_m_world->IsLocked() ) {


// // for  (int i = 0; i < 1; i++) {
// // 			if (!foodSlotLoaded[i]) {
// // 				break;
// // 			}
// // 			else {
// // 				food[i]->position = food[i]->p_body->GetWorldCenter(); // update positions of all the food particles
// // 			}
// // 		}

// // // the user modifies the descriptor

// // // the new descriptor is created into an organism

// // // the organism is shown in the view and in the world



// // }
// // }

// return a positive number if finds a box or negative if not.
int checkNeuroWindow (b2AABB mousePointer) {
	// check to see if you're in a neuro window.
	for (int i = 0; i < N_FISHES; ++i)
	{
		if (fishChecker(i)) {



			if (fishes[i]->brain->networkWindow.Contains(mousePointer)) {
			// printf("melected: %i\n", i);
				return i;
		}
		}
		

	}
	return -1;
}


int checkNeuronsInWindow (b2AABB mousePointer, int fishIndex) {
	// void printConnectionArrayForDebug (networkDescriptor * network) {
	// printf("checkNeuronsInWindow \n");


	// printf("mouz: %f\n", );

	std::list<layerDescriptor>::iterator layer;
	for (layer = fishes[fishIndex]->brain->layers.begin(); layer !=  fishes[fishIndex]->brain->layers.end(); ++layer) 	{
// 		printf(" layer %i neurons: %i\n", i, network->layers[i].n_neurons);

		std::list<neuronDescriptor>::iterator neuron;
 		for ( neuron = layer->neurons.begin(); neuron != layer->neurons.end() ; neuron++) {

//  			printf(" neuron %i connections: %i\n", j, network->layers[i].neurons[j].n_connections);
 			// std::list<connectionDescriptor>::iterator connection;
 			// for (unsigned int k = 0; k < fishes[fishIndex]->brain.layers[i].neurons[j].n_connections; ++k) {
 			// for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {



	// printf("mouz: %f\n", );

	// printf("mouz: %f\n", mousePointer.upperBound.x);

	// printf("faef: %f\n",neuron->position.x);
 				// check neuron
 				// if (mousePointer.Contains(neuron->aabb)) {
 				

 					if (neuron->position.x < mousePointer.upperBound.x && neuron->position.x > mousePointer.lowerBound.x) {
 						if (neuron->position.y < mousePointer.upperBound.y && neuron->position.y > mousePointer.lowerBound.y) {
 								printf("you clicked a neuron: %i\n", neuron->index);
 							neuron->selected = !(neuron->selected);
 							return neuron->index;




 						}
 					}



 				// }

 				
 				// printf(" |%u|, ", fishes[fishIndex]->brain.layers[i].neurons[j].connections[k].connectedTo); // <- it is already fucked up here.
// 			}
			}
		// }
// 	printf("\n");
	}


	return -1;
}


void runBiomechanicalFunctions () {
	// printf("runBiomechanicpalFunctions\n");
	unsigned int spacesUsedSoFar =0;

	for (int i = 0; i < N_FISHES; ++i) {
		if (fishSlotLoaded[i]) {

			if (fishChecker(i)) {
				// reset neuro bounding boxes... this isnt the right place to do this, should be in a graphics function
				fishes[i]->brain->networkWindow.lowerBound = b2Vec2(0.0f,0.0f);
				fishes[i]->brain->networkWindow.upperBound = b2Vec2(0.0f,0.0f);
			}

			// update the fish's senses
			for (int j = 0; j < N_FINGERS; ++j) {
				nonRecursiveSensorUpdater (fishes[i]->bones[j]);
			}
		
			// sensorium size is based on the size of the ANN. Whether or not it is populated with numbers depends on the size of the input connector matrix.
			unsigned long sizeOfInputLayer = 0;
			unsigned long sizeOfOutputLayer = 0;
			unsigned long num_layers =  (unsigned long)(fishes[i]->brain->layers.size());

			std::list<layerDescriptor>::iterator layer;
			layer = fishes[i]->brain->layers.begin();
			sizeOfInputLayer = layer->neurons.size();
			std::advance(layer, num_layers-1);
			sizeOfOutputLayer = layer->neurons.size();

			float sensorium[ sizeOfInputLayer ];
			// float motorsignals[sizeOfOutputLayer ];

			// unsigned int senseInputsUsedSoFar = 0;

			for (unsigned int j = 0; j < sizeOfInputLayer; ++j)
			{
				if (j >= 32) {
					continue;	// if the sensorium array is bigger than the array of input connectors, you can just leave the rest blank.
				}

				switch (fishes[i]->inputMatrix[j].sensorType) {
					case SENSOR_FOODRADAR:
						sensorium[j] = fishes[i]->bones[ fishes[i]->inputMatrix[j].connectedToLimb  ]->sensation_radar;
					break;

					case SENSOR_TOUCH:
						sensorium[j] = fishes[i]->bones[ fishes[i]->inputMatrix[j].connectedToLimb  ]->sensation_touch;
					break;

					case SENSOR_JOINTANGLE:
						sensorium[j] = fishes[i]->bones[ fishes[i]->inputMatrix[j].connectedToLimb  ]->sensation_jointangle;
					break;

					case SENSOR_TIMER:
						// sensorium[j] = fishes[i]->bones[ fishes[i]->inputMatrix[j].connectedToLimb  ]->0.0;

						sensorium[j] = sin(2 * pi * fishes[i]->inputMatrix[j].timerFreq * 	fishes[i]->inputMatrix[j].timerPhase);


						// printf("mekshun: %f\n", sensorium[j]);
						fishes[i]->inputMatrix[j].timerPhase += 0.001;
						if (fishes[i]->inputMatrix[j].timerPhase > 1) {
							fishes[i]->inputMatrix[j].timerPhase = 0;
						}
 
					break;

					case SENSECONNECTOR_UNUSED:
						// sensorium[j] = fishes[i]->bones[ fishes[i]->inputMatrix[j].connectedToLimb  ]->0.0;
						sensorium[j] = 0.0f;
					break;
				 
				}	
				// senseInputsUsedSoFar ++;				
			}

			// feed information into brain
			float * motorSignals = fann_run(fishes[i]->ann, sensorium);
			// for (int j = 0; j < 32; ++j)
			// {
			// 	printf("smegggg: %f\n", motorSignals[j]);
			// }


			for (unsigned int j = 0; j < sizeOfOutputLayer; ++j)
			{
				if (j >= 32) {
					continue;	// if the sensorium array is bigger than the array of input connectors, you can just leave the rest blank.
				}

				switch (fishes[i]->outputMatrix[j].sensorType) {
					case SENSECONNECTOR_MOTOR:
	
						// first just check to make sure the bone and joint is good.
						// this paragraph could be condensed very easily
						if ( !fishes[i]->bones[fishes[i]->outputMatrix[j].connectedToLimb]->isUsed || !fishes[i]->bones[fishes[i]->outputMatrix[j].connectedToLimb]->init) {
							continue;
						}
						else {
							if (fishes[i]->bones[fishes[i]->outputMatrix[j].connectedToLimb]->isRoot) {
								continue;
							}
							if (fishes[i]->bones[fishes[i]->outputMatrix[j].connectedToLimb]->joint->isUsed && fishes[i]->bones[fishes[i]->outputMatrix[j].connectedToLimb]->joint->init) {
							if (fishes[i]->bones[fishes[i]->outputMatrix[j].connectedToLimb]->joint->p_joint != nullptr) {




								float motorSpeed = motorSignals[j] * 10;

								if (false) {


								// clip the possible motor speed to the speed limit.
								float speedLimit = fishes[i]->bones[fishes[i]->outputMatrix[j].connectedToLimb]->joint->speedLimit;
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


								// printf("moraotof: %f\n", motorSpeed);
								// if (motorSpeed > fishes[i]->bones[fishes[i]->outputMatrix[j].connectedToLimb]->joint->speedLimit) {
								// 	motorSpeed = fishes[i]->bones[fishes[i]->outputMatrix[j].connectedToLimb]->joint->speedLimit;
								// }
								// if (motorSpeed < -abs(fishes[i]->bones[fishes[i]->outputMatrix[j].connectedToLimb]->joint->speedLimit)) {
								// 	motorSpeed = -fishes[i]->bones[fishes[i]->outputMatrix[j].connectedToLimb]->joint->speedLimit;
								// }
								fishes[i]->bones[fishes[i]->outputMatrix[j].connectedToLimb]->joint->p_joint->SetMotorSpeed(motorSpeed);
							}
						}
						}	
					break;
				
				}
			}


			if (fishChecker(i)) {
				if (fishes[i]->selected) {
					// print the brainal output
					// drawNeuralNetwork( fishes[i]->ann, motorSignals, sensorium, i, &spacesUsedSoFar, fishes[i]);

					drawNeuralNetworkFromDescriptor(motorSignals, sensorium, &spacesUsedSoFar, fishes[i]);
					//float * motorSignals, float * sensorium, unsigned int * spacesUsedSoFar, BonyFish * fish


				}
			}
		}
	}
}




// void regularModeGraphics () {

// 		for (int i = 0; i < N_FISHES; ++i)
// 		{
// 			/* code */
		

// 			}
// 		}
		
// }


void deepSeaLoop () {



	if (!local_m_world->IsLocked() ) {

		// drawingTest(0);

		// if (readyToEnterScienceMode) {
		// 	readyToEnterScienceMode = false;
		// 	enterScienceMode () ;
		// 	return;
		// }

		if (startNextGeneration ) {
			beginGeneration ( );
		}

		for  (int i = 0; i < N_FOODPARTICLES; i++) {
			if (!foodSlotLoaded[i]) {
				break;
			}
			else {
				food[i]->position = food[i]->p_body->GetWorldCenter(); // update positions of all the food particles
			}
		}

		runBiomechanicalFunctions();

		// regularModeGraphics();
	}
}

void deepSeaControlA () {
	;
}
void deepSeaControlB () {
	;
}


void collisionHandler (void * userDataA, void * userDataB, b2Contact * contact) {
	bool et = false;
	bool fud = false;

	// if (false) {
	// 	return;   // this function ends the program so nice to turn it off
	// }

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

	if( dataA.dataType == TYPE_TOUCHSENSOR ) {
		
		//normal manifold contains all info...
		// int numPoints = contact->GetManifold()->pointCount;

		//...world manifold is helpful for getting locations
		// b2WorldManifold worldManifold;
		// contact->GetWorldManifold( &worldManifold );

		// b2Vec2 vel1 = ((BoneUserData *)(dataB.uData))->p_body->GetLinearVelocityFromWorldPoint( worldManifold.points[0] );
		// b2Vec2 vel2 = ((BoneUserData *)(dataB.uData))->p_body->GetLinearVelocityFromWorldPoint( worldManifold.points[0] );
		// b2Vec2 impactVelocity = vel1 - vel2;

		((BoneUserData *)(dataA.uData))->sensation_touch +=0.5f;// magnitude(impactVelocity);
	}

	if( dataB.dataType == TYPE_TOUCHSENSOR ) {
		
		//normal manifold contains all info...
		// int numPoints = contact->GetManifold()->pointCount;

		//...world manifold is helpful for getting locations
		// b2WorldManifold worldManifold;
		// contact->GetWorldManifold( &worldManifold );

		// b2Vec2 vel1 = ((BoneUserData *)(dataB.uData))->p_body->GetLinearVelocityFromWorldPoint( worldManifold.points[0] );
		// b2Vec2 vel2 = ((BoneUserData *)(dataB.uData))->p_body->GetLinearVelocityFromWorldPoint( worldManifold.points[0] );
		// b2Vec2 impactVelocity = vel1 - vel2;

		((BoneUserData *)(dataB.uData))->sensation_touch +=0.5f;// magnitude(impactVelocity);
	}


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
		    vote(((BoneUserData *)(dataB.uData))->p_owner);
		}
		else if (dataA.dataType == TYPE_MOUTH) {
			vote(((BoneUserData *)(dataB.uData))->p_owner);
		}
	}
}