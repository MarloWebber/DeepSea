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
// #include <iostream>
#include <cmath>

// int currentNumberOfFood = 0;
int currentNumberOfFish = 0;
int generationsThisGame = 0;
bool startNextGeneration = false;
// bool readyToEnterScienceMode = false;

// // int selected = 0;
// bool scienceMode = false;

// bool fishSlotLoaded[N_FISHES];
// bool foodSlotLoaded[N_FOODPARTICLES];

// b2Vec2 star = b2Vec2(0.0f, 3.0f); // the emitter of radiant light. Just using a position for now but maybe more properties later.


bool userControlInputA;
bool userControlInputB;

deepSeaSettings m_deepSeaSettings;

// int m_deepSeaSettings.gameMode = 0;

std::list<BonyFish> fishes;
// std::list<foodParticle_t> food;

std::list<Lamp> lamps;

BoneUserData * food[N_FOODPARTICLES];

b2Body * theActualFuckingFuck;

b2World * local_m_world = nullptr;
b2ParticleSystem * local_m_particleSystem = nullptr;
DebugDraw * local_debugDraw_pointer = nullptr;

b2World * local_m_world_sci = nullptr;
b2ParticleSystem * local_m_particleSystem_sci = nullptr;
DebugDraw * local_debugDraw_pointer_sci = nullptr;

bool m_drawing;
b2ParticleGroup* m_lastGroup;
uint32 m_particleFlags;
uint32 m_groupFlags;
uint32 m_colorIndex;




uint32 DetermineParticleParameter() 
	{
		if (m_drawing)
		{
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

// DeepSeaSettings m_deepSeaSettings;

bool flagAddFood = false;
bool flagAddPlant= false;


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

// foodParticle_t * food[N_FOODPARTICLES];
// BonyFish * fishes[N_FISHES];
// BonyFish * sciFish;
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
	length = 0.5f;
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
		   return  b2Vec2(  (RNG()-0.5) * 25, (RNG()-0.5) * 5.0f  );
}

// add a food particle to the world and register it so the game knows it exists.
// foodParticle_t::foodParticle_t ( b2Vec2 position) {	
// 	energy = 1.0f;
// 	uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_FOOD);
// 	bodyDef.userData = (void*)p_dataWrapper;
// 	bodyDef.type = b2_dynamicBody;
// 	u_body = local_m_world->CreateBody(&bodyDef);
// 	// theActualFuckingFuck = u_body;

// 	flagDelete = false;

// 	b2Vec2 vertices[] = {
// 		b2Vec2(-0.25, -0.25), 
// 		b2Vec2(0.25, -0.25), 
// 		b2Vec2(0.25, 0.25), 
// 		b2Vec2(-0.25, 0.25)
// 	};
		
// 	shape.Set(vertices, 4);

// 	// u_body->CreateFixture(&shape, 1.2f);
// 	init = false;
// 	isUsed = false;
// };

BoneUserData::BoneUserData(
		boneAndJointDescriptor_t boneDescription,
		BonyFish * fish,
		b2Vec2 positionOffset,
		int newCollisionGroup,
		bool attached
	) {

	// if (!boneDescription.used) {
	// 	return;
	// }

	p_owner = fish;

	
	// unused_variable((void *)attachesTo);

	// else {
	// 	attachesTo = NULL;
	// }
	
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
		
		// if (isMouth) {
		// 	uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_MOUTH);
		// 	bodyDef.userData = (void *)p_dataWrapper;
		// }
		// else if (sensor_touch) {

		// 	uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_TOUCHSENSOR);
		// 	bodyDef.userData = (void *)p_dataWrapper;
		// 	}
		// // else if (!attached) {
		// // 	uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_FOOD);
		// // 	bodyDef.userData = (void *)p_dataWrapper;
		// // }
		// 	else if (isLeaf) {

		// 	uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_LEAF);
		// 	bodyDef.userData = (void *)p_dataWrapper;
		// 	}
		// 	else {

		// 		uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_DEFAULT);
		// 		bodyDef.userData = (void *)p_dataWrapper;
		// 	}

		bodyDef.type = b2_dynamicBody;
		p_body = local_m_world->CreateBody(&bodyDef);
		
		shape.Set(vertices, count);

		// reference the physics object from the user data.
		tipCenter = tipCenter;
		rootCenter = b2Vec2(0.0f, 0.0f);
	}
	else {
		// if (attached) {

		// else {
		// 	offsetOnBody = b2Vec2(0.0f, 0.0f);
		// }
		// tipCenter = b2Vec2(0.0f,  length);

		b2Vec2 vertices[] = {
			b2Vec2( + (rootThickness/2), -(length/2)),
			b2Vec2( - (rootThickness/2),  -(length/2)),
			b2Vec2( + tipThickness/2, +(length/2)),
			b2Vec2( - tipThickness/2, +(length/2))
		};

		// attach user data to the body
		// if (isMouth) {
		// 	uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_MOUTH);
		// 	bodyDef.userData = (void *)p_dataWrapper;
		// }
		// else if (sensor_touch) {

		// 	uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_TOUCHSENSOR);
		// 	bodyDef.userData = (void *)p_dataWrapper;
		// 	}
		// 	else if (isLeaf) {

		// 	uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_LEAF);
		// 	bodyDef.userData = (void *)p_dataWrapper;
		// 	}
		// 	else {

		// 		uDataWrap * p_dataWrapper = new uDataWrap(this, TYPE_DEFAULT);
		// 		bodyDef.userData = (void *)p_dataWrapper;
		// 	}

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

		printf("new bone with position offset x: %f, y%f\n", positionOffset.x, positionOffset.y);
		p_body->SetTransform(b2Vec2(positionOffset.x, positionOffset.y + offsetOnBody.y),0);
		

		shape.Set(vertices, count);

		// reference the physics object from the user data.

		// if (!isRoot) {
			
		// } else {
		// 	rootCenter = b2Vec2(0.0f, 0.0f);
		// }


		

	// if (!isRoot) {
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

	// p_bone->p_body->SetUserData


		// uDataWrap * p_dataWrapper = new uDataWrap( p_bone, ((uDataWrap*)p_bone->p_body->GetUserData())->dataType  );

		// p_bone->p_body->SetUserData((void *) p_dataWrapper); //userData->dataType = TYPE_FOOD;


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

	// std::list<foodParticle_t>::iterator foodParticle;
	// for (foodParticle = food.begin(); foodParticle !=  food.end(); ++foodParticle) 	{
			// if (
			// 	//!foodSlotLoaded[i]
			// 	!food[i]) {
			// 	continue;
			// }
			if (food[i]->init && food[i]->isUsed) {
				b2Vec2 boneCenterWorldPosition = p_bone->p_body->GetWorldCenter();
				b2Vec2 positionalDifference = b2Vec2((boneCenterWorldPosition.x - food[i]->position.x),(boneCenterWorldPosition.y - food[i]->position.y));
				float distance = magnitude (positionalDifference);
				if (distance > 0) {
					p_bone->sensation_radar += 1/distance;
				}
			}

			// marker

	

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
	printf("addFoodParticle\n");

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


		// all udatawrap additions being moved to recursiveBoneIncorporaotr
		// uDataWrap * p_dataWrapper = new uDataWrap( &(*food[emptyFoodIndex]), TYPE_FOOD);
		// food[emptyFoodIndex]->p_body->SetUserData((void *) p_dataWrapper); //userData->dataType = TYPE_FOOD;
		food[emptyFoodIndex]->isFood = true;

		food[emptyFoodIndex]->joint->init = false;
		food[emptyFoodIndex]->joint->isUsed = false;
		nonRecursiveBoneIncorporator(food[emptyFoodIndex]);

		food[emptyFoodIndex]->energy = food[emptyFoodIndex]->energy * 5; // this is a constant that sets the value of food. Typical creatures are made from 4 segments; setting this to 4 or above should allow the creature to reproduce after eating just 1 segment.


	// food[currentNumberOfFood] = foodParticle_t(position);
	 // food.push_back( foodParticle_t( position) );
	// foodSlotLoaded[currentNumberOfFood] = true;
	// currentNumberOfFood++;
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
	return new networkDescriptor(temp_ann);
}

void BonyFish::feed(float amount) {
	energy += amount;
}

BonyFish::BonyFish(fishDescriptor_t driedFish, fann * nann, b2Vec2 startingPosition) {
	genes = driedFish;
	// hunger = 0.0f; 								
	// energy = 10000.0f;		// the animal spends energy to move and must replenish it by eating

	reproductionEnergyCost = 0.0f; // the amount of energy required to make a clutch of viable offspring. to be calculated

	flagDelete = false;
	selected = false;

	int randomCollisionGroup = - (RNG() * 16.0f);

	for (int i = 0; i < N_FINGERS; ++i) {
		if (i == 0) {
			driedFish.bones[i].isRoot = true;
		}
		bones[i] = new BoneUserData(driedFish.bones[i], this, startingPosition, randomCollisionGroup, true);
		// bones[i]->joint = new JointUserData(); //(boneAndJointDescriptor_t boneDescription, BoneUserData * p_bone, BonyFish * fish)
		reproductionEnergyCost += bones[i]->energy;
	}

	// energy = reproductionEnergyCost - 1;
	energy = reproductionEnergyCost * 0.5;

	n_bones_used = 0;
	for (int i = 0; i < N_FINGERS; ++i) {
		if (driedFish.bones[i].used) {
			n_bones_used ++;

		}
	}

	printf("reproductionEnergyCost: %f\n", reproductionEnergyCost);

	init = false; 										// true after the particle has been initialized. In most cases, uninitalized particles will be ignored.
	isUsed = false; 									// only true when the part is added to the world







for (unsigned int i = 0; i < N_FINGERS; ++i)
	{
		// if the limb doesn't already have sense and motor connectors, add them in.

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

// bool fishChecker(unsigned int fishIndex) {
	// if ( fishes[fishIndex] == NULL || fishes[fishIndex] == nullptr) { 	return false; }
	// 	if (fishSlotLoaded[fishIndex] ) {
	// 		return true;
	// 	}
	// 	else {
	// 		return false;
	// 	}
// }

void moveAWholeFish (BonyFish * fish, b2Vec2 position) {
	// if ( fishes[fishIndex] == NULL || fishes[fishIndex] == nullptr) { 	return; }
	// if (fishSlotLoaded[fishIndex] ) {
		for (int i = 0; i < N_FINGERS; ++i)
		{
			if ( !fish->bones[i]->isUsed && !fish->bones[i]->init) {
					continue;
			}
			fish->bones[i]->p_body->SetTransform(position, 0.0f);
		}
	// }
}

// void totalFishIncorporator (uint8_t fishIndex) {
	
// }

// void deleteFood (foodParticle_t * snackBar) {

	// 	std::list<foodParticle_t>::iterator foodParticle;
	// for (foodParticle = food.begin(); foodParticle !=  food.end(); ++foodParticle) 	{

	// 	if ( &(*foodParticle) == snackBar ) {
	// 		break;
	// 	}

	// }



	// 	// currentNumberOfFood --;
	// 	foodParticle->flagDelete = false;
	// 	// snackBar->p_body = nullptr;

	// 	// foodSlotLoaded[foodIndex] = false;

	// // if (foodSlotLoaded[foodIndex]) {


	// // if (!snackBar->isUsed || !snackBar->init || snackBar == nullptr || snackBar == NULL) {
	// // 	return;
	// // }

		// printf("deleting food particle\n");





	// b2JointEdge * monogogo = foodParticle->u_body->GetJointList();

	// while (monogogo != NULL) {
	// 	b2JointEdge * mndaee = monogogo->next;
	// 		local_m_world->DestroyJoint(monogogo->joint);
	// 		monogogo = mndaee;
	// }

	// if (monogogo->joint != NULL || monogogo->joint != nullptr) {


// 	local_m_world->DestroyJoint(monogogo->joint);
// }

	// // try {
	// printf("DESROREYING BODYEHN 0x%lu\n", foodParticle->u_body);
	// printf("SNARBYFRK 0x%lu\n",snackBar->u_body);
	// 
	
	// local_m_world->DestroyBody( theActualFuckingFuck );
	// theActualFuckingFuck= nullptr;

	// food.erase(foodParticle);
// }
// catch (...){
// 	;
// }

	// }
// }

void deleteJoint(BoneUserData * bone) {

		if (!bone->isRoot) { // root bones dont have joints

			if (bone->joint != NULL && bone->joint != nullptr) {
					if (bone->joint->isUsed && bone->joint->init) {

						if (bone->joint->p_joint != NULL && bone->joint->p_joint != nullptr) {

						
							local_m_world->DestroyJoint(bone->joint->p_joint);	
							// fish->bones[i]->joint = nullptr;
						}
					}
				}
			}
			

}

void deleteBone (BoneUserData * bone) {

	if (bone->isUsed && bone->init && bone->flagDelete) {

	
			
				 local_m_world->DestroyBody(bone->p_body);
				 // fish->bones[i]->p_body = nullptr;
				 bone->isUsed = false;
				 bone->init = false;
			}
}

// delete a fish from the game world and remove it from memory
void deleteFish (BonyFish * fish) {
	// if (fishSlotLoaded[fishIndex]) {
		// if (!fishChecker(fishIndex)) {
		// 	return;
		// }
	if (!fish->isUsed) {
		return;
	}

	if (fish->flagDelete) {

		for (unsigned int i = 0; i < N_FINGERS ; ++i) {
		fish->bones[i]->flagDelete = true;
			printf("deleting joint %i.\n", i);
			deleteJoint(fish->bones[i]);
		}

	
		for (unsigned int i = 0; i < N_FINGERS ; ++i) {

			printf("deleting bone %i.\n", i);
			deleteBone(fish->bones[i]);
		}

		fish->isUsed = false;
	}


}


// void deleteFishForActual () {
// 	deleteBone(fish->bones[3]);
// }

void loadFish (fishDescriptor_t driedFish, fann * nann, b2Vec2 startingPosition) {
	// fishes[fishIndex] =

	// BonyFish * fish =  new

	

// BonyFIsh buttfish = ;
	

	fishes.push_back(  *(new BonyFish(driedFish, nann, startingPosition)) );

	// std::list<BonyFish>::iterator fish = 
	BonyFish * fish = &(fishes.back());
	fish->isUsed = true;

	for (int i = 0; i < N_FINGERS; ++i) {
		if (driedFish.bones[i].used) {
			nonRecursiveBoneIncorporator( fish->bones[i]);
		}

		fish->bones[i]->p_owner = fish; // you need to update the user data pointer, because when you pushed the fish onto the list you pushed a copy of it not the actual thing.
	}

}

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
	aabb.upperBound = b2Vec2(0.0f,0.0f);
	aabb.lowerBound = b2Vec2(0.0f,0.0f);
	selected = false;
}

layerDescriptor::layerDescriptor () {
	n_neurons = 0;
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
	return nullptr;
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
			

			if (j == layerCake[i]-1) {
				neuron.biasNeuron = true;
			}
			else {
				neuron.biasNeuron = false;
			}

			neuron.index = rollingIndexCounter;
			rollingIndexCounter ++;

			neuron.activation_function = activation_function_hidden;
  			neuron.activation_steepness = activation_steepness_hidden;
  			neuron.n_connections = 0; 	// so not used uninitialized
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
  		layer->n_neurons = layerCake[i];
  		layer->isUsed = true;

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

void polydactyly (fishDescriptor_t * driedFish) {

	for (unsigned int i = 0; i < N_FINGERS; ++i)
	{
		if (!driedFish->bones[i].used) {

			// if (!driedFish->bones[i].init) {
				// driedFish->bones[i] = new boneAndJointDescriptor_t();	
			// }

			driedFish->bones[i].attachedTo = RNG() * i;

			driedFish->bones[i].used = true;

			break;
		}
	}
}

void amputation (fishDescriptor_t * driedFish) {

	unsigned int usedFingers = 0;

	for (unsigned int i = 0; i < N_FINGERS; ++i)
	{
		if (driedFish->bones[i].used) {

			usedFingers ++;
		}
	}

	unsigned int randomFinger = RNG() * usedFingers;

	driedFish->bones[randomFinger].used = false;

	// for all the limbs that were attached to the random one, you have to remove those too.
	for (unsigned int i = 0; i < N_FINGERS; ++i)
	{
		if (driedFish->bones[i].attachedTo == randomFinger ){
			driedFish->bones[i].used = false;
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

			if (RNG() < (mutationChance / 50)) {	
				polydactyly(fish);
			}

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

void removeDeletableFish() {

	std::list<BonyFish>::iterator fish;

	// for (int i = 0; i < N_FISHES; ++i) {

	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		// if ( fishes[i] == NULL || fishes[i] == nullptr) { 	continue; }
		if (fish->flagDelete && fish->isUsed) {
			deleteFish ( &(*fish)) ;
			// break;
		}
	}

	for (int i = 0; i < N_FOODPARTICLES; ++i) {

		if (food[i]->flagDelete) {
		deleteBone(food[i]);
	}

	}
}

// random new generation from the old winner. reset
void reloadTheSim  () {
	// label every fish as delete.


	std::list<BonyFish>::iterator fish;
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		// if ( fishes[i] == NULL || fishes[i] == nullptr) { 	continue; }
		fish->flagDelete = true;
	}
	for (int i = 0; i < N_FOODPARTICLES; ++i) {

	// std::list<foodParticle_t>::iterator foodParticle;
	// for (foodParticle = food.begin(); foodParticle !=  food.end(); ++foodParticle) 	{

		// if ( food[i] == NULL || food[i] == nullptr) { 	continue; }
		food[i]->flagDelete = true;
	}
	startNextGeneration = true;
}


//  prints the winner to file immediately.
void  vote (BonyFish * winner) {

	fann * wann = winner->ann;

	// save the winner to file with a new name.
	std::string nnfilename =  std::string("mostCurrentWinner.net");
    std::string fdescfilename =  std::string("mostCurrentWinner.fsh");
    std::ofstream file { nnfilename };
    saveFishToFile (fdescfilename, winner->genes);
    fann_save(  wann , nnfilename.c_str()); 

	// }
    // startNextGeneration = true;
	reloadTheSim();	
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

					    if (RNG() < 0.1) {							// chance of a mutation occurring
					    	if (val > 1 || val < -1) { 				// if it's a big number, apply the mutation as a fraction of htat number. else, apply a random amount in a small range. this is to prevent weights being stuck at very small numbers.
					    		val += ( (RNG() -0.5f) * val * 1);
					    	}	
					    	else {
					    		val += ( (RNG() -0.5f) * 0.5f ); 	// how much mutation to apply
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

	}
	

	if (true) {
		local_m_world->DrawParticleSystem(*local_m_particleSystem);
	}

	// draw the food particles
	b2Color fishFoodDye 		= b2Color(0.2f, 0.6f, 0.1f);
	b2Color fishFoodDyeOutline 	= b2Color(0.5f, 0.9f, 0.4f);

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
					local_debugDraw_pointer->DrawPolygon(vertices, 4 , fishFoodDyeOutline);

				}
	}

	// for each bone, get the vertices, then add the body's world location to them, and rotate by the body angle around the body world location.
	// to do this you will need to extend the rotate point method to rotate a polygon.
	// for (unsigned int fishIndex = 0; fishIndex < N_FISHES; ++fishIndex) {

	std::list<BonyFish>::iterator fish;
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		// if (fishSlotLoaded[fishIndex]) {
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
						// printf("yenies");
					}
					
					else {
						local_debugDraw_pointer->DrawPolygon(vertices, 4 , b2Color(0,0,0));
					}

					
				}
			}
		// }
	}
}

void verifyNetworkDescriptor (networkDescriptor * network) {
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



// class newCallbackClass {

// 	float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction){

// 		printf("smelly ness farts");

// 		float myBalls= 0.01f;

// 		return myBalls;

// 	}

// };


// float32 lickerdicker(const b2RayCastInput& input, int32 proxyId)
// 	{


// 		printf("boot boot pankackes");

// 				float myBalls= 0.01f;

// 		return myBalls;


// 	}


// struct b2WorldRayCastWrapper
// {
// 	float32 RayCastCallback(const b2RayCastInput& input, int32 proxyId)
// 	{


// 		printf("smelly ness farts");

// 				float myBalls= 0.01f;

// 		return myBalls;


// 	}


// 	const b2BroadPhase* broadPhase;
// 	b2RayCastCallback* callback;
// };


class RayCastClosestCallback : public b2RayCastCallback
{
public:
	RayCastClosestCallback()
	{
		m_hit = false;
	}

	float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction)
	{
		// printf("peeeee\n");
		b2Body* body = fixture->GetBody();
		void* userData = body->GetUserData();
		uDataWrap* myUserData = (uDataWrap* )body->GetUserData();

		if (myUserData->dataType == TYPE_LEAF && m_deepSeaSettings.gameMode == GAME_MODE_ECOSYSTEM) {
			// printf("sunlight fell on a leaf: %f\n", ((BoneUserData *)(myUserData->uData))->p_owner->energy);

			// ((BoneUserData *)(myUserData->uData))->p_owner->energy += 100;
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

	// float rayLength = 12.0f;

	sunbeam.p2 = b2Vec2(nancy->illuminationRadius * cos(randomDirection),nancy->illuminationRadius* sin(randomDirection));
	sunbeam.maxFraction = 1.0f;
	// int32 childIndex = 0;

	if (true) { // print the rays of light

	   local_debugDraw_pointer->DrawSegment(sunbeam.p1, sunbeam.p2, b2Color(1.0f, 1.0f, 1.0f) );
	}

	RayCastClosestCallback stupidMotherFucker;

	local_m_world->RayCast( &stupidMotherFucker, sunbeam.p1, sunbeam.p2);

	// if (itHitSomething) {	
		// printf("itHitSomething");
	// }

}

void ecosystemModeBeginGeneration (BonyFish * fish) {

	// delete the fish
	// fish->flagDelete = true;


	// create n mutant children near that position

	for (int i = 0; i < 3; ++i)
	{
					fishDescriptor_t newFishBody = fish->genes;
					// loadFishFromFile(std::string("mostCurrentWinner.fsh"), newFishBody);

					mutateFishDescriptor (&newFishBody, 0.1, 0.5);

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

					printf("place children at position x: %f, y%f\n",desiredPosition.x, desiredPosition.y);

					loadFish ( newFishBody, jann, desiredPosition) ;



					moveAWholeFish ( &fishes.back(),  desiredPosition);


	// for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
	}


}

void exploratoryModeBeginGeneration ( ) { // select an animal as an evolutionary winner, passing its genes on to the next generation

	// int n_selected  = 0;
	// int partner1index = 0;
	// int partner2index = 0;

	// // count the number of selected fish to see if you can do a fish fucking.
	// // for (int i = 0; i < N_FISHES; ++i) {

	// std::list<BonyFish>::iterator fish;
	// for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
	// 	// if (fishSlotLoaded[i]) {
	// 		if (fish->selected) {
	// 			if (partner1index == 0) {
	// 				partner1index = i;
	// 			}
	// 			else {
	// 				partner2index = i;
	// 			}
	// 			n_selected ++;
	// 			fish->selected = false;
	// 		}
	// 	// }
	// }

	// if (n_selected >= 2) {

		// save partners information to file
		//  saveFishToFile(std::string("partner1.fsh"), fishes[partner1index]->genes);
		//  saveFishToFile(std::string("partner2.fsh"), fishes[partner2index]->genes);
		//  fann_save(  fishes[partner1index]->ann , std::string("partner1.net").c_str()); 
		//  fann_save(  fishes[partner2index]->ann , std::string("partner2.net").c_str()); 

		//  // empty the world
		// removeDeletableFish();

		// // create new generation
		// for (int i = 0; i < N_FISHES; ++i) {

			// fishDescriptor_t newBabyFish = sexBetweenTwoBodies(fishes[partner1index]->genes, fishes[partner2index]->genes);

			// // sexBetweenTwoMinds(std::string("partner1.net"), std::string("partner1.net"));

			// mutateFishDescriptor (&newBabyFish, 0.1, 0.25);

		 //    mutateFANNFileDirectly(std::string("offspring.net"));

		 //    fann *wowGeezBoye = loadFishBrainFromFile (std::string("mutantGimp")) ;
		 //    loadFish (i, newBabyFish, wowGeezBoye, getRandomPosition()) ;

			// totalFishIncorporator(i);	// spawn them into the world to repeat the cycle
		// }

	// }
	// else {
			// otherwise just reproduce 1 of them asexually
			// clear out the old generation



			
			
			for (int i = 0; i < N_FISHES; ++i) {

				
				bool thereIsAFile = false;

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

					mutateFishDescriptor (&newFishBody, 0.1, 0.5);
				
				    mutateFANNFileDirectly(std::string("mostCurrentWinner.net"));

					// now you can load the mutant ANN.
					fann *mann = loadFishBrainFromFile (std::string("mutantGimp")) ;

					// create a neurodescriptor.
					networkDescriptor * muscleCars=  createNeurodescriptorFromFANN (mann) ;

					verifyNetworkDescriptor(muscleCars);

					fann * jann = createFANNbrainFromDescriptor(muscleCars);

					for (int i = 0; i < N_SENSECONNECTORS; ++i)
					{
						if (newFishBody.inputMatrix[i].sensorType == SENSOR_TIMER) {
								newFishBody.inputMatrix[i].timerPhase = RNG();
						}
					}

					loadFish ( newFishBody, jann, getRandomPosition()) ;

				}
				else { 						// if there is no winner, its probably a reset or new install. make one up

					printf("No genetic material found in game folder. Loading default animal.\n");
					fishDescriptor_t nematode = fishDescriptor_t();

					loadFish ( nematode, NULL,  getRandomPosition()) ;
				}

				// totalFishIncorporator(i);	// spawn them into the world to repeat the cycle

			}

			// addFoodParticle(b2Vec2(124.0f, 3.5f));
	// }

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

void meltSelectedFish () {
	std::list<BonyFish>::iterator fish;
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		if (fish->selected) {
			brainMelter ( &(*fish));

			// refresh the water in the brain jar.
			fish->ann = createFANNbrainFromDescriptor(fish->brain);
			return;
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
	// for (int i = 0; i < N_FISHES; ++i) {

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

 			std::list<connectionDescriptor>::iterator connection;
 			for (connection = neuron->connections.begin(); connection != neuron->connections.end(); connection++) {

		    	b2Color segmentColor = b2Color(connection->connectionWeight,connection->connectionWeight,connection->connectionWeight);
		    	if (connection->connectionWeight > 0) {
		    		segmentColor = b2Color(0.0f,0.0f,connection->connectionWeight);
		    	}
		    	else {
		    		segmentColor = b2Color(abs(connection->connectionWeight),0.0f,0.0f);
		    	}

			    local_debugDraw_pointer->DrawSegment(neuron->position, (getNeuronByIndex(fish->brain, connection->connectedTo))->position,segmentColor );
			}
		}
	}
}

// return a positive number if finds a box or negative if not.
BonyFish * checkNeuroWindow (b2AABB mousePointer) {
	// check to see if you're in a neuro window.
	// for (int i = 0; i < N_FISHES; ++i) {

	std::list<BonyFish>::iterator fish;
	// printf("boslimbg");
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{

		// if (fishChecker(i)) {
		// printf("sleb");
		printab2Vec2(fish->brain->networkWindow.upperBound);

		if (fish->selected) {
			if (fish->brain->networkWindow.Contains(mousePointer)) {
				// printf("nmelmul\n");
				return &(*fish);
			}
		}
	}
	// printf("wililil\n");
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

// provides FEA-based lift and drag calculations
void flightModel(BoneUserData * bone) {

	for (int i = 0; i < N_FINGERS; ++i) {

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
			
			// get the face's center point

			b2Vec2 worldCenter = bone->p_body->GetWorldCenter();
			// b2Vec2 centroid = b2Vec2(p1.y - p2.y, p1.x - p2.x);

			b2Vec2 p1r = rotatePoint(0.0f, 0.0f, bone->p_body->GetAngle(), p1 );
			p1r += worldCenter;

			b2Vec2 p2r = rotatePoint(0.0f, 0.0f, bone->p_body->GetAngle(), p2 );
			p2r += worldCenter;

			// b2Vec2 p2r = rotatePoint(worldCenter.x, worldCenter.y, bone->p_body->GetAngle(), p2 );

			b2Vec2 faceCenter = b2Vec2( (p1r.x + p2r.x)/2, (p1r.y + p2r.y)/2 ) ;

			// draw a dot on it so you know you got it right.
			// b2Vec2 gigglenuts  = faceCenter;
			// b2Vec2 gigggle[] = {
			// 	b2Vec2(gigglenuts.x+0.1f, gigglenuts.y-0.1f), 
			// 	b2Vec2(gigglenuts.x+0.1f, gigglenuts.y+0.1f), 
			// 	b2Vec2(gigglenuts.x-0.1f, gigglenuts.y+0.1f), 
			// 	b2Vec2(gigglenuts.x-0.1f, gigglenuts.y-0.1f), 
			// };
			// local_debugDraw_pointer->DrawFlatPolygon(gigggle, 4 ,b2Color(0.5f,0.1f,0.05f) );


			// // calculate the angle of incidence into the oncoming 'wind'
			b2Vec2 linearVelocity = bone->p_body->GetLinearVelocity();
			float angleOfIncidence = atan2(linearVelocity.x, linearVelocity.y) - 0.5 * pi;

			// // draw the angle of incidence 
			b2Color segmentColorA = b2Color(50, 200, 10);
			local_debugDraw_pointer->DrawSegment(faceCenter ,b2Vec2(faceCenter.x+ cos(angleOfIncidence) * 0.1, faceCenter.y+ sin(angleOfIncidence) * 0.1 * -1) ,segmentColorA );
			

			// // based on the angle, add the corresponding aerodynamic force. This paragraph also contains the aerodynamic equation.
			// b2Vec2 linearVelocity = bone->p_body->GetLinearVelocity();
			// b2Vec2 fluidDynamicForce = b2Vec2(cos(angleOfIncidence) * linearVelocity.x, sin(angleOfIncidence) * linearVelocity.y );

			// if (false) {
			// 	// apply the force to the body directly in the center of the face.
			// 	bone->p_body->ApplyForce(fluidDynamicForce, centroid, true);

			// 	// draw the forces applied so you can debug them.
			// 	b2Color segmentColorD = b2Color(200, 50, 10);
			// 	local_debugDraw_pointer->DrawSegment(worldCenter ,b2Vec2(worldCenter.x+ fluidDynamicForce.x , worldCenter.y+  fluidDynamicForce.y) ,segmentColorD );
			// }
		}
	}
}

void runBiomechanicalFunctions () {
	unsigned int spacesUsedSoFar =0;

	// for (int i = 0; i < N_FISHES; ++i) {

	std::list<BonyFish>::iterator fish;
	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		// if (fishSlotLoaded[i]) {

			// if (fishChecker(i)) {
				// reset neuro bounding boxes... this isnt the right place to do this, should be in a graphics function
			// 	fish->brain->networkWindow.lowerBound = b2Vec2(0.0f,0.0f);
			// 	fish->brain->networkWindow.upperBound = b2Vec2(0.0f,0.0f);
			// // }

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

			if (m_deepSeaSettings.gameMode == GAME_MODE_ECOSYSTEM) { // if in ecosystem mode

			
				for (int i = 0; i < N_FINGERS; ++i)
				{
					if (fish->bones[i]->isLeaf && fish->bones[i]->flagPhotosynth) {
						fish->bones[i]->flagPhotosynth = false;

						printf("sunlight fell on a leaf: %f\n", fish->energy);

			// ((BoneUserData *)(myUserData->uData))->p_owner->
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


	// printf("f: %f r: %f\n", fish->energy , fish->reproductionEnergyCost);
				if (fish->energy > fish->reproductionEnergyCost) {
					ecosystemModeBeginGeneration( &(*fish) );
					fish->energy -= fish->reproductionEnergyCost;
				}



			}

			// if (fishChecker(i)) {_		

					// printf("fanucker");
				if (fish->selected) {
					// printf("EEEEEEEEEEEEEEEEEEEEEEEETTTT");
					drawNeuralNetworkFromDescriptor(motorSignals, sensorium, &spacesUsedSoFar, &(*fish));
				}



				


			// }
		// }
	}
}

void deepSeaLoop () {


	// TestMain::SimulationLoop();

	TestMain::PreStep();

	TestMain::Step();

	if (!local_m_world->IsLocked() ) {


		removeDeletableFish();

		if (startNextGeneration && m_deepSeaSettings.gameMode == GAME_MODE_EXPLORATION ) {
			exploratoryModeBeginGeneration ( );
		}

		std::list<Lamp>::iterator lomp;
		for (lomp = lamps.begin(); lomp !=  lamps.end(); ++lomp) 	{
			shine(&(*lomp));
		}

	
		drawingTest();


		runBiomechanicalFunctions();






		if (flagAddFood) {
			flagAddFood = false;

			addFoodParticle(getRandomPosition());
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

			
		// std::list<foodParticle_t>::iterator foodParticle;
		// for (foodParticle = food.begin(); foodParticle !=  food.end(); ++foodParticle) 	{
				food[i]->position = food[i]->p_body->GetWorldCenter(); // update positions of all the food particles
			}
		}
		
		
	}




	TestMain::PostStep();
	

}

void deepSeaControlP () {
	;
	// addFoodParticle(b2Vec2(0.0f, 0.0f));
	flagAddPlant = true;
}

void deepSeaControlA () {
	;
	// addFoodParticle(b2Vec2(0.0f, 0.0f));
	flagAddFood = true;
}
void deepSeaControlB () {


	// for (int i = 0; i <; ++i)
	// {
	// 	/* code */
	// }

	// ;


	std::list<BonyFish>::iterator fish;

	// for (int i = 0; i < N_FISHES; ++i) {

	for (fish = fishes.begin(); fish !=  fishes.end(); ++fish) 	{
		if (fish->selected) {
			fish->flagDelete = true;
			fish->selected = false;

		}

	}
}

void collisionHandler (void * userDataA, void * userDataB, b2Contact * contact) {
	bool et = false;
	bool fud = false;
	bool lef = false;

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

void ParticleDrawingKeyboard(unsigned char key)
{
	m_drawing = key != 'X';
	m_particleFlags = 0;
	m_groupFlags = 0;
	switch (key)
	{
	case 'E':
		m_particleFlags = b2_elasticParticle;
		m_groupFlags = b2_solidParticleGroup;
		break;
	case 'P':
		m_particleFlags = b2_powderParticle;
		break;
	case 'R':
		m_groupFlags = b2_rigidParticleGroup | b2_solidParticleGroup;
		break;
	case 'S':
		m_particleFlags = b2_springParticle;
		m_groupFlags = b2_solidParticleGroup;
		break;
	case 'T':
		m_particleFlags = b2_tensileParticle;
		break;
	case 'V':
		m_particleFlags = b2_viscousParticle;
		break;
	case 'W':
		m_particleFlags = b2_wallParticle;
		m_groupFlags = b2_solidParticleGroup;
		break;
	case 'B':
		m_particleFlags = b2_barrierParticle | b2_wallParticle;
		break;
	case 'H':
		m_particleFlags = b2_barrierParticle;
		m_groupFlags = b2_rigidParticleGroup;
		break;
	case 'N':
		m_particleFlags = b2_barrierParticle | b2_elasticParticle;
		m_groupFlags = b2_solidParticleGroup;
		break;
	case 'M':
		m_particleFlags = b2_barrierParticle | b2_springParticle;
		m_groupFlags = b2_solidParticleGroup;
		break;
	case 'F':
		m_particleFlags = b2_wallParticle | b2_repulsiveParticle;
		break;
	case 'C':
		m_particleFlags = b2_colorMixingParticle;
		break;
	case 'Z':
		m_particleFlags = b2_zombieParticle;
		break;
	default:
		break;
	}
	TestMain::SetParticleParameterValue(DetermineParticleParameter());
}

	// void ParticleGroupDestroyed(b2ParticleGroup* group) {
	// 	// Test::ParticleGroupDestroyed(group);
	
	// }

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


void stepForParticleDrawing () {
	const uint32 parameterValue = TestMain::GetParticleParameterValue();
		m_drawing = (parameterValue & e_parameterMove) != e_parameterMove;
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

		// Test::Step(settings);
		// local_m_debugDraw.DrawString(
		// 	5, m_textLine, "Keys: (L) liquid, (E) elastic, (S) spring");
		// m_textLine += DRAW_STRING_NEW_LINE;
		// local_m_debugDraw.DrawString(
		// 	5, m_textLine, "(R) rigid, (W) wall, (V) viscous, (T) tensile");
		// m_textLine += DRAW_STRING_NEW_LINE;
		// local_m_debugDraw.DrawString(
		// 	5, m_textLine, "(F) repulsive wall, (B) wall barrier");
		// m_textLine += DRAW_STRING_NEW_LINE;
		// local_m_debugDraw.DrawString(
		// 	5, m_textLine, "(H) rigid barrier, (N) elastic barrier, (M) spring barrier");
		// m_textLine += DRAW_STRING_NEW_LINE;
		// local_m_debugDraw.DrawString(
		// 	5, m_textLine, "(C) color mixing, (Z) erase, (X) move");
		// m_textLine += DRAW_STRING_NEW_LINE;
}


