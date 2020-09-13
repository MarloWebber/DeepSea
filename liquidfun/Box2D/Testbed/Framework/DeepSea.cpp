#include "DeepSea.h"
#include "fann.h"
#include "Test.h"

#include <iostream>
#include <fstream>
#include <random>
#include <string>

#include <fstream>

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
		bodyDef.userData = this;

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
		bodyDef.userData = this;

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
	bodyDef.userData = this; // register the fishfood struct as user data on the body
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

	init = true; // true after the particle has been initialized. In most cases, uninitalized particles will be ignored.
	isUsed = false;

	heartSpeed = 50;



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

}




void loadFish (uint8_t fishIndex, fishDescriptor_t driedFish, b2World * m_world, b2ParticleSystem * m_particleSystem, fann * nann) {

	fishes[fishIndex] = new BonyFish(driedFish, fishIndex , m_world, m_particleSystem, nann);
	fishSlotLoaded[fishIndex] = true;

}


fann * loadFishBrainFromFile (std::string fileName) {
	return fann_create_from_file( (fileName + std::string(".net")).c_str() );
}


void LoadFishFromName (uint8_t fishIndex, b2World * m_world, b2ParticleSystem * m_particleSystem) {

	fishDescriptor_t newFish;

	std::string fileName = "225";

	loadFishFromFile(fileName + std::string(".fsh"), newFish);

 	loadFish ( fishIndex,  newFish,m_world,  m_particleSystem, loadFishBrainFromFile (fileName) ) ;

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
// 

	for (int i = 0; i < N_FISHES; ++i)
	{
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


	b2Vec2 drawingStartingPosition = b2Vec2( (2.0f * index) ,2.0f);
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



void collisionHandler (BoneUserData * boneA, BoneUserData * boneB) {

	// if (boneB == isMouth) {

	// }


	// if (boneA == isMouth) {

	// }

}