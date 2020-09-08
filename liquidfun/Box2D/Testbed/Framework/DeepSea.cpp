#include "DeepSea.h"
#include "fann.h"
#include "Test.h"

 int currentNumberOfFood = 0;
 int currentNumberOfFish = 0;


foodParticle_t food[N_FOODPARTICLES];
BonyFish * fishes[N_FISHES] = {
	new BonyFish,
	new BonyFish,
	new BonyFish,
	new BonyFish,
	new BonyFish,
	new BonyFish,
	new BonyFish,
	new BonyFish

};

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

// struct boneUserData_t {
// 	// this will comprise the organs of the rigid animals.
// 	// it is like a linked list that connects to other bones.

// 	float length;
// 	float rootThickness;
// 	float tipThickness;

// 	float density;

// 	b2Vec2 tipCenter; // these are used so the skeleton master can remember his place as he traverses the heirarchy of souls.
// 	b2Vec2 rootCenter; 

// 	jointUserData_t * joint; // the joint that attaches it into its socket			

// 	// the non recursive data model does not have this.
// 	// boneUserData_t* bones[N_FINGERS]; // a collection of the other bones that are attached to it. the root bone is 0
// 	// int n_bones;			  // number of child bones that are actually used.

// 	bool isRoot ;
// 	bool isMouth ;

// 	bool isSensor ;
// 	float sensation;

// 	bool isWeapon ;	// weapons destroy joints to snip off a limb for consumption. optionally, they can produce a physical effect.

// 	float energy; // the nutritive energy stored in the tissue of this limb; used by predators and scavengers

// 	b2Body * body;
// 	b2PolygonShape * shape; 

// 	b2Vec2 position;

// 	bool init;

	boneUserData_t::boneUserData_t() {

		// initialize everything blank

		 length = 0.0f;
		 rootThickness = 0.0f;
		 tipThickness = 0.0f;

		 density = 0.0f;

		 tipCenter = b2Vec2(0.0f,0.0f); // these are used so the skeleton master can remember his place as he traverses the heirarchy of souls.
		 rootCenter = b2Vec2(0.0f,0.0f); 

		joint = new jointUserData_t; //&jointUserData_t(); // the joint that attaches it into its socket			

		// the non recursive data model does not have this.
		// boneUserData_t* bones[N_FINGERS]; // a collection of the other bones that are attached to it. the root bone is 0
		// int n_bones;			  // number of child bones that are actually used.

		 isRoot = false;
		 isMouth = false;

		 isSensor = false ;
		 sensation = 0.0f;

		 isWeapon  = false;	// weapons destroy joints to snip off a limb for consumption. optionally, they can produce a physical effect.

		 energy = 0.0f; // the nutritive energy stored in the tissue of this limb; used by predators and scavengers

		// b2Body * body;
		// b2PolygonShape * shape; 


		 position = b2Vec2(0.0f,0.0f);

		 init = false;

		;
	};


// } ;


void nonRecursiveBoneIncorporator(boneUserData_t * p_bone, b2World * m_world, b2ParticleSystem * m_particleSystem) {

	// boneUserData_t bone = *p_bone;



	b2Vec2 cumulativeBonePosition = p_bone->attachedTo->tipCenter;

	// if(bone.joint != nullptr) {
	// 	// printf("the joint was not null\n");
	// }
	// else {
	// 	printf("the joint was null\n");
	// 	return;
	// }

	float angle = p_bone->joint->normalAngle;

	// then you figure out the position of the tip center and do it again for the vertices at the end of the bone.
	b2Vec2 tipCenter = b2Vec2(cumulativeBonePosition.x, cumulativeBonePosition.y + p_bone->length);

		b2Vec2 rootVertexA = b2Vec2(cumulativeBonePosition.x + (p_bone->rootThickness/2), cumulativeBonePosition.y);
		b2Vec2 rootVertexB = b2Vec2(cumulativeBonePosition.x - (p_bone->rootThickness/2), cumulativeBonePosition.y);
		// rootVertexA = rotatePoint( cumulativeBonePosition.x, cumulativeBonePosition.y, angle, rootVertexA);
		// rootVertexB = rotatePoint( cumulativeBonePosition.x, cumulativeBonePosition.y, angle, rootVertexB);
	
		b2Vec2 tipVertexA = b2Vec2(tipCenter.x + (p_bone->tipThickness/2), tipCenter.y);
		b2Vec2 tipVertexB = b2Vec2(tipCenter.x - (p_bone->tipThickness/2), tipCenter.y);
		// tipVertexA = rotatePoint( tipCenter.x, tipCenter.y, angle, tipVertexA);
		// tipVertexB = rotatePoint( tipCenter.x, tipCenter.y, angle, tipVertexB);

	int count = 4;
	b2Vec2 vertices[count];

	// CW
	vertices[0].Set(rootVertexB.x, rootVertexB.y);
	vertices[1].Set(tipVertexB.x, tipVertexB.y);
	vertices[2].Set(tipVertexA.x, tipVertexB.y);
	vertices[3].Set(rootVertexA.x, rootVertexA.y);

	// figure out the center point.
	b2Vec2 boneCenter = b2Vec2(cumulativeBonePosition.x, cumulativeBonePosition.y + (2*p_bone->length));
	boneCenter = rotatePoint(cumulativeBonePosition.x, cumulativeBonePosition.y, angle, boneCenter);

	// attach pointers to the b2 structs into the user data object and vice versa. for b2body this has to be done before you take the bodydef object to the factory				
	// generate the b2 bodies and shapes. 
	b2BodyDef bd1;

	bd1.userData = p_bone;

	bd1.type = b2_dynamicBody;
	b2Body* body1 = m_world->CreateBody(&bd1);
	b2PolygonShape shape1;

	// they are added into the world
	shape1.SetAsBox(p_bone->rootThickness, p_bone->length, boneCenter, angle);	
	body1->CreateFixture(&shape1, 1.5f);
	m_particleSystem->DestroyParticlesInShape(shape1,
											  body1->GetTransform());

	p_bone->body = body1;
	p_bone->shape = &shape1;

	p_bone->tipCenter = tipCenter;
	p_bone->rootCenter = cumulativeBonePosition;

	// create the joint that pins it to the root bone.


	if (previousBone != nullptr) {
			b2RevoluteJointDef jointDef2;
			jointDef2.bodyA = p_bone->attachedTo->body;
			jointDef2.bodyB = p_bone->body;
			jointDef2.localAnchorA =  cumulativeBonePosition;
			jointDef2.localAnchorB =  cumulativeBonePosition;
			jointDef2.enableLimit = true;
			jointDef2.lowerAngle = p_bone->joint->lowerAngle;
			jointDef2.upperAngle = p_bone->joint->upperAngle;

			jointDef2.enableMotor = true;
            jointDef2.maxMotorTorque = 1.0f;
            jointDef2.motorSpeed = 0.0f;

            jointDef2.userData = p_bone->joint;

            p_bone->joint->init = true;

			m_world->CreateJoint(&jointDef2);
       
	}


	
	// run this same function on any new bones that the bone had.
	// for (int i = 0; i < bone.n_bones; i++) {
	// 	boneUserData_t * p_sub_bone = (boneUserData_t*)bone.bones[i];
	// 	recursiveBoneIncorporator(p_sub_bone, tipCenter, m_world, m_particleSystem, &bone);
	// };

	// set cumulative bone position back to the root, so that it is in the right place for the next lot of sub bones.
	// cumulativeBonePosition = bone.rootCenter;

	// that was my favorite episode of bones.
	p_bone->init = true;

}

// void recursiveBoneIncorporator(boneUserData_t*  p_bone, b2Vec2 cumulativeBonePosition, b2World * m_world, b2ParticleSystem * m_particleSystem, boneUserData_t * previousBone) {

// 	/*
// 	this is intended to be a recursive function that creates the heirarchical bone animals. this function is run manually for the first array of bones and then autoruns subsequently for their children bones.

// 	cumulative bone position is the tip of the previous bone, i.e. the working area for adding new ones.
// 	*/

// 	// generate the b2 vectors for each bone
// 	// first, the center of the root of the bone will be at cumulativeBonePosition.
// 	// then, the vertexes on the root are placed 1/2 the width to either side, and rotated by the bone angle.
// 	boneUserData_t bone = *p_bone;

// 	if(bone.joint != nullptr) {
// 		// printf("the joint was not null\n");
// 	}
// 	else {
// 		printf("the joint was null\n");
// 		return;
// 	}

// 	float angle = bone.joint->normalAngle;

// 	// then you figure out the position of the tip center and do it again for the vertices at the end of the bone.
// 	b2Vec2 tipCenter = b2Vec2(cumulativeBonePosition.x, cumulativeBonePosition.y + bone.length);

// 		b2Vec2 rootVertexA = b2Vec2(cumulativeBonePosition.x + (bone.rootThickness/2), cumulativeBonePosition.y);
// 		b2Vec2 rootVertexB = b2Vec2(cumulativeBonePosition.x - (bone.rootThickness/2), cumulativeBonePosition.y);
// 		// rootVertexA = rotatePoint( cumulativeBonePosition.x, cumulativeBonePosition.y, angle, rootVertexA);
// 		// rootVertexB = rotatePoint( cumulativeBonePosition.x, cumulativeBonePosition.y, angle, rootVertexB);
	
// 		b2Vec2 tipVertexA = b2Vec2(tipCenter.x + (bone.tipThickness/2), tipCenter.y);
// 		b2Vec2 tipVertexB = b2Vec2(tipCenter.x - (bone.tipThickness/2), tipCenter.y);
// 		// tipVertexA = rotatePoint( tipCenter.x, tipCenter.y, angle, tipVertexA);
// 		// tipVertexB = rotatePoint( tipCenter.x, tipCenter.y, angle, tipVertexB);

// 	int count = 4;
// 	b2Vec2 vertices[count];

// 	// CW
// 	vertices[0].Set(rootVertexB.x, rootVertexB.y);
// 	vertices[1].Set(tipVertexB.x, tipVertexB.y);
// 	vertices[2].Set(tipVertexA.x, tipVertexB.y);
// 	vertices[3].Set(rootVertexA.x, rootVertexA.y);

// 	// figure out the center point.
// 	b2Vec2 boneCenter = b2Vec2(cumulativeBonePosition.x, cumulativeBonePosition.y + (2*bone.length));
// 	boneCenter = rotatePoint(cumulativeBonePosition.x, cumulativeBonePosition.y, angle, boneCenter);

// 	// attach pointers to the b2 structs into the user data object and vice versa. for b2body this has to be done before you take the bodydef object to the factory				
// 	// generate the b2 bodies and shapes. 
// 	b2BodyDef bd1;

// 	bd1.userData = &bone;

// 	bd1.type = b2_dynamicBody;
// 	b2Body* body1 = m_world->CreateBody(&bd1);
// 	b2PolygonShape shape1;

// 	// they are added into the world
// 	shape1.SetAsBox(bone.rootThickness, bone.length, boneCenter, angle);	
// 	body1->CreateFixture(&shape1, 1.5f);
// 	m_particleSystem->DestroyParticlesInShape(shape1,
// 											  body1->GetTransform());

// 	bone.body = body1;
// 	bone.shape = &shape1;

// 	bone.tipCenter = tipCenter;
// 	bone.rootCenter = cumulativeBonePosition;

// 	// create the joint that pins it to the root bone.


// 	if (previousBone != nullptr) {
// 			b2RevoluteJointDef jointDef2;
// 			jointDef2.bodyA = previousBone->body;
// 			jointDef2.bodyB = bone.body;
// 			jointDef2.localAnchorA =  cumulativeBonePosition;
// 			jointDef2.localAnchorB =  cumulativeBonePosition;
// 			jointDef2.enableLimit = true;
// 			jointDef2.lowerAngle = bone.joint->lowerAngle;
// 			jointDef2.upperAngle = bone.joint->upperAngle;

// 			jointDef2.enableMotor = true;
//             jointDef2.maxMotorTorque = 1.0f;
//             jointDef2.motorSpeed = 0.0f;

//             jointDef2.userData = &bone.joint;

//             bone.joint->init = true;

// 			m_world->CreateJoint(&jointDef2);
       
// 	}
	
// 	// run this same function on any new bones that the bone had.
// 	for (int i = 0; i < bone.n_bones; i++) {
// 		boneUserData_t * p_sub_bone = (boneUserData_t*)bone.bones[i];
// 		recursiveBoneIncorporator(p_sub_bone, tipCenter, m_world, m_particleSystem, &bone);
// 	};

// 	// set cumulative bone position back to the root, so that it is in the right place for the next lot of sub bones.
// 	cumulativeBonePosition = bone.rootCenter;

// 	// that was my favorite episode of bones.
// 	bone.init = true;
// };


float magnitude (b2Vec2 vector) {
	return sqrt( (vector.x * vector.x) + (vector.y * vector.y));
}


void nonRecursiveSensorUpdater (boneUserData_t * p_bone) {
p_bone->position = p_bone->body->GetPosition();
	p_bone->sensation = 0.0f;


	for  (int i = 0; i < N_FOODPARTICLES; i++) {
		if (food[i].init) {
			
			b2Vec2 positionalDifference = b2Vec2((p_bone->position.x - food[i].position.x),(p_bone->position.y - food[i].position.y));

			float distance = magnitude (positionalDifference);
			if (distance > 0) {

				p_bone->sensation += 1/distance;
			}


		}

	}

	printf("sensation: %f\n", p_bone->sensation);

}

// to traverse the hierarchy and update the sensation of sensors, we need another special recursive function.
// void recursiveSensorUpdater (boneUserData_t * p_bone) {


// 	// update the position of the sensor bone
// 	p_bone->position = p_bone->body->GetPosition();
// 	p_bone->sensation = 0.0f;


// 	for  (int i = 0; i < N_FOODPARTICLES; i++) {
// 		if (food[i].init) {
			
// 			b2Vec2 positionalDifference = b2Vec2((p_bone->position.x - food[i].position.x),(p_bone->position.y - food[i].position.y));

// 			float distance = magnitude (positionalDifference);
// 			if (distance > 0) {

// 				p_bone->sensation += 1/distance;
// 			}


// 		}

// 	}

// 	printf("sensation: %f\n", p_bone->sensation);

// 	// this part segfaults.
// 	// boneUserData_t bone = *p_bone;
// 	for  (int i = 0; i < N_FINGERS; i++) {
// 		if (p_bone->bones[i]->init) {
// 			// recursiveSensorUpdater(p_bone->bones[i]);  // recursing this is required unless the sensor is the root bone. but it segfaults if you do.
// 		}
// 	}



// }






// this should be a class method
// void fishBrainCreator () {
// 	// const unsigned int num_input = 2;
//  //    const unsigned int num_output = 1;
//  //    const unsigned int num_layers = 3;
//  //    const unsigned int num_neurons_hidden = 3;
//  //    const float desired_error = (const float) 0.001;
//  //    const unsigned int max_epochs = 500000;
//  //    const unsigned int epochs_between_reports = 1000;

//  //    struct fann *ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_output);

//  //    fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
//  //    fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);

//     // fann_train_on_file(ann, "xor.data", max_epochs, epochs_between_reports, desired_error);

//     // fann_save(ann, "xor_float.net");

//     // fann_destroy(ann);

//     return ;



// }



// add a food particle to the world and register it so the game knows it exists.
void addFoodParticle ( b2Vec2 position, b2World * m_world, b2ParticleSystem * m_particleSystem) {

		// // add a food particle to the game world
		// 		// no idea how to change the color

				// fishFood = ;
				food[currentNumberOfFood].energy = 1.0f;

				b2BodyDef bd9;
				bd9.userData = &food[currentNumberOfFood]; // register the fishfood struct as user data on the body
				bd9.type = b2_dynamicBody;
				b2Body* body9 = m_world->CreateBody(&bd9);
				b2CircleShape shape9;
				// shape9.SetUserData(&fishFood)	// and also on the shape
				shape9.m_p.Set(-position.x,position.y );
				shape9.m_radius = 0.02f;
				body9->CreateFixture(&shape9, 4.0f);
				m_particleSystem->DestroyParticlesInShape(shape9,
														  body9->GetTransform()); // snip out the particles that are already in that spot so it doesn't explode

				food[currentNumberOfFood].body = body9;
				food[currentNumberOfFood].shape = &shape9;

				food[currentNumberOfFood].init = true;

				currentNumberOfFood++;

}



// void fishIncorporator (BonyFish * p_fish,  b2World * m_world, b2ParticleSystem * m_particleSystem) { // replaced by fish class methods.

// 	fishes[currentNumberOfFish] = *p_fish;
 

// 	recursiveBoneIncorporator(fishes[currentNumberOfFish].bone, fishes[currentNumberOfFish].position, m_world, m_particleSystem, nullptr);

// 	fishes[currentNumberOfFish].init = true;

	

// 	currentNumberOfFish ++;
// }


// void fishDestroyer () {
// 	;
// }

BonyFish::BonyFish()
{
	 	hunger = 0.0f; // the animal spends energy to move and must replenish it by eating
		 position = b2Vec2(0.0f, 0.0f); // the starting position of the fish in the game world
		 init = false; // true after the particle has been initialized. In most cases, uninitalized particles will be ignored.


		for (int i = 0; i < N_FINGERS; ++i)
		{
			/* code */
			bones[i] = boneUserData_t();
		}
}



// struct BonyFish {
// 	// these are the animals made from rigid physical objects in the game world.
// 	// they are comprised of skeletons that brachiate from an origin point. each bone ends in none, one, or several others. bones can be jointed and can move to apply force.
// 	// these have the potential to be simple to implement.


// 	float hunger; // the animal spends energy to move and must replenish it by eating
// 	b2Vec2 position; // the starting position of the fish in the game world
// 	bool init; // true after the particle has been initialized. In most cases, uninitalized particles will be ignored.


// 	// boneUserData_t * bone;
// 	boneUserData_t bones[N_FINGERS]; // for now, let's just get fish working with a small, hard-linked, flat level set of bones.
// 	// jointUserData_t joints[N_FINGERS];

// 	// fishBrain_t * brain;


// 	BonyFish::BonyFish () { // this is supposed to be a constructor that populates all of the objects so none of them are uninitizlied.


// 		 hunger = 0.0f; // the animal spends energy to move and must replenish it by eating
// 		 position = b2Vec2(0.0f, 0.0f); // the starting position of the fish in the game world
// 		 init = false; // true after the particle has been initialized. In most cases, uninitalized particles will be ignored.



// 		for (int i = 0; i < N_FINGERS; ++i)
// 		{
// 			/* code */
// 			bones[i] = boneUserData_t();
// 		}

// 	}

// 	incorporate () { 

// 	}

// 	decorporate () {

// 	}
// };




void makeAJellyfish (BonyFish * p_fish) {

	// this describes the original 3 boned jellyfish.
/*
	float torque; 	
	float speed; 	
	float speedLimit;
	bool driveCW;	// a signal that tells the motor to turn in one direction. This is much simpler than trying to drive it with a number and having to remember positon etc. With this, you just hit the button, or don't.
	bool driveCCW;	// a signal that tells the motor to turn in the other direction.
	float upperAngle;
	float normalAngle;
	float lowerAngle;
	b2RevoluteJoint * joint; // the joint that this user data struct gets pinned to
	*/
jointUserData_t joint1 = {
	1.0f,
	0.0f,
	2.0f,
	false,
	false,
	0.05f,
	-0.15f * pi,
	0.25f,
	nullptr,
	false
};

jointUserData_t joint2 = {
	1.0f,
	0.0f,
	2.0f,
	false,
	false,
	0.05f,
	0.15f * pi,
	0.25f,
	nullptr,
	false
};

jointUserData_t joint3 = {
	1.0f,
	0.0f,
	2.0f,
	false,
	false,
	0.0f,
	0.0f * pi,
	0.0f,
	nullptr,
	false
};

/*


	float length;
	float rootThickness;
	float tipThickness;

	float density;

	b2Vec2 tipCenter; // these are used so the skeleton master can remember his place as he traverses the heirarchy of souls.
	b2Vec2 rootCenter; 

	jointUserData_t * joint; // the joint that attaches it into its socket			

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

	bool init;

*/

boneUserData_t bone0 = {
	// .length = 0.15f,
	// .rootThickness = 0.01f,
	// .tipThickness = 0.01f,
	// .density = 1.5f,
	// .bones[0] = &bone1,
	// .bones[1] = &bone2,
	// .isMouth = true,
	// .isSensor = true
	0.15f,
	0.01f,
	0.01f,
	1.5f,
	b2Vec2(0,0),
	b2Vec2(0,0),
	&joint3,
	nullptr,
	true,
	true,
	true,
	1.0f,
	false,
	0.0f,
	nullptr,
	nullptr,
	b2Vec2(0,0),
	false
};


boneUserData_t bone1 = {
	0.1f,
	0.01f,
	0.01f,
	1.5f,
	b2Vec2(0,0),
	b2Vec2(0,0),
	&joint1,
	&bone0,
	false,
	false,
	false,
	0.0f,
	false,
	0.0f,
	nullptr,
	nullptr,
	b2Vec2(0,0),
	false
};

boneUserData_t bone2 = {
	0.1f,
	0.01f,
	0.01f,
	1.5f,
	b2Vec2(0,0),
	b2Vec2(0,0),
	&joint2,
	&bone0,
	false,
	false,
	false,
	0.0f,
	false,
	0.0f,
	nullptr,
	nullptr,
	b2Vec2(0,0),
	false
};



BonyFish simpleJellyfish = {
	{bone0,bone1,bone2,nullptr,nullptr,nullptr,nullptr,nullptr},
	0.0f,
	b2Vec2(0.0f,2.5f),
	false
}; 

*p_fish = simpleJellyfish;


// the below part puts the fish into the world.


for (int i = 0; i < N_FINGERS; ++i)
{
	/* code */
	if (p_fish->bones[i] != nullptr) {
		nonRecursiveBoneIncorporator(p_fish->bones[i], b2World * m_world, b2ParticleSystem * m_particleSystem) ;
	}

	
}



}




void deepSeaSetup (b2World * m_world, b2ParticleSystem * m_particleSystem) {

	// initialize 10 food particles

		// // add a food particle to the game world
		addFoodParticle(b2Vec2(2.5f, 2.5f), m_world, m_particleSystem);

	// initialize 10 fish

		makeAJellyfish(&fishes[0]);

		// for (int i = 0; i < N_FISHES; ++i)
		// {
		// 	/* code */
		// 	fishes[i] = BonyFish();

		// 	for (int j = 0; j < N_FINGERS; ++j)
		// 	{
		// 		fishes[i].bones[j] = 
		// 	}

		// 	fishes[i].init = true;
		// }

}


void deepSeaLoop () {

	// printf("ngingual\n");

	// get positions of all the food particles
	for  (int i = 0; i < N_FOODPARTICLES; i++) {
		// printf("fenies\n");
		if (food[i].init) {
			// printf("lecmoca\n");
			food[i].position = food[i].body->GetPosition();
		}
	}

	// for each fish in the game world
	for (int i = 0; i < N_FISHES; i++) {
		// printf("tlemnote\n");
		if (fishes[i]->init) {
			// printf("reftif\n");

			// iterate through the bones of the fish and update any sensors
			for (int j = 0; j < N_FINGERS; ++j)
			{
				/* code */
				// nonRecursiveSensorUpdater( &(fishes[i]->bones[j]));
			}


		

		// sensory detection of food
			// each sensor on the animal simply indicates a float number which is the amount of food it 'smells'. it does not tell direction and cannot distinguish different smell sources

		// running of the brain or behavior algorithm

		// motor outputs or other controls


		}



	}




	;
}