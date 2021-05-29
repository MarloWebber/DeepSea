#ifndef DEEPSEA_H
#define DEEPSEA_H

#include "Test.h"
#include "Main.h"
#include <list>
#include "fann.h"

#define N_FINGERS 8 													// the amount of bones a fish can have. you know, fish fingers.
#define N_FOODPARTICLES 8
#define N_SENSECONNECTORS 32

#define GAME_MODE_LABORATORY 0
#define GAME_MODE_ECOSYSTEM 1

#define TERRAIN_TYPE_LIQUID 0
#define TERRAIN_TYPE_OIL 0
#define TERRAIN_TYPE_POWDER 0
#define TERRAIN_TYPE_ELASTIC 0
#define TERRAIN_TYPE_RIGID 0

extern float pi;

struct deepSeaSettings {
	int gameMode;
	int laboratory_nFood;
	int laboratory_nFish;
	b2Vec2 gravity;
	float mutationRate;
	float mutationSeverity;
	float mentalMutationRate;
	float mentalMutationSeverity;
	int terrainPaintType;

	float originTriggerRadius;
	float originFoodRadius;
	float foodRadiusAngleJitter;


	int currentlySelectedSpecies;



};

extern deepSeaSettings m_deepSeaSettings;

b2Vec2 rotatePoint(float cx,float cy,float angle, b2Vec2 p);

struct JointUserData;
struct BoneUserData;
struct BonyFish;

void setUserControlInputA() ;
void setUserControlInputB() ;

extern bool m_drawing;
extern b2ParticleGroup* m_lastGroup;
extern uint32 m_particleFlags;
extern uint32 m_groupFlags;
extern uint32 m_colorIndex;

void ParticleDrawingKeyboard(int key);

enum Parameters {
		e_parameterBegin = (1UL << 31),
		e_parameterMove = e_parameterBegin | (1UL << 0),
		e_parameterRigid = e_parameterBegin | (1UL << 1),
		e_parameterRigidBarrier = e_parameterBegin | (1UL << 2),
		e_parameterElasticBarrier = e_parameterBegin | (1UL << 3),
		e_parameterSpringBarrier = e_parameterBegin | (1UL << 4),
		e_parameterRepulsive = e_parameterBegin | (1UL << 5),
	};

const ParticleParameter::Value k_paramValues[] =
{
	{b2_zombieParticle,
		ParticleParameter::k_DefaultOptions, "erase"},
	{(uint32)e_parameterMove,
		ParticleParameter::k_DefaultOptions, "move"},
	{(uint32)e_parameterRigid,
		ParticleParameter::k_DefaultOptions, "rigid"},
	{(uint32)e_parameterRigidBarrier,
		ParticleParameter::k_DefaultOptions, "rigid barrier"},
	{(uint32)e_parameterElasticBarrier,
		ParticleParameter::k_DefaultOptions, "elastic barrier"},
	{(uint32)e_parameterSpringBarrier,
		ParticleParameter::k_DefaultOptions, "spring barrier"},
	{(uint32)e_parameterRepulsive,
		ParticleParameter::k_DefaultOptions, "repulsive wall"}
};

const ParticleParameter::Definition k_paramDef[] =
{
	{
		ParticleParameter::k_particleTypesPtr,
		ParticleParameter::k_particleTypesCount
	},
	{
		k_paramValues,
		B2_ARRAY_SIZE(k_paramValues)
	},
};
const uint32 k_paramDefCount =
	B2_ARRAY_SIZE(k_paramDef);

struct Lamp {
	unsigned int brightness;
	unsigned int illuminationRadius;
	b2Vec2 position;
	b2Color illuminationColor;

	Lamp();
};

/**
* @brief condensed information about a fish body segment
*/
struct boneAndJointDescriptor_t {
		bool used;
		bool isRoot;
		bool isMouth;
		bool isWeapon;
		bool isLeaf;
		bool isFood;
		bool sensor_radar; // like an olfactory sensor . senses distance from food
		bool sensor_touch; // like how you can feel when things touch your skin.
		bool sensor_jointangle;

		unsigned int attachedTo; // the INDEX (out of N_FINGERS) of the bone it is attached to. Storing data in this way instead of a pointer means that mutating it will have hilarious rather than alarming results.

		float torque;
		float speedLimit;
		float upperAngle;
		float normalAngle;
		float lowerAngle;
		float length;
		float rootThickness;
		float tipThickness;
		b2Color color;
		b2Color outlineColor;

		boneAndJointDescriptor_t();
};

// these type codes are used to route sensory and motor information between brains and limbs.
#define SENSECONNECTOR_UNUSED 				0
#define SENSECONNECTOR_MOTOR 				1
#define SENSOR_FOODRADAR					2
#define SENSOR_TOUCH 						3
#define SENSOR_JOINTANGLE 					4
#define SENSOR_TIMER 						5
#define SENSECONNECTOR_RECURSORRECEIVER 	6
#define SENSECONNECTOR_RECURSORTRANSMITTER 	7

#define SENSECONNECTOR_BUFFERSIZE			64	// the maximum size of the buffer used for recursion delay.


/*!
* @brief associates a brain input to a particular sensor.
*
*
*/
struct senseConnector {
	unsigned int connectedToLimb;		// what limb the sense is coming from, or motor signal is going to.
	unsigned int connectedToNeuron;		// neuron index. The position of this neuron's layer will determine how the program uses it.
	unsigned int sensorType;  			// what kind of sense it is (touch, smell, etc.how the number will be treated)
	float timerFreq;					// if a timer, the frequency.
	float timerPhase;					// if a timer, the phase. Used so that the senseconnector can be a fully self contained timer.
	unsigned int recursorChannel; 		// 
	unsigned int recursorDelay;
	float recursorBuffer[SENSECONNECTOR_BUFFERSIZE];
	unsigned int recursorCursor;

	senseConnector();
};

/*!
* @brief condensed form of fish data for storage and transfer.
* 
*/
struct fishDescriptor_t {

	boneAndJointDescriptor_t bones[N_FINGERS];

	// unsigned int heartSpeed;

	senseConnector inputMatrix[N_SENSECONNECTORS]; // these need to get serialized too. so this is a workable place for them.
	senseConnector outputMatrix[N_SENSECONNECTORS];

	fishDescriptor_t();
};


/*!
* @brief a controllable joint between bodies.
*
*/
struct JointUserData {
	float torque; 	
	float speed; 	
	float speedLimit;
	bool driveCW;	// a signal that tells the motor to turn in one direction. This is much simpler than trying to drive it with a number and having to remember positon etc. With this, you just hit the button, or don't.
	bool driveCCW;	// a signal that tells the motor to turn in the other direction.
	float upperAngle;
	float normalAngle;
	float lowerAngle;

	b2RevoluteJointDef jointDef;
	b2RevoluteJoint * p_joint; // the joint that this user data struct gets pinned to

	bool init;
	bool isUsed;

	BoneUserData * attaches;
	BoneUserData * attachedTo;

	JointUserData(boneAndJointDescriptor_t boneDescription, BoneUserData * p_bone, BonyFish * fish) ;
} ;

/*!
* @brief the rigid segments of an organism, plant, or food particle.
*
*/
struct BoneUserData {
	float length;
	float rootThickness;
	float tipThickness;
	float density;
	b2Vec2 tipCenter; 				// these are used so the skeleton master can remember his place as he traverses the heirarchy of souls.
	b2Vec2 rootCenter; 
	JointUserData * joint; 		// the joint that attaches it into its socket		
	unsigned int attachedTo;	
	bool isRoot ;
	bool isMouth ;
	bool isLeaf;
	bool isFood;

	bool sensor_radar ; // like an olfactory sensor . senses distance from food
	bool sensor_touch; // like how you can feel when things touch your skin.
	bool sensor_jointangle;

	b2Vec2 offsetOnBody;

	float sensation_radar;
	float sensation_touch;
	float sensation_jointangle;

	bool isWeapon ;					// weapons destroy joints to snip off a limb for consumption. optionally, they can produce a physical effect such as an explosion.
	float energy; 					// the nutritive energy stored in the tissue of this limb; used by predators and scavengers

	b2BodyDef bodyDef;
	b2Body * p_body;
	b2PolygonShape shape;
	BonyFish * p_owner;
	b2Fixture * p_fixture;

	int collisionGroup;

	b2Vec2 position;
	bool init;		// if the struct has been created properly
	bool isUsed;	// if the bone is actually used in the game, presumably not all animals will use all 8

	b2Color color;
	b2Color outlineColor;

	bool flagDelete; // flag this whole animal for deletion at the next convenient time.
	bool flagPhotosynth;
	

	bool selected;


	BoneUserData(boneAndJointDescriptor_t boneDescription,
		BonyFish * fish,
		b2Vec2 positionOffset,
		int collisionGroup,
		bool attached);
} ;

/*!
* @brief a connection between two neurons inside the brain.
*
*/
struct connectionDescriptor {
	bool isUsed;
	unsigned int connectedTo;
	float connectionWeight;	

	connectionDescriptor(unsigned int toNeuron);
};




/*!
* @brief a single living brain cell.
*
*/
struct neuronDescriptor {
	bool isUsed;
	// unsigned int n_inputs;
	unsigned int activation_function;
	float activation_steepness;

	// unsigned int n_connections;
	std::list<connectionDescriptor> connections;

	b2Vec2 position;
	b2AABB aabb;

	unsigned int index; // the fann file index of this neuron. handy to refer to

	bool biasNeuron;

	bool selected;
	bool locked;



	neuronDescriptor();
};


/*!
* @brief a layer of neurons inside the brain, the primary method of organisation for brain cells.
*
*/
struct layerDescriptor {
	bool isUsed;
	// unsigned int n_neurons;
	std::list<neuronDescriptor> neurons; // an array is a pointer to the start of the array, and this is actually an array of pointers to objects, so neurons[] is a double pointer.

	bool selected;

	layerDescriptor();
};

/*!
* @brief a whole brain, many layers of many cells connected together.
*
*/
struct networkDescriptor {
	/*
	1. it's impossible to modify a FANN network once it is created
	2. modifying the network stored in text is possible, but hard
	3. i might as well make an easily modifiable descriptor file, and methods to turn it back into the text file.
	*/

	// unsigned int n_layers;
	std::list<layerDescriptor> layers;

	b2AABB networkWindow;



	// std::list<senseConnector> inputRouter; // keeps track of what sense input goes to what input neuron; necessary that each animal keeps track of its own routing, to allow animals with different routing to coexist.

	// std::list<senseConnector> inputMatrix;
	// std::list<senseConnector> outputMatrix;




	networkDescriptor(fann* pann);

	// neuronDescriptor * getNeuronByIndex(unsigned int windex);
};


void LoadFishFromName (unsigned int fishIndex) ;

/*!
* @brief an entire living organism, contain a rigid jointed body, a brain, and stored genetic information that can be passed along.
*
*/
struct BonyFish {

	float reproductionEnergyCost;
	float energy; 	// the animal spends energy to move and must replenish it by eating
	bool init; 		// true after the particle has been initialized. In most cases, uninitalized particles will be ignored.
	bool isUsed;	// 

	BoneUserData * bones[N_FINGERS]; // for now, let's just gnetworkDet fish working with a small, hard-linked, flat level set of bones.
	unsigned int n_bones_used;

	// unsigned int heartCountA; 	// the heart is a neuro input used for timing and frequency control. 
	// unsigned int heartSpeed; 	//  
	// float heartOutputA;	// every heartSpeed timesteps, the output changes state between 1 and 0.

	// unsigned int heartCountB; 	// the heart is a neuro input used for timing and frequency control. 
	// // unsigned int heartSpeedB; 	//  
	// float heartOutputB;	// every heartSpeed timesteps, the output changes state between 1 and 0.

	// unsigned int heartCountC; 	// the heart is a neuro input used for timing and frequency control. 
	// // unsigned int heartSpeedC; 	//  
	// float heartOutputC;	// every heartSpeed timesteps, the output changes state between 1 and 0.

	// unsigned int heartCountD; 	// the heart is a neuro input used for timing and frequency control. 
	// // unsigned int heartSpeedD; 	//  
	// float heartOutputD;	// every heartSpeed timesteps, the output changes state between 1 and 0.

	bool flagDelete;
	// bool flagExtraBoneDeleter;

	// bool flagWinner; // flag the animal as winner for this turn.

	struct fann *ann;


	unsigned int slot; // for reference, the slot the fish is loaded into

	bool selected;

	fishDescriptor_t genes; // the fish carries a copy of its own descriptor which is the genetic infomshun it will pass along.
	networkDescriptor * brain;

	senseConnector inputMatrix[N_SENSECONNECTORS]; // these need to get serialized too. so this is a workable place for them.
	senseConnector outputMatrix[N_SENSECONNECTORS];

	// char species[32]; // a textual string describing what species the fish is.


	b2Vec2 previousRootPosition;
	float distanceMovedSoFar;
	float filteredOutputWiggle;
	float * previousOutputs;

	BonyFish(fishDescriptor_t driedFish, fann * nann, b2Vec2 startingPosition);

	void feed(float amount);

};

struct foodParticle_t {
	b2Vec2 position; 			// starting position of the food in the game world
	float energy; 				// the nutritive value of the food

	b2BodyDef bodyDef;
	b2Body * u_body;
	b2PolygonShape shape; 

	bool init; 					// true after the particle has been initialized. In most cases, uninitalized particles will be ignored.
	bool isUsed;

	bool flagDelete;

	bool selected;

	foodParticle_t(b2Vec2 position);
};


// these type codes are used with uDataWrappers to do stuff when physical bodies touch and collide, or with raycasts
#define TYPE_DEFAULT 0
#define TYPE_MOUTH 1
#define TYPE_FOOD 2
#define TYPE_TOUCHSENSOR 3
#define TYPE_LEAF 4

// bit masks can potentially be used instead so we can have more than one udata type.
// #define MASK_DEFAULT 		1<<0
// #define MASK_MOUTH 			1<<1
// #define MASK_FOOD 			1<<2
// #define MASK_TOUCHSENSOR 	1<<3
// #define MASK_LEAF 			1<<4

struct uDataWrap {
	void * uData;
	unsigned int dataType;

	uDataWrap(void * dat, unsigned int type);
};


struct Species {

	std::list<BonyFish> population;

	std::string name;


	unsigned int nominalPopulation; // how many animals the ecosystem should be set to, if enforced

	bool enforcePopulationLimit;	// whether or not the animals should obey the population limit

	bool asexual;					// whether or not the animal has sexy times sex or one player sex

	bool selected;

	b2Vec2 windowVertices[4];

	bool flagDelete;

	Species();

};

// extern std::string speciesNameBar;


void deepSeaControlA () ;
void deepSeaControlB () ;
void deepSeaControlP () ;

void addFoodParticle ( b2Vec2 position) ;
void fishIncorporator (BonyFish * p_fish) ;

void deepSeaSetup(b2World * m_world, b2ParticleSystem * m_particleSystem, DebugDraw * p_debugDraw ) ;//, b2World * m_world_sci, b2ParticleSystem * m_particleSystem_sci) ;
void deepSeaLoop () ;

void makeAJellyfish (BonyFish * p_fish) ;



extern BoneUserData * food[N_FOODPARTICLES];
// extern std::list<foodParticle_t*> food;
// extern BonyFish * fishes[N_FISHES];
// extern std::list<BonyFish> fishes;

extern std::list<Species> ecosystem;

extern Species * defaultSpecies;// = new Species; // a default start because the list cant be used uninitialized. Also, used as the only active species in the laboratory mode.


extern unsigned int currentNumberOfFood;
extern unsigned int currentNumberOfFish;

extern unsigned int currentlySelectedLimb ;

void collisionHandler (void * boneA, void * boneB, b2Contact * p_contact) ;

void reloadTheSim(int arg);
// bool queryScienceMode () ;
// void enterScienceMode();
// void exitScienceMode ();
// void enterScienceModeInterruptableEntry();
void vote(BonyFish * winner);
void handleReproduceSelectedButton (int arg);

void  mutateFANNFileDirectly();

void drawingTest() ;
BonyFish * checkNeuroWindow (b2AABB mousePointer) ;

int checkNeuronsInWindow (b2AABB mousePointer, BonyFish * fish) ;
extern bool startNextGeneration;
void incrementSelectedConnection();
void decrementSelectedConnection();
void meltSelectedFish(int arg);
void scrambleSelectedFish (int arg) ;
// if (startNextGeneration ) {
void		beginGeneration ();


void pinToGrid(int arg);
void releaseFromGrid(int arg);

void selectFishWithGreatestWiggle(int arg);
void selectFishWhoMovedTheFurthest(int arg);
void selectFurthestFromOrigin(int arg);
void selectClosestToFood (int arg) ;

void flagSelectedFishForDeletion(int arg) ;

void placeLimbOnSelectedFish(int arg);
void amputation(int arg);
void deleteSelectedNeuron (int arg) ;
void addLayerToSelectedFish(int arg) ;

void addNeuronInSelectedLayer(int arg) ;

void deleteSelectedLayer(int arg) ;

void addRecursorPair(int arg) ;

void deselectAll(int arg) ;
void selectAll(int arg) ;

void invertSelection (int arg) ;

void voteSelectedFish(int arg) ;

void addRandomFoodParticle(int arg);


void test_runAllUnitTests();

void deepSeaStart();

void populateSpeciesFromFile(int arg) ;

void saveIndividualToFile (int arg) ;

void selectAllInSpecies (int arg) ;

void mateSelectedFish (int arg) ;

void checkClickInSpeciesWindow(b2AABB mousePointer) ;

void addNewSpecies (int arg) ;
void deleteSelectedSpecies (int arg) ;

void selectLowestEnergyFish(int arg) ;

void speciesNameBarCallback(int arg) ;
	// }
#endif