




		// liquidfun structs contain a place for a pointer to user data.
		// for each different kind of liquidfun struct i use, i should create a corresponding user data struct that pins it into my game.
		// these are going to get generated first, by the game logic



		// prototypes
		class BonyFish;
		// typedef struct SquishyFish;

		
		// 
		typedef struct jointUserData {

			float torque; 	
			float speed; 	
			float speedLimit;

			bool driveCW;	// a signal that tells the motor to turn in one direction. This is much simpler than trying to drive it with a number and having to remember positon etc. With this, you just hit the button, or don't.
			bool driveCCW;	// a signal that tells the motor to turn in the other direction.

			float upperAngle;
			float lowerAngle;

			b2RevoluteJoint joint; // the joint that this user data struct gets pinned to
		}



		typedef struct boneUserData {
			// this will comprise the organs of the rigid animals.

			float length;
			float rootThickness;
			float tipThickness;

			float density;


			jointUserData joint; // the joint that attaches it into its socket			

			boneUserData bones[8]; // a collection of the other bones that are attached to it.
			// 


			bool isSensor = false;
			float foodSensor = 0.0f;


			b2Body body;
			b2PolygonShape shape; 


		}


		
		// typedef struct particleGroupUserData {
		// 	// this will typically comprise organs of the squishy animals.

		// }


		class BonyFish {
			// these are the animals made from rigid physical objects in the game world.
			// they are comprised of skeletons that brachiate from an origin point. each bone ends in none, one, or several others. bones can be jointed and can move to apply force.
			// these have the potential to be simple to implement.

			// let us say that the bone arrays can be capped to 8 for now.
			public:
				b2boneUserData bones[8];

			// a neural network brain

			// the starting position of the fish in the game world



			void incorporate () {

				// for each bone
				// generate the b2 vectors for each vertex
				// rotate and translate them into the appropriate game coordinates
				// generate the b2 bodies and shapes
				// attach pointers to the b2 structs into the user data object and vice versa
				// deploy them into the world

				// for each joint
				// figure out the joint position
				// create the b2 joint object
				// reference it to the user data object
				// insert the joint into the game world

			}


			private:



		}

		// typedef struct SquishyFish {
		// 	// this is the type of animal made of particle groups.
		// 	// liquidfun already provides a range of softbody materials that is enough to construct simple animal bodies, with stiff parts, flexible and elastic parts.
		// 	// these have the potential to be cuter and more hilarious than the very serious rigid-body animals.
		// 	// particle groups can be made into finer and more detailed shapes than the rigid bones.
		// 	// to create a muscle, we must provide force onto the tissue using liquidfun's force tool. we have to figure that out ourselves.
		// }


		// this describes the original 3 boned jellyfish.

				boneUserData bone1 = {
					.length = 0.1f;
					.rootThickness = 0.01f;
					.tipThickness = 0.01f;
					.density = 1.5f;
					.joint = b2JointUserData joint1 = {
						.torque = 1.0f;
						.speed = 0.0f;
						.speedLimit = 2.0f;
						.upperAngle = 0.05f;
						.lowerAngle = 0.25f;
					}
					.bones = {};
				};

				boneUserData bone2 = {
					.length = 0.1f;
					.rootThickness = 0.01f;
					.tipThickness = 0.01f;
					.density = 1.5f;
					.joint = b2JointUserData joint2 = {
						.torque = 1.0f;
						.speed = 0.0f;
						.speedLimit = 2.0f;
						.upperAngle = 0.25f;
						.lowerAngle = 0.05f;
					}
					.bones = {};
				};

				boneUserData bone0 = {
					.length = 0.15f;
					.rootThickness = 0.01f;
					.tipThickness = 0.01f;
					.density = 1.5f;
					.joint = b2JointUserData joint0 = {
						.torque = 0.0f;
						.speed = 0.0f;
						.speedLimit = 0.0f;
						.upperAngle = 0.0f;
						.lowerAngle = 0.0f;
					}
					.bones = {bone1, bone2};
				};

				BonyFish simpleJellyfish;

				simpleJellyfish.bones = {bone0};






		BonyFish[] fishInTheSea = {
			simpleJellyfish
		}


		// methods that are run on bony fish.


		// from a bonyfish struct, generate a serialized fish.
		// recreate a struct from a serialized fish.


		








