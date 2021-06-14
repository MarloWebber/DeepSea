/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
* Copyright (c) 2013 Google, Inc.
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#if !defined(__ANDROID__) && !defined(__IOS__)
#define ENABLE_GLUI 1
#endif  // !defined(__ANDROID__) && !defined(__IOS__)

#include "Render.h"
#include "Test.h"
// #include "Arrow.h"
// #include "FullscreenUI.h"
#include "ParticleParameter.h"
#if ENABLE_GLUI
#include "glui/glui.h"
#else
#include "GL/freeglut.h"
#endif  // ENABLE_GLUI
#include <stdio.h>
#include "AndroidUtil/AndroidLogPrint.h"
#include <algorithm>
#include <string>
#include <sstream>

#include "DeepSea.h"

namespace TestMain
{

namespace
{

	int currentlyPainting = 0;
	int  noClipStatus = 0;
	int originStartStatus = 0;
	// int persistentFoodStatus = 1;
	int triggerRadiusStatus = 1;
	int foodRadiusStatus = 1;
	int showBrainEditWindow = 0;
	int showBodyEditWindow = 0;
	int showSpeciesWindow = 0;
	int voting_mode = 0;
	int showFluidDynamicForces = 0;
	int entropyStatus = 0;
	int barrierRadiusStatus = 1;
	// int 

	int selectedSpeciesEnforcePopLimit = 0;
	int selectedSpeciesSexuality = 0;
	int autosaveSpeciesOnGeneration = 0;
	int selectNinSpecies = 1;

	GLUI_String speciesNameBarContent = "default";
	GLUI_String mapNameBarContent = "default";

	GLUI_EditText* speciesNameBar;
	GLUI_EditText* mapNameBar;

	GLUI_Spinner* nominalPopulationSpinner;

	GLUI_Spinner* barrierRadiusSpinner;

	b2Vec2 lower;
	b2Vec2 upper;

	int32 testIndex = 0;
	int32 testSelection = 0;
	int32 testCount = 0;
	TestEntry* entry;
	Test* test;
	Settings settings;
	int32 width = 640;
	int32 height = 540;
	int32 framePeriod = 16;
	int32 mainWindow;
	float settingsHz = 60.0;
#if ENABLE_GLUI
	GLUI *glui = NULL;
#endif  // ENABLE_GLUI
	float32 viewZoom = 1.0f;
	int tx, ty, tw, th;
	bool rMouseDown = false;

	// State of the mouse on the previous call of Mouse().
	int32 previousMouseState = -1;
	b2Vec2 lastp;
	b2Vec2 extents;

	// Fullscreen UI object.
	// FullscreenUI fullscreenUI;
	// Used to control the behavior of particle tests.
	ParticleParameter particleParameter;
}

GLUI_Spinner* getSpeciesNominalPopulationSpinner() {
	return nominalPopulationSpinner;
}

GLUI_EditText* getSpeciesNameBar() {
	return speciesNameBar;
}

GLUI_EditText* getMapNameBar() {
	return mapNameBar;
}

bool gameIsPaused() {
	return settings.pause;
}

int getBarrierRadiusStatus() {
	return barrierRadiusStatus;
}

int getPaintingStatus() {
	return currentlyPainting;
}

int getNoClipStatus() {
	return noClipStatus;
}

int getOriginStartStatus() {
	return originStartStatus;
}

b2Vec2 getUpperScreenBoundary() {
	return upper;
}
b2Vec2 getLowerScreenBoundary(){
	return lower;
}

// int getPersistentFoodStatus() {
// 	return persistentFoodStatus;
// }

int getBrainWindowStatus() {
	return showBrainEditWindow;
}

int getBodyWindowStatus() {
	return showBodyEditWindow;
}

int getVotingMode () {
	return voting_mode; // used for click-to-select.
}

int getFluidDynamicForcesViewStatus() {
	return showFluidDynamicForces;
}

int getTriggerRadiusStatus() {
	return triggerRadiusStatus;
}

int getFoodRadiusStatus() {
	return foodRadiusStatus;
}

int getSpeciesWindowStatus() {
	return showSpeciesWindow;
}


int getEntropyStatus() {
	return entropyStatus;
}

float32 getZoom () {
	return viewZoom;
}

int getNumberToSelect() {
	return selectNinSpecies;
}

// Set whether to restart the test on particle parameter changes.
// This parameter is re-enabled when the test changes.
void SetRestartOnParticleParameterChange(bool enable)
{
	particleParameter.SetRestartOnChange(enable);
}

// Set the currently selected particle parameter value.  This value must
// match one of the values in TestMain::k_particleTypes or one of the values
// referenced by particleParameterDef passed to SetParticleParameters().
uint32 SetParticleParameterValue(uint32 value)
{
	const int32 index = particleParameter.FindIndexByValue(value);
	// If the particle type isn't found, so fallback to the first entry in the
	// parameter.
	particleParameter.Set(index >= 0 ? index : 0);
	return particleParameter.GetValue();
}

// Get the currently selected particle parameter value and enable particle
// parameter selection arrows on Android.
uint32 GetParticleParameterValue()
{
	// Enable display of particle type selection arrows.
	// fullscreenUI.SetParticleParameterSelectionEnabled(true);
	return particleParameter.GetValue();
}

// Override the default particle parameters for the test.
void SetParticleParameters(
	const ParticleParameter::Definition * const particleParameterDef,
	const uint32 particleParameterDefCount)
{
	particleParameter.SetDefinition(particleParameterDef,
									particleParameterDefCount);
}

static void Resize(int32 w, int32 h)
{
	width = w;
	height = h;

#if ENABLE_GLUI
	GLUI_Master.get_viewport_area(&tx, &ty, &tw, &th);
#else
	tx = 0;
	ty = 0;
	tw = glutGet(GLUT_WINDOW_WIDTH);
	th = glutGet(GLUT_WINDOW_HEIGHT);
#endif  // ENABLE_GLUI
	glViewport(tx, ty, tw, th);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float32 ratio = th ? float32(tw) / float32(th) : 1;

	extents = ratio >= 1 ? b2Vec2(ratio * 25.0f, 25.0f) : b2Vec2(25.0f, 25.0f / ratio);
	extents *= viewZoom;

	lower = settings.viewCenter - extents;
	upper = settings.viewCenter + extents;

	// L/R/B/T
	LoadOrtho2DMatrix(lower.x, upper.x, lower.y, upper.y);


}

static b2Vec2 ConvertScreenToWorld(int32 x, int32 y)
{
	float32 u = x / float32(tw);
	float32 v = (th - y) / float32(th);

	 lower = settings.viewCenter - extents;
	 upper = settings.viewCenter + extents;

	b2Vec2 p;
	p.x = (1.0f - u) * lower.x + u * upper.x;
	p.y = (1.0f - v) * lower.y + v * upper.y;
	return p;
}

// This is used to control the frame rate (60Hz).
static void Timer(int)
{
	glutSetWindow(mainWindow);
	glutPostRedisplay();
	glutTimerFunc(framePeriod, Timer, 0);
}

 void PreStep()
{

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	settings.hz = settingsHz;

	// call this each frame, to function correctly with devices that may recreate
	// the GL Context without us asking for it
	Resize(width, height);
}

void Step () {
	test->Step(&settings);
}


void PostStep() {
	
	// test->DrawTitle(entry->name);

	glutSwapBuffers();
}

static void Keyboard(unsigned char key, int x, int y)
{
	B2_NOT_USED(x);
	B2_NOT_USED(y);

	switch (key)
	{
	case 27:
#ifndef __APPLE__
		// freeglut specific function
		glutLeaveMainLoop();
#endif
		exit(0);
		break;

		// Press 'z' to zoom out.
	case 'z':
		viewZoom = b2Min(1.1f * viewZoom, 20.0f);
		Resize(width, height);
		break;

		// Press 'x' to zoom in.
	case 'x':
		viewZoom = b2Max(0.9f * viewZoom, 0.02f);
		Resize(width, height);
		break;

	case '-':
		decrementSelectedConnection();
		;
		break;

	case '=':
		incrementSelectedConnection();
		break;

	case 'k':
		currentlySelectedLimb--;
		if (currentlySelectedLimb < 0) {
			currentlySelectedLimb = N_FINGERS-1;
		}
		break;

	case 'l':
		currentlySelectedLimb++;
		if (currentlySelectedLimb > N_FINGERS-1) {
			currentlySelectedLimb = 0;
		}
		break;

	default:
		if (test)
		{
			test->Keyboard(key);
		}
	}
}

static void KeyboardSpecial(int key, int x, int y)
{
	B2_NOT_USED(x);
	B2_NOT_USED(y);

	int mod = glutGetModifiers();

	switch (key)
	{
		// Press left to pan left.
	case GLUT_KEY_LEFT:
		if (mod == GLUT_ACTIVE_CTRL)
		{
			b2Vec2 newOrigin(2.0f, 0.0f);
			test->ShiftOrigin(newOrigin);
		}
		else
		{
			settings.viewCenter.x -= 0.5f;
			Resize(width, height);
		}
		break;

		// Press right to pan right.
	case GLUT_KEY_RIGHT:
		if (mod == GLUT_ACTIVE_CTRL)
		{
			b2Vec2 newOrigin(-2.0f, 0.0f);
			test->ShiftOrigin(newOrigin);
		}
		else
		{
			settings.viewCenter.x += 0.5f;
			Resize(width, height);
		}
		break;

		// Press down to pan down.
	case GLUT_KEY_DOWN:
		if (mod == GLUT_ACTIVE_CTRL)
		{
			b2Vec2 newOrigin(0.0f, 2.0f);
			test->ShiftOrigin(newOrigin);
		}
		else
		{
			settings.viewCenter.y -= 0.5f;
			Resize(width, height);
		}
		break;

		// Press up to pan up.
	case GLUT_KEY_UP:
		if (mod == GLUT_ACTIVE_CTRL)
		{
			b2Vec2 newOrigin(0.0f, -2.0f);
			test->ShiftOrigin(newOrigin);
		}
		else
		{
			settings.viewCenter.y += 0.5f;
			Resize(width, height);
		}
		break;

		// Press home to reset the view.
	case GLUT_KEY_HOME:
		viewZoom = 1.0f;
		settings.viewCenter.Set(0.0f, 20.0f);
		Resize(width, height);
		break;
	}
}

static void KeyboardUp(unsigned char key, int x, int y)
{
	B2_NOT_USED(x);
	B2_NOT_USED(y);

	if (test)
	{
		test->KeyboardUp(key);
	}
}

static void Mouse(int32 button, int32 state, int32 x, int32 y)
{
	// Use the mouse to move things around.
	if (button == GLUT_LEFT_BUTTON)
	{
		int mod = glutGetModifiers();
		b2Vec2 p = ConvertScreenToWorld(x, y);

		if (state == GLUT_DOWN)
		{
			b2Vec2 p = ConvertScreenToWorld(x, y);
			if (mod == GLUT_ACTIVE_SHIFT)
			{
				test->ShiftMouseDown(p);
			}
			else
			{
				test->MouseDown(p);
			}
		}

		if (state == GLUT_UP)
		{
			test->MouseUp(p);
		}
	}
	else if (button == GLUT_RIGHT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			lastp = ConvertScreenToWorld(x, y);
			rMouseDown = true;
		}

		if (state == GLUT_UP)
		{
			rMouseDown = false;
		}
	}
	previousMouseState = state;
}

static void MouseMotion(int32 x, int32 y)
{
	b2Vec2 p = ConvertScreenToWorld(x, y);

	test->MouseMove(p);
	
	if (rMouseDown)
	{
		b2Vec2 diff = p - lastp;
		settings.viewCenter.x -= diff.x;
		settings.viewCenter.y -= diff.y;
		Resize(width, height);
		lastp = ConvertScreenToWorld(x, y);
	}
}

#ifdef FREEGLUT
static void MouseWheel(int wheel, int direction, int x, int y)
{
	B2_NOT_USED(wheel);
	B2_NOT_USED(x);
	B2_NOT_USED(y);
	if (direction > 0)
	{
		viewZoom /= 1.1f;
	}
	else
	{
		viewZoom *= 1.1f;
	}
	Resize(width, height);
}
#endif

 void Pause2()
{
	settings.pause = true;
}
 void Resume2()
{
	settings.pause = false;
}


#if ENABLE_GLUI
static void Pause(int)
{
	settings.pause = !settings.pause;
}
#endif  // ENABLE_GLUI

}  // namespace TestMain


void menuHandler(int bobi) {
	// printf("feet %i\n", bobi);
}

void updateParticleDrawingKeyboard(int bobi) {
	ParticleDrawingKeyboard(m_deepSeaSettings.terrainPaintType);
}

void updateSpeciesList(int bobi) {

}


int main(int argc, char** argv)
{
	using namespace TestMain;

	testCount = 0;
	while (g_testEntries[testCount].createFcn != NULL)
	{
		++testCount;
	}

	testIndex = b2Clamp(testIndex, 0, testCount-1);
	testSelection = testIndex;

	entry = g_testEntries + testIndex;
	if (entry && entry->createFcn) {
		test = entry->createFcn();
		testSelection = testIndex;
		testIndex = -1;
	}

	glutInit(&argc, argv);
	glutInitContextVersion(2, 0);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(width, height);
	char title[32];
	sprintf(title, "DeepSea");
	mainWindow = glutCreateWindow(title);

	glutDisplayFunc(deepSeaLoop);


	deepSeaStart();



#if ENABLE_GLUI
	GLUI_Master.set_glutReshapeFunc(Resize);
	GLUI_Master.set_glutKeyboardFunc(Keyboard);
	GLUI_Master.set_glutSpecialFunc(KeyboardSpecial);
	GLUI_Master.set_glutMouseFunc(Mouse);
#else
	{
		glutReshapeFunc(Resize);
		glutKeyboardFunc(Keyboard);
		glutSpecialUpFunc(KeyboardSpecial);
		glutMouseFunc(Mouse);
	}
#endif  // ENABLE_GLUI

#ifdef FREEGLUT
	glutMouseWheelFunc(MouseWheel);
#endif
	glutMotionFunc(MouseMotion);

	glutKeyboardUpFunc(KeyboardUp);

#if ENABLE_GLUI
	glui = GLUI_Master.create_glui_subwindow( mainWindow,
		GLUI_SUBWINDOW_RIGHT );


	GLUI_Rollout* gamePanel =	glui->add_rollout("Game");
	gamePanel->close();

	// GLUI_Listbox* modeList =
		// glui->add_listbox_to_panel(gamePanel, "Mode", &(m_deepSeaSettings.gameMode ));
		// modeList->add_item(GAME_MODE_LABORATORY, "Laboratory Mode" );
		// modeList->add_item(GAME_MODE_ECOSYSTEM, "Ecosystem Mode" );

	glui->add_button_to_panel(gamePanel, "Pause", 0, Pause);

	glui->add_checkbox_to_panel(gamePanel, "Show fluid dynamic forces", &showFluidDynamicForces);



	GLUI_Rollout* laboratoryPanel =	glui->add_rollout("Laboratory");
	// laboratoryPanel->close();

	// GLUI_Spinner* numberOfFishSpinner =
		// glui->add_spinner_to_panel(laboratoryPanel,"Number of fish", GLUI_SPINNER_INT, &m_deepSeaSettings.laboratory_nFish);
	// numberOfFishSpinner->set_int_limits(1, 256);



	glui->add_checkbox_to_panel(laboratoryPanel, "Trigger generation at radius", &triggerRadiusStatus);
	
	GLUI_Spinner* triggerRadiusSpinner =
		glui->add_spinner_to_panel(laboratoryPanel, "Trigger radius", GLUI_SPINNER_FLOAT, &m_deepSeaSettings.originTriggerRadius);
	triggerRadiusSpinner->set_float_limits(0.0f, 100.0f);
	triggerRadiusSpinner->set_float_val(m_deepSeaSettings.originTriggerRadius);

	glui->add_separator_to_panel(laboratoryPanel);


	

	glui->add_checkbox_to_panel(laboratoryPanel, "Food walks around radius", &foodRadiusStatus);
	GLUI_Spinner* foodRadiusSpinner =
		glui->add_spinner_to_panel(laboratoryPanel, "Food walk radius", GLUI_SPINNER_FLOAT, &m_deepSeaSettings.originFoodRadius);
	foodRadiusSpinner->set_float_limits(0.0f, 100.0f);
	foodRadiusSpinner->set_float_val(m_deepSeaSettings.originFoodRadius);

	GLUI_Spinner* foodRadiusAngleJitterSpinner =
		glui->add_spinner_to_panel(laboratoryPanel, "Food angle jitter", GLUI_SPINNER_FLOAT, &m_deepSeaSettings.foodRadiusAngleJitter);
	foodRadiusAngleJitterSpinner->set_float_limits(0.0f, 2 * pi);
	foodRadiusAngleJitterSpinner->set_float_val(m_deepSeaSettings.foodRadiusAngleJitter);

	glui->add_button_to_panel(laboratoryPanel, "Add random food", 0, addRandomFoodParticle);


	glui->add_separator_to_panel(laboratoryPanel);



		glui->add_checkbox_to_panel(laboratoryPanel, "Barrier", &barrierRadiusStatus);
	
	// GLUI_Spinner* 
	barrierRadiusSpinner =
		glui->add_spinner_to_panel(laboratoryPanel, "Barrier radius", GLUI_SPINNER_FLOAT, &m_deepSeaSettings.barrierRadius);
	barrierRadiusSpinner->set_float_limits(0.0f, 1000.0f);
	glui->add_separator_to_panel(laboratoryPanel);




	glui->add_checkbox_to_panel(laboratoryPanel, "Reproduce species to origin", &m_deepSeaSettings.gameMode );

	glui->add_checkbox_to_panel(laboratoryPanel, "Autosave species on generation", &autosaveSpeciesOnGeneration);


	glui->add_separator_to_panel(laboratoryPanel);

	nominalPopulationSpinner =
	glui->add_spinner_to_panel(laboratoryPanel,"Select n", GLUI_SPINNER_INT, &selectNinSpecies);
	nominalPopulationSpinner->set_int_limits(1, 1000);




	glui->add_button_to_panel(laboratoryPanel, "Farthest from Zero", 2, selectFurthestFromOrigin);
	glui->add_button_to_panel(laboratoryPanel, "Closest to Food", 3, selectClosestToFood);
	glui->add_button_to_panel(laboratoryPanel, "Lowest Energy", 3, selectLowestEnergyFish);



	
	GLUI_Rollout* EcosystemPanel =	glui->add_rollout("Ecosystem Mode");
	EcosystemPanel->close();

	// int fakeNumberOfFood;
	// 	GLUI_Spinner* numberOfFoodSpinner =
	// 	glui->add_spinner_to_panel(EcosystemPanel, "Food particles", GLUI_SPINNER_INT, &fakeNumberOfFood);
	// numberOfFoodSpinner->set_int_limits(1, 8);



	// glui->add_checkbox_to_panel(EcosystemPanel, "Persistent food", &persistentFoodStatus);

	glui->add_checkbox_to_panel(EcosystemPanel, "Movement costs energy", &entropyStatus);



	

	GLUI_Rollout* speciesPanel =	glui->add_rollout("Taxonomy");
	speciesPanel->close();

	speciesNameBar = glui->add_edittext_to_panel(speciesPanel, "Species name: ", GLUI_EDITTEXT_TEXT, &speciesNameBarContent, 1, speciesNameBarCallback);
	speciesNameBar->set_text(speciesNameBarContent);
		
	glui->add_button_to_panel(speciesPanel, "Populate selected species from file", 0, populateSpeciesFromFile);
	glui->add_button_to_panel(speciesPanel, "Save selected individual to file", 1, saveIndividualToFile);

	glui->add_button_to_panel(speciesPanel, "Select all in species", 1, selectAllInSpecies);

	glui->add_checkbox_to_panel(speciesPanel, "Show species window", &showSpeciesWindow);

	int NominalPopulation;
	// GLUI_Spinner* n
	nominalPopulationSpinner =
	glui->add_spinner_to_panel(speciesPanel,"Nominal Population", GLUI_SPINNER_INT, &NominalPopulation, 1, nominalPopulationCallback);
	nominalPopulationSpinner->set_int_limits(1, 1000);

	glui->add_checkbox_to_panel(speciesPanel, "Enforce population limit", &selectedSpeciesEnforcePopLimit);

	glui->add_checkbox_to_panel(speciesPanel, "Sexual/Asexual", &selectedSpeciesSexuality);

	glui->add_button_to_panel(speciesPanel, "Add new species", 0, addNewSpecies);
	glui->add_button_to_panel(speciesPanel, "Delete selected species", 0, deleteSelectedSpecies);



	GLUI_Rollout* bioPanel =	glui->add_rollout("Biology");
	bioPanel->close();

	GLUI_Spinner* mutationRateSpinner =
		glui->add_spinner_to_panel(bioPanel, "Body Mutation Rate", GLUI_SPINNER_FLOAT, &m_deepSeaSettings.mutationRate);
	mutationRateSpinner->set_float_limits(0.0f, 1.0f);

	GLUI_Spinner* mutationSeveritySpinner =
		glui->add_spinner_to_panel(bioPanel, "Body Mutation Severity", GLUI_SPINNER_FLOAT, &m_deepSeaSettings.mutationSeverity);
	mutationSeveritySpinner->set_float_limits(0.0f, 100.0f);

	GLUI_Spinner* mentalMutationRateSpinner =
		glui->add_spinner_to_panel(bioPanel, "Mind Mutation Rate", GLUI_SPINNER_FLOAT,&m_deepSeaSettings.mentalMutationRate);
	mentalMutationRateSpinner->set_float_limits(0.0f, 1.0f);

	GLUI_Spinner* mentalMutationSeveritySpinner =
		glui->add_spinner_to_panel(bioPanel, "Mind Mutation Severity", GLUI_SPINNER_FLOAT, &m_deepSeaSettings.mentalMutationSeverity);
	mentalMutationSeveritySpinner->set_float_limits(0.0f, 100.0f);




	GLUI_Rollout* selectionPanel =	glui->add_rollout("Cloning Controls");
	selectionPanel->open();

	// glui->add_button_to_panel(selectionPanel, "Farthest Traveled", 1, selectFishWhoMovedTheFurthest);
	

	// glui->add_separator_to_panel(selectionPanel);

	glui->add_button_to_panel(selectionPanel, "Deselect All", 4, deselectAll);
	glui->add_button_to_panel(selectionPanel, "Select All", 5, selectAll);
	glui->add_button_to_panel(selectionPanel, "Invert Selection", 6, invertSelection);

	glui->add_separator_to_panel(selectionPanel);

	// cloning controls
	glui->add_button_to_panel(selectionPanel, "Delete Selected", 2, flagSelectedFishForDeletion);
	glui->add_separator_to_panel(selectionPanel);

	glui->add_button_to_panel(selectionPanel, "Reproduce 1 Selected", 3, handleReproduceSelectedButton);
	glui->add_button_to_panel(selectionPanel, "Mate 2 Selected", 3, mateSelectedFish);
	glui->add_separator_to_panel(selectionPanel);

	// glui->add_button_to_panel(bioPanel, "Re-run generation", 4, reloadTheSim);

	// glui->add_separator_to_panel(bioPanel);

	glui->add_checkbox_to_panel(selectionPanel, "Select with LMB", &voting_mode);

	



	GLUI_Rollout* brainEditPanel =	glui->add_rollout("Neuroscience");
	brainEditPanel->close();

	glui->add_button_to_panel(brainEditPanel, "Neutralize Brain", 9, meltSelectedFish);
	glui->add_button_to_panel(brainEditPanel, "Randomize Brain", 10, scrambleSelectedFish);

	glui->add_button_to_panel(brainEditPanel, "Delete Selected Neuron", 11, deleteSelectedNeuron);
	glui->add_button_to_panel(brainEditPanel, "Delete Sense Connector", 11, deleteSenseConnector);

	glui->add_button_to_panel(brainEditPanel, "Add Recursor Pair", 12, addRecursorPair);













	glui->add_button_to_panel(brainEditPanel, "Add neuron in selected layer", 13, addNeuronInSelectedLayer);

	glui->add_button_to_panel(brainEditPanel, "Add layer", 14, addLayerToSelectedFish);

	glui->add_button_to_panel(brainEditPanel, "Delete selected layer", 15, deleteSelectedLayer);

	glui->add_checkbox_to_panel(brainEditPanel, "Show brain edit window", &showBrainEditWindow);



	GLUI_Rollout* bodyEditPanel =	glui->add_rollout("Surgery");
	bodyEditPanel->close();

	glui->add_button_to_panel(bodyEditPanel, "Pin to Grid", 5, pinToGrid);
	glui->add_button_to_panel(bodyEditPanel, "Release", 6, releaseFromGrid);

	glui->add_button_to_panel(bodyEditPanel, "Attach Limb", 7, placeLimbOnSelectedFish);
	glui->add_button_to_panel(bodyEditPanel, "Amputate Limb", 8, amputation);


 	glui->add_button_to_panel(bodyEditPanel, "Selected limb eye", 9, selectedLimbEye);
 	glui->add_button_to_panel(bodyEditPanel, "Selected limb foodradar", 10, selectedLimbFoodradar);
 	glui->add_button_to_panel(bodyEditPanel, "Selected limb altradar", 11, selectedLimbAltradar);
 	// glui->add_button_to_panel(bodyEditPanel, "Selected limb motor", 12, selectedLimbMotor);

	glui->add_checkbox_to_panel(bodyEditPanel, "Show body edit window", &showBodyEditWindow);




	GLUI_Rollout* terrainPanel =	glui->add_rollout("Habitat");
	terrainPanel->close();




	mapNameBar = glui->add_edittext_to_panel(terrainPanel, "Map name: ", GLUI_EDITTEXT_TEXT, &mapNameBarContent, 1, mapNameBarCallback);
	mapNameBar->set_text(mapNameBarContent);
	
	glui->add_button_to_panel(terrainPanel, "Load saved map from file", 0, loadSavedMapFromFile);
	glui->add_button_to_panel(terrainPanel, "Save current map to file", 1, saveCurrentMapToFile);
	
	glui->add_checkbox_to_panel(terrainPanel, "Enable terrain paint", &currentlyPainting);


	GLUI_Listbox* terrainTypesList =
		glui->add_listbox_to_panel(terrainPanel, "Terrain Type ", &(m_deepSeaSettings.terrainPaintType ) ,-1, updateParticleDrawingKeyboard);

		terrainTypesList->add_item(0, "Elastic | SolidGroup" );
		terrainTypesList->add_item(1, "Powder" );
		terrainTypesList->add_item(2, "Rigid | SolidGroup" );
		terrainTypesList->add_item(3, "Spring | SolidGroup" );
		terrainTypesList->add_item(4, "Tensile" );
		terrainTypesList->add_item(5, "Viscous" );
		terrainTypesList->add_item(6, "Wall | SolidGroup" );
		terrainTypesList->add_item(7, "Barrier | WallParticle" );
		terrainTypesList->add_item(8, "Barrier | RigidGroup" );
		terrainTypesList->add_item(9, "Barrier | ElasticParticle | SolidGroup" );
		terrainTypesList->add_item(10, "Barrier | SpringParticle | SolidGroup" );
		terrainTypesList->add_item(11, "Wall | RepulsiveParticle" );
		terrainTypesList->add_item(12, "ColorMixing" );
		terrainTypesList->add_item(12, "Zombie" );
		ParticleDrawingKeyboard(0);



	glutCreateMenu(menuHandler);

	glui->set_main_gfx_window( mainWindow );

#endif  // ENABLE_GLUI

	// Configure the fullscreen UI's viewport parameters.
	// fullscreenUI.SetViewParameters(&settings.viewCenter, &extents);


	// Use a timer to control the frame rate.
	glutTimerFunc(framePeriod, Timer, 0);

	test_runAllUnitTests();


	glutMainLoop();

	return 0;
}
