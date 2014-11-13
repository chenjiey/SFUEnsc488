/****************************************************************************
                  (ENSC488 - PHANTOM Haptic Device Sample Code)

Module Name: main.cpp

Original Author: SensAble Technologies (c) 2004

Modified and Commented by:  Wayne Chen (waynec@sfu.ca)

Last Update: 09/24/2004

Description:

  This sample code introduces the use of basic Phantom haptic device
  operations, including initialization, setting forces, getting data,
  and termination.

  This program runs in a "multi-threaded" (multi-processing) fashion. 
  Within an asynchronous callback, one process reads the stylus tip
  position and sets the force to the device.  Within a synchronous
  callback, another process gets the stylus tip position, orientation, forces, and 
  button status, and the scene is redrawn accordingly..

  This program has the following features:
  - Two spheres, which represent a positive and negative charges, are drawn 
    on the screen.
  - The user moves/orients one of the spheres using Phantom's stylus.
  - The user feels the attraction forces between the two spheres.
  - An arrow is drawn to show the magnitude and direction of the attraction force.
  - The user can rotate/scale the other sphere by dragging the LEFT/MIDDLE
    buttons of the mouse.
  - RIGHT-clicking the mouse brings up a popup menu.  Currently two items
    are in the menu: "How to Play" and "About"

******************************************************************************/

//*****************************************************************************
//                INCLUDED HEADER FILES       
//*****************************************************************************
#include <iostream>
#include <stdio.h>
#include <conio.h>
#include <assert.h>

#include <GL/glut.h>            //needed for graphics (OpenGL)
#include <HDU/hduVector.h>      //useful utility (vector)
#include <HDU/hduMath.h>
#include <HDU/hduMatrix.h>      //useful utility (matrix)
#include <HD/hd.h>              //needed for haptic device (general)
#include <HDU/hduError.h>       //needed for haptic device (error handling)


//*****************************************************************************
//                GLOBAL CONSTANTS
//*****************************************************************************
#define SPHERE_RADIUS   12      //the initial radius of the two spheres to be drawn
#define CUBE_SIZE 150
#define SPHERE_MASS 5
//the colours of the axes to be drawn
//the columns are the colour vector (R, G, B, transparency).
const float AXIS_COLOUR[ 4 ][ 3 ] = 
{ 
    { 1, 0, 0 },
    { 0, 1, 0 },
    { 0, 0, 1 },
    { 1, 1, 1 }
};

//the vertices (endpoints) of the axes to be drawn
const float AXIS_VERTEX[ 4 ][ 3 ] = 
{          
    { SPHERE_RADIUS*2, 0, 0 },
    { 0, SPHERE_RADIUS*2, 0 },
    { 0, 0, SPHERE_RADIUS*2 },
    { 0, 0, 0 }
};

//*****************************************************************************
//                USER-DEFINED DATA STRUCTURES
//*****************************************************************************
//storage for recording current data from the haptic device
struct HapticDeviceState
{
    HHD m_hHD;              //handle of the haptic device
    HDint button;           //button status: 0 (no press); 1 (blue button); 2 (white button)
    double position[3];     //position of the stylus tip
    double transform_matrix[16];    //transformation (position & orientation)
                                    // of the stylus tip
    double force[3];        //output force vector (X,Y,Z) of the haptic device
};


//*****************************************************************************
//                GLOBAL VARIABLES
//*****************************************************************************
//for the mouse
int gLastMouseX, gLastMouseY;   //mouse position at previous time stamp
const int MAXTRIANGLES  =   20; //max triabgles for polygon array
int i,j;                    // Variable User as counter in the Loops
double identityTransform[16] = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
double spherePosition[3] = {0,0,0};
double contactPoint[3] = {0,0,0};
double offsetSphere[3] = {0,0,0};
bool gettingBallPosition = true;
bool ballAttached = false;
//double wallForce[3] = {0,0,0};

//Camera Attributes (Rotation/Scaling on the centre sphere)
double CamRotationY = 0;            //rotation (degrees)
double CamRotationX = 0;
double CamZoom = 1;                 //scale factor
double sphereMass = 1;
bool gIsRotatingCamera = false;     //flags to indicate which operation to perform
bool gIsScalingCamera = false;      // according to different mouse clicks.
bool gIsTranslatingCamera = false;

//for the haptic device
HHD ghHD = HD_INVALID_HANDLE;   //handle of the device
HDSchedulerHandle gSchedulerCallback = HD_INVALID_HANDLE;   //handle of the scheduler

//*****************************************************************************
//                USER-DEFINED CLASS
//*****************************************************************************

//*****************************************************************************
//                UTILITY FUNCTION PROTOTYPES
//*****************************************************************************

//=====================================================================
//           FUNCTIONS FOR INITIALIZATION (OF GLUT) and TERMINATION
//=====================================================================

//This procedure intializes the GLUT setting.  It creates the graphics window
// and assigns functions for display, mouse click, mouse movement, and popup menu.
void initGlut(int argc, char* argv[]);

//This procedure detects which mouse button is clicked and sets the
// appropriate flags to indicate which event should occur.
void MyGlutMouse(int button, int state, int x, int y);

//This procedure detects the motion of the mouse (i.e. dragging"),
// and performs operations depending on how the mouse is dragged
// and which flag is set.
void MyGlutMotion(int x, int y);

//This function takes care of operations to perform when the program is idle.
void MyGlutIdle(void);

//This procedure sets up the functionalities of the menu items of the popup
// menus when a mouse button is clicked.
//For this example, both menu items display certain texts.
void MyGlutMenu(int ID);

// This handler gets automatically called when the program is exiting. 
//  The program ensures that the haptic device and the scheduler are
//  properly shutdown
void __cdecl exitHandler();


//=====================================================================
//     <HAPTICS>:FUNCTIONS RELATED TO HAPTIC DEVICE INTERACTION and FORCES
//=====================================================================

//This procedure sends to the scheduler a task (i.e. a callback function)
// that updates the force feedback of the device continuously.
void ScheduleForceFeedback();

//This is the callback function calculates the force (by calling 
// "CalculateForce()" and SET the resulting forces to the device.
//The callback function is scheduled to the scheduler in an "asynchronous"
// fashion, and the function is called repeatedly each time it finishes.
HDCallbackCode HDCALLBACK SettingForceCallback(void *data);

//This callback function has the responsibility of GETTING haptic device data that is
// constantly modified by the device. 
//In the main graphics loop function, this callback function is scheduled 
// to the scheduler in a "synchronous" fashion.  
HDCallbackCode HDCALLBACK GettingDeviceStateCallback(void *pUserData);

//This function calculates the force vector to be sent to the haptic device.
//Currently, the force is calculated based on the current position of the 
// device cursor and Coulomb's Law.
hduVector3Dd CalculateForce(double* spherePosition);

//=====================================================================
//    <GRAPHICS>: FUNCTIONS RELATED TO SETTING UP/DRAWING THE SCENE
//=====================================================================

//This procedure sets up the viewing condition for the scene to be drawn.
//This procedure uses the haptic device coordinate space as model space 
// of our scene.  An orthographic projection is defined to fit this space.
//NOTE: LLB means Low, Left, Back point of device workspace.
//      TRF means Top, Right, Front point of device workspace. 
void initGraphicsViewing(const HDdouble LLB[3], const HDdouble TRF[3]);

//This procedure sets up the lighting (and shading) conditions 
// for the scene to be drawn.
void initGraphicsLighting();

//This is the main display looping function that is run and draws the 
// screen continuously.
//Currently this function draws one fixed sphere (with axes), one movable
// sphere (with axes), and an arrow (representing the force).
void MyGlutDisplay(void);

//This procedure draws the X,Y,Z axis for the current coordinate frame.
void drawAxes();

//This procedure draws a sphere with radius "SPHERE_RADIUS" centred at the origin
// of the current coordinate frame.
void drawFixedSphere(GLUquadricObj* quadObj);

//This procedure draws the "movable sphere" that corresponds to the cursor
// of the haptic device.
void drawMovableSphere(GLUquadricObj* quadObj,
                       const double transform[16],
                       HDint button_state);

//This procedure draws the visual representation (an arrow) that represents
// the magnitude and direction of the Coulomb Force.
void drawForceVisualRepresentation(GLUquadricObj* quadObj,
                                   const double position[3],                       
                                   const double strength);


void drawball(GLUquadricObj* quadObj, HapticDeviceState state);
//void drawHollowCube();
//*****************************************************************************
//                THE MAIN FUNCTION - (this is where things start...)
//*****************************************************************************
int main(int argc, char* argv[])
{
    HDErrorInfo error;

    printf("ENSC488 - Haptic Device Sample Program\n\n");
    printf("Starting application\n");
    
    //Set up the TERMINATION procedures when program finishes.
    //Things to take care of including shutting down the haptic device and scheduler.
    atexit(exitHandler);

    //Initialize the haptic device.  This needs to be called before any actions on 
    // the device.
    ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
    //Check for any initialization error.  If so, terminate the program.
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }

    //Retrieve and display the model name of the haptic device.
    printf("Found device %s\n",hdGetString(HD_DEVICE_MODEL_TYPE));
    
    //Enable haptic force feedback
    hdEnable(HD_FORCE_OUTPUT);
    hdEnable(HD_MAX_FORCE_CLAMPING);

    //Find the workspace of the robotic links of the haptic device.
    //The workspace is defined by two vertices: the Low Left Back point
    // and the Top Right Front point.
    printf("The workspace two corner vertices are:\n");
    HDdouble maxWorkspace[6];
    hdGetDoublev(HD_MAX_WORKSPACE_DIMENSIONS, maxWorkspace);
    printf("%lf %lf %lf %lf %lf %lf\n", maxWorkspace[0],
           maxWorkspace[1],
           maxWorkspace[2],
           maxWorkspace[3],
           maxWorkspace[4],
           maxWorkspace[5]);
    printf("\n");

    //Start the scheduler of the haptic device.
    hdStartScheduler();
    //Check if the scheduler is initialized properly.
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }
    
    //Initializes all GLUT related functions.
    initGlut(argc, argv);

    //Based on the workspace of the haptic device,
    // set the viewing volumn of the scene to correspond
    // to this workspace.
    //Low, Left, Back point of device workspace.
    hduVector3Dd LLB; 
    LLB.set(maxWorkspace[0], maxWorkspace[1], maxWorkspace[2]);
    //Top, Right, Front point of device workspace.
    hduVector3Dd TRF;
    TRF.set(maxWorkspace[3], maxWorkspace[4], maxWorkspace[5]);
    //Initializes the viewing condition for the scene
    initGraphicsViewing(LLB, TRF);

    //Initializes the lighting condition for the scene
    initGraphicsLighting();


    //Schedule the force feedback process to the scheduler
    std::cout << "Starting haptics callback..." << std::endl;
    ScheduleForceFeedback();

    //Enter the main loop for drawing the scene
    std::cout << "Starting graphics callback..." << std::endl;
    glutMainLoop(); //This is the "famous" GLUT infinite loop
                    // that runs forever unless the program exits.

    printf("Done\n");
    return 0;
}

/******************************************************************************/


//=====================================================================
//           FUNCTIONS FOR INITIALIZATION (OF GLUT) and TERMINATION
//=====================================================================

//This procedure intializes the GLUT setting.  It creates the graphics window
// and assigns functions for display, mouse click, mouse movement, and popup menu.
void initGlut(int argc, char* argv[])
{
    glutInit(&argc, argv);          // Initialize GLUT.
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(500, 500);   //graphics window creation
    glutCreateWindow("Haptic Device Demo - Coulomb Force");

    glutDisplayFunc(MyGlutDisplay);   //GLUT callback - display or draw scene
    //glutReshapeFunc(reshape);
    glutMouseFunc(MyGlutMouse);       //GLUT callback - mouse button detection
    glutMotionFunc(MyGlutMotion);     //GLUT callback - mouse button movement
    glutIdleFunc(MyGlutIdle);         //GLUT callback - idle operations

    glutCreateMenu(MyGlutMenu);       //GLUT callback - Setup GLUT popup menu
    glutAddMenuEntry("How to Play", 0);
	glutAddMenuEntry("Increase Sphere Mass", 1);
	glutAddMenuEntry("Decrese Sphere Mass", 2);
    glutAddMenuEntry("About", 3);
    glutAttachMenu(GLUT_RIGHT_BUTTON);//Right click the mouse to launch the popup menu

}//END of initGlut


//This procedure detects which mouse button is clicked and sets the
// appropriate flags to indicate which event should occur.
void MyGlutMouse(int button, int state, int x, int y)
{
    if (state == GLUT_DOWN)
    {   //some mouse button is pressed
        if (button == GLUT_LEFT_BUTTON)
        {   //Left click the mouse to rotate the centre sphere.
            gIsRotatingCamera = true;            
        }
        else if (button == GLUT_MIDDLE_BUTTON)
        {   //Click the centre button to scale the centre sphere.
            gIsScalingCamera = true;            
        }

        //record the current mouse PIXEL location
        gLastMouseX = x;
        gLastMouseY = y;
    }
    else
    {   //no mouse button is pressed...
        gIsRotatingCamera = false;
        gIsScalingCamera = false;
        gIsTranslatingCamera = false;
    }
}//END of MyGlutMouse


//This procedure detects the motion of the mouse (i.e. dragging"),
// and performs operations depending on how the mouse is dragged
// and which flag is set.
void MyGlutMotion(int x, int y)
{
    if (gIsRotatingCamera)
    {   
        //rotate the centre sphere
        //dragging the mouse horizontally rotates the object about Y axis
        CamRotationY = CamRotationY + 0.1*(x - gLastMouseX);
        //dragging the mouse vertically rotates the object about X axis
        CamRotationX = CamRotationX + 0.1*(y - gLastMouseY);
    }
    else if (gIsScalingCamera)
    {
        //dragging the mouse vertically scales the object up or down.
        CamZoom = CamZoom - 0.01*(y - gLastMouseY);
    }

    //record the current mouse PIXEL position.
    gLastMouseX = x;
    gLastMouseY = y;
}//END of MyGlutMotion


//This function takes care of operations to perform when the program is idle.
void MyGlutIdle(void)
{
    //redisplay the scene
    glutPostRedisplay();

    //check if the scheduler has exited... if so, terminate program as well.
    if (!hdWaitForCompletion(gSchedulerCallback, HD_WAIT_CHECK_STATUS))
    {
        printf("The main scheduler callback has exited\n");
        printf("Press any key to quit.\n");
        getch();
        exit(-1);
    }
}//END of MyGlutIdle


//This procedure sets up the functionalities of the menu items of the popup
// menus when a mouse button is clicked.
//For this example, both menu items display certain texts.
void MyGlutMenu(int ID)
{
    switch(ID) 
    {
        case 0: //"How to Play" information in the popup menu.
            
            break;
        case 1: //Increase mass
            sphereMass ++;
            break;
		case 2: //Decrease mass
            sphereMass --;
			if (sphereMass < 0)
				sphereMass = 0;
			break;

		case 3: //"About" information in the popup menu
            
        break;
    }
}//END of MyGlutMenu      


// This handler gets automatically called when the program is exiting. 
//  The program ensures that the haptic device and the scheduler are
//  properly shutdown
void __cdecl exitHandler()
{
    //stop the scheduler and cleans up any unfinished jobs on the scheduler.
    hdStopScheduler();
    hdUnschedule(gSchedulerCallback);

    //if the haptic device hasn't been disabled yet, disable it now.
    if (ghHD != HD_INVALID_HANDLE)
    {
        hdDisableDevice(ghHD);
        ghHD = HD_INVALID_HANDLE;
    }
}//END of exitHandler



//=====================================================================
//   <HAPTICS>: FUNCTIONS RELATED TO HAPTIC DEVICE INTERACTION and FORCES
//=====================================================================

//This procedure sends to the scheduler a task (i.e. a callback function)
// that updates the force feedback of the device continuously.
void ScheduleForceFeedback()
{
    //schedule asynchronously to the scheduler a process for setting forces.
    gSchedulerCallback = hdScheduleAsynchronous(
        SettingForceCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

    //checks if the process is scheduled successfully
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        exit(-1);
    }

}//END of ScheduleForceFeedback



//This is the callback function calculates the force (by calling 
// "CalculateForce()" and SET the resulting forces to the device.
//The callback function is scheduled to the scheduler in an "asynchronous"
// fashion, and the function is called repeatedly each time it finishes.
HDCallbackCode HDCALLBACK SettingForceCallback(void *data)
{
    //get a "handle" on the current haptic device
    HHD hHD = hdGetCurrentDevice();
	hduVector3Dd forceVec;
    //NOTE: Setting forces must be in between the "hdBeginFrame()"
    // and "hdEndFrame()".  Between these two lines, the haptic status
    // (forces) is constant.
    hdBeginFrame(hHD);

    //Obtain the current position of the tip of the stylus
    hduVector3Dd pos;
    hdGetDoublev(HD_CURRENT_POSITION,pos);
    forceVec = CalculateForce(spherePosition);
    hdSetDoublev(HD_CURRENT_FORCE, forceVec);
        
    hdEndFrame(hHD);

    //Check if the scheduler returns any error when executing this process...
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Error during scheduler callback");

        if (hduIsSchedulerError(&error))
        {
            //if there is error in scheduler, terminates (by force) the current
            // process
            return HD_CALLBACK_DONE;
        }
    }

    //continue executing the setting up of the force feedback
    // by telling the scheduler to repeat on the process after it is completed
    // each time.
    //NOTE: this is how we can have a process to set up device forces that runs
    // in parallel with other (graphics, getting data) processes.
    return HD_CALLBACK_CONTINUE;
}//END of SettingForceCallback


//This callback function has the responsibility of GETTING haptic device data that is
// constantly modified by the device. 
//In the main graphics loop function, this callback function is scheduled 
// to the scheduler in a "synchronous" fashion.  
HDCallbackCode HDCALLBACK GettingDeviceStateCallback(void *pUserData)
{
    //rename the variable and type cast it as the user
    // defined data structure "HapticDeviceState"
    HapticDeviceState *pDisplayState = 
        static_cast<HapticDeviceState *>(pUserData);

    //Get current stylus tip position
    hdGetDoublev(HD_CURRENT_POSITION, pDisplayState->position);
    //Get the overal transformation matrix for the stylus tip.
    hdGetDoublev(HD_CURRENT_TRANSFORM, pDisplayState->transform_matrix);
    //Get the force vector that is currently set in the haptic device
    hdGetDoublev(HD_CURRENT_FORCE, pDisplayState->force);
    //Get the info on which button of the stylus is pressed.
    hdGetIntegerv(HD_CURRENT_BUTTONS, &pDisplayState->button);

    //Execute this only once - since we are already doing it repeatedly (inside
    // the GlutDisplay function)
    return HD_CALLBACK_DONE;

}//END of GettingDeviceStateCallback



//This function calculates the force vector to be sent to the haptic device.
//Currently, the force is calculated based on the current position of the 
// device cursor and Coulomb's Law.
hduVector3Dd CalculateForce(double* spherePosition)
{
	hduVector3Dd forceVec;
	//Calculating wall force
	if (ballAttached){
		for( int i = 0; i < 3; i++ ){
			if((abs(spherePosition[i]) + 2*SPHERE_RADIUS) >= CUBE_SIZE/2) {
				if(spherePosition[i] > 0) 
					forceVec[i] = -10;
				else {
					forceVec[i] = 10;
				}
				//std::cout << "feel the force" << forceVec[i] << "\n";
			}
	
		}

		//Add force for gravity
		forceVec[1] += - sphereMass; 
		//std::cout << "feel the gravity" << forceVec[2] << "\n";
		return forceVec;
	}
	forceVec[0] = 0;
	forceVec[1] = 0;
	forceVec[2] = 0;

    return forceVec;
}//END of CalculateForce




//=====================================================================
//    <GRAPHICS>: FUNCTIONS RELATED TO SETTING UP/DRAWING THE SCENE
//=====================================================================

//This procedure sets up the viewing condition for the scene to be drawn.
//This procedure uses the haptic device coordinate space as model space 
// of our scene.  An orthographic projection is defined to fit this space.
//NOTE: LLB means Low, Left, Back point of device workspace.
//      TRF means Top, Right, Front point of device workspace. 
void initGraphicsViewing(const HDdouble LLB[3], const HDdouble TRF[3])
{
    glMatrixMode(GL_PROJECTION); /* Setup perspective projection. */
    glLoadIdentity();

    //Find the (Cartesian) centre point of the device workspace.
    HDdouble centerScreen[3];
    centerScreen[0] = (TRF[0] + LLB[0])/2.0;
    centerScreen[1] = (TRF[1] + LLB[1])/2.0;
    centerScreen[2] = (TRF[2] + LLB[2])/2.0;

    //According to the haptic device workspace, set the view volumn 
    // (screen dimension).
    HDdouble screenDims[3];
    screenDims[0] = TRF[0] - LLB[0];
    screenDims[1] = TRF[1] - LLB[1];
    screenDims[2] = TRF[2] - LLB[2];

    //find the dimension corresponding to the larger "range" 
    // between the X and Y directions
    HDdouble maxDimXY = (screenDims[0] > screenDims[1] ? screenDims[0]:screenDims[1]);
    //find the dimension corresponding to the larger "range" 
    // among the X, Y, and Z directions
    HDdouble maxDim = (maxDimXY > screenDims[2] ? maxDimXY:screenDims[2]);
    maxDim /= 2.0;

    //orthographic projection (1st parameter has largest "range", and so on...)
    glOrtho(centerScreen[0]-maxDim, centerScreen[0]+maxDim, 
             centerScreen[1]-maxDim, centerScreen[1]+maxDim,
             centerScreen[2]-maxDim, centerScreen[2]+maxDim);
    
    //for testing only...
//    printf("glortho %lf %lf %lf %lf %lf %lf\n",
//           centerScreen[0]-maxDim, centerScreen[0]+maxDim, 
//           centerScreen[1]-maxDim, centerScreen[1]+maxDim,
//           centerScreen[2]-maxDim, centerScreen[2]+maxDim);

 
    glMatrixMode(GL_MODELVIEW); // Setup model transformations. 
    glLoadIdentity();

    glClearDepth(1.0); // Setup background colour. 
    glClearColor(0.7, 0.7, 0.7, 0);
    glDisable(GL_DEPTH_TEST);

}//END of initGraphicsWorkspace


//This procedure sets up the lighting (and shading) conditions 
// for the scene to be drawn.
void initGraphicsLighting()
{
    //smooth (interpolated) shading
    glShadeModel(GL_SMOOTH);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHT_MODEL_TWO_SIDE);
    
    GLfloat lightZeroPosition[] = {10.0, 4.0, 100.0, 0.0};
    GLfloat lightZeroColor[] = {0.6, 0.6, 0.6, 1.0}; //grey light
    GLfloat lightOnePosition[] = {-1.0, -2.0, -100.0, 0.0};
    GLfloat lightOneColor[] = {0.6, 0.6, 0.6, 1.0};  //grey light as well
    
    GLfloat light_ambient[] = {0.8, 0.8, 0.8, 1.0}; //Ambient lighting (grey)
    GLfloat light_diffuse[] = {0.0, 0.0, 0.0, 1.0}; //Diffuse lighting (none)    
    GLfloat light_position[] = {0.0, 0.0, 100.0, 1.0}; //light source far away
    
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, 1);
    glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);
    glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightOneColor);
    glEnable(GL_LIGHT0);    //enable light0
    glEnable(GL_LIGHT1);    //enable light1

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

}//END of initGraphicsLighting


//This is the main display looping function that is run and draws the 
// screen continuously.
//Currently this function draws one fixed sphere (with axes), one movable
// sphere (with axes), and an arrow (representing the force).
void MyGlutDisplay(void)
{
    glMatrixMode(GL_MODELVIEW); // Setup model transformations.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();
	glRotatef(15,1,1,1);
	glRotatef(CamRotationY, 0, 1, 0);
	glRotatef(CamRotationX, 1, 0, 0);
	glScalef(CamZoom, CamZoom, CamZoom);
    glPushMatrix();
    //need to enable "colour material" in order to see the
    // different surfaces of objects having different colours (shades)
    glEnable(GL_COLOR_MATERIAL);

    drawAxes();
	glutWireCube(CUBE_SIZE);
	glPopMatrix();
   // drawHollowCube();
    //Draw the end effector (sphere) and the arrow
    // Get the current position/orientation of end effector and
    // the current button state.
    //A process for getting device data is sent to the scheduler
    // (synchronously).
    HapticDeviceState state;
    hdScheduleSynchronous(GettingDeviceStateCallback, &state,
                          HD_MIN_SCHEDULER_PRIORITY);
    GLUquadricObj* quadObj = gluNewQuadric();

   //double forceMag = 400.0 * sqrt(state.force[0]*state.force[0] + 
     //                             state.force[1]*state.force[1] + 
       //                          state.force[2]*state.force[2]);
    
    //draw ball
    drawball(quadObj, state);

    //draw the sphere (tip of the stylus)
    drawMovableSphere(quadObj, state.transform_matrix, state.button);
    //draw the force arrow
    //drawForceVisualRepresentation(quadObj, state.position, forceMag);

    // Always delete a Quadric (since you previouly dynamically allocated one)
    gluDeleteQuadric(quadObj);

    glDisable(GL_COLOR_MATERIAL);
  
    glPopMatrix();

    // Double buffers are used to speed things up...
    glutSwapBuffers();
}//END of MyGlutDisplay


//--------------------------------------------------------
// *** NOTE: The following are the utility functions that
//   draw the individual objects in the scene. ***
//--------------------------------------------------------

//This procedure draws the X,Y,Z axis for the current coordinate frame.
void drawAxes()
{
    //disable lighting and material when drawing axes.
    glDisable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);
    for(int i = 0; i < 3; i++) 
    {               
        glColor3fv(AXIS_COLOUR[ i ]);
        glBegin(GL_LINES);  //draw a line - define two endpoints between
                            // the "glBegin" and "glEnd" pair
        glVertex3fv(AXIS_VERTEX[ i ]);
        glVertex3fv(AXIS_VERTEX[ 3 ]);
        glEnd();
    }
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);

}//END of drawAxes



//This procedure draws a sphere with radius "SPHERE_RADIUS" centred at the origin
// of the current coordinate frame.
void drawFixedSphere(GLUquadricObj* quadObj)
{
    //Display a sphere to show the static electric charge (greenish blue).
    glColor4f(0.2, 0.8, 0.8, 0.8);

    //Draw the center sphere.
    gluSphere(quadObj, SPHERE_RADIUS, 20, 20); 

}//END of drawFixedSphere


//This procedure draws the "movable sphere" that corresponds to the cursor
// of the haptic device.
void drawMovableSphere(GLUquadricObj* quadObj,
                       const double transform[16],
                       HDint button_state)

{
   //Display one sphere to represent the haptic cursor and the dynamic 
   //    charge.
    glPushMatrix();

    glLoadIdentity();
    glMultMatrixd(transform);   //transform the movable sphere
                                // such that its location and orientation
                                // correponds to those of the stylus of the device.

    drawAxes();
    if (button_state == 0)  //no button is pressed
        glColor4f(0.8, 0.2, 0.2, 0.8);      //red colour
    else if (button_state == 1) //the first (blue) button is pressed
        glColor4f(0.2, 0.8, 0.2, 0.8);     //green colour
    else if (button_state == 2) //the second (white) button is pressed
        glColor4f(0.2, 0.2, 0.8, 0.8);      //blue colour

    gluSphere(quadObj, SPHERE_RADIUS, 20, 20);  

    glPopMatrix();

}//END of drawMovableSphere


//This procedure draws the visual representation (an arrow) that represents
// the magnitude and direction of the Coulomb Force.
void drawForceVisualRepresentation(GLUquadricObj* quadObj,
                                   const double position[3],                       
                                   const double strength)
{
    //Display the force vector, change the force magnitude/direction
    // rotote the force vector first. 
    hduVector3Dd ForceVectorAxis(position); 
    ForceVectorAxis *= -1;  //points to the opposite direction of "position"

    //normalize the force vector such that its length is 1.
    hduVector3Dd UnitForceVectorAxis;
    UnitForceVectorAxis = normalize(ForceVectorAxis);

    //For calculation of rotation angle.
    hduVector3Dd ZAxis;  //Z axis is chosen because when drawing a cylinder,
                         // it is drawn along the Z-axis by default.
    ZAxis.set(0.0, 0.0, 1.0);

    hduVector3Dd ToolRotAxis; 
    //the tool rotation axis points to direction perpendicular to 
    // both Z axis and force vector axis
    ToolRotAxis = crossProduct(ZAxis, ForceVectorAxis);
        
    //the angle around the "ToolRotAxis" that will rotate Z axis into 
    // ForceVectorAxis 
    double ToolRotAngle = acos(UnitForceVectorAxis[2]);
    hduMatrix rotMatrix;    
    //the overall rotation matrix for this force vector is...
    //NOTE: defined using "equivalent angle axis" theorem.
    // you don't need to worry about implementation details
    // just find the axis and angle and use the following function (createRotation)
    rotMatrix = hduMatrix::createRotation(ToolRotAxis, ToolRotAngle);

    double rotVals[4][4];
    rotMatrix.get(rotVals); //get the elements of matrix "rotMatrix" and save them
                            // into a 4 by 4 array (something OpenGL understands)
    glMultMatrixd((double*)rotVals);

    //The force arrow: composed of a cylinder and a cone.
    glDisable(GL_LIGHTING);
    glColor3f(0.2, 0.7, 0.2);
    //Draw the cylinder part 
    // parameters are: object_name, base_radius, top_radius, height, slices, stacks)
    gluCylinder(quadObj,SPHERE_RADIUS*0.1, SPHERE_RADIUS*0.1, strength, 16, 2); 
    glTranslatef(0, 0, strength);
    glColor3f(0.2, 0.8, 0.3);
    //Draw the cone part.
    gluCylinder(quadObj, SPHERE_RADIUS*0.2, 0.0, strength*0.15, 16, 2); 
    glEnable(GL_LIGHTING);
}//END of drawForceVisualRepresentation

void drawball(GLUquadricObj* quadObj, HapticDeviceState state)
{
    //contact detection
    double sqrt_dist = sqrt(pow((state.position[0]-spherePosition[0]),2) + pow((state.position[1]
        - spherePosition[1]),2) + pow((state.position[2]-spherePosition[2]),2));

	glPushMatrix();
    glLoadIdentity();
	if(sqrt_dist <= 3*SPHERE_RADIUS && state.button){
		ballAttached = true;
           for( int i = 0; i < 16; i++ ){
               identityTransform[i] = state.transform_matrix[i];
			}
          
		   if (gettingBallPosition) {
			   offsetSphere[0] = state.position[0] - spherePosition[0];
			   offsetSphere[1] = state.position[1] - spherePosition[1];
			   offsetSphere[2] = state.position[2] - spherePosition[2];
			   gettingBallPosition = false;
		   }

		   for( int i = 0; i < 3; i++ ){
               spherePosition[i] = state.position[i] - offsetSphere[i];
		   }
		   
		   glTranslatef(-offsetSphere[0], -offsetSphere[1], -offsetSphere[2]);
		   glMultMatrixd(state.transform_matrix); 

	} else {
		glTranslatef(-offsetSphere[0], -offsetSphere[1], -offsetSphere[2]);
		glMultMatrixd(identityTransform);
		gettingBallPosition = true;
		ballAttached = false;
        
	}
	
	//draw axes
    drawAxes();
    if (sqrt_dist <= 3*SPHERE_RADIUS) {
        glColor4f(0.8, 0.2, 0.2, 0.8);
        
    } else
    {
        //set default sphere color
        glColor4f(0.2, 0.8, 0.8, 0.8);

    }
    
    //Draw the center sphere.
    gluSphere(quadObj, SPHERE_RADIUS*2, 20, 20);

	glPopMatrix();
	glPopMatrix();

	//back
	if (ballAttached){
		if((abs(spherePosition[2]) + 2*SPHERE_RADIUS) >= CUBE_SIZE/2){
			if(spherePosition[2]<0) {
				glBegin(GL_QUADS);
				glColor4f(0.3, 1, 1, 1);
				glVertex3f(-75, -75,-75);
				glVertex3f(75,-75,-75);
				glVertex3f(75,75,-75);
				glVertex3f(-75,75,-75);
				glEnd();
			}
		}
	}

	//right
	if (ballAttached){
		if((abs(spherePosition[0]) + 2*SPHERE_RADIUS) >= CUBE_SIZE/2){
			if(spherePosition[0]>0) {
				glBegin(GL_QUADS);
				glColor4f(0.3, 1, 1, 1);
				glVertex3f(75,-75,-75);
				glVertex3f(75,-75,75);
				glVertex3f(75,75,75);
				glVertex3f(75,75,-75);
				glEnd();
			}
		}
	}

	//down
	if (ballAttached){
		if((abs(spherePosition[1]) + 2*SPHERE_RADIUS) >= CUBE_SIZE/2){
			if(spherePosition[1]<0) {
				glBegin(GL_QUADS);
				glColor4f(0.3, 1, 1, 1);
				glVertex3f(-75,-75,-75);
				glVertex3f(-75,-75,75);
				glVertex3f(75,-75,75);
				glVertex3f(75,-75,-75);
				glEnd();
			}
		}
	}
	//left
	if (ballAttached){
		if((abs(spherePosition[0]) + 2*SPHERE_RADIUS) >= CUBE_SIZE/2){
			if(spherePosition[0]<0) {
				glBegin(GL_QUADS);
				glColor4f(0.3, 1, 1, 1);
				glVertex3f(-75,-75,-75);
				glVertex3f(-75,-75,75);
				glVertex3f(-75,75,75);
				glVertex3f(-75,75,-75);
				glEnd();
			}
		}
	}

	//up
	if (ballAttached){
		if((abs(spherePosition[1]) + 2*SPHERE_RADIUS) >= CUBE_SIZE/2){
			if(spherePosition[1]>0) {
				glBegin(GL_QUADS);
				glColor4f(0.3, 1, 1, 1);
				glVertex3f(-75,75,-75);
				glVertex3f(-75,75,75);
				glVertex3f(75,75,75);
				glVertex3f(75,75,-75);
				glEnd();
			}
		}
	}

		//Front
	if (ballAttached){
		if((abs(spherePosition[2]) + 2*SPHERE_RADIUS) >= CUBE_SIZE/2){
			if(spherePosition[2]>0) {
				glBegin(GL_QUADS);
				glColor4f(0.3, 1, 1, 1);
				glVertex3f(-75,-75,75);
				glVertex3f(75,-75,75);
				glVertex3f(75,75,75);
				glVertex3f(-75,75,75);
				glEnd();
			}
		}
	}
}

//******************************************************************************
//           ~~~~~~  END OF main.cpp   ~~~~~~
//******************************************************************************