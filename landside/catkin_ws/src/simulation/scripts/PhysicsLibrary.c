#define RHO 1000            // density of water
#define WATER_HEIGHT 0      // the z height of the water
#define QUADRATIC_DRAG 1    // true is drag should be quadratic, rather than linear

float grav_vector[3];
float grav;

// some C definitions to make CoppeliaSim happy
extern "C" __declspec(dllexport) unsigned char simStart(void* reserved,int reservedInt);
extern "C" __declspec(dllexport) void simEnd();
extern "C" __declspec(dllexport) void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData);

// see https://www.coppeliarobotics.com/helpFiles/en/plugins.htm

unsigned char simStart(void* reserved,int reservedInt){ // called at the beginning of simulation
    // need to initialize constants in here
    // also note that here, unlike the rest of c, zero return is error, not not nonzero return
    if (simGetArrayParameter(sim_arrayparam_gravity, grav_vector)) return 0;
    grav = grav_vector[2];

    return 1;
}

float* getLinearDrag(float dragCoef, float* linVel, float diameter, float height){
	float dragForce = 0.5 * 1000 * dragCoef * area * 
}

int applyBuoyancy(int handle, float* centerOfBuoy) {
    float buoy[3];
    if (calcBuoyancy(handle, buoy) != 0) return -1;
    if (simAddForce(handle, buoy, centerOfBuoy) != 0) return -1;

    return 0;
}

int calcBuoyancy(int handle, float* buoy) {
    // buoyancy = rho * V * g, where V is volume of object underwater
    // calculates V assuming the object fills the entire space of its bounding box
    float minsize[3];
    float maxsize[3];
    float objsize[3];

    for (int i = 0; i < 2; i++) {
        if (simGetObjectFloatParameter(handle, 15+i, minsize+i) != 0) return -1;
        if (simGetObjectFloatParameter(handle, 18+i, maxsize+i) != 0) return -1;
        objsize[i] = maxsize[i] - minsize[i];
    }

    float vol = size[0] * size[1];
    float zdepth; // the amount that the robot is below the water

    // this whole block can probably be condensed into a one liner
    // but I'm leaving it as is for now because I want to logic to be explicit, in the likely event
    // that we have to change it in the near future.
    if (minsize[2] > WATER_HEIGHT) {
        zdepth = 0;
    } else if (maxsize[2] < WATER_HEIGHT) {
        zdepth = size[2];
    } else {
        zdepth = WATER_HEIGHT - minsize[2];
    }

    vol *= zdepth;

    float buoyForce = RHO * vol * grav;

    buoy[0] = 0;
    buoy[1] = 0;
    buoy[2] = buoyForce;
}

float* getLinDrag(float dragCoef, float* linVel, int linVelSize, float diameter, float length){
    float* dragForce = (float*) malloc(3 * sizeof(float));
    float pi = 3.1415926535897932384626433832;

	for (int i = 1; i < 3; i++){
		dragForce[i] = -0.5 * RHO * dragCoef * * linVel[i] * linVel[i];
	}
    dragForce[0] = -0.5 * RHO * dragCoef * pi * diameter * length * 0.5 * (diameter / 2)*(diameter / 2) * pi * linVel[i] * linVel[i];
    return dragForce;
}

void applyLinDrag(float* force, int objectHandle){
    simAddForceAndTorque(objectHandle, force, NULL);
    free(force); //maybe?
}

float* getLinVelocity(int objectHandle){
	int velSize = 3;
	float* linVel = (float*) malloc(sizeof(float) * velSize);
	float* angVel = (float*) malloc(sizeof(float) * velSize);
	int errorCode = simGetObjectVelocity(objectHandle, linVel, angVel);
	free(angVel);
	return linVel;
}

float* getAngVelocity(int objectHandle){
	int velSize = 3;
	float* linVel = (float*) malloc(sizeof(float) * velSize);
	float* angVel = (float*) malloc(sizeof(float) * velSize);
	int errorCode = simGetObjectVelocity(objectHandle, linVel, angVel);
	free(linVel);
	return angVel;
}

float* getAcc(float* prevVel, float* currVel, float timeStep){
	float* accel = (float*) malloc(sizeof(float) * 3);
	for (int i = 0; i < 3; i++){
		*(accel+i) = (*(currVel+i) - *(prevVel+i)) / timeStep;
	}
	return accel;
}