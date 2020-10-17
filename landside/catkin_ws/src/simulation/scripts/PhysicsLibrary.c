float* getLinDrag(float dragCoef, float* linVel, int linVelSize, float diameter, float height){
	float dragForce[3];
	for (int i = 0; i < linVelSize; i++){
		dragForce[i] = 0.5 * 1000 * dragCoef * area * linVel[i] * linVel[i];

	}
}

void applyLinDrag(float* force, int objectHandle){

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