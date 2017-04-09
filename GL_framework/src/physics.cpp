#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <glm\gtx\rotate_vector.hpp>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\constants.hpp>
#include <glm/gtx/norm.hpp>

#define MURO1 1
#define MURO2 2
#define MURO3 4
#define MURO4 8
#define MURO5 16
#define MURO6 32
#define ESFERA 64


bool show_test_window = false;

extern bool renderSphere;
extern bool renderCapsule;
extern bool renderParticles;
extern bool renderCloths;


namespace Sphere {
	extern void setupSphere(glm::vec3 pos = glm::vec3(0.f, 1.f, 0.f), float radius = 1.f);
	extern void cleanupSphere();
	extern void updateSphere(glm::vec3 pos, float radius = 1.f);
	extern void drawSphere();
}
namespace Capsule {
	extern void setupCapsule(glm::vec3 posA = glm::vec3(-3.f, 2.f, -2.f), glm::vec3 posB = glm::vec3(-4.f, 2.f, 2.f), float radius = 1.f);
	extern void cleanupCapsule();
	extern void updateCapsule(glm::vec3 posA, glm::vec3 posB, float radius = 1.f);
	extern void drawCapsule();
}
namespace LilSpheres {
	extern const int maxParticles;
	extern void setupParticles(int numTotalParticles, float radius = 0.05f);
	extern void cleanupParticles();
	extern void updateParticles(int startIdx, int count, float* array_data);
	extern void drawParticles(int startIdx, int count);
}
namespace ClothMesh {
	extern const int numCols;
	extern const int numRows;
	extern const int numVerts;
	extern void setupClothMesh();
	extern void cleanupClothMesh();
	extern void updateClothMesh(float* array_data);
	extern void drawClothMesh();
}

glm::vec3 gravity;
float elasticCoef;
float frictCoef;

int numOfUpdates;

//Esfera
float SphereRadius;
glm::vec3 SpherePos;

//planos
glm::vec3 lowPlaneNormal(0, 1, 0);
glm::vec3 upperPlaneNormal(0, -1, 0);
glm::vec3 rightPlaneNormal(-1, 0, 0);
glm::vec3 leftPlaneNormal(1, 0, 0);
glm::vec3 frontPlaneNormal(0, 0, 1);
glm::vec3 backPlaneNormal(0, 0, -1);
float lowPlaneD = 0;
float upperPlaneD = 10;
float rightPlaneD = 5;
float leftPlaneD = 5;
float frontPlaneD = 5;
float backPlaneD = 5;

struct Mesh {
public:
	//constructor, destructor y inicializacion
	Mesh() {
		distVertex = 0.5f;
		structuralDist = distVertex;
		shearDist = sqrt(pow(distVertex, 2)*2);
		flexionDist = distVertex * 2;
		maxDistx100 = 20;
		maxDist = distVertex*(1+(maxDistx100/100));

		heightPos = 8;
		vertexPosArray= new glm::vec3*[2];
		vertexPosArray[0] = new glm::vec3[ClothMesh::numVerts];
		vertexPosArray[1] = new glm::vec3[ClothMesh::numVerts];
		arrayToUse = true;

		vertexVelArray= new glm::vec3[ClothMesh::numVerts];
		vertexLastPosArray= new glm::vec3[ClothMesh::numVerts];
		vertexForceArray= new glm::vec3[ClothMesh::numVerts];
		vertexColisionCheckers = new unsigned char[ClothMesh::numVerts];

		rigidezBend = rigidezShear = rigidezStrench = 1000;
		kdBend = kdShear = kdStrench = 20;
		setPosInit();
		setDistancesCol();
	}
	~Mesh() {
		delete[] vertexPosArray[0];
		delete[] vertexPosArray[1];
		delete[] vertexVelArray;
		delete[] vertexLastPosArray;
		delete[] vertexForceArray;
	}
	void setPosInit() {
		for (int i = 0; i < ClothMesh::numRows;i++) {
			for (int j = 0; j < ClothMesh::numCols;j++) {
				vertexPosArray[0][j+i*ClothMesh::numCols] = glm::vec3(i*distVertex - (distVertex*ClothMesh::numRows / 2), heightPos,  j*distVertex - (distVertex*ClothMesh::numCols / 2));
				vertexPosArray[1][j + i*ClothMesh::numCols] = vertexPosArray[0][j + i*ClothMesh::numCols];
				vertexLastPosArray[j + i*ClothMesh::numCols] = glm::vec3(i*distVertex - (distVertex*ClothMesh::numRows / 2), heightPos, j*distVertex - (distVertex*ClothMesh::numCols / 2));
				vertexVelArray[j + i*ClothMesh::numCols] = glm::vec3(0,0,0);
				vertexForceArray[j + i*ClothMesh::numCols] = glm::vec3(0,0,0);
				vertexColisionCheckers[j + i*ClothMesh::numCols] = 0;
			}
		}
	}
	void setDistancesCol() {
		//posiciones relativas a los planos i esfera
		for (int i = 0; i <= ClothMesh::numVerts; i++) {
			vertexColisionCheckers[i] = vertexColisionCheckers[i] | (wallDistance(i, lowPlaneNormal, lowPlaneD)*MURO1);
			vertexColisionCheckers[i] = vertexColisionCheckers[i] | (wallDistance(i, upperPlaneNormal, upperPlaneD)*MURO2);
			vertexColisionCheckers[i] = vertexColisionCheckers[i] | (wallDistance(i, rightPlaneNormal, rightPlaneD)*MURO3);
			vertexColisionCheckers[i] = vertexColisionCheckers[i] | (wallDistance(i, leftPlaneNormal, leftPlaneD)*MURO4);
			vertexColisionCheckers[i] = vertexColisionCheckers[i] | (wallDistance(i, frontPlaneNormal, frontPlaneD)*MURO5);
			vertexColisionCheckers[i] = vertexColisionCheckers[i] | (wallDistance(i, backPlaneNormal, backPlaneD)*MURO6);
		}
	}
	

	void reset() {
		structuralDist = distVertex;
		shearDist = sqrt(pow(distVertex, 2) * 2);
		flexionDist = distVertex * 2;
		maxDist = distVertex*(1 + (maxDistx100 / 100));
		setPosInit();
		setDistancesCol();
	}
	//fuerzas
	inline void addSpringForce(int i0, int i1, float& dist, float& rigidez, float& kd) {
		newForce= (rigidez*(glm::distance(vertexPosArray[!arrayToUse][i0], vertexPosArray[!arrayToUse][i1]) - dist) + glm::dot(kd*(vertexVelArray[i0] - vertexVelArray[i1]), glm::normalize(vertexPosArray[!arrayToUse][i0] - vertexPosArray[!arrayToUse][i1])))*glm::normalize(vertexPosArray[!arrayToUse][i0] - vertexPosArray[!arrayToUse][i1]);
		vertexForceArray[i0] -= newForce;
		vertexForceArray[i1] += newForce;
	}
	void addStructuralForces(const int &i) {
		if ((i+1)%ClothMesh::numCols!=0) {//comprovamos que no pase de fila
			addSpringForce(i, i+1, structuralDist,rigidezStrench,kdStrench);
		}	
		if (i+ClothMesh::numCols<ClothMesh::numVerts) {
			addSpringForce(i,i+ClothMesh::numCols,structuralDist, rigidezStrench, kdStrench);
		}
		
	}
	void addShearForces(const int &i) {
		if (i + ClothMesh::numCols + 1 < ClothMesh::numVerts && (i + ClothMesh::numCols + 1)%ClothMesh::numCols!=0 ) {
			addSpringForce(i, i+ClothMesh::numCols+1, shearDist,rigidezShear,kdShear);
		}	
		if (i + ClothMesh::numCols - 1 < ClothMesh::numVerts && (i + ClothMesh::numCols - 1) % ClothMesh::numCols != ClothMesh::numCols-1) {
			addSpringForce(i,i+ClothMesh::numCols-1, shearDist, rigidezShear, kdShear);
		}
	}
	void addFlexionForces(const int &i) {
		if (i+ (ClothMesh::numCols*2)<ClothMesh::numVerts) {
			addSpringForce(i,i+(ClothMesh::numCols*2), flexionDist,rigidezBend, kdBend);
		}
		if ((i+2)%ClothMesh::numCols!=0 && (i+2)%ClothMesh::numCols!=1) {
			addSpringForce(i,i+2, flexionDist, rigidezBend, kdBend);
		}
	}
	
	//solver
	inline void verletPositionSolver(int &i, float& dt) {
		vertexPosArray[arrayToUse][i] = vertexPosArray[!arrayToUse][i]+(vertexPosArray[!arrayToUse][i]-vertexLastPosArray[i])+vertexForceArray[i]*pow(dt,2);
	}
	inline void verletVelSolver( int& i, float& dt) {
		vertexVelArray[i] = (vertexPosArray[arrayToUse][i] - vertexLastPosArray[i]) / dt;
	}
	
	//constrains
	inline void applyConstrain(int& i0) {
		vertexPosArray[arrayToUse][i0] -= glm::normalize(constrainVertVertVector)*(constrainCurrentDist-maxDist);
	}
	void checkConstrains(int &i) {
		if (i - ClothMesh::numCols >= 0) {
			constrainVertVertVector = vertexPosArray[arrayToUse][i] - vertexPosArray[arrayToUse][i - ClothMesh::numCols];
			constrainCurrentDist = glm::length(constrainVertVertVector);
			if (constrainCurrentDist>maxDist) {
				applyConstrain(i);
				constrainCurrentDist = glm::length(vertexPosArray[arrayToUse][i] - vertexPosArray[arrayToUse][i - ClothMesh::numCols]);
			}	
		}
		if (i - 1 >= 0 && i%ClothMesh::numCols!=0) {
			constrainVertVertVector = vertexPosArray[arrayToUse][i] - vertexPosArray[arrayToUse][i - 1];
			constrainCurrentDist = glm::length(constrainVertVertVector);
			if (constrainCurrentDist>maxDist) {
				applyConstrain(i);
				constrainCurrentDist = glm::length(vertexPosArray[arrayToUse][i] - vertexPosArray[arrayToUse][i-1]);
			}
		}

	}
	
	//colisions
	void mirrorPosition(int& i, glm::vec3& planeNormal, float&d) {
		vertexPosArray[arrayToUse][i] = vertexPosArray[arrayToUse][i] - (1 + elasticCoef)*(glm::dot(vertexPosArray[arrayToUse][i], planeNormal) + d)*planeNormal;
		vertexLastPosArray[i] = vertexLastPosArray[i] - (1 + elasticCoef)*(glm::dot(vertexLastPosArray[i], planeNormal) + d)*planeNormal;
	}
	void mirrorVelocity(int& i, glm::vec3& planeNormal) {
		vertexVelArray[i] = vertexVelArray[i] - (1 + elasticCoef)*(glm::dot(planeNormal, vertexVelArray[i]))*planeNormal;
		vertexVelArray[i] = vertexVelArray[i] - frictCoef*(vertexVelArray[i] - glm::dot(planeNormal, vertexVelArray[i])*planeNormal);
	}

	bool wallDistance(int i, glm::vec3& planeNormal, float& planeD) { 
		if (glm::dot(vertexPosArray[arrayToUse][i], planeNormal) + planeD >= 0) {
			return 1;
		}
		else {
			return 0;
		}
	}	
	void wallColision(int& i, glm::vec3& planeNormal,float& d, int numMuro) {
		if (wallDistance(i, planeNormal, d)*numMuro != (vertexColisionCheckers[i] & numMuro)) {
			mirrorPosition(i, planeNormal, d);
			mirrorVelocity(i,planeNormal);
		}
	}


	void findCollPoint(int& i) {
		//buscamos el vector director que forman la pos anterior y la nueva
		sphereLast2NewVect = vertexLastPosArray[i] - vertexPosArray[arrayToUse][i];

		//hacemos una recta usando ese vector y calculamos sus colisiones con la esfera
		sphereA = pow(sphereLast2NewVect.x, 2) + pow(sphereLast2NewVect.y, 2) + pow(sphereLast2NewVect.z, 2);
		sphereB = 2 * ((sphereLast2NewVect.x*(vertexPosArray[arrayToUse][i].x - SpherePos.x)) + (sphereLast2NewVect.y*(vertexPosArray[arrayToUse][i].y - SpherePos.y)) + (sphereLast2NewVect.z*(vertexPosArray[arrayToUse][i].z - SpherePos.z)));
		sphereC = pow(SpherePos.x, 2) + pow(SpherePos.y, 2) + pow(SpherePos.z, 2) + pow(vertexPosArray[arrayToUse][i].x, 2) + pow(vertexPosArray[arrayToUse][i].y, 2) + pow(vertexPosArray[arrayToUse][i].z, 2) - 2 * ((SpherePos.x*vertexPosArray[arrayToUse][i].x) + (SpherePos.y*vertexPosArray[arrayToUse][i].y) + (SpherePos.z*vertexPosArray[arrayToUse][i].z)) - pow(SphereRadius, 2);
		sphereLambda = (-sphereB + sqrt(pow(sphereB, 2) - (4 * sphereA*sphereC))) / (2 * sphereA);

		sphereRealColPoint.x = vertexPosArray[arrayToUse][i].x + sphereLambda*sphereLast2NewVect.x;
		sphereRealColPoint.y = vertexPosArray[arrayToUse][i].y + sphereLambda*sphereLast2NewVect.y;
		sphereRealColPoint.z = vertexPosArray[arrayToUse][i].z + sphereLambda*sphereLast2NewVect.z;
	}
	void mirrorSphere(int &i) {
		//encontramos el punto real de colision
		findCollPoint(i);
		spherePlaneNormal = SpherePos - sphereRealColPoint;
		//hacemos la normal unitaria
		spherePlaneNormal = glm::normalize(spherePlaneNormal);
		//calculamos la D del plano de colision
		//spherePlaneD = -spherePlaneNormal.x*sphereRealColPoint.x - spherePlaneNormal.y*sphereRealColPoint.y - spherePlaneNormal.z*sphereRealColPoint.z;
		spherePlaneD = -glm::dot(spherePlaneNormal, sphereRealColPoint);
		mirrorPosition(i, spherePlaneNormal, spherePlaneD);
		//mirrorVelocity(i,spherePlaneNormal);
		
	}
	void sphereColision(int &i) {
		if (glm::distance(vertexPosArray[arrayToUse][i],SpherePos)-SphereRadius<0) {
			mirrorSphere(i);
		}
	}

	void checkColisions(int& i) {
		//colisiones con los muros
		wallColision(i,lowPlaneNormal,lowPlaneD, MURO1);
		wallColision(i, upperPlaneNormal, upperPlaneD, MURO2);
		wallColision(i, rightPlaneNormal, rightPlaneD, MURO3);
		wallColision(i, leftPlaneNormal, leftPlaneD, MURO4);
		wallColision(i, frontPlaneNormal, frontPlaneD, MURO5);
		wallColision(i, backPlaneNormal, backPlaneD, MURO6);
		//colision con la esfera	
		sphereColision(i);

	}

	void update(float dt) {
		
		glm::vec3 lastPos;
		for (int i = 1; i < ClothMesh::numVerts;i++) {
			//vertexForceArray[i] = glm::vec3(0, 0, 0);
			vertexForceArray[i] = gravity;
		}
		//caluclo de fuerzas y movimiento
		for (int i = 0; i < ClothMesh::numVerts;i++) {
			//fijamos el segundo punto fijo
				lastPos = vertexPosArray[!arrayToUse][i];
				//calculamos fuerza
				#pragma region CalculoFuerza
					//fuerzas internas
						//fuerzaStructural
						addStructuralForces(i);
						//shearForces
						addShearForces(i);
						//FlexionForces
						addFlexionForces(i);
				#pragma endregion

				//usamos solver para calcular posicion
				if (i != ClothMesh::numCols - 1 && i != 0) {
						verletPositionSolver(i, dt);
					}
				//guardamos posicion anterior
					vertexLastPosArray[i] = lastPos;
				//usamos solver para calcular velocidad
					verletVelSolver(i, dt);

					//checkColisions(i);
				
		}
		//aplicamos colisiones
		for (int i = 1; i < ClothMesh::numVerts;i++) {
			checkColisions(i);
		}
		//aplicamos los contrains
		for (int i =1 ; i < ClothMesh::numVerts;i++) {
			if (i != ClothMesh::numCols - 1) {
				checkConstrains(i);
			}
		}
		//cambiamos los buffers
		swapBuffers();
	}

	inline void swapBuffers() {
		arrayToUse = !arrayToUse;
	}

	//arrays de datos de vertices
	glm::vec3** vertexPosArray;
	bool arrayToUse;
	glm::vec3* vertexVelArray;
	glm::vec3* vertexForceArray;
	glm::vec3* vertexLastPosArray;
	unsigned char* vertexColisionCheckers;

	//distancias
	float distVertex;
	float structuralDist;
	float shearDist;
	float flexionDist;
	float maxDistx100;
	float maxDist;


	//altura original de la malla
	float heightPos;

	//variables de optimizacion
	glm::vec3 newForce;//variable para la suma de fuerzas
	glm::vec3 constrainVertVertVector;
	float constrainCurrentDist;
	
	glm::vec3 sphereLast2NewVect;
	glm::vec3 sphereRealColPoint;
	glm::vec3 spherePlaneNormal;
	float spherePlaneD;
	float sphereA;
	float sphereB;
	float sphereC;
	float sphereLambda;

	float rigidezStrench;
	float rigidezShear;
	float rigidezBend;

	float kdStrench;//resistencia a la velocidad
	float kdShear;
	float kdBend;
};

Mesh TheMesh;
bool useCamaraLenta=false;
bool pause=false;
bool demoMode=false;

float demoModeMaxTime = 3;
float demoModeTime = 0;

void randomSpherePos() {
	SpherePos.x = ((float)rand() / RAND_MAX) * 4;
	SpherePos.y = ((float)rand() / RAND_MAX) * 5;
	SpherePos.z = ((float)rand() / RAND_MAX) * 8 - 4;

	SphereRadius = ((float)rand() / RAND_MAX)*0.5 + TheMesh.maxDist;

	Sphere::updateSphere(SpherePos, SphereRadius - 0.01);
}

void GUI() {
	{	//FrameRate
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		
		ImGui::Checkbox("Pause",&pause);
		if (ImGui::Button("Demo Mode")) {
			demoMode = !demoMode;
			TheMesh.reset();
			randomSpherePos();
			demoModeTime = 0;
		}
		if (!demoMode) {
			if (ImGui::Button("Manual Reset")) {
				TheMesh.reset();
			}
			ImGui::Checkbox("Camara Lenta", &useCamaraLenta);
		}
		else {
			ImGui::DragFloat("Demo Mode Time",&demoModeMaxTime, 0.1,1,10);
		}
		ImGui::DragInt("Num of update Repeats per frame", &numOfUpdates,1, 5,30);

		ImGui::Text("\n");
		ImGui::DragFloat("Accepted Elongation %", &TheMesh.maxDistx100, 0.5, 20, 100);
		ImGui::DragFloat("Initial Distance", &TheMesh.distVertex, 0.01, 0.2, 0.6);

		ImGui::Text("\n");
		ImGui::DragFloat("Constant Direct-Link springs", &TheMesh.rigidezStrench,10,200,2000);
		ImGui::DragFloat("Constant Diagonal-Link springs", &TheMesh.rigidezShear, 10, 200, 2000);
		ImGui::DragFloat("Constant Second-Link springs", &TheMesh.rigidezBend, 10, 200, 2000);

		ImGui::Text("\n");
		ImGui::DragFloat("Damping Direct-Link springs", &TheMesh.kdStrench, 0.5, 20, 70);
		ImGui::DragFloat("Damping Diagonal-Link springs", &TheMesh.kdShear, 0.5, 20, 70);
		ImGui::DragFloat("Damping Second-Link springs", &TheMesh.kdBend, 0.5, 20, 70);
		if (!demoMode) {
			ImGui::DragFloat3("Sphere Position", &SpherePos.x, 0.01);
			ImGui::DragFloat("SphereRadius", &SphereRadius, 0.01, 0.1, 3);
		}
	}

	// ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
		
	}
}

void PhysicsInit() {
	numOfUpdates = 10;
	gravity = glm::vec3(0,-9.81,0);
	elasticCoef = 0.8;
	frictCoef = 0.5;

	SpherePos = glm::vec3(2, 2, 0);
	SphereRadius = 1.3;
	Sphere::updateSphere(SpherePos,SphereRadius-0.01);

	ClothMesh::updateClothMesh(&TheMesh.vertexPosArray[TheMesh.arrayToUse][0].x);
	//ClothMesh::updateClothMesh();
}
void PhysicsUpdate(float dt) {
	//TODO
	if (!pause) {
		if (demoMode) {
			if (demoModeTime<=demoModeMaxTime) {
				for (int i = 0; i < numOfUpdates; i++) {
					TheMesh.update(dt / numOfUpdates);
				}
				demoModeTime += dt;
			}
			else {
				demoModeTime = 0;
				randomSpherePos();
				TheMesh.reset();
			}
		}
		else {
			if (useCamaraLenta) {
				TheMesh.update(dt / numOfUpdates);
			}
			else {
				for (int i = 0; i < numOfUpdates; i++) {
					TheMesh.update(dt / numOfUpdates);
				}
			}
			Sphere::updateSphere(SpherePos, SphereRadius-0.01);
		}
	}
		ClothMesh::updateClothMesh(&TheMesh.vertexPosArray[!TheMesh.arrayToUse][0].x);
	
}
void PhysicsCleanup() {
	
}