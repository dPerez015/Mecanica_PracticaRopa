#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <glm\gtx\rotate_vector.hpp>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\constants.hpp>
#include <glm/gtx/norm.hpp>


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
int numOfUpdates;


struct Mesh {
public:
	Mesh() {
		distVertex = 0.5f;
		shearDist = sqrt(pow(distVertex, 2)*2);
		flexionDist = distVertex * 2;

		heightPos = 8;
		vertexPosArray= new glm::vec3*[2];
		vertexPosArray[0] = new glm::vec3[ClothMesh::numVerts];
		vertexPosArray[1] = new glm::vec3[ClothMesh::numVerts];
		arrayToUse = true;

		vertexVelArray= new glm::vec3[ClothMesh::numVerts];
		vertexLastPosArray= new glm::vec3[ClothMesh::numVerts];
		vertexForceArray= new glm::vec3[ClothMesh::numVerts];

		rigidez = 500;
		kd = 50;
		setPosInit();
	}
	void setPosInit() {
		for (int i = 0; i < ClothMesh::numRows;i++) {
			for (int j = 0; j < ClothMesh::numCols;j++) {
				vertexPosArray[0][j+i*ClothMesh::numCols] = glm::vec3(i*distVertex - (distVertex*ClothMesh::numRows / 2), heightPos,  j*distVertex - (distVertex*ClothMesh::numCols / 2));
				vertexPosArray[1][j + i*ClothMesh::numCols] = vertexPosArray[0][j + i*ClothMesh::numCols];
				vertexLastPosArray[j + i*ClothMesh::numCols] = glm::vec3(i*distVertex - (distVertex*ClothMesh::numRows / 2), heightPos, j*distVertex - (distVertex*ClothMesh::numCols / 2));
				vertexVelArray[j + i*ClothMesh::numCols] = glm::vec3(0,0,0);
				vertexForceArray[j + i*ClothMesh::numCols] = glm::vec3(0,0,0);
			}
		}
	}

	inline void addSpringForce(int i0, int i1, float& dist) {
		newForce= (rigidez*(glm::distance(vertexPosArray[!arrayToUse][i0], vertexPosArray[!arrayToUse][i1]) - dist) + glm::dot(kd*(vertexVelArray[i0] - vertexVelArray[i1]), glm::normalize(vertexPosArray[!arrayToUse][i0] - vertexPosArray[!arrayToUse][i1])))*glm::normalize(vertexPosArray[!arrayToUse][i0] - vertexPosArray[!arrayToUse][i1]);
		vertexForceArray[i0] -= newForce;
		vertexForceArray[i1] += newForce;
	}

	void addStructuralForces( int &i) {
		if (i + 1 < ClothMesh::numVerts && (i+1)%ClothMesh::numCols!=0) {//comprovamos que no pase de fila
			addSpringForce(i, i+1, distVertex);
		}
		/*if (i-1>=0 && (i-1)%ClothMesh::numCols!=ClothMesh::numCols-1) {//comprovamos que no haya vuelto atras
			addSpringForce(i,i-1, distVertex);
		}*/
		if (i+ClothMesh::numCols<ClothMesh::numVerts) {
			addSpringForce(i,i+ClothMesh::numCols, distVertex);
		}
		/*
		if (i-ClothMesh::numCols>=0) {
			addSpringForce(i,i-ClothMesh::numCols, distVertex);
		}*/
	}
	void addShearForces(int &i) {
		if (i + ClothMesh::numCols + 1 < ClothMesh::numVerts && (i + ClothMesh::numCols + 1)%ClothMesh::numCols!=0 ) {
			addSpringForce(i, i+ClothMesh::numCols+1, shearDist);
		}
		/*
		if (i + ClothMesh::numCols - 1 < ClothMesh::numVerts && (i + ClothMesh::numCols - 1) % ClothMesh::numCols != ClothMesh::numCols-1) {
			addSpringForce(i,i+ClothMesh::numCols-1, shearDist);
		}*/
		if (i-ClothMesh::numCols+1>0 && (i - ClothMesh::numCols + 1) % ClothMesh::numCols != 0) {
			addSpringForce(i,i-ClothMesh::numCols+1, shearDist);
		}
		/*
		if (i-ClothMesh::numCols-1>=0  && (i - ClothMesh::numCols - 1) % ClothMesh::numCols != ClothMesh::numCols - 1) {
			addSpringForce(i,i-ClothMesh::numCols-1, shearDist);
		}*/
	}
	void addFlexionForces(int &i) {
		if (i+ (ClothMesh::numCols*2)<ClothMesh::numVerts) {
			addSpringForce(i,i+(ClothMesh::numCols*2), flexionDist);
		}/*
		if (i-(ClothMesh::numCols*2)>=0) {
			addSpringForce(i,i-(ClothMesh::numCols*2), flexionDist);
		}*/
		if (i+2<ClothMesh::numVerts	&&	(i+2)%ClothMesh::numCols!=0 && (i+2)%ClothMesh::numCols!=1) {
			addSpringForce(i,i+2, flexionDist);
		}
		/*
		if (i-2>=0 && (i-2)%ClothMesh::numCols!=ClothMesh::numCols-1	&& (i - 2) % ClothMesh::numCols != ClothMesh::numCols - 2) {
			addSpringForce(i,i-2, flexionDist);
		}*/
	}
	
	inline void verletPositionSolver(int &i, float& dt) {
		vertexPosArray[arrayToUse][i] = vertexPosArray[!arrayToUse][i]+(vertexPosArray[!arrayToUse][i]-vertexLastPosArray[i])+vertexForceArray[i]*pow(dt,2);
	}
	inline void verletVelSolver(int& i, float& dt) {
		vertexVelArray[i] = (vertexPosArray[arrayToUse][i] - vertexLastPosArray[i]) / dt;
	}

	void update(float dt) {
		glm::vec3 lastPos;
		
		for (int i = 1; i < ClothMesh::numVerts;i++) {
			if (i!=ClothMesh::numCols-1) {//fijamos el segundo punto fijo
				lastPos = vertexPosArray[!arrayToUse][i];
				//calculamos fuerza
				#pragma region CalculoFuerza
					//fuerzas externas
					vertexForceArray[i] = gravity;
					//fuerzas internas
						//fuerzaStructural
						addStructuralForces(i);
						//shearForces
						addShearForces(i);
						//FlexionForces
						addFlexionForces(i);
				#pragma endregion

				//usamos solver para calcular posicion
					verletPositionSolver(i,dt);
				//aplicamos los constrains

				//guardamos posicion anterior
					vertexLastPosArray[i] = lastPos;
				//usamos solver para calcular velocidad
					verletVelSolver(i, dt);
			}
		}
		swapBuffers();
	}

	inline void swapBuffers() {
		arrayToUse = !arrayToUse;
	}

	glm::vec3** vertexPosArray;
	bool arrayToUse;
	glm::vec3* vertexVelArray;
	glm::vec3* vertexForceArray;
	glm::vec3* vertexLastPosArray;
	float distVertex;
	float shearDist;
	float flexionDist;
	float heightPos;
	//variable para la suma de fuerzas
	glm::vec3 newForce;

	float rigidez;
	float kd;//resistencia a la velocidad
};
Mesh TheMesh;

void GUI() {
	{	//FrameRate
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		//TODO
	}

	// ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

void PhysicsInit() {
	numOfUpdates = 30;
	gravity = glm::vec3(0,-9.81,0);
	ClothMesh::updateClothMesh(&TheMesh.vertexPosArray[TheMesh.arrayToUse][0].x);
	//ClothMesh::updateClothMesh();
}
void PhysicsUpdate(float dt) {
	//TODO

	for (int i = 0; i < numOfUpdates;i++) {
		TheMesh.update(dt/numOfUpdates);
	}

	ClothMesh::updateClothMesh(&TheMesh.vertexPosArray[!TheMesh.arrayToUse][0].x);

}
void PhysicsCleanup() {
	//TODO
}