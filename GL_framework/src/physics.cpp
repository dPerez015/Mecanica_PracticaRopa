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
	extern int numCols;
	extern int numRows;
	extern int numVerts;
	extern void setupClothMesh();
	extern void cleanupClothMesh();
	extern void updateClothMesh(float* array_data);
	extern void drawClothMesh();
}

struct MeshVertex {
	glm::vec3 position;

};

struct Mesh {
public:
	Mesh() {
		distVertex = 0.5f;
		vertexArray = new MeshVertex[ClothMesh::numVerts];
		setPosInit();
	}
	void setPosInit() {
		for (int i = 0; i < ClothMesh::numCols;i++) {
			for (int j = 0; j < ClothMesh::numRows;j++) {
				vertexArray[i*ClothMesh::numRows + j].position = glm::vec3(i*distVertex-(distVertex*ClothMesh::numCols/2),8,j*distVertex-(distVertex*ClothMesh::numRows/2));
			}


		}

	}
	MeshVertex* vertexArray;
	float distVertex;


};


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
	//TODO

}
void PhysicsUpdate(float dt) {
	//TODO
}
void PhysicsCleanup() {
	//TODO
}