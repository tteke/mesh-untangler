#include "stdafx.h"
#include <iostream>
#include "OBJ_Loader.h"
#include <Eigen/Dense>
#include <ceres/ceres.h>

using namespace std;
using namespace Eigen;
using ceres::DynamicAutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;


struct Triangle {
public:
    int idx;
	vector<int> verts;
    Triangle(int v1, int v2, int v3, int id);
};

Triangle::Triangle(int v1, int v2, int v3, int id) {
	verts.push_back(v1);
	verts.push_back(v2);
	verts.push_back(v3);
	idx = id;
}

struct Eq1Error {
public:
    const Triangle tri;
    const int size;
    Eq1Error(const Triangle &_tri, const int &_size) : tri(_tri), size(_size) {};

    static DynamicAutoDiffCostFunction<Eq1Error>* Create(const Triangle &_tri, const int _size) {
        return (new DynamicAutoDiffCostFunction<Eq1Error>(new Eq1Error(_tri, _size)));
    }
	
    template <typename T>
    bool operator()(T const* const* XYs, T* residuals) const {
        Matrix<T,3,1> v1;
        v1 << T(XYs[tri.verts[1]] - XYs[tri.verts[0]]), T(XYs[tri.verts[2] + size] - XYs[tri.verts[0] + size]), T(0);
        Matrix<T,3,1> v2;
        v2 << T(XYs[tri.verts[2]] - XYs[tri.verts[2]]), T(XYs[tri.verts[2]+ size] - XYs[tri.verts[1]+ size]), T(0);

        Matrix<T,3,1> area = v1.cross(v2);

        residuals[0] = T( abs(area[2]) - area[2] );

        return true;
    }
};

int main(int argc, char* argv[]) {
//  argv[1] is the first argument.

    cout << argv[1] << endl;

    objl::Loader Loader;

    bool loadout = Loader.LoadFile(argv[1]);

    if (loadout) {
        for ( objl::Mesh mesh : Loader.LoadedMeshes ) {
            vector <Triangle*> tris;
            int i, id;            

            int mesh_size = mesh.Vertices.size();           

            for (i=0, id=0; i < mesh.Indices.size(); i += 3, id++) {
                tris.push_back( new Triangle (mesh.Indices[i], mesh.Indices[i+1], mesh.Indices[i+2], id) );
            }

            // MESH TANGLING
			double tanglePercent = 0.4;
			int numTangledTris = (int)tris.size()*tanglePercent;

			for (int i = 0; i < numTangledTris; i++) {
				int tId = rand() % tris.size(); //random triangle
				int vId = rand() % 3; //random vertex
				int dId = rand() % 4; //random direction

				int vnId; //neigbor vertex
				if (vId != 0) vnId = 0;
				else if (vId != 1) vnId = 1;
				else if (vId != 2) vnId = 2;

				double length = sqrt(pow(mesh.Vertices[tris[tId]->verts[vId]].Position.X - mesh.Vertices[tris[tId]->verts[vnId]].Position.X, 2) - pow(mesh.Vertices[tris[tId]->verts[vId]].Position.Y - mesh.Vertices[tris[tId]->verts[vnId]].Position.Y, 2));

				if (dId == 0) {
					mesh.Vertices[vId].Position.X = mesh.Vertices[vId].Position.X + length * 2;
				}
				else if (dId == 1) {
					mesh.Vertices[vId].Position.X = mesh.Vertices[vId].Position.X - length * 2;
				}
				else if (dId == 2) {
					mesh.Vertices[vId].Position.Y = mesh.Vertices[vId].Position.Y + length * 2;
				}
				else if (dId == 3) {
					mesh.Vertices[vId].Position.Y = mesh.Vertices[vId].Position.Y - length * 2;
				}
			}

			Eigen::Matrix<double, Dynamic, 1> XYs;
			XYs.resize(mesh_size * 2, 1);

            i = 0;
            for ( objl::Vertex vertex : mesh.Vertices ) {
                XYs(i) = (double)vertex.Position.X;
                XYs(i + mesh_size) = (double)vertex.Position.Y;
                i++;
            }

			mesh;

			
            Problem problem;

            for ( Triangle* triangle: tris ) {
                DynamicAutoDiffCostFunction<Eq1Error>* cost_function = Eq1Error::Create(*triangle, mesh_size);
                cost_function->AddParameterBlock(mesh_size*3);
                cost_function->SetNumResiduals(1);

                problem.AddResidualBlock(cost_function, NULL, XYs.data());
            }
			

            cout << "here" << endl;
        }
    } else {
        cout << "File could not be loaded." << endl;
    }

}
