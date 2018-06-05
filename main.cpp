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
    int v1i, v2i, v3i, idx;
    Triangle(int v1, int v2, int v3, int id);
};

Triangle::Triangle(int v1, int v2, int v3, int id) : v1i(v1), v2i(v2), v3i(v3), idx(id) {}

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
        v1 << T(XYs[tri.v2i] - XYs[tri.v1i]), T(XYs[tri.v2i + size] - XYs[tri.v1i + size]), T(0);
        Matrix<T,3,1> v2;
        v2 << T(XYs[tri.v3i] - XYs[tri.v2i]), T(XYs[tri.v3i + size] - XYs[tri.v2i + size]), T(0);

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
            Eigen::Matrix<double, Dynamic, 1> XYs;

            int mesh_size = mesh.Vertices.size();

            XYs.resize(mesh_size*2, 1);

            for (i=0, id=0; i < mesh.Indices.size(); i += 3, id++) {
                tris.push_back( new Triangle (mesh.Indices[i], mesh.Indices[i+1], mesh.Indices[i+2], id) );
            }

            // MESH TANGLING GOES HERE







            // MESH TANGLING

            i = 0;
            for ( objl::Vertex vertex : mesh.Vertices ) {
                XYs(i) = (double)vertex.Position.X;
                XYs(i + mesh_size) = (double)vertex.Position.Y;
                i++;
            }


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
