#include <iostream>
#include "OBJ_Loader.h"
#include <Eigen/Dense>
#include <ceres/ceres.h>

using namespace std;
using namespace Eigen;
using ceres::AutoDiffCostFunction;
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
    Eq1Error(const Triangle &_tri) : tri(_tri) {};
    static CostFunction* Create(const Triangle &_tri, const int &_size) {
        return (new AutoDiffCostFunction<Eq1Error, 1, _size, _size, _size>(new Eq1Error(_tri)));
    }
    template <typename T>
    bool operator()(const T* const Xs, const T* const Ys, const T* const Zs, T* residuals) const {
        Matrix<T,3,1> v1;
        v1 << T(Xs(tri.v2i) - Xs[tri.v1i]), T(Ys[tri.v2i] - Ys[tri.v1i]), T(Zs[tri.v2i] - Zs[tri.v1i]);
        Matrix<T,3,1> v2;
        v2 << T(Xs(tri.v3i) - Xs[tri.v2i]), T(Ys[tri.v3i] - Ys[tri.v2i]), T(Zs[tri.v3i] - Zs[tri.v2i]);

        Matrix<T,3,1> area = v1.cross(v2);

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
            Eigen::Matrix<float, Dynamic, 1> Xs;
            Eigen::Matrix<float, Dynamic, 1> Ys;
            Eigen::Matrix<float, Dynamic, 1> Zs;

            Xs.resize(mesh.Vertices.size(), 1);
            Ys.resize(mesh.Vertices.size(), 1);
            Zs.resize(mesh.Vertices.size(), 1);

            for (i=0, id=0; i < mesh.Indices.size(); i += 3, id++) {
                tris.push_back( new Triangle (mesh.Indices[i], mesh.Indices[i+1], mesh.Indices[i+2], id) );
            }

            i = 0;
            for ( objl::Vertex vertex : mesh.Vertices ) {
                Xs(i) = vertex.Position.X;
                Ys(i) = vertex.Position.Y;
                Zs(i) = vertex.Position.Z;
                i++;
            }

            cout << "here" << endl;
        }
    } else {
        cout << "File could not be loaded." << endl;
    }

}
