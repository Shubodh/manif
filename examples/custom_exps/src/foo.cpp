// manif
#include "manif/SE2.h"

// Std
#include <cstdlib>
#include <list>
#include <map>
#include <vector>

// Debug
#include <iomanip>
#include <iostream>

// read input
#include <fstream>

// std shortcuts and namespaces
using std::cout;
using std::endl;
using std::ifstream;
using std::list;
using std::map;
using std::pair;
using std::string;
using std::vector;

// Eigen namespace
using namespace Eigen;

// manif namespace and shortcuts
using manif::SE2d;
using manif::SE2Tangentd;

static constexpr int DoF = SE2d::DoF;  // 3 degrees of freedom
static constexpr int Dim = SE2d::Dim;  // 2D world

// Define many data types (Tangent refers to the tangent of SE2)
typedef Array<double, DoF, 1> ArrayT;       // tangent-size array
typedef Matrix<double, DoF, 1> VectorT;     // tangent-size vector
typedef Matrix<double, DoF, DoF> MatrixT;   // tangent-size square matrix
typedef Matrix<double, Dim, 1> VectorB;     // landmark-size vector
typedef Array<double, Dim, 1> ArrayY;       // measurement-size array
typedef Matrix<double, Dim, 1> VectorY;     // measurement-size vector
typedef Matrix<double, Dim, Dim> MatrixY;   // measurement-size square matrix
typedef Matrix<double, Dim, DoF> MatrixYT;  // measurement x tangent size matrix
typedef Matrix<double, Dim, Dim> MatrixYB;  // measurement x landmark size matrix

// some experiment constants
static const int NUM_POSES = 40;
static const int NUM_STATES = NUM_POSES * DoF;
static const int NUM_MEAS = NUM_POSES * DoF;
static const int MAX_ITER = 20;  // for the solver

struct edge {
    int ind1, ind2;
    SE2Tangentd u;
};

vector<struct edge> read_input(string input_file, vector<SE2d>& poses) {
    //read edges from file
    ifstream fin(input_file);

    string line, token, delimiter = " ";
    vector<struct edge> controls;  // robot controls

    while (getline(fin, line)) {
        vector<string> tokens;
        int pos = 0;
        while ((pos = line.find(delimiter)) != string::npos) {
            token = line.substr(0, pos);
            tokens.push_back(token);
            line.erase(0, pos + delimiter.length());
        }
        tokens.push_back(line);

        if (tokens[0] == "EDGE_SE2") {
            struct edge edg;
            edg.ind1 = atoi(tokens[1].c_str()), edg.ind2 = atoi(tokens[2].c_str());
            float a = atof(tokens[3].c_str()), b = atof(tokens[4].c_str()), c = atof(tokens[5].c_str());
            edg.u << a, b, c;
            controls.push_back(edg);
        }
        if (tokens[0] == "VERTEX_SE2") {
            int a = atoi(tokens[2].c_str()), b = atoi(tokens[3].c_str()), c = atoi(tokens[4].c_str());
            poses.emplace_back(a, b, c);
        }
    }
    return controls;
}

void initial_pose(vector<SE2d>& poses, vector<struct edge>& controls) {
    // create initial pose guess from given edges
    for (auto edg : controls) {
        if (edg.ind2 - edg.ind1 == 1) {
            SE2d X_j = poses[edg.ind1] + edg.u;
            poses.push_back(X_j);
        }
    }
}

int main() {
    // DEBUG INFO
    cout << endl;
    cout << "2D Smoothing and Mapping. 3 poses, 5 landmarks." << endl;
    cout << "-----------------------------------------------" << endl;
    cout << std::fixed << std::setprecision(3) << std::showpos;

    // START CONFIGURATION

    vector<SE2d> poses;
    vector<struct edge> controls;

    // taking edges input
    controls = read_input("src/edges.txt", poses);
    // initial pose guess
    initial_pose(poses, controls);

    // Define a control vector and its noise and covariance in the tangent of SE2
    ArrayT u_sigmas;  // control noise std specification
    MatrixT W;        // sqrt Info

    u_sigmas << 0.01, 0.01, 0.01;
    W = u_sigmas.inverse().matrix().asDiagonal();  // this is Q^(-T/2)

    // Declare some temporaries
    SE2Tangentd d;           // motion expectation d = Xj (-) Xi = Xj.minus ( Xi )
    MatrixT J_d_xi, J_d_xj;  // Jacobian of motion wrt poses i and j
    SE2Tangentd dx;          // optimal pose correction step

    // Problem-size variables
    Matrix<double, NUM_STATES, 1> dX;        // optimal update step for all the SAM problem
    Matrix<double, NUM_MEAS, NUM_STATES> J;  // full Jacobian
    Matrix<double, NUM_MEAS, 1> r;           // full residual

    // END CONFIGURATION

    // Estimator #################################################################

    // DEBUG INFO
    // cout << "prior" << std::showpos << endl;
    // for (const auto& X : poses)
    //     cout << "pose  : " << X.translation().transpose() << " " << X.angle() << endl;
    // cout << "-----------------------------------------------" << endl;

    // iterate
    SE2d Xi, Xj;
    SE2Tangentd u;

    cout << "ITERATIONS" << std::noshowpos << endl;

    for (int iteration = 0; iteration < MAX_ITER; ++iteration) {
        // Clear residual vector and Jacobian matrix
        r.setZero();
        J.setZero();

        // row and column for the full Jacobian matrix J, and row for residual r
        int row = 0, col = 0;

        // residual and Jacobian.
        // Notes:
        //   We have residual = expectation - measurement, in global tangent space
        //   We have the Jacobian in J_r_p0 = J.block<DoF, DoF>(row, col);
        // We compute the whole in a one-liner:
        r.segment<DoF>(row) = poses[0].lminus(SE2d::Identity(), J.block<DoF, DoF>(row, col)).coeffs();

        // advance rows
        row += DoF;

        // loop poses
        for (auto edg : controls) {
            // 2. evaluate motion factors -----------------------
            int i = edg.ind1, j = edg.ind2;

            // recover related states and data
            Xi = poses[i];
            Xj = poses[j];
            u = edg.u;

            //???????????????????????????????????????????????????????
            if (i >= NUM_POSES || j >= NUM_POSES) {
                continue;
            }

            // expectation (use right-minus since motion measurements are local)
            d = Xj.rminus(Xi, J_d_xj, J_d_xi);  // expected motion = Xj (-) Xi

            // residual
            r.segment<DoF>(row) = W * (d - u).coeffs();  // residual

            // Jacobian of residual wrt first pose
            col = i * DoF;
            J.block<DoF, DoF>(row, col) = W * J_d_xi;

            // Jacobian of residual wrt second pose
            col = j * DoF;
            J.block<DoF, DoF>(row, col) = W * J_d_xj;

            // advance rows
            row += DoF;
        }

        // 4. Solve -----------------------------------------------------------------

        // compute optimal step
        // ATTENTION: This is an expensive step!!
        // ATTENTION: Use QR factorization and column reordering for larger problems!!
        dX = -(J.transpose() * J).inverse() * J.transpose() * r;

        // update all poses
        for (int i = 0; i < NUM_POSES; ++i) {
            // we go very verbose here
            int row = i * DoF;
            constexpr int size = DoF;
            dx = dX.segment<size>(row);
            poses[i] = poses[i] + dx;
        }

        // DEBUG INFO
        cout << "residual norm: " << std::scientific << r.norm() << ", step norm: " << dX.norm() << endl;

        // conditional exit
        if (dX.norm() < 1e-6) break;
    }
    cout << "-----------------------------------------------" << endl;

    cout << std::fixed;

    // solved problem
    // cout << "posterior" << std::showpos << endl;
    // for (const auto& X : poses)
    //     cout << "pose  : " << X.translation().transpose() << " " << X.angle() << endl;
    // cout << "-----------------------------------------------" << endl;

    // ground truth
    // cout << "ground truth1" << std::showpos << endl;
    // for (const auto& X : poses_simu)
    //     cout << "pose  : " << X.translation().transpose() << " " << X.angle() << endl;
    // cout << "-----------------------------------------------" << endl;

    return 0;
}
