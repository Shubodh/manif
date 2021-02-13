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
using std::ofstream;
using std::pair;
using std::string;
using std::vector;

// Eigen namespace
using namespace Eigen;

// manif namespace and shortcuts
using manif::SE2d;
using manif::SE2Tangentd;

static constexpr int DoF = SE2d::DoF;  // 3 degrees of freedom

// Define many data types (Tangent refers to the tangent of SE2)
typedef Array<double, DoF, 1> ArrayT;      // tangent-size array
typedef Matrix<double, DoF, DoF> MatrixT;  // tangent-size square matrix
typedef Matrix<double, Dynamic, Dynamic> Matrixd;

static const int MAX_ITER = 30;

//static const string input_edges_file = "../src/input/input_intel.g2o";
//static const string output_opt_file = "../src/output/opt_intel.g2o";


// Information matrix
static const double WEIGHT_ODOMETRY = 100;
static const double WEIGHT_LOOP_CLOSURE = 100;

struct edge {
    int ind1, ind2;
    SE2Tangentd u;
};

vector<struct edge> read_input(vector<SE2d>& poses, const string& input_edges_file) {
    //read edges from file
    ifstream fin(input_edges_file);

    string line, token, delimiter = " ";
    vector<struct edge> controls;  // robot controls

    while (getline(fin, line)) {
        vector<string> tokens;
        int pos = 0;
        // tokenize string by spaces
        while ((pos = line.find(delimiter)) != string::npos) {
            token = line.substr(0, pos);
            tokens.push_back(token);
            line.erase(0, pos + delimiter.length());
        }
        tokens.push_back(line);

        if (tokens[0] == "EDGE_SE2") {
            struct edge edg;
            edg.ind1 = atoi(tokens[1].c_str()), edg.ind2 = atoi(tokens[2].c_str());
            double a = atof(tokens[3].c_str()), b = atof(tokens[4].c_str()), c = atof(tokens[5].c_str());
            edg.u << a, b, c;
            controls.push_back(edg);
        }
        if (tokens[0] == "VERTEX_SE2") {
            double a = atof(tokens[2].c_str()), b = atof(tokens[3].c_str()), c = atof(tokens[4].c_str());
            poses.emplace_back(a, b, c);
        }
    }
    return controls;
}

//HOWTO: ./sam inputFile outputFile
int main(const int argc, const char *argv[]) {
//    //Default arguments
    string input_edges_file = "../src/input/input_simulated.g2o";
    string output_opt_file = "../src/output/opt_simulated.g2o";

    //Input args
    if (argc > 1){
        input_edges_file = argv[1];
        output_opt_file = argv[2];
    }

    // DEBUG INFO
    cout << "-----------------------------------------------" << endl;
    cout << "2D Smoothing and Mapping." << endl;
    cout << "-----------------------------------------------" << endl;
    cout << std::fixed << std::setprecision(5) << std::showpos;

    // START CONFIGURATION
    vector<SE2d> poses;            // robots poses
    vector<struct edge> controls;  // robots controls

    // reading input edges and pose
    controls = read_input(poses, input_edges_file);

    cout << "Number of poses " << poses.size() << endl;
    cout << "Number of edges " << controls.size() << endl;
    cout << "-----------------------------------------------" << endl;

    // variables
    int NUM_POSES = poses.size();
    int NUM_STATES = NUM_POSES * DoF;
    int NUM_MEAS = (controls.size() + 1) * DoF;  // + 1 for anchor

    // information matrix
    ArrayT odom_info, loop_info;
    odom_info << WEIGHT_ODOMETRY, WEIGHT_ODOMETRY, WEIGHT_ODOMETRY;
    loop_info << WEIGHT_LOOP_CLOSURE, WEIGHT_LOOP_CLOSURE, WEIGHT_LOOP_CLOSURE;
    MatrixT W;

    // Declare some temporaries
    SE2Tangentd d;           // motion expectation d = Xj (-) Xi = Xj.minus ( Xi )
    SE2Tangentd dx;          // optimal pose correction step
    MatrixT J_d_xi, J_d_xj;  // Jacobian of motion wrt poses i and j

    // Problem-size variables
    VectorXd dX(NUM_STATES, 1);       // optimal update step for all the SAM problem
    Matrixd J(NUM_MEAS, NUM_STATES);  // full Jacobian
    VectorXd r(NUM_MEAS, 1);          // full residual

    // END CONFIGURATION

    // iterate
    SE2d Xi, Xj;
    SE2Tangentd u;
    SE2d anchor = poses[0];

    cout << "ITERATIONS" << std::noshowpos << endl;
    cout << "-----------------------------------------------" << endl;

    for (int iteration = 0; iteration < MAX_ITER; ++iteration) {
        cout << "iteration number: " << iteration << endl;
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
        r.segment<DoF>(row) = poses[0].lminus(anchor, J.block<DoF, DoF>(row, col)).coeffs();

        // advance rows
        row += DoF;

        // loop poses
        for (auto edg : controls) {
            // evaluate motion factors
            int i = edg.ind1, j = edg.ind2;

            // info matrix according to type of edge
            if (abs(i - j) == 1) {
                W = odom_info.matrix().asDiagonal();
            } else {
                W = loop_info.matrix().asDiagonal();
            }

            // recover related states and data
            Xi = poses[i];
            Xj = poses[j];
            u = edg.u;

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

        // compute optimal step
        // ATTENTION: This is an expensive step!!
        // ATTENTION: Use QR factorization and column reordering for larger problems!!
        // dX = -(J.transpose() * J).inverse() * J.transpose() * r;
        auto H = J.transpose() * J;
        auto b = -J.transpose() * r;
        cout << "1 DEBUG" << endl;
        cout << H.size() << endl;
        dX = H.llt().solve(b);
        cout << "2 DEBUG" << endl;

        // update all poses
        for (int i = 0; i < NUM_POSES; ++i) {
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

    // writing final optimised poses to file
    int ind = 0;
    ofstream fout(output_opt_file);
    for (const auto& X : poses) {
        fout << "VERTEX_SE2 " << ind++ << " " << X.translation().transpose() << " " << X.angle() << endl;
    }

    return 0;
}