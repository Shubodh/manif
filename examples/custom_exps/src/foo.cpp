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
static const int NUM_POSES = 3;
static const int NUM_LMKS = 5;
static const int NUM_FACTORS = 9;
static const int NUM_STATES = NUM_POSES * DoF + NUM_LMKS * Dim;
static const int NUM_MEAS = NUM_POSES * DoF + NUM_FACTORS * Dim;
static const int MAX_ITER = 20;  // for the solver

struct edge {
    int ind1, ind2;
    SE2Tangentd u;
};

vector<struct edge> read_input(string input_file, vector<SE2d> &poses) {
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

void initial_pose(vector<SE2d> &poses, vector<struct edge> &controls) {
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
    //
    //

    // Define the robot pose elements
    SE2d X_simu,         // pose of the simulated robot
        Xi,              // robot pose at time i
        Xj;              // robot pose at time j
    vector<SE2d> poses;  // estimator poses

    vector<struct edge> controls = read_input("src/edges.txt", poses);
    initial_pose(poses, controls);
    cout << poses.size() << endl;
    // for (auto ps : poses) {
    //     cout << ps << endl;
    // }

    // Xi.setIdentity();
    // X_simu.setIdentity();

    // // Define a control vector and its noise and covariance in the tangent of SE2
    // SE2Tangentd u;                 // control signal, generic
    // SE2Tangentd u_nom;             // nominal control signal
    // ArrayT u_sigmas;               // control noise std specification
    // VectorT u_noise;               // control noise
    // MatrixT Q;                     // Covariance
    // MatrixT W;                     // sqrt Info
    // vector<SE2Tangentd> controls;  // robot controls

    // u_nom << 0.1, 0.0, 0.05;
    // u_sigmas << 0.01, 0.01, 0.01;
    // Q = (u_sigmas * u_sigmas).matrix().asDiagonal();
    // W = u_sigmas.inverse().matrix().asDiagonal();  // this is Q^(-T/2)

    // // Landmarks in R^2 and map
    // VectorB b;  // Landmark, generic
    // vector<VectorB> landmarks(NUM_LMKS), landmarks_simu;
    // {
    //     // Define five landmarks (beacons) in R^2
    //     VectorB b0, b1, b2, b3, b4;
    //     b0 << 3.0, 0.0;
    //     b1 << 2.0, -1.0;
    //     b2 << 2.0, 1.0;
    //     b3 << 3.0, -1.0;
    //     b4 << 3.0, 1.0;
    //     landmarks_simu.push_back(b0);
    //     landmarks_simu.push_back(b1);
    //     landmarks_simu.push_back(b2);
    //     landmarks_simu.push_back(b3);
    //     landmarks_simu.push_back(b4);
    // }  // destroy b0...b4

    // // Define the beacon's measurements in R^2
    // VectorY y, y_noise;
    // ArrayY y_sigmas;
    // MatrixY R;                                          // Covariance
    // MatrixY S;                                          // sqrt Info
    // vector<map<int, VectorY>> measurements(NUM_POSES);  // y = measurements[pose_id][lmk_id]

    // y_sigmas << 0.001, 0.001;
    // R = (y_sigmas * y_sigmas).matrix().asDiagonal();
    // S = y_sigmas.inverse().matrix().asDiagonal();  // this is R^(-T/2)

    // // Declare some temporaries
    // SE2Tangentd d;           // motion expectation d = Xj (-) Xi = Xj.minus ( Xi )
    // VectorY e;               // measurement expectation e = h(X, b)
    // MatrixT J_d_xi, J_d_xj;  // Jacobian of motion wrt poses i and j
    // MatrixT J_ix_x;          // Jacobian of inverse pose wrt pose
    // MatrixYT J_e_ix;         // Jacobian of measurement expectation wrt inverse pose
    // MatrixYT J_e_x;          // Jacobian of measurement expectation wrt pose
    // MatrixYB J_e_b;          // Jacobian of measurement expectation wrt lmk
    // SE2Tangentd dx;          // optimal pose correction step
    // VectorB db;              // optimal landmark correction step

    // // Problem-size variables
    // Matrix<double, NUM_STATES, 1> dX;        // optimal update step for all the SAM problem
    // Matrix<double, NUM_MEAS, NUM_STATES> J;  // full Jacobian
    // Matrix<double, NUM_MEAS, 1> r;           // full residual

    // /*
    //  * The factor graph of the SAM problem looks like this:
    //  *
    //  *                  ------- b1
    //  *          b3    /         |
    //  *          |   /       b4  |
    //  *          | /       /    \|
    //  *          X0 ---- X1 ---- X2
    //  *          | \   /   \   /
    //  *          |   b0      b2
    //  *          *
    //  *
    //  * where:
    //  *   - Xi are poses
    //  *   - bk are landmarks or beacons
    //  *   - * is a pose prior to anchor the map and make the problem observable
    //  *
    //  * Define pairs of nodes for all the landmark measurements
    //  * There are 3 pose nodes [0..2] and 5 landmarks [0..4].
    //  * A pair pose -- lmk means that the lmk was measured from the pose
    //  * Each pair declares a factor in the factor graph
    //  * We declare 9 pairs, or 9 factors, as follows:
    //  */
    // vector<list<int>> pairs(NUM_POSES);
    // pairs[0].push_back(0);  // 0-0
    // pairs[0].push_back(1);  // 0-1
    // pairs[0].push_back(3);  // 0-3
    // pairs[1].push_back(0);  // 1-0
    // pairs[1].push_back(2);  // 1-2
    // pairs[1].push_back(4);  // 1-4
    // pairs[2].push_back(1);  // 2-1
    // pairs[2].push_back(2);  // 2-2
    // pairs[2].push_back(4);  // 2-4

    // //
    // //
    // // END CONFIGURATION

    // //// Simulator ###################################################################
    // poses_simu.push_back(X_simu);
    // poses.push_back(Xi + SE2Tangentd::Random());  // use very noisy priors

    // // temporal loop
    // for (int i = 0; i < NUM_POSES; ++i) {
    //     // make measurements
    //     for (const auto& k : pairs[i]) {
    //         // simulate measurement
    //         b = landmarks_simu[k];                  // lmk coordinates in world frame
    //         y_noise = y_sigmas * ArrayY::Random();  // measurement noise
    //         y = X_simu.inverse().act(b);            // landmark measurement, before adding noise

    //         // add noise and compute prior lmk from prior pose
    //         measurements[i][k] = y + y_noise;      // store noisy measurements
    //         b = Xi.act(y + y_noise);               // mapped landmark with noise
    //         landmarks[k] = b + VectorB::Random();  // use very noisy priors //COMMENTS: Doesn't make complete sense, landmarks[k] get replaced again and again with respect to observation in last pose, which doesnt make complete sense. I mean, yes we did have a discussion about how to initialize landmarks and ended up with no specific answer, perhaps the same case here as well.
    //     }

    //     // make motions
    //     if (i < NUM_POSES - 1)  // do not make the last motion since we're done after 3rd pose
    //     {
    //         // move simulator, without noise
    //         X_simu = X_simu + u_nom;

    //         // move prior, with noise
    //         u_noise = u_sigmas * ArrayT::Random();
    //         Xi = Xi + (u_nom + u_noise);

    //         // store
    //         poses_simu.push_back(X_simu);
    //         poses.push_back(Xi + SE2Tangentd::Random());  // use very noisy priors
    //         controls.push_back(u_nom + u_noise);
    //     }
    // }

    // //// Estimator #################################################################

    // // DEBUG INFO
    // cout << "prior" << std::showpos << endl;
    // for (const auto& X : poses)
    //     cout << "pose  : " << X.translation().transpose() << " " << X.angle() << endl;
    // for (const auto& b : landmarks)
    //     cout << "lmk : " << b.transpose() << endl;
    // cout << "-----------------------------------------------" << endl;

    // // iterate
    // // DEBUG INFO
    // cout << "iterations" << std::noshowpos << endl;
    // for (int iteration = 0; iteration < MAX_ITER; ++iteration) {
    //     // Clear residual vector and Jacobian matrix
    //     r.setZero();
    //     J.setZero();

    //     // row and column for the full Jacobian matrix J, and row for residual r
    //     int row = 0, col = 0;

    //     // 1. evaluate prior factor ---------------------
    //     /*
    //      *  NOTE (see Chapter 2, Section E, of Sola-18):
    //      *
    //      *  To compute any residual, we consider the following variables:
    //      *      r: residual
    //      *      e: expectation
    //      *      y: prior specification 'measurement'
    //      *      W: sqrt information matrix of the measurement noise.
    //      *
    //      *  In case of a non-trivial prior measurement, we need to consider
    //      *  the nature of it: is it a global or a local specification?
    //      *
    //      *  When prior information `y` is provided in the global reference,
    //      *  we need a left-minus operation (.-) to compute the residual.
    //      *  This is usually the case for pose priors, since it is natural
    //      *  to specify position and orientation wrt a global reference,
    //      *
    //      *     r = W * (e (.-) y)
    //      *       = W * (e * y.inv).log()
    //      *
    //      *  When `y` is provided as a local reference, then right-minus (-.) is required,
    //      *
    //      *     r = W * (e (-.) y)
    //      *       = W * (y.inv * e).log()
    //      *
    //      *  Notice that if y = Identity() then local and global residuals are the same.
    //      *
    //      *
    //      *  Here, expectation, measurement and info matrix are trivial, as follows
    //      *
    //      *  expectation
    //      *     e = poses[0];            // first pose
    //      *
    //      *  measurement
    //      *     y = SE2d::Identity()     // robot at the origin
    //      *
    //      *  info matrix:
    //      *     W = I                    // trivial
    //      *
    //      *  residual uses left-minus since reference measurement is global
    //      *     r = W * (poses[0] (.-) measurement) = log(poses[0] * Id.inv) = poses[0].log()
    //      *
    //      *  Jacobian matrix :
    //      *     J_r_p0 = Jr_inv(log(poses[0]))         // see proof below
    //      *
    //      *     Proof: Let p0 = poses[0] and y = measurement. We have the partials
    //      *       J_r_p0 = W^(T/2) * d(log(p0 * y.inv)/d(poses[0])
    //      *
    //      *     with W = i and y = I. Since d(log(r))/d(r) = Jr_inv(r) for any r in the Lie algebra, we have
    //      *       J_r_p0 = Jr_inv(log(p0))
    //      */

    //     // residual and Jacobian.
    //     // Notes:
    //     //   We have residual = expectation - measurement, in global tangent space
    //     //   We have the Jacobian in J_r_p0 = J.block<DoF, DoF>(row, col);
    //     // We compute the whole in a one-liner:
    //     r.segment<DoF>(row) = poses[0].lminus(SE2d::Identity(), J.block<DoF, DoF>(row, col)).coeffs();

    //     // advance rows
    //     row += DoF;

    //     // loop poses
    //     for (int i = 0; i < NUM_POSES; ++i) {
    //         // 2. evaluate motion factors -----------------------
    //         if (i < NUM_POSES - 1)  // do not make the last motion since we're done after 3rd pose
    //         {
    //             int j = i + 1;  // this is next pose's id

    //             // recover related states and data
    //             Xi = poses[i];
    //             Xj = poses[j];
    //             u = controls[i];

    //             // expectation (use right-minus since motion measurements are local)
    //             d = Xj.rminus(Xi, J_d_xj, J_d_xi);  // expected motion = Xj (-) Xi

    //             // residual
    //             r.segment<DoF>(row) = W * (d - u).coeffs();  // residual

    //             // Jacobian of residual wrt first pose
    //             col = i * DoF;
    //             J.block<DoF, DoF>(row, col) = W * J_d_xi;

    //             // Jacobian of residual wrt second pose
    //             col = j * DoF;
    //             J.block<DoF, DoF>(row, col) = W * J_d_xj;

    //             // advance rows
    //             row += DoF;
    //         }

    //         // 3. evaluate measurement factors ---------------------------
    //         for (const auto& k : pairs[i]) {
    //             // recover related states and data
    //             Xi = poses[i];
    //             b = landmarks[k];
    //             y = measurements[i][k];

    //             // expectation
    //             e = Xi.inverse(J_ix_x).act(b, J_e_ix, J_e_b);  // expected measurement = Xi.inv * bj
    //             J_e_x = J_e_ix * J_ix_x;                       // chain rule

    //             // residual
    //             r.segment<Dim>(row) = S * (e - y);

    //             // Jacobian of residual wrt pose
    //             col = i * DoF;
    //             J.block<Dim, DoF>(row, col) = S * J_e_x;

    //             // Jacobian of residual wrt lmk
    //             col = NUM_POSES * DoF + k * Dim;
    //             J.block<Dim, Dim>(row, col) = S * J_e_b;

    //             // advance rows
    //             row += Dim;
    //         }
    //     }

    //     // 4. Solve -----------------------------------------------------------------

    //     // compute optimal step
    //     // ATTENTION: This is an expensive step!!
    //     // ATTENTION: Use QR factorization and column reordering for larger problems!!
    //     dX = -(J.transpose() * J).inverse() * J.transpose() * r;

    //     // update all poses
    //     for (int i = 0; i < NUM_POSES; ++i) {
    //         // we go very verbose here
    //         int row = i * DoF;
    //         constexpr int size = DoF;
    //         dx = dX.segment<size>(row);
    //         poses[i] = poses[i] + dx;
    //     }

    //     // update all landmarks
    //     for (int k = 0; k < NUM_LMKS; ++k) {
    //         // we go very verbose here
    //         int row = NUM_POSES * DoF + k * Dim;
    //         constexpr int size = Dim;
    //         db = dX.segment<size>(row);
    //         landmarks[k] = landmarks[k] + db;
    //     }

    //     // DEBUG INFO
    //     cout << "residual norm: " << std::scientific << r.norm() << ", step norm: " << dX.norm() << endl;

    //     // conditional exit
    //     if (dX.norm() < 1e-6) break;
    // }
    // cout << "-----------------------------------------------" << endl;

    // //// Print results ####################################################################

    // cout << std::fixed;

    // // solved problem
    // cout << "posterior" << std::showpos << endl;
    // for (const auto& X : poses)
    //     cout << "pose  : " << X.translation().transpose() << " " << X.angle() << endl;
    // for (const auto& b : landmarks)
    //     cout << "lmk : " << b.transpose() << endl;
    // cout << "-----------------------------------------------" << endl;

    // // ground truth
    // cout << "ground truth1" << std::showpos << endl;
    // for (const auto& X : poses_simu)
    //     cout << "pose  : " << X.translation().transpose() << " " << X.angle() << endl;
    // for (const auto& b : landmarks_simu)
    //     cout << "lmk : " << b.transpose() << endl;
    // cout << "-----------------------------------------------" << endl;

    return 0;
}
