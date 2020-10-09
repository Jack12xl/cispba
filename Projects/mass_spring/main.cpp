#include "main.h"


int main(int argc, char* argv[])
{
    

    SimulationDriver<T,dim> driver;

    // node data
    std::vector<T> m;
    std::vector<TV> x;
    std::vector<TV> v;
    std::vector<bool> node_is_fixed;

    // segment data
    std::vector<Eigen::Matrix<int,2,1> > segments;
    std::vector<T> rest_length;

    if (argc < 2) 
    {
        std::cout << "Please indicate test case number: 0 (cloth) or 1 (volumetric bunny)" << std::endl;
        exit(0);
    }

    if (strcmp(argv[1], "0") == 0) // cloth case
    {
        // TODO
        /* 
            1. Create node data: position, mass, velocity
            2. Fill segments and rest_length, including struct springs, shearing springs and bending springs.
            3. Choose proper youngs_modulus, damping_coeff and dt.
            4. Set boundary condition (node_is_fixed) and helper function (to achieve moving boundary condition).
            5. Generate quad mesh for rendering.
        */
        //1. Create node data : position, mass, velocity
        assert(X_RES >= 2);
        assert(Y_RES >= 2);

        m.resize(numPoint, uniform_M);
        
        v.resize(numPoint, TV::Zero());
        x.resize(numPoint, TV::Zero());
        for (int i = 0; i < X_RES; i++) {
            for (int j = 0; j < Y_RES; j++) {
                int idx = i * Y_RES + j;
                x[idx](0) = (float)i;
                x[idx](1) = (float)j;
                x[idx](2) = 0.0f;
            }
        }

        node_is_fixed.resize(numPoint, false);
#pragma region addSpring
        // vertical struct spring
        for (int i = 0; i < X_RES - 1; i++) {
            for (int j = 0; j < Y_RES; j++) {
                Eigen::Matrix<int, 2, 1> cur_seg;
                int p1 = i * Y_RES + j, p2 = (i + 1) * Y_RES + j;
                cur_seg << p1, p2;
                segments.emplace_back(cur_seg);
                rest_length.emplace_back( (x[p1] - x[p2] ).norm());
            }
        }
        // horizontal struct spring
        for (int i = 0; i < X_RES ; i++) {
            for (int j = 0; j < Y_RES - 1; j++) {
                Eigen::Matrix<int, 2, 1> cur_seg;
                int p1 = i * Y_RES + j, p2 = i * Y_RES + j + 1;
                cur_seg << p1, p2;
                segments.emplace_back(cur_seg);
                rest_length.emplace_back((x[p1] - x[p2]).norm());
            }
        }

        // up left 2 bottom right shear
        for (int i = 0; i < X_RES - 1; i++) {
            for (int j = 0; j < Y_RES - 1; j++) {
                Eigen::Matrix<int, 2, 1> cur_seg;
                int p1 = i * Y_RES + j, p2 = (i + 1) * Y_RES + j + 1;
                cur_seg << p1, p2;
                segments.emplace_back(cur_seg);
                rest_length.emplace_back((x[p1] - x[p2]).norm());
            }
        }

        // bottom left 2 up right  shear
        for (int i = 1; i < X_RES; i++) {
            for (int j = 0; j < Y_RES - 1; j++) {
                Eigen::Matrix<int, 2, 1> cur_seg;
                int p1 = i * Y_RES + j, p2 = (i - 1) * Y_RES + j + 1;
                cur_seg << p1, p2;
                segments.emplace_back(cur_seg);
                rest_length.emplace_back((x[p1] - x[p2]).norm());
            }
        }

        // bent horizontal
        for (int i = 0; i < X_RES ; i++) {
            for (int j = 0; j < Y_RES - 2; j++) {
                Eigen::Matrix<int, 2, 1> cur_seg;
                int p1 = i * Y_RES + j, p2 = i  * Y_RES + j + 2;
                cur_seg << p1, p2;
                segments.emplace_back(cur_seg);
                rest_length.emplace_back((x[p1] - x[p2]).norm());
            }
        }

        for (int i = 0; i < X_RES - 2; i++) {
            for (int j = 0; j < Y_RES; j++) {
                Eigen::Matrix<int, 2, 1> cur_seg;
                int p1 = i * Y_RES + j, p2 = (i + 2) * Y_RES + j;
                cur_seg << p1, p2;
                segments.emplace_back(cur_seg);
                rest_length.emplace_back((x[p1] - x[p2]).norm());
            }
        }
#pragma endregion

        driver.helper = [&](T t, T dt) {
            // TODO
        };
        driver.test="cloth";
    }

    else if (strcmp(argv[1], "1") == 0) // volumetric bunny case
    { 
        // TODO
        /* 
            1. Create node data from data/points: The first line indicates the number of points and dimension (which is 3). 
            2. Fill segments and rest_length from data/cells: The first line indicates the number of tetrahedra and the number of vertices of each tet (which is 6). Each edge in this tetrahedral mesh will be a segment. Be careful not to create duplicate edges. 
            3. Choose proper youngs_modulus, damping_coeff, dt; 
            4. Set boundary condition (node_is_fixed) and helper function (to achieve moving boundary condition).
        */

        driver.helper = [&](T t, T dt) {
            // TODO
        };
        driver.test="bunny";
    }

    else {
        std::cout << "Wrong case number!" << std::endl;
        exit(0);
    }

    // simulate
    
    driver.dt = dt;
    driver.ms.segments = segments;
    driver.ms.m = m;
    driver.ms.v = v;
    driver.ms.x = x;
    driver.ms.youngs_modulus = youngs_modulus;
    driver.ms.damping_coeff = damping_coeff;
    driver.ms.node_is_fixed = node_is_fixed;
    driver.ms.rest_length = rest_length;

    driver.run(120);

    return 0;
}
