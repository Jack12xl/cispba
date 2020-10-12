#include "main.h"


void dumpObj(
    const std::string& filename,
    std::vector<TV> _x
)
{
    std::ofstream fs;
    fs.open(filename);

    // add vertex 
    
    for (auto X : _x) {
        fs << "v" ;
        for (int i = 0; i < dim; i++)
            fs << " " << X(i);
        if (dim == 2)
            fs << " 0";
        fs << "\n";
    }

    for (int i = 0; i < X_RES; i++) {
        for (int j = 0; j < Y_RES; j++) {
            fs << "f";
            fs << " " << i * Y_RES + j ;
            fs << " " << (i + 1) * Y_RES + j;
            fs << " " << (i + 1) * Y_RES + j + 1;
            fs << " " << i * Y_RES + j + 1;
            fs << "\n";
        }
    }
    fs << "##End of files";
    fs.close();
    
}

void read_point(
    const std::string & file_path,
    std::vector<TV>& m_x
) {
    std::ifstream fp_in;
    int numFace, cur_dim;

    fp_in.open(file_path);
    if (!fp_in.is_open()) {
        std::cout << "Error reading from file - aborting!" << std::endl;
        throw;
    }

    int line_num = 0;
    while (fp_in.good()) {
        std::string line;
        utility::safeGetline(fp_in, line);

        if (!line.empty()) {
            std::vector<std::string> tokens = utility::tokenizeString(line);

            if (line_num == 0) {
                numFace = atoi(tokens[0].c_str());
                cur_dim = atoi(tokens[1].c_str());
                std::cout << "In total " << numFace << " faces."<< std::endl;
                
            }
            else {
                TV cur_x;
                /*for (int i = 0; i < dim; i++) {
                    cur_x << atoi(tokens[i].c_str());
                }*/
                cur_x <<
                    atoi(tokens[0].c_str()),
                    atoi(tokens[1].c_str()),
                    atoi(tokens[2].c_str());

                m_x.push_back(cur_x);
            }
            line_num++;
        }

    }
    assert(line_num - 1 == m_x.size());
    assert(cur_dim == dim);
}

struct seg_cmp {
    bool operator() (
        const Eigen::Matrix<int, 2, 1>& lhs,
        const Eigen::Matrix<int, 2, 1>& rhs
        ) const {
        return lhs(0) < rhs(0) && lhs(1) < rhs(1);
    }
};

void read_cell(
    const std::string& file_path,
    std::vector<Eigen::Matrix<int, 2, 1> >& _segments,
    const std::vector<TV>& _x,
    std::vector<T>& _rest_length
) {
    std::ifstream fp_in;
    int numTetrahedra, dim;

    fp_in.open(file_path);
    if (!fp_in.is_open()) {
        std::cout << "Error reading from file - aborting!" << std::endl;
        throw;
    }

    std::set< Eigen::Matrix<int, 2, 1>, seg_cmp > m_set;

    int line_num = 0;
    while (fp_in.good()) {
        std::string line;
        utility::safeGetline(fp_in, line);

        if (!line.empty()) {
            std::vector<std::string> tokens = utility::tokenizeString(line);

            if (line_num == 0) {
                numTetrahedra = atoi(tokens[0].c_str());
                dim = atoi(tokens[1].c_str());
                std::cout << "In all " << numTetrahedra << " tetrahedras." << std::endl;

            }
            else {
                Eigen::Matrix<int, 2, 1> cur_seg = Eigen::Matrix<int, 2, 1>::Zero();
                
                // insert the bunny structure 
                // a tetra
                for (int i = 0; i < dim ; i++) {
                    int p_0 = atoi(tokens[i].c_str());
                    int p_1 = atoi(tokens[(i + 1) % dim].c_str());
                    
                    cur_seg(0) = std::min(p_0, p_1);
                    cur_seg(1) = std::max(p_0, p_1);
                    
                    auto insert_result = m_set.insert(cur_seg);
                    if (insert_result.second) {
                        _rest_length.emplace_back((_x[p_0] - _x[p_1]).norm());
                    }
                }
            }
            line_num++;
        }

    }
    _segments.assign( m_set.begin(), m_set.end() );
    assert(_segments.size() <= line_num);
    std::cout << "In all " << _segments.size() << " segments !" << std::endl;
}



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
#pragma region node_data_fill_segments
        assert(X_RES >= 2);
        assert(Y_RES >= 2);

        m.resize(numPoint, uniform_M);
        
        v.resize(numPoint, TV::Zero());
        x.resize(numPoint, TV::Zero());
        for (int i = 0; i < X_RES; i++) {
            for (int j = 0; j < Y_RES; j++) {
                int idx = i * Y_RES + j;
                x[idx](0) = X_offset + (float)i * X_d;
                x[idx](1) = Y_offset + (float)j * Y_d;
                x[idx](2) = Z_offset;
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
                rest_length.emplace_back( ( x[p1] - x[p2] ).norm() );
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
#pragma endregion
        //youngs_modulus = 1024.0f;
        //damping_coeff = 0.04;
        //dt = 0.005f;

        // boundary
        // hold two up corner
        node_is_fixed[Y_RES - 1] = true;
        node_is_fixed[X_RES * Y_RES - 1 ] = true;
        
        // TODO Generate quad mesh for rendering.
        dumpObj("cloth.obj", x);

        const float x_right_up_offset = x[X_RES * Y_RES - 1](0);
        driver.helper = [&](T t, T dt) {
            // TODO
            TV& p_left_up = driver.ms.x[Y_RES - 1];
            TV& p_right_up = driver.ms.x[X_RES * Y_RES - 1];
#if Z_move
            p_left_up(2) = Z_amplitude * sin(Z_omega * t);
            p_right_up(2) = Z_amplitude * sin(Z_omega * t);
#endif // Z_move

            
#if X_move
            p_left_up(0) = X_amplitude * sin(X_amplitude * t);
            p_right_up(0) = X_amplitude * sin(X_amplitude * (-t)) + x_right_up_offset;
#endif
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
        // 1, 2
        read_point("E:\\Jack12\\cis563-pba\\proj1_explicit_mass_spring\\Projects\\mass_spring\\data\\points", x);
        read_cell("E:\\Jack12\\cis563-pba\\proj1_explicit_mass_spring\\Projects\\mass_spring\\data\\cells", 
            segments, 
            x,
            rest_length);

        numPoint = x.size();
        v.resize(numPoint, TV::Zero());
        m.resize(numPoint, uniform_M);

        node_is_fixed.resize(numPoint, false);
        node_is_fixed[0] = true;

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

    driver.run(total_sequence_num);

    return 0;
}
