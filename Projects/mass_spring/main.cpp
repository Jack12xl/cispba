#include "main.h"

#pragma region read_from_hw1
void read_point(
    const std::string& file_path,
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
                std::cout << "In total " << numFace << " faces." << std::endl;

            }
            else {
                TV cur_x;
                /*for (int i = 0; i < dim; i++) {
                    cur_x << atoi(tokens[i].c_str());
                }*/
                cur_x <<
                    atof(tokens[0].c_str()),
                    atof(tokens[1].c_str()),
                    atof(tokens[2].c_str());

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
        return lhs(0) < rhs(0) || (lhs(0) == rhs(0) && lhs(1) < rhs(1));
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
    // do not want to introduce hash function, so I use m_set
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
                for (int i = 0; i < dim; i++) {
                    int p_0 = atoi(tokens[i].c_str());
                    int p_1 = atoi(tokens[(i + 1) % dim].c_str());

                    cur_seg(0) = std::min(p_0, p_1);
                    cur_seg(1) = std::max(p_0, p_1);

                    auto insert_result = m_set.insert(cur_seg);
                    if (insert_result.second) {
                        _segments.emplace_back(cur_seg);
                        _rest_length.emplace_back((_x[p_0] - _x[p_1]).norm());
                    }
                }
            }
            line_num++;
        }

    }
    //_segments.assign( m_set.begin(), m_set.end() );
    std::cout << "In all " << _segments.size() << " segments !" << std::endl;
}

#pragma endregion

int main(int argc, char* argv[])
{
    

    SimulationDriver<T,dim> driver;

    // set up mass spring system
    T youngs_modulus = 0;
    T damping_coeff = 0; 
    T dt = T(1/24.);

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
        std::cout << "Please indicate test case number: 1 (bunny) or 2 (brush)" << std::endl;
        exit(0);
    }

    if (strcmp(argv[1], "1") == 0) { // bunny case
        for (T youngs_modulus : m_bunny_young_modules) {
            m.clear();
            x.clear();
            v.clear();
            node_is_fixed.clear();

            segments.clear();
            rest_length.clear();

            //youngs_modulus = 0.1; // TODO: iterate in [0.1, 1, 10, 100, 1000]
            damping_coeff = 2;
            // TODO: 
            /*
                1. Copy the loading codes from your hw1. Fix two ears (2140, 2346) only, and you don't need helper function here.
                2. Set the initial velocities of non_fixed_nodes to be (10, 0, 0)

                The output folder will automatically renamed by bunny_[youngs_modulus], don't worry about overwriting.
            */
            // 1. Copy the loading codes from your hw1.Fix two ears(2140, 2346) only, and you don't need helper function here.
            read_point("E:\\Jack12\\cis563-pba\\proj1_explicit_mass_spring\\Projects\\mass_spring\\data\\points",
                x);
            read_cell("E:\\Jack12\\cis563-pba\\proj1_explicit_mass_spring\\Projects\\mass_spring\\data\\cells",
                segments,
                x,
                rest_length);

            // 2. Set the initial velocities of non_fixed_nodes to be (10, 0, 0)
            numPoint = x.size();
            TV v_non_fixed_nodes;
            v_non_fixed_nodes << 10, 0, 0;

            node_is_fixed.resize(numPoint, false);
            v.resize(numPoint, v_non_fixed_nodes);
            v[2140] = TV::Zero();
            v[2346] = TV::Zero();
            node_is_fixed[2140] = true;
            node_is_fixed[2346] = true;
            m.resize(numPoint, total_M / numPoint);

            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << youngs_modulus;
            driver.test = "bunny_" + ss.str();

            driver.dt = dt;
            driver.ms.segments = segments;
            driver.ms.m = m;
            driver.ms.v = v;
            driver.ms.x = x;
            driver.ms.target_x = x;
            driver.ms.youngs_modulus = youngs_modulus;
            driver.ms.damping_coeff = damping_coeff;
            driver.ms.node_is_fixed = node_is_fixed;
            driver.ms.rest_length = rest_length;

            std::cout << "Run for Youngs module: " << driver.ms.youngs_modulus << std::endl;
            driver.run(5);
        }
        
       
    }

    else if (strcmp(argv[1], "2") == 0) { //brush case
        driver.gravity.setZero();
        driver.collision_stiffness = 0.1;
        youngs_modulus = 10000;
        damping_coeff = 100; // 0
        int N = 32; // z direction
        int M = 4; // y direction
        int L = 32; // x direction
        int N_points = N*M*L;
        T dx = (T)0.1/(N-1);
        m.resize(N_points);
        x.resize(N_points);
        v.resize(N_points);
        node_is_fixed = std::vector<bool>(N_points, false);
        for(int i=0; i<N; i++){ // z
            for(int j=0; j<M; j++) { // y
                for (int k=0; k<L; k++) { // x
                    int id = i * M * L + j * L + k;
                    m[id] = (T)0.001/N_points;
                    x[id](2) = i*dx;
                    x[id](1) = j*dx;
                    x[id](0) = k*dx;
                    v[id] = TV::Zero();
                    if (k <= 2) node_is_fixed[id] = true;
                    // struct spring
                    if (k > 0) { 
                        segments.push_back(Eigen::Matrix<int,2,1>(id, id-1));
                        rest_length.push_back((x[id]-x[id-1]).norm());
                    }
                    // bending spring
                    if (k > 1) {
                        segments.push_back(Eigen::Matrix<int,2,1>(id, id-2));
                        rest_length.push_back((x[id]-x[id-2]).norm());
                    }
                }
            }
        }

        driver.sphere_radius = 0.04;
        driver.sphere_center = TV(0.07, -0.045, 0.05);

        driver.helper = [&](T t, T dt) {
            if(t < 4) {
                // driver.sphere_center = TV(0.12, t/4. * 0.15 + (1-t/4.) * (-0.06), 0.05);
                for(size_t i = 0; i < driver.ms.x.size(); ++i) {
                    if (driver.ms.node_is_fixed[i]) {
                        driver.ms.target_x[i](1) -= 0.15 / 4 * dt;
                    }
                }
            }
        };

        driver.test="brush";

        driver.dt = dt;
        driver.ms.segments = segments;
        driver.ms.m = m;
        driver.ms.v = v;
        driver.ms.x = x;
        driver.ms.target_x = x;
        driver.ms.youngs_modulus = youngs_modulus;
        driver.ms.damping_coeff = damping_coeff;
        driver.ms.node_is_fixed = node_is_fixed;
        driver.ms.rest_length = rest_length;

        driver.run(20);
    }

    else {
        std::cout << "Wrong case number!" << std::endl;
        exit(0);
    }

    // simulate
    /*driver.dt = dt;
    driver.ms.segments = segments;
    driver.ms.m = m;
    driver.ms.v = v;
    driver.ms.x = x;
    driver.ms.target_x = x;
    driver.ms.youngs_modulus = youngs_modulus;
    driver.ms.damping_coeff = damping_coeff;
    driver.ms.node_is_fixed = node_is_fixed;
    driver.ms.rest_length = rest_length;

    driver.run(5);*/
   

    return 0;
}
