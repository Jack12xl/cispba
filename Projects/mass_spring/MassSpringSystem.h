#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>
#include <string>
#include <fstream>

enum class updateScheme {
    ForwardEuler,
    SymplecticEuler,
    BackwardEuler
};

template<class T, int dim>
class MassSpringSystem{
public:
    using TV = Eigen::Matrix<T,dim,1>;
    
    std::vector<Eigen::Matrix<int,2,1> > segments;
    std::vector<T> m;
    std::vector<TV> x;
    std::vector<TV> v;
    T youngs_modulus;
    T damping_coeff;
    std::vector<bool> node_is_fixed;
    std::vector<T> rest_length;

    updateScheme m_method;

    std::vector<TV> f_spring;
    std::vector<TV> f_damping;

    MassSpringSystem()
    {
        this -> m_method = updateScheme::ForwardEuler;
    }

    void evaluateSpringForces(std::vector<TV >& f)
    {
        // TODO: evaluate spring force
        //init 
        f.resize( m.size(), TV::Zero() );
        
        assert(segments.size() == rest_length.size());
        for (int i = 0; i < segments.size(); i ++) {
            int p1 = segments[i](0), p2 = segments[i](1);

            T l_0 = rest_length[i];
            T l = (x[p1] - x[p2]).norm();
            TV n_12 = ( x[p1] - x[p2] ).normalized();
            
            TV tmp = -youngs_modulus * (l / l_0 - 1.0f) * n_12;
            f[p1] += tmp;
            f[p2] += -tmp;
        }
    }

    void evaluateDampingForces(std::vector<TV >& f)
    {
        // TODO: evaluate damping force

        f.resize(m.size(), TV::Zero());

        assert(segments.size() == rest_length.size());
        for (int i = 0; i < segments.size(); i++) {
            int p1 = segments[i](0), p2 = segments[i](1);

            TV n_12 = (x[p1] - x[p2]).normalized();
            TV v_rel = (v[p1] - v[p2]).cwiseProduct(n_12);

            TV tmp = -damping_coeff * v_rel.cwiseProduct(n_12);
            f[p1] += tmp;
            f[p2] += -tmp;
        }
    }

    void dumpPoly(std::string filename)
    {
        std::ofstream fs;
        fs.open(filename);
        fs << "POINTS\n";
        int count = 0;
        for (auto X : x) {
            fs << ++count << ":";
            for (int i = 0; i < dim; i++)
                fs << " " << X(i);
            if (dim == 2)
                fs << " 0";
            fs << "\n";
        }
        fs << "POLYS\n";
        count = 0;
        for (const Eigen::Matrix<int, 2, 1>& seg : segments)
            fs << ++count << ": " << seg(0) + 1 << " " << seg(1) + 1 << "\n"; // poly segment mesh is 1-indexed
        fs << "END\n";
        fs.close();
    }

    void SubstepUpdate(const std::vector<TV >& f_spring, const std::vector<TV >& f_damping, const TV& gravity) {
        
        int num_p = x.size();
        assert(f_spring.size() == num_p);
        assert(f_damping.size() == num_p);
        for (int p = 0; p < num_p; p++) {
            if (this->node_is_fixed[p]) {
                this->v[p] = TV::Zero();
            }
            else {

                if (m_method == updateScheme::ForwardEuler) {
                    this->x[p] += this->v[p] * dt;
                    this->v[p] += ((f_spring[p] + f_damping[p]) / m[p] + gravity) * dt;
                }
                else if (m_method == updateScheme::SymplecticEuler) {
                    this->v[p] += ((f_spring[p] + f_damping[p]) / m[p] + gravity) * dt;
                    this->x[p] += this->v[p] * dt;
                }

                
            }
        }       
    }
};
