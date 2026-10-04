#include <physics/physics.h>
#include <cmath>
#include <iomanip>
#include <iostream>
int main() {
    try {
        using namespace PhysicsEngine;
        constexpr double pi=3.14159265358979323846, duration=.4, u=.7, v=-.2;
        std::cout<<std::setprecision(17);
        for(const std::size_t n:{24,48,96}) {
            PeriodicScalarGridConfig c{n,n/2,1.0/n,2.0/n}; PeriodicScalarTransport grid(c);
            auto q=grid.state();
            const double factor=std::sin(pi*c.spacingX)/(pi*c.spacingX)*std::sin(2*pi*c.spacingY)/(2*pi*c.spacingY);
            for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i)
                q[i+n*j]=1+.3*factor*std::cos(2*pi*((i+.5)*c.spacingX+2*(j+.5)*c.spacingY));
            grid.setState(q); grid.setVelocities({std::vector<double>(q.size(),u),std::vector<double>(q.size(),v)});
            ScalarTransportConfig options; options.maxSubstep=duration/n;
            const auto d=grid.step(duration,options); const auto actual=grid.state(); double square=0;
            for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
                const double exact=1+.3*factor*std::cos(2*pi*((i+.5)*c.spacingX+2*(j+.5)*c.spacingY-(u+2*v)*duration));
                square+=std::pow(actual[i+n*j]-exact,2);
            }
            std::cout<<"{\"columns\":"<<n<<",\"rows\":"<<n/2<<",\"duration\":"<<duration
                <<",\"substeps\":"<<d.substeps<<",\"cell_visits\":"<<d.cellVisits<<",\"maximum_cfl\":"<<d.maximumCfl
                <<",\"integral_before\":"<<d.initialIntegratedScalar<<",\"integral_after\":"<<d.finalIntegratedScalar
                <<",\"integral_drift\":"<<d.integratedScalarDrift<<",\"conservation_allowance\":"<<d.conservationRoundoffAllowance
                <<",\"minimum\":"<<d.finalMinimum<<",\"maximum\":"<<d.finalMaximum
                <<",\"continuous_l2_error\":"<<std::sqrt(square/q.size())<<"}\n";
        }
        return 0;
    } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
