#include "physics/core/fluids/periodic_scalar_transport.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace PhysicsEngine {
namespace {
double Checked(double value) {
    if (!std::isfinite(value)) throw std::overflow_error("Scalar transport arithmetic exceeds float64 range.");
    return value;
}
// Stage exponents so a finite complete product/quotient does not depend on
// avoidable overflow of its intermediate factors. Nonzero underflow is rejected.
double Scaled(double a, double b=1, double c=1, double divisor=1) {
    if (a==0 || b==0 || c==0) return 0;
    int ea=0, eb=0, ec=0, ed=0;
    const double m=std::frexp(a,&ea)*std::frexp(b,&eb)*std::frexp(c,&ec)/std::frexp(divisor,&ed);
    const double result=Checked(std::ldexp(m,ea+eb+ec-ed));
    if (result==0) throw std::overflow_error("Scalar transport nonzero arithmetic underflows float64 range.");
    return result;
}
double Normalized(double value, double scale) {
    const double result=Checked(value/scale);
    if (value!=0 && result==0) throw std::overflow_error("Scalar transport dynamic range loses a nonzero normalized value.");
    return result;
}
double DifferenceRate(double a, double b, double spacing) {
    const double difference=a-b;
    // Subtract stored face values first: separate divisions can erase a small
    // represented difference on top of a large common velocity.
    return std::isfinite(difference) ? Scaled(difference,1,1,spacing)
        : Checked(Scaled(a,1,1,spacing)-Scaled(b,1,1,spacing));
}
void Add(double& sum, double& correction, double value) {
    const double next=Checked(sum+value);
    correction=Checked(correction+(std::abs(sum)>=std::abs(value)
        ? (sum-next)+value : (value-next)+sum));
    sum=next;
}
double UpperAdd(double sum, double value) {
    if (value==0) return sum;
    return Checked(std::nextafter(Checked(sum+value),std::numeric_limits<double>::infinity()));
}
struct Summary {
    double integral=0, absolute=0, minimum=0, maximum=0, scale=0;
};
Summary Summarize(const std::vector<double>& q, double area) {
    Summary result;
    result.minimum=result.maximum=q[0];
    for (const double value:q) {
        result.minimum=std::min(result.minimum,value); result.maximum=std::max(result.maximum,value);
        result.scale=std::max(result.scale,std::abs(value));
    }
    const double scale=result.scale==0 ? 1 : result.scale;
    double sum=0, correction=0, absolute=0, absoluteCorrection=0;
    for (const double value:q) {
        const double normalized=Normalized(value,scale);
        Add(sum,correction,normalized); Add(absolute,absoluteCorrection,std::abs(normalized));
    }
    result.integral=Scaled(Checked(sum+correction),result.scale,area);
    result.absolute=Scaled(Checked(absolute+absoluteCorrection),result.scale,area);
    return result;
}
void ValidateValues(const std::vector<double>& values, std::size_t size) {
    if (values.size()!=size) throw std::invalid_argument("Scalar transport state size does not match the grid.");
    for (const double value:values)
        if (!std::isfinite(value)) throw std::invalid_argument("Scalar transport values must be finite.");
}
double Updated(double old, double normalizedIncrement, double scale) {
    if (normalizedIncrement==0) return old; // Avoid a needless normalize/rescale round trip.
    // A signed increment can overflow even when cancellation with old leaves a
    // finite new state. Only that opposite-sign case needs normalized addition.
    if (std::abs(normalizedIncrement)>1 && scale>std::numeric_limits<double>::max()/std::abs(normalizedIncrement))
        return Scaled(Checked(Normalized(old,scale)+normalizedIncrement),scale);
    return Checked(old+Scaled(normalizedIncrement,scale));
}
} // namespace

void PeriodicScalarGridConfig::Validate() const {
    if (columns<2 || rows<2 || columns>MaximumCells || rows>MaximumCells || columns>MaximumCells/rows)
        throw std::invalid_argument("Periodic scalar grid dimensions exceed supported bounds.");
    if (!std::isfinite(spacingX) || spacingX<=0 || !std::isfinite(spacingY) || spacingY<=0)
        throw std::invalid_argument("Scalar grid spacings must be finite and positive.");
    const auto area=Scaled(spacingX,spacingY);
    const auto width=Scaled(spacingX,double(columns)), height=Scaled(spacingY,double(rows));
    if (area<=0 || width<=0 || height<=0) throw std::invalid_argument("Scalar grid geometry is unrepresentable.");
}
void ScalarTransportConfig::Validate() const {
    if (!std::isfinite(cflSafety) || cflSafety<=0 || cflSafety>=1 || !std::isfinite(maxSubstep) || maxSubstep<=0)
        throw std::invalid_argument("Scalar transport requires strict (0,1) CFL safety and positive finite maxSubstep.");
    if (maximumSubsteps>MaximumSubsteps || maximumCellVisits>MaximumCellVisits)
        throw std::invalid_argument("Scalar transport work budgets exceed hard ceilings.");
}
PeriodicScalarTransport::PeriodicScalarTransport(const PeriodicScalarGridConfig& config):config_(config) {
    config_.Validate();
    const auto n=config_.columns*config_.rows;
    scalar_.assign(n,0); velocities_.xFaces.assign(n,0); velocities_.yFaces.assign(n,0);
}
void PeriodicScalarTransport::setState(const std::vector<double>& scalar) {
    ValidateValues(scalar,scalar_.size());
    auto staged=scalar; scalar_.swap(staged);
}
void PeriodicScalarTransport::setVelocities(const MacVelocityState& velocities) {
    ValidateValues(velocities.xFaces,scalar_.size()); ValidateValues(velocities.yFaces,scalar_.size());
    auto staged=velocities; velocities_=std::move(staged);
}
ScalarTransportDiagnostics PeriodicScalarTransport::step(double duration,const ScalarTransportConfig& options) {
    options.Validate();
    if (!std::isfinite(duration) || duration<0) throw std::invalid_argument("Scalar transport duration must be finite and nonnegative.");
    const auto n=scalar_.size(), nx=config_.columns, ny=config_.rows;
    if (options.maximumCellVisits/n<3) throw std::runtime_error("Scalar transport cell-visit budget exhausted.");
    ScalarTransportDiagnostics d;
    d.duration=duration; d.timeBefore=time_; d.timeAfter=Checked(time_+duration);
    if (duration>0 && d.timeAfter<=time_) throw std::overflow_error("Scalar transport time increment is unrepresentable.");
    d.discreteDivergenceFree=true;
    for (std::size_t j=0;j<ny;++j) for (std::size_t i=0;i<nx;++i) {
        const auto k=i+nx*j, right=(i+1)%nx+nx*j, up=i+nx*((j+1)%ny);
        const double rates[]={Scaled(std::max(velocities_.xFaces[right],0.0),1,1,config_.spacingX),
            Scaled(std::max(-velocities_.xFaces[k],0.0),1,1,config_.spacingX),
            Scaled(std::max(velocities_.yFaces[up],0.0),1,1,config_.spacingY),
            Scaled(std::max(-velocities_.yFaces[k],0.0),1,1,config_.spacingY)};
        double outward=0, upper=0;
        for (double rate:rates) { outward=Checked(outward+rate); upper=UpperAdd(upper,rate); }
        d.maximumOutflowRate=std::max(d.maximumOutflowRate,outward); d.outflowRateBound=std::max(d.outflowRateBound,upper);
        const double divergence=Checked(DifferenceRate(velocities_.xFaces[right],velocities_.xFaces[k],config_.spacingX)
            +DifferenceRate(velocities_.yFaces[up],velocities_.yFaces[k],config_.spacingY));
        d.maximumAbsDivergence=std::max(d.maximumAbsDivergence,std::abs(divergence));
        d.discreteDivergenceFree=d.discreteDivergenceFree && divergence==0;
    }
    const double area=Scaled(config_.spacingX,config_.spacingY);
    const auto initial=Summarize(scalar_,area);
    d.initialIntegratedScalar=initial.integral; d.initialAbsoluteIntegral=initial.absolute;
    d.initialMinimum=initial.minimum; d.initialMaximum=initial.maximum; d.nonnegativeInput=initial.minimum>=0;
    if (duration==0) {
        d.zeroDurationNoOp=true; d.cellVisits=3*n;
        d.finalIntegratedScalar=initial.integral; d.finalAbsoluteIntegral=initial.absolute;
        d.finalMinimum=initial.minimum; d.finalMaximum=initial.maximum;
        diagnostics_=d; return d;
    }
    double limit=options.maxSubstep;
    if (d.outflowRateBound>0) {
        const double cflLimit=options.cflSafety/d.outflowRateBound;
        if (std::isfinite(cflLimit)) limit=std::min(limit,std::nextafter(cflLimit,0.0));
    }
    if (limit<=0) throw std::overflow_error("Scalar transport stable substep is unrepresentable.");
    // Begin at floor and inspect actual h. An upward-rounded quotient near an
    // integer must not charge an unnecessary extra step when duration/count is
    // already within the representable limit.
    const double count=std::floor(duration/limit);
    if (!std::isfinite(count) || options.maximumSubsteps==0 || count>double(options.maximumSubsteps))
        throw std::runtime_error("Scalar transport substep budget exhausted.");
    d.substeps=std::max<std::size_t>(1,static_cast<std::size_t>(count));
    double h=duration/d.substeps;
    while (h>limit) {
        if (d.substeps==options.maximumSubsteps) throw std::runtime_error("Scalar transport partition exceeds substep budget.");
        h=duration/++d.substeps;
    }
    if (h<=0) throw std::overflow_error("Scalar transport partition underflows.");
    if (options.maximumCellVisits/n<7+4*d.substeps)
        throw std::runtime_error("Scalar transport cell-visit budget exhausted.");
    d.cellVisits=(7+4*d.substeps)*n; d.lastSubstep=h;
    d.maximumCfl=Scaled(h,d.outflowRateBound);
    if (d.maximumCfl>options.cflSafety) throw std::runtime_error("Scalar transport CFL partition exceeds safety.");
    // All work/count checks above precede scratch allocation. The fixed scale
    // keeps pair transfers representable across the supplied scalar magnitudes.
    const double scale=initial.scale==0 ? 1 : initial.scale;
    for (double q:scalar_) (void)Normalized(q,scale);
    auto staged=scalar_;
    std::vector<double> delta(n), correction(n);
    for (std::size_t step=0;step<d.substeps;++step) {
        std::fill(delta.begin(),delta.end(),0); std::fill(correction.begin(),correction.end(),0);
        for (std::size_t j=0;j<ny;++j) for (std::size_t i=0;i<nx;++i) {
            const auto right=i+nx*j, left=(i+nx-1)%nx+nx*j;
            const double velocity=velocities_.xFaces[right];
            const auto donor=velocity>=0 ? left : right;
            const double flux=Scaled(h,velocity,Normalized(staged[donor],scale),config_.spacingX);
            Add(delta[left],correction[left],-flux); Add(delta[right],correction[right],flux);
        }
        for (std::size_t j=0;j<ny;++j) for (std::size_t i=0;i<nx;++i) {
            const auto up=i+nx*j, down=i+nx*((j+ny-1)%ny);
            const double velocity=velocities_.yFaces[up];
            const auto donor=velocity>=0 ? down : up;
            const double flux=Scaled(h,velocity,Normalized(staged[donor],scale),config_.spacingY);
            Add(delta[down],correction[down],-flux); Add(delta[up],correction[up],flux);
        }
        for (std::size_t k=0;k<n;++k) {
            staged[k]=Updated(staged[k],Checked(delta[k]+correction[k]),scale);
            if (d.nonnegativeInput && staged[k]<0) throw std::runtime_error("Scalar transport rounded state violates positivity.");
        }
    }
    const auto final=Summarize(staged,area);
    d.finalIntegratedScalar=final.integral; d.finalAbsoluteIntegral=final.absolute;
    d.finalMinimum=final.minimum; d.finalMaximum=final.maximum;
    d.integratedScalarDrift=Checked(final.integral-initial.integral);
    const double roundoff=(32.0*n+64.0*d.substeps+64)*std::numeric_limits<double>::epsilon();
    d.conservationRoundoffAllowance=Scaled(roundoff,std::max(initial.absolute,final.absolute));
    d.rangeRoundoffAllowance=Scaled(roundoff,std::max(initial.scale,final.scale));
    if (std::abs(d.integratedScalarDrift)>d.conservationRoundoffAllowance)
        throw std::runtime_error("Scalar transport stored integral violates conservation roundoff guard.");
    if (d.discreteDivergenceFree && ((final.minimum<initial.minimum
        && Checked(initial.minimum-final.minimum)>d.rangeRoundoffAllowance)
        || (final.maximum>initial.maximum && Checked(final.maximum-initial.maximum)>d.rangeRoundoffAllowance)))
        throw std::runtime_error("Scalar transport stored range violates divergence-free roundoff guard.");
    // Final publication is nonthrowing; no numerical guard changes the state.
    scalar_.swap(staged); time_=d.timeAfter; diagnostics_=d;
    return d;
}
} // namespace PhysicsEngine
