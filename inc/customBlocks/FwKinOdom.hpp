#ifndef FWKINODOM_HPP_
#define FWKINODOM_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/control/InputSub.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/Output.hpp>

using namespace eeros::control;

class FwKinOdom : public Block
{
public:
    FwKinOdom(double B, eeros::math::Vector2 GrRInit = 0.0, double phiInit = 0.0)
    : B(B),
    vw(this),
    RJW([&, B]()
        {
        eeros::math::Matrix<3,2> M = {5.0, 0.0, -1.0, 1.0/B};
        eeros::math::Vector3 out = M*RJW.getIn().getSignal().getValue();
        for (int i = 0; i < 3; i++)
        {
            RJW.getOut(i).getSignal().setValue(out(i));
            RJW.getOut(i).getSignal().setTimestamp(RJW.getIn().getSignal().getTimestamp());

        } }),
    GRR([&]()
        {
            double phi = GRR.getIn(2).getSignal().getValue();
            double cphi = std::cos(phi);
            double sphi = std::sin(phi);
            eeros::math::Matrix<3, 3> M = {cphi, sphi, 0.0, -sphi, cphi, 0.0, 0.0, 0.0, 1.0};
            eeros::math::Vector3 v = {GRR.getIn(0).getSignal().getValue(), GRR.getIn(1).getSignal().getValue(), 0.0};
            eeros::math::Vector3 out = M*v;
            GRR.getOut().getSignal().setValue(out.getSubMatrix<2, 1>(0, 0));
            GRR.getOut().getSignal().setTimestamp(GRR.getIn(0).getSignal().getTimestamp());
        })
    {
        // Init integrators
        GrR.setInitCondition(GrRInit);
        phi.setInitCondition(phiInit);

        // Name blocks
        RJW.setName("FwKinOdom->RJW");
        GRR.setName("FwKinOdom->GRR");
        GrR.setName("FwKinOdom->GrR");
        phi.setName("FwKinOdom->phi");

        // Name signals
        RJW.getOut(0).getSignal().setName("RvRx [m/s]");
        RJW.getOut(1).getSignal().setName("RvRy [m/s]");
        RJW.getOut(2).getSignal().setName("omegaR [rad/s]");
        GRR.getOut().getSignal().setName("GvR [m/s]");
        GrR.getOut().getSignal().setName("GrR [m]");
        phi.getOut().getSignal().setName("phi [rad]");

        // Connect signals
        RJW.getIn().connect(vw);
        GRR.getIn(0).connect(RJW.getOut(0));
        GRR.getIn(1).connect(RJW.getOut(1));
        GRR.getIn(2).connect(phi.getOut());
        phi.getIn().connect(RJW.getOut(2));
        GrR.getIn().connect(GRR.getOut());
    }

    
    Input<eeros::math::Vector2> &getIn()
        {
            return vw;
        }
    
    Output<eeros::math::Vector2> &getOutGvR()
        {
            return GRR.getOut();
        }

    Output<eeros::math::Vector2> &getOutGrR()
        {
            return GrR.getOut();
        }

    Output<> &getOutPhi()
        {
            return phi.getOut();
        }

    Output<> &getOutOmegaR()
        {
            return RJW.getOut(2);
        }

    virtual void run()
    {
        RJW.run();
        GRR.run();
        GrR.run();
        phi.run();
    }

    void enable(void)
    {
        GrR.enable();
        phi.enable();
    }

    void disable(void)
    {
        GrR.disable();
        phi.disable();
    }

    void setPose(eeros::math::Vector2 GrR, double phi)
    {
        this->GrR.setInitCondition(GrR);
        this->phi.setInitCondition(phi);
    }

protected:
    double B;
    InputSub<eeros::math::Vector2> vw;
    Blockio<1, 3, eeros::math::Vector2, double> RJW;
    Blockio<3, 1, double, eeros::math::Vector2> GRR;
    I<eeros::math::Vector2> GrR;
    I<> phi;
};

#endif //FWKINODOM_HPP_
