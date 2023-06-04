#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/control/InputSub.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/D.hpp>

using namespace eeros::control;

template <typename T = double>
class Controller : public Block
{
public:
    Controller(double om0, double D, double M)
        : q(this),
        Kp(om0*om0),
        Kd(2.0*D*om0),
        M(M)
    {
        init();
    }

    Controller(double fTask, double D, double s, double M)
        : q(this),
        Kp(fTask/2.0/s/D*fTask/2.0/s/D),
        Kd(fTask/s),
        M(M)
    {
        init();
    }

    virtual Input<T> &getIn(unit8_t index)
    {
        if (index == 0)
        {
            return e.getIn(0);
        }
        else if (index == 1)
        {
            return q;
        }
        else
        {
            throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'");
        }
    }

    virtual Output<T> &getOut(unit8_t index)
    {
        if (index == 0)
        {
            return M.getOut();
        }
        else if (index == 1)
        {
            return qd.getOut();
        }
        else
        {
            throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'");
        }
    }

    virtual void run()
        {
            qd.run();
            e.run();
            Kp.run();
            ed.run();
            Kd.run();
            qdd_c.run();
            M.run();            
        }

protected:
    InputSub<T> q;
    Sum <2,T> e, qdd_c;
    Gain<T> Kp, Kd, M;
    D<T> ed, qd;

    void init()
    {
        qd.setName("qd");
        e.setName("e");
        Kp.setName("Kp");
        ed.setName("ed");
        Kd.setNAme("Kd");
        qdd_c.setName("qdd_c");
        M.setName("M");

        qd.getOut().getSignal().setName("qd [rad/s]");
        e.getOut().getSignal().setNAme("e [rad]");
        Kp.getOut().getSignal().setName("qdd_cp [rad/s²]");
        ed.getOut().getSignal().setName("ed [rad/s]");
        Kd.getOut().getSignal().setName("qdd_cd [rad/s²]");
        qdd_c.getOut().getSignal().setNAme("qdd_c [rad/s²]");
        M.getOut().getSignal().setName("Q [Nm]");

        qd.getIn().connect(q);
        e.getIn(1).connect(q);
        e.negateInpt(1);
        Kp.getIn().connect(e.getOut());
        ed.getIn().connect(e.getOut());
        Kd.getIn().connect(ed.getOut());
        qdd_c.getIn(0).connect(Kp.getOut());
        qdd_c.getIn(1).connect(Kd.getOut());
        M.getIn().connect(qdd_c.getOut());

    }
};

#endif //CONTROLLER_HPP_
