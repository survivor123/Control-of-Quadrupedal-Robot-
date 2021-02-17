// This file is part of RAVL, Recognition And Vision Library
// Copyright (C) 2001, University of Surrey
// This code may be redistributed under the terms of the GNU Lesser
// General Public License (LGPL). See the lgpl.licence file for details or
// see http://www.gnu.org/copyleft/lesser.html

#include "ReactRobotics/LineABC2d.hh"

namespace ReactRoboticsN
{

  LineABC2dC & LineABC2dC::MakeUnitNormal() {
    RealT size = normal.norm();
    normal /= size;
    d      /= size;
    return *this;
  }

  bool LineABC2dC::AreParallel(const LineABC2dC & line) const {
    RealT crossSize = Cross(Normal(),line.Normal());
    return  IsAlmostZero(crossSize);
  }

  bool LineABC2dC::Intersection(const LineABC2dC & line,Eigen::Vector2f &here) const  {
    RealT crossSize = Cross(Normal(),line.Normal());
    if ( IsAlmostZero(crossSize) )
      return false;
    here = Eigen::Vector2f((line.C()*B() - line.B()*C())/crossSize,
                    (line.A()*C() - line.C()*A())/crossSize);
    return true;
  }

  LineABC2dC::RealT LineABC2dC::SqrEuclidDistance(const Eigen::Vector2f & point) const {
    RealT t = Residuum(point);
    return t*t/normal.squaredNorm();
  }
}
