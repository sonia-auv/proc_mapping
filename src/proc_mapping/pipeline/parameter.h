/**
 * \file	parameter.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	09/05/2016
 *
 * \copyright Copyright (c) 2016 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PROC_MAPPING_PIPELINE_PARAMETER_H_
#define PROC_MAPPING_PIPELINE_PARAMETER_H_

#include <sonia_msgs/ProcUnitParameter.h>
#include <boost/any.hpp>
#include <memory>
#include <vector>

namespace proc_mapping {

class ParameterInterface {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ParameterInterface>;
  using ConstPtr = std::shared_ptr<const ParameterInterface>;
  using PtrList = std::vector<ParameterInterface::Ptr>;
  using ConstPtrList = std::vector<ParameterInterface::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit ParameterInterface(const std::string &name);

  virtual ~ParameterInterface() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  std::string GetName() const;

  virtual std::string GetStringValue() const = 0;

  virtual void SetFromRosMessage(
      const sonia_msgs::ProcUnitParameter &parameter) = 0;

  virtual sonia_msgs::ProcUnitParameter BuildRosMessage() const = 0;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::string name_;
};

template <class Tp_>
class Parameter : public ParameterInterface {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Parameter<Tp_>>;
  using ConstPtr = std::shared_ptr<const Parameter<Tp_>>;
  using PtrList = std::vector<Parameter<Tp_>::Ptr>;
  using ConstPtrList = std::vector<Parameter<Tp_>::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit Parameter(const std::string &name,
                     std::vector<ParameterInterface *> &parameters);
  explicit Parameter(const std::string &name, const Tp_ &value,
                     std::vector<ParameterInterface *> &parameters);
  virtual ~Parameter() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  Tp_ GetValue() const;

  void SetValue(const Tp_ &value);

  virtual void SetFromRosMessage(
      const sonia_msgs::ProcUnitParameter &parameter) override;

  virtual sonia_msgs::ProcUnitParameter BuildRosMessage() const override;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  Tp_ value_;
};

//==============================================================================
// I N L I N E   M E T H O D S

//------------------------------------------------------------------------------
//
inline ParameterInterface::ParameterInterface(const std::string &name)
    : name_(name) {}

//------------------------------------------------------------------------------
//
inline std::string ParameterInterface::GetName() const { return name_; }

//------------------------------------------------------------------------------
//
template <class Tp_>
inline Parameter<Tp_>::Parameter(const std::string &name,
                                 std::vector<ParameterInterface *> &parameters)
    : ParameterInterface(name), value_() {
  parameters.push_back(this);
}

//------------------------------------------------------------------------------
//
template <class Tp_>
inline Parameter<Tp_>::Parameter(const std::string &name, const Tp_ &value,
                                 std::vector<ParameterInterface *> &parameters)
    : ParameterInterface(name), value_(value) {
  parameters.push_back(this);
}

//------------------------------------------------------------------------------
//
template <class Tp_>
inline sonia_msgs::ProcUnitParameter Parameter<Tp_>::BuildRosMessage() const {
  return sonia_msgs::ProcUnitParameter();
}

//------------------------------------------------------------------------------
//
template <class Tp_>
inline void Parameter<Tp_>::SetFromRosMessage(
    const sonia_msgs::ProcUnitParameter &parameter) {}

//------------------------------------------------------------------------------
//
template <class Tp_>
inline void Parameter<Tp_>::SetValue(const Tp_ &value) {}

//------------------------------------------------------------------------------
//
template <class Tp_>
inline Tp_ Parameter<Tp_>::GetValue() const {
  return nullptr;
}

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PIPELINE_PARAMETER_H_
