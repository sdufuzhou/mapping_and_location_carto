/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-10-01 16:22:48
 * @LastEditTime: 2022-10-01 16:22:48
 */
/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PLUGINS_MAPPING_LIB_INCLUDE_KARTO_MAPPING_SPA_SOLVER_H_
#define PLUGINS_MAPPING_LIB_INCLUDE_KARTO_MAPPING_SPA_SOLVER_H_

#ifndef EIGEN_USE_NEW_STDVECTOR
#define EIGEN_USE_NEW_STDVECTOR
#endif  // EIGEN_USE_NEW_STDVECTOR

#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(10)
#include <Eigen/Eigen>

#include "include/mapping_lib/karto_mapping/Mapper.h"
#include "include/mapping_lib/karto_mapping/spa2d.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

typedef std::vector<karto::Matrix3> CovarianceVector;

class SpaSolver : public karto::ScanSolver {
 public:
  SpaSolver();
  virtual ~SpaSolver();

 public:
  virtual void Clear();
  virtual void Compute();
  virtual const karto::ScanSolver::IdPoseVector& GetCorrections() const;

  virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex);
  virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge);

  // Get the underlying graph from SBA
  // return the graph of constraints
  /// x,y -> x',y'   4 floats per connection
  void getGraph(std::vector<float>& g) { m_Spa.getGraph(g); }

 private:
  karto::ScanSolver::IdPoseVector corrections;

  sba::SysSPA2d m_Spa;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros

#endif  // PLUGINS_MAPPING_LIB_INCLUDE_KARTO_MAPPING_SPA_SOLVER_H_
