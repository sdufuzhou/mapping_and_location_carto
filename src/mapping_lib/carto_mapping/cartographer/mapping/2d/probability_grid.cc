/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "cartographer/mapping/2d/probability_grid.h"

#include <limits>

#include "absl/memory/memory.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/submaps.h"

namespace cartographer {
namespace mapping {

/**
 * @brief ProbabilityGrid的构造函数
 * 
 * @param[in] limits 地图坐标信息
 * @param[in] conversion_tables 转换表
 */
ProbabilityGrid::ProbabilityGrid(const MapLimits& limits,
                                 ValueConversionTables* conversion_tables)
    : Grid2D(limits, kMinCorrespondenceCost, kMaxCorrespondenceCost,
             conversion_tables),
      conversion_tables_(conversion_tables) {}

ProbabilityGrid::ProbabilityGrid(const proto::Grid2D& proto,
                                 ValueConversionTables* conversion_tables)
    : Grid2D(proto, conversion_tables), conversion_tables_(conversion_tables) {
  CHECK(proto.has_probability_grid_2d());
}

// Sets the probability of the cell at 'cell_index' to the given
// 'probability'. Only allowed if the cell was unknown before.
// 将索引处单元格的概率设置为给定的概率, 仅当单元格之前处于未知状态时才允许
void ProbabilityGrid::SetProbability(const Eigen::Array2i& cell_index,//cell_index二维像素索引
                                     const float probability) {
  // 获取对应栅格的引用
  uint16& cell =
      (*mutable_correspondence_cost_cells())[ToFlatIndex(cell_index)];//先把二维转换为一维索引
  CHECK_EQ(cell, kUnknownProbabilityValue);//只能对处于未知状态的栅格设置概率
  // 为栅格赋值 value（传入的是占用的概率，先转换为free的概率，再把free的概率转成value值）
  cell =
      CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(probability));
  // 更新bounding_box
  mutable_known_cells_box()->extend(cell_index.matrix());
  //cell_index.matrix()把坐标转换成矩阵的形式，通过extend函数进行更新，返回的也是一个像素坐标，从而更新
}

// Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
// to the probability of the cell at 'cell_index' if the cell has not already
// been updated. Multiple updates of the same cell will be ignored until
// FinishUpdate() is called. Returns true if the cell was updated.
// 如果单元格尚未更新,则将调用 ComputeLookupTableToApplyOdds() 时指定的 'odds' 应用于单元格在 'cell_index' 处的概率
// 在调用 FinishUpdate() 之前，将忽略同一单元格的多次更新。如果单元格已更新，则返回 true
//
// If this is the first call to ApplyOdds() for the specified cell, its value
// will be set to probability corresponding to 'odds'.
// 如果这是对指定单元格第一次调用 ApplyOdds(),则其值将设置为与 'odds' 对应的概率

// 使用查找表对指定栅格进行栅格值的更新
bool ProbabilityGrid::ApplyLookupTable(const Eigen::Array2i& cell_index,
                                       const std::vector<uint16>& table) {
  DCHECK_EQ(table.size(), kUpdateMarker);
  const int flat_index = ToFlatIndex(cell_index);
  // 获取对应栅格的指针
  uint16* cell = &(*mutable_correspondence_cost_cells())[flat_index];
  // 对处于更新状态的栅格, 不再进行更新了，同一个栅格在一帧雷达数据下是不进行多次更新的
  if (*cell >= kUpdateMarker) {
    return false;
  }
  // 未更新的就进行更新
  mutable_update_indices()->push_back(flat_index);
  // 更新栅格值
  *cell = table[*cell];//*cell：代表栅格的一个值，传入到table(地图更新的一个查找表)里，传入老的栅格值返回一个新的栅格值
  DCHECK_GE(*cell, kUpdateMarker);
  // 更新bounding_box
  mutable_known_cells_box()->extend(cell_index.matrix());
  return true;
}

GridType ProbabilityGrid::GetGridType() const {
  return GridType::PROBABILITY_GRID;
}

// Returns the probability of the cell with 'cell_index'.
// 获取索引处单元格的占用概率
float ProbabilityGrid::GetProbability(const Eigen::Array2i& cell_index) const {
  if (!limits().Contains(cell_index)) return kMinProbability;
  return CorrespondenceCostToProbability(ValueToCorrespondenceCost(
      correspondence_cost_cells()[ToFlatIndex(cell_index)]));
 //返回的是correspondence_cost_cells_：地图栅格值，存储的是free的概率转成unit16后的[0,32767]范围内的值，0代表未知
 //ValueToCorrespondenceCost这个函数实现了把value值转换成[0.1-0.9]的浮点数
}


proto::Grid2D ProbabilityGrid::ToProto() const {
  proto::Grid2D result;
  result = Grid2D::ToProto();
  result.mutable_probability_grid_2d();
  return result;
}

// 根据bounding_box对栅格地图进行裁剪直到正好包含点云
std::unique_ptr<Grid2D> ProbabilityGrid::ComputeCroppedGrid() const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  // 根据bounding_box对栅格地图进行裁剪
  ComputeCroppedLimits(&offset, &cell_limits);
  const double resolution = limits().resolution();
  // 重新计算最大值坐标（物理坐标）
  const Eigen::Vector2d max =
      limits().max() - resolution * Eigen::Vector2d(offset.y(), offset.x());
//resolution * Eigen::Vector2d(offset.y(), offset.x())解释:把最接近像素坐标系左上角的坐标乘以分辨率，可以获得一个该坐标相对于原点的物理坐标
//新的max是相对于  最小像素坐标的物理坐标的

  // 重新定义概率栅格地图的大小
  std::unique_ptr<ProbabilityGrid> cropped_grid =
      absl::make_unique<ProbabilityGrid>(
          MapLimits(resolution, max, cell_limits), conversion_tables_);
  // 给新栅格地图赋值（这里可以用SetProbability这个函数，新的栅格地图，都是未更新过的，如果更新过的则采用ApplyLookupTable函数更新）
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) continue;
    cropped_grid->SetProbability(xy_index, GetProbability(xy_index + offset));
  }
  // 返回新地图的指针
  return std::unique_ptr<Grid2D>(cropped_grid.release());
}

// 获取压缩后的地图栅格数据
bool ProbabilityGrid::DrawToSubmapTexture(
    proto::SubmapQuery::Response::SubmapTexture* const texture,
    transform::Rigid3d local_pose) const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  // 根据bounding_box对栅格地图进行裁剪
  ComputeCroppedLimits(&offset, &cell_limits);

  std::string cells;
  // 遍历地图, 将栅格数据存入cells（栅格）
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) {
      cells.push_back(0 /* unknown log odds value */);
      cells.push_back(0 /* alpha */);
      continue;
    }
    // We would like to add 'delta' but this is not possible using a value and
    // alpha. We use premultiplied alpha, so when 'delta' is positive we can
    // add it by setting 'alpha' to zero. If it is negative, we set 'value' to
    // zero, and use 'alpha' to subtract. This is only correct when the pixel
    // is currently white, so walls will look too gray. This should be hard to
    // detect visually for the user, though.
    // 我们想添加 'delta'，但使用值和 alpha 是不可能的
    // 我们使用预乘 alpha，因此当 'delta' 为正时，我们可以通过将 'alpha' 设置为零来添加它。 
    // 如果它是负数，我们将 'value' 设置为零，并使用 'alpha' 进行减法。 这仅在像素当前为白色时才正确，因此墙壁看起来太灰。 
    // 但是，这对于用户来说应该很难在视觉上检测到。
    
    // delta处于[-127, 127]：128-[1-255]得到
    const int delta =
        128 - ProbabilityToLogOddsInteger(GetProbability(xy_index + offset));
    const uint8 alpha = delta > 0 ? 0 : -delta;
    const uint8 value = delta > 0 ? delta : 0;
    // 存数据时存了2个值, 一个是栅格值value, 另一个是alpha透明度
    cells.push_back(value);
    cells.push_back((value || alpha) ? alpha : 1);
  }

  // 保存地图栅格数据时进行压缩
  common::FastGzipString(cells, texture->mutable_cells());
  
  // 填充地图描述信息
  texture->set_width(cell_limits.num_x_cells);
  texture->set_height(cell_limits.num_y_cells);
  const double resolution = limits().resolution();
  texture->set_resolution(resolution);
  const double max_x = limits().max().x() - resolution * offset.y();
  const double max_y = limits().max().y() - resolution * offset.x();
  *texture->mutable_slice_pose() = transform::ToProto(
      local_pose.inverse() *
      transform::Rigid3d::Translation(Eigen::Vector3d(max_x, max_y, 0.)));

  return true;
}

}  // namespace mapping
}  // namespace cartographer
