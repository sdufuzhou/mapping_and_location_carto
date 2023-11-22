/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-09-30 22:09:50
 * @LastEditTime: 2022-09-30 22:21:49
 */
/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Global map (grid-based)
 * Author: Andrew Howard
 * Date: 6 Feb 2003
 * CVS: $Id: map.c 1713 2003-08-23 04:03:43Z inspectorg $
 **************************************************************************/

#include "include/location_lib/amcl/map.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace gomros {
namespace data_process {
namespace mapping_and_location {
namespace amcl {

// Create a new map
map_t *map_alloc(void) {
  map_t *map;

  map = (map_t *)malloc(sizeof(map_t));

  // Assume we start at (0, 0)
  map->origin_x = 0;
  map->origin_y = 0;

  // Make the size odd
  map->size_x = 0;
  map->size_y = 0;
  map->scale = 0;

  // Allocate storage for main map
  map->cells = (map_cell_t *)NULL;

  return map;
}

// Destroy a map
void map_free(map_t *map) {
  free(map->cells);
  free(map);
  return;
}

// Get the cell at the given point
map_cell_t *map_get_cell(map_t *map, double ox, double oy, double oa) {
  int i, j;
  map_cell_t *cell;

  i = MAP_GXWX(map, ox);
  j = MAP_GYWY(map, oy);

  if (!MAP_VALID(map, i, j)) return NULL;

  cell = map->cells + MAP_INDEX(map, i, j);
  return cell;
}

}  // namespace amcl
}  // namespace location
}  // namespace data_process
}  // namespace gomros
