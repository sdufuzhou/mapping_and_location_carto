/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-09-28 14:24:17
 * @LastEditTime: 2023-04-03 11:14:44
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
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL laser routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_laser.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include "include/location_lib/amcl/amcl_laser.h"

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <sys/types.h>  // required by Darwin
#include <unistd.h>

namespace gomros {
namespace data_process {
namespace mapping_and_location {
namespace amcl {

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLLaser::AMCLLaser(size_t max_beams, map_t *map)
    : AMCLSensor(), max_samples(0), max_obs(0), temp_obs(NULL) {
  this->time = 0.0;

  this->max_beams = max_beams;
  this->map = map;

  return;
}

AMCLLaser::~AMCLLaser() {
  if (temp_obs) {
    for (int k = 0; k < max_samples; k++) {
      delete[] temp_obs[k];
    }
    delete[] temp_obs;
  }
}

void AMCLLaser::SetModelBeam(double z_hit, double z_short, double z_max,
                             double z_rand, double sigma_hit,
                             double lambda_short, double chi_outlier) {
  this->model_type = LASER_MODEL_BEAM;
  this->z_hit = z_hit;
  this->z_short = z_short;
  this->z_max = z_max;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->lambda_short = lambda_short;
  this->chi_outlier = chi_outlier;
}

void AMCLLaser::SetModelLikelihoodField(double z_hit, double z_rand,
                                        double sigma_hit, double max_occ_dist) {
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;

  map_update_cspace(this->map, max_occ_dist);
}

void AMCLLaser::SetModelLikelihoodFieldProb(
    double z_hit, double z_rand, double sigma_hit, double max_occ_dist,
    bool do_beamskip, double beam_skip_distance, double beam_skip_threshold,
    double beam_skip_error_threshold) {
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->do_beamskip = do_beamskip;
  this->beam_skip_distance = beam_skip_distance;
  this->beam_skip_threshold = beam_skip_threshold;
  this->beam_skip_error_threshold = beam_skip_error_threshold;
  map_update_cspace(this->map, max_occ_dist);
}

////////////////////////////////////////////////////////////////////////////////
// Apply the laser sensor model
bool AMCLLaser::UpdateSensor(pf_t *pf, AMCLSensorData *data) {
  if (this->max_beams < 2) return false;

  // Apply the laser sensor model
  if (this->model_type == LASER_MODEL_BEAM)
    pf_update_sensor(pf, (pf_sensor_model_fn_t)BeamModel, data);
  else if (this->model_type == LASER_MODEL_LIKELIHOOD_FIELD)
    pf_update_sensor(pf, (pf_sensor_model_fn_t)LikelihoodFieldModel, data);
  else if (this->model_type == LASER_MODEL_LIKELIHOOD_FIELD_PROB)
    pf_update_sensor(pf, (pf_sensor_model_fn_t)LikelihoodFieldModelProb, data);
  else
    pf_update_sensor(pf, (pf_sensor_model_fn_t)BeamModel, data);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Determine the probability for the given pose
double AMCLLaser::BeamModel(AMCLLaserData *data, pf_sample_set_t *set) {
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double map_range;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;

  self = (AMCLLaser *)data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++) {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    step = (data->range_count - 1) / (self->max_beams - 1);
    for (i = 0; i < data->range_count; i += step) {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // Compute the range according to the map
      map_range = map_calc_range(self->map, pose.v[0], pose.v[1],
                                 pose.v[2] + obs_bearing, data->range_max);
      pz = 0.0;

      // Part 1: good, but noisy, hit
      z = obs_range - map_range;
      pz +=
          self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit));

      // Part 2: short reading from unexpected obstacle (e.g., a person)
      if (z < 0)
        pz += self->z_short * self->lambda_short *
              exp(-self->lambda_short * obs_range);

      // Part 3: Failure to detect obstacle, reported as max-range
      if (obs_range == data->range_max) pz += self->z_max * 1.0;

      // Part 4: Random measurements
      if (obs_range < data->range_max)
        pz += self->z_rand * 1.0 / data->range_max;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz * pz * pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return (total_weight);
}

double AMCLLaser::LikelihoodFieldModel(AMCLLaserData *data,
                                       pf_sample_set_t *set) {
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double obs_range, obs_bearing;
  double total_weight;
  int range_count;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;
  // float ang_detect = M_PI/2;
  // float ang_del = M_PI/180*0.5;
  // int range_count = 2*ang_detect/ang_del;
  float real_partilces = 0;
  float cur_sample_laser_count;     // 当前粒子对应的有效激光数目
  int particle_num_for_locate = 0;  // 用于匹配的激光点数目
  self = (AMCLLaser *)data->sensor;
  // step = (range_count - 1) / (self->max_beams - 1);
  // real_partilces = (float)range_count/((float)step);
  total_weight = 0.0;
  // printf("range_count=%d,cur_sample_laser_count=%f,step=%d\n",range_count,cur_sample_laser_count,step);
  // printf("运行LikelihoodFieldModel模型函数\n");
  int mzp5t3 = 0;  // 0.5-3米的激光数目
  int m3t10 = 0;   // 3-10米的激光数目
  int m10t17 = 0;  // 10-17米的激光数目
  int m17t = 0;    // 大于17米的激光数目
  // float len1 = (data->range_max-1.0) / 3;
  // float len2 = len1*2;
  // float len4 = (data->range_max - 1);
  float len1 = 3.0;
  float len2 = 10.0;
  float len3 = 17.0;
  float len4 = (data->range_max - 1);
  for (i = 0; i < data->range_count; i++) {  // 计算本次扫描的有效激光点数目
    obs_range = data->ranges[i][0];
    if (obs_range < (data->range_max - 1.0)) {
      particle_num_for_locate++;
      /*    if (obs_range > 0 && (obs_range <= (len1)))
              mzp5t3++;
          else if (obs_range > len1 && (obs_range <= (len2)))
              m3t10++;
          else if (obs_range > len2 && (obs_range <= (len3)))
              m10t17++;
          else if (obs_range > len3 && (obs_range <= (len4)))
              m17t++;*/
    }
  }
  // int locateRangeSelect = 0;
  // float maxRange = 0;
  // float minRange = 0;
  // if (m10t17 > (self->max_beams * 2))
  //{
  //     minRange = 10;
  //     maxRange = 17;
  //     locateRangeSelect = 1;
  //     particle_num_for_locate = m10t17;
  // }
  // else if (m3t10 > (self->max_beams * 2))
  //{
  //     minRange = 3;
  //     maxRange = 10;
  //     locateRangeSelect = 2;
  //     particle_num_for_locate = m3t10;
  // }
  // else if (mzp5t3 > (self->max_beams * 2))
  //{
  //     minRange = 0.5;
  //     maxRange = 3;
  //     locateRangeSelect = 3;
  //     particle_num_for_locate = mzp5t3;
  // }
  // else if (m17t > (self->max_beams * 2))
  //{
  //     minRange = 17;
  //     maxRange = data->range_max - 0.1;
  //     locateRangeSelect = 4;
  //     particle_num_for_locate = m17t;
  // }
  //  Compute the sample weights
  step = (particle_num_for_locate - 1) / (self->max_beams - 1);
  // step = (data->range_count - 1) / (self->max_beams - 1);
  real_partilces =
      static_cast<float>(data->range_count) / (static_cast<float>(step));

  for (j = 0; j < set->sample_count; j++) {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    // Pre-compute a couple of things
    double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
    double z_rand_mult = 1.0 / data->range_max;

    //    printf("运行LikelihoodFieldModel模型函数range_count=%d,max_beams=%d,step=%d\n",data->range_count,self->max_beams,step);
    // Step size must be at least 1
    if (step < 1) step = 1;
    cur_sample_laser_count = 0.0;
    for (i = 0; i < data->range_count; i += step) {
      // if((i >= data->range_count) ;
      int mi, mj;
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];
      // if((obs_bearing>ang_detect)||(obs_bearing<-ang_detect))
      //  continue;
      //
      // printf("step=%d第%d个距离%f对应角度%f>>>>>\n",step,i,obs_range,obs_bearing);
      //  This model ignores max range readings

      while (i < data->range_count) {
        obs_range = data->ranges[i][0];
        obs_bearing = data->ranges[i][1];
        if (obs_range < (data->range_max - 1.0)) break;
        i++;
      }
      if (i >= data->range_count) break;
      // if ((obs_range<minRange)&& (obs_range > maxRange))
      //     continue;

      // Check for NaN
      if (obs_range != obs_range) continue;
      cur_sample_laser_count += 1.0;
      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.

      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);

      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      bool occ_dected = false;
      if (!MAP_VALID(self->map, mi, mj)) {
        z = self->map->max_occ_dist;
      } else {
        z = self->map->cells[MAP_INDEX(self->map, mi, mj)].occ_dist;
        occ_dected = true;
      }
      // 如果当前激光点不在地图似然域内，则在step激光范围内查找位于似然域内的点。
      if (!occ_dected) {
        for (int k = 1; k < step; k++) {
          obs_range = data->ranges[i][0];
          obs_bearing = data->ranges[i][1];

          if (obs_range >= (data->range_max - 1.0)) continue;
          if (obs_range != obs_range) continue;
          pz = 0.0;
          hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
          hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

          mi = MAP_GXWX(self->map, hit.v[0]);
          mj = MAP_GYWY(self->map, hit.v[1]);

          if (!MAP_VALID(self->map, mi, mj)) {
            z = self->map->max_occ_dist;
          } else {
            z = self->map->cells[MAP_INDEX(self->map, mi, mj)].occ_dist;
            occ_dected = true;
            break;
          }
        }
      }
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      // TODO: outlier rejection for short readings

      // assert(pz <= 1.0);
      // assert(pz >= 0.0);
      if (pz > 1 || pz < 0) {
        p = 1.0;
        break;
      }
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz * pz * pz;
    }

    sample->weight *= p;
    // total_weight += sample->weight/cur_sample_laser_count*real_partilces;
    total_weight += sample->weight;
  }
  // my_real_time_data.amcl_laser_prop = total_weight/(0.90*real_partilces);
  // printf("当前LikelihoodFieldModel 概率=%f,laser_prop=%f\n",total_weight);
  return (total_weight);
}

double AMCLLaser::LikelihoodFieldModelProb(AMCLLaserData *data,
                                           pf_sample_set_t *set) {
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double log_p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = (AMCLLaser *)data->sensor;

  total_weight = 0.0;

  step = ceil((data->range_count) / static_cast<double>(self->max_beams));

  // Step size must be at least 1
  if (step < 1) step = 1;

  // Pre-compute a couple of things
  double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
  double z_rand_mult = 1.0 / data->range_max;

  double max_dist_prob =
      exp(-(self->map->max_occ_dist * self->map->max_occ_dist) / z_hit_denom);

  // Beam skipping - ignores beams for which a majoirty of particles do not
  // agree with the map prevents correct particles from getting down weighted
  // because of unexpected obstacles such as humans

  bool do_beamskip = self->do_beamskip;
  double beam_skip_distance = self->beam_skip_distance;
  double beam_skip_threshold = self->beam_skip_threshold;

  // we only do beam skipping if the filter has converged
  if (do_beamskip && !set->converged) {
    do_beamskip = false;
  }

  // we need a count the no of particles for which the beam agreed with the map
  int *obs_count = new int[self->max_beams]();

  // we also need a mask of which observations to integrate (to decide which
  // beams to integrate to all particles)
  bool *obs_mask = new bool[self->max_beams]();

  int beam_ind = 0;

  // realloc indicates if we need to reallocate the temp data structure needed
  // to do beamskipping
  bool realloc = false;

  if (do_beamskip) {
    if (self->max_obs < self->max_beams) {
      realloc = true;
    }

    if (self->max_samples < set->sample_count) {
      realloc = true;
    }

    if (realloc) {
      self->reallocTempData(set->sample_count, self->max_beams);
      fprintf(stderr, "Reallocing temp weights %d - %d\n", self->max_samples,
              self->max_obs);
    }
  }

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++) {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    log_p = 0;

    beam_ind = 0;

    for (i = 0; i < data->range_count; i += step, beam_ind++) {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if (obs_range >= data->range_max) {
        continue;
      }

      // Check for NaN
      if (obs_range != obs_range) {
        continue;
      }

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);

      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance

      if (!MAP_VALID(self->map, mi, mj)) {
        pz += self->z_hit * max_dist_prob;
      } else {
        z = self->map->cells[MAP_INDEX(self->map, mi, mj)].occ_dist;
        if (z < beam_skip_distance) {
          obs_count[beam_ind] += 1;
        }
        pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      }

      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)

      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      assert(pz <= 1.0);
      assert(pz >= 0.0);

      // TODO: outlier rejection for short readings

      if (!do_beamskip) {
        log_p += log(pz);
      } else {
        self->temp_obs[j][beam_ind] = pz;
      }
    }
    if (!do_beamskip) {
      sample->weight *= exp(log_p);
      total_weight += sample->weight;
    }
  }

  if (do_beamskip) {
    int skipped_beam_count = 0;
    for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++) {
      if ((obs_count[beam_ind] / static_cast<double>(set->sample_count)) >
          beam_skip_threshold) {
        obs_mask[beam_ind] = true;
      } else {
        obs_mask[beam_ind] = false;
        skipped_beam_count++;
      }
    }

    // we check if there is at least a critical number of beams that agreed with
    // the map otherwise it probably indicates that the filter converged to a
    // wrong solution if that's the case we integrate all the beams and hope the
    // filter might converge to the right solution
    bool error = false;

    if (skipped_beam_count >= (beam_ind * self->beam_skip_error_threshold)) {
      fprintf(stderr,
              "Over %f%% of the observations were not in the map - pf may have "
              "converged to wrong pose - integrating all observations\n",
              (100 * self->beam_skip_error_threshold));
      error = true;
    }

    for (j = 0; j < set->sample_count; j++) {
      sample = set->samples + j;
      pose = sample->pose;

      log_p = 0;

      for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++) {
        if (error || obs_mask[beam_ind]) {
          log_p += log(self->temp_obs[j][beam_ind]);
        }
      }

      sample->weight *= exp(log_p);

      total_weight += sample->weight;
    }
  }

  delete[] obs_count;
  delete[] obs_mask;
  return (total_weight);
}

void AMCLLaser::reallocTempData(int new_max_samples, int new_max_obs) {
  if (temp_obs) {
    for (int k = 0; k < max_samples; k++) {
      delete[] temp_obs[k];
    }
    delete[] temp_obs;
  }
  max_obs = new_max_obs;
  max_samples = fmax(max_samples, new_max_samples);

  temp_obs = new double *[max_samples]();
  for (int k = 0; k < max_samples; k++) {
    temp_obs[k] = new double[max_obs]();
  }
}

}  // namespace amcl
}  // namespace location
}  // namespace data_process
}  // namespace gomros
