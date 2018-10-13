/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include "mikata_arm_toolbox/cli_core.h"

using namespace std;

void print_table();

std::vector<double> q;
std::vector<double> link_x_pos;
std::vector<double> link_y_pos;
std::vector<double> link_z_pos;
std::vector<int> link_vel;
std::vector<int> link_acel;

int main()
{
  dxl_setup();
  pingAll();

  if (!dxl_read(GRIPPER_ID, ADDR_X_DRIVE_MODE, sizeof(int8_t)))
    throw std::runtime_error("Please run dxl_setup before execution.");

  setChain();
  enableAll();
  setVelAll(20);
  setAcelAll(2, false);
  dxl_write(GRIPPER_ID, 30, ADDR_X_PROFILE_VEL, sizeof(int32_t));
  dxl_write(GRIPPER_ID, 250, ADDR_X_P_GAIN, sizeof(int16_t));
  
  while(1) {
    
    //Clear Parameters
    clear();
    q.clear();
    link_x_pos.clear();
    link_y_pos.clear();
    link_z_pos.clear();
    link_vel.clear();
    link_acel.clear();

    //Set Parameters
    q = readAll(true);
    link_vel = getVelAll();
    link_acel = dxl_readAll(ADDR_X_PROFILE_ACEL, sizeof(int32_t));
    solveFK(q);
    for(int i=0; i<=LINK_NUM; i++) {
      int id = i+1;
      Vector3d val = chain[i].getPos();
      link_x_pos.push_back(val[0]);
      link_y_pos.push_back(val[1]);
      link_z_pos.push_back(val[2]);
    }

    init_pos();
   }



}

