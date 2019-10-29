/*
;    Project:       Open Vehicle Monitor System
;    Date:          17 July 2018
;
;    Changes:
;    1.0  Initial release
;
;    (C) 2018       Paul Rensel / Reho Engineering
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#include "ovms_log.h"
static const char *TAG = "v-zeva";

#include <stdio.h>
#include "vehicle_zeva.h"
#include "metrics_standard.h"

OvmsVehicleZeva::OvmsVehicleZeva()
  {
  ESP_LOGI(TAG, "Zeva vehicle module");

  // Change CANbus speed to either 125 or 250KBPS 
  RegisterCanBus(1,CAN_MODE_ACTIVE,CAN_SPEED_250KBPS);

  StandardMetrics.ms_v_type->SetValue("ZEVA");
  StandardMetrics.ms_v_vin->SetValue("ZEVAZEVAZEVAZEVA");

  // Commented out default values, set as needed
  StandardMetrics.ms_v_bat_soc->SetValue(0);
  StandardMetrics.ms_v_bat_soh->SetValue(0);
  StandardMetrics.ms_v_bat_cac->SetValue(0);
  StandardMetrics.ms_v_bat_voltage->SetValue(0);
  StandardMetrics.ms_v_bat_current->SetValue(0);
  StandardMetrics.ms_v_bat_power->SetValue(0);
  StandardMetrics.ms_v_bat_energy_used->SetValue(0);
  StandardMetrics.ms_v_bat_energy_recd->SetValue(0);
  StandardMetrics.ms_v_bat_range_full->SetValue(0);
  StandardMetrics.ms_v_bat_range_ideal->SetValue(0);
  StandardMetrics.ms_v_bat_range_est->SetValue(0);
  StandardMetrics.ms_v_bat_12v_voltage->SetValue(0);
  StandardMetrics.ms_v_bat_12v_current->SetValue(0);
  StandardMetrics.ms_v_bat_temp->SetValue(0);
  StandardMetrics.ms_v_charge_voltage->SetValue(0);
  StandardMetrics.ms_v_charge_current->SetValue(0);
  StandardMetrics.ms_v_charge_climit->SetValue(0);
  StandardMetrics.ms_v_charge_kwh->SetValue(0);
  StandardMetrics.ms_v_charge_mode->SetValue("standard");
  StandardMetrics.ms_v_charge_timermode->SetValue(false);
  StandardMetrics.ms_v_charge_timerstart->SetValue(0);
  StandardMetrics.ms_v_charge_state->SetValue("done");
  StandardMetrics.ms_v_charge_substate->SetValue("stopped");
  StandardMetrics.ms_v_charge_type->SetValue("type2");
  StandardMetrics.ms_v_charge_pilot->SetValue(false);
  StandardMetrics.ms_v_charge_inprogress->SetValue(false);
  StandardMetrics.ms_v_charge_limit_range->SetValue(0);
  StandardMetrics.ms_v_charge_limit_soc->SetValue(0);
  StandardMetrics.ms_v_charge_duration_full->SetValue(0);
  StandardMetrics.ms_v_charge_duration_range->SetValue(0);
  StandardMetrics.ms_v_charge_duration_soc->SetValue(0);
  StandardMetrics.ms_v_charge_temp->SetValue(0);
  StandardMetrics.ms_v_inv_temp->SetValue(0);
  StandardMetrics.ms_v_mot_rpm->SetValue(0);
  StandardMetrics.ms_v_mot_temp->SetValue(30);
  StandardMetrics.ms_v_door_fl->SetValue(false);
  StandardMetrics.ms_v_door_fr->SetValue(false);
  StandardMetrics.ms_v_door_rl->SetValue(false);
  StandardMetrics.ms_v_door_rr->SetValue(false);
  StandardMetrics.ms_v_door_chargeport->SetValue(false);
  StandardMetrics.ms_v_door_hood->SetValue(false);
  StandardMetrics.ms_v_door_trunk->SetValue(false);
  StandardMetrics.ms_v_env_drivemode->SetValue(0);
  StandardMetrics.ms_v_env_handbrake->SetValue(false);
  StandardMetrics.ms_v_env_awake->SetValue(false);
  StandardMetrics.ms_v_env_charging12v->SetValue(false);
  StandardMetrics.ms_v_env_cooling->SetValue(false);
  StandardMetrics.ms_v_env_heating->SetValue(false);
  StandardMetrics.ms_v_env_hvac->SetValue(false);
  StandardMetrics.ms_v_env_on->SetValue(false);
  StandardMetrics.ms_v_env_locked->SetValue(false);
  StandardMetrics.ms_v_env_valet->SetValue(false);
  StandardMetrics.ms_v_env_headlights->SetValue(false);
  StandardMetrics.ms_v_env_alarm->SetValue(false);
  StandardMetrics.ms_v_env_ctrl_login->SetValue(false);
  StandardMetrics.ms_v_env_ctrl_config->SetValue(false);
  StandardMetrics.ms_v_env_temp->SetValue(0);
  StandardMetrics.ms_v_pos_gpslock->SetValue(false);
  StandardMetrics.ms_v_pos_satcount->SetValue(0);
  //StandardMetrics.ms_v_pos_latitude->SetValue(22.280868);
  //StandardMetrics.ms_v_pos_longitude->SetValue(114.160598);
  StandardMetrics.ms_v_pos_direction->SetValue(0);
  StandardMetrics.ms_v_pos_altitude->SetValue(0);
  StandardMetrics.ms_v_pos_speed->SetValue(0);
  StandardMetrics.ms_v_pos_odometer->SetValue(0);
  StandardMetrics.ms_v_pos_trip->SetValue(0);
  StandardMetrics.ms_v_tpms_fl_t->SetValue(33);
  StandardMetrics.ms_v_tpms_fr_t->SetValue(33);
  StandardMetrics.ms_v_tpms_rr_t->SetValue(34);
  StandardMetrics.ms_v_tpms_rl_t->SetValue(34);
  StandardMetrics.ms_v_tpms_fl_p->SetValue(206.843);
  StandardMetrics.ms_v_tpms_fr_p->SetValue(206.843);
  StandardMetrics.ms_v_tpms_rr_p->SetValue(275.79);
  StandardMetrics.ms_v_tpms_rl_p->SetValue(275.79);
  }

OvmsVehicleZeva::~OvmsVehicleZeva()
{
  ESP_LOGI(TAG, "Shutdown Zeva vehicle module");
  // Release GPS
  MyEvents.SignalEvent("vehicle.release.gps", NULL);
  MyEvents.SignalEvent("vehicle.release.gpstime", NULL);

}

void OvmsVehicleZeva::IncomingFrameCan1(CAN_frame_t* p_frame)
  {
  uint8_t *d = p_frame->data.u8;

  switch (p_frame->MsgID)
    {
    case 0x0A: // broadcast at 4Hz by Zeva Core and Lite module
      {

      uint8_t zeva_status = (d[0] & 0x07); // contains status bits
      //uint8_t zeva_error = d[0] >> 3; // contains error bits
      
      switch (zeva_status)
      {
      case 0: // Idle
        {
         StandardMetrics.ms_v_env_on->SetValue(false);
         StandardMetrics.ms_v_charge_inprogress->SetValue(false);
        break;
        }
      case 1: // Pre-charging
        {
         //StandardMetrics.ms_v_env_on->SetValue(false);
         //StandardMetrics.ms_v_charge_inprogress->SetValue(true);
        break;
        }
      case 2:  // Running
        {
         StandardMetrics.ms_v_env_on->SetValue(true);
         StandardMetrics.ms_v_charge_inprogress->SetValue(false);
         StandardMetrics.ms_v_charge_voltage->SetValue(0);
         StandardMetrics.ms_v_charge_current->SetValue(0);
        break;
        }

      case 3:  // Charging
        {
         StandardMetrics.ms_v_charge_state->SetValue("charging"); // charging, stopped, topoff, done, prepare, timerwait, heating
         StandardMetrics.ms_v_charge_substate->SetValue("onrequest"); // scheduledstart, onrequest, timerwait, powerwait, stopped
         StandardMetrics.ms_v_env_on->SetValue(false);   
         StandardMetrics.ms_v_charge_inprogress->SetValue(true);
         StandardMetrics.ms_v_charge_voltage->SetValue(StandardMetrics.ms_v_bat_voltage->AsFloat());
         StandardMetrics.ms_v_charge_current->SetValue(StandardMetrics.ms_v_bat_current->AsFloat());

        break;
        }

      case 4:  // Setup
        {
         StandardMetrics.ms_v_env_on-> SetValue(false);
         StandardMetrics.ms_v_charge_inprogress->SetValue(false);
        break;
        }
      default:
        break;
     }

      StandardMetrics.ms_v_bat_current->SetValue( (((unsigned int) d[4] << 4) + (d[3] & 0x0F)) - 2048 );
      StandardMetrics.ms_v_bat_voltage->SetValue( (((unsigned int) (d[3] & 0xF0) << 4) + d[2]) );
      StandardMetrics.ms_v_bat_power->SetValue(StandardMetrics.ms_v_bat_current->AsInt() * StandardMetrics.ms_v_bat_voltage->AsInt());
      
      //Set SOC when CANdata received
      float soc = d[1];
      StandardMetrics.ms_v_bat_soc->SetValue(soc);
      StandardMetrics.ms_v_bat_range_ideal->SetValue(150 * soc / 100);
      StandardMetrics.ms_v_bat_range_est->SetValue(125 * soc / 100);
 
      StandardMetrics.ms_v_bat_temp->SetValue(d[7]);
  
      // d[5] contains 12V battery voltage as read by BMS
      // StandardMetrics.ms_v_bat_12v_voltage->SetValue(d[5] / 10)
      
      // d[6] contains insulation leakage if present: only for Core module
      break;
      }

    case 0x610: // Brusa NLG5 variables, need to check the byte ordering
      StandardMetrics.ms_v_charge_pilot->SetValue(d[0] & 0x80 ); // bit 7

      break;


    case 0x611: // Brusa NLG5 variables, need to check the byte ordering
      StandardMetrics.ms_v_charge_current->SetValue( ((unsigned int) d[0] << 8) + d[1] );
      StandardMetrics.ms_v_charge_voltage->SetValue( ((unsigned int) d[2] << 8) + d[3] );
      StandardMetrics.ms_v_bat_voltage->SetValue( ((unsigned int) d[4] << 8) + d[5] );
      StandardMetrics.ms_v_bat_current->SetValue( ((unsigned int) d[6] << 8) + d[7] );
      //StandardMetrics.ms_v_bat_power->SetValue(StandardMetrics.ms_v_bat_current->AsInt() * StandardMetrics.ms_v_bat_voltage->AsInt());
      break;

    case 0x612: // Brusa NLG5 variables, need to check the byte ordering
      StandardMetrics.ms_v_charge_climit->SetValue( ((unsigned int) d[0] << 8) + d[1] ); // mains current limit by ControlPilot
      if (d[2] < StandardMetrics.ms_v_charge_climit->AsInt()) 
        {
          StandardMetrics.ms_v_charge_climit->SetValue( (unsigned int) d[2]); // mains current limit by PowerIndicator
        }
      break;

    case 0x613: // Brusa NLG5 variables, need to check the byte ordering
      StandardMetrics.ms_v_charge_temp->SetValue( ((unsigned int) d[0] << 8) + d[1] ); // Power stage temperature
      break;


    default:
      break;
    }
  }

OvmsVehicle::vehicle_command_t OvmsVehicleZeva::CommandStartCharge()
  {
  CAN_frame_t frame;
  memset(&frame,0,sizeof(frame));

  frame.origin = m_can1;
  frame.FIR.U = 0;
  frame.FIR.B.DLC = 7;
  frame.FIR.B.FF = CAN_frame_std;
  frame.MsgID = 0x618;
  frame.data.u8[0] = 0x00;
  frame.data.u8[1] = 0x00;
  frame.data.u8[2] = 0x00;
  frame.data.u8[3] = 0x00;
  frame.data.u8[4] = 0x00; //(uint8_t)limit;
  frame.data.u8[5] = 0x00;
  frame.data.u8[6] = 0x00;
  m_can1->Write(&frame);

  return Success;
  }

OvmsVehicle::vehicle_command_t OvmsVehicleZeva::CommandStopCharge()
  {
  CAN_frame_t frame;
  memset(&frame,0,sizeof(frame));

  frame.origin = m_can1;
  frame.FIR.U = 0;
  frame.FIR.B.DLC = 7;
  frame.FIR.B.FF = CAN_frame_std;
  frame.MsgID = 0x618;
  frame.data.u8[0] = 0x00;
  frame.data.u8[1] = 0x00;
  frame.data.u8[2] = 0x00;
  frame.data.u8[3] = 0x00;
  frame.data.u8[4] = 0x00;
  frame.data.u8[5] = 0x00;
  frame.data.u8[6] = 0x00;
  m_can1->Write(&frame);

  return Success;
  }


void OvmsVehicleZeva::Ticker1(uint32_t ticker)
  {
  // Just a stub to fill later
  //OvmsVehicle::Ticker1(ticker);

  //if (StandardMetrics.ms_v_env_on->AsBool())
  //  {
    // We are driving
    //int speed = StandardMetrics.ms_v_pos_speed->AsInt() + (rand()%3) -1;
    //if (speed<0) speed = 0;
    //else if (speed>100) speed = 100;
    //StandardMetrics.ms_v_pos_speed->SetValue(speed);
    //StandardMetrics.ms_v_mot_rpm->SetValue(speed*112);
  //  }
  }

void OvmsVehicleZeva::Ticker10(uint32_t ticker)
  {
  //OvmsVehicle::Ticker10(ticker);

  //if (StandardMetrics.ms_v_charge_inprogress->AsBool())
  //  {
    // We are charging
    //float soc = StandardMetrics.ms_v_bat_soc->AsFloat();
    //if (soc < 100)
    //  {
    //  soc += 0.1;
    //  StandardMetrics.ms_v_bat_soc->SetValue(soc);
    //  }
    //else
    //  {
    //  // Car full
    //  StandardMetrics.ms_v_charge_inprogress->SetValue(false);
    //  StandardMetrics.ms_v_door_chargeport->SetValue(false);
    //  StandardMetrics.ms_v_charge_state->SetValue("done");
    //  StandardMetrics.ms_v_charge_substate->SetValue("stopped");
    //  StandardMetrics.ms_v_charge_pilot->SetValue(false);
    //  StandardMetrics.ms_v_charge_voltage->SetValue(0);
    //  StandardMetrics.ms_v_charge_current->SetValue(0);
    //  }
    //}
  //else if (StandardMetrics.ms_v_env_on->AsBool())
    //{
    // We are driving
    //float soc = StandardMetrics.ms_v_bat_soc->AsFloat();
    //if (soc > 0)
    //  {
    //  int speed = StandardMetrics.ms_v_pos_speed->AsInt() + (rand()%3) -1;
    //  if (speed<0) speed = 0;
    //  else if (speed>100) speed = 100;
    //  soc -= 0.1;
    //  StandardMetrics.ms_v_bat_soc->SetValue(soc);
    //   StandardMetrics.ms_v_pos_speed->SetValue(speed);
    //  StandardMetrics.ms_v_mot_rpm->SetValue(speed*112);
    //  }
    //else
    //  {
    //   // Battery is empty. Charge the car...
    //  StandardMetrics.ms_v_pos_speed->SetValue(0);
    //  StandardMetrics.ms_v_mot_rpm->SetValue(0);
    //  StandardMetrics.ms_v_env_on->SetValue(false);
    // StandardMetrics.ms_v_charge_inprogress->SetValue(true);
    //  StandardMetrics.ms_v_door_chargeport->SetValue(true);
    //  StandardMetrics.ms_v_charge_state->SetValue("charging");
    //  StandardMetrics.ms_v_charge_substate->SetValue("onrequest");
    //  StandardMetrics.ms_v_charge_pilot->SetValue(true);
    //  StandardMetrics.ms_v_charge_voltage->SetValue(220);
    //  StandardMetrics.ms_v_charge_current->SetValue(32);
    //  }
    //}
  }

class OvmsVehicleZevaInit
  {
  public: OvmsVehicleZevaInit();
  } MyOvmsVehicleZevaInit  __attribute__ ((init_priority (9000)));

OvmsVehicleZevaInit::OvmsVehicleZevaInit()
  {
  ESP_LOGI(TAG, "Registering Vehicle: ZEVA (9000)");

  MyVehicleFactory.RegisterVehicle<OvmsVehicleZeva>("ZEVA","ZEVA BMS");
  }
