/*
;    Project:       Open Vehicle Monitor System
;    Module:        CAN logging framework
;    Date:          18th January 2018
;
;    (C) 2018       Michael Balzer
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
static const char *TAG = "canlog-tcpclient";

#include "can.h"
#include "canformat.h"
#include "canlog_tcpclient.h"
#include "ovms_config.h"
#include "ovms_peripherals.h"

canlog_tcpclient* MyCanLogTcpClient = NULL;

void can_log_tcpclient_start(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  std::string format(cmd->GetName());
  std::string mode(cmd->GetParent()->GetName());
  canlog_tcpclient* logger = new canlog_tcpclient(argv[0],format,GetFormatModeType(mode));
  logger->Open();

  if (logger->IsOpen())
    {
    if (argc>1)
      { MyCan.AddLogger(logger, argc-1, &argv[1]); }
    else
      { MyCan.AddLogger(logger); }
    writer->printf("CAN logging as TCP client: %s\n", logger->GetInfo().c_str());
    }
  else
    {
    writer->printf("Error: Could not start CAN logging as TCP client: %s\n",logger->GetInfo().c_str());
    delete logger;
    }
  }

class OvmsCanLogTcpClientInit
  {
  public:
    OvmsCanLogTcpClientInit();
    void NetManInit(std::string event, void* data);
    void NetManStop(std::string event, void* data);
  } MyOvmsCanLogTcpClientInit  __attribute__ ((init_priority (4560)));

OvmsCanLogTcpClientInit::OvmsCanLogTcpClientInit()
  {
  ESP_LOGI(TAG, "Initialising CAN logging as TCP client (4560)");

  OvmsCommand* cmd_can = MyCommandApp.FindCommand("can");
  if (cmd_can)
    {
    OvmsCommand* cmd_can_log = cmd_can->FindCommand("log");
    if (cmd_can_log)
      {
      OvmsCommand* cmd_can_log_start = cmd_can_log->FindCommand("start");
      if (cmd_can_log_start)
        {
        // We have a place to put our command tree..
        OvmsCommand* start = cmd_can_log_start->RegisterCommand("tcpclient", "CAN logging as TCP client");
        OvmsCommand* discard = start->RegisterCommand("discard","CAN logging as TCP client (discard mode)");
        OvmsCommand* simulate = start->RegisterCommand("simulate","CAN logging as TCP client (simulate mode)");
        OvmsCommand* transmit = start->RegisterCommand("transmit","CAN logging as TCP client (transmit mode)");
        MyCanFormatFactory.RegisterCommandSet(discard, "Start CAN logging as TCP client (discard mode)",
          can_log_tcpclient_start,
          "<host:port> [filter1] ... [filterN]\n"
          "Filter: <bus> | <id>[-<id>] | <bus>:<id>[-<id>]\n"
          "Example: 2:2a0-37f",
          1, 9);
        MyCanFormatFactory.RegisterCommandSet(simulate, "Start CAN logging as TCP client (simulate mode)",
          can_log_tcpclient_start,
          "<host:port> [filter1] ... [filterN]\n"
          "Filter: <bus> | <id>[-<id>] | <bus>:<id>[-<id>]\n"
          "Example: 2:2a0-37f",
          1, 9);
        MyCanFormatFactory.RegisterCommandSet(transmit, "Start CAN logging as TCP client (transmit mode)",
          can_log_tcpclient_start,
          "<host:port> [filter1] ... [filterN]\n"
          "Filter: <bus> | <id>[-<id>] | <bus>:<id>[-<id>]\n"
          "Example: 2:2a0-37f",
          1, 9);
        }
      }
    }
  using std::placeholders::_1;
  using std::placeholders::_2;
  MyEvents.RegisterEvent(TAG, "network.mgr.init", std::bind(&OvmsCanLogTcpClientInit::NetManInit, this, _1, _2));
  MyEvents.RegisterEvent(TAG, "network.mgr.stop", std::bind(&OvmsCanLogTcpClientInit::NetManStop, this, _1, _2));
  }

void OvmsCanLogTcpClientInit::NetManInit(std::string event, void* data)
  {
  if (MyCanLogTcpClient) MyCanLogTcpClient->Open();
  }

void OvmsCanLogTcpClientInit::NetManStop(std::string event, void* data)
  {
  if (MyCanLogTcpClient) MyCanLogTcpClient->Close();
  }

static void tcMongooseHandler(struct mg_connection *nc, int ev, void *p)
  {
  if (MyCanLogTcpClient)
    MyCanLogTcpClient->MongooseHandler(nc, ev, p);
  else if (ev == MG_EV_ACCEPT)
    {
    ESP_LOGI(TAG, "Log service connection rejected (logger not running)");
    nc->flags |= MG_F_CLOSE_IMMEDIATELY;
    }
  }

canlog_tcpclient::canlog_tcpclient(std::string path, std::string format, canformat::canformat_serve_mode_t mode)
  : canlog("tcpclient", format, mode)
  {
  MyCanLogTcpClient = this;
  m_mgconn = NULL;
  m_isopen = false;
  m_path = path;
  }

canlog_tcpclient::~canlog_tcpclient()
  {
  Close();
  MyCanLogTcpClient = NULL;
  }

bool canlog_tcpclient::Open()
  {
  if (m_isopen) return true;

  struct mg_mgr* mgr = MyNetManager.GetMongooseMgr();
  if (mgr != NULL)
    {
    if (MyNetManager.m_network_any)
      {
      ESP_LOGI(TAG, "Launching TCP client to %s",m_path.c_str());
      struct mg_connect_opts opts;
      const char* err;
      memset(&opts, 0, sizeof(opts));
      opts.error_string = &err;
      if ((m_mgconn = mg_connect_opt(mgr, m_path.c_str(), tcMongooseHandler, opts)) != NULL)
        {
        m_isopen = true;
        return true;
        }
      else
        {
        ESP_LOGE(TAG,"Could not connect to %s",m_path.c_str());
        return false;
        }
      }
    else
      {
      ESP_LOGI(TAG,"Delay TCP client (as network manager not up)");
      return true;
      }
    }
  else
    {
    ESP_LOGE(TAG,"Network manager is not available");
    return false;
    }
  }

void canlog_tcpclient::Close()
  {
  if ((m_isopen)&&(m_mgconn != NULL))
    {
    ESP_LOGI(TAG, "Closed TCP client log: %s", GetStats().c_str());
    m_mgconn->flags |= MG_F_CLOSE_IMMEDIATELY;
    m_mgconn = NULL;
    m_isopen = false;
    }
  }

bool canlog_tcpclient::IsOpen()
  {
  return m_isopen;
  }

std::string canlog_tcpclient::GetInfo()
  {
  std::string result = canlog::GetInfo();
  result.append(" Path:");
  result.append(m_path);
  return result;
  }

void canlog_tcpclient::OutputMsg(CAN_log_message_t& msg)
  {
  if (m_formatter == NULL) return;

  if ((m_mgconn != NULL)&&(m_isopen))
    {
    std::string result = m_formatter->get(&msg);
    if (result.length()>0)
      {
      OvmsMutexLock lock(&m_mgmutex);
      if (m_mgconn->send_mbuf.len < 4096)
        {
        mg_send(m_mgconn, (const char*)result.c_str(), result.length());
        }
      else
        {
        m_dropcount++;
        }
      }
    }
  }

void canlog_tcpclient::MongooseHandler(struct mg_connection *nc, int ev, void *p)
  {
  OvmsMutexLock lock(&m_mgmutex);

  switch (ev)
    {
    case MG_EV_CONNECT:
      {
      int *success = (int*)p;
      ESP_LOGV(TAG, "MongooseHandler(MG_EV_CONNECT=%d)",*success);
      if (*success == 0)
        {
        // Successful connection
        ESP_LOGI(TAG, "Connection successful to %s",m_path.c_str());
        if (m_formatter != NULL)
          {
          std::string result = m_formatter->getheader();
          if (result.length()>0)
            {
            mg_send(nc, (const char*)result.c_str(), result.length());
            }
          }
        }
      else
        {
        // Connection failed
        ESP_LOGE(TAG, "Connection failed to %s",m_path.c_str());
        m_mgconn = NULL;
        m_isopen = false;
        }
      }
      break;
    case MG_EV_CLOSE:
      ESP_LOGV(TAG, "MongooseHandler(MG_EV_CLOSE)");
      if (m_isopen)
        {
        ESP_LOGE(TAG,"Disconnected from %s",m_path.c_str());
        m_mgconn = NULL;
        m_isopen = false;
        }
      break;
    case MG_EV_RECV:
      {
      ESP_LOGV(TAG, "MongooseHandler(MG_EV_RECV)");
      size_t used = nc->recv_mbuf.len;
      if (m_formatter != NULL)
        {
        used = m_formatter->Serve((uint8_t*)nc->recv_mbuf.buf, used);
        }
      if (used > 0)
        {
        mbuf_remove(&nc->recv_mbuf, used);
        }
      break;
      }
    default:
      break;
    }
  }
