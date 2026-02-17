#include <chrono>
#include <random>
#include <fstream>

#include "hwinfo/hwinfo.h"
#include "hwinfo/utils/unit.h"

#include <rclcpp_components/register_node_macro.hpp>
#include "lgdxrobot_cloud_adapter/CloudAdapter.hpp"

namespace LGDXRobotCloud
{
  
CloudAdapter::CloudAdapter(const rclcpp::NodeOptions &options) : Node("lgdxrobot_cloud_adapter_node", options)
{
  timer = this->create_wall_timer(std::chrono::microseconds(1), [this]() {this->Initalise();});
}

void CloudAdapter::Initalise()
{
  timer->cancel();

  RCLCPP_INFO(this->get_logger(), "Initialising Cloud Adapter");

  // Parameters
  auto cloudSlamEnableParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudSlamEnableParam.description = "Enable LGDXRobot Cloud SLAM Mode.";
  this->declare_parameter("cloud_slam_enable", false, cloudSlamEnableParam);
  auto cloudAddressParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudAddressParam.description = "Address of LGDXRobot2 Cloud.";
  this->declare_parameter("cloud_address", "", cloudAddressParam);
  auto cloudRootCertParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudRootCertParam.description = "Path to server root certificate, required in LGDXRobot2 Cloud.";
  this->declare_parameter("cloud_root_cert", "", cloudRootCertParam);
  auto cloudClientKeyParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudClientKeyParam.description = "Path to client's private key, required in LGDXRobot2 Cloud.";
  this->declare_parameter("cloud_client_key", "", cloudClientKeyParam);
  auto cloudClientCertParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudClientCertParam.description = "Path to client's certificate chain, required in LGDXRobot2 Cloud.";
  this->declare_parameter("cloud_client_cert", "", cloudClientCertParam);

  // Cloud Initalise
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(1000, 6000);
  int cloudRetryWait = dis(gen);
  RCLCPP_INFO(this->get_logger(), "LGDXRobot Cloud is enabled, the break for reconnection to the cloud is %d ms.", cloudRetryWait);
  cloudRetryTimer = this->create_wall_timer(std::chrono::milliseconds(cloudRetryWait), 
    std::bind(&CloudAdapter::HandleError, this));
  cloudRetryTimer->cancel();
  
  // Connect to Cloud
  std::string serverAddress = this->get_parameter("cloud_address").as_string();
  std::string rootCertPath = this->get_parameter("cloud_root_cert").as_string();
  std::string clientKeyPath = this->get_parameter("cloud_client_key").as_string();
  std::string clientCertPath = this->get_parameter("cloud_client_cert").as_string();
  std::string rootCert = ReadCertificate(rootCertPath.c_str());
  std::string clientKey = ReadCertificate(clientKeyPath.c_str());
  std::string clientCert = ReadCertificate(clientCertPath.c_str());
  grpc::SslCredentialsOptions sslOptions = {rootCert, clientKey, clientCert};

  grpcChannel = grpc::CreateChannel(serverAddress, grpc::SslCredentials(sslOptions));
  grpcStub = RobotClientsService::NewStub(grpcChannel);
  accessToken = grpc::AccessTokenCredentials("");
}


std::string CloudAdapter::ReadCertificate(const char *filename)
{
  std::ifstream file(filename, std::ios::in);
  if (file.is_open())
  {
    std::stringstream ss;
		ss << file.rdbuf();
		file.close();
		return ss.str();
  }
  return {};
}

#ifdef __linux__ 
std::string CloudAdapter::GetMotherBoardSerialNumber()
{
  std::ifstream file("/sys/class/dmi/id/board_serial", std::ios::in);
  std::string serialNumber;
  if (file.is_open())
  {
    std::getline(file, serialNumber);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to read motherboard serial number.");
  }
  return serialNumber;
}
#endif

void CloudAdapter::SetSystemInfo(RobotClientsSystemInfo *info)
{
  hwinfo::MainBoard main_board;
  info->set_motherboard(main_board.name());
  #ifdef __linux__ 
    info->set_motherboardserialnumber(GetMotherBoardSerialNumber());
  #else
    info->set_motherboardserialnumber(main_board.serialNumber());
  #endif
  const auto cpus = hwinfo::getAllCPUs();
  if (cpus.size() > 0)
  {
    hwinfo::CPU cpu = cpus.at(0);
    info->set_cpu(cpu.modelName());
  }
  else
  {
    info->set_cpu("");
  }
  hwinfo::OS os;
  info->set_os(os.name());
  info->set_is32bit(os.is32bit());
  info->set_islittleendian(os.isLittleEndian());
  const auto gpus = hwinfo::getAllGPUs();
  if (gpus.size() > 0)
  {
    hwinfo::GPU gpu = gpus.at(0);
    info->set_gpu(gpu.name());
  }
  hwinfo::Memory memory;
  info->set_rammib(hwinfo::unit::bytes_to_MiB(memory.total_Bytes()));
}

void CloudAdapter::HandleError()
{
  cloudRetryTimer->cancel();
  Greet(cloudErrorRetryData.mcuSerialNumber);
  // set hasError to false when greet success
}


void CloudAdapter::Greet(std::string mcuSN)
{
  cloudErrorRetryData.mcuSerialNumber = mcuSN;
  RCLCPP_INFO(this->get_logger(), "Connecting to the cloud.");
  
  grpc::ClientContext *context = new grpc::ClientContext();
  auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(kGrpcWaitSec);
  context->set_deadline(deadline);

  RobotClientsGreet *request = new RobotClientsGreet();
  RobotClientsSystemInfo *systemInfo = new RobotClientsSystemInfo();
  SetSystemInfo(systemInfo);
  if(!mcuSN.empty())
  {
    systemInfo->set_mcuserialnumber(mcuSN);
  }
  request->set_allocated_systeminfo(systemInfo);

  RobotClientsGreetResponse *response = new RobotClientsGreetResponse();
  grpcStub->async()->Greet(context, request, response, [context, request, response, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      accessToken = grpc::AccessTokenCredentials(response->accesstoken());
      if (grpcRealtimeStub == nullptr)
      {
        grpcRealtimeStub = RobotClientsService::NewStub(grpcChannel);
      }
      /*
      if (isCloudSlam)
      {
        RCLCPP_INFO(logger_, "Connected to the cloud, start SLAM data exchange.");
        slamExchangeStream = std::make_unique<SlamExchangeStream>(grpcRealtimeStub.get(), accessToken, cloudSignals);
      }
      else
      {
        RCLCPP_INFO(logger_, "Connected to the cloud, start data exchange.");
        cloudSignals->Connected();
        cloudExchangeStream = std::make_unique<CloudExchangeStream>(grpcRealtimeStub.get(), accessToken, cloudSignals);
      }*/
      // Start the timer to exchange data
      //cloudSignals->NextExchange();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to connect the cloud, will try again.");
      OnErrorOccured();
    }
    delete context;
    delete request;
    delete response;
  });
}

void CloudAdapter::Exchange(const RobotClientsData &robotData,
  const RobotClientsNextToken &nextToken,
  const RobotClientsAbortToken &abortToken)
{
  /*
  if (cloudExchangeStream != nullptr)
  {
    cloudExchangeStream->SendMessage(robotData, nextToken, abortToken);
  }*/
}

void CloudAdapter::SlamExchange(const RobotClientsSlamStatus status,
  const RobotClientsData &robotData,
  const RobotClientsMapData &mapData)
{
  /*
  if (slamExchangeStream != nullptr)
  {
    slamExchangeStream->SendMessage(status, robotData, mapData);
  }*/
}

void CloudAdapter::OnErrorOccured()
{
  if (cloudRetryTimer->is_canceled())
  {
    RCLCPP_ERROR(this->get_logger(), "Cloud error occured.");
    cloudRetryTimer->reset();
  }
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(LGDXRobotCloud::CloudAdapter)