// Generated by gencpp from file dji_sdk/MissionHpGetInfo.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_MISSIONHPGETINFO_H
#define DJI_SDK_MESSAGE_MISSIONHPGETINFO_H

#include <ros/service_traits.h>


#include <dji_sdk/MissionHpGetInfoRequest.h>
#include <dji_sdk/MissionHpGetInfoResponse.h>


namespace dji_sdk
{

struct MissionHpGetInfo
{

typedef MissionHpGetInfoRequest Request;
typedef MissionHpGetInfoResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct MissionHpGetInfo
} // namespace dji_sdk


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dji_sdk::MissionHpGetInfo > {
  static const char* value()
  {
    return "8c08f93488e030961f6001dc630fd2c2";
  }

  static const char* value(const ::dji_sdk::MissionHpGetInfo&) { return value(); }
};

template<>
struct DataType< ::dji_sdk::MissionHpGetInfo > {
  static const char* value()
  {
    return "dji_sdk/MissionHpGetInfo";
  }

  static const char* value(const ::dji_sdk::MissionHpGetInfo&) { return value(); }
};


// service_traits::MD5Sum< ::dji_sdk::MissionHpGetInfoRequest> should match
// service_traits::MD5Sum< ::dji_sdk::MissionHpGetInfo >
template<>
struct MD5Sum< ::dji_sdk::MissionHpGetInfoRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dji_sdk::MissionHpGetInfo >::value();
  }
  static const char* value(const ::dji_sdk::MissionHpGetInfoRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dji_sdk::MissionHpGetInfoRequest> should match
// service_traits::DataType< ::dji_sdk::MissionHpGetInfo >
template<>
struct DataType< ::dji_sdk::MissionHpGetInfoRequest>
{
  static const char* value()
  {
    return DataType< ::dji_sdk::MissionHpGetInfo >::value();
  }
  static const char* value(const ::dji_sdk::MissionHpGetInfoRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dji_sdk::MissionHpGetInfoResponse> should match
// service_traits::MD5Sum< ::dji_sdk::MissionHpGetInfo >
template<>
struct MD5Sum< ::dji_sdk::MissionHpGetInfoResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dji_sdk::MissionHpGetInfo >::value();
  }
  static const char* value(const ::dji_sdk::MissionHpGetInfoResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dji_sdk::MissionHpGetInfoResponse> should match
// service_traits::DataType< ::dji_sdk::MissionHpGetInfo >
template<>
struct DataType< ::dji_sdk::MissionHpGetInfoResponse>
{
  static const char* value()
  {
    return DataType< ::dji_sdk::MissionHpGetInfo >::value();
  }
  static const char* value(const ::dji_sdk::MissionHpGetInfoResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DJI_SDK_MESSAGE_MISSIONHPGETINFO_H
