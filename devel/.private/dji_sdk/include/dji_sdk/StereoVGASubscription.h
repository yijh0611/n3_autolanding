// Generated by gencpp from file dji_sdk/StereoVGASubscription.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_STEREOVGASUBSCRIPTION_H
#define DJI_SDK_MESSAGE_STEREOVGASUBSCRIPTION_H

#include <ros/service_traits.h>


#include <dji_sdk/StereoVGASubscriptionRequest.h>
#include <dji_sdk/StereoVGASubscriptionResponse.h>


namespace dji_sdk
{

struct StereoVGASubscription
{

typedef StereoVGASubscriptionRequest Request;
typedef StereoVGASubscriptionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct StereoVGASubscription
} // namespace dji_sdk


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dji_sdk::StereoVGASubscription > {
  static const char* value()
  {
    return "a636307470f5a4e2ecf4fbcb4aeca4ed";
  }

  static const char* value(const ::dji_sdk::StereoVGASubscription&) { return value(); }
};

template<>
struct DataType< ::dji_sdk::StereoVGASubscription > {
  static const char* value()
  {
    return "dji_sdk/StereoVGASubscription";
  }

  static const char* value(const ::dji_sdk::StereoVGASubscription&) { return value(); }
};


// service_traits::MD5Sum< ::dji_sdk::StereoVGASubscriptionRequest> should match
// service_traits::MD5Sum< ::dji_sdk::StereoVGASubscription >
template<>
struct MD5Sum< ::dji_sdk::StereoVGASubscriptionRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dji_sdk::StereoVGASubscription >::value();
  }
  static const char* value(const ::dji_sdk::StereoVGASubscriptionRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dji_sdk::StereoVGASubscriptionRequest> should match
// service_traits::DataType< ::dji_sdk::StereoVGASubscription >
template<>
struct DataType< ::dji_sdk::StereoVGASubscriptionRequest>
{
  static const char* value()
  {
    return DataType< ::dji_sdk::StereoVGASubscription >::value();
  }
  static const char* value(const ::dji_sdk::StereoVGASubscriptionRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dji_sdk::StereoVGASubscriptionResponse> should match
// service_traits::MD5Sum< ::dji_sdk::StereoVGASubscription >
template<>
struct MD5Sum< ::dji_sdk::StereoVGASubscriptionResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dji_sdk::StereoVGASubscription >::value();
  }
  static const char* value(const ::dji_sdk::StereoVGASubscriptionResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dji_sdk::StereoVGASubscriptionResponse> should match
// service_traits::DataType< ::dji_sdk::StereoVGASubscription >
template<>
struct DataType< ::dji_sdk::StereoVGASubscriptionResponse>
{
  static const char* value()
  {
    return DataType< ::dji_sdk::StereoVGASubscription >::value();
  }
  static const char* value(const ::dji_sdk::StereoVGASubscriptionResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DJI_SDK_MESSAGE_STEREOVGASUBSCRIPTION_H
