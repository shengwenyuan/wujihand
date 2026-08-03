cmake_minimum_required(VERSION 3.8)
project(wujihand_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedRigidBodySample.msg"
  "msg/TrackingLifecycleEvent.msg"
  "msg/HandObservationEnvelope.msg"
  "msg/RouteCommand.msg"
  "msg/SafetyEvent.msg"
  "msg/TeleoperationTickTrace.msg"
  "msg/SceneRigidBodyState.msg"
  "msg/RunRecordingStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
