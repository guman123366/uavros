cmake_minimum_required(VERSION 2.8.3)
project(mavros_msgs)

find_package(catkin REQUIRED COMPONENTS geographic_msgs geometry_msgs message_generation sensor_msgs std_msgs)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  ADSBVehicle.msg
  ActuatorControl.msg
  Altitude.msg
  AttitudeTarget.msg
  BatteryStatus.msg
  CamIMUStamp.msg
  CellularStatus.msg
  CameraImageCaptured.msg
  CommandCode.msg
  CompanionProcessStatus.msg
  OnboardComputerStatus.msg
  DebugValue.msg
  ESCInfo.msg
  ESCInfoItem.msg
  ESCStatus.msg
  ESCStatusItem.msg
  ESCTelemetry.msg
  ESCTelemetryItem.msg
  EstimatorStatus.msg
  ExtendedState.msg
  FileEntry.msg
  GlobalPositionTarget.msg
  GPSINPUT.msg
  GPSRAW.msg
  GPSRTK.msg
  HilActuatorControls.msg
  HilControls.msg
  HilGPS.msg
  HilSensor.msg
  HilStateQuaternion.msg
  HomePosition.msg
  LandingTarget.msg
  LogData.msg
  LogEntry.msg
  MagnetometerReporter.msg
  ManualControl.msg
  Mavlink.msg
  MountControl.msg
  NavControllerOutput.msg
  OpticalFlowRad.msg
  OverrideRCIn.msg
  Param.msg
  ParamValue.msg
  PlayTuneV2.msg
  PositionTarget.msg
  RCIn.msg
  RCOut.msg
  RTCM.msg
  RadioStatus.msg
  RTKBaseline.msg
  State.msg
  StatusText.msg
  TerrainReport.msg
  Thrust.msg
  TimesyncStatus.msg
  Trajectory.msg
  Tunnel.msg
  VFR_HUD.msg
  VehicleInfo.msg
  Vibration.msg
  Waypoint.msg
  WaypointList.msg
  WaypointReached.msg
  WheelOdomStamped.msg
  UAT_rosmsg.msg
  UAT_Distsensor.msg
  UAT_Opticflow.msg 
  UAT_AprilTag.msg
  UAT_Lpos.msg
  UAT_Gpos.msg 
  UAT_OffboardState.msg 
  Uat_NetlinkState.msg 
  UAT_HangerStatus.msg
  UAT_MqttAuthenStatus.msg
  Uat_GimbalState.msg
  Uat_CameraState.msg
  Uat_DiagnosticStatus.msg
  Uat_RecognizeState.msg
  UAT_Battery.msg
  UAT_Mav2ros.msg
  UAT_Ros2mavlink.msg
  UAT_HighFreqGimbalTarget.msg
  UAT_MsgCode.msg
  Uat_CommonStatus.msg
  Uat_PeripheralVersion.msg
  Uat_FlightInformation.msg
  Sensors_Health.msg
  Uat_SysStatus.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  CommandAck.srv
  CommandBool.srv
  CommandHome.srv
  CommandInt.srv
  CommandLong.srv
  CommandTOL.srv
  CommandTriggerControl.srv
  CommandTriggerInterval.srv
  CommandVtolTransition.srv
  FileChecksum.srv
  FileClose.srv
  FileList.srv
  FileMakeDir.srv
  FileOpen.srv
  FileRead.srv
  FileRemove.srv
  FileRemoveDir.srv
  FileRename.srv
  FileTruncate.srv
  FileWrite.srv
  LogRequestData.srv
  LogRequestEnd.srv
  LogRequestList.srv
  MountConfigure.srv
  MessageInterval.srv
  ParamGet.srv
  ParamPull.srv
  ParamPush.srv
  ParamSet.srv
  SetMavFrame.srv
  SetMode.srv
  StreamRate.srv
  VehicleInfoGet.srv
  WaypointClear.srv
  WaypointPull.srv
  WaypointPush.srv
  WaypointSetCurrent.srv
  UAT_rossrv.srv
  UAT_CommonCtl.srv
  UAT_UpgradeCMD.srv
  UAT_HangarCtl.srv
  UAT_AuthenCtl.srv
  UAT_MountCtl.srv
  Uat_GimbalTarget.srv
)

# add_action_files(
#   DIRECTORY action
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(DEPENDENCIES geographic_msgs geometry_msgs sensor_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geographic_msgs geometry_msgs message_runtime sensor_msgs std_msgs)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
