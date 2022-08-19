
"use strict";

let MissionWaypointTask = require('./MissionWaypointTask.js');
let MissionWaypointAction = require('./MissionWaypointAction.js');
let PayloadData = require('./PayloadData.js');
let VOPosition = require('./VOPosition.js');
let GPSUTC = require('./GPSUTC.js');
let Gimbal = require('./Gimbal.js');
let MissionWaypoint = require('./MissionWaypoint.js');
let MobileData = require('./MobileData.js');
let FCTimeInUTC = require('./FCTimeInUTC.js');
let WaypointList = require('./WaypointList.js');
let FlightAnomaly = require('./FlightAnomaly.js');
let MissionHotpointTask = require('./MissionHotpointTask.js');
let Waypoint = require('./Waypoint.js');

module.exports = {
  MissionWaypointTask: MissionWaypointTask,
  MissionWaypointAction: MissionWaypointAction,
  PayloadData: PayloadData,
  VOPosition: VOPosition,
  GPSUTC: GPSUTC,
  Gimbal: Gimbal,
  MissionWaypoint: MissionWaypoint,
  MobileData: MobileData,
  FCTimeInUTC: FCTimeInUTC,
  WaypointList: WaypointList,
  FlightAnomaly: FlightAnomaly,
  MissionHotpointTask: MissionHotpointTask,
  Waypoint: Waypoint,
};
