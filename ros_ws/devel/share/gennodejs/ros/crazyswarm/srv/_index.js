
"use strict";

let UpdateParams = require('./UpdateParams.js')
let GoTo = require('./GoTo.js')
let StartTrajectory = require('./StartTrajectory.js')
let SetGroupMask = require('./SetGroupMask.js')
let Takeoff = require('./Takeoff.js')
let Stop = require('./Stop.js')
let NotifySetpointsStop = require('./NotifySetpointsStop.js')
let Land = require('./Land.js')
let UploadTrajectory = require('./UploadTrajectory.js')

module.exports = {
  UpdateParams: UpdateParams,
  GoTo: GoTo,
  StartTrajectory: StartTrajectory,
  SetGroupMask: SetGroupMask,
  Takeoff: Takeoff,
  Stop: Stop,
  NotifySetpointsStop: NotifySetpointsStop,
  Land: Land,
  UploadTrajectory: UploadTrajectory,
};
