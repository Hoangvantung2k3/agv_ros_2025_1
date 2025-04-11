
"use strict";

let SaveMap = require('./SaveMap.js')
let SetMapProjections = require('./SetMapProjections.js')
let GetPointMap = require('./GetPointMap.js')
let GetPointMapROI = require('./GetPointMapROI.js')
let GetMapROI = require('./GetMapROI.js')
let ProjectedMapsInfo = require('./ProjectedMapsInfo.js')

module.exports = {
  SaveMap: SaveMap,
  SetMapProjections: SetMapProjections,
  GetPointMap: GetPointMap,
  GetPointMapROI: GetPointMapROI,
  GetMapROI: GetMapROI,
  ProjectedMapsInfo: ProjectedMapsInfo,
};
