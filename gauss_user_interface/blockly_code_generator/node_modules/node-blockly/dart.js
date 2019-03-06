'use strict';

var Blockly = require('./_blockly')

Blockly.Dart = require('./lib/dart_compressed')(Blockly);

module.exports = Blockly;