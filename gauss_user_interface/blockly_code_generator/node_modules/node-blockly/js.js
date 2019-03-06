'use strict';

var Blockly = require('./_blockly')

Blockly.JavaScript = require('./lib/javascript_compressed')(Blockly);

module.exports = Blockly;