'use strict';

var Blockly = require('./_blockly')

Blockly.Python = require('./lib/python_compressed')(Blockly);

module.exports = Blockly;