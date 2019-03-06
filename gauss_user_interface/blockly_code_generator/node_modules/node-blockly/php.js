'use strict';

var Blockly = require('./_blockly')

Blockly.PHP = require('./lib/php_compressed')(Blockly);

module.exports = Blockly;