'use strict';

var Blockly = require('./_blockly')

Blockly.Lua = require('./lib/lua_compressed')(Blockly);

module.exports = Blockly;