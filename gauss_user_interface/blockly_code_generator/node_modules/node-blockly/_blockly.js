const Blockly = require('./lib/blockly_compressed');


Blockly.setLocale = function(locale) {
  Blockly.Msg = Object.assign(locale, Blockly.Msg);
  Blockly.Msg = Blockly.Msg();
}

Blockly.utils.getMessageArray_ = function () {
  return Blockly.Msg
}

Blockly.setLocale(require('./lib/i18n/en'))

Blockly.Blocks = Object.assign(Blockly.Blocks, require('./lib/blocks_compressed')(Blockly));

module.exports = Blockly;