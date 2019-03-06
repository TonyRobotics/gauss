var Blockly = require('../index');

var xmlText = '<xml xmlns="http://www.w3.org/1999/xhtml">' +
    '<block type="variables_set">' +
    '<field name="VAR">blockly</field>' +
    '<value name="VALUE">' +
    '<block type="text">' +
    '<field name="TEXT">Hello world!</field>' +
    '</block>' +
    '</value>' +
    '</block>' +
    '</xml>';

try {
  var xml = Blockly.Xml.textToDom(xmlText);
}
catch (e) {
  console.log(e);
  return ''
}

var workspace = new Blockly.Workspace();
Blockly.Xml.domToWorkspace(xml, workspace);
var code = Blockly.JavaScript.workspaceToCode(workspace);

console.log(code) // blockly = 'Hello Node.js!'

/**
 var blockly;

 blockly == 'Hello Node.js!';
 */
