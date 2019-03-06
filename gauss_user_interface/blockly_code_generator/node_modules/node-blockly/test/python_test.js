'use strict';

var assert = require('chai').assert;

var Blockly = require('../index.js'),
    ifBlockXml = require('./xml/if');

var res = `if 6 * 7 == 42:
  print('Dont panic')
else:
  print('Panic')
`

function xmlToPython(xml) {
  try {
    var xml = Blockly.Xml.textToDom(xml);
  }
  catch (e) {
    return ''
  }

  var workspace = new Blockly.Workspace();
  Blockly.Xml.domToWorkspace(xml, workspace);
  return Blockly.Python.workspaceToCode(workspace);
}

describe('PHP Generator', function() {
  it('should convert valid xml to js code', function() {
    var code = xmlToPython(ifBlockXml);

    assert.equal(code, res)
  });

  it('should convert invalid xml to empty string', function() {
    var code = xmlToPython('<block type="math_number"><field name="NUM">42</field></block>');

    assert.equal(code, '')
  });
});


