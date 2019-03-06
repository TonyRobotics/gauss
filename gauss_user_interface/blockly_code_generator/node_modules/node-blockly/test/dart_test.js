'use strict';

var assert = require('chai').assert;

var Blockly = require('../index.js'),
    ifBlockXml = require('./xml/if');

var res = `main() {
  if (6 * 7 == 42) {
    print('Dont panic');
  } else {
    print('Panic');
  }
}`

function xmlToDart(xml) {
  try {
    var xml = Blockly.Xml.textToDom(xml);
  }
  catch (e) {s
    return ''
  }
  
  var workspace = new Blockly.Workspace();
  Blockly.Xml.domToWorkspace(xml, workspace);
  return Blockly.Dart.workspaceToCode(workspace);
}

describe('Dart Generator', function() {
  it('should convert valid xml to js code', function() {
    var code = xmlToDart(ifBlockXml);
    
    assert.equal(code, res)
  });
  
  it('should convert invalid xml to empty string', function() {
    var code = xmlToDart('<block type="math_number"><field name="NUM">42</field></block>');
    
    assert.equal(code, 'main() {\n}')
  });
});


