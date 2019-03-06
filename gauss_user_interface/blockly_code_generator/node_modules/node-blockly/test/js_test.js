'use strict';

var assert = require('chai').assert;

var Blockly = require('../index.js'),
    ifBlockXml = require('./xml/if');


var res = `if (6 * 7 == 42) {
  window.alert('Dont panic');
} else {
  window.alert('Panic');
}
`

function xmlToJs(xml) {
  try {
    var xml = Blockly.Xml.textToDom(xml);
  }
  catch (e) {
    console.log(e);
    return ''
  }

  var workspace = new Blockly.Workspace();
  Blockly.Xml.domToWorkspace(xml, workspace);
  return Blockly.JavaScript.workspaceToCode(workspace);
}

describe('JS Generator', function() {
  it('should convert valid xml to js code', function() {
    var code = xmlToJs(ifBlockXml);

    assert.equal(code, res)
  });

  it('should convert invalid xml to empty string', function() {
    var code = xmlToJs('<block type="math_number"><field name="NUM">42</field></block>');

    assert.equal(code, '')
  });


  it('should convert example from server', function() {
    var code = xmlToJs(`<xml xmlns="http://www.w3.org/1999/xhtml">
  <variables></variables>
  <block type="controls_repeat_ext" id="(BFo|WQ,-6A?iUwjWMq*" x="238" y="63">
    <value name="TIMES">
      <shadow type="math_number" id="QU(s4?wLG]]%mQZ:pPQP">
        <field name="NUM">10</field>
      </shadow>
    </value>
  </block>
</xml>`);

    assert.equal(code, `for (var count = 0; count < 10; count++) {\n}\n`)
  });
});





