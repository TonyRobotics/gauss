'use strict';

var assert = require('chai').assert;

var Blockly = require('../index.js');

var XML_TEXT = `
<xml xmlns="http://www.w3.org/1999/xhtml">
  <block type="controls_repeat_ext" id="10" inline="true" x="21" y="23" xmlns="http://www.w3.org/1999/xhtml">
  <value name="TIMES" xmlns="http://www.w3.org/1999/xhtml">
    <block type="math_number" id="11" xmlns="http://www.w3.org/1999/xhtml">
    <field name="NUM" xmlns="http://www.w3.org/1999/xhtml">10</field></block></value>
    <statement name="DO" xmlns="http://www.w3.org/1999/xhtml">
    <block type="variables_set" id="139" inline="true" xmlns="http://www.w3.org/1999/xhtml">
      <field name="VAR" xmlns="http://www.w3.org/1999/xhtml">item</field>
      <value name="VALUE" xmlns="http://www.w3.org/1999/xhtml">
      <block type="lists_create_empty" id="171" xmlns="http://www.w3.org/1999/xhtml"/>        
    </value>        
    <next xmlns="http://www.w3.org/1999/xhtml">          
    <block type="text_print" id="78" inline="false" xmlns="http://www.w3.org/1999/xhtml">           
     <value name="TEXT" xmlns="http://www.w3.org/1999/xhtml">              
     <block type="text" id="189" xmlns="http://www.w3.org/1999/xhtml">                
     <field name="TEXT" xmlns="http://www.w3.org/1999/xhtml">Hello</field>              
     </block>            
     </value>          
     </block>        
     </next>      
     </block>    
     </statement>
     </block>
     </xml>
`;

describe('XML', function() {
  it('textToDom', function() {
    var dom = Blockly.Xml.textToDom(XML_TEXT);
    assert.equal('xml', dom.nodeName, 'XML tag');
    assert.equal(6, dom.getElementsByTagName('block').length, 'Block tags');
  });

  it('domToText', function() {
    var dom = Blockly.Xml.textToDom(XML_TEXT);
    var text = Blockly.Xml.domToText(dom);

    assert.isAbove(text.length, 0, 'Round trip');
  });

  it('domToPrettyText', function() {
    var dom = Blockly.Xml.textToDom(XML_TEXT);
    var text = Blockly.Xml.domToPrettyText(dom);
    assert.isAbove(text.length, 0, 'Round trip');
  });
});