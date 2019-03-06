# Blockly for Node.js and Browser via CommonJS module

![Build](https://travis-ci.org/mo4islona/node-blockly.svg?branch=master)


Supports `JavaScript`, `PHP`, `Dart`, `Lua` and `Python` generators.

[Live demo](http://mo4islona.github.io/blockly/) with async locales


## Install
```
yarn add node-blockly
```
## Usage
**Node.js**

All generators
```js
var Blockly = require('node-blockly');
```
Or you may use standalone generators to decrease memory usage
```js 
var Blockly = require('node-blockly/lua');
```

**Browser**

All generators
```js
var Blockly = require('node-blockly/browser');
```

## Example
**Node.js**
```js
var Blockly = require('node-blockly');

var xmlText = `<xml xmlns="http://www.w3.org/1999/xhtml">
        <block type="variables_set">
            <field name="VAR">blockly</field>
            <value name="VALUE">
                <block type="text">
                    <field name="TEXT">Hello Node.js!</field>
                </block>
            </value>
        </block>
    </xml>`;

try {
    var xml = Blockly.Xml.textToDom(xmlText);
}
catch (e) {
    console.log(e);
    return
}

var workspace = new Blockly.Workspace();
Blockly.Xml.domToWorkspace(xml, workspace);
var code = Blockly.JavaScript.workspaceToCode(workspace);

console.log(code)  
```
Compiled result

```js
var blockly; 

blockly = 'Hello Node.js!';
```

**Browser**

[Live demo](http://mo4islona.github.io/blockly/) ([source](https://github.com/mo4islona/mo4islona.github.io/blob/master/blockly/index.js))

## Internationalization

```js
import Blockly from 'node-blockly/browser';
import De from 'node-blockly/lib/i18n/de';
Blockly.setLocale(De)
```

Dynamic imports also works but Blockly doesn't re-render workspace. You must [re-render it manually after locale loaded](https://github.com/mo4islona/mo4islona.github.io/blob/master/blockly/index.js#L6)



